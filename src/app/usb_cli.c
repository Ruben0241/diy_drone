/*
 * usb_cli.c - Befehlszeile ueber USB (virtueller COM-Port)
 *
 * Befehle (eine Zeile, Enter):
 *   help            Liste der Befehle
 *   info            Firmware, Takt, Flash-Belegung, Zustand
 *   status          eine Zeile mit Lage, Gyro, Sticks, Akku
 *   live            status 20x pro Sekunde, beliebige Taste beendet
 *   dump            Flugdaten herunterladen: "DUMP <n>\r\n", n Byte binaer, "\r\nOK\r\n"
 *   erase           beschriebenen Bereich loeschen (Fortschritt in %)
 *   erase all       ganzen Flash loeschen (bis 200 s)
 *   dfu             Neustart in den Bootloader (zum Flashen)
 *   reboot          Neustart
 *   get [anfang]    Einstellungen anzeigen (optional nur Namen mit diesem Anfang)
 *   set name wert   Einstellung aendern (sofort wirksam, disarmed)
 *   save            Einstellungen im Flash speichern
 *   diff            nur Abweichungen vom Standard anzeigen (fuer die Dokumentation)
 *   defaults        alle Einstellungen auf Standard (erst mit save dauerhaft)
 *   motor n prozent Motortest ohne Propeller: n = 1..4 oder all, max. 40 %, 3 s
 *   motor stop      Motortest beenden
 *   acccal          Beschleunigungssensor kalibrieren (waagrecht und still hinstellen)
 *
 * dump/erase/dfu/reboot sind nur moeglich, wenn nicht gearmt. Waehrend dump
 * und erase ist das Armen gesperrt. Das Gegenstueck am PC: tools/fc_cli.py
 */
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "tusb.h"

#include "app_shared.h"
#include "board.h"
#include "flash_w25q.h"
#include "params.h"
#include "system.h"

/* ---------- USB-Hardware ---------- */

void OTG_FS_IRQHandler(void)
{
    tusb_int_handler(0, true);
}

static void usb_hw_init(void)
{
    gpio_config(GPIOA, 11, GPIO_AF, 10, GPIO_NOPULL, true);   /* DM */
    gpio_config(GPIOA, 12, GPIO_AF, 10, GPIO_NOPULL, true);   /* DP */
    RCC->AHB2ENR |= RCC_AHB2ENR_OTGFSEN;
    (void)RCC->AHB2ENR;
    /* VBUS-Erkennung aus: PA9 ist auf diesem Board UART1 TX */
    USB_OTG_FS->GCCFG &= ~USB_OTG_GCCFG_VBDEN;
    USB_OTG_FS->GOTGCTL |= USB_OTG_GOTGCTL_BVALOEN | USB_OTG_GOTGCTL_BVALOVAL;
    NVIC_SetPriority(OTG_FS_IRQn, 8);   /* >= configMAX_SYSCALL_INTERRUPT_PRIORITY */
}

static void usb_device_task(void *arg)
{
    (void)arg;
    usb_hw_init();
    const tusb_rhport_init_t init = { .role = TUSB_ROLE_DEVICE, .speed = TUSB_SPEED_FULL };
    tusb_init(0, &init);
    for (;;) {
        tud_task();     /* blockiert, bis ein USB-Ereignis anliegt */
    }
}

/* ---------- Ausgabe ---------- */

static char out[192];
static uint32_t out_len;

static bool write_raw(const void *data, uint32_t len)
{
    const uint8_t *p = data;
    while (len) {
        if (!tud_cdc_connected()) return false;
        uint32_t n = tud_cdc_write_available();
        if (n == 0) {
            tud_cdc_write_flush();
            vTaskDelay(1);
            continue;
        }
        if (n > len) n = len;
        tud_cdc_write(p, n);
        p += n;
        len -= n;
    }
    return true;
}

static void o_str(const char *s)
{
    while (*s && out_len < sizeof(out) - 3) out[out_len++] = *s++;
}

static void o_u32(uint32_t v)
{
    char tmp[11];
    int i = 0;
    do { tmp[i++] = (char)('0' + v % 10u); v /= 10u; } while (v && i < 10);
    while (i--) { if (out_len < sizeof(out) - 3) out[out_len++] = tmp[i]; }
}

static void o_i32(int32_t v)
{
    if (v < 0) { o_str("-"); o_u32((uint32_t)(-(v + 1)) + 1u); }
    else o_u32((uint32_t)v);
}

static void o_fix(float v, int decimals)
{
    int32_t scale = 1;
    for (int i = 0; i < decimals; i++) scale *= 10;
    const float r = v * (float)scale + (v >= 0.0f ? 0.5f : -0.5f);
    int32_t x = (r > 2.0e9f) ? 2000000000 : (r < -2.0e9f) ? -2000000000 : (int32_t)r;
    if (x < 0) { o_str("-"); x = -x; }
    o_u32((uint32_t)(x / scale));
    if (decimals) {
        o_str(".");
        int32_t frac = x % scale;
        for (int32_t d = scale / 10; d > 0; d /= 10) {
            const char c[2] = { (char)('0' + (frac / d) % 10), 0 };
            o_str(c);
        }
    }
}

static void o_hex(uint32_t v, int digits)
{
    static const char hex[] = "0123456789ABCDEF";
    for (int i = digits - 1; i >= 0; i--) {
        const char c[2] = { hex[(v >> (4 * i)) & 0xF], 0 };
        o_str(c);
    }
}

static void o_line(void)
{
    out[out_len++] = '\r';
    out[out_len++] = '\n';
    write_raw(out, out_len);
    tud_cdc_write_flush();
    out_len = 0;
}

static void say(const char *s)
{
    o_str(s);
    o_line();
}

/* ---------- Befehle ---------- */

static void cmd_help(void)
{
    say("Befehle: help, info, status, live, dump, erase, erase all, dfu, reboot,");
    say("         get [anfang], set name wert, save, diff, defaults, motor n %, motor stop, acccal");
}

/* ---------- Einstellungen ---------- */

static flight_config_t cfg_edit;
static bool cfg_edit_ok;

static void cfg_edit_init(void)
{
    if (!cfg_edit_ok) {
        cfg_edit = g_flight.cfg;
        cfg_edit_ok = true;
    }
}

static void push_config(void)
{
    for (int i = 0; i < 100 && g_cfg_new_pending; i++) vTaskDelay(1);
    g_cfg_new = cfg_edit;
    __DMB();
    g_cfg_new_pending = true;
}

static void print_param(const flight_config_t *c, int i)
{
    char buf[64];
    params_format(c, i, buf, sizeof(buf));
    say(buf);
}

static void cmd_get(const char *prefix)
{
    cfg_edit_init();
    const size_t n = strlen(prefix);
    for (int i = 0; i < params_count(); i++) {
        if (n == 0 || strncmp(params_def(i)->name, prefix, n) == 0) print_param(&cfg_edit, i);
    }
}

static void cmd_diff(void)
{
    cfg_edit_init();
    flight_config_t def;
    flight_default_config(&def);
    say("# Abweichungen vom Standard");
    for (int i = 0; i < params_count(); i++) {
        const float a = params_get(&cfg_edit, i), b = params_get(&def, i);
        if (a - b > 1e-5f || b - a > 1e-5f) {
            char buf[64];
            params_format(&cfg_edit, i, buf, sizeof(buf));
            o_str("set "); o_str(buf); o_line();
        }
    }
}

static bool refuse_if_armed(void);

static void cmd_set(char *args)
{
    if (refuse_if_armed()) return;
    cfg_edit_init();
    char *name = args;
    while (*name == ' ') name++;
    char *val = strchr(name, ' ');
    if (!val) { say("ERR Aufruf: set name wert"); return; }
    *val++ = 0;
    char *eq = strchr(name, '=');   /* auch "set name=wert" */
    if (eq) { *eq = 0; val = eq + 1; }
    while (*val == ' ' || *val == '=') val++;
    if (!params_set(&cfg_edit, name, val)) {
        const int i = params_find(name);
        if (i < 0) { o_str("ERR unbekannt: "); o_str(name); o_line(); return; }
        const param_def_t *d = params_def(i);
        o_str("ERR Bereich "); o_fix(d->min, d->decimals); o_str(" .. "); o_fix(d->max, d->decimals);
        o_line();
        return;
    }
    push_config();
    print_param(&cfg_edit, params_find(name));
    say("OK (dauerhaft erst mit save)");
}

static void cmd_save(void)
{
    if (refuse_if_armed()) return;
    cfg_edit_init();
    g_cli_busy = true;
    xSemaphoreTake(g_flash_mutex, portMAX_DELAY);
    const bool ok = config_store_save(&cfg_edit);
    xSemaphoreGive(g_flash_mutex);
    g_cli_busy = false;
    say(ok ? "OK gespeichert" : "ERR speichern fehlgeschlagen");
}

static void cmd_defaults(void)
{
    if (refuse_if_armed()) return;
    flight_default_config(&cfg_edit);
    cfg_edit.loop_hz = g_flight.cfg.loop_hz;
    cfg_edit_ok = true;
    push_config();
    say("OK Standardwerte aktiv (dauerhaft erst mit save)");
}

/* ---------- Motortest und ACC-Kalibrierung ---------- */

static void cmd_motor(char *args)
{
    if (refuse_if_armed()) return;
    while (*args == ' ') args++;
    if (strncmp(args, "stop", 4) == 0) {
        g_motor_test_until_ms = xTaskGetTickCount();
        for (int m = 0; m < 4; m++) g_motor_test_value[m] = 0.0f;
        say("OK");
        return;
    }
    char *pct = strchr(args, ' ');
    float p;
    if (!pct || !params_parse_float(pct + 1, &p) || p < 0.0f || p > 40.0f) {
        say("ERR Aufruf: motor <1-4|all> <0-40>  (ohne Propeller!)");
        return;
    }
    *pct = 0;
    int sel = -1;
    if (strcmp(args, "all") != 0) {
        float n;
        if (!params_parse_float(args, &n) || n < 1.0f || n > 4.0f) { say("ERR Motor 1-4 oder all"); return; }
        sel = (int)n - 1;
    }
    for (int m = 0; m < 4; m++) g_motor_test_value[m] = (sel < 0 || sel == m) ? p / 100.0f : 0.0f;
    g_motor_test_until_ms = xTaskGetTickCount() + 3000u;
    say("OK 3 s (ohne Propeller!)");
}

static void cmd_acccal(void)
{
    if (refuse_if_armed()) return;
    cfg_edit_init();
    g_acc_cal_request = true;
    say("ACC-Kalibrierung: Drohne waagrecht und still halten ...");
    for (int i = 0; i < 40; i++) {
        vTaskDelay(pdMS_TO_TICKS(100));
        if (!g_acc_cal_request && g_flight.acc_cal_done) {
            for (int k = 0; k < 3; k++) cfg_edit.acc_bias[k] = g_flight.cfg.acc_bias[k];
            o_str("acc_bias x="); o_fix(cfg_edit.acc_bias[0], 4);
            o_str(" y="); o_fix(cfg_edit.acc_bias[1], 4);
            o_str(" z="); o_fix(cfg_edit.acc_bias[2], 4); o_line();
            say("OK (dauerhaft erst mit save)");
            return;
        }
    }
    say("ERR Kalibrierung nicht abgeschlossen (bewegt?)");
}

static void cmd_info(void)
{
    o_str("fw "); o_hex(FW_GIT_HASH, 8); o_line();
    o_str("clock "); o_str(system_clock_is_hse() ? "HSE 8MHz" : "HSI (Quarz nicht angelaufen!)"); o_line();

    xSemaphoreTake(g_flash_mutex, portMAX_DELAY);
    const uint32_t used = bb_used_bytes(&g_blackbox);
    const bool full = g_blackbox.full, ok = g_blackbox.ok;
    const uint32_t written = g_blackbox.records_written, dropped = g_blackbox.records_dropped;
    xSemaphoreGive(g_flash_mutex);
    const uint32_t cap = flash_capacity();

    o_str("flash jedec="); o_hex(flash_jedec_id(), 6);
    o_str(" ok="); o_u32(ok);
    o_str(" capacity="); o_u32(cap);
    o_str(" used="); o_u32(used);
    o_str(" percent="); o_u32(cap ? (uint32_t)((uint64_t)used * 100u / cap) : 0u);
    o_str(" full="); o_u32(full);
    o_line();
    o_str("log records="); o_u32(written); o_str(" dropped="); o_u32(dropped);
    o_str(" queue_drops="); o_u32(g_log_queue_drops); o_line();
    o_str("state armed="); o_u32(g_flight.armed); o_str(" failsafe="); o_u32(g_flight.failsafe);
    o_str(" calibrated="); o_u32(g_flight.calibrated); o_str(" gyro_ok="); o_u32(g_gyro_ok);
    o_str(" mode="); o_str(g_flight.mode == MODE_ANGLE ? "ANGLE" : "RATE");
    o_str(" loop_overruns="); o_u32(g_loop_overruns); o_line();
    o_str("rx frames="); o_u32(g_rc_frames); o_str(" crc_errors="); o_u32(g_rc_crc_errors);
    o_str(" lq="); o_u32(g_link_quality); o_line();
    o_str("loop avg_us="); o_fix((float)g_loop_us_avg_x10 / 10.0f, 1);
    o_str(" max_us="); o_u32(g_loop_us_max);
    o_str(" period_us min="); o_u32(g_cycle_us_min); o_str(" max="); o_u32(g_cycle_us_max); o_line();
    o_str("config "); o_str(g_cfg_from_flash ? "gespeichert (Flash)" : "Standardwerte");
    o_str(g_cfg_new_pending ? " (Aenderung wartet auf disarm)" : ""); o_line();
    o_str("airmode="); o_u32(g_flight.airmode_active);
    o_str(" runaway_triggered="); o_u32(g_flight.runaway_triggered); o_line();
    o_str("vbat "); o_fix(g_vbat_volts, 2); o_line();
}

static void cmd_status(void)
{
    const flight_t *f = &g_flight;
    o_str("att r="); o_fix(f->attitude_deg[0], 1);
    o_str(" p="); o_fix(f->attitude_deg[1], 1);
    o_str(" y="); o_fix(f->attitude_deg[2], 1);
    o_str(" gyro r="); o_fix(f->gyro_f[0], 1);
    o_str(" p="); o_fix(f->gyro_f[1], 1);
    o_str(" y="); o_fix(f->gyro_f[2], 1);
    o_str(" rc");
    for (int i = 0; i < 6; i++) { o_str(" "); o_u32(g_rc_channels[i]); }
    o_str(" sp r="); o_i32((int32_t)f->setpoint[0]);
    o_str(" p="); o_i32((int32_t)f->setpoint[1]);
    o_str(" y="); o_i32((int32_t)f->setpoint[2]);
    o_str(" armed="); o_u32(f->armed);
    o_str(" fs="); o_u32(f->failsafe);
    o_str(" vbat="); o_fix(g_vbat_volts, 2);
    o_line();
}

static void cmd_live(void)
{
    say("LIVE (beliebige Taste beendet)");
    while (tud_cdc_connected()) {
        if (tud_cdc_available()) {
            uint8_t c;
            while (tud_cdc_available()) tud_cdc_read(&c, 1);
            break;
        }
        cmd_status();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    say("OK");
}

static bool refuse_if_armed(void)
{
    if (g_flight.armed) {
        say("ERR armed");
        return true;
    }
    return false;
}

static void cmd_dump(void)
{
    if (refuse_if_armed()) return;
    g_cli_busy = true;
    xSemaphoreTake(g_flash_mutex, portMAX_DELAY);
    bb_flush(&g_blackbox);
    const uint32_t total = bb_used_bytes(&g_blackbox);

    o_str("DUMP "); o_u32(total); o_line();
    static uint8_t buf[1024];
    bool ok = true;
    for (uint32_t addr = 0; addr < total && ok; addr += sizeof(buf)) {
        const uint32_t n = (total - addr < sizeof(buf)) ? total - addr : sizeof(buf);
        ok = flash_read(addr, buf, n) && write_raw(buf, n);
    }
    tud_cdc_write_flush();
    xSemaphoreGive(g_flash_mutex);
    g_cli_busy = false;
    o_line();
    say(ok ? "OK" : "ERR read");
}

static uint32_t last_percent;
static void erase_progress(uint32_t percent)
{
    if (percent / 10 != last_percent / 10 || percent == 100) {
        o_str("ERASE "); o_u32(percent); o_str("%"); o_line();
    }
    last_percent = percent;
}

static void cmd_erase(bool all)
{
    if (refuse_if_armed()) return;
    g_cli_busy = true;
    xSemaphoreTake(g_flash_mutex, portMAX_DELAY);
    bool ok;
    if (all) {
        say("ERASE ALL (dauert bis 200 s)");
        ok = bb_erase_all(&g_blackbox);
    } else {
        last_percent = 0;
        ok = bb_erase_used(&g_blackbox, erase_progress);
    }
    g_blackbox.records_written = 0;
    g_blackbox.records_dropped = 0;
    xSemaphoreGive(g_flash_mutex);
    g_cli_busy = false;
    say(ok ? "OK" : "ERR erase");
}

static void cmd_reboot(bool dfu)
{
    if (refuse_if_armed()) return;
    say(dfu ? "OK dfu" : "OK reboot");
    vTaskDelay(pdMS_TO_TICKS(100));
    system_reboot(dfu);
}

static void handle_line(char *line)
{
    while (*line == ' ') line++;
    if (!*line) return;
    if (!strcmp(line, "help") || !strcmp(line, "?")) cmd_help();
    else if (!strcmp(line, "info")) cmd_info();
    else if (!strcmp(line, "status")) cmd_status();
    else if (!strcmp(line, "live")) cmd_live();
    else if (!strcmp(line, "dump")) cmd_dump();
    else if (!strcmp(line, "erase")) cmd_erase(false);
    else if (!strcmp(line, "erase all")) cmd_erase(true);
    else if (!strcmp(line, "dfu")) cmd_reboot(true);
    else if (!strcmp(line, "reboot")) cmd_reboot(false);
    else if (!strcmp(line, "get")) cmd_get("");
    else if (!strncmp(line, "get ", 4)) cmd_get(line + 4);
    else if (!strncmp(line, "set ", 4)) cmd_set(line + 4);
    else if (!strcmp(line, "save")) cmd_save();
    else if (!strcmp(line, "diff")) cmd_diff();
    else if (!strcmp(line, "defaults")) cmd_defaults();
    else if (!strncmp(line, "motor ", 6)) cmd_motor(line + 6);
    else if (!strcmp(line, "acccal")) cmd_acccal();
    else { o_str("ERR unbekannt: "); o_str(line); o_line(); }
}

static void flash_wait_hook(void)
{
    vTaskDelay(1);
}

static void cli_task(void *arg)
{
    (void)arg;
    flash_set_wait_hook(flash_wait_hook);
    char line[64];
    uint32_t len = 0;
    for (;;) {
        if (!tud_cdc_connected() || !tud_cdc_available()) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        uint8_t c;
        while (tud_cdc_read(&c, 1) == 1) {
            if (c == '\r' || c == '\n') {
                line[len] = 0;
                handle_line(line);
                len = 0;
            } else if (len < sizeof(line) - 1) {
                line[len++] = (char)c;
            }
        }
    }
}

void usb_cli_start(void)
{
    xTaskCreate(usb_device_task, "usbd", 1024, NULL, 2, NULL);
    xTaskCreate(cli_task, "cli", 1024, NULL, 1, NULL);
}
