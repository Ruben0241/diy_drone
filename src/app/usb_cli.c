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
    say("Befehle: help, info, status, live, dump, erase, erase all, dfu, reboot");
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
