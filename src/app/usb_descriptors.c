/*
 * usb_descriptors.c - USB-Beschreibung: ein virtueller COM-Port (CDC-ACM)
 *
 * VID/PID 1209:0001 ist die Test-ID von pid.codes (fuer Prototypen gedacht).
 * macOS/Linux/Windows 10+ erkennen CDC-ACM ohne Treiber.
 */
#include <string.h>
#include "board.h"
#include "tusb.h"

#define USB_VID 0x1209
#define USB_PID 0x0001

static const tusb_desc_device_t desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = TUSB_CLASS_MISC,          /* noetig fuer IAD (CDC) */
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = USB_VID,
    .idProduct          = USB_PID,
    .bcdDevice          = 0x0100,
    .iManufacturer      = 1,
    .iProduct           = 2,
    .iSerialNumber      = 3,
    .bNumConfigurations = 1,
};

const uint8_t *tud_descriptor_device_cb(void)
{
    return (const uint8_t *)&desc_device;
}

enum { ITF_CDC = 0, ITF_CDC_DATA, ITF_COUNT };
#define EP_CDC_NOTIF 0x81
#define EP_CDC_OUT   0x02
#define EP_CDC_IN    0x82
#define CONFIG_LEN   (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)

static const uint8_t desc_config[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_COUNT, 0, CONFIG_LEN, 0x00, 100),
    TUD_CDC_DESCRIPTOR(ITF_CDC, 4, EP_CDC_NOTIF, 8, EP_CDC_OUT, EP_CDC_IN, 64),
};

const uint8_t *tud_descriptor_configuration_cb(uint8_t index)
{
    (void)index;
    return desc_config;
}

static const char *const strings[] = {
    NULL,                      /* 0: Sprache, separat behandelt */
    "KZU Maturaarbeit",        /* 1: Hersteller */
    "DIY Drone FC",            /* 2: Produkt */
    NULL,                      /* 3: Seriennummer aus der Chip-ID */
    "DIY Drone FC CLI",        /* 4: CDC-Schnittstelle */
};

static uint16_t desc_str[32 + 1];

const uint16_t *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    (void)langid;
    uint32_t len;
    if (index == 0) {
        desc_str[1] = 0x0409;                 /* Englisch (USA) */
        len = 1;
    } else if (index == 3) {
        static const char hex[] = "0123456789ABCDEF";
        const uint8_t *uid = (const uint8_t *)UID_BASE;   /* 96-Bit-Chip-ID */
        len = 0;
        for (int i = 0; i < 12; i++) {
            desc_str[1 + len++] = (uint16_t)hex[uid[i] >> 4];
            desc_str[1 + len++] = (uint16_t)hex[uid[i] & 0xF];
        }
    } else if (index < sizeof(strings) / sizeof(strings[0]) && strings[index]) {
        const char *s = strings[index];
        len = (uint32_t)strlen(s);
        if (len > 32) len = 32;
        for (uint32_t i = 0; i < len; i++) desc_str[1 + i] = (uint16_t)s[i];
    } else {
        return NULL;
    }
    desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2 * len + 2));
    return desc_str;
}
