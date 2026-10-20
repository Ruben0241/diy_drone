#include "blackbox.h"
#include <string.h>

static bool page_is_erased(const blackbox_t *bb, uint32_t addr)
{
    uint8_t buf[BB_PAGE_SIZE];
    if (!bb->ops->read(addr, buf, sizeof(buf))) return false;
    for (uint32_t i = 0; i < sizeof(buf); i++) {
        if (buf[i] != 0xFF) return false;
    }
    return true;
}

static bool page_starts_erased(const blackbox_t *bb, uint32_t page)
{
    uint8_t b = 0;
    bb->ops->read(page * BB_PAGE_SIZE, &b, 1);
    return b == 0xFF;
}

bool bb_init(blackbox_t *bb, const bb_flash_ops_t *ops)
{
    memset(bb, 0, sizeof(*bb));
    bb->ops = ops;
    if (!ops || ops->capacity < BB_BLOCK_SIZE) {
        return false;
    }
    const uint32_t pages = ops->capacity / BB_PAGE_SIZE;

    /* Binaere Suche nach der ersten Seite, die mit 0xFF beginnt.
     * Annahme: Seiten werden lueckenlos ab 0 geschrieben. */
    uint32_t lo = 0, hi = pages;   /* Ergebnis in [lo, hi] */
    while (lo < hi) {
        const uint32_t mid = lo + (hi - lo) / 2;
        if (page_starts_erased(bb, mid)) hi = mid;
        else lo = mid + 1;
    }
    /* Eine teilweise geschriebene Seite (z. B. Stromausfall) ueberspringen */
    while (lo < pages && !page_is_erased(bb, lo * BB_PAGE_SIZE)) {
        lo++;
    }
    bb->write_addr = lo * BB_PAGE_SIZE;
    bb->full = (lo >= pages);
    bb->ok = true;
    return true;
}

static void write_page(blackbox_t *bb)
{
    if (bb->write_addr + BB_PAGE_SIZE > bb->ops->capacity) {
        bb->full = true;
        return;
    }
    if (!bb->ops->program_page(bb->write_addr, bb->page)) {
        bb->write_errors++;
    }
    bb->write_addr += BB_PAGE_SIZE;
    if (bb->write_addr + BB_PAGE_SIZE > bb->ops->capacity) {
        bb->full = true;
    }
}

bool bb_append(blackbox_t *bb, const uint8_t rec[LOGREC_SIZE])
{
    if (!bb->ok || bb->full) {
        bb->records_dropped++;
        return false;
    }
    memcpy(&bb->page[bb->page_fill], rec, LOGREC_SIZE);
    bb->page_fill += LOGREC_SIZE;
    bb->records_written++;
    if (bb->page_fill >= BB_PAGE_SIZE) {
        write_page(bb);
        bb->page_fill = 0;
    }
    return true;
}

void bb_flush(blackbox_t *bb)
{
    if (!bb->ok || bb->page_fill == 0) return;
    uint8_t pad[LOGREC_SIZE];
    logrec_encode_pad(pad);
    while (bb->page_fill < BB_PAGE_SIZE) {
        memcpy(&bb->page[bb->page_fill], pad, LOGREC_SIZE);
        bb->page_fill += LOGREC_SIZE;
    }
    if (!bb->full) write_page(bb);
    bb->page_fill = 0;
}

uint32_t bb_used_bytes(const blackbox_t *bb)
{
    return bb->write_addr;
}

bool bb_erase_used(blackbox_t *bb, void (*progress)(uint32_t percent))
{
    if (!bb->ok) return false;
    const uint32_t end = bb->write_addr;
    const uint32_t blocks = (end + BB_BLOCK_SIZE - 1) / BB_BLOCK_SIZE;
    bool ok = true;
    for (uint32_t b = 0; b < blocks; b++) {
        ok &= bb->ops->erase_block(b * BB_BLOCK_SIZE);
        if (progress) progress((b + 1) * 100u / blocks);
    }
    bb->write_addr = 0;
    bb->page_fill = 0;
    bb->full = false;
    return ok;
}

bool bb_erase_all(blackbox_t *bb)
{
    if (!bb->ok) return false;
    const bool ok = bb->ops->erase_chip();
    bb->write_addr = 0;
    bb->page_fill = 0;
    bb->full = false;
    return ok;
}
