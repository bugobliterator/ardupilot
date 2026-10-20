#include <AP_Math/crc.h>
#include <AP_ModuleM4_Crc32.h>

void ModuleM4::crc32_feed(Crc32 &crc, const uint8_t *data, uint32_t length)
{
    crc.state = crc32_software_update(crc.state, data, length);
}

uint32_t crc32_small(uint32_t crc, const uint8_t *buf, uint32_t size)
{
    return ModuleM4::crc32_software_update(crc, buf, size);
}
