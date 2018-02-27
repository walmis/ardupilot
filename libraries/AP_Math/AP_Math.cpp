#include "AP_Math.h"

#include <float.h>

/*
  simple 16 bit random number generator
 */
uint16_t get_random16(void)
{
    static uint32_t m_z = 1234;
    static uint32_t m_w = 76542;
    m_z = 36969 * (m_z & 0xFFFFu) + (m_z >> 16);
    m_w = 18000 * (m_w & 0xFFFFu) + (m_w >> 16);
    return ((m_z << 16) + m_w) & 0xFFFF;
}


#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
// generate a random float between -1 and 1, for use in SITL
float rand_float(void)
{
    return ((((unsigned)random()) % 2000000) - 1.0e6) / 1.0e6;
}

Vector3f rand_vec3f(void)
{
    Vector3f v = Vector3f(rand_float(),
                          rand_float(),
                          rand_float());
    if (v.length() != 0.0f) {
        v.normalize();
    }
    return v;
}
#endif

