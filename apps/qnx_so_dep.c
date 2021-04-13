/*
 * These symbols need to be defined for QNX shared object build of
 * vision_apps to compile properly, though they are not needed
 * in any apps/libs that are actually being used.
 */
#include <stdint.h>

uint64_t TTBR3_BASE_ADDR, TTBR2_BASE_ADDR, TTBR1_BASE_ADDR;
