#ifndef SPINALI_CORE_COMMON_H
#define SPINALI_CORE_COMMON_H

#include <zephyr/device.h>
const struct device *get_device(const struct device *const dev);

extern const char *banner_spinali;

#endif // SPINALI_CORE_COMMON_H
