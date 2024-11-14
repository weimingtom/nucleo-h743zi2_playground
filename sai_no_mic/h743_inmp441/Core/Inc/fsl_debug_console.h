#pragma once

#ifdef __cplusplus
extern "C" {
#endif

extern void ei_printf(const char *format, ...);

#define PRINTF ei_printf

#ifdef __cplusplus
}
#endif
