#include "../badhal.h"

void swo_init(u32 swo_freq);
void swo_putc(char c);
void swo_write(const char *data, u32 size);
void swo_writestr(const char *str);