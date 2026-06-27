#pragma once
#include "../hal/badhal.h"

// Implement these callbacks
u32 loader_cb_erase8k(u32 addr);
u32 loader_cb_write(u32 addr, const u8 *data, u32 n);
u32 loader_cb_read(u32 addr, u8 *data, u32 n);

void loader_main(USART_t *usart);
