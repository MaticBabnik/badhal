#pragma once
#include "../badhal.h"

void usart_use_hsi();
void usart_setup_basic(struct USART_t *usart, u32 baudrate);
char usart_recv(struct USART_t *usart);
void usart_send(struct USART_t *usart, char chr);
void usart_send_string(struct USART_t *usart, const char *string);