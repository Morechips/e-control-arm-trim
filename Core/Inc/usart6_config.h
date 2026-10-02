#ifndef USART6_CONFIG_H
#define USART6_CONFIG_H

/* USART6 PC6/PC7 is a mutually exclusive shared port. Both configurations are
 * recorded here, but only one owner may be selected in a firmware image. */
#define USART6_OWNER_BLUETOOTH 1U
#define USART6_OWNER_XDK42 2U

/* Current firmware selection: keep the existing Bluetooth implementation. */
#ifndef USART6_CONFIGURED_OWNER
#define USART6_CONFIGURED_OWNER USART6_OWNER_BLUETOOTH
#endif

#if USART6_CONFIGURED_OWNER != USART6_OWNER_BLUETOOTH && \
    USART6_CONFIGURED_OWNER != USART6_OWNER_XDK42
#error USART6_CONFIGURED_OWNER_must_select_exactly_one_owner
#endif

/* Stored XDK42 candidate configuration only. XDK42 RX/protocol processing is
 * intentionally not enabled while Bluetooth owns USART6. */
#ifndef XDK42_USART6_BAUD_RATE
#define XDK42_USART6_BAUD_RATE 115200U
#endif

#endif
