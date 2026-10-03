#ifndef VISION_CONFIG_H
#define VISION_CONFIG_H

/* UART4 MaxiCam port: PC10 TX / PC11 RX, AF8, 8N1, no flow control. */
#ifndef MAXICAM_UART_BAUD_RATE
#define MAXICAM_UART_BAUD_RATE 115200U
#endif

/* Compatibility name retained for existing project configuration. */
#ifndef VISION_UART_BAUD_RATE
#define VISION_UART_BAUD_RATE MAXICAM_UART_BAUD_RATE
#endif

/* Mode byte: single blocking byte on UART4, 5 ms is ample at 115200. */
#ifndef MAXICAM_MODE_TX_TIMEOUT_MS
#define MAXICAM_MODE_TX_TIMEOUT_MS 5U
#endif
/* The mode byte is only effective after a QR notification, has no acknowledge
 * and cannot be verified, so a request is deferred and retried rather than
 * reported as a failure: three fast attempts, then a slow self-healing retry. */
#ifndef MAXICAM_MODE_RETRY_GAP_MS
#define MAXICAM_MODE_RETRY_GAP_MS 20U
#endif
#ifndef MAXICAM_MODE_RETRY_MAX
#define MAXICAM_MODE_RETRY_MAX 3U
#endif
#ifndef MAXICAM_MODE_RETRY_SLOW_MS
#define MAXICAM_MODE_RETRY_SLOW_MS 200U
#endif
/* Bytes arriving right after a mode change belong to the previous mode's frame
 * stream; discard them instead of shifting the parser alignment. */
#ifndef MAXICAM_MODE_FLUSH_MS
#define MAXICAM_MODE_FLUSH_MS 50U
#endif
/* Repeated QR notices are logged at most this often. */
#ifndef MAXICAM_QR_LOG_PERIOD_MS
#define MAXICAM_QR_LOG_PERIOD_MS 1000U
#endif


#define VISION_FOLLOW_RPM 8
#define VISION_FRAME_TIMEOUT_MS 200U
#define VISION_SETTLE_MS 300U
#define VISION_SAMPLE_INTERVAL_MS 100U
#define VISION_STABLE_FRAMES 3U
#define VISION_STABLE_SPREAD_X 4


#endif
