/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include "wired.h"

#include <zephyr/sys/crc.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#if IS_ENABLED(CONFIG_ZMK_SPLIT_WIRED_UART_MODE_DEFAULT_POLLING)

void zmk_split_wired_poll_out(struct ring_buf *tx_buf, const struct device *uart) {
    uint8_t *buf;
    uint32_t claim_len;
    while ((claim_len = ring_buf_get_claim(tx_buf, &buf, MIN(32, tx_buf->size))) > 0) {
        LOG_HEXDUMP_DBG(buf, claim_len, "TX Bytes");
        for (int i = 0; i < claim_len; i++) {
            uart_poll_out(uart, buf[i]);
        }

        ring_buf_get_finish(tx_buf, claim_len);
    }
}

void zmk_split_wired_poll_in(struct ring_buf *rx_buf, const struct device *uart,
                             struct k_work *process_data_work, size_t envelope_size) {
    uint8_t *buf;
    uint32_t read = 0;
    uint32_t claim_len = ring_buf_put_claim(rx_buf, &buf, ring_buf_space_get(rx_buf));
    if (claim_len < 1) {
        LOG_WRN("No room available for reading in from the serial port");
        return;
    }

    while (read < claim_len) {
        if (uart_poll_in(uart, buf + read) < 0) {
            break;
        }

        read++;
    }

    ring_buf_put_finish(rx_buf, read);

    if (ring_buf_size_get(rx_buf) >= envelope_size) {
        k_work_submit(process_data_work);
    }
}

#endif // IS_ENABLED(CONFIG_ZMK_SPLIT_WIRED_UART_MODE_DEFAULT_POLLING)

#if IS_ENABLED(CONFIG_ZMK_SPLIT_WIRED_UART_MODE_DEFAULT_INTERRUPT)

void zmk_split_wired_fifo_read(const struct device *dev, struct ring_buf *buf,
                               struct k_work *process_work) {
    // TODO: Add error checking on platforms that support it
    uint32_t last_read = 0, len = 0;
    do {
        uint8_t *buffer;
        len = ring_buf_put_claim(buf, &buffer, buf->size);
        if (len > 0) {
            last_read = uart_fifo_read(dev, buffer, len);

            ring_buf_put_finish(buf, last_read);
        } else {
            LOG_ERR("Dropping incoming RPC byte, insufficient room in the RX buffer. Bump "
                    "CONFIG_ZMK_STUDIO_RPC_RX_BUF_SIZE.");
            uint8_t dummy;
            last_read = uart_fifo_read(dev, &dummy, 1);
        }
    } while (last_read && last_read == len);

    k_work_submit(process_work);
}

void zmk_split_wired_fifo_fill(const struct device *dev, struct ring_buf *tx_buf) {
    uint32_t len;
    while ((len = ring_buf_size_get(tx_buf)) > 0) {
        uint8_t *buf;
        uint32_t claim_len = ring_buf_get_claim(tx_buf, &buf, tx_buf->size);

        if (claim_len <= 0) {
            break;
        }

        int sent = uart_fifo_fill(dev, buf, claim_len);

        ring_buf_get_finish(tx_buf, MAX(sent, 0));

        if (sent <= 0) {
            break;
        }
    }

    if (ring_buf_size_get(tx_buf) == 0) {
        uart_irq_tx_disable(dev);
    }
}

#endif // IS_ENABLED(CONFIG_ZMK_SPLIT_WIRED_UART_MODE_DEFAULT_INTERRUPT)

int zmk_split_wired_get_item(struct ring_buf *rx_buf, uint8_t *env, size_t env_size) {
    while (ring_buf_size_get(rx_buf) >= env_size) {
        size_t bytes_left = env_size;

        uint8_t prefix[5] = {0};

        uint32_t peek_read = ring_buf_peek(rx_buf, prefix, 4);
        __ASSERT(peek_read == 4, "Somehow read less than we expect from the RX buffer");

        if (strcmp(prefix, ZMK_SPLIT_WIRED_ENVELOPE_MAGIC_PREFIX) != 0) {
            uint8_t discarded_byte;
            ring_buf_get(rx_buf, &discarded_byte, 1);

            LOG_WRN("Prefix mismatch, discarding byte %0x", discarded_byte);

            continue;
        }

        while (bytes_left > 0) {
            size_t read = ring_buf_get(rx_buf, env + (env_size - bytes_left), bytes_left);
            bytes_left -= read;
        }

        LOG_HEXDUMP_DBG(env, env_size, "Env data");

        // Avoid unaligned access by copying into our 32-bit integer on the stack
        uint32_t env_crc;
        memcpy(&env_crc, env + (env_size - 4), 4);

        // Exclude the trailing 4 bytes that contain the received CRC
        uint32_t crc = crc32_ieee(env, env_size - 4);
        if (crc != env_crc) {
            LOG_WRN("Data corruption in received peripheral event, ignoring %d vs %d", crc,
                    env_crc);
            return -EINVAL;
        }

        return 0;
    }

    return -EAGAIN;
}