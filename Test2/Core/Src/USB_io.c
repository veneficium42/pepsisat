/*
 * USB_io.c
 *
 *  Created on: 22 apr 2026
 *      Author: Bruno
 */

#include "stm32h7xx_hal.h"
#include "usbd_cdc_if.h"
#include "usb_io.h"

static usb_tx_state_t tx_state = USB_TX_IDLE;

static uint8_t header[8];

static uint8_t *tx_buf = NULL;
static uint32_t tx_len = 0;
static uint32_t tx_offset = 0;

static uint8_t *payload_buf = NULL;
static uint32_t payload_len = 0;

void USB_SendBuffer(uint8_t *buf, uint32_t len)
{
    tx_buf = buf;
    tx_len = len;
    tx_offset = 0;

    USB_SendNextChunk();
}

void USB_SendFrame(uint8_t *frame, uint32_t len)
{
    if (tx_state != USB_TX_IDLE) return;  // busy

    const uint8_t sync[4] = {0xAA, 0x55, 0xAA, 0x55};

    memcpy(header, sync, 4);
    memcpy(header + 4, &len, 4);

    // prepare header transmission
    tx_buf = header;
    tx_len = 8;
    tx_offset = 0;

    // store payload for next phase
    payload_buf = frame;
    payload_len = len;

    tx_state = USB_TX_HEADER;

    USB_SendNextChunk();
}

void USB_SendNextChunk(void)
{
    if (tx_offset >= tx_len)
    {
        // phase completed
        if (tx_state == USB_TX_HEADER)
        {
            // switch to payload
            tx_buf = payload_buf;
            tx_len = payload_len;
            tx_offset = 0;

            tx_state = USB_TX_PAYLOAD;
        }
        else
        {
            // all done
            tx_state = USB_TX_IDLE;

    		// Resume DCMI
    		HAL_DCMI_Resume(&hdcmi);

            return;
        }
    }

    uint16_t chunk = (tx_len - tx_offset > 64) ? 64 : (tx_len - tx_offset);

    if (CDC_Transmit_FS(tx_buf + tx_offset, chunk) == USBD_OK)
    {
        tx_offset += chunk;
    }
}


