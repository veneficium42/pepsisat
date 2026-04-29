/*
 * USB_io.h
 *
 *  Created on: 22 apr 2026
 *      Author: matte
 */

#ifndef INC_USB_IO_H_
#define INC_USB_IO_H_

	extern USBD_HandleTypeDef hUsbDeviceFS;
	extern DCMI_HandleTypeDef hdcmi;

	typedef enum {
	    USB_TX_IDLE = 0,
	    USB_TX_HEADER,
	    USB_TX_PAYLOAD
	} usb_tx_state_t;

	//=======================================================================================================
	//												FUNCTIONS
	//=======================================================================================================
	void USB_SendFrame(uint8_t *frame, uint32_t len);
	void USB_SendBuffer(uint8_t *buf, uint32_t len);
	void USB_SendNextChunk(void);

#endif /* INC_USB_IO_H_ */
