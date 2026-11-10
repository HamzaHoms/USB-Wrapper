/*
 * usbd_cdc_wrapper.h
 *
 *  Created on: Oct 15, 2026
 *      Author: Hamza
 */

#ifndef INC_USB_CDC_WRAPPER_H_
#define INC_USB_CDC_WRAPPER_H_

#include "usbd_cdc.h"
#include "usb_device.h"
#include "buffers.h"
#include <stdbool.h>

#define USBD_TX_PACKET_BUF_SIZE 64

typedef struct{
  USBD_HandleTypeDef* _husb;
  int8_t _tx_sts;
  uint32_t _last_tx_tick;
  bool _tx_zlp_req;

  FIFO_t* _rx_fifo;

  FIFO_t* _tx_fifo;
  uint8_t _tx_buf_int[USBD_TX_PACKET_BUF_SIZE];

  void (*_DataReady_Callback)(void);
} wUSBD;

void USB_Init(wUSBD* wusb,   USBD_HandleTypeDef* hUSBD);
void USB_SetDRCallback(wUSBD* wusb,   void (*dr_callback_func)(void));
int8_t USB_CDC_Receive_FS_Impl(wUSBD* wUSBD, uint8_t* buf, uint32_t *len);
int8_t USB_ReadRXFIFO_G(wUSBD* wUSBD, uint8_t* buf, uint32_t len);
uint32_t USB_ReadRXFIFO(wUSBD* wUSBD, uint8_t* buf, uint32_t len);
int8_t USB_WriteTX(wUSBD* wUSBD, uint8_t* buf, uint32_t len);
int8_t _USB_TX(wUSBD* wusb, uint8_t* buf, uint32_t len);
void USB_CDC_TransmitCplt_FS_Impl(wUSBD* wUSBD);



#define USBD_RX_FIFO_SIZE 512

#define USBD_TX_FIFO_SIZE 512


#define USB_CDC_TX_TIMEOUT_MS 50

#endif /* INC_USB_CDC_WRAPPER_H_ */
