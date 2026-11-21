/*
 * usb_cdc_wrapper.c
 *
 *  Created on: Oct 15, 2026
 *      Author: Hamza
 */


#include "usb_cdc_wrapper.h"

void USB_Init(wUSBD* wusb,   USBD_HandleTypeDef* hUSBD)
{
  //Initializes the USB wrapper
  //wusb is the wrapper to be initialized
  //hUSBD is the usb handler that is going to be used
  //this function resets the usb wrapper state and creates the fifos

  wusb->_husb = hUSBD;
  wusb->_tx_sts = USBD_OK;
  wusb->_DataReady_Callback = NULL;

  FIFO_Init(&wusb->_rx_fifo, wusb->_rx_fifo_buf, USBD_RX_FIFO_SIZE);
  FIFO_Init(&wusb->_tx_fifo, wusb->_tx_fifo_buf, USBD_TX_FIFO_SIZE);

}

void USB_SetDRCallback(wUSBD* wusb,   void (*dr_callback_func)(void))
{
  //this functions sets the function pointer of the callback that is called when data is ready
  wusb->_DataReady_Callback = dr_callback_func;
}

int8_t USB_CDC_Receive_FS_Impl(wUSBD* wusb, uint8_t* buf, uint32_t *len)
{
  //this is an implementation function that is meant to be called inside of the usb cdc receive function
  //it reads 'len' bytes from the usb 'buf' and into the wrapper's fifo
  //returns USBD_Busy if the fifo does not have enough space

  BUFFER_Status sts = FIFO_Append(&wusb->_rx_fifo, buf, *len);

  USBD_CDC_SetRxBuffer(wusb->_husb, &buf[0]);
  USBD_CDC_ReceivePacket(wusb->_husb);

  if(sts == BUF_ERROR) return USBD_FAIL;

  if(wusb->_DataReady_Callback != NULL) wusb->_DataReady_Callback();
  return USBD_OK;
}

int8_t USB_ReadRXFIFO_G(wUSBD* wusb, uint8_t* buf, uint32_t len)
{
  //does a guarded pop of the usb rx fifo into buf
  //returns USBD_OK for a successful pop
  //returns USBD_FAIL for an unsuccessful pop
  BUFFER_Status sts = FIFO_Pop_G(&wusb->_rx_fifo, buf, len);
  if(sts == BUF_ERROR) return USBD_FAIL;

  return USBD_OK;
}

uint32_t USB_ReadRXFIFO(wUSBD* wusb, uint8_t* buf, uint32_t len)
{
  //does an unguarded pop of the usb rx fifo into buf and returns number of bytes popped
  return FIFO_Pop(&wusb->_rx_fifo, buf, len);
}

int8_t _USB_TX(wUSBD* wusb, uint8_t* buf, uint32_t len)
{
  //internal function that calls the transmit functions of the usb peripheral
  //returns the status returned by USBD_CDC_TransmitPacket
  //sets the usb state to USBD_FAIL if the transmit failed

  USBD_CDC_SetTxBuffer(wusb->_husb, buf, len);
  int8_t sts = USBD_CDC_TransmitPacket(wusb->_husb);
  if(sts == USBD_FAIL) wusb->_tx_sts = USBD_FAIL;
  return sts;
}

int8_t USB_WriteTX(wUSBD* wusb, uint8_t* buf, uint32_t len)
{
  //writes and transmits 'len' bytes of the passed buffer
  //bytes are appended to the tx fifo, then immediatly popped out if the peripheral is clear
  //if the peripheral is still busy the ISR pops them
  //if the tx fifo is full, the function returns USBD_FAIL
  //if the usb peripheral is free the transmit is started immediatly
  //if the return is USBD_fail then that means a full fifo or failed TX

  if(HAL_GetTick() - wusb->_last_tx_tick > USB_CDC_TX_TIMEOUT_MS && wusb->_tx_sts == USBD_BUSY)
    {
      __disable_irq();

      FIFO_Clear(&wusb->_tx_fifo);
      USBD_LL_FlushEP(wusb->_husb, CDC_IN_EP);
      ((USBD_CDC_HandleTypeDef*)wusb->_husb->pClassData)->TxState = 0;
      wusb->_tx_zlp_req = false;
      wusb->_tx_sts = USBD_OK;

      __enable_irq();
    }

  if(FIFO_Append(&wusb->_tx_fifo, buf, len) != BUF_OK) return USBD_FAIL;

  if(FIFO_GetCount(&wusb->_tx_fifo) % 64 == 0) wusb->_tx_zlp_req = true;
  else wusb->_tx_zlp_req = false;

  if(wusb->_tx_sts == USBD_OK)
    {
      wusb->_tx_sts = USBD_BUSY;

      uint32_t pop_size = FIFO_Pop(&wusb->_tx_fifo, wusb->_tx_buf_inter, USBD_TX_PACKET_BUF_SIZE);

      wusb->_last_tx_tick = HAL_GetTick();

      return _USB_TX(wusb, wusb->_tx_buf_inter, pop_size);
    }

  return USBD_OK;
}

void USB_CDC_TransmitCplt_FS_Impl(wUSBD* wusb)
{
  //function that is called on the completion of a packet transmission with usb
  //it deals with ZLP internally and restarts the transmit if the fifo is still not empty
  //if the transmit fails the usb status variable is set to USBD_FAIL

  if(FIFO_GetCount(&wusb->_tx_fifo) == 0)
    {
      if(wusb->_tx_zlp_req)
	{
	  wusb->_last_tx_tick = HAL_GetTick();

	  if(_USB_TX(wusb, wusb->_tx_buf_inter, 0) != USBD_OK) return;
	  wusb->_tx_zlp_req = false;

	  return;
	}

      wusb->_tx_sts = USBD_OK;
      return;
    }

  uint32_t pop_size = FIFO_Pop(&wusb->_tx_fifo, wusb->_tx_buf_inter, USBD_TX_PACKET_BUF_SIZE);

  wusb->_last_tx_tick = HAL_GetTick();

  _USB_TX(wusb, wusb->_tx_buf_inter, pop_size);
}
