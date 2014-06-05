/*----------------------------------------------------------------------------
 *      R T L  -  T C P N E T
 *----------------------------------------------------------------------------
 *      Name:    LM3S_EMAC.C
 *      Purpose: Wrapper for the CMSIS LM3S EMAC driver
 *      Rev.:    V3.22
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2004-2008 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <stddef.h>
#include <Net_Config.h>
#include "EthDev.h"

extern U8 own_hw_adr[];                                    /* Net_Config.c    */
extern EthDev_IOB *pEth;

/*----------------------------------------------------------------------------
  Ethernet Device
  callback frame received
 *----------------------------------------------------------------------------*/
void *RxFrame (int size) {
  OS_FRAME *frame;

  frame = alloc_mem (size | 0x80000000);                   /* Flag 0x80000000 to skip sys_error()  */
                                                           /* call when out of memory.             */
  if (frame != NULL) {                                     /* 'alloc_mem()' ok */
    put_in_queue (frame);
    return (&frame->data[0]);
  }
  return (NULL);
}

/*----------------------------------------------------------------------------
  Ethernet Device
  callback frame copied
 *----------------------------------------------------------------------------*/
void RxFrameReady (int size) {
  int frameSize;

  frameSize = size;                                        /* test purpose only */
}

/*--------------------------- init_ethernet ---------------------------------*/

void init_ethernet (void) {
  int i;
  unsigned char McAddr[6];

  pEth->Mode = EthDev_MODE_AUTO;

  for (i = 0; i < 6; i++) {
    pEth->HwAddr[i] = own_hw_adr[i];
  }

  pEth->RxFrame = RxFrame;
  pEth->RxFrameReady = RxFrameReady;
  pEth->Init ();
  
  pEth->SetMCFilter (1, McAddr);

}

/*--------------------------- send_frame ------------------------------------*/

void send_frame (OS_FRAME *frame) {
  pEth->TxFrame (frame->data, frame->length);
}

/*--------------------------- int_enable_eth --------------------------------*/

void int_enable_eth (void) {
  pEth->UnLock ();
}


/*--------------------------- int_disable_eth -------------------------------*/

void int_disable_eth (void) {
  pEth->Lock ();
}


/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
