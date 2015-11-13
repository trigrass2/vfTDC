/*
 * File:
 *    vfTDCLibTest.c
 *
 * Description:
 *    Test vfTDC Library readout with pipeline TI
 *
 *
 */


#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "jvme.h"
#include "tiLib.h"
#include "vfTDCLib.h"
/* #include "remexLib.h" */

DMA_MEM_ID vmeIN,vmeOUT;
extern DMANODE *the_event;
extern unsigned int *dma_dabufp;

extern int tiA32Base;

#define BLOCKLEVEL 1

#define DO_READOUT

/* Interrupt Service routine */
void
mytiISR(int arg)
{
  volatile unsigned short reg;
  int dCnt, len=0,idata;
  DMANODE *outEvent;
  int blkReady=0, timeout=0;
  int printout = 1;

  unsigned int tiIntCount = tiGetIntCount();

#ifdef DO_READOUT
  GETEVENT(vmeIN,tiIntCount);

#ifdef DOINT
  blkReady = tiBReady();
  if(blkReady==ERROR)
    {
      printf("%s: ERROR: tiIntPoll returned ERROR.\n",__FUNCTION__);
      return;
    }

  if(blkReady==0 && timeout<100)
    {
      printf("NOT READY!\n");
      blkReady=tiBReady();
      timeout++;
    }

  if(timeout>=100)
    {
      printf("TIMEOUT!\n");
      return;
    }
#endif

/*   dCnt = tiReadBlock(dma_dabufp,3*BLOCKLEVEL+10,1); */
  dCnt = tiReadTriggerBlock(dma_dabufp);
  if(dCnt<=0)
    {
      printf("No data or error.  dCnt = %d\n",dCnt);
    }
  else
    {
      tiCheckTriggerBlock(dma_dabufp);
/*       dma_dabufp += dCnt; */
      /*       printf("dCnt = %d\n",dCnt); */
    
    }

  timeout=0;
  blkReady = vfTDCBReady(0);
  if(blkReady==0 && timeout<100)
    {
      printf("NOT READY!\n");
      blkReady = vfTDCBReady(0);
      timeout++;
    }

  if(timeout>=100)
    {
      printf("TIMEOUT!\n");
      return;
    }

  dCnt = vfTDCReadBlock(0,dma_dabufp,BLOCKLEVEL*(10*192+10),1);
  if(dCnt<=0)
    {
      printf("No data or error.  dCnt = %d\n",dCnt);
    }
  else
    {
      dma_dabufp += dCnt;
    }



  PUTEVENT(vmeOUT);

  outEvent = dmaPGetItem(vmeOUT);
#define READOUT
#ifdef READOUT
  if(tiIntCount%printout==0)
    {
      printf("Received %d triggers...\n",
	     tiIntCount);

      len = outEvent->length;
      
      for(idata=0;idata<len;idata++)
	{
/* 	  if((idata%5)==0) printf("\n\t"); */
/* 	  printf("  0x%08x ",(unsigned int)LSWAP(outEvent->data[idata])); */
	  vfTDCDataDecode(LSWAP(outEvent->data[idata]));
	}
      printf("\n\n");
    }
#endif
  dmaPFreeItem(outEvent);
#else /* DO_READOUT */
  /*   tiResetBlockReadout(); */

#endif /* DO_READOUT */
  if(tiIntCount%printout==0)
    printf("intCount = %d\n",tiIntCount );
/*     sleep(1); */

  static int bl = BLOCKLEVEL;
  if(tiGetSyncEventFlag())                                                      
    {                                                                           
/*       tiSetBlockLevel(bl++);                                               */
      printf("SE: Curr BL = %d\n",tiGetCurrentBlockLevel());                    
      printf("SE: Next BL = %d\n",tiGetNextBlockLevel());                       
    }                                                                           
 
}


int 
main(int argc, char *argv[]) {

  int stat;

  printf("\nJLAB vfTDC Tests\n");
  printf("----------------------------\n");

/*   remexSetCmsgServer("dafarm28"); */
/*   remexInit(NULL,1); */

  vmeOpenDefaultWindows();

  /* Setup Address and data modes for DMA transfers
   *   
   *  vmeDmaConfig(addrType, dataType, sstMode);
   *
   *  addrType = 0 (A16)    1 (A24)    2 (A32)
   *  dataType = 0 (D16)    1 (D32)    2 (BLK32) 3 (MBLK) 4 (2eVME) 5 (2eSST)
   *  sstMode  = 0 (SST160) 1 (SST267) 2 (SST320)
   */
  vmeDmaConfig(2,5,1);

  /* INIT dmaPList */

  dmaPFreeAll();
  vmeIN  = dmaPCreate("vmeIN",10244,500,0);
  vmeOUT = dmaPCreate("vmeOUT",0,0,0);
    
  dmaPStatsAll();

  dmaPReInitAll();

  /*     gefVmeSetDebugFlags(vmeHdl,0x0); */
  /* Set the TI structure pointer */
  /*     tiInit((2<<19),TI_READOUT_EXT_POLL,0); */
  tiA32Base=0x08000000;
  if(tiInit(0,TI_READOUT_EXT_POLL,0)!=OK)
    goto CLOSE;

  tiCheckAddresses();

  tiSetSyncEventInterval(10);

  tiSetEventFormat(3);

  char mySN[20];
  printf("0x%08x\n",tiGetSerialNumber((char **)&mySN));
  printf("mySN = %s\n",mySN);

#ifndef DO_READOUT
  tiDisableDataReadout();
  tiDisableA32();
#endif

  tiLoadTriggerTable(0);
    
  tiSetTriggerHoldoff(1,4,0);
  tiSetTriggerHoldoff(2,4,0);

  tiSetPrescale(0);
  tiSetBlockLevel(BLOCKLEVEL);

  stat = tiIntConnect(TI_INT_VEC, mytiISR, 0);
  if (stat != OK) 
    {
      printf("ERROR: tiIntConnect failed \n");
      goto CLOSE;
    } 
  else 
    {
      printf("INFO: Attached TI Interrupt\n");
    }

  /*     tiSetTriggerSource(TI_TRIGGER_TSINPUTS); */
  tiSetTriggerSource(TI_TRIGGER_PULSER);
  tiEnableTSInput(0x1);

  /*     tiSetFPInput(0x0); */
  /*     tiSetGenInput(0xffff); */
  /*     tiSetGTPInput(0x0); */

/*   tiSetBusySource(TI_BUSY_LOOPBACK,1); */

  tiSetBlockBufferLevel(1);

  tiSetFiberDelay(1,2);
  tiSetSyncDelayWidth(1,0x3f,1);
    
  tiSetBlockLimit(10);

  /*************************************************************/
  /* VFTDC initialization                                      */
  /*************************************************************/
  extern unsigned int vfTDCA32Base;

  vfTDCA32Base=0x09000000;
  vfTDCInit(14<<19, 1<<19, 1, 
	    VFTDC_INIT_VXS_SYNCRESET |
	    VFTDC_INIT_VXS_TRIG      |
	    VFTDC_INIT_VXS_CLKSRC);

  vfTDCSetBlockLevel(0, BLOCKLEVEL);
  vfTDCSetWindowParamters(0, 1, 250);
  vfTDCStatus(0,0);

  printf("Hit enter to reset stuff\n");
  getchar();

  tiClockReset();
  taskDelay(1);
  tiTrigLinkReset();
  taskDelay(1);
  tiEnableVXSSignals();

  int again=0;
 AGAIN:
  taskDelay(1);

  tiSyncReset(1);

  taskDelay(1);
    
  tiStatus(1);
  vfTDCStatus(0,0);

  printf("Hit enter to start triggers\n");
  getchar();

  tiIntEnable(0);
  tiStatus(1);
#define SOFTTRIG
#ifdef SOFTTRIG
  tiSetRandomTrigger(1,0x7);
/*   taskDelay(10); */
/*   tiSoftTrig(1,0x1,0x700,0); */
#endif

  printf("Hit any key to Disable TID and exit.\n");
  getchar();
  tiStatus(1);

#ifdef SOFTTRIG
  /* No more soft triggers */
  /*     tidSoftTrig(0x0,0x8888,0); */
  tiSoftTrig(1,0,0x700,0);
  tiDisableRandomTrigger();
#endif

  tiIntDisable();

  tiIntDisconnect();

  vfTDCStatus(0,0);

  if(again==1)
    {
      again=0;
      goto AGAIN;
    }


 CLOSE:

  dmaPFreeAll();
  vmeCloseDefaultWindows();

  exit(0);
}

