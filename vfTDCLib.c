/*----------------------------------------------------------------------------*/
/**
 * @mainpage
 * <pre>
 *  Copyright (c) 2015        Southeastern Universities Research Association, *
 *                            Thomas Jefferson National Accelerator Facility  *
 *                                                                            *
 *    This software was developed under a United States Government license    *
 *    described in the NOTICE file included as part of this distribution.     *
 *                                                                            *
 *    Authors: Bryan Moffit                                                   *
 *             moffit@jlab.org                   Jefferson Lab, MS-12B3       *
 *             Phone: (757) 269-5660             12000 Jefferson Ave.         *
 *             Fax:   (757) 269-5800             Newport News, VA 23606       *
 *                                                                            *
 *----------------------------------------------------------------------------*
 *
 * Description:
 *     Jefferson Lab VXS FPGA-Based Time to Digital Converter module library.
 *
 * </pre>
 *----------------------------------------------------------------------------*/

#define _GNU_SOURCE

#ifdef VXWORKS
#include <vxWorks.h>
#include <sysLib.h>
#include <logLib.h>
#include <taskLib.h>
#include <intLib.h>
#include <iv.h>
#include <semLib.h>
#include <vxLib.h>
#include "vxCompat.h"
#include "../jvme/jvme.h"
#else 
#include <sys/prctl.h>
#include <unistd.h>
#include "jvme.h"
#endif
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include "vfTDCLib.h"

/* Mutex to guard TI read/writes */
pthread_mutex_t   vfTDCMutex = PTHREAD_MUTEX_INITIALIZER;
#define VLOCK     if(pthread_mutex_lock(&vfTDCMutex)<0) perror("pthread_mutex_lock");
#define VUNLOCK   if(pthread_mutex_unlock(&vfTDCMutex)<0) perror("pthread_mutex_unlock");

/* Global Variables */
volatile struct vfTDC_struct       *TDCp[VFTDC_MAX_BOARDS+1];  /* pointer to vfTDC memory map */
volatile        unsigned int       *TDCpd[VFTDC_MAX_BOARDS+1]; /* pointer to vfTDC data FIFO */
volatile        unsigned int       *TDCpmb=NULL;/* pointer to vfTDC data FIFO */
int                 vfTDCID[VFTDC_MAX_BOARDS];      /* array of slot numbers for FADCs */
unsigned int        vfTDCAddrList[VFTDC_MAX_BOARDS];            /* array of a24 addresses for FADCs */
unsigned long       vfTDCA24Offset   = 0;       /* Difference in CPU A24 Base and VME A24 Base */
unsigned int        vfTDCA32Base     = 0x08000000;   /* Minimum VME A32 Address for use by TI */
unsigned long       vfTDCA32Offset   = 0;       /* Difference in CPU A32 Base and VME A32 Base */
unsigned int        vfTDCIntCount    = 0;
unsigned int        vfTDCAckCount    = 0;
int                 vfTDCDoAck       = 0;
int                 vfTDCNeedAck     = 0;
static BOOL         vfTDCIntRunning  = FALSE;   /* running flag */
static VOIDFUNCPTR  vfTDCIntRoutine  = NULL;    /* user intererrupt service routine */
static int          vfTDCIntArg      = 0;       /* arg to user routine */
static unsigned int vfTDCIntLevel    = VFTDC_INT_LEVEL;       /* VME Interrupt level */
static unsigned int vfTDCIntVec      = VFTDC_INT_VEC;  /* default interrupt vector */
#ifdef NOTYET
static VOIDFUNCPTR  vfTDCAckRoutine  = NULL;    /* user trigger acknowledge routine */
static int          vfTDCAckArg      = 0;       /* arg to user trigger ack routine */
#endif
int                 vfTDCBlockError  = VFTDC_BLOCKERROR_NO_ERROR; /* Whether (>0) or not (0) Block Transfer had an error */
int                 nvfTDC           = 0;       /* Number of initialized TDCs */

/* Interrupt/Polling routine prototypes (static) */
#ifdef NOTYET
static void vfTDCInt(void);
#ifndef VXWORKS
static void vfTDCPoll(void);
static void vfTDCStartPollingThread(void);
/* polling thread pthread and pthread_attr */
pthread_attr_t vfTDCpollthread_attr;
pthread_t      vfTDCpollthread;
#endif
#endif

#ifdef VXWORKS
extern  int sysBusToLocalAdrs(int, char *, char **);
extern  int intDisconnect(int);
extern  int sysIntEnable(int);
IMPORT  STATUS sysIntDisable(int);
IMPORT  STATUS sysVmeDmaDone(int, int);
IMPORT  STATUS sysVmeDmaSend(UINT32, UINT32, int, BOOL);
#endif

/**
 * @defgroup PreInit Pre-Initialization
 * @defgroup Config Initialization/Configuration
 * @defgroup Status Status
 * @defgroup Readout Data Readout
 * @defgroup IntPoll Interrupt/Polling
 * @defgroup Deprec Deprecated - To be removed
 */


/**
 *  @ingroup Config
 *  @brief Initialize JLAB vfTDC Library. 
 *
 * @param addr
 *  - A24 VME Address of the vfTDC
 * @param addr_inc
 *  - Amount to increment addr to find the next vfTDC
 * @param ntdc
 *  - Number of times to increment
 *
 *  @param Flag 18 bit integer
 * <pre>
 *       Low 7 bits - Specifies the default Signal distribution (clock,trigger) 
 *                    sources for the board (Internal, FrontPanel, VXS, VME(Soft))
 *       bit  1-0:  defines Sync Reset source
 *                   0 0  VME (Software Sync-Reset)
 *                   0 1  HFBR1
 *                   1 0  VXS
 *       bits 4-2  defines Trigger source
 *             0 0 0  VME (Software Triggers)
 *             0 0 1  HFBR1
 *             0 1 0  VXS
 *       bits 6-5:  defines Clock Source
 *         0 0  Internal 250MHz Clock
 *         0 1  HFBR1
 *         1 0  VXS
 * </pre>
 *
 * <pre>
 *      bit 16:  Exit before board initialization
 *             0 Initialize FADC (default behavior)
 *             1 Skip initialization (just setup register map pointers)
 *
 *      bit 17:  Use fadcAddrList instead of addr and addr_inc
 *               for VME addresses.
 *             0 Initialize with addr and addr_inc
 *             1 Use fadcAddrList 
 *
 *      bit 18:  Skip firmware check.  Useful for firmware updating.
 *             0 Perform firmware check
 *             1 Skip firmware check
 * </pre>
 *      
 *
 * @return OK, or ERROR if the address is invalid or a board is not present.
 */
STATUS 
vfTDCInit(UINT32 addr, UINT32 addr_inc, int ntdc, int iFlag)
{
  int ii, res, errFlag = 0;
  int boardID = 0;
  int maxSlot = 1;
  int minSlot = 21;
  int trigSrc=0, clkSrc=0, srSrc=0;
  unsigned int rdata=0, a32addr=0, wreg=0, fwvers=0;
  unsigned long laddr=0, laddr_inc=0;
  volatile struct vfTDC_struct *ft;
  int noBoardInit=0;
  int useList=0;
  int noFirmwareCheck=0;

  /* Check if we are to exit when pointers are setup */
  noBoardInit     = (iFlag&VFTDC_INIT_SKIP)>>16;

  /* Check if we're initializing using a list */
  useList         = (iFlag&VFTDC_INIT_USE_ADDRLIST)>>17;

  /* Are we skipping the firmware check? */
  noFirmwareCheck = (iFlag&VFTDC_INIT_SKIP_FIRMWARE_CHECK)>>18;

  /* Determine clock, sync, and trigger sources */
  srSrc   = (iFlag&VFTDC_INIT_SYNCRESETSRC_MASK);
  trigSrc = (iFlag&VFTDC_INIT_TRIGSRC_MASK);
  clkSrc  = (iFlag&VFTDC_INIT_CLKSRC_MASK);

  /* Check for valid address */
  if(addr==0) 
    {
      printf("%s: ERROR: Must specify a Bus (VME-based A24) address for vfTDC 0\n",
	     __FUNCTION__);
      return(ERROR);
    }
  else if(addr > 0x00ffffff) 
    { /* A24 Addressing */
      printf("%s: ERROR: A32 Addressing not allowed for FADC configuration space\n",
	     __FUNCTION__);
      return(ERROR);
    }
  else
    { /* A24 Addressing */
      if( ((addr_inc==0)||(ntdc==0)) && (useList==0) )
	ntdc = 1; /* assume only one vfTDC to initialize */

      /* get the vfTDC address */
#ifdef VXWORKS
      res = sysBusToLocalAdrs(0x39,(char *)addr,(char **)&laddr);
#else
      res = vmeBusToLocalAdrs(0x39,(char *)(unsigned long)addr,(char **)&laddr);
#endif
      if (res != 0) 
	{
#ifdef VXWORKS
	  printf("%s: ERROR in sysBusToLocalAdrs(0x39,0x%x,&laddr) \n",
		 __FUNCTION__,addr);
#else
	  printf("%s: ERROR in vmeBusToLocalAdrs(0x39,0x%x,&laddr) \n",
		 __FUNCTION__,addr);
#endif
	  return(ERROR);
	}
      vfTDCA24Offset = laddr - addr;
    }

  /* Init Some Global variables */
  nvfTDC = 0;
  memset((char *)vfTDCID,0,sizeof(vfTDCID));

  for (ii=0;ii<ntdc;ii++) 
    {
      if(useList==1)
	{
	  laddr_inc = vfTDCAddrList[ii] + vfTDCA24Offset;
	}
      else
	{
	  laddr_inc = laddr +ii*addr_inc;
	}
      ft = (struct vfTDC_struct *)laddr_inc;
      /* Check if Board exists at that address */
#ifdef VXWORKS
      res = vxMemProbe((char *) &(ft->boardID),VX_READ,4,(char *)&rdata);
#else
      res = vmeMemProbe((char *) &(ft->boardID),4,(char *)&rdata);
#endif
      if(res < 0) 
	{
#ifdef VXWORKS
	  printf("%s: WARN: No addressable board at addr=0x%x\n",
		 __FUNCTION__,(UINT32) ft);
#else
	  printf("%s: WARN: No addressable board at VME (Local) addr=0x%x (0x%lx)\n",
		 __FUNCTION__,
		 (UINT32)(laddr_inc-vfTDCA24Offset), (unsigned long) ft);
#endif
	  errFlag = 1;
	  continue;
	}
      else 
	{
	  /* Check that it is an FA board */
	  if(((rdata&VFTDC_BOARDID_TYPE_MASK)>>16) != VFTDC_BOARDID_TYPE_VFTDC) 
	    {
	      printf("%s: WARN: For board at 0x%x, Invalid Board ID: 0x%x\n",
		     __FUNCTION__,
		     (UINT32)(laddr_inc-vfTDCA24Offset), rdata);
	      continue;
	    }
	  else 
	    {
	      /* Check if this is board has a valid slot number */
	      boardID =  (rdata&VFTDC_BOARDID_GEOADR_MASK)>>8;

	      if((boardID <= 0)||(boardID >21)) 
		{
		  printf("%s: ERROR: For Board at 0x%x,  Slot number is not in range: %d\n",
			 __FUNCTION__,(UINT32)(laddr_inc-vfTDCA24Offset), boardID);
		  continue;
		}

	      fwvers = (vmeRead32(&ft->status)&VFTDC_STATUS_FIRMWARE_VERSION_MASK)>>20;
	      if(!noFirmwareCheck)
		{
		  /* Check FPGA firmware version */
		  if( fwvers != VFTDC_SUPPORTED_FIRMWARE )
		    {
		      printf("%s: ERROR: Slot %2d: FPGA Firmware (0x%02x) not supported by this driver.\n",
			     __FUNCTION__,boardID, fwvers);
		      printf("\tUpdate to 0x%02x to use this driver.\n",VFTDC_SUPPORTED_FIRMWARE);
		      continue;
		    }

		}
	      else
		{
		  /* Check FPGA firmware version */
		  if( fwvers != VFTDC_SUPPORTED_FIRMWARE )
		      printf("%s: WARN: Slot %2d: FPGA Firmware (0x%02x) not supported by this driver (ignored).\n",
			     __FUNCTION__,boardID, fwvers);
		}


	      TDCp[boardID] = (struct vfTDC_struct *)(laddr_inc);
	      vfTDCID[nvfTDC] = boardID;
	      if(boardID >= maxSlot) maxSlot = boardID;
	      if(boardID <= minSlot) minSlot = boardID;
	      
	      printf("Initialized VFTDC %2d  Slot #%2d at VME (Local) address 0x%06x (0x%lx) \n",
		     nvfTDC,vfTDCID[nvfTDC],
		     (UINT32) ((unsigned long)(TDCp[(vfTDCID[nvfTDC])])-vfTDCA24Offset),
		     (unsigned long) TDCp[(vfTDCID[nvfTDC])]);
	    }
	  nvfTDC++;
	}
    }

  /* Hard Reset of all VFTDC boards in the Crate */
  if(!noBoardInit)
    {
      for(ii=0;ii<nvfTDC;ii++) 
	{
	  vmeWrite32(&TDCp[vfTDCID[ii]]->reset,VFTDC_RESET_SOFT | VFTDC_RESET_SCALERS_RESET);
	}
      taskDelay(60); 
    }

  /* Initialize Interrupt variables */
  vfTDCIntRunning = FALSE;
  vfTDCIntLevel = VFTDC_VME_INT_LEVEL;
  vfTDCIntVec = VFTDC_VME_INT_VEC;
  vfTDCIntRoutine = NULL;
  vfTDCIntArg = 0;

  /* Calculate the A32 Offset for use in Block Transfers */
#ifdef VXWORKS
  res = sysBusToLocalAdrs(0x09,(char *)vfTDCA32Base,(char **)&laddr);
  if (res != 0) 
    {
      printf("%s: ERROR in sysBusToLocalAdrs(0x09,0x%x,&laddr) \n",
	     __FUNCTION__,vfTDCA32Base);
      return(ERROR);
    } 
  else 
    {
      vfTDCA32Offset = laddr - vfTDCA32Base;
    }
#else
  res = vmeBusToLocalAdrs(0x09,(char *)(unsigned long)vfTDCA32Base,(char **)&laddr);
  if (res != 0) 
    {
      printf("%s: ERROR in vmeBusToLocalAdrs(0x09,0x%x,&laddr) \n",
	     __FUNCTION__,vfTDCA32Base);
      return(ERROR);
    } 
  else 
    {
      vfTDCA32Offset = laddr - vfTDCA32Base;
    }
#endif


  /* Enable Clock, Trigger, and syncReset sources */ 
  if(!noBoardInit)
    {
      
      switch(clkSrc)
	{
	case VFTDC_INIT_INT_CLKSRC:
	  wreg = VFTDC_CLOCK_INTERNAL;
	  break;

	case VFTDC_INIT_HFBR1_CLKSRC:
	  wreg = VFTDC_CLOCK_HFBR1;
	  break;

	case VFTDC_INIT_VXS_CLKSRC:
	  wreg = VFTDC_CLOCK_VXS;
	  break;

	default:
	  printf("%s: ERROR: Invalid Clock Source (%d). Setting internal.\n",
		 __FUNCTION__,clkSrc);
	  clkSrc = VFTDC_INIT_INT_CLKSRC;
	  wreg = VFTDC_CLOCK_INTERNAL;
	  break;
	}

      for(ii=0;ii<nvfTDC;ii++) 
	{
	  vmeWrite32(&TDCp[vfTDCID[ii]]->clock, 
		     (wreg) | (wreg<<2) | (wreg<<4) | (wreg<<6));
	  taskDelay(1);
	  vmeWrite32(&TDCp[vfTDCID[ii]]->reset,VFTDC_RESET_CLK250);
	  taskDelay(1);
	  vmeWrite32(&TDCp[vfTDCID[ii]]->reset,VFTDC_RESET_IODELAY);
	  taskDelay(1);
	  vmeWrite32(&TDCp[vfTDCID[ii]]->reset,VFTDC_RESET_SOFT);
	  taskDelay(1);
	}
      taskDelay(5);

      /* Setup Trigger and Sync Reset sources */
      switch(trigSrc)
	{
	case VFTDC_INIT_SOFT_TRIG:
	  wreg = VFTDC_TRIGSRC_VME;
	  break;

	case VFTDC_INIT_HFBR1_TRIG:
	  wreg = VFTDC_TRIGSRC_HFBR1;
	  break;

	case VFTDC_INIT_VXS_TRIG:
	  wreg = VFTDC_TRIGSRC_VXS;
	  break;

	default:
	  printf("%s: ERROR: Invalid trigger source (%d). Using software.\n",
		 __FUNCTION__,trigSrc>>2);
	  trigSrc= VFTDC_INIT_SOFT_TRIG;
	  wreg = VFTDC_TRIGSRC_VME;
	}

      for(ii=0;ii<nvfTDC;ii++) 
	{
	  vmeWrite32(&TDCp[vfTDCID[ii]]->trigsrc, wreg);
	}

      switch(srSrc)
	{
	case VFTDC_INIT_SOFT_SYNCRESET:
	  wreg = VFTDC_SYNC_VME;
	  break;

	case VFTDC_INIT_HFBR1_SYNCRESET:
	  wreg = VFTDC_SYNC_HFBR1;
	  break;

	case VFTDC_INIT_VXS_SYNCRESET:
	  wreg = VFTDC_SYNC_VXS;
	  break;

	default:
	  printf("%s: ERROR: Invalid syncReset source (%d). Using software\n",
		 __FUNCTION__,srSrc>>5);
	  srSrc = VFTDC_INIT_SOFT_SYNCRESET;
	  wreg = VFTDC_SYNC_VME;
	}

      for(ii=0;ii<nvfTDC;ii++) 
	{
	  vmeWrite32(&TDCp[vfTDCID[ii]]->sync, wreg);
	}

    }

  /* Write A32 configuration registers with default blocklevel */
  for(ii=0;ii<nvfTDC;ii++) 
    {
    
      /* Program an A32 access address for this VFTDC's FIFO */
      a32addr = vfTDCA32Base + ii*VFTDC_MAX_A32_MEM;
#ifdef VXWORKS
      res = sysBusToLocalAdrs(0x09,(char *)a32addr,(char **)&laddr);
      if (res != 0) 
	{
	  printf("%s: ERROR in sysBusToLocalAdrs(0x09,0x%x,&laddr) \n",
		 __FUNCTION__,a32addr);
	  return(ERROR);
	}
#else
      res = vmeBusToLocalAdrs(0x09,(char *)(unsigned long)a32addr,(char **)&laddr);
      if (res != 0) 
	{
	  printf("%s: ERROR in vmeBusToLocalAdrs(0x09,0x%x,&laddr) \n",
		 __FUNCTION__,a32addr);
	  return(ERROR);
	}
#endif
      TDCpd[vfTDCID[ii]] = (unsigned int *)(laddr);  /* Set a pointer to the FIFO */
      if(!noBoardInit)
	{
	  vmeWrite32(&TDCp[vfTDCID[ii]]->adr32,a32addr);  /* Write the register */
	  vmeWrite32(&TDCp[vfTDCID[ii]]->vmeControl,
		     vmeRead32(&TDCp[vfTDCID[ii]]->vmeControl) | 
		     VFTDC_VMECONTROL_A32 | VFTDC_VMECONTROL_BERR);
	
	  /* Set Default Block Level to 1 */
	  vmeWrite32(&TDCp[vfTDCID[ii]]->blocklevel,1);

	}

    }

#ifdef NOTYET
  /* If there are more than 1 VFTDC in the crate then setup the Muliblock Address
     window. This must be the same on each board in the crate */
  if(nvfTDC > 1) 
    {
      a32addr = vfTDCA32Base + (nvfTDC+1)*VFTDC_MAX_A32_MEM; /* set MB base above individual board base */
#ifdef VXWORKS
      res = sysBusToLocalAdrs(0x09,(char *)a32addr,(char **)&laddr);
      if (res != 0) 
	{
	  printf("%s: ERROR in sysBusToLocalAdrs(0x09,0x%x,&laddr) \n",
		 __FUNCTION__,a32addr);
	  return(ERROR);
	}
#else
      res = vmeBusToLocalAdrs(0x09,(char *)(unsigned long)a32addr,(char **)&laddr);
      if (res != 0) 
	{
	  printf("%s: ERROR in vmeBusToLocalAdrs(0x09,0x%x,&laddr) \n",
		 __FUNCTION__,a32addr);
	  return(ERROR);
	}
#endif
      TDCpmb = (unsigned int *)(laddr);  /* Set a pointer to the FIFO */
      if(!noBoardInit)
	{
	  for (ii=0;ii<nvfTDC;ii++) 
	    {
	      /* Write the register and enable */
	      vmeWrite32(&TDCp[vfTDCID[ii]]->adr_mb,
			 (a32addr+VFTDC_MAX_A32MB_SIZE) + (a32addr>>16) + VFTDC_A32_ENABLE);
	    }
	}    
      /* Set First Board and Last Board */
      vfTDCMaxSlot = maxSlot;
      vfTDCMinSlot = minSlot;
      if(!noBoardInit)
	{
	  vmeWrite32(&TDCp[minSlot]->ctrl1,
		     vmeRead32(&(TDCp[minSlot]->ctrl1)) | VFTDC_FIRST_BOARD);
	  vmeWrite32(&TDCp[maxSlot]->ctrl1,
		     vmeRead32(&(TDCp[maxSlot]->ctrl1)) | VFTDC_LAST_BOARD);
	}    
    }
#endif /* NOTYET */

  if(errFlag > 0) 
    {
      printf("%s: WARN: Unable to initialize all requested VFTDC Modules (%d)\n",
	     __FUNCTION__,ntdc);
      if(nvfTDC > 0)
	printf("%s: %d VFTDC(s) successfully initialized\n",__FUNCTION__,nvfTDC );
      return(ERROR);
    } 
  else 
    {
      return(OK);
    }
}

int
vfTDCCheckAddresses()
{
  unsigned long offset=0, expected=0, base=0;
  struct vfTDC_struct *tp;
  int rval=OK;

  printf("%s:\n\t ---------- Checking VFTDC address space ---------- \n",__FUNCTION__);

  base = (unsigned long) &tp->boardID;

  offset = ((unsigned long) &tp->trigsrc) - base;
  expected = 0x20;
  if(offset != expected)
    {
      printf("%s: ERROR TDCp->trigSrc not at offset = 0x%lx (@ 0x%lx)\n",
	     __FUNCTION__,expected,offset);
      rval = ERROR;
    }

  offset = ((unsigned long) &tp->blockBuffer) - base;
  expected = 0x4c;
  if(offset != expected)
    {
      printf("%s: ERROR TDCp->blockBuffer not at offset = 0x%lx (@ 0x%lx)\n",
	     __FUNCTION__,expected,offset);
      rval = ERROR;
    }

  offset = ((unsigned long) &tp->livetime) - base;
  expected = 0xa8;
  if(offset != expected)
    {
      printf("%s: ERROR TDCp->livetime not at offset = 0x%lx (@ 0x%lx)\n",
	     __FUNCTION__,expected,offset);
      rval = ERROR;
    }

  offset = ((unsigned long) &tp->reset) - base;
  expected = 0x100;
  if(offset != expected)
    {
      printf("%s: ERROR TDCp->reset not at offset = 0x%lx (@ 0x%lx)\n",
	     __FUNCTION__,expected,offset);
      rval = ERROR;
    }

  return rval;
}

/**
 * @ingroup Status
 * @brief Print some status information of the TI to standard out
 * 
 * @param id Slot Number
 * @param pflag if pflag>0, print out raw registers
 *
 */

void
vfTDCStatus(int id, int pflag)
{
  struct vfTDC_struct vr;

  if(id==0) id=vfTDCID[0];

  if((id<=0) || (id>21) || (TDCp[id] == NULL)) 
    {
      printf("%s: ERROR : TDC in slot %d is not initialized \n",
	     __FUNCTION__,id);
      return;
    }

  VLOCK;
  vr.boardID        = vmeRead32(&TDCp[id]->boardID);
  vr.ptw            = vmeRead32(&TDCp[id]->ptw);
  vr.intsetup       = vmeRead32(&TDCp[id]->intsetup);
  vr.pl             = vmeRead32(&TDCp[id]->pl);
  vr.adr32          = vmeRead32(&TDCp[id]->adr32);
  vr.blocklevel     = vmeRead32(&TDCp[id]->blocklevel);
  vr.vmeControl     = vmeRead32(&TDCp[id]->vmeControl);
  vr.trigsrc        = vmeRead32(&TDCp[id]->trigsrc);
  vr.sync           = vmeRead32(&TDCp[id]->sync);
  vr.busy           = vmeRead32(&TDCp[id]->busy);
  vr.clock          = (vmeRead32(&TDCp[id]->clock))>>8;
  vr.blockBuffer    = vmeRead32(&TDCp[id]->blockBuffer);
  vr.runningMode    = vmeRead32(&TDCp[id]->runningMode);
  vr.livetime       = vmeRead32(&TDCp[id]->livetime);
  vr.busytime       = vmeRead32(&TDCp[id]->busytime);
  vr.eventNumber_hi = vmeRead32(&TDCp[id]->eventNumber_hi);
  vr.eventNumber_lo = vmeRead32(&TDCp[id]->eventNumber_lo);

  vmeWrite32(&TDCp[id]->reset,VFTDC_RESET_SCALERS_LATCH);

  vr.trig1_scaler   = vmeRead32(&TDCp[id]->trig1_scaler);
  vr.trig2_scaler   = vmeRead32(&TDCp[id]->trig2_scaler);
  vr.sync_scaler    = vmeRead32(&TDCp[id]->sync_scaler);
  vr.berr_scaler    = vmeRead32(&TDCp[id]->berr_scaler);

  vr.status         = vmeRead32(&TDCp[id]->status);

  VUNLOCK;

  printf("\n");
#ifdef VXWORKS
  printf("STATUS for vfTDC at base address 0x%08x \n",
	 (unsigned int) TDCp[id]);
#else
  printf("STATUS for vfTDC at VME (Local) base address 0x%08lx (0x%lx) \n",
	 (unsigned long) TDCp[id] - vfTDCA24Offset, (unsigned long) TDCp[id]);
#endif
  printf("--------------------------------------------------------------------------------\n");

  printf(" Board Firmware Rev.Vers = %d.%d  -- %s\n\n",
	 (vr.status&VFTDC_STATUS_FIRMWARE_VERS_MASK)>>24,
	 (vr.status&VFTDC_STATUS_FIRMWARE_REV_MASK)>>20,
	 (vr.status&VFTDC_STATUS_HI_REZ_MODE)?
	 "High Resolution (96 chan)":"Normal Resolution (192 chan)");

  if(vr.vmeControl&VFTDC_VMECONTROL_A32M) 
    {
      printf(" Alternate VME Addressing: Multiblock Enabled\n");
      if(vr.vmeControl&VFTDC_VMECONTROL_A32)
	printf("   A32 Enabled at VME (Local) base 0x%08x (0x%lx)\n",
	       vr.adr32&VFTDC_ADR32_BASE_MASK,(unsigned long) TDCpd[id]);
      else
	printf("   A32 Disabled\n");
    
      printf("   Multiblock VME Address Range 0x%08x - 0x%08x\n",
	     (vr.adr32&VFTDC_ADR32_MBLK_ADDR_MIN_MASK)<<10,
	     (vr.adr32&VFTDC_ADR32_MBLK_ADDR_MAX_MASK)<<22);
    }
  else
    {
      printf(" Alternate VME Addressing: Multiblock Disabled\n");
      if(vr.vmeControl&VFTDC_VMECONTROL_A32)
	printf("   A32 Enabled at VME (Local) base 0x%08x (0x%lx)\n",
	       vr.adr32&VFTDC_ADR32_BASE_MASK,(unsigned long) TDCpd[id]);
      else
	printf("   A32 Disabled\n");
    }

  if(vr.intsetup&VFTDC_INTSETUP_ENABLE) 
    {
      printf("\n");
      printf("  Interrupts ENABLED:\n");
    
      printf("  VME INT Vector = 0x%x  Level = %d\n",
	     (vr.intsetup&VFTDC_INTSETUP_VECTOR_MASK),
	     (vr.intsetup&VFTDC_INTSETUP_LEVEL_MASK)>>8);
    }

  printf("\n");
  printf(" Signal Sources: \n");

  printf("   Ref Clock : ");
  if((vr.clock&VFTDC_CLOCK_MASK)==VFTDC_CLOCK_INTERNAL) 
    {
      printf("Internal\n");
    }
  else if((vr.clock&VFTDC_CLOCK_MASK)==VFTDC_CLOCK_VXS) 
    {
      printf("VXS\n");
    }
  else if((vr.clock&VFTDC_CLOCK_MASK)==VFTDC_CLOCK_HFBR1) 
    {
      printf("Fiber 1\n");
    }
  else
    {
      printf("Undefined (%d)\n",(vr.clock&VFTDC_CLOCK_MASK));
    }

  printf("   Trig Src  : ");
  if(vr.trigsrc & VFTDC_TRIGSRC_VXS)
    printf("VXS  ");
  if(vr.trigsrc & VFTDC_TRIGSRC_HFBR1)
    printf("Fiber 1  ");
  if(vr.trigsrc & VFTDC_TRIGSRC_FP)
    printf("Front Panel  ");
  if(vr.trigsrc & VFTDC_TRIGSRC_VME)
    printf("VME (Software)");
  printf("\n");
  
  printf("   Sync Reset: ");
  if(vr.sync & VFTDC_SYNC_VXS)
    printf("VXS  ");
  if(vr.sync & VFTDC_SYNC_HFBR1)
    printf("Fiber 1  ");
  if(vr.sync & VFTDC_SYNC_FP)
    printf("Front Panel  ");
  if(vr.sync & VFTDC_SYNC_VME)
    printf("VME (Software)");
  printf("\n\n");


  if(vr.vmeControl&VFTDC_VMECONTROL_MBLK) 
    {
      if(vr.vmeControl&VFTDC_VMECONTROL_FIRST_BOARD)
	printf("   MultiBlock transfer ENABLED (First Board)\n");
      else if(vr.vmeControl&VFTDC_VMECONTROL_LAST_BOARD)
	printf("   MultiBlock transfer ENABLED (Last Board)\n");
      else
	printf("   MultiBlock transfer ENABLED\n");
    }
  else 
    {
      printf("   MultiBlock transfer DISABLED\n");
    }
  printf("\n");

  printf(" Trigger   Scaler         = %d\n",vr.trig1_scaler);
  printf(" Trigger 2 Scaler         = %d\n",vr.trig2_scaler);
  printf(" SyncReset Scaler         = %d\n",vr.sync_scaler);
  printf(" Bus Error Scaler         = %d\n",vr.berr_scaler);

  printf("--------------------------------------------------------------------------------\n");
  printf("\n\n");

}

#ifdef NOTYET
/**
 * @ingroup Status
 * @brief Get the Firmware Version
 *
 * @param id Slot Number
 * @return Firmware Version if successful, ERROR otherwise
 *
 */
int
vfTDCGetFirmwareVersion()
{
  unsigned int rval=0;
  if(id==0) id=vfTDCID[0];

  if((id<=0) || (id>21) || (TDCp[id] == NULL)) 
    {
      printf("%s: ERROR : TDC in slot %d is not initialized \n",
	     __FUNCTION__,id);
      return ERROR;
    }

  VLOCK;
  /* reset the VME_to_JTAG engine logic */
  vmeWrite32(&TDCp->reset,VFTDC_RESET_JTAG);

  /* Reset FPGA JTAG to "reset_idle" state */
  vmeWrite32(&TDCp->JTAGFPGABase[(0x003C)>>2],0);

  /* enable the user_code readback */
  vmeWrite32(&TDCp->JTAGFPGABase[(0x092C)>>2],0x3c8);

  /* shift in 32-bit to FPGA JTAG */
  vmeWrite32(&TDCp->JTAGFPGABase[(0x1F1C)>>2],0);
  
  /* Readback the firmware version */
  rval = vmeRead32(&TDCp->JTAGFPGABase[(0x1F1C)>>2]);
  VUNLOCK;

  return rval;
}


/**
 * @ingroup Status
 * @brief Get the Module Serial Number
 *
 * @param id Slot Number
 * @param rSN  Pointer to string to pass Serial Number
 *
 * @return SerialNumber if successful, ERROR otherwise
 *
 */
unsigned int
vfTDCGetSerialNumber(char **rSN)
{
  unsigned int rval=0;
  char retSN[10];

  memset(retSN,0,sizeof(retSN));
  
  if(id==0) id=vfTDCID[0];

  if((id<=0) || (id>21) || (TDCp[id] == NULL)) 
    {
      printf("%s: ERROR : TDC in slot %d is not initialized \n",
	     __FUNCTION__,id);
      return ERROR;
    }

  VLOCK;
  vmeWrite32(&TDCp->reset,VFTDC_RESET_JTAG);           /* reset */
  vmeWrite32(&TDCp->JTAGPROMBase[(0x3c)>>2],0);     /* Reset_idle */
  vmeWrite32(&TDCp->JTAGPROMBase[(0xf2c)>>2],0xFD); /* load the UserCode Enable */
  vmeWrite32(&TDCp->JTAGPROMBase[(0x1f1c)>>2],0);   /* shift in 32-bit of data */
  rval = vmeRead32(&TDCp->JTAGPROMBase[(0x1f1c)>>2]);
  VUNLOCK;

  if(rSN!=NULL)
    {
      sprintf(retSN,"TI-%d",rval&0x7ff);
      strcpy((char *)rSN,retSN);
    }


  printf("%s: TI Serial Number is %s (0x%08x)\n", 
	 __FUNCTION__,retSN,rval);

  return rval;
  

}
#endif /* NOTYET */

/**
 * @ingroup Config
 * @brief Perform a soft reset of the TI
 *
 * @param id Slot Number
 * @return OK if successful, ERROR otherwise
 *
 */
int
vfTDCReset(int id)
{
  if(id==0) id=vfTDCID[0];

  if((id<=0) || (id>21) || (TDCp[id] == NULL)) 
    {
      printf("%s: ERROR : TDC in slot %d is not initialized \n",
	     __FUNCTION__,id);
      return ERROR;
    }
  
  VLOCK;
  vmeWrite32(&TDCp[id]->reset,VFTDC_RESET_SOFT);
  VUNLOCK;
  return OK;
}

/**
 * @ingroup Config
 * @brief Set the number of events per block
 * @param id Slot Number
 * @param blockLevel Number of events per block
 * @return OK if successful, ERROR otherwise
 *
 */
int
vfTDCSetBlockLevel(int id, int blockLevel)
{
  if(id==0) id=vfTDCID[0];

  if((id<=0) || (id>21) || (TDCp[id] == NULL)) 
    {
      printf("%s: ERROR : TDC in slot %d is not initialized \n",
	     __FUNCTION__,id);
      return ERROR;
    }

  if((blockLevel<1) || (blockLevel>255))
    {
      printf("%s: ERROR: Invalid blockLevel (%d)\n",
	     __FUNCTION__,blockLevel);
      return ERROR;
    }

  VLOCK;
  vmeWrite32(&TDCp[id]->blocklevel,blockLevel);
  VUNLOCK;
  return OK;
}

/**
 * @ingroup Config
 * @brief Set trigger sources with specified trigmask
 *
 * @param id Slot Number
 * @param trigmask bits:  
 *         - 0: VXS
 *         - 1: HFBR#1
 *         - 4: Software
 *
 * @return OK if successful, ERROR otherwise
 *
 */
int
vfTDCSetTriggerSource(int id, unsigned int trigmask)
{
  unsigned int supported = 
    VFTDC_TRIGSRC_VXS |
    VFTDC_TRIGSRC_HFBR1 | 
    VFTDC_TRIGSRC_VME;

  if(id==0) id=vfTDCID[0];

  if((id<=0) || (id>21) || (TDCp[id] == NULL)) 
    {
      printf("%s: ERROR : TDC in slot %d is not initialized \n",
	     __FUNCTION__,id);
      return ERROR;
    }

  if(trigmask & ~supported)
    {
      printf("%s: ERROR: Unsupported Trigger source mask (0x%x).\n",
	     __FUNCTION__,trigmask);
      return ERROR;
    }

  VLOCK;
  vmeWrite32(&TDCp[id]->trigsrc, trigmask);
  VUNLOCK;

  return OK;
}

#ifdef NOTYET
/**
 * @ingroup Config
 * @brief Enable trigger sources
 * Enable trigger sources set by 
 *                          tiSetTriggerSource(...) or
 *                          tiSetTriggerSourceMask(...)
 * @param id Slot Number
 * @sa tiSetTriggerSource
 * @sa tiSetTriggerSourceMask
 *
 * @return OK if successful, ERROR otherwise
 *
 */
int
vfTDCEnableTriggerSource()
{
  if(id==0) id=vfTDCID[0];

  if((id<=0) || (id>21) || (TDCp[id] == NULL)) 
    {
      printf("%s: ERROR : TDC in slot %d is not initialized \n",
	     __FUNCTION__,id);
      return ERROR;
    }

  if(tiTriggerSource==0)
    {
      printf("%s: WARN: No Trigger Sources Enabled\n",__FUNCTION__);
    }

  VLOCK;
  vmeWrite32(&TDCp->trigsrc, tiTriggerSource);
  VUNLOCK;

  return OK;

}

/**
 * @ingroup Config
 * @brief Disable trigger sources
 *    
 * @param id Slot Number
 * @param fflag 
 *   -  0: Disable Triggers
 *   - >0: Disable Triggers and generate enough triggers to fill the current block
 *
 * @return OK if successful, ERROR otherwise
 *
 */
int
vfTDCDisableTriggerSource(int fflag)
{
  int regset=0;
  if(id==0) id=vfTDCID[0];

  if((id<=0) || (id>21) || (TDCp[id] == NULL)) 
    {
      printf("%s: ERROR : TDC in slot %d is not initialized \n",
	     __FUNCTION__,id);
      return ERROR;
    }

  VLOCK;

  if(tiMaster)
    regset = VFTDC_TRIGSRC_LOOPBACK;

  vmeWrite32(&TDCp->trigsrc,regset);

  VUNLOCK;
  if(fflag && tiMaster)
    {
      tiFillToEndBlock();      
    }

  return OK;

}
#endif /* NOTYET */

/**
 * @ingroup Config
 * @brief Set the Sync source mask
 *
 * @param id Slot Number
 * @param sync - MASK indicating the sync source
 *       bit: description
 *       -  0: VXS
 *       -  1: HFBR1
 *       -  4: Software
 *
 * @return OK if successful, ERROR otherwise
 *
 */
int
vfTDCSetSyncSource(int id, unsigned int sync)
{
  unsigned int supported = 
    VFTDC_SYNC_VXS |
    VFTDC_SYNC_HFBR1 | 
    VFTDC_SYNC_VME;

  if(id==0) id=vfTDCID[0];

  if((id<=0) || (id>21) || (TDCp[id] == NULL)) 
    {
      printf("%s: ERROR : TDC in slot %d is not initialized \n",
	     __FUNCTION__,id);
      return ERROR;
    }

  if(sync & ~supported)
    {
      printf("%s: ERROR: Invalid Sync Source Mask (%d).\n",
	     __FUNCTION__,sync);
      return ERROR;
    }

  VLOCK;
  vmeWrite32(&TDCp[id]->sync,sync);
  VUNLOCK;

  return OK;
}

/**
 * @ingroup Config
 * @brief Perform a "software" trigger
 *
 * @param id Slot Number
 *
 * @return OK if successful, ERROR otherwise
 *
 */
int
vfTDCSoftTrig(int id)
{
  if(id==0) id=vfTDCID[0];

  if((id<=0) || (id>21) || (TDCp[id] == NULL)) 
    {
      printf("%s: ERROR : TDC in slot %d is not initialized \n",
	     __FUNCTION__,id);
      return ERROR;
    }

  VLOCK;
  vmeWrite32(&TDCp[id]->reset, VFTDC_RESET_TRIGGER);
  VUNLOCK;

  return OK;

}

/**
 * @ingroup Config
 * @brief Set the trigger window paramters
 *
 * @param id Slot Number
 * @param latency Look back since readout trigger (steps of 4ns)
 * @param width   width of the TDC readout window (steps of 4ns)
 *
 * @return OK if successful, ERROR otherwise
 *
 */
int
vfTDCSetWindowParamters(int id, int latency, int width)
{
  if(id==0) id=vfTDCID[0];

  if((id<=0) || (id>21) || (TDCp[id] == NULL)) 
    {
      printf("%s: ERROR : TDC in slot %d is not initialized \n",
	     __FUNCTION__,id);
      return ERROR;
    }

  if((latency<0) || (latency>VFTDC_PL_MASK))
    {
      printf("%s: ERROR: Invalid latency (%d)\n",
	     __FUNCTION__,latency);
      return ERROR;
    }

  if((width<0)||(width>VFTDC_PTW_MASK))
    {
      printf("%s: ERROR: Invalid width (%d)\n",
	     __FUNCTION__,width);
      return ERROR;
    }

  VLOCK;
  vmeWrite32(&TDCp[id]->pl,  latency);
  vmeWrite32(&TDCp[id]->ptw, width);
  VUNLOCK;

  return OK;
}

const char *vfTDC_blockerror_names[VFTDC_BLOCKERROR_NTYPES] =
  {
    "No Error",
    "Termination on word count",
    "Unknown Bus Error",
    "Zero Word Count",
    "DmaDone(..) Error"
  };

/**
 *  @ingroup Readout
 *  @brief Return the block error flag and optionally print out the description to standard out
 *  @param pflag If >0 will print the error flag to standard out.
 *  @return Block Error flag.
 *  @sa VFTDC_BLOCKERROR_FLAGS
 */
int
vfTDCReadBlockStatus(int pflag)
{
  if(pflag)
    {
      if(vfTDCBlockError!=VFTDC_BLOCKERROR_NO_ERROR)
	{
	  printf("\n%s: ERROR: %s\n",
		 __FUNCTION__,vfTDC_blockerror_names[vfTDCBlockError]);
	}
    }

  return vfTDCBlockError;
}


/**
 *  @ingroup Readout
 *  @brief General Data readout routine
 *
 *  @param  id     Slot number of module to read
 *  @param  data   local memory address to place data
 *  @param  nwrds  Max number of words to transfer
 *  @param  rflag  Readout Flag
 * <pre>
 *              0 - programmed I/O from the specified board
 *              1 - DMA transfer using Universe/Tempe DMA Engine 
 *                    (DMA VME transfer Mode must be setup prior)
 *              2 - Multiblock DMA transfer (Multiblock must be enabled
 *                     and daisychain in place or SD being used)
 * </pre>
 *  @return Number of words inserted into data if successful.  Otherwise ERROR.
 */
int
vfTDCReadBlock(int id, volatile UINT32 *data, int nwrds, int rflag)
{
  int ii, blknum;
  int stat, retVal, xferCount, rmode;
  int dCnt, berr=0;
  int dummy=0;
  volatile unsigned int *laddr;
  unsigned int bhead, ehead, val;
  unsigned int vmeAdr, csr;

  if(id==0) id=vfTDCID[0];

  if((id<=0) || (id>21) || (TDCp[id] == NULL)) 
    {
      logMsg("\nvfTDCReadBlock: ERROR : VFTDC in slot %d is not initialized\n\n",id,0,0,0,0,0);
      return(ERROR);
    }

  if(data==NULL) 
    {
      logMsg("\nvfTDCReadBlock: ERROR: Invalid Destination address\n\n",0,0,0,0,0,0);
      return(ERROR);
    }

  vfTDCBlockError=VFTDC_BLOCKERROR_NO_ERROR;
  if(nwrds <= 0) nwrds= (VFTDC_MAX_TDC_CHANNELS*VFTDC_MAX_DATA_PER_CHANNEL) + 8;
  rmode = rflag&0x0f;
  
  if(rmode >= 1) 
    { /* Block Transfers */
    
      /*Assume that the DMA programming is already setup. */
      /* Don't Bother checking if there is valid data - that should be done prior
	 to calling the read routine */

      /* Check for 8 byte boundary for address - insert dummy word (Slot 0 VFTDC Dummy DATA)*/
      if((unsigned long) (data)&0x7) 
	{
#ifdef VXWORKS
	  *data = VFTDC_DUMMY_DATA;
#else
	  *data = LSWAP(VFTDC_DUMMY_DATA);
#endif
	  dummy = 1;
	  laddr = (data + 1);
	} 
      else 
	{
	  dummy = 0;
	  laddr = data;
	}

      VLOCK;
      if(rmode == 2) 
	{ /* Multiblock Mode */
#ifdef NOTSUPPORTED
	  if((vmeRead32(&TDCp[id]->vmeControl)&VFTDC_VMECONTROL_FIRST_BOARD)==0) 
	    {
	      logMsg("\nvfTDCReadBlock: ERROR: VFTDC in slot %d is not First Board\n\n",id,0,0,0,0,0);
	      VUNLOCK
	      return(ERROR);
	    }
	  vmeAdr = (unsigned int)((unsigned long)(VFTDCpmb) - vfTDCA32Offset);
#else
	  logMsg("\nvfTDCReadBlock: ERROR: Multiblock readout not yet supported\n\n",1,2,3,4,5,6);
	  VUNLOCK;
	  return ERROR;
#endif
	}
      else
	{
	  vmeAdr = (unsigned int)((unsigned long)TDCpd[id] - vfTDCA32Offset);
	}
#ifdef VXWORKS
      retVal = sysVmeDmaSend((UINT32)laddr, vmeAdr, (nwrds<<2), 0);
#else
      retVal = vmeDmaSend((unsigned long)laddr, vmeAdr, (nwrds<<2));
#endif
      if(retVal != 0) 
	{
	  logMsg("\nvfTDCReadBlock: ERROR in DMA transfer Initialization 0x%x\n\n",retVal,0,0,0,0,0);
	  VUNLOCK
	  return(retVal);
	}

      /* Wait until Done or Error */
#ifdef VXWORKS
      retVal = sysVmeDmaDone(10000,1);
#else
      retVal = vmeDmaDone();
#endif

      if(retVal > 0) 
	{
	  /* Check to see that Bus error was generated by VFTDC */
	  if(rmode == 2) 
	    {
#ifdef NOTSUPPORTED
	      csr = vmeRead32(&TDCp[vfTDCMaxSlot]->status);  /* from Last VFTDC */
#endif
	    }
	  else
	    {
	      csr = vmeRead32(&TDCp[id]->status);
	    }
	  stat = (csr)&VFTDC_STATUS_BERR;

	  if((retVal>0) && (stat)) 
	    {
#ifdef VXWORKS
	      xferCount = (nwrds - (retVal>>2) + dummy);  /* Number of Longwords transfered */
#else
	      xferCount = ((retVal>>2) + dummy);  /* Number of Longwords transfered */
#endif
	      VUNLOCK
	      return(xferCount); /* Return number of data words transfered */
	    }
	  else
	    {
#ifdef VXWORKS
	      xferCount = (nwrds - (retVal>>2) + dummy);  /* Number of Longwords transfered */
#else
	      xferCount = ((retVal>>2) + dummy);  /* Number of Longwords transfered */
#endif
	      logMsg("vfTDCReadBlock: DMA transfer terminated by unknown BUS Error (csr=0x%x xferCount=%d id=%d)\n",
		     csr,xferCount,id,0,0,0);
	      VUNLOCK
	      vfTDCBlockError=VFTDC_BLOCKERROR_UNKNOWN_BUS_ERROR;
	      return(xferCount);
	    }
	} 
      else if (retVal == 0)
	{ /* Block Error finished without Bus Error */
#ifdef VXWORKS
	  logMsg("vfTDCReadBlock: WARN: DMA transfer terminated by word count 0x%x\n",nwrds,0,0,0,0,0);
	  vfTDCBlockError=VFTDC_BLOCKERROR_TERM_ON_WORDCOUNT;
#else
	  logMsg("vfTDCReadBlock: WARN: DMA transfer returned zero word count 0x%x\n",nwrds,0,0,0,0,0);
	  vfTDCBlockError=VFTDC_BLOCKERROR_ZERO_WORD_COUNT;
#endif
	  VUNLOCK
	  return(nwrds);
	} 
      else 
	{  /* Error in DMA */
#ifdef VXWORKS
	  logMsg("\nvfTDCReadBlock: ERROR: sysVmeDmaDone returned an Error\n\n",0,0,0,0,0,0);
#else
	  logMsg("\nvfTDCReadBlock: ERROR: vmeDmaDone returned an Error\n\n",0,0,0,0,0,0);
#endif
	  VUNLOCK
	  vfTDCBlockError=VFTDC_BLOCKERROR_DMADONE_ERROR;
	  return(retVal>>2);
	}

    } 
  else 
    {  /*Programmed IO */

      /* Check if Bus Errors are enabled. If so then disable for Prog I/O reading */
      VLOCK;
      berr = vmeRead32(&TDCp[id]->vmeControl)&VFTDC_VMECONTROL_BERR;
      if(berr)
	vmeWrite32(&TDCp[id]->vmeControl, 
		   vmeRead32(&TDCp[id]->vmeControl) & ~VFTDC_VMECONTROL_BERR);

      dCnt = 0;
      /* Read Block Header - should be first word */
      bhead = (unsigned int) *TDCpd[id]; 
#ifndef VXWORKS
      bhead = LSWAP(bhead);
#endif
      if((bhead&VFTDC_DATA_TYPE_DEFINE)&&((bhead&VFTDC_DATA_TYPE_MASK) == VFTDC_DATA_BLOCK_HEADER)) 
	{
	  blknum = bhead&VFTDC_DATA_BLKNUM_MASK;
	  ehead = (unsigned int) *TDCpd[id];
#ifndef VXWORKS
	  ehead = LSWAP(ehead);
#endif
#ifdef VXWORKS
	  data[dCnt] = bhead;
#else
	  data[dCnt] = LSWAP(bhead); /* Swap back to little-endian */
#endif
	  dCnt++;
#ifdef VXWORKS
	  data[dCnt] = ehead;
#else
	  data[dCnt] = LSWAP(ehead); /* Swap back to little-endian */
#endif
	  dCnt++;
	}
      else
	{
	  /* We got bad data - Check if there is any data at all */
	  if( ((vmeRead32(&TDCp[id]->blockBuffer) & 
		VFTDC_BLOCKBUFFER_BLOCKS_READY_MASK)>>16) == 0) 
	    {
	      logMsg("vfTDCReadBlock: FIFO Empty (0x%08x)\n",bhead,0,0,0,0,0);
	      VUNLOCK
	      return(0);
	    } 
	  else 
	    {
	      logMsg("\nvfTDCReadBlock: ERROR: Invalid Header Word 0x%08x\n\n",bhead,0,0,0,0,0);
	      VUNLOCK
	      return(ERROR);
	    }
	}

      ii=0;
      while(ii<nwrds) 
	{
	  val = (unsigned int) *TDCpd[id];
	  data[ii+2] = val;
#ifndef VXWORKS
	  val = LSWAP(val);
#endif
	  if( (val&VFTDC_DATA_TYPE_DEFINE) 
	      && ((val&VFTDC_DATA_TYPE_MASK) == VFTDC_DATA_BLOCK_TRAILER) )
	    break;
	  ii++;
	}
      ii++;
      dCnt += ii;


      if(berr)
	vmeWrite32(&TDCp[id]->vmeControl, 
		   vmeRead32(&TDCp[id]->vmeControl) | VFTDC_VMECONTROL_BERR);

      VUNLOCK
      return(dCnt);
    }

  VUNLOCK
  return(OK);
}

#ifdef NOTYET
/**
 * @ingroup Config
 * @brief Enable Fiber transceiver
 *
 *  Note:  All Fiber are enabled by default 
 *         (no harm, except for 1-2W power usage)
 *
 * @sa tiDisableFiber
 * @param id Slot Number
 * @param   fiber: integer indicative of the transceiver to enable
 *
 *
 * @return OK if successful, ERROR otherwise.
 *
 */
int
vfTDCEnableFiber(int id, unsigned int fiber)
{
  unsigned int sval;
  unsigned int fiberbit;

  if(id==0) id=vfTDCID[0];

  if((id<=0) || (id>21) || (TDCp[id] == NULL)) 
    {
      printf("%s: ERROR : TDC in slot %d is not initialized \n",
	     __FUNCTION__,id);
      return ERROR;
    }

  if((fiber<1) | (fiber>8))
    {
      printf("%s: ERROR: Invalid value for fiber (%d)\n",
	     __FUNCTION__,fiber);
      return ERROR;
    }

  fiberbit = (1<<(fiber-1));

  VLOCK;
  sval = vmeRead32(&TDCp->fiber);
  vmeWrite32(&TDCp->fiber,
	     sval | fiberbit );
  VUNLOCK;

  return OK;
  
}

/**
 * @ingroup Config
 * @brief Disnable Fiber transceiver
 *
 * @sa tiEnableFiber
 *
 * @param id Slot Number
 * @param   fiber: integer indicative of the transceiver to disable
 *
 *
 * @return OK if successful, ERROR otherwise.
 *
 */
int
vfTDCDisableFiber(int id, unsigned int fiber)
{
  unsigned int rval;
  unsigned int fiberbit;

  if(id==0) id=vfTDCID[0];

  if((id<=0) || (id>21) || (TDCp[id] == NULL)) 
    {
      printf("%s: ERROR : TDC in slot %d is not initialized \n",
	     __FUNCTION__,id);
      return ERROR;
    }

  if((fiber<1) | (fiber>8))
    {
      printf("%s: ERROR: Invalid value for fiber (%d)\n",
	     __FUNCTION__,fiber);
      return ERROR;
    }

  fiberbit = (1<<(fiber-1));

  VLOCK;
  rval = vmeRead32(&TDCp->fiber);
  vmeWrite32(&TDCp->fiber,
	   rval & ~fiberbit );
  VUNLOCK;

  return rval;
  
}
#endif /* NOTYET */


/**
 * @ingroup Config
 * @brief Enable Bus Errors to terminate Block Reads
 * @param id Slot Number
 * @sa tiDisableBusError
 * @return OK if successful, otherwise ERROR
 */
int
vfTDCEnableBusError(int id)
{

  if(id==0) id=vfTDCID[0];

  if((id<=0) || (id>21) || (TDCp[id] == NULL)) 
    {
      printf("%s: ERROR : TDC in slot %d is not initialized \n",
	     __FUNCTION__,id);
      return ERROR;
    }

  VLOCK;
  vmeWrite32(&TDCp[id]->vmeControl,
	   vmeRead32(&TDCp[id]->vmeControl) | (VFTDC_VMECONTROL_BERR) );
  VUNLOCK;
  return OK;
}

/**
 * @ingroup Config
 * @brief Disable Bus Errors to terminate Block Reads
 * @param id Slot Number
 * @sa tiEnableBusError
 * @return OK if successful, otherwise ERROR
 */
int
vfTDCDisableBusError(int id)
{

  if(id==0) id=vfTDCID[0];

  if((id<=0) || (id>21) || (TDCp[id] == NULL)) 
    {
      printf("%s: ERROR : TDC in slot %d is not initialized \n",
	     __FUNCTION__,id);
      return ERROR;
    }

  VLOCK;
  vmeWrite32(&TDCp[id]->vmeControl,
	   vmeRead32(&TDCp[id]->vmeControl) & ~(VFTDC_VMECONTROL_BERR) );
  VUNLOCK;
  return OK;
}


/**
 * @ingroup Config
 * @brief Generate a software SyncReset signal.
 *
 *  @param id Slot Number
 *
 * @return OK if successful, otherwise ERROR
 */
int
vfTDCSyncReset(int id)
{
  if(id==0) id=vfTDCID[0];

  if((id<=0) || (id>21) || (TDCp[id] == NULL)) 
    {
      printf("%s: ERROR : TDC in slot %d is not initialized \n",
	     __FUNCTION__,id);
      return ERROR;
    }
  
  VLOCK;
  vmeWrite32(&TDCp[id]->reset, VFTDC_RESET_SYNCRESET); 
  taskDelay(1);
  VUNLOCK;
  
  return OK;
}

/**
 * @ingroup Config
 * @brief Routine to set the A32 Base
 *
 * @param id Slot Number
 * @return OK if successful, otherwise ERROR
 */
int
vfTDCSetAdr32(int id, unsigned int a32base)
{
  unsigned long laddr=0;
  int res=0,a32Enabled=0;

  if(id==0) id=vfTDCID[0];

  if((id<=0) || (id>21) || (TDCp[id] == NULL)) 
    {
      printf("%s: ERROR : TDC in slot %d is not initialized \n",
	     __FUNCTION__,id);
      return ERROR;
    }

  if(a32base<0x00800000)
    {
      printf("%s: ERROR: a32base out of range (0x%08x)\n",
	     __FUNCTION__,a32base);
      return ERROR;
    }

  VLOCK;
  vmeWrite32(&TDCp[id]->adr32, 
	     (a32base & VFTDC_ADR32_BASE_MASK) );

  vmeWrite32(&TDCp[id]->vmeControl, 
	     vmeRead32(&TDCp[id]->vmeControl) | VFTDC_VMECONTROL_A32);

  a32Enabled = vmeRead32(&TDCp[id]->vmeControl)&(VFTDC_VMECONTROL_A32);
  if(!a32Enabled)
    {
      printf("%s: ERROR: Failed to enable A32 Address\n",__FUNCTION__);
      VUNLOCK;
      return ERROR;
    }

#ifdef VXWORKS
  res = sysBusToLocalAdrs(0x09,(char *)a32base,(char **)&laddr);
  if (res != 0) 
    {
      printf("%s: ERROR in sysBusToLocalAdrs(0x09,0x%x,&laddr) \n",
	     __FUNCTION__,a32base);
      VUNLOCK;
      return(ERROR);
    }
#else
  res = vmeBusToLocalAdrs(0x09,(char *)(unsigned long)a32base,(char **)&laddr);
  if (res != 0) 
    {
      printf("%s: ERROR in vmeBusToLocalAdrs(0x09,0x%x,&laddr) \n",
	     __FUNCTION__,a32base);
      VUNLOCK;
      return(ERROR);
    }
#endif

  vfTDCA32Base = a32base;
  vfTDCA32Offset = laddr - vfTDCA32Base;
  TDCpd[id] = (unsigned int *)(laddr);  /* Set a pointer to the FIFO */
  VUNLOCK;

  printf("%s: A32 Base address set to 0x%08x\n",
	 __FUNCTION__,vfTDCA32Base);

  return OK;
}

/**
 * @ingroup Config
 * @brief Disable A32
 *
 * @param id Slot Number
 * @return OK if successful, otherwise ERROR
 */
int
vfTDCDisableA32(int id)
{
  if(id==0) id=vfTDCID[0];

  if((id<=0) || (id>21) || (TDCp[id] == NULL)) 
    {
      printf("%s: ERROR : TDC in slot %d is not initialized \n",
	     __FUNCTION__,id);
      return ERROR;
    }
  
  VLOCK;
  vmeWrite32(&TDCp[id]->adr32,0x0);
  vmeWrite32(&TDCp[id]->vmeControl, 
	     vmeRead32(&TDCp[id]->vmeControl) & ~VFTDC_VMECONTROL_A32);
  VUNLOCK;

  return OK;
}

/**
 * @ingroup Config
 * @brief Reset the L1A counter, as incremented by the TI.
 *
 * @param id Slot Number
 * @return OK if successful, otherwise ERROR
 */
int
vfTDCResetEventCounter(int id)
{
  if(id==0) id=vfTDCID[0];

  if((id<=0) || (id>21) || (TDCp[id] == NULL)) 
    {
      printf("%s: ERROR : TDC in slot %d is not initialized \n",
	     __FUNCTION__,id);
      return ERROR;
    }
  
  VLOCK;
  vmeWrite32(&TDCp[id]->reset, VFTDC_RESET_SCALERS_RESET);
  VUNLOCK;

  return OK;
}

/**
 * @ingroup Status
 * @brief Returns the event counter (48 bit)
 *
 * @param id Slot Number
 * @return Number of accepted events if successful, otherwise ERROR
 */
unsigned long long int
vfTDCGetEventCounter(int id)
{
  unsigned long long int rval=0;
  unsigned int lo=0, hi=0;

  if(id==0) id=vfTDCID[0];

  if((id<=0) || (id>21) || (TDCp[id] == NULL)) 
    {
      printf("%s: ERROR : TDC in slot %d is not initialized \n",
	     __FUNCTION__,id);
      return ERROR;
    }

  VLOCK;
  lo = vmeRead32(&TDCp[id]->eventNumber_lo);
  hi = (vmeRead32(&TDCp[id]->eventNumber_hi) & VFTDC_EVENTNUMBER_HI_MASK)>>16;

  rval = lo | ((unsigned long long)hi<<32);
  VUNLOCK;
  
  return rval;
}


/**
 * @ingroup Readout
 * @brief Returns the number of Blocks available for readout
 *
 * @param id Slot Number
 * @return Number of blocks available for readout if successful, otherwise ERROR
 *
 */
unsigned int
vfTDCBReady(int id)
{
  unsigned int blockBuffer=0, rval=0;

  if(id==0) id=vfTDCID[0];

  if((id<=0) || (id>21) || (TDCp[id] == NULL)) 
    {
      printf("%s: ERROR : TDC in slot %d is not initialized \n",
	     __FUNCTION__,id);
      return ERROR;
    }

  VLOCK;
  blockBuffer = vmeRead32(&TDCp[id]->blockBuffer);
  rval        = (blockBuffer&VFTDC_BLOCKBUFFER_BLOCKS_READY_MASK)>>8;
  VUNLOCK;

  return rval;
}

/**
 * @ingroup Config
 * @brief Set the clock to the specified source.
 *
 * @param id Slot Number
 * @param   source
 *         -   0:  HFBR1
 *         -   2:  Internal clock
 *         -   3:  VXS
 *
 * @return OK if successful, otherwise ERROR
 */
int
vfTDCSetClockSource(int id, unsigned int source)
{
  int rval=OK;
  unsigned int clkread=0, clkset=0;
  char sClock[20] = "";

  if(id==0) id=vfTDCID[0];

  if((id<=0) || (id>21) || (TDCp[id] == NULL)) 
    {
      printf("%s: ERROR : TDC in slot %d is not initialized \n",
	     __FUNCTION__,id);
      return ERROR;
    }

  switch(source)
    {
    case VFTDC_CLOCK_INTERNAL:
      sprintf(sClock,"ONBOARD (%d)",source);
      break;
    case VFTDC_CLOCK_HFBR1:
      sprintf(sClock,"EXTERNAL-HFBR1 (%d)",source);
      break;
    case VFTDC_CLOCK_VXS:
      sprintf(sClock,"EXTERNAL-VXS (%d)",source);
      break;
    default:
      printf("%s: ERROR: Invalid Clock Souce (%d)\n",__FUNCTION__,source);
      return ERROR;      
    }

  printf("%s: Setting clock source to %s\n",__FUNCTION__,sClock);


  VLOCK;
  vmeWrite32(&TDCp[id]->clock, source);
  taskDelay(1);

    // Resets
  vmeWrite32(&TDCp[id]->reset,VFTDC_RESET_CLK250);
  taskDelay(1);
  vmeWrite32(&TDCp[id]->reset,VFTDC_RESET_IODELAY);
  taskDelay(1);

  if(source==1) /* Turn on running mode for External Clock verification */
    {
      vmeWrite32(&TDCp[id]->runningMode,VFTDC_RUNNINGMODE_ENABLE);
      taskDelay(1);
      clkread = vmeRead32(&TDCp[id]->clock) & VFTDC_CLOCK_MASK;
      if(clkread != clkset)
	{
	  printf("%s: ERROR Setting Clock Source (clkset = 0x%x, clkread = 0x%x)\n",
		 __FUNCTION__,source, clkread);
	  rval = ERROR;
	}
      vmeWrite32(&TDCp[id]->runningMode,VFTDC_RUNNINGMODE_DISABLE);
    }
  VUNLOCK;

  return rval;
}

/**
 * @ingroup Status
 * @brief Get the current clock source
 * @param id Slot Number
 * @return Current Clock Source
 */
int
vfTDCGetClockSource(int id)
{
  int rval=0;
  if(id==0) id=vfTDCID[0];

  if((id<=0) || (id>21) || (TDCp[id] == NULL)) 
    {
      printf("%s: ERROR : TDC in slot %d is not initialized \n",
	     __FUNCTION__,id);
      return ERROR;
    }

  VLOCK;
  rval = vmeRead32(&TDCp[id]->clock) & VFTDC_CLOCK_MASK;
  VUNLOCK;

  return rval;
}

/**
 * @ingroup Status
 * @brief Return geographic address as provided from a VME-64X crate.
 * @param id Slot Number
 * @return Geographic Address if successful, otherwise ERROR.
 */

int
vfTDCGetGeoAddress(int id)
{
  int rval=0;
  if(id==0) id=vfTDCID[0];

  if((id<=0) || (id>21) || (TDCp[id] == NULL)) 
    {
      printf("%s: ERROR : TDC in slot %d is not initialized \n",
	     __FUNCTION__,id);
      return ERROR;
    }

  VLOCK;
  rval = (vmeRead32(&TDCp[id]->boardID) & VFTDC_BOARDID_GEOADR_MASK)>>8;
  VUNLOCK;

  return rval;
}

#ifdef NOTYET
/*************************************************************
 Library Interrupt/Polling routines
*************************************************************/

/*******************************************************************************
 *
 *  vfTDCInt
 *  - Default interrupt handler
 *    Handles the TI interrupt.  Calls a user defined routine,
 *    if it was connected with vfTDCIntConnect()
 *    
 */
static void
vfTDCInt(void)
{
  vfTDCIntCount++;

  INTLOCK;

  if (vfTDCIntRoutine != NULL)	/* call user routine */
    (*vfTDCIntRoutine) (vfTDCIntArg);

  /* Acknowledge trigger */
  if(tiDoAck==1)
    {
      vfTDCIntAck();
    }
  INTUNLOCK;

}

/*******************************************************************************
 *
 *  tiPoll
 *  - Default Polling Server Thread
 *    Handles the polling of latched triggers.  Calls a user
 *    defined routine if was connected with vfTDCIntConnect.
 *
 */
#ifndef VXWORKS
static void
vfTDCPoll(void)
{
  int tdc_data;
  int policy=0;
  struct sched_param sp;
/* #define DO_CPUAFFINITY */
#ifdef DO_CPUAFFINITY
  int j;
  cpu_set_t testCPU;

  if (pthread_getaffinity_np(pthread_self(), sizeof(testCPU), &testCPU) <0) 
    {
      perror("pthread_getaffinity_np");
    }
  printf("%s: CPUset = ",__FUNCTION__);
  for (j = 0; j < CPU_SETSIZE; j++)
    if (CPU_ISSET(j, &testCPU))
      printf(" %d", j);
  printf("\n");

  CPU_ZERO(&testCPU);
  CPU_SET(7,&testCPU);
  if (pthread_setaffinity_np(pthread_self(),sizeof(testCPU), &testCPU) <0) 
    {
      perror("pthread_setaffinity_np");
    }
  if (pthread_getaffinity_np(pthread_self(), sizeof(testCPU), &testCPU) <0) 
    {
      perror("pthread_getaffinity_np");
    }

  printf("%s: CPUset = ",__FUNCTION__);
  for (j = 0; j < CPU_SETSIZE; j++)
    if (CPU_ISSET(j, &testCPU))
      printf(" %d", j);

  printf("\n");


#endif
  
  /* Set scheduler and priority for this thread */
  policy=SCHED_FIFO;
  sp.sched_priority=40;
  printf("%s: Entering polling loop...\n",__FUNCTION__);
  pthread_setschedparam(pthread_self(),policy,&sp);
  pthread_getschedparam(pthread_self(),&policy,&sp);
  printf ("%s: INFO: Running at %s/%d\n",__FUNCTION__,
	  (policy == SCHED_FIFO ? "FIFO"
	   : (policy == SCHED_RR ? "RR"
	      : (policy == SCHED_OTHER ? "OTHER"
		 : "unknown"))), sp.sched_priority);  
  prctl(PR_SET_NAME,"tiPoll");

  while(1) 
    {

      pthread_testcancel();

      /* If still need Ack, don't test the Trigger Status */
      if(tiNeedAck>0) 
	{
	  continue;
	}

      tdc_data = 0;
	  
      tdc_data = tiBReady();
      if(tdc_data == ERROR) 
	{
	  printf("%s: ERROR: vfTDCIntPoll returned ERROR.\n",__FUNCTION__);
	  break;
	}

      if(tdc_data && vfTDCIntRunning)
	{
	  INTLOCK; 
	  tiDaqCount = tdc_data;
	  vfTDCIntCount++;

	  if (vfTDCIntRoutine != NULL)	/* call user routine */
	    (*vfTDCIntRoutine) (vfTDCIntArg);
	
	  /* Write to TI to Acknowledge Interrupt */	  
	  if(tiDoAck==1) 
	    {
	      vfTDCIntAck();
	    }
	  INTUNLOCK;
	}
    
    }
  printf("%s: Read ERROR: Exiting Thread\n",__FUNCTION__);
  pthread_exit(0);

}
#endif 


/*******************************************************************************
 *
 *  vfTDCStartPollingThread
 *  - Routine that launches tiPoll in its own thread 
 *
 */
#ifndef VXWORKS
static void
vfTDCStartPollingThread(void)
{
  int pvfTDC_status;

  pvfTDC_status = 
    pthread_create(&vfTDCpollthread,
		   NULL,
		   (void*(*)(void *)) vfTDCPoll,
		   (void *)NULL);
  if(pvfTDC_status!=0) 
    {						
      printf("%s: ERROR: vfTDC Polling Thread could not be started.\n",
	     __FUNCTION__);	
      printf("\t pthread_create returned: %d\n",pvfTDC_status);
    }

}
#endif

/**
 * @ingroup IntPoll
 * @brief Connect a user routine to the TI Interrupt or
 *    latched trigger, if polling.
 *
 * @param vector VME Interrupt Vector
 * @param routine Routine to call if block is available
 * @param arg argument to pass to routine
 *
 * @return OK if successful, otherwise ERROR
 */
int
vfTDCIntConnect(unsigned int vector, VOIDFUNCPTR routine, unsigned int arg)
{
#ifndef VXWORKS
  int status;
#endif

  if(id==0) id=vfTDCID[0];

  if((id<=0) || (id>21) || (TDCp[id] == NULL)) 
    {
      printf("%s: ERROR : TDC in slot %d is not initialized \n",
	     __FUNCTION__,id);
      return ERROR;
    }


#ifdef VXWORKS
  /* Disconnect any current interrupts */
  if((intDisconnect(vfTDCIntVec) !=0))
    printf("%s: Error disconnecting Interrupt\n",__FUNCTION__);
#endif

  vfTDCIntCount = 0;
  vfTDCAckCount = 0;
  vfTDCDoAck = 1;

  /* Set Vector and Level */
  if((vector < 0xFF)&&(vector > 0x40)) 
    {
      vfTDCIntVec = vector;
    }
  else
    {
      vfTDCIntVec = VFTDC_INT_VEC;
    }

  VLOCK;
  vmeWrite32(&TDCp->intsetup, (vfTDCIntLevel<<8) | vfTDCIntVec );
  VUNLOCK;

  switch (tiReadoutMode)
    {
    case VFTDC_READOUT_TS_POLL:
    case VFTDC_READOUT_EXT_POLL:
      break;

    case VFTDC_READOUT_TS_INT:
    case VFTDC_READOUT_EXT_INT:
#ifdef VXWORKS
      intConnect(INUM_TO_IVEC(vfTDCIntVec),vfTDCInt,arg);
#else
      status = vmeIntConnect (vfTDCIntVec, vfTDCIntLevel,
			      vfTDCInt,arg);
      if (status != OK) 
	{
	  printf("%s: vmeIntConnect failed with status = 0x%08x\n",
		 __FUNCTION__,status);
	  return(ERROR);
	}
#endif  
      break;

    default:
      printf("%s: ERROR: TI Mode not defined (%d)\n",
	     __FUNCTION__,tiReadoutMode);
      return ERROR;
    }

  printf("%s: INFO: Interrupt Vector = 0x%x  Level = %d\n",
	 __FUNCTION__,vfTDCIntVec,vfTDCIntLevel);

  if(routine) 
    {
      vfTDCIntRoutine = routine;
      vfTDCIntArg = arg;
    }
  else
    {
      vfTDCIntRoutine = NULL;
      vfTDCIntArg = 0;
    }

  return(OK);

}

/**
 * @ingroup IntPoll
 * @brief Disable interrupts or kill the polling service thread
 *
 *
 * @return OK if successful, otherwise ERROR
 */
int
vfTDCIntDisconnect()
{
#ifndef VXWORKS
  int status;
  void *res;
#endif

  if(id==0) id=vfTDCID[0];

  if((id<=0) || (id>21) || (TDCp[id] == NULL)) 
    {
      printf("%s: ERROR : TDC in slot %d is not initialized \n",
	     __FUNCTION__,id);
      return ERROR;
    }

  if(vfTDCIntRunning) 
    {
      logMsg("vfTDCIntDisconnect: ERROR: vfTDC is Enabled - Call vfTDCIntDisable() first\n",
	     1,2,3,4,5,6);
      return ERROR;
    }

  INTLOCK;

  switch (tiReadoutMode) 
    {
    case VFTDC_READOUT_TS_INT:
    case VFTDC_READOUT_EXT_INT:

#ifdef VXWORKS
      /* Disconnect any current interrupts */
      sysIntDisable(vfTDCIntLevel);
      if((intDisconnect(vfTDCIntVec) !=0))
	printf("%s: Error disconnecting Interrupt\n",__FUNCTION__);
#else
      status = vmeIntDisconnect(vfTDCIntLevel);
      if (status != OK) 
	{
	  printf("vmeIntDisconnect failed\n");
	}
#endif
      break;

    case VFTDC_READOUT_TS_POLL:
    case VFTDC_READOUT_EXT_POLL:
#ifndef VXWORKS
      if(vfTDCpollthread) 
	{
	  if(pthread_cancel(vfTDCpollthread)<0) 
	    perror("pthread_cancel");
	  if(pthread_join(vfTDCpollthread,&res)<0)
	    perror("pthread_join");
	  if (res == PTHREAD_CANCELED)
	    printf("%s: Polling thread canceled\n",__FUNCTION__);
	  else
	    printf("%s: ERROR: Polling thread NOT canceled\n",__FUNCTION__);
	}
#endif
      break;
    default:
      break;
    }

  INTUNLOCK;

  printf("%s: Disconnected\n",__FUNCTION__);

  return OK;
  
}

/**
 * @ingroup IntPoll
 * @brief Connect a user routine to be executed instead of the default 
 *  TI interrupt/trigger latching acknowledge prescription
 *
 * @param routine Routine to call 
 * @param arg argument to pass to routine
 * @return OK if successful, otherwise ERROR
 */
int
vfTDCAckConnect(VOIDFUNCPTR routine, unsigned int arg)
{
  if(routine)
    {
      vfTDCAckRoutine = routine;
      vfTDCAckArg = arg;
    }
  else
    {
      printf("%s: WARN: routine undefined.\n",__FUNCTION__);
      vfTDCAckRoutine = NULL;
      vfTDCAckArg = 0;
      return ERROR;
    }
  return OK;
}

/**
 * @ingroup IntPoll
 * @brief Acknowledge an interrupt or latched trigger.  This "should" effectively 
 *  release the "Busy" state of the TI.
 *
 *  Execute a user defined routine, if it is defined.  Otherwise, use
 *  a default prescription.
 */
void
vfTDCIntAck()
{
  int resetbits=0;
  if(id==0) id=vfTDCID[0];

  if((id<=0) || (id>21) || (TDCp[id] == NULL)) 
    {
      printf("%s: ERROR : TDC in slot %d is not initialized \n",
	     __FUNCTION__,id);
      return ERROR;
    }

  if (vfTDCAckRoutine != NULL)
    {
      /* Execute user defined Acknowlege, if it was defined */
      VLOCK;
      (*vfTDCAckRoutine) (vfTDCAckArg);
      VUNLOCK;
    }
  else
    {
      VLOCK;
      vfTDCDoAck = 1;
      vfTDCAckCount++;
      resetbits = VFTDC_RESET_BUSYACK;

      if(!vfTDCReadoutEnabled)
	{
	  /* Readout Acknowledge and decrease the number of available blocks by 1 */
	  resetbits |= VFTDC_RESET_BLOCK_READOUT;
	}
      
      vmeWrite32(&TDCp->reset, resetbits);

      VUNLOCK;
    }

}

/**
 * @ingroup IntPoll
 * @brief Enable interrupts or latching triggers (depending on set TI mode)
 *  
 * @param iflag if = 1, trigger counter will be reset
 *
 * @return OK if successful, otherwise ERROR
 */
int
vfTDCIntEnable(int iflag)
{
#ifdef VXWORKS
  int lock_key=0;
#endif

  if(id==0) id=vfTDCID[0];

  if((id<=0) || (id>21) || (TDCp[id] == NULL)) 
    {
      printf("%s: ERROR : TDC in slot %d is not initialized \n",
	     __FUNCTION__,id);
      return ERROR;
    }

  VLOCK;
  if(iflag == 1)
    {
      vfTDCIntCount = 0;
      vfTDCAckCount = 0;
    }

  vfTDCIntRunning = 1;
  vfTDCDoAck      = 1;
  vfTDCNeedAck    = 0;

  switch (vfTDCReadoutMode)
    {
    case VFTDC_READOUT_TS_POLL:
    case VFTDC_READOUT_EXT_POLL:
#ifndef VXWORKS
      vfTDCStartPollingThread();
#endif
      break;

    case VFTDC_READOUT_TS_INT:
    case VFTDC_READOUT_EXT_INT:
#ifdef VXWORKS
      lock_key = intLock();
      sysIntEnable(vfTDCIntLevel);
#endif
      printf("%s: ******* ENABLE INTERRUPTS *******\n",__FUNCTION__);
      vmeWrite32(&TDCp->intsetup,
	       vmeRead32(&TDCp->intsetup) | VFTDC_INTSETUP_ENABLE );
      break;

    default:
      vfTDCIntRunning = 0;
#ifdef VXWORKS
      if(lock_key)
	intUnlock(lock_key);
#endif
      printf("%s: ERROR: vfTDC Readout Mode not defined %d\n",
	     __FUNCTION__,tiReadoutMode);
      VUNLOCK;
      return(ERROR);
      
    }

  vmeWrite32(&TDCp->runningMode,0x71);
  VUNLOCK; /* Locks performed in tiEnableTriggerSource() */

  taskDelay(30);
  vfTDCEnableTriggerSource();

#ifdef VXWORKS
  if(lock_key)
    intUnlock(lock_key);
#endif

  return(OK);

}

/**
 * @ingroup IntPoll
 * @brief Disable interrupts or latching triggers
 *
*/
void 
vfTDCIntDisable()
{

  if(id==0) id=vfTDCID[0];

  if((id<=0) || (id>21) || (TDCp[id] == NULL)) 
    {
      printf("%s: ERROR : TDC in slot %d is not initialized \n",
	     __FUNCTION__,id);
      return ERROR;
    }

  vfTDCDisableTriggerSource(1);

  VLOCK;
  vmeWrite32(&TDCp->intsetup,
	     vmeRead32(&TDCp->intsetup) & ~(VFTDC_INTSETUP_ENABLE));
  vmeWrite32(&TDCp->runningMode,0x0);
  vfTDCIntRunning = 0;
  VUNLOCK;
}

/**
 * @ingroup Status
 * @brief Return current readout count
 */
unsigned int
vfTDCGetIntCount()
{
  unsigned int rval=0;

  VLOCK;
  rval = vfTDCIntCount;
  VUNLOCK;

  return(rval);
}

/**
 * @ingroup Status
 * @brief Return current acknowledge count
 */
unsigned int
vfTDCGetAckCount()
{
  unsigned int rval=0;

  VLOCK;
  rval = vfTDCAckCount;
  VUNLOCK;

  return(rval);
}
#endif /* NOTYET */

/**
 *  @ingroup Status
 *  @brief Decode a data word from an vfTDC and print to standard out.
 *  @param data 32bit vfTDC data word
 */

struct vftdc_data_struct vftdc_data;

void 
vfTDCDataDecode(unsigned int data)
{
  int i_print = 1;
  static unsigned int type_last = 15;	/* initialize to type FILLER WORD */
  static unsigned int time_last = 0;

  if( data & 0x80000000 )		/* data type defining word */
    {
      vftdc_data.new_type = 1;
      vftdc_data.type = (data & 0x78000000) >> 27;
    }
  else
    {
      vftdc_data.new_type = 0;
      vftdc_data.type = type_last;
    }
        
  switch( vftdc_data.type )
    {
    case 0:		/* BLOCK HEADER */
      vftdc_data.slot_id_hd = ((data) & 0x7C00000) >> 22;
      vftdc_data.modID      = (data & 0x3C0000)>>18;
      vftdc_data.blk_num    = (data & 0x3FF00) >> 8;
      vftdc_data.n_evts     = (data & 0xFF);
      if( i_print ) 
	printf("%8X - BLOCK HEADER - slot = %d  modID = %d   n_evts = %d   n_blk = %d\n",
	       data, vftdc_data.slot_id_hd, 
	       vftdc_data.modID, vftdc_data.n_evts, vftdc_data.blk_num);
      break;

    case 1:		/* BLOCK TRAILER */
      vftdc_data.slot_id_tr = (data & 0x7C00000) >> 22;
      vftdc_data.n_words = (data & 0x3FFFFF);
      if( i_print ) 
	printf("%8X - BLOCK TRAILER - slot = %d   n_words = %d\n",
	       data, vftdc_data.slot_id_tr, vftdc_data.n_words);
      break;

    case 2:		/* EVENT HEADER */
      vftdc_data.slot_id_evh = (data & 0x7C00000) >> 22;
      vftdc_data.evt_num_1 = (data & 0x3FFFFF);
      if( i_print ) 
	printf("%8X - EVENT HEADER - slot = %d   evt_num = %d\n", data, 
	       vftdc_data.slot_id_evh, vftdc_data.evt_num_1);
      break;

    case 3:		/* TRIGGER TIME */
      if( vftdc_data.new_type )
	{
	  vftdc_data.time_1 = (data & 0x7FFFFFF);
	  if( i_print ) 
	    printf("%8X - TRIGGER TIME 1 - time = %08x\n", data, vftdc_data.time_1);
	  vftdc_data.time_now = 1;
	  time_last = 1;
	}    
      else
	{
	  if( time_last == 1 )
	    {
	      vftdc_data.time_2 = (data & 0xFFFFFF);
	      if( i_print ) 
		printf("%8X - TRIGGER TIME 2 - time = %08x\n", data, vftdc_data.time_2);
	      vftdc_data.time_now = 2;
	    }    
	  else
	    if( i_print ) 
	      printf("%8X - TRIGGER TIME - (ERROR)\n", data);
	                
	  time_last = vftdc_data.time_now;
	}    
      break;

    case 7:
      vftdc_data.group        = (data & 0x07000000)>>24;
      vftdc_data.chan         = (data & 0x00f80000)>>19;
      vftdc_data.edge_type    = (data & 0x00040000)>>18;
      vftdc_data.time_coarse  = (data & 0x0003ff00)>>8;
      vftdc_data.two_ns       = (data & 0x00000080)>>7;
      vftdc_data.time_fine    = (data & 0x0000007f)>>0;

      printf("%8X - TDC - grp = %d  ch = %2d  edge = %d  coarse = %4d  2ns = %d  fine time = %3d\n", 
	     data, 
	     vftdc_data.group,
	     vftdc_data.chan,
	     vftdc_data.edge_type,
	     vftdc_data.time_coarse,
	     vftdc_data.two_ns,
	     vftdc_data.time_fine);
      break;
 
    case 4:
    case 5:
    case 6:
    case 8:
    case 9:
    case 10:
    case 11:
    case 12:		/* UNDEFINED TYPE */
    case 13:
      if( i_print ) 
	printf("%8X - UNDEFINED TYPE = %d\n", data, vftdc_data.type);
      break;
 
    case 14:		/* DATA NOT VALID (no data available) */
      if( i_print ) 
	printf("%8X - DATA NOT VALID = %d\n", data, vftdc_data.type);
      break;

    case 15:		/* FILLER WORD */
      if( i_print ) 
	printf("%8X - FILLER WORD = %d\n", data, vftdc_data.type);
      break;
    }
	
  type_last = vftdc_data.type;	/* save type of current data word */
		   
}        
