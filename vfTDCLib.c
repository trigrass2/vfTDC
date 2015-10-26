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
volatile struct VFTDC_A24RegStruct *TDCp[VFTDC_MAX_BOARDS+1];  /* pointer to vfTDC memory map */
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
static VOIDFUNCPTR  vfTDCAckRoutine  = NULL;    /* user trigger acknowledge routine */
static int          vfTDCAckArg      = 0;       /* arg to user trigger ack routine */
int                 vfTDCBlockError  = VFTDC_BLOCKERROR_NO_ERROR; /* Whether (>0) or not (0) Block Transfer had an error */

/* Interrupt/Polling routine prototypes (static) */
static void vfTDCInt(void);
#ifndef VXWORKS
static void vfTDCPoll(void);
static void vfTDCStartPollingThread(void);
/* polling thread pthread and pthread_attr */
pthread_attr_t vfTDCpollthread_attr;
pthread_t      vfTDCpollthread;
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
 * @defgroup Config Initialization/Configuration
 * @defgroup SDCConfig SDC Initialization/Configuration
 *   @ingroup Config
 * @defgroup Status Status
 * @defgroup SDCStatus SDC Status
 *   @ingroup Status
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
 * @param nadc
 *  - Number of times to increment
 *
 *  @param iFlag 18 bit integer
 * <pre>
 *       Low 6 bits - Specifies the default Signal distribution (clock,trigger) 
 *                    sources for the board (Internal, FrontPanel, VXS, VME(Soft))
 *       bit    0:  defines Sync Reset source
 *                     0  VME (Software Sync-Reset)
 *                     1  Front Panel/VXS/P2 (Depends on Clk/Trig source selection)
 *       bits 3-1:  defines Trigger source
 *               0 0 0  VME (Software Triggers)
 *               0 0 1  Front Panel Input
 *               0 1 0  VXS (P0) 
 *               1 0 0  Internal Trigger Logic (HITSUM FPGA)
 *               (all others Undefined - default to VME/Software)
 *       bits 5-4:  defines Clock Source
 *           0 0  Internal 250MHz Clock
 *           0 1  Front Panel 
 *           1 0  VXS (P0)
 *           1 1  P2 Connector (Backplane)
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
vfTDCInit(UINT32 addr, UINT32 addr_inc, int nadc, int iFlag)
{
  int ii, res, errFlag = 0;
  int boardID = 0;
  int maxSlot = 1;
  int minSlot = 21;
  int trigSrc=0, clkSrc=0, srSrc=0;
  unsigned int rdata=0, a32addr=0;
  unsigned long laddr=0, laddr_inc=0;
  volatile struct vfTDC_struct *fa;
  unsigned short sdata;
  int noBoardInit=0;
  int useList=0;
  int noFirmwareCheck=0;

  /* Check if we are to exit when pointers are setup */
  noBoardInit=(iFlag&VFTDC_INIT_SKIP)>>16;

  /* Check if we're initializing using a list */
  useList=(iFlag&VFTDC_INIT_USE_ADDRLIST)>>17;

  /* Are we skipping the firmware check? */
  noFirmwareCheck=(iFlag&VFTDC_INIT_SKIP_FIRMWARE_CHECK)>>18;

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
      if( ((addr_inc==0)||(nadc==0)) && (useList==0) )
	nadc = 1; /* assume only one vfTDC to initialize */

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
  vfTDCSource = iFlag&VFTDC_SOURCE_MASK;
  vfTDCInited = nvfTDC = 0;
  bzero((char *)vfTDCID,sizeof(vfTDCID));

  for (ii=0;ii<nadc;ii++) 
    {
      if(useList==1)
	{
	  laddr_inc = vfTDCAddrList[ii] + vfTDCA24Offset;
	}
      else
	{
	  laddr_inc = laddr +ii*addr_inc;
	}
      fa = (struct vfTDC_struct *)laddr_inc;
      /* Check if Board exists at that address */
#ifdef VXWORKS
      res = vxMemProbe((char *) &(fa->version),VX_READ,4,(char *)&rdata);
#else
      res = vmeMemProbe((char *) &(fa->version),4,(char *)&rdata);
#endif
      if(res < 0) 
	{
#ifdef VXWORKS
	  printf("%s: WARN: No addressable board at addr=0x%x\n",
		 __FUNCTION__,(UINT32) fa);
#else
	  printf("%s: WARN: No addressable board at VME (Local) addr=0x%x (0x%lx)\n",
		 __FUNCTION__,
		 (UINT32)(laddr_inc-vfTDCA24Offset), (unsigned long) fa);
#endif
	  errFlag = 1;
	  continue;
	}
      else 
	{
	  /* Check that it is an FA board */
	  if((rdata&VFTDC_BOARD_MASK) != VFTDC_BOARD_ID) 
	    {
	      printf("%s: WARN: For board at 0x%x, Invalid Board ID: 0x%x\n",
		     __FUNCTION__,
		     (UINT32)(laddr_inc-vfTDCA24Offset), rdata);
	      continue;
	    }
	  else 
	    {
	      /* Check if this is board has a valid slot number */
	      boardID =  ((vmeRead32(&(fa->intr)))&VFTDC_SLOT_ID_MASK)>>16;

	      if((boardID <= 0)||(boardID >21)) 
		{
		  printf("%s: ERROR: For Board at 0x%x,  Slot number is not in range: %d\n",
			 __FUNCTION__,(UINT32)(laddr_inc-vfTDCA24Offset), boardID);
		  continue;
		}

	      if(!noFirmwareCheck)
		{
		  /* Check Control FPGA firmware version */
		  if( (rdata&VFTDC_VERSION_MASK) < VFTDC_SUPPORTED_CTRL_FIRMWARE )
		    {
		      printf("%s: ERROR: Slot %2d: Control FPGA Firmware (0x%02x) not supported by this driver.\n",
			     __FUNCTION__,boardID, rdata & VFTDC_VERSION_MASK);
		      printf("\tUpdate to 0x%02x to use this driver.\n",VFTDC_SUPPORTED_CTRL_FIRMWARE);
		      continue;
		    }

		  /* Check Processing FPGA firmware version */
		  proc_version = 
		    (unsigned short)(vmeRead32(&fa->adc_status[0]) & VFTDC_ADC_VERSION_MASK);

		  for(icheck=0; icheck<VFTDC_SUPPORTED_PROC_FIRMWARE_NUMBER; icheck++)
		    {
		      if(proc_version == supported_proc[icheck])
			proc_supported=1;
		    }

		  if(proc_supported==0)
		    {
		      printf("%s: ERROR: Slot %2d: Proc FPGA Firmware (0x%02x) not supported by this driver.\n",
			     __FUNCTION__,boardID, proc_version);
		      printf("\tSupported Proc Firmware:  ");
		      for(icheck=0; icheck<VFTDC_SUPPORTED_PROC_FIRMWARE_NUMBER; icheck++)
			{
			  printf("0x%02x ",supported_proc[icheck]);
			}
		      printf("\n");
		      continue;
		    }
		}
	      else
		{
		  /* Check Control FPGA firmware version */
		  if( (rdata&VFTDC_VERSION_MASK) < VFTDC_SUPPORTED_CTRL_FIRMWARE )
		    {
		      printf("%s: WARN: Slot %2d: Control FPGA Firmware (0x%02x) not supported by this driver (ignored).\n",
			     __FUNCTION__,boardID, rdata & VFTDC_VERSION_MASK);
		    }

		  /* Check Processing FPGA firmware version */
		  proc_version = 
		    (unsigned short)(vmeRead32(&fa->adc_status[0]) & VFTDC_ADC_VERSION_MASK);

		  for(icheck=0; icheck<VFTDC_SUPPORTED_PROC_FIRMWARE_NUMBER; icheck++)
		    {
		      if(proc_version == supported_proc[icheck])
			proc_supported=1;
		    }

		  if(proc_supported==0)
		    {
		      printf("%s: WARN: Slot %2d: Proc FPGA Firmware (0x%02x) not supported by this driver (ignored).\n",
			     __FUNCTION__,boardID, proc_version & VFTDC_VERSION_MASK);
		    }
		}

	      FAp[boardID] = (struct vfTDC_struct *)(laddr_inc);
	      vfTDCRev[boardID] = rdata&VFTDC_VERSION_MASK;
	      vfTDCProcRev[boardID] = proc_version;
	      vfTDCID[nvfTDC] = boardID;
	      if(boardID >= maxSlot) maxSlot = boardID;
	      if(boardID <= minSlot) minSlot = boardID;
	      
	      printf("Initialized VFTDC %2d  Slot #%2d at VME (Local) address 0x%06x (0x%lx) \n",
		     nvfTDC,vfTDCID[nvfTDC],
		     (UINT32) (unsigned long)(FAp[(vfTDCID[nvfTDC])]-vfTDCA24Offset),
		     (unsigned long) FAp[(vfTDCID[nvfTDC])]);
	    }
	  nvfTDC++;
	}
    }

  /* Check if we are using a JLAB VFTDC Signal Distribution Card (SDC)
     NOTE the SDC board only supports 7 VFTDCs - so if there are
     more than 7 VFTDCs in the crate they can only be controlled by daisychaining 
     multiple SDCs together - or by using a VXS Crate with SD switch card 
  */
  a16addr = iFlag&VFTDC_SDC_ADR_MASK;
  if(a16addr) 
    {
#ifdef VXWORKS
      res = sysBusToLocalAdrs(0x29,(char *)a16addr,(char **)&laddr);
      if (res != 0) 
	{
	  printf("%s: ERROR in sysBusToLocalAdrs(0x29,0x%x,&laddr) \n",
		 __FUNCTION__,a16addr);
	  return(ERROR);
	}

      res = vxMemProbe((char *) laddr,VX_READ,2,(char *)&sdata);
#else
      res = vmeBusToLocalAdrs(0x29,(char *)(unsigned long)a16addr,(char **)&laddr);
      if (res != 0) 
	{
	  printf("%s: ERROR in vmeBusToLocalAdrs(0x29,0x%x,&laddr) \n",
		 __FUNCTION__,a16addr);
	  return(ERROR);
	}
      res = vmeMemProbe((char *) laddr,2,(char *)&sdata);
#endif
      if(res < 0) 
	{
	  printf("%s: ERROR: No addressable SDC board at addr=0x%x\n",
		 __FUNCTION__,(UINT32) laddr);
	} 
      else 
	{
	  vfTDCA16Offset = laddr-a16addr;
	  FASDCp = (struct vfTDC_sdc_struct *) laddr;
	  if(!noBoardInit)
	    vmeWrite16(&(FASDCp->ctrl),FASDC_CSR_INIT);   /* Reset the Module */

	  if(nvfTDC>7) 
	    {
	      printf("WARN: A Single JLAB VFTDC Signal Distribution Module only supports 7 VFTDCs\n");
	      printf("WARN: You must use multiple SDCs to support more VFTDCs - this must be configured in hardware\n");
	    }
#ifdef VXWORKS
	  printf("Using JLAB VFTDC Signal Distribution Module at address 0x%x\n",
		 (UINT32) FASDCp); 
#else
	  printf("Using JLAB VFTDC Signal Distribution Module at VME (Local) address 0x%x (0x%lx)\n",
		 (UINT32)a16addr, (unsigned long) FASDCp); 
#endif
	  vfTDCUseSDC=1;
	}
      if(vfTDCSource == VFTDC_SOURCE_SDC) 
	{  /* Check if SDC will be used */
	  vfTDCUseSDC = 1;
	  printf("%s: JLAB VFTDC Signal Distribution Card is Assumed in Use\n",
		 __FUNCTION__);
	  printf("%s: Front Panel Inputs will be enabled. \n",
		 __FUNCTION__);
	}
      else
	{
	  vfTDCUseSDC = 0;
	  printf("%s: JLAB VFTDC Signal Distribution Card will not be Used\n",
		 __FUNCTION__);
	}
    }

  /* Hard Reset of all VFTDC boards in the Crate */
  if(!noBoardInit)
    {
      for(ii=0;ii<nvfTDC;ii++) 
	{
	  vmeWrite32(&(FAp[vfTDCID[ii]]->reset),VFTDC_RESET_ALL);
	}
      taskDelay(60); 
    }

  /* Initialize Interrupt variables */
  vfTDCIntID = -1;
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

  if(!noBoardInit)
    {
      /* what are the Trigger Sync Reset and Clock sources */
      if (vfTDCSource == VFTDC_SOURCE_VXS)
	{
	  printf("%s: Enabling VFTDC for VXS Clock ",__FUNCTION__);
	  clkSrc  = VFTDC_REF_CLK_P0;
	  switch (iFlag&0xf) 
	    {
	    case 0: case 1:
	      printf("and Software Triggers (Soft Sync Reset)\n");
	      trigSrc = VFTDC_TRIG_VME | VFTDC_ENABLE_SOFT_TRIG;
	      srSrc   = VFTDC_SRESET_VME | VFTDC_ENABLE_SOFT_SRESET;
	      break;
	    case 2:
	      printf("and Front Panel Triggers (Soft Sync Reset)\n");
	      trigSrc = VFTDC_TRIG_FP_ISYNC;
	      srSrc   = VFTDC_SRESET_VME | VFTDC_ENABLE_SOFT_SRESET;
	      break;
	    case 3:
	      printf("and Front Panel Triggers (FP Sync Reset)\n");
	      trigSrc = VFTDC_TRIG_FP_ISYNC;
	      srSrc   = VFTDC_SRESET_FP_ISYNC;
	      break;
	    case 4: case 6:
	      printf("and VXS Triggers (Soft Sync Reset)\n");
	      trigSrc = VFTDC_TRIG_P0_ISYNC;
	      srSrc   = VFTDC_SRESET_VME | VFTDC_ENABLE_SOFT_SRESET;
	      break;
	    case 5: case 7:
	      printf("and VXS Triggers (VXS Sync Reset)\n");
	      trigSrc = VFTDC_TRIG_P0_ISYNC;
	      srSrc   = VFTDC_SRESET_P0_ISYNC;
	      break;
	    case 8: case 10: case 12: case 14:
	      printf("and Internal Trigger Logic (Soft Sync Reset)\n");
	      trigSrc = VFTDC_TRIG_INTERNAL;
	      srSrc   = VFTDC_SRESET_VME | VFTDC_ENABLE_SOFT_SRESET;
	      break;
	    case 9: case 11: case 13: case 15:
	      printf("and Internal Trigger Logic (VXS Sync Reset)\n");
	      trigSrc = VFTDC_TRIG_INTERNAL;
	      srSrc   = VFTDC_SRESET_FP_ISYNC;
	      break;
	    }
	}
      else if (vfTDCSource == VFTDC_SOURCE_SDC) 
	{
	  printf("%s: Enabling VFTDC for SDC Clock (Front Panel) ",__FUNCTION__);
	  clkSrc  = VFTDC_REF_CLK_FP;
	  switch (iFlag&0xf) 
	    {
	    case 0: case 1:
	      printf("and Software Triggers (Soft Sync Reset)\n");
	      trigSrc = VFTDC_TRIG_VME | VFTDC_ENABLE_SOFT_TRIG;
	      srSrc   = VFTDC_SRESET_VME | VFTDC_ENABLE_SOFT_SRESET;
	      break;
	    case 2: case 4: case 6:
	      printf("and Front Panel Triggers (Soft Sync Reset)\n");
	      trigSrc = VFTDC_TRIG_FP_ISYNC;
	      srSrc   = VFTDC_SRESET_VME | VFTDC_ENABLE_SOFT_SRESET;
	      break;
	    case 3: case 5: case 7:
	      printf("and Front Panel Triggers (FP Sync Reset)\n");
	      trigSrc = VFTDC_TRIG_FP_ISYNC;
	      srSrc   = VFTDC_SRESET_FP_ISYNC;
	      break;
	    case 8: case 10: case 12: case 14:
	      printf("and Internal Trigger Logic (Soft Sync Reset)\n");
	      trigSrc = VFTDC_TRIG_INTERNAL;
	      srSrc   = VFTDC_SRESET_VME | VFTDC_ENABLE_SOFT_SRESET;
	      break;
	    case 9: case 11: case 13: case 15:
	      printf("and Internal Trigger Logic (Front Panel Sync Reset)\n");
	      trigSrc = VFTDC_TRIG_INTERNAL;
	      srSrc   = VFTDC_SRESET_FP_ISYNC;
	      break;
	    }
	  faSDC_Config(0,0);
	}
      else 
	{  /* Use internal Clk */
	  printf("%s: Enabling VFTDC Internal Clock, ",__FUNCTION__);
	  clkSrc = VFTDC_REF_CLK_INTERNAL;
	  switch (iFlag&0xf) 
	    {
	    case 0: case 1:
	      printf("and Software Triggers (Soft Sync Reset)\n");
	      trigSrc = VFTDC_TRIG_VME | VFTDC_ENABLE_SOFT_TRIG;
	      srSrc   = VFTDC_SRESET_VME | VFTDC_ENABLE_SOFT_SRESET ;
	      break;
	    case 2:
	      printf("and Front Panel Triggers (Soft Sync Reset)\n");
	      trigSrc = VFTDC_TRIG_FP_ISYNC;
	      srSrc   = VFTDC_SRESET_VME | VFTDC_ENABLE_SOFT_SRESET;
	      break;
	    case 3:
	      printf("and Front Panel Triggers (FP Sync Reset)\n");
	      trigSrc = VFTDC_TRIG_FP_ISYNC;
	      srSrc   = VFTDC_SRESET_FP_ISYNC;
	      break;
	    case 4: case 6:
	      printf("and VXS Triggers (Soft Sync Reset)\n");
	      trigSrc = VFTDC_TRIG_P0_ISYNC;
	      srSrc   = VFTDC_SRESET_VME | VFTDC_ENABLE_SOFT_SRESET;
	      break;
	    case 5: case 7:
	      printf("and VXS Triggers (VXS Sync Reset)\n");
	      trigSrc = VFTDC_TRIG_P0_ISYNC;
	      srSrc   = VFTDC_SRESET_P0_ISYNC;
	      break;
	    case 8: case 10: case 12: case 14:
	      printf("and Internal Trigger Logic (Soft Sync Reset)\n");
	      trigSrc = VFTDC_TRIG_INTERNAL;
	      srSrc   = VFTDC_SRESET_VME | VFTDC_ENABLE_SOFT_SRESET;
	      break;
	    case 9: case 11: case 13: case 15:
	      printf("and Internal Trigger Logic (Front Panel Sync Reset)\n");
	      trigSrc = VFTDC_TRIG_INTERNAL;
	      srSrc   = VFTDC_SRESET_FP_ISYNC;
	      break;
	    }
	}
    }

  /* Enable Clock source - Internal Clk enabled by default */ 
  if(!noBoardInit)
    {
      for(ii=0;ii<nvfTDC;ii++) 
	{
	  vmeWrite32(&(FAp[vfTDCID[ii]]->ctrl1),(clkSrc | VFTDC_ENABLE_INTERNAL_CLK)) ;
	}
      taskDelay(20);


      /* Hard Reset FPGAs and FIFOs */
      for(ii=0;ii<nvfTDC;ii++) 
	{
	  vmeWrite32(&(FAp[vfTDCID[ii]]->reset),
		     (VFTDC_RESET_ADC_FPGA1 | VFTDC_RESET_ADC_FIFO1 |
		      VFTDC_RESET_DAC | VFTDC_RESET_EXT_RAM_PT));

#ifdef CLAS12
	  vmeWrite32(&(FAp[vfTDCID[ii]]->gtx_ctrl),0x203); /*put reset*/
	  vmeWrite32(&(FAp[vfTDCID[ii]]->gtx_ctrl),0x800); /*release reset*/
#else
	  /* Release reset on MGTs */
	  vmeWrite32(&(FAp[vfTDCID[ii]]->mgt_ctrl),VFTDC_RELEASE_MGT_RESET);
	  vmeWrite32(&(FAp[vfTDCID[ii]]->mgt_ctrl),VFTDC_MGT_RESET);
	  vmeWrite32(&(FAp[vfTDCID[ii]]->mgt_ctrl),VFTDC_RELEASE_MGT_RESET);
#endif
	}
      taskDelay(5);
    }

  /* Write configuration registers with default/defined Sources */
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
      FApd[vfTDCID[ii]] = (unsigned int *)(laddr);  /* Set a pointer to the FIFO */
      if(!noBoardInit)
	{
	  vmeWrite32(&(FAp[vfTDCID[ii]]->adr32),(a32addr>>16) + 1);  /* Write the register and enable */
	
	  /* Set Default Block Level to 1 */
	  vmeWrite32(&(FAp[vfTDCID[ii]]->blk_level),1);
	}
      vfTDCBlockLevel=1;

      /* Setup Trigger and Sync Reset sources */
      if(!noBoardInit)
	{
	  vmeWrite32(&(FAp[vfTDCID[ii]]->ctrl1),
		     vmeRead32(&(FAp[vfTDCID[ii]]->ctrl1)) | 
		     (srSrc | trigSrc) );
	}
#ifndef CLAS12      
      /* Set default stop and busy conditions (modified in faSetProcMode(..)) */
      faSetTriggerStopCondition(vfTDCID[ii], 9);
      faSetTriggerBusyCondition(vfTDCID[ii], 9);
#endif
    }

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
      FApmb = (unsigned int *)(laddr);  /* Set a pointer to the FIFO */
      if(!noBoardInit)
	{
	  for (ii=0;ii<nvfTDC;ii++) 
	    {
	      /* Write the register and enable */
	      vmeWrite32(&(FAp[vfTDCID[ii]]->adr_mb),
			 (a32addr+VFTDC_MAX_A32MB_SIZE) + (a32addr>>16) + VFTDC_A32_ENABLE);
	    }
	}    
      /* Set First Board and Last Board */
      vfTDCMaxSlot = maxSlot;
      vfTDCMinSlot = minSlot;
      if(!noBoardInit)
	{
	  vmeWrite32(&(FAp[minSlot]->ctrl1),
		     vmeRead32(&(FAp[minSlot]->ctrl1)) | VFTDC_FIRST_BOARD);
	  vmeWrite32(&(FAp[maxSlot]->ctrl1),
		     vmeRead32(&(FAp[maxSlot]->ctrl1)) | VFTDC_LAST_BOARD);
	}    
    }

  if(!noBoardInit)
    vfTDCInited = nvfTDC;

  if(errFlag > 0) 
    {
      printf("%s: WARN: Unable to initialize all requested VFTDC Modules (%d)\n",
	     __FUNCTION__,nadc);
      if(nvfTDC > 0)
	printf("%s: %d VFTDC(s) successfully initialized\n",FUNCTION__,nvfTDC );
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
  
  if(TDCp==NULL)
    {
      printf("%s: ERROR: VFTDC not initialized\n",__FUNCTION__);
      return ERROR;
    }

  printf("%s:\n\t ---------- Checking TI address space ---------- \n",__FUNCTION__);

  base = (unsigned long) &TDCp->boardID;

  offset = ((unsigned long) &TDCp->trigsrc) - base;
  expected = 0x20;
  if(offset != expected)
    printf("%s: ERROR TDCp->triggerSource not at offset = 0x%lx (@ 0x%lx)\n",
	   __FUNCTION__,expected,offset);


  return OK;
}

/**
 * @ingroup Status
 * @brief Print some status information of the TI to standard out
 * 
 * @param pflag if pflag>0, print out raw registers
 *
 */

void
vfTDCStatus(int pflag)
{
  if(TDCp==NULL)
    {
      printf("%s: ERROR: vfTDC not initialized\n",__FUNCTION__);
      return;
    }

  VLOCK;
  VUNLOCK;

  printf("\n");
#ifdef VXWORKS
  printf("STATUS for vfTDC at base address 0x%08x \n",
	 (unsigned int) TDCp);
#else
  printf("STATUS for vfTDC at VME (Local) base address 0x%08lx (0x%lx) \n",
	 (unsigned long) TDCp - vfTDCA24Offset, (unsigned long) TDCp);
#endif
  printf("--------------------------------------------------------------------------------\n");

  printf("--------------------------------------------------------------------------------\n");
  printf("\n\n");

}


/**
 * @ingroup Status
 * @brief Get the Firmware Version
 *
 * @return Firmware Version if successful, ERROR otherwise
 *
 */
int
vfTDCGetFirmwareVersion()
{
  unsigned int rval=0;
  if(TDCp==NULL)
    {
      printf("%s: ERROR: vfTDC not initialized\n",__FUNCTION__);
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
  
  if(TDCp==NULL)
    {
      printf("%s: ERROR: vfTDC not initialized\n",__FUNCTION__);
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

/**
 * @ingroup Config
 * @brief Perform a soft reset of the TI
 *
 * @return OK if successful, ERROR otherwise
 *
 */
int
vfTDCReset()
{
  if(TDCp==NULL)
    {
      printf("%s: ERROR: vfTDC not initialized\n",__FUNCTION__);
      return ERROR;
    }
  
  VLOCK;
  vmeWrite32(&TDCp->reset,VFTDC_RESET_SOFT);
  VUNLOCK;
  return OK;
}

/**
 * @ingroup MasterConfig
 * @brief Set the number of events per block
 * @param blockLevel Number of events per block
 * @return OK if successful, ERROR otherwise
 *
 */
int
vfTDCSetBlockLevel(int blockLevel)
{
  return tiBroadcastNextBlockLevel(blockLevel);
}

/**
 * @ingroup Config
 * @brief Set the trigger source
 *     This routine will set a library variable to be set in the TI registers
 *     at a call to vfTDCIntEnable.  
 *
 *  @param trig - integer indicating the trigger source
 *         - 0: P0
 *         - 1: HFBR#1
 *         - 2: Front Panel (TRG)
 *         - 3: Front Panel TS Inputs
 *         - 4: TS (rev2) 
 *         - 5: Random
 *         - 6-9: TS Partition 1-4
 *         - 10: HFBR#5
 *         - 11: Pulser Trig 2 then Trig1 after specified delay 
 *
 * @return OK if successful, ERROR otherwise
 *
 */
int
vfTDCSetTriggerSource(int trig)
{
  unsigned int trigenable=0;

  if(TDCp==NULL)
    {
      printf("%s: ERROR: vfTDC not initialized\n",__FUNCTION__);
      return ERROR;
    }

  if( (trig>10) || (trig<0) )
    {
      printf("%s: ERROR: Invalid Trigger Source (%d).  Must be between 0 and 10.\n",
	     __FUNCTION__,trig);
      return ERROR;
    }


  if(!tiMaster)
    { 
      /* Setup for TI Slave */
      trigenable = VFTDC_TRIGSRC_VME;

      if((trig>=6) && (trig<=9)) /* TS partition specified */
	{
	  if(tiSlaveFiberIn!=1)
	    {
	      printf("%s: WARN: Partition triggers NOT USED on Fiber Port 5.\n",
		     __FUNCTION__);
	      trigenable |= VFTDC_TRIGSRC_HFBR5;
	    }

	  trigenable |= VFTDC_TRIGSRC_HFBR1;
	  switch(trig)
	    {
	    case VFTDC_TRIGGER_PART_1:
	      trigenable |= VFTDC_TRIGSRC_PART_1;
	      break;
	  
	    case VFTDC_TRIGGER_PART_2:
	      trigenable |= VFTDC_TRIGSRC_PART_2;
	      break;
	  
	    case VFTDC_TRIGGER_PART_3:
	      trigenable |= VFTDC_TRIGSRC_PART_3;
	      break;

	    case VFTDC_TRIGGER_PART_4:
	      trigenable |= VFTDC_TRIGSRC_PART_4;
	      break;
	    }
	}
      else
	{
	  if(tiSlaveFiberIn==1)
	    {
	      trigenable |= VFTDC_TRIGSRC_HFBR1;
	    }
	  else if(tiSlaveFiberIn==5)
	    {
	      trigenable |= VFTDC_TRIGSRC_HFBR5;
	    }
	  if( (trig != VFTDC_TRIGGER_HFBR1) || (trig != VFTDC_TRIGGER_HFBR5) )
	    {
	      printf("%s: WARN:  Only valid trigger source for TI Slave is HFBR%d (trig = %d)",
		     __FUNCTION__, tiSlaveFiberIn,
		     (tiSlaveFiberIn==1)?VFTDC_TRIGGER_HFBR1:VFTDC_TRIGGER_HFBR5);
	      printf("  Ignoring specified trig (%d)\n",trig);
	    }
	}

    }
  else
    {
      /* Setup for TI Master */

      /* Set VME and Loopback by default */
      trigenable  = VFTDC_TRIGSRC_VME;
      trigenable |= VFTDC_TRIGSRC_LOOPBACK;

      switch(trig)
	{
	case VFTDC_TRIGGER_P0:
	  trigenable |= VFTDC_TRIGSRC_P0;
	  break;

	case VFTDC_TRIGGER_HFBR1:
	  trigenable |= VFTDC_TRIGSRC_HFBR1;
	  break;

	case VFTDC_TRIGGER_HFBR5:
	  trigenable |= VFTDC_TRIGSRC_HFBR5;
	  break;

	case VFTDC_TRIGGER_FPTRG:
	  trigenable |= VFTDC_TRIGSRC_FPTRG;
	  break;

	case VFTDC_TRIGGER_TSINPUTS:
	  trigenable |= VFTDC_TRIGSRC_TSINPUTS;
	  break;

	case VFTDC_TRIGGER_TSREV2:
	  trigenable |= VFTDC_TRIGSRC_TSREV2;
	  break;

	case VFTDC_TRIGGER_PULSER:
	  trigenable |= VFTDC_TRIGSRC_PULSER;
	  break;

	case VFTDC_TRIGGER_TRIG21:
	  trigenable |= VFTDC_TRIGSRC_PULSER;
	  trigenable |= VFTDC_TRIGSRC_TRIG21;
	  break;

	default:
	  printf("%s: ERROR: Invalid Trigger Source (%d) for TI Master\n",
		 __FUNCTION__,trig);
	  return ERROR;
	}
    }

  tiTriggerSource = trigenable;
  printf("%s: INFO: tiTriggerSource = 0x%x\n",__FUNCTION__,tiTriggerSource);

  return OK;
}

/**
 * @ingroup Config
 * @brief Set trigger sources with specified trigmask
 *    This routine is for special use when tiSetTriggerSource(...) does
 *    not set all of the trigger sources that is required by the user.
 *
 * @param trigmask bits:  
 *        -         0:  P0
 *        -         1:  HFBR #1 
 *        -         2:  TI Master Loopback
 *        -         3:  Front Panel (TRG) Input
 *        -         4:  VME Trigger
 *        -         5:  Front Panel TS Inputs
 *        -         6:  TS (rev 2) Input
 *        -         7:  Random Trigger
 *        -         8:  FP/Ext/GTP 
 *        -         9:  P2 Busy 
 *        -        10:  HFBR #5
 *        -        11:  Pulser Trig2 with delayed Trig1 (only compatible with 2 and 7)
 *
 * @return OK if successful, ERROR otherwise
 *
 */
int
vfTDCSetTriggerSourceMask(int trigmask)
{
  if(TDCp==NULL)
    {
      printf("%s: ERROR: vfTDC not initialized\n",__FUNCTION__);
      return ERROR;
    }

  /* Check input mask */
  if(trigmask>VFTDC_TRIGSRC_SOURCEMASK)
    {
      printf("%s: ERROR: Invalid trigger source mask (0x%x).\n",
	     __FUNCTION__,trigmask);
      return ERROR;
    }

  tiTriggerSource = trigmask;

  return OK;
}

/**
 * @ingroup Config
 * @brief Enable trigger sources
 * Enable trigger sources set by 
 *                          tiSetTriggerSource(...) or
 *                          tiSetTriggerSourceMask(...)
 * @sa tiSetTriggerSource
 * @sa tiSetTriggerSourceMask
 *
 * @return OK if successful, ERROR otherwise
 *
 */
int
vfTDCEnableTriggerSource()
{
  if(TDCp==NULL)
    {
      printf("%s: ERROR: vfTDC not initialized\n",__FUNCTION__);
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
  if(TDCp==NULL)
    {
      printf("%s: ERROR: vfTDC not initialized\n",__FUNCTION__);
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

/**
 * @ingroup Config
 * @brief Set the Sync source mask
 *
 * @param sync - MASK indicating the sync source
 *       bit: description
 *       -  0: P0
 *       -  1: HFBR1
 *       -  2: HFBR5
 *       -  3: FP
 *       -  4: LOOPBACK
 *
 * @return OK if successful, ERROR otherwise
 *
 */
int
vfTDCSetSyncSource(unsigned int sync)
{
  if(TDCp==NULL)
    {
      printf("%s: ERROR: vfTDC not initialized\n",__FUNCTION__);
      return ERROR;
    }

  if(sync>VFTDC_SYNC_SOURCEMASK)
    {
      printf("%s: ERROR: Invalid Sync Source Mask (%d).\n",
	     __FUNCTION__,sync);
      return ERROR;
    }

  VLOCK;
  vmeWrite32(&TDCp->sync,sync);
  VUNLOCK;

  return OK;
}

/**
 * @ingroup MasterConfig
 * @brief Set and enable the "software" trigger
 *
 *  @param trigger  trigger type 1 or 2 (playback trigger)
 *  @param nevents  integer number of events to trigger
 *  @param period_inc  period multiplier, depends on range (0-0x7FFF)
 *  @param range  
 *     - 0: small period range (min: 120ns, increments of 120ns)
 *     - 1: large period range (min: 120ns, increments of 245.7us)
 *
 * @return OK if successful, ERROR otherwise
 *
 */
int
vfTDCSoftTrig(int trigger, unsigned int nevents, unsigned int period_inc, int range)
{
  unsigned int periodMax=(VFTDC_FIXEDPULSER1_PERIOD_MASK>>16);
  unsigned int reg=0;
  int time=0;

  if(TDCp==NULL)
    {
      logMsg("\ntiSoftTrig: ERROR: TI not initialized\n",1,2,3,4,5,6);
      return ERROR;
    }

  if(trigger!=1 && trigger!=2)
    {
      logMsg("\ntiSoftTrig: ERROR: Invalid trigger type %d\n",trigger,2,3,4,5,6);
      return ERROR;
    }

  if(nevents>VFTDC_FIXEDPULSER1_NTRIGGERS_MASK)
    {
      logMsg("\ntiSoftTrig: ERROR: nevents (%d) must be less than %d\n",nevents,
	     VFTDC_FIXEDPULSER1_NTRIGGERS_MASK,3,4,5,6);
      return ERROR;
    }
  if(period_inc>periodMax)
    {
      logMsg("\ntiSoftTrig: ERROR: period_inc (%d) must be less than %d ns\n",
	     period_inc,periodMax,3,4,5,6);
      return ERROR;
    }
  if( (range!=0) && (range!=1) )
    {
      logMsg("\ntiSoftTrig: ERROR: range must be 0 or 1\n",
	     periodMax,2,3,4,5,6);
      return ERROR;
    }

  if(range==0)
    time = 120+120*period_inc;
  if(range==1)
    time = 120+120*period_inc*2048;

  logMsg("\ntiSoftTrig: INFO: Setting software trigger for %d nevents with period of %d\n",
	 nevents,time,3,4,5,6);

  reg = (range<<31)| (period_inc<<16) | (nevents);
  VLOCK;
  if(trigger==1)
    {
      vmeWrite32(&TDCp->fixedPulser1, reg);
    }
  else if(trigger==2)
    {
      vmeWrite32(&TDCp->fixedPulser2, reg);
    }
  VUNLOCK;

  return OK;

}

/**
 * @ingroup Readout
 * @brief Read a block of events from the TI
 *
 * @param   data  - local memory address to place data
 * @param   nwrds - Max number of words to transfer
 * @param   rflag - Readout Flag
 *       -       0 - programmed I/O from the specified board
 *       -       1 - DMA transfer using Universe/Tempe DMA Engine 
 *                    (DMA VME transfer Mode must be setup prior)
 *
 * @return Number of words transferred to data if successful, ERROR otherwise
 *
 */
int
vfTDCReadBlock(volatile unsigned int *data, int nwrds, int rflag)
{
  int ii, dummy=0;
  int dCnt, retVal, xferCount;
  volatile unsigned int *laddr;
  unsigned int vmeAdr, val;

  if(TDCp==NULL)
    {
      logMsg("\ntiReadBlock: ERROR: TI not initialized\n",1,2,3,4,5,6);
      return ERROR;
    }

  if(TDCpd==NULL)
    {
      logMsg("\ntiReadBlock: ERROR: TI A32 not initialized\n",1,2,3,4,5,6);
      return ERROR;
    }

  if(data==NULL) 
    {
      logMsg("\ntiReadBlock: ERROR: Invalid Destination address\n",0,0,0,0,0,0);
      return(ERROR);
    }

  VLOCK;
  if(rflag >= 1)
    { /* Block transfer */
      if(tiBusError==0)
	{
	  logMsg("tiReadBlock: WARN: Bus Error Block Termination was disabled.  Re-enabling\n",
		 1,2,3,4,5,6);
	  VUNLOCK;
	  tiEnableBusError();
	  VLOCK;
	}
      /* Assume that the DMA programming is already setup. 
	 Don't Bother checking if there is valid data - that should be done prior
	 to calling the read routine */
      
      /* Check for 8 byte boundary for address - insert dummy word (Slot 0 VFTDC Dummy DATA)*/
      if((unsigned long) (data)&0x7)
	{
#ifdef VXWORKS
	  *data = (VFTDC_DATA_TYPE_DEFINE_MASK) | (VFTDC_FILLER_WORD_TYPE) | (tiSlotNumber<<22);
#else
	  *data = LSWAP((VFTDC_DATA_TYPE_DEFINE_MASK) | (VFTDC_FILLER_WORD_TYPE) | (tiSlotNumber<<22));
#endif
	  dummy = 1;
	  laddr = (data + 1);
	} 
      else 
	{
	  dummy = 0;
	  laddr = data;
	}
      
      vmeAdr = (unsigned long)TDCpd - vfTDCA32Offset;

#ifdef VXWORKS
      retVal = sysVmeDmaSend((UINT32)laddr, vmeAdr, (nwrds<<2), 0);
#else
      retVal = vmeDmaSend((unsigned long)laddr, vmeAdr, (nwrds<<2));
#endif
      if(retVal != 0) 
	{
	  logMsg("\ntiReadBlock: ERROR in DMA transfer Initialization 0x%x\n",retVal,0,0,0,0,0);
	  VUNLOCK;
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
#ifdef VXWORKS
	  xferCount = (nwrds - (retVal>>2) + dummy); /* Number of longwords transfered */
#else
	  xferCount = ((retVal>>2) + dummy); /* Number of longwords transfered */
#endif
	  VUNLOCK;
	  return(xferCount);
	}
      else if (retVal == 0) 
	{
#ifdef VXWORKS
	  logMsg("\ntiReadBlock: WARN: DMA transfer terminated by word count 0x%x\n",
		 nwrds,0,0,0,0,0);
#else
	  logMsg("\ntiReadBlock: WARN: DMA transfer returned zero word count 0x%x\n",
		 nwrds,0,0,0,0,0,0);
#endif
	  VUNLOCK;
	  return(nwrds);
	}
      else 
	{  /* Error in DMA */
#ifdef VXWORKS
	  logMsg("\ntiReadBlock: ERROR: sysVmeDmaDone returned an Error\n",
		 0,0,0,0,0,0);
#else
	  logMsg("\ntiReadBlock: ERROR: vmeDmaDone returned an Error\n",
		 0,0,0,0,0,0);
#endif
	  VUNLOCK;
	  return(retVal>>2);
	  
	}
    }
  else
    { /* Programmed IO */
      if(tiBusError==1)
	{
	  logMsg("tiReadBlock: WARN: Bus Error Block Termination was enabled.  Disabling\n",
		 1,2,3,4,5,6);
	  VUNLOCK;
	  tiDisableBusError();
	  VLOCK;
	}

      dCnt = 0;
      ii=0;

      while(ii<nwrds) 
	{
	  val = (unsigned int) *TDCpd;
#ifndef VXWORKS
	  val = LSWAP(val);
#endif
	  if(val == (VFTDC_DATA_TYPE_DEFINE_MASK | VFTDC_BLOCK_TRAILER_WORD_TYPE 
		     | (tiSlotNumber<<22) | (ii+1)) )
	    {
#ifndef VXWORKS
	      val = LSWAP(val);
#endif
	      data[ii] = val;
	      if(((ii+1)%2)!=0)
		{
		  /* Read out an extra word (filler) in the fifo */
		  val = (unsigned int) *TDCpd;
#ifndef VXWORKS
		  val = LSWAP(val);
#endif
		  if(((val & VFTDC_DATA_TYPE_DEFINE_MASK) != VFTDC_DATA_TYPE_DEFINE_MASK) ||
		     ((val & VFTDC_WORD_TYPE_MASK) != VFTDC_FILLER_WORD_TYPE))
		    {
		      logMsg("\ntiReadBlock: ERROR: Unexpected word after block trailer (0x%08x)\n",
			     val,2,3,4,5,6);
		    }
		}
	      break;
	    }
#ifndef VXWORKS
	  val = LSWAP(val);
#endif
	  data[ii] = val;
	  ii++;
	}
      ii++;
      dCnt += ii;

      VUNLOCK;
      return(dCnt);
    }

  VUNLOCK;

  return OK;
}

/**
 * @ingroup Config
 * @brief Enable Fiber transceiver
 *
 *  Note:  All Fiber are enabled by default 
 *         (no harm, except for 1-2W power usage)
 *
 * @sa tiDisableFiber
 * @param   fiber: integer indicative of the transceiver to enable
 *
 *
 * @return OK if successful, ERROR otherwise.
 *
 */
int
vfTDCEnableFiber(unsigned int fiber)
{
  unsigned int sval;
  unsigned int fiberbit;

  if(TDCp==NULL)
    {
      printf("%s: ERROR: vfTDC not initialized\n",__FUNCTION__);
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
 * @param   fiber: integer indicative of the transceiver to disable
 *
 *
 * @return OK if successful, ERROR otherwise.
 *
 */
int
vfTDCDisableFiber(unsigned int fiber)
{
  unsigned int rval;
  unsigned int fiberbit;

  if(TDCp==NULL)
    {
      printf("%s: ERROR: vfTDC not initialized\n",__FUNCTION__);
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

/**
 * @ingroup Config
 * @brief Set the busy source with a given sourcemask sourcemask bits: 
 *
 * @param sourcemask 
 *  - 0: SWA
 *  - 1: SWB
 *  - 2: P2
 *  - 3: FP-FTDC
 *  - 4: FP-FADC
 *  - 5: FP
 *  - 6: Unused
 *  - 7: Loopack
 *  - 8-15: Fiber 1-8
 *
 * @param rFlag - decision to reset the global source flags
 *       -      0: Keep prior busy source settings and set new "sourcemask"
 *       -      1: Reset, using only that specified with "sourcemask"
 * @return OK if successful, ERROR otherwise.
 */
int
vfTDCSetBusySource(unsigned int sourcemask, int rFlag)
{
  unsigned int busybits=0;

  if(TDCp==NULL)
    {
      printf("%s: ERROR: vfTDC not initialized\n",__FUNCTION__);
      return ERROR;
    }
  
  if(sourcemask>VFTDC_BUSY_SOURCEMASK)
    {
      printf("%s: ERROR: Invalid value for sourcemask (0x%x)\n",
	     __FUNCTION__, sourcemask);
      return ERROR;
    }

  VLOCK;
  if(rFlag)
    {
      /* Read in the previous value , resetting previous BUSYs*/
      busybits = vmeRead32(&TDCp->busy) & ~(VFTDC_BUSY_SOURCEMASK);
    }
  else
    {
      /* Read in the previous value , keeping previous BUSYs*/
      busybits = vmeRead32(&TDCp->busy);
    }

  busybits |= sourcemask;

  vmeWrite32(&TDCp->busy, busybits);
  VUNLOCK;

  return OK;

}

/**
 * @ingroup Config
 * @brief Enable Bus Errors to terminate Block Reads
 * @sa tiDisableBusError
 * @return OK if successful, otherwise ERROR
 */
void
vfTDCEnableBusError()
{

  if(TDCp==NULL)
    {
      printf("%s: ERROR: vfTDC not initialized\n",__FUNCTION__);
      return;
    }

  VLOCK;
  vmeWrite32(&TDCp->vmeControl,
	   vmeRead32(&TDCp->vmeControl) | (VFTDC_VMECONTROL_BERR) );
  tiBusError=1;
  VUNLOCK;

}

/**
 * @ingroup Config
 * @brief Disable Bus Errors to terminate Block Reads
 * @sa tiEnableBusError
 * @return OK if successful, otherwise ERROR
 */
void
vfTDCDisableBusError()
{

  if(TDCp==NULL)
    {
      printf("%s: ERROR: vfTDC not initialized\n",__FUNCTION__);
      return;
    }

  VLOCK;
  vmeWrite32(&TDCp->vmeControl,
	   vmeRead32(&TDCp->vmeControl) & ~(VFTDC_VMECONTROL_BERR) );
  tiBusError=0;
  VUNLOCK;

}


/**
 * @ingroup MasterConfig
 * @brief Generate a Sync Reset signal.  This signal is sent to the loopback and
 *    all configured TI Slaves.
 *
 *  @param blflag Option to change block level, after SyncReset issued
 *       -   0: Do not change block level
 *       -  >0: Broadcast block level to all connected slaves (including self)
 *            BlockLevel broadcasted will be set to library value
 *            (Set with tiSetBlockLevel)
 *
 */
void
vfTDCSyncReset(int blflag)
{
  if(TDCp == NULL) 
    {
      printf("%s: ERROR: vfTDC not initialized\n",__FUNCTION__);
      return;
    }
  
  VLOCK;
  vmeWrite32(&TDCp->syncCommand,tiSyncResetType); 
  taskDelay(1);
  vmeWrite32(&TDCp->syncCommand,VFTDC_SYNCCOMMAND_RESET_EVNUM); 
  taskDelay(1);
  VUNLOCK;
  
  if(blflag) /* Set the block level from "Next" to Current */
    {
      printf("%s: INFO: Setting Block Level to %d\n",
	     __FUNCTION__,tiNextBlockLevel);
      tiBroadcastNextBlockLevel(tiNextBlockLevel);
    }

}

/**
 * @ingroup Config
 * @brief Routine to set the A32 Base
 *
 * @return OK if successful, otherwise ERROR
 */
int
vfTDCSetAdr32(unsigned int a32base)
{
  unsigned long laddr=0;
  int res=0,a32Enabled=0;

  if(TDCp == NULL) 
    {
      printf("%s: ERROR: vfTDC not initialized\n",__FUNCTION__);
      return ERROR;
    }

  if(a32base<0x00800000)
    {
      printf("%s: ERROR: a32base out of range (0x%08x)\n",
	     __FUNCTION__,a32base);
      return ERROR;
    }

  VLOCK;
  vmeWrite32(&TDCp->adr32, 
	     (a32base & VFTDC_ADR32_BASE_MASK) );

  vmeWrite32(&TDCp->vmeControl, 
	     vmeRead32(&TDCp->vmeControl) | VFTDC_VMECONTROL_A32);

  a32Enabled = vmeRead32(&TDCp->vmeControl)&(VFTDC_VMECONTROL_A32);
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
  TDCpd = (unsigned int *)(laddr);  /* Set a pointer to the FIFO */
  VUNLOCK;

  printf("%s: A32 Base address set to 0x%08x\n",
	 __FUNCTION__,vfTDCA32Base);

  return OK;
}

/**
 * @ingroup Config
 * @brief Disable A32
 *
 * @return OK if successful, otherwise ERROR
 */
int
vfTDCDisableA32()
{
  if(TDCp == NULL) 
    {
      printf("%s: ERROR: vfTDC not initialized\n",__FUNCTION__);
      return ERROR;
    }
  
  VLOCK;
  vmeWrite32(&TDCp->adr32,0x0);
  vmeWrite32(&TDCp->vmeControl, 
	     vmeRead32(&TDCp->vmeControl) & ~VFTDC_VMECONTROL_A32);
  VUNLOCK;

  return OK;
}

/**
 * @ingroup Config
 * @brief Reset the L1A counter, as incremented by the TI.
 *
 * @return OK if successful, otherwise ERROR
 */
int
vfTDCResetEventCounter()
{
  if(TDCp == NULL) 
    {
      printf("%s: ERROR: vfTDC not initialized\n",__FUNCTION__);
      return ERROR;
    }
  
  VLOCK;
  vmeWrite32(&TDCp->reset, VFTDC_RESET_SCALERS_RESET);
  VUNLOCK;

  return OK;
}

/**
 * @ingroup Status
 * @brief Returns the event counter (48 bit)
 *
 * @return Number of accepted events if successful, otherwise ERROR
 */
unsigned long long int
vfTDCGetEventCounter()
{
  unsigned long long int rval=0;
  unsigned int lo=0, hi=0;

  if(TDCp == NULL) 
    {
      printf("%s: ERROR: vfTDC not initialized\n",__FUNCTION__);
      return ERROR;
    }

  VLOCK;
  lo = vmeRead32(&TDCp->eventNumber_lo);
  hi = (vmeRead32(&TDCp->eventNumber_hi) & VFTDC_EVENTNUMBER_HI_MASK)>>16;

  rval = lo | ((unsigned long long)hi<<32);
  VUNLOCK;
  
  return rval;
}


/**
 * @ingroup Readout
 * @brief Returns the number of Blocks available for readout
 *
 * @return Number of blocks available for readout if successful, otherwise ERROR
 *
 */
unsigned int
vfTDCBReady()
{
  unsigned int blockBuffer=0, readyInt=0, rval=0;

  if(TDCp == NULL) 
    {
      logMsg("vfTDCBReady: ERROR: vfTDC not initialized\n",1,2,3,4,5,6);
      return 0;
    }

  VLOCK;
  blockBuffer = vmeRead32(&TDCp->blockBuffer);
  rval        = (blockBuffer&VFTDC_BLOCKBUFFER_BLOCKS_READY_MASK)>>8;
  readyInt    = (blockBuffer&VFTDC_BLOCKBUFFER_BREADY_INT_MASK)>>24;
  tiSyncEventReceived = (blockBuffer&VFTDC_BLOCKBUFFER_SYNCEVENT)>>31;
  tiNReadoutEvents = (blockBuffer&VFTDC_BLOCKBUFFER_RO_NEVENTS_MASK)>>24;

  if( (readyInt==1) && (tiSyncEventReceived) )
    tiSyncEventFlag = 1;
  else
    tiSyncEventFlag = 0;

  VUNLOCK;

  return rval;
}

/**
 * @ingroup Config
 * @brief Set the clock to the specified source.
 *
 * @param   source
 *         -   0:  Onboard clock
 *         -   1:  External clock (HFBR1 input)
 *         -   5:  External clock (HFBR5 input)
 *
 * @return OK if successful, otherwise ERROR
 */
int
vfTDCSetClockSource(unsigned int source)
{
  int rval=OK;
  unsigned int clkset=0;
  unsigned int clkread=0;
  char sClock[20] = "";

  if(TDCp == NULL) 
    {
      printf("%s: ERROR: vfTDC not initialized\n",__FUNCTION__);
      return ERROR;
    }

  switch(source)
    {
    case 0: /* ONBOARD */
      clkset = VFTDC_CLOCK_INTERNAL;
      sprintf(sClock,"ONBOARD (%d)",source);
      break;
    case 1: /* EXTERNAL (HFBR1) */
      clkset = VFTDC_CLOCK_HFBR1;
      sprintf(sClock,"EXTERNAL-HFBR1 (%d)",source);
      break;
    case 5: /* EXTERNAL (HFBR5) */
      clkset = VFTDC_CLOCK_HFBR5;
      sprintf(sClock,"EXTERNAL-HFBR5 (%d)",source);
      break;
    default:
      printf("%s: ERROR: Invalid Clock Souce (%d)\n",__FUNCTION__,source);
      return ERROR;      
    }

  printf("%s: Setting clock source to %s\n",__FUNCTION__,sClock);


  VLOCK;
  vmeWrite32(&TDCp->clock, clkset);
  /* Reset DCM (Digital Clock Manager) - 250/200MHz */
  vmeWrite32(&TDCp->reset,VFTDC_RESET_CLK250);
  taskDelay(1);
  /* Reset DCM (Digital Clock Manager) - 125MHz */
  vmeWrite32(&TDCp->reset,VFTDC_RESET_CLK125);
  taskDelay(1);

  if(source==1) /* Turn on running mode for External Clock verification */
    {
      vmeWrite32(&TDCp->runningMode,VFTDC_RUNNINGMODE_ENABLE);
      taskDelay(1);
      clkread = vmeRead32(&TDCp->clock) & VFTDC_CLOCK_MASK;
      if(clkread != clkset)
	{
	  printf("%s: ERROR Setting Clock Source (clkset = 0x%x, clkread = 0x%x)\n",
		 __FUNCTION__,clkset, clkread);
	  rval = ERROR;
	}
      vmeWrite32(&TDCp->runningMode,VFTDC_RUNNINGMODE_DISABLE);
    }
  VUNLOCK;

  return rval;
}

/**
 * @ingroup Status
 * @brief Get the current clock source
 * @return Current Clock Source
 */
int
vfTDCGetClockSource()
{
  int rval=0;
  if(TDCp == NULL) 
    {
      printf("%s: ERROR: vfTDC not initialized\n",__FUNCTION__);
      return ERROR;
    }

  VLOCK;
  rval = vmeRead32(&TDCp->clock) & 0x3;
  VUNLOCK;

  return rval;
}

/**
 * @ingroup Status
 * @brief Return geographic address as provided from a VME-64X crate.
 * @return Geographic Address if successful, otherwise ERROR.  0 would indicate that the TI is not in a VME-64X crate.
 */

int
vfTDCGetGeoAddress()
{
  int rval=0;
  if(TDCp==NULL)
    {
      printf("%s: ERROR: vfTDC not initialized\n",__FUNCTION__);
      return ERROR;
    }

  VLOCK;
  rval = (vmeRead32(&TDCp->adr24) & VFTDC_ADR24_GEOADDR_MASK)>>10;
  VUNLOCK;

  return rval;
}

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

  if(TDCp == NULL) 
    {
      printf("%s: ERROR: vfTDC not initialized\n",__FUNCTION__);
      return(ERROR);
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

  if(TDCp == NULL) 
    {
      printf("%s: ERROR: vfTDC not initialized\n",__FUNCTION__);
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
  if(TDCp == NULL) {
    logMsg("vfTDCIntAck: ERROR: vfTDC not initialized\n",0,0,0,0,0,0);
    return;
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

  if(TDCp == NULL) 
    {
      printf("%s: ERROR: vfTDC not initialized\n",__FUNCTION__);
      return(-1);
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

  if(TDCp == NULL) 
    {
      printf("%s: ERROR: vfTDC not initialized\n",__FUNCTION__);
      return;
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


