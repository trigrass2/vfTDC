/*----------------------------------------------------------------------------*
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
 *----------------------------------------------------------------------------*/
#ifndef VFTDCLIB_H
#define VFTDCLIB_H

/* Define default Interrupt vector and level */
#define VFTDC_INT_VEC      0xec
#define VFTDC_INT_LEVEL    5

#define VFTDC_MAX_BOARDS             20
#define VFTDC_MAX_ADC_CHANNELS       16
#define VFTDC_MAX_DATA_PER_CHANNEL  251
#define VFTDC_MAX_A32_MEM      0x800000   /* 8 Meg */
#define VFTDC_MAX_A32MB_SIZE   0x800000  /*  8 MB */
#define VFTDC_VME_INT_LEVEL           3     
#define VFTDC_VME_INT_VEC          0xFA

#define VFTDC_SUPPORTED_FIRMWARE 0xA2

#ifndef VXWORKS
#include <pthread.h>

#else
/* #include <intLib.h> */
extern int intLock();
extern int intUnlock();
#endif

#ifdef VXWORKS
int intLockKeya;
#define INTLOCK {				\
    intLockKeya = intLock();			\
}

#define INTUNLOCK {				\
    intUnlock(intLockKeya);			\
}
#else
#define INTLOCK {				\
    vmeBusLock();				\
}
#define INTUNLOCK {				\
    vmeBusUnlock();				\
}
#endif

struct vfTDC_struct
{
  /** 0x0000 */ volatile unsigned int boardID;
  /** 0x0004 */ volatile unsigned int ptw;
  /** 0x0008 */ volatile unsigned int intsetup;
  /** 0x000C */ volatile unsigned int pl;
  /** 0x0010 */ volatile unsigned int adr32;
  /** 0x0014 */ volatile unsigned int blocklevel;
  /** 0x0018 */          unsigned int blank0;
  /** 0x001C */ volatile unsigned int vmeControl;
  /** 0x0020 */ volatile unsigned int trigsrc;
  /** 0x0024 */ volatile unsigned int sync;
  /** 0x0028 */ volatile unsigned int busy;
  /** 0x002C */ volatile unsigned int clock;
  /** 0x0030 */          unsigned int blank1[(0x4C-0x30)/4];
  /** 0x004C */ volatile unsigned int blockBuffer;
  /** 0x0050 */          unsigned int blank2[(0x9C-0x50)/4];
  /** 0x009C */ volatile unsigned int runningMode;
  /** 0x00A0 */          unsigned int blank3[(0xA8-0xA0)/4];
  /** 0x00A8 */ volatile unsigned int livetime;
  /** 0x00AC */ volatile unsigned int busytime;
  /** 0x00B0 */          unsigned int blank4[(0xD8-0xB0)/4];
  /** 0x00D8 */ volatile unsigned int eventNumber_hi;
  /** 0x00DC */ volatile unsigned int eventNumber_lo;
  /** 0x00E0 */          unsigned int blank5[(0xEC-0xE0)/4];
  /** 0x00EC */ volatile unsigned int rocEnable;
  /** 0x00F0 */          unsigned int blank6[(0x100-0xF0)/4];
  /** 0x0100 */ volatile unsigned int reset;

};

/* 0x0 boardID bits and masks */
#define VFTDC_BOARDID_TYPE_VFTDC        0xF7DC
#define VFTDC_BOARDID_TYPE_MASK     0xFFFF0000
#define VFTDC_BOARDID_GEOADR_MASK   0x00001F00
#define VFTDC_BOARDID_CRATEID_MASK  0x000000FF

/* 0x8 intsetup bits and masks */
#define VFTDC_INTSETUP_VECTOR_MASK   0x000000FF
#define VFTDC_INTSETUP_LEVEL_MASK    0x00000F00
#define VFTDC_INTSETUP_ENABLE        (1<<16)

/* 0x10 adr32 bits and masks */
#define VFTDC_ADR32_MBLK_ADDR_MAX_MASK  0x000003FE
#define VFTDC_ADR32_MBLK_ADDR_MIN_MASK  0x003FC000
#define VFTDC_ADR32_BASE_MASK           0xFF800000

/* 0x1C vmeControl bits and masks */
#define VFTDC_VMECONTROL_BERR           (1<<0)
#define VFTDC_VMECONTROL_TOKEN_TESTMODE (1<<1)
#define VFTDC_VMECONTROL_MBLK           (1<<2)
#define VFTDC_VMECONTROL_A32M           (1<<3)
#define VFTDC_VMECONTROL_A32            (1<<4)
#define VFTDC_VMECONTROL_ERROR_INT      (1<<7)
#define VFTDC_VMECONTROL_I2CDEV_HACK    (1<<8)
#define VFTDC_VMECONTROL_TOKENOUT_HI    (1<<9)
#define VFTDC_VMECONTROL_FIRST_BOARD    (1<<10)
#define VFTDC_VMECONTROL_LAST_BOARD     (1<<11)
#define VFTDC_VMECONTROL_BUFFER_DISABLE (1<<15)

/* 0x20 trigsrc bits and masks */
#define VFTDC_TRIGSRC_SOURCEMASK       0x0000FFFF
#define VFTDC_TRIGSRC_P0               (1<<0)
#define VFTDC_TRIGSRC_HFBR1            (1<<1)
#define VFTDC_TRIGSRC_FPTRG            (1<<3)
#define VFTDC_TRIGSRC_VME              (1<<4)
#define VFTDC_TRIGSRC_PULSER           (1<<7)
#define VFTDC_TRIGSRC_MONITOR_MASK     0xFFFF0000

/* 0x24 sync bits and masks */
#define VFTDC_SYNC_SOURCEMASK              0x0000FFFF
#define VFTDC_SYNC_P0                      (1<<0)
#define VFTDC_SYNC_HFBR1                   (1<<1)
#define VFTDC_SYNC_FP                      (1<<3)
#define VFTDC_SYNC_VME                     (1<<4)
#define VFTDC_SYNC_MONITOR_MASK            0xFF000000

/* 0x28 busy bits and masks */
#define VFTDC_BUSY_SOURCEMASK      0x0000FFFF
#define VFTDC_BUSY_SWA              (1<<0)
#define VFTDC_BUSY_SWB              (1<<1)
#define VFTDC_BUSY_P2               (1<<2)
#define VFTDC_BUSY_FP_FTDC          (1<<3)
#define VFTDC_BUSY_FP_FADC          (1<<4)
#define VFTDC_BUSY_FP               (1<<5)
#define VFTDC_BUSY_LOOPBACK         (1<<7)
#define VFTDC_BUSY_HFBR1            (1<<8)
#define VFTDC_BUSY_HFBR2            (1<<9)
#define VFTDC_BUSY_HFBR3            (1<<10)
#define VFTDC_BUSY_HFBR4            (1<<11)
#define VFTDC_BUSY_HFBR5            (1<<12)
#define VFTDC_BUSY_HFBR6            (1<<13)
#define VFTDC_BUSY_HFBR7            (1<<14)
#define VFTDC_BUSY_HFBR8            (1<<15)
#define VFTDC_BUSY_MONITOR_FIFOFULL (1<<16)
#define VFTDC_BUSY_MONITOR_MASK     0xFFFF0000

/* 0x2C clock bits and mask  */
#define VFTDC_CLOCK_FP          (0)
#define VFTDC_CLOCK_INTERNAL    (2)
#define VFTDC_CLOCK_P0          (3)
#define VFTDC_CLOCK_MASK        0x00000003

/* 0x34 blockBuffer bits and masks */
#define VFTDC_BLOCKBUFFER_BLOCKS_READY_MASK     0x0000FF00
#define VFTDC_BLOCKBUFFER_BREADY_INT_MASK       0x00FF0000
#define VFTDC_BLOCKBUFFER_TRIGGERS_IN_BLOCK     0xFF000000

/* 0x9C runningMode settings */
#define VFTDC_RUNNINGMODE_ENABLE          0xF7
#define VFTDC_RUNNINGMODE_CALIB_P2_AD     0xF8
#define VFTDC_RUNNINGMODE_CALIB_P2_CD     0xF9
#define VFTDC_RUNNINGMODE_CALIB_FP_A      0xFA
#define VFTDC_RUNNINGMODE_CALIB_FP_B      0xFB
#define VFTDC_RUNNINGMODE_CALIB_FP_C      0xFC
#define VFTDC_RUNNINGMODE_CALIB_FP_D      0xFD
#define VFTDC_RUNNINGMODE_DISABLE         0x00

/* 0xD8 eventNumber_hi bits and masks */
#define TI_EVENTNUMBER_HI_MASK        0xFFFF0000

/* 0xEC rocEnable bits and masks */
#define TI_ROCENABLE_MASK             0x000000FF
#define TI_ROCENABLE_ROC(x)           (1<<(x))

/* 0x100 reset bits and masks */
#define TI_RESET_I2C                  (1<<1)
#define TI_RESET_SOFT                 (1<<4)
#define TI_RESET_SYNCRESET            (1<<5)
#define TI_RESET_BUSYACK              (1<<7)
#define TI_RESET_CLK250               (1<<8)
#define TI_RESET_MGT                  (1<<10)
#define TI_RESET_AUTOALIGN_HFBR1_SYNC (1<<11)
#define TI_RESET_TRIGGER              (1<<12)
#define TI_RESET_IODELAY              (1<<14)
#define TI_RESET_TAKE_TOKEN           (1<<16)
#define TI_RESET_BLOCK_READOUT        (1<<17)
#define TI_RESET_SCALERS_LATCH        (1<<24)
#define TI_RESET_SCALERS_RESET        (1<<25)

/* faInit initialization flag bits */
#define VFTDC_INIT_SOFT_SYNCRESET      (0<<0)
#define VFTDC_INIT_EXT_SYNCRESET       (1<<0)
#define VFTDC_INIT_SOFT_TRIG           (0<<1)
#define VFTDC_INIT_FP_TRIG             (1<<1)
#define VFTDC_INIT_VXS_TRIG            (2<<1)
#define VFTDC_INIT_INT_TRIG            (4<<1)
#define VFTDC_INIT_INT_CLKSRC          (0<<4)
#define VFTDC_INIT_FP_CLKSRC           (1<<4)
#define VFTDC_INIT_VXS_CLKSRC          (2<<4)
#define VFTDC_INIT_P2_CLKSRC           ((1<<4) | (2<<4))
#define VFTDC_INIT_SKIP                (1<<16)
#define VFTDC_INIT_USE_ADDRLIST        (1<<17)
#define VFTDC_INIT_SKIP_FIRMWARE_CHECK (1<<18)

/* vfTDCBlockError values */
#define VFTDC_BLOCKERROR_NO_ERROR          0
#define VFTDC_BLOCKERROR_TERM_ON_WORDCOUNT 1
#define VFTDC_BLOCKERROR_UNKNOWN_BUS_ERROR 2
#define VFTDC_BLOCKERROR_ZERO_WORD_COUNT   3
#define VFTDC_BLOCKERROR_DMADONE_ERROR     4
#define VFTDC_BLOCKERROR_NTYPES            5

/* Function prototypes */



#endif /* VFTDCLIB_H */
