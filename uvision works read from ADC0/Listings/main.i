# 1 "main.c"
# 1 "<built-in>" 1
# 1 "<built-in>" 3
# 379 "<built-in>" 3
# 1 "<command line>" 1
# 1 "<built-in>" 2
# 1 "main.c" 2
# 10 "main.c"
# 1 "./SPL/MDR32Fx\\MDR32F9Qx_config.h" 1
# 54 "./SPL/MDR32Fx\\MDR32F9Qx_config.h"
# 1 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdint.h" 1 3
# 56 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdint.h" 3
typedef signed char int8_t;
typedef signed short int int16_t;
typedef signed int int32_t;
typedef signed long long int int64_t;


typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long long int uint64_t;





typedef signed char int_least8_t;
typedef signed short int int_least16_t;
typedef signed int int_least32_t;
typedef signed long long int int_least64_t;


typedef unsigned char uint_least8_t;
typedef unsigned short int uint_least16_t;
typedef unsigned int uint_least32_t;
typedef unsigned long long int uint_least64_t;




typedef signed int int_fast8_t;
typedef signed int int_fast16_t;
typedef signed int int_fast32_t;
typedef signed long long int int_fast64_t;


typedef unsigned int uint_fast8_t;
typedef unsigned int uint_fast16_t;
typedef unsigned int uint_fast32_t;
typedef unsigned long long int uint_fast64_t;






typedef signed int intptr_t;
typedef unsigned int uintptr_t;



typedef signed long long intmax_t;
typedef unsigned long long uintmax_t;
# 55 "./SPL/MDR32Fx\\MDR32F9Qx_config.h" 2
# 1 "./RTE/_Target_1\\RTE_Components.h" 1
# 56 "./SPL/MDR32Fx\\MDR32F9Qx_config.h" 2
# 81 "./SPL/MDR32Fx\\MDR32F9Qx_config.h"
# 1 "./CMSIS/MDR32Fx/DeviceSupport/MDR1986VE9x/inc\\MDR32Fx.h" 1
# 30 "./CMSIS/MDR32Fx/DeviceSupport/MDR1986VE9x/inc\\MDR32Fx.h"
typedef enum IRQn
{

  NonMaskableInt_IRQn = -14,
  HardFault_IRQn = -13,
  MemoryManagement_IRQn = -12,
  BusFault_IRQn = -11,
  UsageFault_IRQn = -10,
  SVCall_IRQn = -5,
  PendSV_IRQn = -2,
  SysTick_IRQn = -1,


  CAN1_IRQn = 0,
  CAN2_IRQn = 1,
  USB_IRQn = 2,
  DMA_IRQn = 5,
  UART1_IRQn = 6,
  UART2_IRQn = 7,
  SSP1_IRQn = 8,
  I2C_IRQn = 10,
  POWER_IRQn = 11,
  WWDG_IRQn = 12,
  Timer1_IRQn = 14,
  Timer2_IRQn = 15,
  Timer3_IRQn = 16,
  ADC_IRQn = 17,
  COMPARATOR_IRQn = 19,
  SSP2_IRQn = 20,
  BACKUP_IRQn = 27,
  EXT_INT1_IRQn = 28,
  EXT_INT2_IRQn = 29,
  EXT_INT3_IRQn = 30,
  EXT_INT4_IRQn = 31
}IRQn_Type;
# 79 "./CMSIS/MDR32Fx/DeviceSupport/MDR1986VE9x/inc\\MDR32Fx.h"
# 1 "./SPL/MDR32Fx\\MDR32F9Qx_config.h" 1
# 80 "./CMSIS/MDR32Fx/DeviceSupport/MDR1986VE9x/inc\\MDR32Fx.h" 2
# 1 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cm3.h" 1
# 140 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cm3.h"
# 1 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cmInstr.h" 1
# 325 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cmInstr.h"
__attribute__( ( always_inline ) ) static inline void __NOP(void)
{
  __asm volatile ("nop");
}







__attribute__( ( always_inline ) ) static inline void __WFI(void)
{
  __asm volatile ("wfi");
}







__attribute__( ( always_inline ) ) static inline void __WFE(void)
{
  __asm volatile ("wfe");
}






__attribute__( ( always_inline ) ) static inline void __SEV(void)
{
  __asm volatile ("sev");
}
# 369 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cmInstr.h"
__attribute__( ( always_inline ) ) static inline void __ISB(void)
{
  __asm volatile ("isb");
}







__attribute__( ( always_inline ) ) static inline void __DSB(void)
{
  __asm volatile ("dsb");
}







__attribute__( ( always_inline ) ) static inline void __DMB(void)
{
  __asm volatile ("dmb");
}
# 404 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cmInstr.h"
__attribute__( ( always_inline ) ) static inline uint32_t __REV(uint32_t value)
{



  uint32_t result;

  __asm volatile ("rev %0, %1" : "=r" (result) : "r" (value) );
  return(result);

}
# 424 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cmInstr.h"
__attribute__( ( always_inline ) ) static inline uint32_t __REV16(uint32_t value)
{
  uint32_t result;

  __asm volatile ("rev16 %0, %1" : "=r" (result) : "r" (value) );
  return(result);
}
# 440 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cmInstr.h"
__attribute__( ( always_inline ) ) static inline int32_t __REVSH(int32_t value)
{



  uint32_t result;

  __asm volatile ("revsh %0, %1" : "=r" (result) : "r" (value) );
  return(result);

}
# 461 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cmInstr.h"
__attribute__( ( always_inline ) ) static inline uint32_t __ROR(uint32_t op1, uint32_t op2)
{
  return (op1 >> op2) | (op1 << (32 - op2));
}
# 487 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cmInstr.h"
__attribute__( ( always_inline ) ) static inline uint32_t __RBIT(uint32_t value)
{
  uint32_t result;

   __asm volatile ("rbit %0, %1" : "=r" (result) : "r" (value) );
   return(result);
}
# 503 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cmInstr.h"
__attribute__( ( always_inline ) ) static inline uint8_t __LDREXB(volatile uint8_t *addr)
{
    uint32_t result;







   __asm volatile ("ldrexb %0, [%1]" : "=r" (result) : "r" (addr) : "memory" );

   return ((uint8_t) result);
}
# 526 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cmInstr.h"
__attribute__( ( always_inline ) ) static inline uint16_t __LDREXH(volatile uint16_t *addr)
{
    uint32_t result;







   __asm volatile ("ldrexh %0, [%1]" : "=r" (result) : "r" (addr) : "memory" );

   return ((uint16_t) result);
}
# 549 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cmInstr.h"
__attribute__( ( always_inline ) ) static inline uint32_t __LDREXW(volatile uint32_t *addr)
{
    uint32_t result;

   __asm volatile ("ldrex %0, %1" : "=r" (result) : "Q" (*addr) );
   return(result);
}
# 567 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cmInstr.h"
__attribute__( ( always_inline ) ) static inline uint32_t __STREXB(uint8_t value, volatile uint8_t *addr)
{
   uint32_t result;

   __asm volatile ("strexb %0, %2, %1" : "=&r" (result), "=Q" (*addr) : "r" ((uint32_t)value) );
   return(result);
}
# 585 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cmInstr.h"
__attribute__( ( always_inline ) ) static inline uint32_t __STREXH(uint16_t value, volatile uint16_t *addr)
{
   uint32_t result;

   __asm volatile ("strexh %0, %2, %1" : "=&r" (result), "=Q" (*addr) : "r" ((uint32_t)value) );
   return(result);
}
# 603 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cmInstr.h"
__attribute__( ( always_inline ) ) static inline uint32_t __STREXW(uint32_t value, volatile uint32_t *addr)
{
   uint32_t result;

   __asm volatile ("strex %0, %2, %1" : "=&r" (result), "=Q" (*addr) : "r" (value) );
   return(result);
}







__attribute__( ( always_inline ) ) static inline void __CLREX(void)
{
  __asm volatile ("clrex" ::: "memory");
}
# 662 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cmInstr.h"
__attribute__( ( always_inline ) ) static inline uint8_t __CLZ(uint32_t value)
{
  uint32_t result;

  __asm volatile ("clz %0, %1" : "=r" (result) : "r" (value) );
   return ((uint8_t) result);
}
# 141 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cm3.h" 2
# 1 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cmFunc.h" 1
# 329 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline void __enable_irq(void)
{
  __asm volatile ("cpsie i" : : : "memory");
}







__attribute__( ( always_inline ) ) static inline void __disable_irq(void)
{
  __asm volatile ("cpsid i" : : : "memory");
}
# 352 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline uint32_t __get_CONTROL(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, control" : "=r" (result) );
  return(result);
}
# 367 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline void __set_CONTROL(uint32_t control)
{
  __asm volatile ("MSR control, %0" : : "r" (control) : "memory");
}
# 379 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline uint32_t __get_IPSR(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, ipsr" : "=r" (result) );
  return(result);
}
# 394 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline uint32_t __get_APSR(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, apsr" : "=r" (result) );
  return(result);
}
# 409 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline uint32_t __get_xPSR(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, xpsr" : "=r" (result) );
  return(result);
}
# 424 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline uint32_t __get_PSP(void)
{
  register uint32_t result;

  __asm volatile ("MRS %0, psp\n" : "=r" (result) );
  return(result);
}
# 439 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline void __set_PSP(uint32_t topOfProcStack)
{
  __asm volatile ("MSR psp, %0\n" : : "r" (topOfProcStack) : "sp");
}
# 451 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline uint32_t __get_MSP(void)
{
  register uint32_t result;

  __asm volatile ("MRS %0, msp\n" : "=r" (result) );
  return(result);
}
# 466 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline void __set_MSP(uint32_t topOfMainStack)
{
  __asm volatile ("MSR msp, %0\n" : : "r" (topOfMainStack) : "sp");
}
# 478 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline uint32_t __get_PRIMASK(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, primask" : "=r" (result) );
  return(result);
}
# 493 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline void __set_PRIMASK(uint32_t priMask)
{
  __asm volatile ("MSR primask, %0" : : "r" (priMask) : "memory");
}
# 506 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline void __enable_fault_irq(void)
{
  __asm volatile ("cpsie f" : : : "memory");
}







__attribute__( ( always_inline ) ) static inline void __disable_fault_irq(void)
{
  __asm volatile ("cpsid f" : : : "memory");
}
# 529 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline uint32_t __get_BASEPRI(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, basepri_max" : "=r" (result) );
  return(result);
}
# 544 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline void __set_BASEPRI(uint32_t value)
{
  __asm volatile ("MSR basepri, %0" : : "r" (value) : "memory");
}
# 556 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline uint32_t __get_FAULTMASK(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, faultmask" : "=r" (result) );
  return(result);
}
# 571 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline void __set_FAULTMASK(uint32_t faultMask)
{
  __asm volatile ("MSR faultmask, %0" : : "r" (faultMask) : "memory");
}
# 142 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cm3.h" 2
# 215 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cm3.h"
typedef union
{
  struct
  {

    uint32_t _reserved0:27;





    uint32_t Q:1;
    uint32_t V:1;
    uint32_t C:1;
    uint32_t Z:1;
    uint32_t N:1;
  } b;
  uint32_t w;
} APSR_Type;




typedef union
{
  struct
  {
    uint32_t ISR:9;
    uint32_t _reserved0:23;
  } b;
  uint32_t w;
} IPSR_Type;




typedef union
{
  struct
  {
    uint32_t ISR:9;

    uint32_t _reserved0:15;





    uint32_t T:1;
    uint32_t IT:2;
    uint32_t Q:1;
    uint32_t V:1;
    uint32_t C:1;
    uint32_t Z:1;
    uint32_t N:1;
  } b;
  uint32_t w;
} xPSR_Type;




typedef union
{
  struct
  {
    uint32_t nPRIV:1;
    uint32_t SPSEL:1;
    uint32_t FPCA:1;
    uint32_t _reserved0:29;
  } b;
  uint32_t w;
} CONTROL_Type;
# 300 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cm3.h"
typedef struct
{
  volatile uint32_t ISER[8];
       uint32_t RESERVED0[24];
  volatile uint32_t ICER[8];
       uint32_t RSERVED1[24];
  volatile uint32_t ISPR[8];
       uint32_t RESERVED2[24];
  volatile uint32_t ICPR[8];
       uint32_t RESERVED3[24];
  volatile uint32_t IABR[8];
       uint32_t RESERVED4[56];
  volatile uint8_t IP[240];
       uint32_t RESERVED5[644];
  volatile uint32_t STIR;
} NVIC_Type;
# 332 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cm3.h"
typedef struct
{
  volatile const uint32_t CPUID;
  volatile uint32_t ICSR;
  volatile uint32_t VTOR;
  volatile uint32_t AIRCR;
  volatile uint32_t SCR;
  volatile uint32_t CCR;
  volatile uint8_t SHP[12];
  volatile uint32_t SHCSR;
  volatile uint32_t CFSR;
  volatile uint32_t HFSR;
  volatile uint32_t DFSR;
  volatile uint32_t MMFAR;
  volatile uint32_t BFAR;
  volatile uint32_t AFSR;
  volatile const uint32_t PFR[2];
  volatile const uint32_t DFR;
  volatile const uint32_t ADR;
  volatile const uint32_t MMFR[4];
  volatile const uint32_t ISAR[5];
       uint32_t RESERVED0[5];
  volatile uint32_t CPACR;
} SCB_Type;
# 557 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cm3.h"
typedef struct
{
       uint32_t RESERVED0[1];
  volatile const uint32_t ICTR;



       uint32_t RESERVED1[1];

} SCnSCB_Type;
# 594 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cm3.h"
typedef struct
{
  volatile uint32_t CTRL;
  volatile uint32_t LOAD;
  volatile uint32_t VAL;
  volatile const uint32_t CALIB;
} SysTick_Type;
# 644 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cm3.h"
typedef struct
{
  volatile union
  {
    volatile uint8_t u8;
    volatile uint16_t u16;
    volatile uint32_t u32;
  } PORT [32];
       uint32_t RESERVED0[864];
  volatile uint32_t TER;
       uint32_t RESERVED1[15];
  volatile uint32_t TPR;
       uint32_t RESERVED2[15];
  volatile uint32_t TCR;
       uint32_t RESERVED3[29];
  volatile uint32_t IWR;
  volatile const uint32_t IRR;
  volatile uint32_t IMCR;
       uint32_t RESERVED4[43];
  volatile uint32_t LAR;
  volatile const uint32_t LSR;
       uint32_t RESERVED5[6];
  volatile const uint32_t PID4;
  volatile const uint32_t PID5;
  volatile const uint32_t PID6;
  volatile const uint32_t PID7;
  volatile const uint32_t PID0;
  volatile const uint32_t PID1;
  volatile const uint32_t PID2;
  volatile const uint32_t PID3;
  volatile const uint32_t CID0;
  volatile const uint32_t CID1;
  volatile const uint32_t CID2;
  volatile const uint32_t CID3;
} ITM_Type;
# 745 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cm3.h"
typedef struct
{
  volatile uint32_t CTRL;
  volatile uint32_t CYCCNT;
  volatile uint32_t CPICNT;
  volatile uint32_t EXCCNT;
  volatile uint32_t SLEEPCNT;
  volatile uint32_t LSUCNT;
  volatile uint32_t FOLDCNT;
  volatile const uint32_t PCSR;
  volatile uint32_t COMP0;
  volatile uint32_t MASK0;
  volatile uint32_t FUNCTION0;
       uint32_t RESERVED0[1];
  volatile uint32_t COMP1;
  volatile uint32_t MASK1;
  volatile uint32_t FUNCTION1;
       uint32_t RESERVED1[1];
  volatile uint32_t COMP2;
  volatile uint32_t MASK2;
  volatile uint32_t FUNCTION2;
       uint32_t RESERVED2[1];
  volatile uint32_t COMP3;
  volatile uint32_t MASK3;
  volatile uint32_t FUNCTION3;
} DWT_Type;
# 890 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cm3.h"
typedef struct
{
  volatile uint32_t SSPSR;
  volatile uint32_t CSPSR;
       uint32_t RESERVED0[2];
  volatile uint32_t ACPR;
       uint32_t RESERVED1[55];
  volatile uint32_t SPPR;
       uint32_t RESERVED2[131];
  volatile const uint32_t FFSR;
  volatile uint32_t FFCR;
  volatile const uint32_t FSCR;
       uint32_t RESERVED3[759];
  volatile const uint32_t TRIGGER;
  volatile const uint32_t FIFO0;
  volatile const uint32_t ITATBCTR2;
       uint32_t RESERVED4[1];
  volatile const uint32_t ITATBCTR0;
  volatile const uint32_t FIFO1;
  volatile uint32_t ITCTRL;
       uint32_t RESERVED5[39];
  volatile uint32_t CLAIMSET;
  volatile uint32_t CLAIMCLR;
       uint32_t RESERVED7[8];
  volatile const uint32_t DEVID;
  volatile const uint32_t DEVTYPE;
} TPI_Type;
# 1044 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cm3.h"
typedef struct
{
  volatile const uint32_t TYPE;
  volatile uint32_t CTRL;
  volatile uint32_t RNR;
  volatile uint32_t RBAR;
  volatile uint32_t RASR;
  volatile uint32_t RBAR_A1;
  volatile uint32_t RASR_A1;
  volatile uint32_t RBAR_A2;
  volatile uint32_t RASR_A2;
  volatile uint32_t RBAR_A3;
  volatile uint32_t RASR_A3;
} MPU_Type;
# 1136 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cm3.h"
typedef struct
{
  volatile uint32_t DHCSR;
  volatile uint32_t DCRSR;
  volatile uint32_t DCRDR;
  volatile uint32_t DEMCR;
} CoreDebug_Type;
# 1296 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cm3.h"
static inline void NVIC_SetPriorityGrouping(uint32_t PriorityGroup)
{
  uint32_t reg_value;
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07);

  reg_value = ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR;
  reg_value &= ~((0xFFFFUL << 16) | (7UL << 8));
  reg_value = (reg_value |
                ((uint32_t)0x5FA << 16) |
                (PriorityGroupTmp << 8));
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR = reg_value;
}
# 1316 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cm3.h"
static inline uint32_t NVIC_GetPriorityGrouping(void)
{
  return ((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR & (7UL << 8)) >> 8);
}
# 1328 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cm3.h"
static inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));
}
# 1340 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cm3.h"
static inline void NVIC_DisableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));
}
# 1356 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cm3.h"
static inline uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  return((uint32_t) ((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[(uint32_t)(IRQn) >> 5] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));
}
# 1368 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cm3.h"
static inline void NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));
}
# 1380 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cm3.h"
static inline void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICPR[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));
}
# 1395 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cm3.h"
static inline uint32_t NVIC_GetActive(IRQn_Type IRQn)
{
  return((uint32_t)((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IABR[(uint32_t)(IRQn) >> 5] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));
}
# 1410 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cm3.h"
static inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if(IRQn < 0) {
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[((uint32_t)(IRQn) & 0xF)-4] = ((priority << (8 - 3)) & 0xff); }
  else {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[(uint32_t)(IRQn)] = ((priority << (8 - 3)) & 0xff); }
}
# 1430 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cm3.h"
static inline uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{

  if(IRQn < 0) {
    return((uint32_t)(((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[((uint32_t)(IRQn) & 0xF)-4] >> (8 - 3))); }
  else {
    return((uint32_t)(((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[(uint32_t)(IRQn)] >> (8 - 3))); }
}
# 1452 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cm3.h"
static inline uint32_t NVIC_EncodePriority (uint32_t PriorityGroup, uint32_t PreemptPriority, uint32_t SubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & 0x07);
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7 - PriorityGroupTmp) > 3) ? 3 : 7 - PriorityGroupTmp;
  SubPriorityBits = ((PriorityGroupTmp + 3) < 7) ? 0 : PriorityGroupTmp - 7 + 3;

  return (
           ((PreemptPriority & ((1 << (PreemptPriorityBits)) - 1)) << SubPriorityBits) |
           ((SubPriority & ((1 << (SubPriorityBits )) - 1)))
         );
}
# 1480 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cm3.h"
static inline void NVIC_DecodePriority (uint32_t Priority, uint32_t PriorityGroup, uint32_t* pPreemptPriority, uint32_t* pSubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & 0x07);
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7 - PriorityGroupTmp) > 3) ? 3 : 7 - PriorityGroupTmp;
  SubPriorityBits = ((PriorityGroupTmp + 3) < 7) ? 0 : PriorityGroupTmp - 7 + 3;

  *pPreemptPriority = (Priority >> SubPriorityBits) & ((1 << (PreemptPriorityBits)) - 1);
  *pSubPriority = (Priority ) & ((1 << (SubPriorityBits )) - 1);
}






static inline void NVIC_SystemReset(void)
{
  __DSB();

  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR = ((0x5FA << 16) |
                 (((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR & (7UL << 8)) |
                 (1UL << 2));
  __DSB();
  while(1);
}
# 1537 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cm3.h"
static inline uint32_t SysTick_Config(uint32_t ticks)
{
  if ((ticks - 1) > (0xFFFFFFUL << 0)) return (1);

  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD = ticks - 1;
  NVIC_SetPriority (SysTick_IRQn, (1<<3) - 1);
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL = 0;
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL = (1UL << 2) |
                   (1UL << 1) |
                   (1UL << 0);
  return (0);
}
# 1561 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cm3.h"
extern volatile int32_t ITM_RxBuffer;
# 1575 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cm3.h"
static inline uint32_t ITM_SendChar (uint32_t ch)
{
  if ((((ITM_Type *) (0xE0000000UL) )->TCR & (1UL << 0)) &&
      (((ITM_Type *) (0xE0000000UL) )->TER & (1UL << 0) ) )
  {
    while (((ITM_Type *) (0xE0000000UL) )->PORT[0].u32 == 0);
    ((ITM_Type *) (0xE0000000UL) )->PORT[0].u8 = (uint8_t) ch;
  }
  return (ch);
}
# 1594 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cm3.h"
static inline int32_t ITM_ReceiveChar (void) {
  int32_t ch = -1;

  if (ITM_RxBuffer != 0x5AA55AA5) {
    ch = ITM_RxBuffer;
    ITM_RxBuffer = 0x5AA55AA5;
  }

  return (ch);
}
# 1613 "./CMSIS/MDR32Fx/CoreSupport/CM3\\core_cm3.h"
static inline int32_t ITM_CheckChar (void) {

  if (ITM_RxBuffer == 0x5AA55AA5) {
    return (0);
  } else {
    return (1);
  }
}
# 81 "./CMSIS/MDR32Fx/DeviceSupport/MDR1986VE9x/inc\\MDR32Fx.h" 2
# 1 "./CMSIS/MDR32Fx/DeviceSupport/MDR1986VE9x/startup/arm\\system_MDR32F9Qx.h" 1
# 26 "./CMSIS/MDR32Fx/DeviceSupport/MDR1986VE9x/startup/arm\\system_MDR32F9Qx.h"
extern uint32_t SystemCoreClock;
# 35 "./CMSIS/MDR32Fx/DeviceSupport/MDR1986VE9x/startup/arm\\system_MDR32F9Qx.h"
extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);
# 82 "./CMSIS/MDR32Fx/DeviceSupport/MDR1986VE9x/inc\\MDR32Fx.h" 2





typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus, BitStatus;



typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;


typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;
# 111 "./CMSIS/MDR32Fx/DeviceSupport/MDR1986VE9x/inc\\MDR32Fx.h"
typedef struct
{
  volatile uint32_t ID;
  volatile uint32_t DLC;
  volatile uint32_t DATAL;
  volatile uint32_t DATAH;
}MDR_CAN_BUF_TypeDef;


typedef struct
{
  volatile uint32_t MASK;
  volatile uint32_t FILTER;
}MDR_CAN_BUF_FILTER_TypeDef;


typedef struct
{
  volatile uint32_t CONTROL;
  volatile uint32_t STATUS;
  volatile uint32_t BITTMNG;
       uint32_t RESERVED0;
  volatile uint32_t INT_EN;
       uint32_t RESERVED1[2];
  volatile uint32_t OVER;
  volatile uint32_t RXID;
  volatile uint32_t RXDLC;
  volatile uint32_t RXDATAL;
  volatile uint32_t RXDATAH;
  volatile uint32_t TXID;
  volatile uint32_t TXDLC;
  volatile uint32_t DATAL;
  volatile uint32_t DATAH;
  volatile uint32_t BUF_CON[32];
  volatile uint32_t INT_RX;
  volatile uint32_t RX;
  volatile uint32_t INT_TX;
  volatile uint32_t TX;
       uint32_t RESERVED2[76];
    MDR_CAN_BUF_TypeDef CAN_BUF[32];
       uint32_t RESERVED3[64];
    MDR_CAN_BUF_FILTER_TypeDef CAN_BUF_FILTER[32];
}MDR_CAN_TypeDef;
# 389 "./CMSIS/MDR32Fx/DeviceSupport/MDR1986VE9x/inc\\MDR32Fx.h"
typedef struct
{
  volatile uint32_t CTRL;
  volatile uint32_t STS;
  volatile uint32_t TS;
  volatile uint32_t NTS;
}MDR_USB_SEP_TypeDef;


typedef struct
{
  volatile uint32_t RXFD;
       uint32_t RESERVED0;
  volatile uint32_t RXFDC_L;
  volatile uint32_t RXFDC_H;
  volatile uint32_t RXFC;
       uint32_t RESERVED1[11];
  volatile uint32_t TXFD;
       uint32_t RESERVED2[3];
  volatile uint32_t TXFDC;
       uint32_t RESERVED3[11];
}MDR_USB_SEP_FIFO_TypeDef;


typedef struct
{
  volatile uint32_t HTXC;
  volatile uint32_t HTXT;
  volatile uint32_t HTXLC;
  volatile uint32_t HTXSE;
  volatile uint32_t HTXA;
  volatile uint32_t HTXE;
  volatile uint32_t HFN_L;
  volatile uint32_t HFN_H;
  volatile uint32_t HIS;
  volatile uint32_t HIM;
  volatile uint32_t HRXS;
  volatile uint32_t HRXP;
  volatile uint32_t HRXA;
  volatile uint32_t HRXE;
  volatile uint32_t HRXCS;
  volatile uint32_t HSTM;
       uint32_t RESERVED0[16];
  volatile uint32_t HRXFD;
       uint32_t RESERVED1;
  volatile uint32_t HRXFDC_L;
  volatile uint32_t HRXFDC_H;
  volatile uint32_t HRXFC;
       uint32_t RESERVED2[11];
  volatile uint32_t HTXFD;
       uint32_t RESERVED3[3];
  volatile uint32_t HTXFC;
       uint32_t RESERVED4[11];
    MDR_USB_SEP_TypeDef USB_SEP[4];
  volatile uint32_t SC;
  volatile uint32_t SLS;
  volatile uint32_t SIS;
  volatile uint32_t SIM;
  volatile uint32_t SA;
  volatile uint32_t SFN_L;
  volatile uint32_t SFN_H;
       uint32_t RESERVED5[9];
    MDR_USB_SEP_FIFO_TypeDef USB_SEP_FIFO[4];
  volatile uint32_t HSCR;
  volatile uint32_t HSVR;
}MDR_USB_TypeDef;
# 731 "./CMSIS/MDR32Fx/DeviceSupport/MDR1986VE9x/inc\\MDR32Fx.h"
typedef struct
{
  volatile uint32_t CMD;
  volatile uint32_t ADR;
  volatile uint32_t DI;
  volatile uint32_t DO;
  volatile uint32_t KEY;
}MDR_EEPROM_TypeDef;
# 790 "./CMSIS/MDR32Fx/DeviceSupport/MDR1986VE9x/inc\\MDR32Fx.h"
typedef struct
{
  volatile uint32_t CLOCK_STATUS;
  volatile uint32_t PLL_CONTROL;
  volatile uint32_t HS_CONTROL;
  volatile uint32_t CPU_CLOCK;
  volatile uint32_t USB_CLOCK;
  volatile uint32_t ADC_MCO_CLOCK;
  volatile uint32_t RTC_CLOCK;
  volatile uint32_t PER_CLOCK;
  volatile uint32_t CAN_CLOCK;
  volatile uint32_t TIM_CLOCK;
  volatile uint32_t UART_CLOCK;
  volatile uint32_t SSP_CLOCK;
}MDR_RST_CLK_TypeDef;
# 1046 "./CMSIS/MDR32Fx/DeviceSupport/MDR1986VE9x/inc\\MDR32Fx.h"
typedef struct
{
  volatile uint32_t STATUS;
  volatile uint32_t CFG;
  volatile uint32_t CTRL_BASE_PTR;
  volatile uint32_t ALT_CTRL_BASE_PTR;
  volatile uint32_t WAITONREQ_STATUS;
  volatile uint32_t CHNL_SW_REQUEST;
  volatile uint32_t CHNL_USEBURST_SET;
  volatile uint32_t CHNL_USEBURST_CLR;
  volatile uint32_t CHNL_REQ_MASK_SET;
  volatile uint32_t CHNL_REQ_MASK_CLR;
  volatile uint32_t CHNL_ENABLE_SET;
  volatile uint32_t CHNL_ENABLE_CLR;
  volatile uint32_t CHNL_PRI_ALT_SET;
  volatile uint32_t CHNL_PRI_ALT_CLR;
  volatile uint32_t CHNL_PRIORITY_SET;
  volatile uint32_t CHNL_PRIORITY_CLR;
       uint32_t RESERVED0[3];
  volatile uint32_t ERR_CLR;
}MDR_DMA_TypeDef;
# 1121 "./CMSIS/MDR32Fx/DeviceSupport/MDR1986VE9x/inc\\MDR32Fx.h"
typedef struct
{
  volatile uint32_t DR;
  volatile uint32_t RSR_ECR;
       uint32_t RESERVED0[4];
  volatile uint32_t FR;
       uint32_t RESERVED1;
  volatile uint32_t ILPR;
  volatile uint32_t IBRD;
  volatile uint32_t FBRD;
  volatile uint32_t LCR_H;
  volatile uint32_t CR;
  volatile uint32_t IFLS;
  volatile uint32_t IMSC;
  volatile uint32_t RIS;
  volatile uint32_t MIS;
  volatile uint32_t ICR;
  volatile uint32_t DMACR;
}MDR_UART_TypeDef;
# 1453 "./CMSIS/MDR32Fx/DeviceSupport/MDR1986VE9x/inc\\MDR32Fx.h"
typedef struct
{
  volatile uint32_t CR0;
  volatile uint32_t CR1;
  volatile uint32_t DR;
  volatile uint32_t SR;
  volatile uint32_t CPSR;
  volatile uint32_t IMSC;
  volatile uint32_t RIS;
  volatile uint32_t MIS;
  volatile uint32_t ICR;
  volatile uint32_t DMACR;
}MDR_SSP_TypeDef;
# 1634 "./CMSIS/MDR32Fx/DeviceSupport/MDR1986VE9x/inc\\MDR32Fx.h"
typedef struct
{
  volatile uint32_t PRL;
  volatile uint32_t PRH;
  volatile uint32_t CTR;
  volatile uint32_t RXD;
  volatile uint32_t STA;
  volatile uint32_t TXD;
  volatile uint32_t CMD;
}MDR_I2C_TypeDef;
# 1725 "./CMSIS/MDR32Fx/DeviceSupport/MDR1986VE9x/inc\\MDR32Fx.h"
typedef struct
{
  volatile uint32_t PVDCS;
}MDR_POWER_TypeDef;
# 1778 "./CMSIS/MDR32Fx/DeviceSupport/MDR1986VE9x/inc\\MDR32Fx.h"
typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t CFR;
  volatile uint32_t SR;
}MDR_WWDG_TypeDef;
# 1836 "./CMSIS/MDR32Fx/DeviceSupport/MDR1986VE9x/inc\\MDR32Fx.h"
typedef struct
{
  volatile uint32_t KR;
  volatile uint32_t PR;
  volatile uint32_t RLR;
  volatile uint32_t SR;
}MDR_IWDG_TypeDef;
# 1878 "./CMSIS/MDR32Fx/DeviceSupport/MDR1986VE9x/inc\\MDR32Fx.h"
typedef struct
{
  volatile uint32_t CNT;
  volatile uint32_t PSG;
  volatile uint32_t ARR;
  volatile uint32_t CNTRL;
  volatile uint32_t CCR1;
  volatile uint32_t CCR2;
  volatile uint32_t CCR3;
  volatile uint32_t CCR4;
  volatile uint32_t CH1_CNTRL;
  volatile uint32_t CH2_CNTRL;
  volatile uint32_t CH3_CNTRL;
  volatile uint32_t CH4_CNTRL;
  volatile uint32_t CH1_CNTRL1;
  volatile uint32_t CH2_CNTRL1;
  volatile uint32_t CH3_CNTRL1;
  volatile uint32_t CH4_CNTRL1;
  volatile uint32_t CH1_DTG;
  volatile uint32_t CH2_DTG;
  volatile uint32_t CH3_DTG;
  volatile uint32_t CH4_DTG;
  volatile uint32_t BRKETR_CNTRL;
  volatile uint32_t STATUS;
  volatile uint32_t IE;
  volatile uint32_t DMA_RE;
  volatile uint32_t CH1_CNTRL2;
  volatile uint32_t CH2_CNTRL2;
  volatile uint32_t CH3_CNTRL2;
  volatile uint32_t CH4_CNTRL2;
  volatile uint32_t CCR11;
  volatile uint32_t CCR21;
  volatile uint32_t CCR31;
  volatile uint32_t CCR41;
}MDR_TIMER_TypeDef;
# 2144 "./CMSIS/MDR32Fx/DeviceSupport/MDR1986VE9x/inc\\MDR32Fx.h"
typedef struct
{
  volatile uint32_t ADC1_CFG;
  volatile uint32_t ADC2_CFG;
  volatile uint32_t ADC1_H_LEVEL;
  volatile uint32_t ADC2_H_LEVEL;
  volatile uint32_t ADC1_L_LEVEL;
  volatile uint32_t ADC2_L_LEVEL;
  volatile uint32_t ADC1_RESULT;
  volatile uint32_t ADC2_RESULT;
  volatile uint32_t ADC1_STATUS;
  volatile uint32_t ADC2_STATUS;
  volatile uint32_t ADC1_CHSEL;
  volatile uint32_t ADC2_CHSEL;
}MDR_ADC_TypeDef;
# 2297 "./CMSIS/MDR32Fx/DeviceSupport/MDR1986VE9x/inc\\MDR32Fx.h"
typedef struct
{
  volatile uint32_t CFG;
  volatile uint32_t DAC1_DATA;
  volatile uint32_t DAC2_DATA;
}MDR_DAC_TypeDef;
# 2374 "./CMSIS/MDR32Fx/DeviceSupport/MDR1986VE9x/inc\\MDR32Fx.h"
typedef struct
{
  volatile uint32_t CFG;
  volatile uint32_t RESULT;
  volatile uint32_t RESULT_LATCH;
}MDR_COMP_TypeDef;
# 2448 "./CMSIS/MDR32Fx/DeviceSupport/MDR1986VE9x/inc\\MDR32Fx.h"
typedef struct
{
  volatile uint32_t RXTX;
  volatile uint32_t OE;
  volatile uint32_t FUNC;
  volatile uint32_t ANALOG;
  volatile uint32_t PULL;
  volatile uint32_t PD;
  volatile uint32_t PWR;
  volatile uint32_t GFEN;
}MDR_PORT_TypeDef;
# 2595 "./CMSIS/MDR32Fx/DeviceSupport/MDR1986VE9x/inc\\MDR32Fx.h"
typedef struct
{
  volatile uint32_t REG_00;
  volatile uint32_t REG_01;
  volatile uint32_t REG_02;
  volatile uint32_t REG_03;
  volatile uint32_t REG_04;
  volatile uint32_t REG_05;
  volatile uint32_t REG_06;
  volatile uint32_t REG_07;
  volatile uint32_t REG_08;
  volatile uint32_t REG_09;
  volatile uint32_t REG_0A;
  volatile uint32_t REG_0B;
  volatile uint32_t REG_0C;
  volatile uint32_t REG_0D;
  volatile uint32_t REG_0E;
  volatile uint32_t REG_0F;
  volatile uint32_t RTC_CNT;
  volatile uint32_t RTC_DIV;
  volatile uint32_t RTC_PRL;
  volatile uint32_t RTC_ALRM;
  volatile uint32_t RTC_CS;
}MDR_BKP_TypeDef;
# 2732 "./CMSIS/MDR32Fx/DeviceSupport/MDR1986VE9x/inc\\MDR32Fx.h"
typedef struct
{
       uint32_t RESERVED0[20];
  volatile uint32_t NAND_CYCLES;
  volatile uint32_t CONTROL;



}MDR_EBC_TypeDef;
# 82 "./SPL/MDR32Fx\\MDR32F9Qx_config.h" 2
# 11 "main.c" 2
# 1 "./MDR32F9Qx_usb_handlers.h" 1
# 29 "./MDR32F9Qx_usb_handlers.h"
# 1 "./SPL/MDR32Fx/inc/USB_Library\\MDR32F9Qx_usb_default_handlers.h" 1
# 32 "./SPL/MDR32Fx/inc/USB_Library\\MDR32F9Qx_usb_default_handlers.h"
# 1 "./SPL/MDR32Fx/inc/USB_Library/MDR32F9Qx_usb_CDC.h" 1
# 33 "./SPL/MDR32Fx/inc/USB_Library/MDR32F9Qx_usb_CDC.h"
# 1 "./SPL/MDR32Fx/inc\\USB_Library/MDR32F9Qx_usb_device.h" 1
# 33 "./SPL/MDR32Fx/inc\\USB_Library/MDR32F9Qx_usb_device.h"
# 1 "./SPL/MDR32Fx/inc\\MDR32F9Qx_usb.h" 1
# 49 "./SPL/MDR32Fx/inc\\MDR32F9Qx_usb.h"
typedef enum
{
    USB_EP0 = 0,
    USB_EP1 = 1,
    USB_EP2 = 2,
    USB_EP3 = 3,
    Num_USB_EndPoints
} USB_EP_TypeDef;






typedef enum
{
    USB_C1HSIdiv1 = ((uint32_t)0x00),
    USB_C1HSIdiv2 = ((uint32_t)0x01),
    USB_C1HSEdiv1 = ((uint32_t)0x02),
    USB_C1HSEdiv2 = ((uint32_t)0x03)
} USB_C1_Source_TypeDef;






typedef enum
{
    USB_PLLUSBMUL1 = ((uint32_t)0x00),
    USB_PLLUSBMUL2 = ((uint32_t)0x01),
    USB_PLLUSBMUL3 = ((uint32_t)0x02),
    USB_PLLUSBMUL4 = ((uint32_t)0x03),
    USB_PLLUSBMUL5 = ((uint32_t)0x04),
    USB_PLLUSBMUL6 = ((uint32_t)0x05),
    USB_PLLUSBMUL7 = ((uint32_t)0x06),
    USB_PLLUSBMUL8 = ((uint32_t)0x07),
    USB_PLLUSBMUL9 = ((uint32_t)0x08),
    USB_PLLUSBMUL10 = ((uint32_t)0x09),
    USB_PLLUSBMUL11 = ((uint32_t)0x0A),
    USB_PLLUSBMUL12 = ((uint32_t)0x0B),
    USB_PLLUSBMUL13 = ((uint32_t)0x0C),
    USB_PLLUSBMUL14 = ((uint32_t)0x0D),
    USB_PLLUSBMUL15 = ((uint32_t)0x0E),
    USB_PLLUSBMUL16 = ((uint32_t)0x0F)
} USB_PLL_Source_TypeDef;







typedef struct
{
    USB_C1_Source_TypeDef USB_USBC1_Source;

    USB_PLL_Source_TypeDef USB_PLLUSBMUL;

} USB_Clock_TypeDef;




typedef struct
{
    uint8_t USB_Version;
    uint8_t USB_Revision;
} USB_Version_TypeDef;
# 637 "./SPL/MDR32Fx/inc\\MDR32F9Qx_usb.h"
void USB_BRGInit(const USB_Clock_TypeDef* USB_Clock_InitStruct);
void USB_Reset(void);





uint32_t USB_GetHSCR(void);
void USB_SetHSCR(uint32_t RegValue);

USB_Version_TypeDef USB_GetHSVR(void);





uint32_t USB_GetHTXC(void);
void USB_SetHTXC(uint32_t RegValue);
uint32_t USB_GetHTXT(void);
void USB_SetHTXT(uint32_t RegValue);
uint32_t USB_GetHTXLC(void);
void USB_SetHTXLC(uint32_t RegValue);
uint32_t USB_GetHTXSE(void);
void USB_SetHTXSE(uint32_t RegValue);
uint32_t USB_GetHTXA(void);
void USB_SetHTXA(uint32_t RegValue);
uint32_t USB_GetHTXE(void);
void USB_SetHTXE(uint32_t RegValue);
uint32_t USB_GetHFN(void);
uint32_t USB_GetHIS(void);
void USB_SetHIS(uint32_t RegValue);
uint32_t USB_GetHIM(void);
void USB_SetHIM(uint32_t RegValue);
uint32_t USB_GetHRXS(void);
uint32_t USB_GetHRXP(void);
uint32_t USB_GetHRXA(void);
uint32_t USB_GetHRXE(void);
uint32_t USB_GetHRXCS(void);
uint32_t USB_GetHSTM(void);
uint32_t USB_GetHRXFD(void);
uint32_t USB_GetHRXFDC(void);
uint32_t USB_GetHRXFC(void);
void USB_SetHRXFC(uint32_t RegValue);
uint32_t USB_GetHTXFD(void);
void USB_SetHTXFD(uint32_t RegValue);
uint32_t USB_GetHTXFC(void);
void USB_SetHTXFC(uint32_t RegValue);





uint32_t USB_GetSEPxCTRL(USB_EP_TypeDef EndPointNumber);
void USB_SetSEPxCTRL(USB_EP_TypeDef EndPointNumber, uint32_t RegValue);
uint32_t USB_GetSEPxSTS(USB_EP_TypeDef EndPointNumber);
uint32_t USB_GetSEPxTS(USB_EP_TypeDef EndPointNumber);
uint32_t USB_GetSEPxNTS(USB_EP_TypeDef EndPointNumber);
uint32_t USB_GetSC(void);
void USB_SetSC(uint32_t RegValue);
uint32_t USB_GetSLS(void);
uint32_t USB_GetSIS(void);
void USB_SetSIS(uint32_t RegValue);
uint32_t USB_GetSIM(void);
void USB_SetSIM(uint32_t RegValue);
uint32_t USB_GetSA(void);
void USB_SetSA(uint32_t RegValue);
uint32_t USB_GetSFN(void);
uint32_t USB_GetSEPxRXFD(USB_EP_TypeDef EndPointNumber);
uint32_t USB_GetSEPxRXFDC(USB_EP_TypeDef EndPointNumber);
uint32_t USB_GetSEPxRXFC(USB_EP_TypeDef EndPointNumber);
void USB_SetSEPxRXFC(USB_EP_TypeDef EndPointNumber, uint32_t RegValue);
uint32_t USB_GetSEPxTXFD(USB_EP_TypeDef EndPointNumber);
void USB_SetSEPxTXFD(USB_EP_TypeDef EndPointNumber, uint32_t RegValue);
uint32_t USB_GetSEPxTXFDC(USB_EP_TypeDef EndPointNumber);
void USB_SetSEPxTXFDC(USB_EP_TypeDef EndPointNumber, uint32_t RegValue);
void USB_SEPxToggleEPDATASEQ(USB_EP_TypeDef EndPointNumber);
# 34 "./SPL/MDR32Fx/inc\\USB_Library/MDR32F9Qx_usb_device.h" 2
# 50 "./SPL/MDR32Fx/inc\\USB_Library/MDR32F9Qx_usb_device.h"
typedef enum
{
    USB_HOST_TO_DEVICE = 0x00,
    USB_DEVICE_TO_HOST = 0x80
}USB_RequestTypeDT_TypeDef;




typedef enum
{
    USB_TYPE_STANDARD = 0x00,
    USB_TYPE_CLASS = 0x20,
    USB_TYPE_VENDOR = 0x40
} USB_RequestType_TypeDef;







typedef enum
{
    USB_GET_STATUS = 0,
    USB_CLEAR_FEATURE,
    USB_Reserved0,
    USB_SET_FEATURE,
    USB_Reserved1,
    USB_SET_ADDRESS,
    USB_GET_DESCRIPTOR,
    USB_SET_DESCRIPTOR,
    USB_GET_CONFIGURATION,
    USB_SET_CONFIGURATION,
    USB_GET_INTERFACE,
    USB_SET_INTERFACE,
    USB_SYNCH_FRAME
} USB_Standard_Setup_TypeDef;




typedef enum
{
    USB_DEVICE = 1,
    USB_CONFIGURATION,
    USB_STRING,
    USB_INTERFACE,
    USB_ENDPOINT,
    USB_DEVICE_QUALIFIER,
    USB_OTHER_SPEED_CONFIGURATION,
    USB_INTERFACE_POWER
} USB_Standard_Descriptor_TypeDef;




typedef enum
{
    USB_ENDPOINT_HALT = 0,
    USB_DEVICE_REMOTE_WAKEUP,
    USB_TEST_MODE
} USB_Standard_Festure_Selector_TypeDef;




typedef enum
{
    USB_RECIPIENT_DEVICE = 0x00,
    USB_RECIPIENT_INTERFACE = 0x01,
    USB_RECIPIENT_ENDPOINT = 0x02,
    USB_RECIPIENT_OTHER = 0x03
} USB_RequestRecipient_TypeDef;
# 132 "./SPL/MDR32Fx/inc\\USB_Library/MDR32F9Qx_usb_device.h"
typedef struct
{
    uint8_t mRequestTypeData;



    uint8_t bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
} USB_SetupPacket_TypeDef;
# 157 "./SPL/MDR32Fx/inc\\USB_Library/MDR32F9Qx_usb_device.h"
typedef enum
{
    USB_SUCCESS = 0x0000,
    USB_ERROR = 0x0001,
    USB_ERR_INV_REQ = 0x0002,
    USB_ERR_BUSY = 0x0200,
} USB_Result;




typedef enum {USB_STALL_PROTO = 0x0, USB_STALL_HALT = 0x1} USB_StallType;





typedef USB_Result (*USB_EP_IO_Handler)(USB_EP_TypeDef EPx, uint8_t* Buffer, uint32_t Length);
typedef USB_Result (*USB_EP_Setup_Handler)(USB_EP_TypeDef EPx, const USB_SetupPacket_TypeDef* USB_SetupPacket);
typedef USB_Result (*USB_EP_Error_Handler)(USB_EP_TypeDef EPx, uint32_t STS, uint32_t TS, uint32_t CTRL);
# 200 "./SPL/MDR32Fx/inc\\USB_Library/MDR32F9Qx_usb_device.h"
USB_Result USB_EP_Init(USB_EP_TypeDef EPx, uint32_t USB_EP_Ctrl, USB_EP_Error_Handler onError);
USB_Result USB_EP_Reset(USB_EP_TypeDef EPx);
USB_Result USB_EP_Idle(USB_EP_TypeDef EPx);
USB_Result USB_EP_Stall(USB_EP_TypeDef EPx, USB_StallType bHalt);

USB_Result USB_EP_doDataIn(USB_EP_TypeDef EPx, uint8_t* Buffer, uint32_t Length, USB_EP_IO_Handler onInDone);
USB_Result USB_EP_doDataOut(USB_EP_TypeDef EPx, uint8_t* Buffer, uint32_t Length, USB_EP_IO_Handler onOutDone);

USB_Result USB_EP_setSetupHandler(USB_EP_TypeDef EPx, USB_SetupPacket_TypeDef* USB_SetupPacket, USB_EP_Setup_Handler onSetupPacket);

USB_Result USB_EP_dispatchEvent(USB_EP_TypeDef EPx, uint32_t USB_IT);
# 227 "./SPL/MDR32Fx/inc\\USB_Library/MDR32F9Qx_usb_device.h"
typedef enum
{
    USB_DEV_STATE_UNKNOWN = 0,
    USB_DEV_STATE_ATTACHED,
    USB_DEV_STATE_POWERED,
    USB_DEV_STATE_DEFAULT,
    USB_DEV_STATE_ADDRESS,
    USB_DEV_STATE_CONFIGURED,
    Num_USB_DEV_STATE
} USB_DeviceState_TypeDef;




typedef enum
{
    USB_DEV_SELF_POWERED_OFF = 0,
    USB_DEV_SELF_POWERED_ON = 1
} USB_DeviceSelfPoweredState_TypeDef;




typedef enum
{
    USB_DEV_REMOTE_WAKEUP_DISABLED = 0,
    USB_DEV_REMOTE_WAKEUP_ENABLED = 1
} USB_DeviceRemoteWakeup_TypeDef;




typedef struct
{







    uint32_t Reserved;

} Usb_DeviceStatus_TypeDef;




typedef struct
{
    USB_DeviceState_TypeDef USB_DeviceState;
    Usb_DeviceStatus_TypeDef USB_DeviceStatus;
    uint32_t Address;
} USB_DeviceContext_TypeDef;




typedef struct {
    uint32_t PULL;





    uint32_t SPEED;



    uint32_t MODE;



} USB_DeviceBUSParam_TypeDef;
# 323 "./SPL/MDR32Fx/inc\\USB_Library/MDR32F9Qx_usb_device.h"
extern USB_SetupPacket_TypeDef USB_CurrentSetupPacket;




extern USB_DeviceContext_TypeDef USB_DeviceContext;
# 337 "./SPL/MDR32Fx/inc\\USB_Library/MDR32F9Qx_usb_device.h"
USB_Result USB_DeviceInit(const USB_Clock_TypeDef* USB_Clock_InitStruct, USB_DeviceBUSParam_TypeDef* USB_DeviceBUSParam);
USB_Result USB_DevicePowerOn(void);
USB_Result USB_DevicePowerOff(void);




USB_Result USB_DeviceReset(void);
USB_Result USB_DeviceSuspend(void);
USB_Result USB_DeviceResume(void);

USB_Result USB_DeviceSetupPacket(USB_EP_TypeDef EPx, const USB_SetupPacket_TypeDef* USB_SetupPacket);

USB_Result USB_DeviceClearFeature(USB_RequestRecipient_TypeDef Recipient, uint16_t wVALUE, uint16_t wINDEX);
USB_Result USB_DeviceSetFeature(USB_RequestRecipient_TypeDef Recipient, uint16_t wVALUE, uint16_t wINDEX);

USB_Result USB_DeviceDoStatusInAck(USB_EP_TypeDef EPx, uint8_t* Buffer, uint32_t Length);
USB_Result USB_DeviceDoStatusOutAck(USB_EP_TypeDef EPx, uint8_t* Buffer, uint32_t Length);

USB_Result USB_DeviceDispatchEvent(void);


    void USB_IRQHandler(void);






USB_Result USB_DeviceDummyGetStatus(USB_RequestRecipient_TypeDef Recipient, uint16_t wINDEX);
USB_Result USB_DeviceDummySetAddress(uint16_t wVALUE);
USB_Result USB_DeviceDummyGetDescriptor(uint16_t wVALUE, uint16_t wINDEX, uint16_t wLENGTH);
USB_Result USB_DeviceDummySetDescriptor(uint16_t wVALUE, uint16_t wINDEX, uint16_t wLENGTH);
uint8_t USB_DeviceDummyGetConfiguration(void);
USB_Result USB_DeviceDummySetConfiguration(uint16_t wVALUE);
uint8_t USB_DeviceDummyGetInterface(uint16_t wINDEX);
USB_Result USB_DeviceDummySetInterface(uint16_t wVALUE, uint16_t wINDEX);
USB_Result USB_DeviceDummySyncFrame(uint16_t wINDEX, uint8_t* DATA);
USB_Result USB_DeviceDummyClassRequest(void);
USB_Result USB_DeviceDummyVendorRequest(void);
USB_Result USB_DeviceDummyDataError(USB_EP_TypeDef EPx, uint32_t STS, uint32_t TS, uint32_t CTRL);
# 34 "./SPL/MDR32Fx/inc/USB_Library/MDR32F9Qx_usb_CDC.h" 2
# 54 "./SPL/MDR32Fx/inc/USB_Library/MDR32F9Qx_usb_CDC.h"
typedef enum
{
    USB_CDC_SEND_ENCAPSULATED_COMMAND = 0x00,
    USB_CDC_GET_ENCAPSULATED_RESPONSE,
    USB_CDC_SET_COMM_FEATURE,
    USB_CDC_GET_COMM_FEATURE,
    USB_CDC_CLEAR_COMM_FEATURE,
    USB_CDC_SET_AUX_LINE_STATE = 0x10,
    USB_CDC_SET_HOOK_STATE,
    USB_CDC_PULSE_SETUP,
    USB_CDC_SEND_PULSE,
    USB_CDC_SET_PULSE_TIME,
    USB_CDC_RING_AUX_JACK,
    USB_CDC_SET_LINE_CODING = 0x20,
    USB_CDC_GET_LINE_CODING,
    USB_CDC_SET_CONTROL_LINE_STATE,
    USB_CDC_SEND_BREAK,
    USB_CDC_SET_RINGER_PARAMS = 0x30,
    USB_CDC_GET_RINGER_PARAMS,
    USB_CDC_SET_OPERATION_PARAMS,
    USB_CDC_GET_OPERATION_PARAMS,
    USB_CDC_SET_LINE_PARAMS,
    USB_CDC_GET_LINE_PARAMS,
    USB_CDC_DIAL_DIGITS
} USB_CDC_Class_Setup_TypeDef;





typedef enum
{
    USB_CDC_bRxCarrier = 0x01,
    USB_CDC_bTxCarrier = 0x02,
    USB_CDC_bBreak = 0x04,
    USB_CDC_bRingSignal = 0x08,
    USB_CDC_bFraming = 0x10,
    USB_CDC_bParity = 0x20,
    USB_CDC_bOverRun = 0x40
} USB_CDCSerialState_TypeDef;




typedef enum
{
    USB_CDC_STOP_BITS1 = 0x00,
    USB_CDC_STOP_BITS1_5 = 0x01,
    USB_CDC_STOP_BITS2 = 0x02,
} USB_CDC_CharFormat_TypeDef;




typedef enum
{
    USB_CDC_PARITY_NONE = 0x00,
    USB_CDC_PARITY_ODD = 0x01,
    USB_CDC_PARITY_EVEN = 0x02,
    USB_CDC_PARITY_MARK = 0x03,
    USB_CDC_PARITY_SPACE = 0x04
} USB_CDC_ParityType_TypeDef;




typedef enum
{
    USB_CDC_DATA_BITS5 = 0x05,
    USB_CDC_DATA_BITS6 = 0x06,
    USB_CDC_DATA_BITS7 = 0x07,
    USB_CDC_DATA_BITS8 = 0x08,
    USB_CDC_DATA_BITS16 = 0x0A
} USB_CDC_DataBits_TypeDef;




typedef struct
{
    uint32_t dwDTERate;
    uint8_t bCharFormat;
    uint8_t bParityType;
    uint8_t bDataBits;
} USB_CDC_LineCoding_TypeDef;




typedef enum
{
    USB_CDC_DTR_PRESENT = 0x01,
    USB_CDC_RTS_ACTIVATE_CARRIER = 0x02
} USB_CDC_ControlLineState_TypeDef;





typedef enum
{
    USB_CDC_RING_DETECT = 0x09,
    USB_CDC_SERIAL_STATE = 0x20,
    USB_CDC_CALL_STATE_CHANGE = 0x28,
    USB_CDC_LINE_STATE_CHANGE = 0x29,
    USB_CDC_CONNECTION_SPEED_CHANGE = 0x2A
} USB_CDC_LineStateReport_TypeDef;
# 186 "./SPL/MDR32Fx/inc/USB_Library/MDR32F9Qx_usb_CDC.h"
USB_Result USB_CDC_Init(uint8_t* ReceiveBuffer, uint32_t DataPortionLength, FlagStatus StartReceiving);

USB_Result USB_CDC_SetReceiveBuffer(uint8_t* ReceiveBuffer, uint32_t DataPortionLength);
USB_Result USB_CDC_ReceiveStart(void);
USB_Result USB_CDC_ReceiveStop(void);

USB_Result USB_CDC_SendData(uint8_t* Buffer, uint32_t Length);
int check_status_cdc(void);



USB_Result USB_CDC_ReportState(uint16_t LineState);






USB_Result USB_CDC_Reset(void);
USB_Result USB_CDC_GetDescriptor(uint16_t wVALUE, uint16_t wINDEX, uint16_t wLENGTH);
USB_Result USB_CDC_ClassRequest(void);







USB_Result USB_CDC_DummyDataReceive(uint8_t* Buffer, uint32_t Length);
USB_Result USB_CDC_DummyDataSent(void);


USB_Result USB_CDC_DummySendEncapsulatedCMD(uint16_t wINDEX, uint16_t wLENGTH);
USB_Result USB_CDC_DummyGetEncapsulatedResp(uint16_t wINDEX, uint16_t wLENGTH);



USB_Result USB_CDC_DummyGetCommFeature(uint16_t wVALUE, uint16_t wINDEX, uint16_t* DATA);
USB_Result USB_CDC_DummySetCommFeature(uint16_t wVALUE, uint16_t wINDEX, uint16_t DATA);
USB_Result USB_CDC_DummyClearCommFeature(uint16_t wVALUE, uint16_t wINDEX);



USB_Result USB_CDC_DummyGetLineCoding(uint16_t wINDEX, USB_CDC_LineCoding_TypeDef* DATA);
USB_Result USB_CDC_DummySetLineCoding(uint16_t wINDEX, const USB_CDC_LineCoding_TypeDef* DATA);



USB_Result USB_CDC_DummyControlLineState(uint16_t wVALUE, uint16_t wINDEX);



USB_Result USB_CDC_DummySendBreak(uint16_t wVALUE, uint16_t wINDEX);
# 33 "./SPL/MDR32Fx/inc/USB_Library\\MDR32F9Qx_usb_default_handlers.h" 2
# 30 "./MDR32F9Qx_usb_handlers.h" 2
# 85 "./MDR32F9Qx_usb_handlers.h"
USB_Result USB_CDC_RecieveData(uint8_t* Buffer, uint32_t Length);


    USB_Result USB_CDC_GetLineCoding(uint16_t wINDEX, USB_CDC_LineCoding_TypeDef* DATA);
    USB_Result USB_CDC_SetLineCoding(uint16_t wINDEX, const USB_CDC_LineCoding_TypeDef* DATA);
# 12 "main.c" 2
# 1 "./SPL/MDR32Fx/inc\\MDR32F9Qx_rst_clk.h" 1
# 49 "./SPL/MDR32Fx/inc\\MDR32F9Qx_rst_clk.h"
typedef struct
{
    uint32_t CPU_CLK_Frequency;
    uint32_t USB_CLK_Frequency;
    uint32_t ADC_CLK_Frequency;
    uint32_t RTCHSI_Frequency;
    uint32_t RTCHSE_Frequency;
} RST_CLK_FreqTypeDef;




typedef struct
{
    uint32_t REG_0F;
} Init_NonVolatile_RST_CLK_TypeDef;




typedef enum
{
    RST_CLK_HSE_OFF = ((uint32_t)0x00),
    RST_CLK_HSE_ON = ((uint32_t)0x01),
    RST_CLK_HSE_Bypass = ((uint32_t)0x02)
} RST_CLK_HSE_Mode;
# 99 "./SPL/MDR32Fx/inc\\MDR32F9Qx_rst_clk.h"
typedef enum
{
    RST_CLK_LSE_OFF = ((uint32_t)0x00),
    RST_CLK_LSE_ON = ((uint32_t)0x01),
    RST_CLK_LSE_Bypass = ((uint32_t)0x02)
} RST_CLK_LSE_Mode;
# 113 "./SPL/MDR32Fx/inc\\MDR32F9Qx_rst_clk.h"
typedef enum
{
    RST_CLK_CPU_PLLsrcHSIdiv1 = ((uint32_t)0x00),
    RST_CLK_CPU_PLLsrcHSIdiv2 = ((uint32_t)0x01),
    RST_CLK_CPU_PLLsrcHSEdiv1 = ((uint32_t)0x02),
    RST_CLK_CPU_PLLsrcHSEdiv2 = ((uint32_t)0x03)
} RST_CLK_CPU_PLL_Source;
# 129 "./SPL/MDR32Fx/inc\\MDR32F9Qx_rst_clk.h"
typedef enum
{
    RST_CLK_CPU_C1srcHSIdiv1 = ((uint32_t)0x00),
    RST_CLK_CPU_C1srcHSIdiv2 = ((uint32_t)0x01),
    RST_CLK_CPU_C1srcHSEdiv1 = ((uint32_t)0x02),
    RST_CLK_CPU_C1srcHSEdiv2 = ((uint32_t)0x03)
} RST_CLK_CPU_C1_Source;
# 145 "./SPL/MDR32Fx/inc\\MDR32F9Qx_rst_clk.h"
typedef enum
{
    RST_CLK_CPU_PLLmul1 = ((uint32_t)0x00),
    RST_CLK_CPU_PLLmul2 = ((uint32_t)0x01),
    RST_CLK_CPU_PLLmul3 = ((uint32_t)0x02),
    RST_CLK_CPU_PLLmul4 = ((uint32_t)0x03),
    RST_CLK_CPU_PLLmul5 = ((uint32_t)0x04),
    RST_CLK_CPU_PLLmul6 = ((uint32_t)0x05),
    RST_CLK_CPU_PLLmul7 = ((uint32_t)0x06),
    RST_CLK_CPU_PLLmul8 = ((uint32_t)0x07),
    RST_CLK_CPU_PLLmul9 = ((uint32_t)0x08),
    RST_CLK_CPU_PLLmul10 = ((uint32_t)0x09),
    RST_CLK_CPU_PLLmul11 = ((uint32_t)0x0A),
    RST_CLK_CPU_PLLmul12 = ((uint32_t)0x0B),
    RST_CLK_CPU_PLLmul13 = ((uint32_t)0x0C),
    RST_CLK_CPU_PLLmul14 = ((uint32_t)0x0D),
    RST_CLK_CPU_PLLmul15 = ((uint32_t)0x0E),
    RST_CLK_CPU_PLLmul16 = ((uint32_t)0x0F)
} RST_CLK_CPU_PLL_Multiplier;






typedef enum
{
    RST_CLK_USB_PLLsrcHSIdiv1 = ((uint32_t)0x00),
    RST_CLK_USB_PLLsrcHSIdiv2 = ((uint32_t)0x01),
    RST_CLK_USB_PLLsrcHSEdiv1 = ((uint32_t)0x02),
    RST_CLK_USB_PLLsrcHSEdiv2 = ((uint32_t)0x03)
} RST_CLK_USB_PLL_Source;
# 186 "./SPL/MDR32Fx/inc\\MDR32F9Qx_rst_clk.h"
typedef enum
{
    RST_CLK_USB_PLLmul1 = ((uint32_t)0x00),
    RST_CLK_USB_PLLmul2 = ((uint32_t)0x01),
    RST_CLK_USB_PLLmul3 = ((uint32_t)0x02),
    RST_CLK_USB_PLLmul4 = ((uint32_t)0x03),
    RST_CLK_USB_PLLmul5 = ((uint32_t)0x04),
    RST_CLK_USB_PLLmul6 = ((uint32_t)0x05),
    RST_CLK_USB_PLLmul7 = ((uint32_t)0x06),
    RST_CLK_USB_PLLmul8 = ((uint32_t)0x07),
    RST_CLK_USB_PLLmul9 = ((uint32_t)0x08),
    RST_CLK_USB_PLLmul10 = ((uint32_t)0x09),
    RST_CLK_USB_PLLmul11 = ((uint32_t)0x0A),
    RST_CLK_USB_PLLmul12 = ((uint32_t)0x0B),
    RST_CLK_USB_PLLmul13 = ((uint32_t)0x0C),
    RST_CLK_USB_PLLmul14 = ((uint32_t)0x0D),
    RST_CLK_USB_PLLmul15 = ((uint32_t)0x0E),
    RST_CLK_USB_PLLmul16 = ((uint32_t)0x0F)
} RST_CLK_USB_PLL_Multiplier;






typedef enum
{
    RST_CLK_CPUclkDIV1 = ((uint32_t)0x00),
    RST_CLK_CPUclkDIV2 = ((uint32_t)0x08),
    RST_CLK_CPUclkDIV4 = ((uint32_t)0x09),
    RST_CLK_CPUclkDIV8 = ((uint32_t)0x0A),
    RST_CLK_CPUclkDIV16 = ((uint32_t)0x0B),
    RST_CLK_CPUclkDIV32 = ((uint32_t)0x0C),
    RST_CLK_CPUclkDIV64 = ((uint32_t)0x0D),
    RST_CLK_CPUclkDIV128 = ((uint32_t)0x0E),
    RST_CLK_CPUclkDIV256 = ((uint32_t)0x0F)
} RST_CLK_CPU_C3_Divisor;
# 237 "./SPL/MDR32Fx/inc\\MDR32F9Qx_rst_clk.h"
typedef enum
{
    RST_CLK_CPUclkHSI = ((uint32_t)0x0000),
    RST_CLK_CPUclkCPU_C3 = ((uint32_t)0x0100),
    RST_CLK_CPUclkLSE = ((uint32_t)0x0200),
    RST_CLK_CPUclkLSI = ((uint32_t)0x0300)
} RST_CLK_HCLK_Source;
# 253 "./SPL/MDR32Fx/inc\\MDR32F9Qx_rst_clk.h"
typedef enum
{
    RST_CLK_ADCclkCPU_C1 = ((uint32_t)0x0020),
    RST_CLK_ADCclkUSB_C1 = ((uint32_t)0x0021),
    RST_CLK_ADCclkCPU_C2 = ((uint32_t)0x0022),
    RST_CLK_ADCclkUSB_C2 = ((uint32_t)0x0023),
    RST_CLK_ADCclkLSE = ((uint32_t)0x0000),
    RST_CLK_ADCclkLSI = ((uint32_t)0x0010),
    RST_CLK_ADCclkHSI_C1 = ((uint32_t)0x0030)
} RST_CLK_ADC_Source;
# 275 "./SPL/MDR32Fx/inc\\MDR32F9Qx_rst_clk.h"
typedef enum
{
    RST_CLK_ADCclkDIV1 = ((uint32_t)0x00),
    RST_CLK_ADCclkDIV2 = ((uint32_t)0x08),
    RST_CLK_ADCclkDIV4 = ((uint32_t)0x09),
    RST_CLK_ADCclkDIV8 = ((uint32_t)0x0A),
    RST_CLK_ADCclkDIV16 = ((uint32_t)0x0B),
    RST_CLK_ADCclkDIV32 = ((uint32_t)0x0C),
    RST_CLK_ADCclkDIV64 = ((uint32_t)0x0D),
    RST_CLK_ADCclkDIV128 = ((uint32_t)0x0E),
    RST_CLK_ADCclkDIV256 = ((uint32_t)0x0F)
} RST_CLK_ADC_C3_Divisor;
# 349 "./SPL/MDR32Fx/inc\\MDR32F9Qx_rst_clk.h"
typedef enum
{
    RST_CLK_FLAG_HSIRDY = ((uint32_t)(0x00 | 23)),
    RST_CLK_FLAG_LSIRDY = ((uint32_t)(0x00 | 21)),
    RST_CLK_FLAG_HSERDY = ((uint32_t)(0x20 | 2)),
    RST_CLK_FLAG_HSE2RDY = ((uint32_t)(0x20 | 3)),
    RST_CLK_FLAG_LSERDY = ((uint32_t)(0x00 | 13)),
    RST_CLK_FLAG_PLLCPURDY = ((uint32_t)(0x20 | 1)),
    RST_CLK_FLAG_PLLUSBRDY = ((uint32_t)(0x20 | 0)),
    RST_CLK_FLAG_PLLDSPRDY = ((uint32_t)(0x20 | 3))
} RST_CLK_Flags;
# 373 "./SPL/MDR32Fx/inc\\MDR32F9Qx_rst_clk.h"
typedef enum
{
    RST_CLK_HSIclkDIV1 = ((uint32_t)0x00),
    RST_CLK_HSIclkDIV2 = ((uint32_t)0x08),
    RST_CLK_HSIclkDIV4 = ((uint32_t)0x09),
    RST_CLK_HSIclkDIV8 = ((uint32_t)0x0A),
    RST_CLK_HSIclkDIV16 = ((uint32_t)0x0B),
    RST_CLK_HSIclkDIV32 = ((uint32_t)0x0C),
    RST_CLK_HSIclkDIV64 = ((uint32_t)0x0D),
    RST_CLK_HSIclkDIV128 = ((uint32_t)0x0E),
    RST_CLK_HSIclkDIV256 = ((uint32_t)0x0F)
} RST_CLK_HSI_C1_Divisor;
# 399 "./SPL/MDR32Fx/inc\\MDR32F9Qx_rst_clk.h"
typedef enum
{
    RST_CLK_HSEclkDIV1 = ((uint32_t)0x00),
    RST_CLK_HSEclkDIV2 = ((uint32_t)0x08),
    RST_CLK_HSEclkDIV4 = ((uint32_t)0x09),
    RST_CLK_HSEclkDIV8 = ((uint32_t)0x0A),
    RST_CLK_HSEclkDIV16 = ((uint32_t)0x0B),
    RST_CLK_HSEclkDIV32 = ((uint32_t)0x0C),
    RST_CLK_HSEclkDIV64 = ((uint32_t)0x0D),
    RST_CLK_HSEclkDIV128 = ((uint32_t)0x0E),
    RST_CLK_HSEclkDIV256 = ((uint32_t)0x0F)
} RST_CLK_HSE_C1_Divisor;
# 683 "./SPL/MDR32Fx/inc\\MDR32F9Qx_rst_clk.h"
void RST_CLK_DeInit(void);
void RST_CLK_WarmDeInit(void);

void RST_CLK_HSEconfig(RST_CLK_HSE_Mode RST_CLK_HSE);
ErrorStatus RST_CLK_HSEstatus(void);





void RST_CLK_LSEconfig(RST_CLK_LSE_Mode RST_CLK_LSE);
ErrorStatus RST_CLK_LSEstatus(void);

void RST_CLK_HSIcmd(FunctionalState NewState);
void RST_CLK_HSIadjust(uint32_t HSItrimValue);
ErrorStatus RST_CLK_HSIstatus(void);

void RST_CLK_LSIcmd(FunctionalState NewState);
void RST_CLK_LSIadjust(uint32_t LSItrimValue);
ErrorStatus RST_CLK_LSIstatus(void);

void RST_CLK_CPU_PLLconfig(RST_CLK_CPU_PLL_Source RST_CLK_CPU_PLLsource, uint32_t RST_CLK_CPU_PLLmul);
void RST_CLK_CPU_PLLuse(FunctionalState UsePLL);
void RST_CLK_CPU_PLLcmd(FunctionalState NewState);
ErrorStatus RST_CLK_CPU_PLLstatus(void);

void RST_CLK_CPUclkPrescaler(RST_CLK_CPU_C3_Divisor CPUclkDivValue);
void RST_CLK_CPUclkSelection(RST_CLK_HCLK_Source CPU_CLK);

void RST_CLK_USB_PLLconfig(RST_CLK_USB_PLL_Source RST_CLK_USB_PLLsource, uint32_t RST_CLK_USB_PLLmul);
void RST_CLK_USB_PLLuse(FunctionalState UsePLL);
void RST_CLK_USB_PLLcmd(FunctionalState NewState);
ErrorStatus RST_CLK_USB_PLLstatus(void);

void RST_CLK_USBclkPrescaler(FunctionalState NewState);
void RST_CLK_USBclkEnable(FunctionalState NewState);

void RST_CLK_ADCclkSelection(RST_CLK_ADC_Source ADC_CLK);
void RST_CLK_ADCclkPrescaler(RST_CLK_ADC_C3_Divisor ADCclkDivValue);
void RST_CLK_ADCclkEnable(FunctionalState NewState);

void RST_CLK_HSIclkPrescaler(RST_CLK_HSI_C1_Divisor HSIclkDivValue);
void RST_CLK_RTC_HSIclkEnable(FunctionalState NewState);

void RST_CLK_HSEclkPrescaler(RST_CLK_HSE_C1_Divisor HSEclkDivValue);
void RST_CLK_RTC_HSEclkEnable(FunctionalState NewState);

void RST_CLK_CPUclkSelectionC1(RST_CLK_CPU_C1_Source CPU_CLK);

void RST_CLK_PCLKcmd(uint32_t RST_CLK_PCLK, FunctionalState NewState);
# 744 "./SPL/MDR32Fx/inc\\MDR32F9Qx_rst_clk.h"
void RST_CLK_GetClocksFreq(RST_CLK_FreqTypeDef* RST_CLK_Clocks);

FlagStatus RST_CLK_GetFlagStatus(RST_CLK_Flags RST_CLK_FLAG);
# 13 "main.c" 2
# 1 "./SPL/MDR32Fx/inc\\MDR32F9Qx_ssp.h" 1
# 49 "./SPL/MDR32Fx/inc\\MDR32F9Qx_ssp.h"
typedef enum
{
    SSP_ModeMaster = ((uint32_t)0x00),
    SSP_ModeSlave = ((uint32_t)0x04)
} SSP_Mode_TypeDef;







typedef enum
{
    SSP_WordLength4b = ((uint16_t)0x03),
    SSP_WordLength5b = ((uint16_t)0x04),
    SSP_WordLength6b = ((uint16_t)0x05),
    SSP_WordLength7b = ((uint16_t)0x06),
    SSP_WordLength8b = ((uint16_t)0x07),
    SSP_WordLength9b = ((uint16_t)0x08),
    SSP_WordLength10b = ((uint16_t)0x09),
    SSP_WordLength11b = ((uint16_t)0x0A),
    SSP_WordLength12b = ((uint16_t)0x0B),
    SSP_WordLength13b = ((uint16_t)0x0C),
    SSP_WordLength14b = ((uint16_t)0x0D),
    SSP_WordLength15b = ((uint16_t)0x0E),
    SSP_WordLength16b = ((uint16_t)0x0F)
} SSP_Word_Length_TypeDef;
# 95 "./SPL/MDR32Fx/inc\\MDR32F9Qx_ssp.h"
typedef enum
{
    SSP_SPH_1Edge = ((uint16_t)0x00),
    SSP_SPH_2Edge = ((uint16_t)0x80)
} SSP_Clock_Phase_TypeDef;







typedef enum
{
    SSP_SPO_Low = ((uint16_t)0x00),
    SSP_SPO_High = ((uint16_t)0x40)
} SSP_Clock_Polarity_TypeDef;







typedef enum
{
    SSP_FRF_SPI_Motorola = ((uint16_t)0x00),
    SSP_FRF_SSI_TI = ((uint16_t)0x10),
    SSP_FRF_Microwire = ((uint16_t)0x20)
} SSP_Frame_Format_TypeDef;
# 133 "./SPL/MDR32Fx/inc\\MDR32F9Qx_ssp.h"
typedef enum
{
    SSP_HardwareFlowControl_None = ((uint16_t)0x00),
    SSP_HardwareFlowControl_SOD = ((uint16_t)0x08),
    SSP_HardwareFlowControl_SSE = ((uint16_t)0x02),
    SSP_HardwareFlowControl_LBM = ((uint16_t)0x01)
} SSP_Hardware_Flow_Control_TypeDef;
# 152 "./SPL/MDR32Fx/inc\\MDR32F9Qx_ssp.h"
typedef enum
{
    SSP_FLAG_BSY = ((uint16_t)0x10),
    SSP_FLAG_RFF = ((uint16_t)0x08),
    SSP_FLAG_RNE = ((uint16_t)0x04),
    SSP_FLAG_TNF = ((uint16_t)0x02),
    SSP_FLAG_TFE = ((uint16_t)0x01)
} SSP_Flags_TypeDef;
# 170 "./SPL/MDR32Fx/inc\\MDR32F9Qx_ssp.h"
typedef enum
{
    SSP_IT_TX = ((uint32_t)0x08),
    SSP_IT_RX = ((uint32_t)0x04),
    SSP_IT_RT = ((uint32_t)0x02),
    SSP_IT_ROR = ((uint32_t)0x01)
} SSP_IT_TypeDef;
# 192 "./SPL/MDR32Fx/inc\\MDR32F9Qx_ssp.h"
typedef enum
{
    SSP_DMA_RXE = ((uint32_t)0x01),
    SSP_DMA_TXE = ((uint32_t)0x02)
} SSP_DMA_Req_TypeDef;
# 205 "./SPL/MDR32Fx/inc\\MDR32F9Qx_ssp.h"
typedef enum
{
    SSP_HCLKdiv1 = ((uint32_t)0x00),
    SSP_HCLKdiv2 = ((uint32_t)0x01),
    SSP_HCLKdiv4 = ((uint32_t)0x02),
    SSP_HCLKdiv8 = ((uint32_t)0x03),
    SSP_HCLKdiv16 = ((uint32_t)0x04),
    SSP_HCLKdiv32 = ((uint32_t)0x05),
    SSP_HCLKdiv64 = ((uint32_t)0x06),
    SSP_HCLKdiv128 = ((uint32_t)0x07)
} SSP_Clock_BRG_TypeDef;






typedef struct
{
    uint16_t SSP_SCR;



    uint16_t SSP_CPSDVSR;

    SSP_Mode_TypeDef SSP_Mode;

    SSP_Word_Length_TypeDef SSP_WordLength;

    SSP_Clock_Phase_TypeDef SSP_SPH;

    SSP_Clock_Polarity_TypeDef SSP_SPO;

    SSP_Frame_Format_TypeDef SSP_FRF;

    SSP_Hardware_Flow_Control_TypeDef SSP_HardwareFlowControl;

} SSP_InitTypeDef;
# 264 "./SPL/MDR32Fx/inc\\MDR32F9Qx_ssp.h"
void SSP_DeInit(MDR_SSP_TypeDef* SSPx);
void SSP_Init(MDR_SSP_TypeDef* SSPx, const SSP_InitTypeDef* SSP_InitStruct);
void SSP_StructInit(SSP_InitTypeDef* SSP_InitStruct);
void SSP_Cmd(MDR_SSP_TypeDef* SSPx, FunctionalState NewState);

void SSP_ITConfig(MDR_SSP_TypeDef* SSPx, uint32_t SSP_IT, FunctionalState NewState);
ITStatus SSP_GetITStatus(MDR_SSP_TypeDef* SSPx, SSP_IT_TypeDef SSP_IT);
ITStatus SSP_GetITStatusMasked(MDR_SSP_TypeDef* SSPx, SSP_IT_TypeDef SSP_IT);
void SSP_ClearITPendingBit(MDR_SSP_TypeDef* SSPx, SSP_IT_TypeDef SSP_IT);

void SSP_DMACmd(MDR_SSP_TypeDef* SSPx, uint32_t SSP_DMAReq, FunctionalState NewState);

void SSP_SendData(MDR_SSP_TypeDef* SSPx, uint16_t Data);
uint16_t SSP_ReceiveData(MDR_SSP_TypeDef* SSPx);

FlagStatus SSP_GetFlagStatus(MDR_SSP_TypeDef* SSPx, SSP_Flags_TypeDef SSP_FLAG);
void SSP_BRGInit(MDR_SSP_TypeDef* SSPx, SSP_Clock_BRG_TypeDef SSP_BRG);
# 14 "main.c" 2
# 1 "./SPL/MDR32Fx/inc\\MDR32F9Qx_port.h" 1
# 49 "./SPL/MDR32Fx/inc\\MDR32F9Qx_port.h"
typedef enum
{
    PORT_OE_IN = 0x0,
    PORT_OE_OUT = 0x1
} PORT_OE_TypeDef;






typedef enum
{
    PORT_MODE_ANALOG = 0x0,
    PORT_MODE_DIGITAL = 0x1
} PORT_MODE_TypeDef;






typedef enum
{
    PORT_PULL_UP_OFF = 0x0,
    PORT_PULL_UP_ON = 0x1
} PORT_PULL_UP_TypeDef;






typedef enum
{
    PORT_PULL_DOWN_OFF = 0x0,
    PORT_PULL_DOWN_ON = 0x1
} PORT_PULL_DOWN_TypeDef;







typedef enum
{
    PORT_PD_SHM_OFF = 0x0,
    PORT_PD_SHM_ON = 0x1
} PORT_PD_SHM_TypeDef;







typedef enum
{
    PORT_PD_DRIVER = 0x0,
    PORT_PD_OPEN = 0x1
} PORT_PD_TypeDef;






typedef enum
{
    PORT_GFEN_OFF = 0x0,
    PORT_GFEN_ON = 0x1
} PORT_GFEN_TypeDef;






typedef enum
{
    PORT_FUNC_PORT = 0x0,
    PORT_FUNC_MAIN = 0x1,
    PORT_FUNC_ALTER = 0x2,
    PORT_FUNC_OVERRID = 0x3
} PORT_FUNC_TypeDef;







typedef enum
{
    PORT_OUTPUT_OFF = 0x0,
    PORT_SPEED_SLOW = 0x1,
    PORT_SPEED_FAST = 0x2,
    PORT_SPEED_MAXFAST = 0x3
} PORT_SPEED_TypeDef;







typedef enum
{
    PORT_Pin_0 = 0x0001U,
    PORT_Pin_1 = 0x0002U,
    PORT_Pin_2 = 0x0004U,
    PORT_Pin_3 = 0x0008U,
    PORT_Pin_4 = 0x0010U,
    PORT_Pin_5 = 0x0020U,
    PORT_Pin_6 = 0x0040U,
    PORT_Pin_7 = 0x0080U,
    PORT_Pin_8 = 0x0100U,
    PORT_Pin_9 = 0x0200U,
    PORT_Pin_10 = 0x0400U,
    PORT_Pin_11 = 0x0800U,
    PORT_Pin_12 = 0x1000U,
    PORT_Pin_13 = 0x2000U,
    PORT_Pin_14 = 0x4000U,
    PORT_Pin_15 = 0x8000U,
    PORT_Pin_All = 0xFFFFU
} PORT_Pin_TypeDef;
# 200 "./SPL/MDR32Fx/inc\\MDR32F9Qx_port.h"
typedef struct
{
    uint16_t PORT_Pin;

    PORT_OE_TypeDef PORT_OE;

    PORT_PULL_UP_TypeDef PORT_PULL_UP;

    PORT_PULL_DOWN_TypeDef PORT_PULL_DOWN;

    PORT_PD_SHM_TypeDef PORT_PD_SHM;

    PORT_PD_TypeDef PORT_PD;

    PORT_GFEN_TypeDef PORT_GFEN;

    PORT_FUNC_TypeDef PORT_FUNC;

    PORT_SPEED_TypeDef PORT_SPEED;

    PORT_MODE_TypeDef PORT_MODE;

} PORT_InitTypeDef;







typedef enum
{
    Bit_RESET = 0,
    Bit_SET
} BitAction;
# 276 "./SPL/MDR32Fx/inc\\MDR32F9Qx_port.h"
void PORT_DeInit(MDR_PORT_TypeDef* MDR_PORTx);
void PORT_Init(MDR_PORT_TypeDef* MDR_PORTx, const PORT_InitTypeDef* PORT_InitStruct);
void PORT_StructInit(PORT_InitTypeDef* PORT_InitStruct);

uint8_t PORT_ReadInputDataBit(MDR_PORT_TypeDef* MDR_PORTx, PORT_Pin_TypeDef PORT_Pin);
uint32_t PORT_ReadInputData(MDR_PORT_TypeDef* MDR_PORTx);

void PORT_SetBits(MDR_PORT_TypeDef* MDR_PORTx, uint32_t PORT_Pin);
void PORT_ResetBits(MDR_PORT_TypeDef* MDR_PORTx, uint32_t PORT_Pin);


    void PORT_WriteBit(MDR_PORT_TypeDef* MDR_PORTx, uint32_t PORT_Pin, BitAction BitVal);



void PORT_Write(MDR_PORT_TypeDef* MDR_PORTx, uint32_t PortVal);
# 15 "main.c" 2
# 1 "./SPL/MDR32Fx/inc\\MDR32F9Qx_adc.h" 1
# 135 "./SPL/MDR32Fx/inc\\MDR32F9Qx_adc.h"
typedef enum
{
    ADC_SyncMode_Independent = (((uint32_t)0x0) << 16),
    ADC_SyncMode_Synchronous = (((uint32_t)0x1) << 16)
} ADC_SyncMode;
# 149 "./SPL/MDR32Fx/inc\\MDR32F9Qx_adc.h"
typedef enum
{
    ADC_TEMP_SENSOR_Disable = (((uint32_t)0x0) << 17),
    ADC_TEMP_SENSOR_Enable = (((uint32_t)0x1) << 17)
} ADC_Temp_Sensor;







typedef enum
{
    ADC_TEMP_SENSOR_AMPLIFIER_Disable = (((uint32_t)0x0) << 18),
    ADC_TEMP_SENSOR_AMPLIFIER_Enable = (((uint32_t)0x1) << 18)
} ADC_Temp_Sensor_Amplifier;







typedef enum
{
    ADC_TEMP_SENSOR_CONVERSION_Disable = (((uint32_t)0x0) << 19),
    ADC_TEMP_SENSOR_CONVERSION_Enable = (((uint32_t)0x1) << 19)
} ADC_Temp_Sensor_Conversion;







typedef enum
{
    ADC_VREF_CONVERSION_Disable = (((uint32_t)0x0) << 20),
    ADC_VREF_CONVERSION_Enable = (((uint32_t)0x1) << 20)
} ADC_Int_VRef_Conversion;







typedef enum
{
    ADC_CLOCK_SOURCE_CPU = (((uint32_t)0x0) << 2),
    ADC_CLOCK_SOURCE_ADC = (((uint32_t)0x1) << 2)
} ADCx_Clock_Source;







typedef enum
{
    ADC_SAMPLING_MODE_SINGLE_CONV = (((uint32_t)0x0) << 3),
    ADC_SAMPLING_MODE_CYCLIC_CONV = (((uint32_t)0x1) << 3)
} ADCx_Sampling_Mode;
# 226 "./SPL/MDR32Fx/inc\\MDR32F9Qx_adc.h"
typedef enum
{
    ADC_CH_SWITCHING_Disable = (((uint32_t)0x0) << 9),
    ADC_CH_SWITCHING_Enable = (((uint32_t)0x1) << 9)
} ADCx_Channel_Switching;







typedef enum
{
    ADC_CH_ADC0 = ((uint32_t)0x00),
    ADC_CH_ADC1 = ((uint32_t)0x01),
    ADC_CH_ADC2 = ((uint32_t)0x02),
    ADC_CH_ADC3 = ((uint32_t)0x03),
    ADC_CH_ADC4 = ((uint32_t)0x04),
    ADC_CH_ADC5 = ((uint32_t)0x05),
    ADC_CH_ADC6 = ((uint32_t)0x06),
    ADC_CH_ADC7 = ((uint32_t)0x07),
    ADC_CH_ADC8 = ((uint32_t)0x08),
    ADC_CH_ADC9 = ((uint32_t)0x09),
    ADC_CH_ADC10 = ((uint32_t)0x0A),
    ADC_CH_ADC11 = ((uint32_t)0x0B),
    ADC_CH_ADC12 = ((uint32_t)0x0C),
    ADC_CH_ADC13 = ((uint32_t)0x0D),
    ADC_CH_ADC14 = ((uint32_t)0x0E),
    ADC_CH_ADC15 = ((uint32_t)0x0F),
    ADC_CH_INT_VREF = ((uint32_t)0x1E),
    ADC_CH_TEMP_SENSOR = ((uint32_t)0x1F)
} ADCx_Channel_Number;
# 284 "./SPL/MDR32Fx/inc\\MDR32F9Qx_adc.h"
typedef enum
{
    ADC_LEVEL_CONTROL_Disable = (((uint32_t)0x0) << 10),
    ADC_LEVEL_CONTROL_Enable = (((uint32_t)0x1) << 10)
} ADCx_Level_Control;
# 299 "./SPL/MDR32Fx/inc\\MDR32F9Qx_adc.h"
typedef enum
{
    ADC_VREF_SOURCE_INTERNAL = (((uint32_t)0x0) << 11),
    ADC_VREF_SOURCE_EXTERNAL = (((uint32_t)0x1) << 11)
} ADCx_VRef_Source;







typedef enum
{
    ADC_INT_VREF_SOURCE_INEXACT = ((uint32_t)0x0),
    ADC_INT_VREF_SOURCE_EXACT = ((uint32_t)0x1)
} ADCx_Int_VRef_Source;







typedef enum
{
    ADC_CLK_div_None = (((uint32_t)0x0) << 12),
    ADC_CLK_div_2 = (((uint32_t)0x1) << 12),
    ADC_CLK_div_4 = (((uint32_t)0x2) << 12),
    ADC_CLK_div_8 = (((uint32_t)0x3) << 12),
    ADC_CLK_div_16 = (((uint32_t)0x4) << 12),
    ADC_CLK_div_32 = (((uint32_t)0x5) << 12),
    ADC_CLK_div_64 = (((uint32_t)0x6) << 12),
    ADC_CLK_div_128 = (((uint32_t)0x7) << 12),
    ADC_CLK_div_256 = (((uint32_t)0x8) << 12),
    ADC_CLK_div_512 = (((uint32_t)0x9) << 12),
    ADC_CLK_div_1024 = (((uint32_t)0xA) << 12),
    ADC_CLK_div_2048 = (((uint32_t)0xB) << 12)
} ADCx_Prescaler;
# 355 "./SPL/MDR32Fx/inc\\MDR32F9Qx_adc.h"
typedef enum
{
    ADCx_FLAG_OVERWRITE = (((uint32_t)0x1) << 0),
    ADCx_FLAG_OUT_OF_RANGE = (((uint32_t)0x1) << 1),
    ADCx_FLAG_END_OF_CONVERSION = (((uint32_t)0x1) << 2)
} ADCx_Flags;
# 369 "./SPL/MDR32Fx/inc\\MDR32F9Qx_adc.h"
typedef enum
{
    ADC1_FLAG_OVERWRITE = (ADCx_FLAG_OVERWRITE << 0),
    ADC1_FLAG_OUT_OF_RANGE = (ADCx_FLAG_OUT_OF_RANGE << 0),
    ADC1_FLAG_END_OF_CONVERSION = (ADCx_FLAG_END_OF_CONVERSION << 0),

    ADC2_FLAG_OVERWRITE = (ADCx_FLAG_OVERWRITE << 16),
    ADC2_FLAG_OUT_OF_RANGE = (ADCx_FLAG_OUT_OF_RANGE << 16),
    ADC2_FLAG_END_OF_CONVERSION = (ADCx_FLAG_END_OF_CONVERSION << 16)

} ADC_Flags;
# 399 "./SPL/MDR32Fx/inc\\MDR32F9Qx_adc.h"
typedef enum
{
    ADCx_IT_OUT_OF_RANGE = (((uint32_t)0x1) << 1),
    ADCx_IT_END_OF_CONVERSION = (((uint32_t)0x1) << 2)
} ADCx_IT_Def;







typedef enum
{
    ADC1_IT_OUT_OF_RANGE = (ADCx_IT_OUT_OF_RANGE << 0),
    ADC1_IT_END_OF_CONVERSION = (ADCx_IT_END_OF_CONVERSION << 0),

    ADC2_IT_OUT_OF_RANGE = (ADCx_IT_OUT_OF_RANGE << 16),
    ADC2_IT_END_OF_CONVERSION = (ADCx_IT_END_OF_CONVERSION << 16)

} ADC_IT_Def;
# 453 "./SPL/MDR32Fx/inc\\MDR32F9Qx_adc.h"
typedef struct {

    ADC_SyncMode ADC_SynchronousMode;

    uint32_t ADC_StartDelay;


    ADC_Temp_Sensor ADC_TempSensor;

    ADC_Temp_Sensor_Amplifier ADC_TempSensorAmplifier;

    ADC_Temp_Sensor_Conversion ADC_TempSensorConversion;

    ADC_Int_VRef_Conversion ADC_IntVRefConversion;

    uint32_t ADC_IntVRefTrimming;






} ADC_InitTypeDef;




typedef struct
{
    ADCx_Clock_Source ADC_ClockSource;


    ADCx_Sampling_Mode ADC_SamplingMode;


    ADCx_Channel_Switching ADC_ChannelSwitching;


    ADCx_Channel_Number ADC_ChannelNumber;


    uint32_t ADC_Channels;


    ADCx_Level_Control ADC_LevelControl;


    uint16_t ADC_LowLevel;


    uint16_t ADC_HighLevel;


    ADCx_VRef_Source ADC_VRefSource;


    ADCx_Int_VRef_Source ADC_IntVRefSource;


    ADCx_Prescaler ADC_Prescaler;


    uint32_t ADC_DelayGo;

} ADCx_InitTypeDef;
# 526 "./SPL/MDR32Fx/inc\\MDR32F9Qx_adc.h"
void ADC_DeInit(void);

void ADC_Init(const ADC_InitTypeDef* ADC_InitStruct);
void ADC_StructInit(ADC_InitTypeDef* ADC_InitStruct);

void ADC_SetTrim(uint32_t Trim);

void ADC1_Init(const ADCx_InitTypeDef* ADCx_InitStruct);
void ADCx_StructInit(ADCx_InitTypeDef* ADCx_InitStruct);

void ADC1_Cmd(FunctionalState NewState);

void ADC1_SetChannel(ADCx_Channel_Number Channel);
void ADC1_SetChannels(uint32_t ChannelMask);

void ADC1_OperationModeConfig(ADCx_Sampling_Mode SamplingMode, ADCx_Channel_Switching SwitchingMode);
void ADC1_SamplingModeConfig(ADCx_Sampling_Mode SamplingMode);
void ADC1_ChannelSwithingConfig(ADCx_Channel_Switching SwitchingMode);

void ADC1_LevelsConfig(uint32_t LowLevel, uint32_t HighLevel, ADCx_Level_Control NewState);
void ADC1_SetLowLevel(uint32_t LowLevel);
void ADC1_SetHighLevel(uint32_t HighLevel);

void ADC1_Start(void);

uint32_t ADC1_GetResult(void);

uint32_t ADC_GetStatus(void);
uint32_t ADC1_GetStatus(void);

FlagStatus ADC_GetFlagStatus(ADC_Flags Flag);
FlagStatus ADC1_GetFlagStatus(ADCx_Flags Flag);

void ADC1_ClearOverwriteFlag(void);
void ADC1_ClearOutOfRangeFlag(void);

void ADC_ITConfig(ADC_IT_Def ADC_IT, FunctionalState NewState);
void ADC1_ITConfig(ADC_IT_Def ADC_IT, FunctionalState NewState);

ITStatus ADC_GetITStatus(ADC_IT_Def ADC_IT);
ITStatus ADC1_GetITStatus(ADC_IT_Def ADC_IT);



    void ADC2_Init(const ADCx_InitTypeDef* ADCx_InitStruct);
    void ADC2_Cmd(FunctionalState NewState);
    void ADC2_SetChannel(ADCx_Channel_Number Channel);
    void ADC2_SetChannels(uint32_t ChannelMask);
    void ADC2_OperationModeConfig(ADCx_Sampling_Mode SamplingMode, ADCx_Channel_Switching SwitchingMode);
    void ADC2_OperationModeConfig(ADCx_Sampling_Mode SamplingMode, ADCx_Channel_Switching SwitchingMode);
    void ADC2_SamplingModeConfig(ADCx_Sampling_Mode SamplingMode);
    void ADC2_ChannelSwithingConfig(ADCx_Channel_Switching SwitchingMode);
    void ADC2_LevelsConfig(uint32_t LowLevel, uint32_t HighLevel, ADCx_Level_Control NewState);
    void ADC2_SetLowLevel(uint32_t LowLevel);
    void ADC2_SetHighLevel(uint32_t HighLevel);
    void ADC2_Start(void);
    uint32_t ADC2_GetResult(void);
    uint32_t ADC2_GetStatus(void);
    FlagStatus ADC2_GetFlagStatus(ADCx_Flags Flag);
    void ADC2_ClearOverwriteFlag(void);
    void ADC2_ClearOutOfRangeFlag(void);
    void ADC2_ITConfig(ADC_IT_Def ADC_IT, FunctionalState NewState);
    ITStatus ADC2_GetITStatus(ADC_IT_Def ADC_IT);
# 16 "main.c" 2
# 1 "./SPL/MDR32Fx/inc\\MDR32F9Qx_dac.h" 1
# 49 "./SPL/MDR32Fx/inc\\MDR32F9Qx_dac.h"
typedef enum
{
    DAC_SYNC_MODE_Independent = (((uint32_t)0x0) << 4),
    DAC_SYNC_MODE_Synchronous = (((uint32_t)0x1) << 4)
} DAC_Sync_Mode;







typedef enum
{
    DAC1_AVCC = (((uint32_t)0x0) << 0),
    DAC1_REF = (((uint32_t)0x1) << 0)
} DAC1_Ref_Src;







typedef enum
{
    DAC2_AVCC = (((uint32_t)0x0) << 1),
    DAC2_REF = (((uint32_t)0x1) << 1)
} DAC2_Ref_Src;
# 100 "./SPL/MDR32Fx/inc\\MDR32F9Qx_dac.h"
void DAC_DeInit(void);

void DAC_Init(DAC_Sync_Mode SyncMode, DAC1_Ref_Src DAC1_Ref, DAC2_Ref_Src DAC2_Ref);

void DAC1_Init(DAC1_Ref_Src DAC1_Ref);
void DAC2_Init(DAC2_Ref_Src DAC2_Ref);

void DAC1_Cmd(FunctionalState NewState);
void DAC2_Cmd(FunctionalState NewState);

void DAC1_SetData(uint32_t Data);
void DAC2_SetData(uint32_t Data);

uint32_t DAC1_GetData(void);
uint32_t DAC2_GetData(void);
# 17 "main.c" 2
# 1 "./SPL/MDR32Fx/inc\\MDR32F9Qx_dma.h" 1
# 57 "./SPL/MDR32Fx/inc\\MDR32F9Qx_dma.h"
typedef enum
{
    DMA_ALternateDataDisabled = ((uint16_t)(0x00)),
    DMA_ALternateDataEnabled = ((uint16_t)(0x01))
} DMA_Alt_Data_Usage;







typedef enum
{

    DMA_Channel_UART1_TX = ((uint8_t)(0)),
    DMA_Channel_UART1_RX = ((uint8_t)(1)),
    DMA_Channel_UART2_TX = ((uint8_t)(2)),
    DMA_Channel_UART2_RX = ((uint8_t)(3)),
    DMA_Channel_SSP1_TX = ((uint8_t)(4)),
    DMA_Channel_SSP1_RX = ((uint8_t)(5)),
    DMA_Channel_SSP2_TX = ((uint8_t)(6)),
    DMA_Channel_SSP2_RX = ((uint8_t)(7)),
    DMA_Channel_ADC1 = ((uint8_t)(8)),
    DMA_Channel_ADC2 = ((uint8_t)(9)),
    DMA_Channel_TIM1 = ((uint8_t)(10)),
    DMA_Channel_TIM2 = ((uint8_t)(11)),
    DMA_Channel_TIM3 = ((uint8_t)(12)),
    DMA_Channel_SW1 = ((uint8_t)(13)),
    DMA_Channel_SW2 = ((uint8_t)(14)),
    DMA_Channel_SW3 = ((uint8_t)(15)),
    DMA_Channel_SW4 = ((uint8_t)(16)),
    DMA_Channel_SW5 = ((uint8_t)(17)),
    DMA_Channel_SW6 = ((uint8_t)(18)),
    DMA_Channel_SW7 = ((uint8_t)(19)),
    DMA_Channel_SW8 = ((uint8_t)(20)),
    DMA_Channel_SW9 = ((uint8_t)(21)),
    DMA_Channel_SW10 = ((uint8_t)(22)),
    DMA_Channel_SW11 = ((uint8_t)(23)),
    DMA_Channel_SW12 = ((uint8_t)(24)),
    DMA_Channel_SW13 = ((uint8_t)(25)),
    DMA_Channel_SW14 = ((uint8_t)(26)),
    DMA_Channel_SW15 = ((uint8_t)(27)),
    DMA_Channel_SW16 = ((uint8_t)(28)),
    DMA_Channel_SW17 = ((uint8_t)(29)),
    DMA_Channel_SW18 = ((uint8_t)(30)),
    DMA_Channel_SW19 = ((uint8_t)(31)),
# 299 "./SPL/MDR32Fx/inc\\MDR32F9Qx_dma.h"
} DMA_Valid_Channels;






typedef enum
{
    DMA_SourceIncByte = ((uint32_t)0x00),
    DMA_SourceIncHalfword = ((uint32_t)0x01),
    DMA_SourceIncWord = ((uint32_t)0x02),
    DMA_SourceIncNo = ((uint32_t)0x03)
} DMA_Src_Inc_Mode;
# 322 "./SPL/MDR32Fx/inc\\MDR32F9Qx_dma.h"
typedef enum
{
    DMA_DestIncByte = ((uint32_t)0x00),
    DMA_DestIncHalfword = ((uint32_t)0x01),
    DMA_DestIncWord = ((uint32_t)0x02),
    DMA_DestIncNo = ((uint32_t)0x03)
} DMA_Dest_Inc_Mode;
# 338 "./SPL/MDR32Fx/inc\\MDR32F9Qx_dma.h"
typedef enum
{
    DMA_MemoryDataSize_Byte = ((uint32_t)(0x00 << 24)),
    DMA_MemoryDataSize_HalfWord = ((uint32_t)(0x11 << 24)),
    DMA_MemoryDataSize_Word = ((uint32_t)(0x22 << 24))
} DMA_Mem_Data_Size;
# 352 "./SPL/MDR32Fx/inc\\MDR32F9Qx_dma.h"
typedef enum
{
    DMA_Mode_Stop = ((uint32_t)0x0),
    DMA_Mode_Basic = ((uint32_t)0x1),
    DMA_Mode_AutoRequest = ((uint32_t)0x2),
    DMA_Mode_PingPong = ((uint32_t)0x3),
    DMA_Mode_MemScatterPri = ((uint32_t)0x4),
    DMA_Mode_MemScatterAlt = ((uint32_t)0x5),
    DMA_Mode_PerScatterPri = ((uint32_t)0x6),
    DMA_Mode_PerScatterAlt = ((uint32_t)0x7),
    DMA_Mode_PerScatterAltBurst = ((uint32_t)0xF)
} DMA_Operating_Mode;
# 386 "./SPL/MDR32Fx/inc\\MDR32F9Qx_dma.h"
typedef enum
{
    DMA_Priority_Default = ((uint8_t)0x00),
    DMA_Priority_High = ((uint8_t)0x01)
} DMA_Priority_Level;







typedef enum
{
    DMA_BurstClear = ((uint8_t)0x00),
    DMA_BurstSet = ((uint8_t)0x01)
} DMA_Burst_Mode;







typedef enum
{
    DMA_SourceCacheable = ((uint32_t)(0x01 << 20)),
    DMA_SourceBufferable = ((uint32_t)(0x01 << 19)),
    DMA_SourcePrivileged = ((uint32_t)(0x01 << 18))
} DMA_Src_Protection_Control;






typedef enum
{
    DMA_DestCacheable = ((uint32_t)(0x01 << 23)),
    DMA_DestBufferable = ((uint32_t)(0x01 << 22)),
    DMA_DestPrivileged = ((uint32_t)(0x01 << 21))
} DMA_Dest_Protection_Control;
# 442 "./SPL/MDR32Fx/inc\\MDR32F9Qx_dma.h"
typedef enum
{
    DMA_AHB_Cacheable = ((uint32_t)(0x01 << 7)),
    DMA_AHB_Bufferable = ((uint32_t)(0x01 << 6)),
    DMA_AHB_Privileged = ((uint32_t)(0x01 << 5))
} DMA_AHB_Protection_Control;






typedef enum
{
    DMA_Transfers_1 = ((uint32_t)(0x00 << 14)),
    DMA_Transfers_2 = ((uint32_t)(0x01 << 14)),
    DMA_Transfers_4 = ((uint32_t)(0x02 << 14)),
    DMA_Transfers_8 = ((uint32_t)(0x03 << 14)),
    DMA_Transfers_16 = ((uint32_t)(0x04 << 14)),
    DMA_Transfers_32 = ((uint32_t)(0x05 << 14)),
    DMA_Transfers_64 = ((uint32_t)(0x06 << 14)),
    DMA_Transfers_128 = ((uint32_t)(0x07 << 14)),
    DMA_Transfers_256 = ((uint32_t)(0x08 << 14)),
    DMA_Transfers_512 = ((uint32_t)(0x09 << 14)),
    DMA_Transfers_1024 = ((uint32_t)(0x0A << 14))
} DMA_Number_Continuous_Transfers;
# 484 "./SPL/MDR32Fx/inc\\MDR32F9Qx_dma.h"
typedef enum
{
    DMA_CTRL_DATA_PRIMARY = ((uint8_t)0x00),

    DMA_CTRL_DATA_ALTERNATE = ((uint8_t)0x01)

} DMA_Data_Struct_Selection;
# 502 "./SPL/MDR32Fx/inc\\MDR32F9Qx_dma.h"
typedef enum
{
    DMA_FLAG_DMA_ENA = ((uint8_t)0x01),
    DMA_FLAG_DMA_ERR = ((uint8_t)0x02),
    DMA_FLAG_CHNL_ENA = ((uint8_t)0x03),
    DMA_FLAG_CHNL_MASK = ((uint8_t)0x04),
    DMA_FLAG_CHNL_WAIT = ((uint8_t)0x05),
    DMA_FLAG_CHNL_BURST = ((uint8_t)0x06),
    DMA_FLAG_CHNL_ALT = ((uint8_t)0x07),
    DMA_FLAG_CHNL_PRIORITY = ((uint8_t)0x08)
} DMA_Flags;
# 526 "./SPL/MDR32Fx/inc\\MDR32F9Qx_dma.h"
typedef struct
{
    uint32_t DMA_SourceBaseAddr;
    uint32_t DMA_DestBaseAddr;
    DMA_Src_Inc_Mode DMA_SourceIncSize;

    DMA_Dest_Inc_Mode DMA_DestIncSize;

    DMA_Mem_Data_Size DMA_MemoryDataSize;

    DMA_Operating_Mode DMA_Mode;

    uint32_t DMA_CycleSize;

    DMA_Number_Continuous_Transfers DMA_NumContinuous;

    uint32_t DMA_SourceProtCtrl;

    uint32_t DMA_DestProtCtrl;

} DMA_CtrlDataInitTypeDef;




typedef struct
{
    uint32_t DMA_SourceEndAddr;
    uint32_t DMA_DestEndAddr;
    uint32_t DMA_Control;
    uint32_t DMA_Unused;
} DMA_CtrlDataTypeDef;




typedef struct
{
    DMA_CtrlDataInitTypeDef *DMA_PriCtrlData;

    DMA_CtrlDataInitTypeDef *DMA_AltCtrlData;

    uint32_t DMA_ProtCtrl;

    DMA_Priority_Level DMA_Priority;

    DMA_Burst_Mode DMA_UseBurst;

    DMA_Data_Struct_Selection DMA_SelectDataStructure;

} DMA_ChannelInitTypeDef;




typedef struct
{
    DMA_CtrlDataTypeDef *DMA_SG_TaskArray;




    uint32_t DMA_SG_TaskNumber;
    uint32_t DMA_SourceProtCtrl;

    uint32_t DMA_DestProtCtrl;

    uint32_t DMA_ProtCtrl;

    DMA_Priority_Level DMA_Priority;

    DMA_Burst_Mode DMA_UseBurst;

} DMA_Channel_SG_InitTypeDef;
# 608 "./SPL/MDR32Fx/inc\\MDR32F9Qx_dma.h"
void DMA_DeInit(void);

void DMA_CtrlDataInit(DMA_CtrlDataInitTypeDef *DMA_ctrl_data_ptr, DMA_CtrlDataTypeDef *DMA_ctrl_table_ptr);
void DMA_CtrlInit(uint8_t DMA_Channel, DMA_Data_Struct_Selection DMA_CtrlDataType, DMA_CtrlDataInitTypeDef* DMA_CtrlStruct);
void DMA_SG_CtrlInit(uint32_t DMA_Task, DMA_CtrlDataTypeDef *DMA_SG_TaskArray, DMA_CtrlDataInitTypeDef* DMA_CtrlStruct);

void DMA_SG_Init(uint8_t DMA_Channel, DMA_Channel_SG_InitTypeDef *DMA_SG_InitStruct);
void DMA_Init(uint8_t DMA_Channel, DMA_ChannelInitTypeDef* DMA_InitStruct);

void DMA_StructInit(DMA_ChannelInitTypeDef* DMA_InitStruct);
void DMA_SG_StructInit(DMA_Channel_SG_InitTypeDef* DMA_InitStruct);

void DMA_Cmd(uint8_t DMA_Channel, FunctionalState NewState);

void DMA_Request(uint8_t DMA_Channel);
void DMA_ClearError(void);

uint32_t DMA_GetCurrTransferCounter(uint8_t DMA_Channel, DMA_Data_Struct_Selection DMA_CtrlData);

FlagStatus DMA_GetFlagStatus(uint8_t DMA_Channel, DMA_Flags DMA_Flag);
# 18 "main.c" 2
# 1 "./SPL/MDR32Fx/inc\\MDR32F9Qx_timer.h" 1
# 49 "./SPL/MDR32Fx/inc\\MDR32F9Qx_timer.h"
typedef enum
{
    TIMER_CntMode_ClkFixedDir = (((uint32_t)0x0) << 6),
    TIMER_CntMode_ClkChangeDir = (((uint32_t)0x1) << 6),
    TIMER_CntMode_EvtFixedDir = (((uint32_t)0x2) << 6),
    TIMER_CntMode_EvtChangeDir = (((uint32_t)0x3) << 6)
} TIMER_Counter_Mode_TypeDef;
# 65 "./SPL/MDR32Fx/inc\\MDR32F9Qx_timer.h"
typedef enum
{
    TIMER_CntDir_Up = (((uint32_t)0x0) << 3),
    TIMER_CntDir_Dn = (((uint32_t)0x1) << 3)
} TIMER_Counter_Dir_TypeDef;







typedef enum
{
    TIMER_EvSrc_TIM_CLK = (((uint32_t)0x0) << 8),
    TIMER_EvSrc_TM1 = (((uint32_t)0x1) << 8),
    TIMER_EvSrc_TM2 = (((uint32_t)0x2) << 8),
    TIMER_EvSrc_TM3 = (((uint32_t)0x3) << 8),
    TIMER_EvSrc_CH1 = (((uint32_t)0x4) << 8),
    TIMER_EvSrc_CH2 = (((uint32_t)0x5) << 8),
    TIMER_EvSrc_CH3 = (((uint32_t)0x6) << 8),
    TIMER_EvSrc_CH4 = (((uint32_t)0x7) << 8),
    TIMER_EvSrc_ETR = (((uint32_t)0x8) << 8)
} TIMER_Event_Src_TypeDef;
# 108 "./SPL/MDR32Fx/inc\\MDR32F9Qx_timer.h"
typedef enum
{
    TIMER_FDTS_TIMER_CLK_div_1 = (((uint32_t)0x0) << 4),
    TIMER_FDTS_TIMER_CLK_div_2 = (((uint32_t)0x1) << 4),
    TIMER_FDTS_TIMER_CLK_div_3 = (((uint32_t)0x2) << 4),
    TIMER_FDTS_TIMER_CLK_div_4 = (((uint32_t)0x3) << 4)
} TIMER_Filter_Sampl_Clk_TypeDef;
# 124 "./SPL/MDR32Fx/inc\\MDR32F9Qx_timer.h"
typedef enum
{
    TIMER_ARR_Update_Immediately = (((uint32_t)0x0) << 1),
    TIMER_ARR_Update_On_CNT_Overflow = (((uint32_t)0x1) << 1)
} TIMER_ARR_Update_Mode_TypeDef;







typedef enum
{
    TIMER_Filter_1FF_at_TIMER_CLK = ((uint32_t)0x0),
    TIMER_Filter_2FF_at_TIMER_CLK = ((uint32_t)0x1),
    TIMER_Filter_4FF_at_TIMER_CLK = ((uint32_t)0x2),
    TIMER_Filter_8FF_at_TIMER_CLK = ((uint32_t)0x3),
    TIMER_Filter_6FF_at_FTDS_div_2 = ((uint32_t)0x4),
    TIMER_Filter_8FF_at_FTDS_div_2 = ((uint32_t)0x5),
    TIMER_Filter_6FF_at_FTDS_div_4 = ((uint32_t)0x6),
    TIMER_Filter_8FF_at_FTDS_div_4 = ((uint32_t)0x7),
    TIMER_Filter_6FF_at_FTDS_div_8 = ((uint32_t)0x8),
    TIMER_Filter_8FF_at_FTDS_div_8 = ((uint32_t)0x9),
    TIMER_Filter_5FF_at_FTDS_div_16 = ((uint32_t)0xA),
    TIMER_Filter_6FF_at_FTDS_div_16 = ((uint32_t)0xB),
    TIMER_Filter_8FF_at_FTDS_div_16 = ((uint32_t)0xC),
    TIMER_Filter_5FF_at_FTDS_div_32 = ((uint32_t)0xD),
    TIMER_Filter_6FF_at_FTDS_div_32 = ((uint32_t)0xE),
    TIMER_Filter_8FF_at_FTDS_div_32 = ((uint32_t)0xF)
} TIMER_Filter_Config_TypeDef;
# 176 "./SPL/MDR32Fx/inc\\MDR32F9Qx_timer.h"
typedef enum
{
    TIMER_ETR_Prescaler_None = (((uint32_t)0x0) << 2),
    TIMER_ETR_Prescaler_div_2 = (((uint32_t)0x1) << 2),
    TIMER_ETR_Prescaler_div_4 = (((uint32_t)0x2) << 2),
    TIMER_ETR_Prescaler_div_8 = (((uint32_t)0x3) << 2)
} TIMER_ETR_Prescaler_TypeDef;
# 192 "./SPL/MDR32Fx/inc\\MDR32F9Qx_timer.h"
typedef enum
{
   TIMER_ETRPolarity_NonInverted = (((uint32_t)0x0) << 1),
   TIMER_ETRPolarity_Inverted = (((uint32_t)0x1) << 1)
} TIMER_ETR_Polarity_TypeDef;







typedef enum
{
    TIMER_BRKPolarity_NonInverted = (((uint32_t)0x0) << 0),
    TIMER_BRKPolarity_Inverted = (((uint32_t)0x1) << 0)
} TIMER_BRK_Polarity_TypeDef;







typedef enum
{
    TIMER_CHANNEL1 = ((uint32_t)0x0),
    TIMER_CHANNEL2 = ((uint32_t)0x1),
    TIMER_CHANNEL3 = ((uint32_t)0x2),
    TIMER_CHANNEL4 = ((uint32_t)0x3)
} TIMER_Channel_Number_TypeDef;
# 232 "./SPL/MDR32Fx/inc\\MDR32F9Qx_timer.h"
typedef enum
{
    TIMER_CH_MODE_PWM = (((uint32_t)0x0) << 15),
    TIMER_CH_MODE_CAPTURE = (((uint32_t)0x1) << 15)
} TIMER_CH_Mode_TypeDef;







typedef enum
{
    TIMER_CH_ETR_RESET_Disable = (((uint32_t)0x0) << 13),
    TIMER_CH_ETR_RESET_Enable = (((uint32_t)0x1) << 13)
} TIMER_CH_ETR_RESET_TypeDef;







typedef enum
{
    TIMER_CH_BRK_RESET_Disable = (((uint32_t)0x0) << 12),
    TIMER_CH_BRK_RESET_Enable = (((uint32_t)0x1) << 12)
} TIMER_CH_BRK_RESET_TypeDef;







typedef enum
{
    TIMER_CH_REF_Format0 = (((uint32_t)0x0) << 9),
    TIMER_CH_REF_Format1 = (((uint32_t)0x1) << 9),



    TIMER_CH_REF_Format2 = (((uint32_t)0x2) << 9),



    TIMER_CH_REF_Format3 = (((uint32_t)0x3) << 9),


    TIMER_CH_REF_Format4 = (((uint32_t)0x4) << 9),
    TIMER_CH_REF_Format5 = (((uint32_t)0x5) << 9),
    TIMER_CH_REF_Format6 = (((uint32_t)0x6) << 9),



    TIMER_CH_REF_Format7 = (((uint32_t)0x7) << 9)



} TIMER_CH_REF_Format_TypeDef;
# 306 "./SPL/MDR32Fx/inc\\MDR32F9Qx_timer.h"
typedef enum
{
    TIMER_CH_Prescaler_None = ((uint32_t)0x0),
    TIMER_CH_Prescaler_div_2 = ((uint32_t)0x1),
    TIMER_CH_Prescaler_div_4 = ((uint32_t)0x2),
    TIMER_CH_Prescaler_div_8 = ((uint32_t)0x3)
} TIMER_CH_Prescaler_TypeDef;
# 322 "./SPL/MDR32Fx/inc\\MDR32F9Qx_timer.h"
typedef enum
{
    TIMER_CH_EvSrc_PE = (((uint32_t)0x0) << 4),
    TIMER_CH_EvSrc_NE = (((uint32_t)0x1) << 4),
    TIMER_CH_EvSrc_PE_OC1 = (((uint32_t)0x2) << 4),
    TIMER_CH_EvSrc_PE_OC2 = (((uint32_t)0x3) << 4)
} TIMER_CH_Event_Src_TypeDef;
# 338 "./SPL/MDR32Fx/inc\\MDR32F9Qx_timer.h"
typedef enum
{
    TIMER_CH_CCR1EvSrc_PE = (((uint32_t)0x0) << 0),
    TIMER_CH_CCR1EvSrc_NE = (((uint32_t)0x1) << 0),
    TIMER_CH_CCR1EvSrc_NE_OC1 = (((uint32_t)0x2) << 0),
    TIMER_CH_CCR1EvSrc_NE_OC2 = (((uint32_t)0x3) << 0)
} TIMER_CH_CCR1_Event_Src_TypeDef;
# 354 "./SPL/MDR32Fx/inc\\MDR32F9Qx_timer.h"
typedef enum
{
    TIMER_CH_CCR_Update_Immediately = (((uint32_t)0x0) << 3),
    TIMER_CH_CCR_Update_On_CNT_eq_0 = (((uint32_t)0x1) << 3)
} TIMER_CH_CCR_Update_Mode_TypeDef;







typedef enum
{
    TIMER_CHOPolarity_NonInverted = ((uint32_t)0x0),
    TIMER_CHOPolarity_Inverted = ((uint32_t)0x1)
} TIMER_CH_OUT_Polarity_TypeDef;







typedef enum
{
    TIMER_CH_OutSrc_Only_0 = ((uint32_t)0x0),
    TIMER_CH_OutSrc_Only_1 = ((uint32_t)0x1),
    TIMER_CH_OutSrc_REF = ((uint32_t)0x2),
    TIMER_CH_OutSrc_DTG = ((uint32_t)0x3)
} TIMER_CH_OUT_Src_TypeDef;
# 394 "./SPL/MDR32Fx/inc\\MDR32F9Qx_timer.h"
typedef enum
{
    TIMER_CH_OutMode_Input = ((uint32_t)0x0),
    TIMER_CH_OutMode_Output = ((uint32_t)0x1),
    TIMER_CH_OutMode_REF_as_OE = ((uint32_t)0x2),
    TIMER_CH_OutMode_DTG_as_OE = ((uint32_t)0x3)
} TIMER_CH_OUT_Mode_TypeDef;
# 410 "./SPL/MDR32Fx/inc\\MDR32F9Qx_timer.h"
typedef enum
{
    TIMER_CH_DTG_ClkSrc_TIMER_CLK = (((uint32_t)0x0) << 4),
    TIMER_CH_DTG_ClkSrc_FDTS = (((uint32_t)0x1) << 4)
} TIMER_CH_DTG_Clk_Src_TypeDef;







typedef enum
{
    TIMER_STATUS_CNT_ZERO = (((uint32_t)0x1) << 0),
    TIMER_STATUS_CNT_ARR = (((uint32_t)0x1) << 1),
    TIMER_STATUS_ETR_RISING_EDGE = (((uint32_t)0x1) << 2),
    TIMER_STATUS_ETR_FALLING_EDGE = (((uint32_t)0x1) << 3),
    TIMER_STATUS_BRK = (((uint32_t)0x1) << 4),
    TIMER_STATUS_CCR_CAP_CH1 = (((uint32_t)0x1) << 5),
    TIMER_STATUS_CCR_CAP_CH2 = (((uint32_t)0x1) << 6),
    TIMER_STATUS_CCR_CAP_CH3 = (((uint32_t)0x1) << 7),
    TIMER_STATUS_CCR_CAP_CH4 = (((uint32_t)0x1) << 8),
    TIMER_STATUS_CCR_REF_CH1 = (((uint32_t)0x1) << 9),
    TIMER_STATUS_CCR_REF_CH2 = (((uint32_t)0x1) << 10),
    TIMER_STATUS_CCR_REF_CH3 = (((uint32_t)0x1) << 11),
    TIMER_STATUS_CCR_REF_CH4 = (((uint32_t)0x1) << 12),
    TIMER_STATUS_CCR_CAP1_CH1 = (((uint32_t)0x1) << 13),
    TIMER_STATUS_CCR_CAP1_CH2 = (((uint32_t)0x1) << 14),
    TIMER_STATUS_CCR_CAP1_CH3 = (((uint32_t)0x1) << 15),
    TIMER_STATUS_CCR_CAP1_CH4 = (((uint32_t)0x1) << 16)
} TIMER_Status_Flags_TypeDef;
# 506 "./SPL/MDR32Fx/inc\\MDR32F9Qx_timer.h"
typedef enum
{
    TIMER_HCLKdiv1 = ((uint32_t)0x00),
    TIMER_HCLKdiv2 = ((uint32_t)0x01),
    TIMER_HCLKdiv4 = ((uint32_t)0x02),
    TIMER_HCLKdiv8 = ((uint32_t)0x03),
    TIMER_HCLKdiv16 = ((uint32_t)0x04),
    TIMER_HCLKdiv32 = ((uint32_t)0x05),
    TIMER_HCLKdiv64 = ((uint32_t)0x06),
    TIMER_HCLKdiv128 = ((uint32_t)0x07)
} TIMER_Clock_BRG_TypeDef;
# 531 "./SPL/MDR32Fx/inc\\MDR32F9Qx_timer.h"
typedef struct
{

    uint16_t TIMER_IniCounter;





    uint16_t TIMER_Prescaler;



    uint16_t TIMER_Period;







    uint16_t TIMER_CounterMode;

    uint16_t TIMER_CounterDirection;

    uint16_t TIMER_EventSource;

    uint16_t TIMER_FilterSampling;

    uint16_t TIMER_ARR_UpdateMode;

    uint16_t TIMER_ETR_FilterConf;

    uint16_t TIMER_ETR_Prescaler;

    uint16_t TIMER_ETR_Polarity;

    uint16_t TIMER_BRK_Polarity;

} TIMER_CntInitTypeDef;




typedef struct
{
    uint16_t TIMER_CH_Number;

    uint16_t TIMER_CH_Mode;

    uint16_t TIMER_CH_ETR_Ena;

    uint16_t TIMER_CH_ETR_Reset;

    uint16_t TIMER_CH_BRK_Reset;

    uint16_t TIMER_CH_REF_Format;

    uint16_t TIMER_CH_Prescaler;

    uint16_t TIMER_CH_EventSource;

    uint16_t TIMER_CH_FilterConf;

    uint16_t TIMER_CH_CCR_UpdateMode;

    uint16_t TIMER_CH_CCR1_Ena;

    uint16_t TIMER_CH_CCR1_EventSource;

} TIMER_ChnInitTypeDef;




typedef struct
{
    uint16_t TIMER_CH_Number;

    uint16_t TIMER_CH_DirOut_Polarity;

    uint16_t TIMER_CH_DirOut_Source;

    uint16_t TIMER_CH_DirOut_Mode;

    uint16_t TIMER_CH_NegOut_Polarity;

    uint16_t TIMER_CH_NegOut_Source;

    uint16_t TIMER_CH_NegOut_Mode;

    uint16_t TIMER_CH_DTG_MainPrescaler;


    uint16_t TIMER_CH_DTG_AuxPrescaler;


    uint16_t TIMER_CH_DTG_ClockSource;

} TIMER_ChnOutInitTypeDef;
# 669 "./SPL/MDR32Fx/inc\\MDR32F9Qx_timer.h"
void TIMER_DeInit(MDR_TIMER_TypeDef* TIMERx);

void TIMER_CntInit(MDR_TIMER_TypeDef* TIMERx, const TIMER_CntInitTypeDef* TIMER_CntInitStruct);
void TIMER_CntStructInit(TIMER_CntInitTypeDef* TIMER_CntInitStruct);

void TIMER_Cmd(MDR_TIMER_TypeDef* TIMERx, FunctionalState NewState);





    void TIMER_SetCounter(MDR_TIMER_TypeDef* TIMERx, uint16_t Counter);
    uint16_t TIMER_GetCounter(MDR_TIMER_TypeDef* TIMERx);


void TIMER_SetCntPrescaler(MDR_TIMER_TypeDef* TIMERx, uint16_t Prescaler);
uint16_t TIMER_GetCntPrescaler(MDR_TIMER_TypeDef* TIMERx);






    void TIMER_SetCntAutoreload(MDR_TIMER_TypeDef* TIMERx, uint16_t Autoreload);
    void TIMER_CntAutoreloadConfig(MDR_TIMER_TypeDef* TIMERx, uint16_t Autoreload, TIMER_ARR_Update_Mode_TypeDef UpdateMode);
    uint16_t TIMER_GetCntAutoreload(MDR_TIMER_TypeDef* TIMERx);


void TIMER_CntEventSourceConfig(MDR_TIMER_TypeDef* TIMERx, TIMER_Event_Src_TypeDef EventSource);
void TIMER_FilterSamplingConfig(MDR_TIMER_TypeDef* TIMERx, TIMER_Filter_Sampl_Clk_TypeDef Prescaler);
void TIMER_CounterModeConfig(MDR_TIMER_TypeDef* TIMERx, TIMER_Counter_Mode_TypeDef Mode);
void TIMER_SetCounterDirection(MDR_TIMER_TypeDef* TIMERx, TIMER_Counter_Dir_TypeDef Direction);
void TIMER_ETRInputConfig(MDR_TIMER_TypeDef* TIMERx, TIMER_ETR_Prescaler_TypeDef Prescaler, TIMER_ETR_Polarity_TypeDef Polarity, TIMER_Filter_Config_TypeDef Filter);
void TIMER_ETRFilterConfig(MDR_TIMER_TypeDef* TIMERx, TIMER_Filter_Config_TypeDef Filter);
void TIMER_ETRPrescalerConfig(MDR_TIMER_TypeDef* TIMERx, TIMER_ETR_Prescaler_TypeDef Prescaler);
void TIMER_ETRPolarityConfig(MDR_TIMER_TypeDef* TIMERx, TIMER_ETR_Polarity_TypeDef Polarity);
void TIMER_BRKPolarityConfig(MDR_TIMER_TypeDef* TIMERx, TIMER_BRK_Polarity_TypeDef Polarity);
TIMER_Counter_Dir_TypeDef TIMER_GetCounterDirection(MDR_TIMER_TypeDef* TIMERx);
FlagStatus TIMER_GetCntWriteComplete(MDR_TIMER_TypeDef* TIMERx);

void TIMER_ChnInit(MDR_TIMER_TypeDef* TIMERx, const TIMER_ChnInitTypeDef* TIMER_ChnInitStruct);
void TIMER_ChnStructInit(TIMER_ChnInitTypeDef* TIMER_ChnInitStruct);






    void TIMER_SetChnCompare(MDR_TIMER_TypeDef* TIMERx, TIMER_Channel_Number_TypeDef Channel, uint16_t Compare);
    void TIMER_ChnCompareConfig(MDR_TIMER_TypeDef* TIMERx, TIMER_Channel_Number_TypeDef Channel, uint16_t Compare, TIMER_CH_CCR_Update_Mode_TypeDef UpdateMode);
    uint16_t TIMER_GetChnCapture(MDR_TIMER_TypeDef* TIMERx, TIMER_Channel_Number_TypeDef Channel);







    void TIMER_SetChnCompare1(MDR_TIMER_TypeDef* TIMERx, TIMER_Channel_Number_TypeDef Channel, uint16_t Compare);
    void TIMER_ChnCompare1Config(MDR_TIMER_TypeDef* TIMERx, TIMER_Channel_Number_TypeDef Channel, uint16_t Compare, TIMER_CH_CCR_Update_Mode_TypeDef UpdateMode);
    uint16_t TIMER_GetChnCapture1(MDR_TIMER_TypeDef* TIMERx, TIMER_Channel_Number_TypeDef Channel);


void TIMER_ChnETR_Cmd(MDR_TIMER_TypeDef* TIMERx, TIMER_Channel_Number_TypeDef Channel, FunctionalState NewState);
void TIMER_ChnETRResetConfig(MDR_TIMER_TypeDef* TIMERx, TIMER_Channel_Number_TypeDef Channel, TIMER_CH_ETR_RESET_TypeDef NewState);
void TIMER_ChnBRKResetConfig(MDR_TIMER_TypeDef* TIMERx, TIMER_Channel_Number_TypeDef Channel, TIMER_CH_BRK_RESET_TypeDef NewState);
void TIMER_ChnREFFormatConfig(MDR_TIMER_TypeDef* TIMERx, TIMER_Channel_Number_TypeDef Channel, TIMER_CH_REF_Format_TypeDef Format);
void TIMER_ChnCapturePrescalerConfig(MDR_TIMER_TypeDef* TIMERx, TIMER_Channel_Number_TypeDef Channel, TIMER_CH_Prescaler_TypeDef Prescaler);
void TIMER_ChnEventSourceConfig(MDR_TIMER_TypeDef* TIMERx, TIMER_Channel_Number_TypeDef Channel, TIMER_CH_Event_Src_TypeDef EventSource);
void TIMER_ChnFilterConfig(MDR_TIMER_TypeDef* TIMERx, TIMER_Channel_Number_TypeDef Channel, TIMER_Filter_Config_TypeDef Filter);
FlagStatus TIMER_GetChnWriteComplete(MDR_TIMER_TypeDef* TIMERx, TIMER_Channel_Number_TypeDef Channel);
void TIMER_ChnCCR1_EventSourceConfig(MDR_TIMER_TypeDef* TIMERx, TIMER_Channel_Number_TypeDef Channel, TIMER_CH_CCR1_Event_Src_TypeDef EventSource);
void TIMER_ChnCCR1_Cmd(MDR_TIMER_TypeDef* TIMERx, TIMER_Channel_Number_TypeDef Channel, FunctionalState NewState);

void TIMER_ChnOutInit(MDR_TIMER_TypeDef* TIMERx, const TIMER_ChnOutInitTypeDef* TIMER_ChnOutInitStruct);
void TIMER_ChnOutStructInit(TIMER_ChnOutInitTypeDef* TIMER_ChnOutInitStruct);
void TIMER_ChnOutConfig(MDR_TIMER_TypeDef* TIMERx, TIMER_Channel_Number_TypeDef Channel, TIMER_CH_OUT_Src_TypeDef OutSource, TIMER_CH_OUT_Mode_TypeDef Mode, TIMER_CH_OUT_Polarity_TypeDef Polarity);
void TIMER_ChnOutSourceConfig(MDR_TIMER_TypeDef* TIMERx, TIMER_Channel_Number_TypeDef Channel, TIMER_CH_OUT_Src_TypeDef OutSource);
void TIMER_ChnOutModeConfig(MDR_TIMER_TypeDef* TIMERx, TIMER_Channel_Number_TypeDef Channel, TIMER_CH_OUT_Mode_TypeDef Mode);
void TIMER_ChnOutPolarityConfig(MDR_TIMER_TypeDef* TIMERx, TIMER_Channel_Number_TypeDef Channel, TIMER_CH_OUT_Polarity_TypeDef Polarity);
void TIMER_ChnNOutConfig(MDR_TIMER_TypeDef* TIMERx, TIMER_Channel_Number_TypeDef Channel, TIMER_CH_OUT_Src_TypeDef OutSource, TIMER_CH_OUT_Mode_TypeDef Mode, TIMER_CH_OUT_Polarity_TypeDef Polarity);
void TIMER_ChnNOutSourceConfig(MDR_TIMER_TypeDef* TIMERx, TIMER_Channel_Number_TypeDef Channel, TIMER_CH_OUT_Src_TypeDef OutSource);
void TIMER_ChnNOutModeConfig(MDR_TIMER_TypeDef* TIMERx, TIMER_Channel_Number_TypeDef Channel, TIMER_CH_OUT_Mode_TypeDef Mode);
void TIMER_ChnNOutPolarityConfig(MDR_TIMER_TypeDef* TIMERx, TIMER_Channel_Number_TypeDef Channel, TIMER_CH_OUT_Polarity_TypeDef Polarity);
void TIMER_ChnOutDTGConfig(MDR_TIMER_TypeDef* TIMERx, TIMER_Channel_Number_TypeDef Channel, uint32_t MainPrescaler, uint32_t AuxPrescaler, TIMER_CH_DTG_Clk_Src_TypeDef ClockSource);

uint32_t TIMER_GetStatus(MDR_TIMER_TypeDef* TIMERx);
FlagStatus TIMER_GetFlagStatus(MDR_TIMER_TypeDef* TIMERx, TIMER_Status_Flags_TypeDef Flag);
void TIMER_ClearFlag(MDR_TIMER_TypeDef* TIMERx, uint32_t Flags);



    void TIMER_DMACmd(MDR_TIMER_TypeDef* TIMERx, uint32_t TIMER_DMASource, FunctionalState NewState);


void TIMER_ITConfig(MDR_TIMER_TypeDef* TIMERx, uint32_t TIMER_IT, FunctionalState NewState);
ITStatus TIMER_GetITStatus(MDR_TIMER_TypeDef* TIMERx, TIMER_Status_Flags_TypeDef TIMER_IT);

void TIMER_BRGInit(MDR_TIMER_TypeDef* TIMERx, TIMER_Clock_BRG_TypeDef TIMER_BRG);
# 19 "main.c" 2

# 1 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stddef.h" 1 3
# 38 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stddef.h" 3
  typedef signed int ptrdiff_t;
# 53 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stddef.h" 3
    typedef unsigned int size_t;
# 71 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stddef.h" 3
      typedef unsigned short wchar_t;
# 21 "main.c" 2
# 1 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\string.h" 1 3
# 58 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\string.h" 3
extern __attribute__((__nothrow__)) void *memcpy(void * __restrict ,
                    const void * __restrict , size_t ) __attribute__((__nonnull__(1,2)));






extern __attribute__((__nothrow__)) void *memmove(void * ,
                    const void * , size_t ) __attribute__((__nonnull__(1,2)));
# 77 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\string.h" 3
extern __attribute__((__nothrow__)) char *strcpy(char * __restrict , const char * __restrict ) __attribute__((__nonnull__(1,2)));






extern __attribute__((__nothrow__)) char *strncpy(char * __restrict , const char * __restrict , size_t ) __attribute__((__nonnull__(1,2)));
# 93 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\string.h" 3
extern __attribute__((__nothrow__)) char *strcat(char * __restrict , const char * __restrict ) __attribute__((__nonnull__(1,2)));






extern __attribute__((__nothrow__)) char *strncat(char * __restrict , const char * __restrict , size_t ) __attribute__((__nonnull__(1,2)));
# 117 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\string.h" 3
extern __attribute__((__nothrow__)) int memcmp(const void * , const void * , size_t ) __attribute__((__nonnull__(1,2)));







extern __attribute__((__nothrow__)) int strcmp(const char * , const char * ) __attribute__((__nonnull__(1,2)));






extern __attribute__((__nothrow__)) int strncmp(const char * , const char * , size_t ) __attribute__((__nonnull__(1,2)));
# 141 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\string.h" 3
extern __attribute__((__nothrow__)) int strcasecmp(const char * , const char * ) __attribute__((__nonnull__(1,2)));







extern __attribute__((__nothrow__)) int strncasecmp(const char * , const char * , size_t ) __attribute__((__nonnull__(1,2)));
# 158 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\string.h" 3
extern __attribute__((__nothrow__)) int strcoll(const char * , const char * ) __attribute__((__nonnull__(1,2)));
# 169 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\string.h" 3
extern __attribute__((__nothrow__)) size_t strxfrm(char * __restrict , const char * __restrict , size_t ) __attribute__((__nonnull__(2)));
# 193 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\string.h" 3
extern __attribute__((__nothrow__)) void *memchr(const void * , int , size_t ) __attribute__((__nonnull__(1)));
# 209 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\string.h" 3
extern __attribute__((__nothrow__)) char *strchr(const char * , int ) __attribute__((__nonnull__(1)));
# 218 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\string.h" 3
extern __attribute__((__nothrow__)) size_t strcspn(const char * , const char * ) __attribute__((__nonnull__(1,2)));
# 232 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\string.h" 3
extern __attribute__((__nothrow__)) char *strpbrk(const char * , const char * ) __attribute__((__nonnull__(1,2)));
# 247 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\string.h" 3
extern __attribute__((__nothrow__)) char *strrchr(const char * , int ) __attribute__((__nonnull__(1)));
# 257 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\string.h" 3
extern __attribute__((__nothrow__)) size_t strspn(const char * , const char * ) __attribute__((__nonnull__(1,2)));
# 270 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\string.h" 3
extern __attribute__((__nothrow__)) char *strstr(const char * , const char * ) __attribute__((__nonnull__(1,2)));
# 280 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\string.h" 3
extern __attribute__((__nothrow__)) char *strtok(char * __restrict , const char * __restrict ) __attribute__((__nonnull__(2)));
extern __attribute__((__nothrow__)) char *_strtok_r(char * , const char * , char ** ) __attribute__((__nonnull__(2,3)));
# 321 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\string.h" 3
extern __attribute__((__nothrow__)) void *memset(void * , int , size_t ) __attribute__((__nonnull__(1)));





extern __attribute__((__nothrow__)) char *strerror(int );







extern __attribute__((__nothrow__)) size_t strlen(const char * ) __attribute__((__nonnull__(1)));






extern __attribute__((__nothrow__)) size_t strlcpy(char * , const char * , size_t ) __attribute__((__nonnull__(1,2)));
# 362 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\string.h" 3
extern __attribute__((__nothrow__)) size_t strlcat(char * , const char * , size_t ) __attribute__((__nonnull__(1,2)));
# 388 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\string.h" 3
extern __attribute__((__nothrow__)) void _membitcpybl(void * , const void * , int , int , size_t ) __attribute__((__nonnull__(1,2)));
extern __attribute__((__nothrow__)) void _membitcpybb(void * , const void * , int , int , size_t ) __attribute__((__nonnull__(1,2)));
extern __attribute__((__nothrow__)) void _membitcpyhl(void * , const void * , int , int , size_t ) __attribute__((__nonnull__(1,2)));
extern __attribute__((__nothrow__)) void _membitcpyhb(void * , const void * , int , int , size_t ) __attribute__((__nonnull__(1,2)));
extern __attribute__((__nothrow__)) void _membitcpywl(void * , const void * , int , int , size_t ) __attribute__((__nonnull__(1,2)));
extern __attribute__((__nothrow__)) void _membitcpywb(void * , const void * , int , int , size_t ) __attribute__((__nonnull__(1,2)));
extern __attribute__((__nothrow__)) void _membitmovebl(void * , const void * , int , int , size_t ) __attribute__((__nonnull__(1,2)));
extern __attribute__((__nothrow__)) void _membitmovebb(void * , const void * , int , int , size_t ) __attribute__((__nonnull__(1,2)));
extern __attribute__((__nothrow__)) void _membitmovehl(void * , const void * , int , int , size_t ) __attribute__((__nonnull__(1,2)));
extern __attribute__((__nothrow__)) void _membitmovehb(void * , const void * , int , int , size_t ) __attribute__((__nonnull__(1,2)));
extern __attribute__((__nothrow__)) void _membitmovewl(void * , const void * , int , int , size_t ) __attribute__((__nonnull__(1,2)));
extern __attribute__((__nothrow__)) void _membitmovewb(void * , const void * , int , int , size_t ) __attribute__((__nonnull__(1,2)));
# 22 "main.c" 2
# 1 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 1 3
# 96 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 3
typedef struct div_t { int quot, rem; } div_t;

typedef struct ldiv_t { long int quot, rem; } ldiv_t;


typedef struct lldiv_t { long long quot, rem; } lldiv_t;
# 139 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 3
extern __attribute__((__nothrow__)) int __aeabi_MB_CUR_MAX(void);
# 158 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 3
extern __attribute__((__nothrow__)) double atof(const char * ) __attribute__((__nonnull__(1)));





extern __attribute__((__nothrow__)) int atoi(const char * ) __attribute__((__nonnull__(1)));





extern __attribute__((__nothrow__)) long int atol(const char * ) __attribute__((__nonnull__(1)));






extern __attribute__((__nothrow__)) long long atoll(const char * ) __attribute__((__nonnull__(1)));







extern __attribute__((__nothrow__)) double strtod(const char * __restrict , char ** __restrict ) __attribute__((__nonnull__(1)));
# 206 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 3
extern __attribute__((__nothrow__)) float strtof(const char * __restrict , char ** __restrict ) __attribute__((__nonnull__(1)));
extern __attribute__((__nothrow__)) long double strtold(const char * __restrict , char ** __restrict ) __attribute__((__nonnull__(1)));




extern __attribute__((__nothrow__)) long int strtol(const char * __restrict ,
                        char ** __restrict , int ) __attribute__((__nonnull__(1)));
# 243 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 3
extern __attribute__((__nothrow__)) unsigned long int strtoul(const char * __restrict ,
                                       char ** __restrict , int ) __attribute__((__nonnull__(1)));
# 275 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 3
extern __attribute__((__nothrow__)) long long strtoll(const char * __restrict ,
                                  char ** __restrict , int )
                          __attribute__((__nonnull__(1)));






extern __attribute__((__nothrow__)) unsigned long long strtoull(const char * __restrict ,
                                            char ** __restrict , int )
                                   __attribute__((__nonnull__(1)));






extern __attribute__((__nothrow__)) int rand(void);
# 303 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 3
extern __attribute__((__nothrow__)) void srand(unsigned int );
# 313 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 3
struct _rand_state { int __x[57]; };
extern __attribute__((__nothrow__)) int _rand_r(struct _rand_state *);
extern __attribute__((__nothrow__)) void _srand_r(struct _rand_state *, unsigned int);
struct _ANSI_rand_state { int __x[1]; };
extern __attribute__((__nothrow__)) int _ANSI_rand_r(struct _ANSI_rand_state *);
extern __attribute__((__nothrow__)) void _ANSI_srand_r(struct _ANSI_rand_state *, unsigned int);





extern __attribute__((__nothrow__)) void *calloc(size_t , size_t );





extern __attribute__((__nothrow__)) void free(void * );







extern __attribute__((__nothrow__)) void *malloc(size_t );





extern __attribute__((__nothrow__)) void *realloc(void * , size_t );
# 374 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 3
typedef int (*__heapprt)(void *, char const *, ...);
extern __attribute__((__nothrow__)) void __heapstats(int (* )(void * ,
                                           char const * , ...),
                        void * ) __attribute__((__nonnull__(1)));
# 390 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 3
extern __attribute__((__nothrow__)) int __heapvalid(int (* )(void * ,
                                           char const * , ...),
                       void * , int ) __attribute__((__nonnull__(1)));
# 411 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 3
extern __attribute__((__nothrow__)) __attribute__((__noreturn__)) void abort(void);
# 422 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 3
extern __attribute__((__nothrow__)) int atexit(void (* )(void)) __attribute__((__nonnull__(1)));
# 444 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 3
extern __attribute__((__nothrow__)) __attribute__((__noreturn__)) void exit(int );
# 460 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 3
extern __attribute__((__nothrow__)) __attribute__((__noreturn__)) void _Exit(int );
# 471 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 3
extern __attribute__((__nothrow__)) char *getenv(const char * ) __attribute__((__nonnull__(1)));
# 484 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 3
extern __attribute__((__nothrow__)) int system(const char * );
# 497 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 3
extern void *bsearch(const void * , const void * ,
              size_t , size_t ,
              int (* )(const void *, const void *)) __attribute__((__nonnull__(1,2,5)));
# 532 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 3
extern void qsort(void * , size_t , size_t ,
           int (* )(const void *, const void *)) __attribute__((__nonnull__(1,4)));
# 560 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 3
extern __attribute__((__nothrow__)) __attribute__((__const__)) int abs(int );






extern __attribute__((__nothrow__)) __attribute__((__const__)) div_t div(int , int );
# 579 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 3
extern __attribute__((__nothrow__)) __attribute__((__const__)) long int labs(long int );
# 589 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 3
extern __attribute__((__nothrow__)) __attribute__((__const__)) ldiv_t ldiv(long int , long int );
# 610 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 3
extern __attribute__((__nothrow__)) __attribute__((__const__)) long long llabs(long long );
# 620 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 3
extern __attribute__((__nothrow__)) __attribute__((__const__)) lldiv_t lldiv(long long , long long );
# 644 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 3
typedef struct __sdiv32by16 { long quot, rem; } __sdiv32by16;
typedef struct __udiv32by16 { unsigned long quot, rem; } __udiv32by16;

typedef struct __sdiv64by32 { long rem, quot; } __sdiv64by32;

__attribute__((__value_in_regs__)) extern __attribute__((__nothrow__)) __attribute__((__const__)) __sdiv32by16 __rt_sdiv32by16(
     int ,
     short int );



__attribute__((__value_in_regs__)) extern __attribute__((__nothrow__)) __attribute__((__const__)) __udiv32by16 __rt_udiv32by16(
     unsigned int ,
     unsigned short );



__attribute__((__value_in_regs__)) extern __attribute__((__nothrow__)) __attribute__((__const__)) __sdiv64by32 __rt_sdiv64by32(
     int , unsigned int ,
     int );







extern __attribute__((__nothrow__)) unsigned int __fp_status(unsigned int , unsigned int );
# 705 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 3
extern __attribute__((__nothrow__)) int mblen(const char * , size_t );
# 720 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 3
extern __attribute__((__nothrow__)) int mbtowc(wchar_t * __restrict ,
                   const char * __restrict , size_t );
# 739 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 3
extern __attribute__((__nothrow__)) int wctomb(char * , wchar_t );
# 761 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 3
extern __attribute__((__nothrow__)) size_t mbstowcs(wchar_t * __restrict ,
                      const char * __restrict , size_t ) __attribute__((__nonnull__(2)));
# 779 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 3
extern __attribute__((__nothrow__)) size_t wcstombs(char * __restrict ,
                      const wchar_t * __restrict , size_t ) __attribute__((__nonnull__(2)));
# 798 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdlib.h" 3
extern __attribute__((__nothrow__)) void __use_realtime_heap(void);
extern __attribute__((__nothrow__)) void __use_realtime_division(void);
extern __attribute__((__nothrow__)) void __use_two_region_memory(void);
extern __attribute__((__nothrow__)) void __use_no_heap(void);
extern __attribute__((__nothrow__)) void __use_no_heap_region(void);

extern __attribute__((__nothrow__)) char const *__C_library_version_string(void);
extern __attribute__((__nothrow__)) int __C_library_version_number(void);
# 23 "main.c" 2
# 1 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 1 3
# 68 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
    typedef __builtin_va_list __va_list;
# 87 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
typedef struct __fpos_t_struct {
    unsigned long long int __pos;





    struct {
        unsigned int __state1, __state2;
    } __mbstate;
} fpos_t;
# 108 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
typedef struct __FILE FILE;
# 119 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
struct __FILE {
    union {
        long __FILE_alignment;



        char __FILE_size[84];

    } __FILE_opaque;
};
# 138 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
extern FILE __stdin, __stdout, __stderr;
extern FILE *__aeabi_stdin, *__aeabi_stdout, *__aeabi_stderr;
# 224 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) int remove(const char * ) __attribute__((__nonnull__(1)));







extern __attribute__((__nothrow__)) int rename(const char * , const char * ) __attribute__((__nonnull__(1,2)));
# 243 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) FILE *tmpfile(void);






extern __attribute__((__nothrow__)) char *tmpnam(char * );
# 265 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) int fclose(FILE * ) __attribute__((__nonnull__(1)));
# 275 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) int fflush(FILE * );
# 285 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) FILE *fopen(const char * __restrict ,
                           const char * __restrict ) __attribute__((__nonnull__(1,2)));
# 329 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) FILE *freopen(const char * __restrict ,
                    const char * __restrict ,
                    FILE * __restrict ) __attribute__((__nonnull__(2,3)));
# 342 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) void setbuf(FILE * __restrict ,
                    char * __restrict ) __attribute__((__nonnull__(1)));






extern __attribute__((__nothrow__)) int setvbuf(FILE * __restrict ,
                   char * __restrict ,
                   int , size_t ) __attribute__((__nonnull__(1)));
# 370 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
#pragma __printf_args
extern __attribute__((__nothrow__)) int fprintf(FILE * __restrict ,
                    const char * __restrict , ...) __attribute__((__nonnull__(1,2)));
# 393 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
#pragma __printf_args
extern __attribute__((__nothrow__)) int _fprintf(FILE * __restrict ,
                     const char * __restrict , ...) __attribute__((__nonnull__(1,2)));





#pragma __printf_args
extern __attribute__((__nothrow__)) int printf(const char * __restrict , ...) __attribute__((__nonnull__(1)));






#pragma __printf_args
extern __attribute__((__nothrow__)) int _printf(const char * __restrict , ...) __attribute__((__nonnull__(1)));





#pragma __printf_args
extern __attribute__((__nothrow__)) int sprintf(char * __restrict , const char * __restrict , ...) __attribute__((__nonnull__(1,2)));








#pragma __printf_args
extern __attribute__((__nothrow__)) int _sprintf(char * __restrict , const char * __restrict , ...) __attribute__((__nonnull__(1,2)));






#pragma __printf_args
extern __attribute__((__nothrow__)) int __ARM_snprintf(char * __restrict , size_t ,
                     const char * __restrict , ...) __attribute__((__nonnull__(3)));


#pragma __printf_args
extern __attribute__((__nothrow__)) int snprintf(char * __restrict , size_t ,
                     const char * __restrict , ...) __attribute__((__nonnull__(3)));
# 460 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
#pragma __printf_args
extern __attribute__((__nothrow__)) int _snprintf(char * __restrict , size_t ,
                      const char * __restrict , ...) __attribute__((__nonnull__(3)));





#pragma __scanf_args
extern __attribute__((__nothrow__)) int fscanf(FILE * __restrict ,
                    const char * __restrict , ...) __attribute__((__nonnull__(1,2)));
# 503 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
#pragma __scanf_args
extern __attribute__((__nothrow__)) int _fscanf(FILE * __restrict ,
                     const char * __restrict , ...) __attribute__((__nonnull__(1,2)));





#pragma __scanf_args
extern __attribute__((__nothrow__)) int scanf(const char * __restrict , ...) __attribute__((__nonnull__(1)));








#pragma __scanf_args
extern __attribute__((__nothrow__)) int _scanf(const char * __restrict , ...) __attribute__((__nonnull__(1)));





#pragma __scanf_args
extern __attribute__((__nothrow__)) int sscanf(const char * __restrict ,
                    const char * __restrict , ...) __attribute__((__nonnull__(1,2)));
# 541 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
#pragma __scanf_args
extern __attribute__((__nothrow__)) int _sscanf(const char * __restrict ,
                     const char * __restrict , ...) __attribute__((__nonnull__(1,2)));







extern __attribute__((__nothrow__)) int vfscanf(FILE * __restrict , const char * __restrict , __va_list) __attribute__((__nonnull__(1,2)));
extern __attribute__((__nothrow__)) int vscanf(const char * __restrict , __va_list) __attribute__((__nonnull__(1)));
extern __attribute__((__nothrow__)) int vsscanf(const char * __restrict , const char * __restrict , __va_list) __attribute__((__nonnull__(1,2)));

extern __attribute__((__nothrow__)) int _vfscanf(FILE * __restrict , const char * __restrict , __va_list) __attribute__((__nonnull__(1,2)));
extern __attribute__((__nothrow__)) int _vscanf(const char * __restrict , __va_list) __attribute__((__nonnull__(1)));
extern __attribute__((__nothrow__)) int _vsscanf(const char * __restrict , const char * __restrict , __va_list) __attribute__((__nonnull__(1,2)));
extern __attribute__((__nothrow__)) int __ARM_vsscanf(const char * __restrict , const char * __restrict , __va_list) __attribute__((__nonnull__(1,2)));

extern __attribute__((__nothrow__)) int vprintf(const char * __restrict , __va_list ) __attribute__((__nonnull__(1)));







extern __attribute__((__nothrow__)) int _vprintf(const char * __restrict , __va_list ) __attribute__((__nonnull__(1)));





extern __attribute__((__nothrow__)) int vfprintf(FILE * __restrict ,
                    const char * __restrict , __va_list ) __attribute__((__nonnull__(1,2)));
# 584 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) int vsprintf(char * __restrict ,
                     const char * __restrict , __va_list ) __attribute__((__nonnull__(1,2)));
# 594 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) int __ARM_vsnprintf(char * __restrict , size_t ,
                     const char * __restrict , __va_list ) __attribute__((__nonnull__(3)));

extern __attribute__((__nothrow__)) int vsnprintf(char * __restrict , size_t ,
                     const char * __restrict , __va_list ) __attribute__((__nonnull__(3)));
# 609 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) int _vsprintf(char * __restrict ,
                      const char * __restrict , __va_list ) __attribute__((__nonnull__(1,2)));





extern __attribute__((__nothrow__)) int _vfprintf(FILE * __restrict ,
                     const char * __restrict , __va_list ) __attribute__((__nonnull__(1,2)));





extern __attribute__((__nothrow__)) int _vsnprintf(char * __restrict , size_t ,
                      const char * __restrict , __va_list ) __attribute__((__nonnull__(3)));
# 635 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
#pragma __printf_args
extern __attribute__((__nothrow__)) int __ARM_asprintf(char ** , const char * __restrict , ...) __attribute__((__nonnull__(2)));
extern __attribute__((__nothrow__)) int __ARM_vasprintf(char ** , const char * __restrict , __va_list ) __attribute__((__nonnull__(2)));
# 649 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) int fgetc(FILE * ) __attribute__((__nonnull__(1)));
# 659 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) char *fgets(char * __restrict , int ,
                    FILE * __restrict ) __attribute__((__nonnull__(1,3)));
# 673 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) int fputc(int , FILE * ) __attribute__((__nonnull__(2)));
# 683 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) int fputs(const char * __restrict , FILE * __restrict ) __attribute__((__nonnull__(1,2)));






extern __attribute__((__nothrow__)) int getc(FILE * ) __attribute__((__nonnull__(1)));
# 704 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
    extern __attribute__((__nothrow__)) int (getchar)(void);
# 713 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) char *gets(char * ) __attribute__((__nonnull__(1)));
# 725 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) int putc(int , FILE * ) __attribute__((__nonnull__(2)));
# 737 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
    extern __attribute__((__nothrow__)) int (putchar)(int );






extern __attribute__((__nothrow__)) int puts(const char * ) __attribute__((__nonnull__(1)));







extern __attribute__((__nothrow__)) int ungetc(int , FILE * ) __attribute__((__nonnull__(2)));
# 778 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) size_t fread(void * __restrict ,
                    size_t , size_t , FILE * __restrict ) __attribute__((__nonnull__(1,4)));
# 794 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) size_t __fread_bytes_avail(void * __restrict ,
                    size_t , FILE * __restrict ) __attribute__((__nonnull__(1,3)));
# 810 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) size_t fwrite(const void * __restrict ,
                    size_t , size_t , FILE * __restrict ) __attribute__((__nonnull__(1,4)));
# 822 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) int fgetpos(FILE * __restrict , fpos_t * __restrict ) __attribute__((__nonnull__(1,2)));
# 833 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) int fseek(FILE * , long int , int ) __attribute__((__nonnull__(1)));
# 850 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) int fsetpos(FILE * __restrict , const fpos_t * __restrict ) __attribute__((__nonnull__(1,2)));
# 863 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) long int ftell(FILE * ) __attribute__((__nonnull__(1)));
# 877 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) void rewind(FILE * ) __attribute__((__nonnull__(1)));
# 886 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) void clearerr(FILE * ) __attribute__((__nonnull__(1)));







extern __attribute__((__nothrow__)) int feof(FILE * ) __attribute__((__nonnull__(1)));




extern __attribute__((__nothrow__)) int ferror(FILE * ) __attribute__((__nonnull__(1)));




extern __attribute__((__nothrow__)) void perror(const char * );
# 917 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdio.h" 3
extern __attribute__((__nothrow__)) int _fisatty(FILE * ) __attribute__((__nonnull__(1)));



extern __attribute__((__nothrow__)) void __use_no_semihosting_swi(void);
extern __attribute__((__nothrow__)) void __use_no_semihosting(void);
# 24 "main.c" 2
# 1 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdarg.h" 1 3
# 40 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdarg.h" 3
  typedef __builtin_va_list va_list;
# 134 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\stdarg.h" 3
     typedef va_list __gnuc_va_list;
# 25 "main.c" 2
# 1 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\math.h" 1 3
# 157 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\math.h" 3
extern __attribute__((__pcs__("aapcs"))) unsigned __ARM_dcmp4(double , double );
extern __attribute__((__pcs__("aapcs"))) unsigned __ARM_fcmp4(float , float );







extern __attribute__((__nothrow__)) __attribute__((__pcs__("aapcs"))) int __ARM_fpclassifyf(float );
extern __attribute__((__nothrow__)) __attribute__((__pcs__("aapcs"))) int __ARM_fpclassify(double );



static __inline __attribute__((__nothrow__)) __attribute__((__pcs__("aapcs"))) int __ARM_isfinitef(float __x)
{
    return (((*(unsigned *)&(__x)) >> 23) & 0xff) != 0xff;
}
static __inline __attribute__((__nothrow__)) __attribute__((__pcs__("aapcs"))) int __ARM_isfinite(double __x)
{
    return (((*(1 + (unsigned *)&(__x))) >> 20) & 0x7ff) != 0x7ff;
}



static __inline __attribute__((__nothrow__)) __attribute__((__pcs__("aapcs"))) int __ARM_isinff(float __x)
{
    return ((*(unsigned *)&(__x)) << 1) == 0xff000000;
}
static __inline __attribute__((__nothrow__)) __attribute__((__pcs__("aapcs"))) int __ARM_isinf(double __x)
{
    return (((*(1 + (unsigned *)&(__x))) << 1) == 0xffe00000) && ((*(unsigned *)&(__x)) == 0);
}



static __inline __attribute__((__nothrow__)) __attribute__((__pcs__("aapcs"))) int __ARM_islessgreaterf(float __x, float __y)
{
    unsigned __f = __ARM_fcmp4(__x, __y) >> 28;
    return (__f == 8) || (__f == 2);
}
static __inline __attribute__((__nothrow__)) __attribute__((__pcs__("aapcs"))) int __ARM_islessgreater(double __x, double __y)
{
    unsigned __f = __ARM_dcmp4(__x, __y) >> 28;
    return (__f == 8) || (__f == 2);
}





static __inline __attribute__((__nothrow__)) __attribute__((__pcs__("aapcs"))) int __ARM_isnanf(float __x)
{
    return (0x7f800000 - ((*(unsigned *)&(__x)) & 0x7fffffff)) >> 31;
}
static __inline __attribute__((__nothrow__)) __attribute__((__pcs__("aapcs"))) int __ARM_isnan(double __x)
{
    unsigned __xf = (*(1 + (unsigned *)&(__x))) | (((*(unsigned *)&(__x)) == 0) ? 0 : 1);
    return (0x7ff00000 - (__xf & 0x7fffffff)) >> 31;
}



static __inline __attribute__((__nothrow__)) __attribute__((__pcs__("aapcs"))) int __ARM_isnormalf(float __x)
{
    unsigned __xe = ((*(unsigned *)&(__x)) >> 23) & 0xff;
    return (__xe != 0xff) && (__xe != 0);
}
static __inline __attribute__((__nothrow__)) __attribute__((__pcs__("aapcs"))) int __ARM_isnormal(double __x)
{
    unsigned __xe = ((*(1 + (unsigned *)&(__x))) >> 20) & 0x7ff;
    return (__xe != 0x7ff) && (__xe != 0);
}



static __inline __attribute__((__nothrow__)) __attribute__((__pcs__("aapcs"))) int __ARM_signbitf(float __x)
{
    return (*(unsigned *)&(__x)) >> 31;
}
static __inline __attribute__((__nothrow__)) __attribute__((__pcs__("aapcs"))) int __ARM_signbit(double __x)
{
    return (*(1 + (unsigned *)&(__x))) >> 31;
}
# 266 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\math.h" 3
  typedef float float_t;
  typedef double double_t;
# 282 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\math.h" 3
extern const int math_errhandling;







extern __attribute__((__nothrow__)) double acos(double );



extern __attribute__((__nothrow__)) double asin(double );





extern __attribute__((__nothrow__)) __attribute__((__const__)) double atan(double );



extern __attribute__((__nothrow__)) double atan2(double , double );





extern __attribute__((__nothrow__)) double cos(double );




extern __attribute__((__nothrow__)) double sin(double );





extern void __use_accurate_range_reduction(void);



extern __attribute__((__nothrow__)) double tan(double );





extern __attribute__((__nothrow__)) double cosh(double );




extern __attribute__((__nothrow__)) double sinh(double );






extern __attribute__((__nothrow__)) __attribute__((__const__)) double tanh(double );



extern __attribute__((__nothrow__)) double exp(double );






extern __attribute__((__nothrow__)) double frexp(double , int * ) __attribute__((__nonnull__(2)));







extern __attribute__((__nothrow__)) double ldexp(double , int );




extern __attribute__((__nothrow__)) double log(double );





extern __attribute__((__nothrow__)) double log10(double );



extern __attribute__((__nothrow__)) double modf(double , double * ) __attribute__((__nonnull__(2)));





extern __attribute__((__nothrow__)) double pow(double , double );






extern __attribute__((__nothrow__)) double sqrt(double );
# 410 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\math.h" 3
    static __inline double _sqrt(double __x) { return sqrt(__x); }
# 427 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\math.h" 3
    static __inline float _sqrtf(float __x) { return (float)sqrt(__x); }







extern __attribute__((__nothrow__)) __attribute__((__const__)) double ceil(double );


extern __attribute__((__nothrow__)) __attribute__((__const__)) double fabs(double );



extern __attribute__((__nothrow__)) __attribute__((__const__)) double floor(double );



extern __attribute__((__nothrow__)) double fmod(double , double );
# 467 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\math.h" 3
extern __attribute__((__nothrow__)) double acosh(double );



extern __attribute__((__nothrow__)) double asinh(double );



extern __attribute__((__nothrow__)) double atanh(double );



extern __attribute__((__nothrow__)) double cbrt(double );



static __inline __attribute__((__nothrow__)) __attribute__((__const__)) double copysign(double __x, double __y)



{
    (*(1 + (unsigned *)&(__x))) = ((*(1 + (unsigned *)&(__x))) & 0x7fffffff) | ((*(1 + (unsigned *)&(__y))) & 0x80000000);
    return __x;
}
static __inline __attribute__((__nothrow__)) __attribute__((__const__)) float copysignf(float __x, float __y)



{
    (*(unsigned *)&(__x)) = ((*(unsigned *)&(__x)) & 0x7fffffff) | ((*(unsigned *)&(__y)) & 0x80000000);
    return __x;
}
extern __attribute__((__nothrow__)) double erf(double );



extern __attribute__((__nothrow__)) double erfc(double );



extern __attribute__((__nothrow__)) double expm1(double );
# 533 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\math.h" 3
extern __attribute__((__nothrow__)) double hypot(double , double );






extern __attribute__((__nothrow__)) int ilogb(double );



extern __attribute__((__nothrow__)) int ilogbf(float );



extern __attribute__((__nothrow__)) int ilogbl(long double );
# 646 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\math.h" 3
extern __attribute__((__nothrow__)) double lgamma (double );




extern __attribute__((__nothrow__)) double log1p(double );



extern __attribute__((__nothrow__)) double logb(double );



extern __attribute__((__nothrow__)) float logbf(float );



extern __attribute__((__nothrow__)) long double logbl(long double );



extern __attribute__((__nothrow__)) double nextafter(double , double );




extern __attribute__((__nothrow__)) float nextafterf(float , float );




extern __attribute__((__nothrow__)) long double nextafterl(long double , long double );




extern __attribute__((__nothrow__)) double nexttoward(double , long double );




extern __attribute__((__nothrow__)) float nexttowardf(float , long double );




extern __attribute__((__nothrow__)) long double nexttowardl(long double , long double );




extern __attribute__((__nothrow__)) double remainder(double , double );



extern __attribute__((__nothrow__)) __attribute__((__const__)) double rint(double );



extern __attribute__((__nothrow__)) double scalbln(double , long int );



extern __attribute__((__nothrow__)) float scalblnf(float , long int );



extern __attribute__((__nothrow__)) long double scalblnl(long double , long int );



extern __attribute__((__nothrow__)) double scalbn(double , int );



extern __attribute__((__nothrow__)) float scalbnf(float , int );



extern __attribute__((__nothrow__)) long double scalbnl(long double , int );
# 740 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\math.h" 3
extern __attribute__((__nothrow__)) __attribute__((__const__)) float fabsf(float);
static __inline __attribute__((__nothrow__)) __attribute__((__const__)) float _fabsf(float __f) { return fabsf(__f); }
extern __attribute__((__nothrow__)) float sinf(float );
extern __attribute__((__nothrow__)) float cosf(float );
extern __attribute__((__nothrow__)) float tanf(float );
extern __attribute__((__nothrow__)) float acosf(float );
extern __attribute__((__nothrow__)) float asinf(float );
extern __attribute__((__nothrow__)) float atanf(float );
extern __attribute__((__nothrow__)) float atan2f(float , float );
extern __attribute__((__nothrow__)) float sinhf(float );
extern __attribute__((__nothrow__)) float coshf(float );
extern __attribute__((__nothrow__)) float tanhf(float );
extern __attribute__((__nothrow__)) float expf(float );
extern __attribute__((__nothrow__)) float logf(float );
extern __attribute__((__nothrow__)) float log10f(float );
extern __attribute__((__nothrow__)) float powf(float , float );
extern __attribute__((__nothrow__)) float sqrtf(float );
extern __attribute__((__nothrow__)) float ldexpf(float , int );
extern __attribute__((__nothrow__)) float frexpf(float , int * ) __attribute__((__nonnull__(2)));
extern __attribute__((__nothrow__)) __attribute__((__const__)) float ceilf(float );
extern __attribute__((__nothrow__)) __attribute__((__const__)) float floorf(float );
extern __attribute__((__nothrow__)) float fmodf(float , float );
extern __attribute__((__nothrow__)) float modff(float , float * ) __attribute__((__nonnull__(2)));
# 780 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\math.h" 3
__attribute__((__nothrow__)) long double acosl(long double );
__attribute__((__nothrow__)) long double asinl(long double );
__attribute__((__nothrow__)) long double atanl(long double );
__attribute__((__nothrow__)) long double atan2l(long double , long double );
__attribute__((__nothrow__)) long double ceill(long double );
__attribute__((__nothrow__)) long double cosl(long double );
__attribute__((__nothrow__)) long double coshl(long double );
__attribute__((__nothrow__)) long double expl(long double );
__attribute__((__nothrow__)) long double fabsl(long double );
__attribute__((__nothrow__)) long double floorl(long double );
__attribute__((__nothrow__)) long double fmodl(long double , long double );
__attribute__((__nothrow__)) long double frexpl(long double , int* ) __attribute__((__nonnull__(2)));
__attribute__((__nothrow__)) long double ldexpl(long double , int );
__attribute__((__nothrow__)) long double logl(long double );
__attribute__((__nothrow__)) long double log10l(long double );
__attribute__((__nothrow__)) long double modfl(long double , long double * ) __attribute__((__nonnull__(2)));
__attribute__((__nothrow__)) long double powl(long double , long double );
__attribute__((__nothrow__)) long double sinl(long double );
__attribute__((__nothrow__)) long double sinhl(long double );
__attribute__((__nothrow__)) long double sqrtl(long double );
__attribute__((__nothrow__)) long double tanl(long double );
__attribute__((__nothrow__)) long double tanhl(long double );






extern __attribute__((__nothrow__)) float acoshf(float );
__attribute__((__nothrow__)) long double acoshl(long double );
extern __attribute__((__nothrow__)) float asinhf(float );
__attribute__((__nothrow__)) long double asinhl(long double );
extern __attribute__((__nothrow__)) float atanhf(float );
__attribute__((__nothrow__)) long double atanhl(long double );
__attribute__((__nothrow__)) long double copysignl(long double , long double );
extern __attribute__((__nothrow__)) float cbrtf(float );
__attribute__((__nothrow__)) long double cbrtl(long double );
extern __attribute__((__nothrow__)) float erff(float );
__attribute__((__nothrow__)) long double erfl(long double );
extern __attribute__((__nothrow__)) float erfcf(float );
__attribute__((__nothrow__)) long double erfcl(long double );
extern __attribute__((__nothrow__)) float expm1f(float );
__attribute__((__nothrow__)) long double expm1l(long double );
extern __attribute__((__nothrow__)) float log1pf(float );
__attribute__((__nothrow__)) long double log1pl(long double );
extern __attribute__((__nothrow__)) float hypotf(float , float );
__attribute__((__nothrow__)) long double hypotl(long double , long double );
extern __attribute__((__nothrow__)) float lgammaf(float );
__attribute__((__nothrow__)) long double lgammal(long double );
extern __attribute__((__nothrow__)) float remainderf(float , float );
__attribute__((__nothrow__)) long double remainderl(long double , long double );
extern __attribute__((__nothrow__)) float rintf(float );
__attribute__((__nothrow__)) long double rintl(long double );







extern __attribute__((__nothrow__)) double exp2(double );
extern __attribute__((__nothrow__)) float exp2f(float );
__attribute__((__nothrow__)) long double exp2l(long double );
extern __attribute__((__nothrow__)) double fdim(double , double );
extern __attribute__((__nothrow__)) float fdimf(float , float );
__attribute__((__nothrow__)) long double fdiml(long double , long double );
# 855 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\math.h" 3
extern __attribute__((__nothrow__)) double fma(double , double , double );
extern __attribute__((__nothrow__)) float fmaf(float , float , float );

static __inline __attribute__((__nothrow__)) long double fmal(long double __x, long double __y, long double __z) { return (long double)fma((double)__x, (double)__y, (double)__z); }


extern __attribute__((__nothrow__)) __attribute__((__const__)) double fmax(double , double );
extern __attribute__((__nothrow__)) __attribute__((__const__)) float fmaxf(float , float );
__attribute__((__nothrow__)) long double fmaxl(long double , long double );
extern __attribute__((__nothrow__)) __attribute__((__const__)) double fmin(double , double );
extern __attribute__((__nothrow__)) __attribute__((__const__)) float fminf(float , float );
__attribute__((__nothrow__)) long double fminl(long double , long double );
extern __attribute__((__nothrow__)) double log2(double );
extern __attribute__((__nothrow__)) float log2f(float );
__attribute__((__nothrow__)) long double log2l(long double );
extern __attribute__((__nothrow__)) long lrint(double );
extern __attribute__((__nothrow__)) long lrintf(float );

static __inline __attribute__((__nothrow__)) long lrintl(long double __x) { return lrint((double)__x); }


extern __attribute__((__nothrow__)) long long llrint(double );
extern __attribute__((__nothrow__)) long long llrintf(float );

static __inline __attribute__((__nothrow__)) long long llrintl(long double __x) { return llrint((double)__x); }


extern __attribute__((__nothrow__)) long lround(double );
extern __attribute__((__nothrow__)) long lroundf(float );

static __inline __attribute__((__nothrow__)) long lroundl(long double __x) { return lround((double)__x); }


extern __attribute__((__nothrow__)) long long llround(double );
extern __attribute__((__nothrow__)) long long llroundf(float );

static __inline __attribute__((__nothrow__)) long long llroundl(long double __x) { return llround((double)__x); }


extern __attribute__((__nothrow__)) __attribute__((__const__)) double nan(const char * );
extern __attribute__((__nothrow__)) __attribute__((__const__)) float nanf(const char * );

static __inline __attribute__((__nothrow__)) __attribute__((__const__)) long double nanl(const char *__t) { return (long double)nan(__t); }
# 908 "C:\\Keil_v5\\ARM\\ARMCLANG\\Bin\\..\\include\\math.h" 3
extern __attribute__((__nothrow__)) __attribute__((__const__)) double nearbyint(double );
extern __attribute__((__nothrow__)) __attribute__((__const__)) float nearbyintf(float );
__attribute__((__nothrow__)) long double nearbyintl(long double );
extern __attribute__((__nothrow__)) double remquo(double , double , int * );
extern __attribute__((__nothrow__)) float remquof(float , float , int * );

static __inline long double remquol(long double __x, long double __y, int *__q) { return (long double)remquo((double)__x, (double)__y, __q); }


extern __attribute__((__nothrow__)) __attribute__((__const__)) double round(double );
extern __attribute__((__nothrow__)) __attribute__((__const__)) float roundf(float );
__attribute__((__nothrow__)) long double roundl(long double );
extern __attribute__((__nothrow__)) double tgamma(double );
extern __attribute__((__nothrow__)) float tgammaf(float );
__attribute__((__nothrow__)) long double tgammal(long double );
extern __attribute__((__nothrow__)) __attribute__((__const__)) double trunc(double );
extern __attribute__((__nothrow__)) __attribute__((__const__)) float truncf(float );
__attribute__((__nothrow__)) long double truncl(long double );
# 26 "main.c" 2


# 1 "./CustomLibs/inc\\DMA_for_proj.h" 1



void SetupDMA();
# 29 "main.c" 2
# 1 "./CustomLibs/inc\\DAC_for_proj.h" 1



void SetupDAC();
void SetupTIM2();
void Set_DAC_Table(int freq);
# 30 "main.c" 2
# 1 "./CustomLibs/inc\\ADC_for_proj.h" 1



void SetupADC();
# 31 "main.c" 2
# 1 "./CustomLibs/inc\\SysCLK_for_proj.h" 1




void Setup_CPU_Clock(void);
void delayTick(uint32_t count);
# 32 "main.c" 2
# 1 "./CustomLibs/inc\\USB_for_proj.h" 1



void Setup_USB(void);
void VCom_Configuration(void);
# 33 "main.c" 2
# 1 "./CustomLibs/inc\\Command_system.h" 1



void execute_command();
# 34 "main.c" 2


# 1 "./CustomLibs/inc\\defines_for_proj.h" 1
# 37 "main.c" 2

int command_recived = 0;
static char Buffer[128];
char RcBuffer[128];
extern char RecBuf[128];

uint16_t ADC1_array_m[128];
uint16_t ADC1_array_a[128];


extern DMA_ChannelInitTypeDef sDMA_ADC1;
extern DMA_ChannelInitTypeDef sDMA_TIM2;
extern DMA_CtrlDataInitTypeDef sDMA_PriCtrlData_ADC1;
extern DMA_CtrlDataInitTypeDef sDMA_AltCtrlData_ADC1;
extern DMA_CtrlDataInitTypeDef sDMA_PriCtrlData_TIM2;
extern DMA_CtrlDataInitTypeDef sDMA_AltCtrlData_TIM2;



int main(void) {
 VCom_Configuration();


 SetupADC();
 SetupDMA();
 USB_CDC_Init((uint8_t *)Buffer, 1, SET);
 Setup_CPU_Clock();
 Setup_USB();
 Set_DAC_Table(100);
 SetupDAC();

 SetupTIM2();

 DMA_Cmd(DMA_Channel_TIM2, ENABLE);

 ADC1_Cmd (ENABLE);
 DMA_Cmd(DMA_Channel_ADC1, ENABLE);


 while (1) {
  if (command_recived == 1) {

   command_recived = 0;
   execute_command(RecBuf);
   for(int i = 0; i < 128; i++) {
    Buffer[i] = 0;
   }
  }

  while (DMA_GetFlagStatus(DMA_Channel_ADC1, DMA_FLAG_CHNL_ALT) == 0)
   ;
  DMA_CtrlInit(DMA_Channel_ADC1, DMA_CTRL_DATA_PRIMARY, &sDMA_PriCtrlData_ADC1);




  USB_CDC_SendData((uint8_t *)(ADC1_array_m), ((128) * 2 ));


  while (DMA_GetFlagStatus(DMA_Channel_ADC1, DMA_FLAG_CHNL_ALT) != 0)
   ;
  DMA_CtrlInit(DMA_Channel_ADC1, DMA_CTRL_DATA_ALTERNATE, &sDMA_AltCtrlData_ADC1);





  USB_CDC_SendData((uint8_t *)(ADC1_array_a), ((128) * 2));
 }
}
