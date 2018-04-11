#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Src\\stm32f7xx_hal_timebase_rtc_wakeup_template.c"






























































 

 
#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal.h"




































  

 







 
#line 1 "..\\USER\\stm32f7xx_hal_conf.h"



































  

 







 
 

 


 





   
#line 69 "..\\USER\\stm32f7xx_hal_conf.h"
 
#line 95 "..\\USER\\stm32f7xx_hal_conf.h"
 




 




 












 






 







 












 





 

 


      






 



 
 

 

 

 
#line 182 "..\\USER\\stm32f7xx_hal_conf.h"

    





 
 

  

 





 



 
#line 215 "..\\USER\\stm32f7xx_hal_conf.h"




  
 










 




 



 


 

#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc.h"



































 

 







 
#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_def.h"




































 

 







 
#line 1 "..\\USER\\stm32f7xx.h"













































 



 



 
    






  


 



 






 
#line 93 "..\\USER\\stm32f7xx.h"



 

#line 106 "..\\USER\\stm32f7xx.h"



 
#line 118 "..\\USER\\stm32f7xx.h"


 



 
#line 1 "..\\USER\\stm32f767xx.h"









































 



 



 
    






  


 




 
typedef enum
{
 
  NonMaskableInt_IRQn         = -14,     
  MemoryManagement_IRQn       = -12,     
  BusFault_IRQn               = -11,     
  UsageFault_IRQn             = -10,     
  SVCall_IRQn                 = -5,      
  DebugMonitor_IRQn           = -4,      
  PendSV_IRQn                 = -2,      
  SysTick_IRQn                = -1,      
 
  WWDG_IRQn                   = 0,       
  PVD_IRQn                    = 1,       
  TAMP_STAMP_IRQn             = 2,       
  RTC_WKUP_IRQn               = 3,       
  FLASH_IRQn                  = 4,       
  RCC_IRQn                    = 5,       
  EXTI0_IRQn                  = 6,       
  EXTI1_IRQn                  = 7,       
  EXTI2_IRQn                  = 8,       
  EXTI3_IRQn                  = 9,       
  EXTI4_IRQn                  = 10,      
  DMA1_Stream0_IRQn           = 11,      
  DMA1_Stream1_IRQn           = 12,      
  DMA1_Stream2_IRQn           = 13,      
  DMA1_Stream3_IRQn           = 14,      
  DMA1_Stream4_IRQn           = 15,      
  DMA1_Stream5_IRQn           = 16,      
  DMA1_Stream6_IRQn           = 17,      
  ADC_IRQn                    = 18,      
  CAN1_TX_IRQn                = 19,      
  CAN1_RX0_IRQn               = 20,      
  CAN1_RX1_IRQn               = 21,      
  CAN1_SCE_IRQn               = 22,      
  EXTI9_5_IRQn                = 23,      
  TIM1_BRK_TIM9_IRQn          = 24,      
  TIM1_UP_TIM10_IRQn          = 25,      
  TIM1_TRG_COM_TIM11_IRQn     = 26,      
  TIM1_CC_IRQn                = 27,      
  TIM2_IRQn                   = 28,      
  TIM3_IRQn                   = 29,      
  TIM4_IRQn                   = 30,      
  I2C1_EV_IRQn                = 31,      
  I2C1_ER_IRQn                = 32,      
  I2C2_EV_IRQn                = 33,      
  I2C2_ER_IRQn                = 34,        
  SPI1_IRQn                   = 35,      
  SPI2_IRQn                   = 36,      
  USART1_IRQn                 = 37,      
  USART2_IRQn                 = 38,      
  USART3_IRQn                 = 39,      
  EXTI15_10_IRQn              = 40,      
  RTC_Alarm_IRQn              = 41,      
  OTG_FS_WKUP_IRQn            = 42,          
  TIM8_BRK_TIM12_IRQn         = 43,      
  TIM8_UP_TIM13_IRQn          = 44,      
  TIM8_TRG_COM_TIM14_IRQn     = 45,      
  TIM8_CC_IRQn                = 46,      
  DMA1_Stream7_IRQn           = 47,      
  FMC_IRQn                    = 48,      
  SDMMC1_IRQn                 = 49,      
  TIM5_IRQn                   = 50,      
  SPI3_IRQn                   = 51,      
  UART4_IRQn                  = 52,      
  UART5_IRQn                  = 53,      
  TIM6_DAC_IRQn               = 54,      
  TIM7_IRQn                   = 55,      
  DMA2_Stream0_IRQn           = 56,      
  DMA2_Stream1_IRQn           = 57,      
  DMA2_Stream2_IRQn           = 58,      
  DMA2_Stream3_IRQn           = 59,      
  DMA2_Stream4_IRQn           = 60,      
  ETH_IRQn                    = 61,      
  ETH_WKUP_IRQn               = 62,      
  CAN2_TX_IRQn                = 63,      
  CAN2_RX0_IRQn               = 64,      
  CAN2_RX1_IRQn               = 65,      
  CAN2_SCE_IRQn               = 66,      
  OTG_FS_IRQn                 = 67,      
  DMA2_Stream5_IRQn           = 68,      
  DMA2_Stream6_IRQn           = 69,      
  DMA2_Stream7_IRQn           = 70,      
  USART6_IRQn                 = 71,      
  I2C3_EV_IRQn                = 72,      
  I2C3_ER_IRQn                = 73,      
  OTG_HS_EP1_OUT_IRQn         = 74,      
  OTG_HS_EP1_IN_IRQn          = 75,      
  OTG_HS_WKUP_IRQn            = 76,      
  OTG_HS_IRQn                 = 77,      
  DCMI_IRQn                   = 78,      
  RNG_IRQn                    = 80,      
  FPU_IRQn                    = 81,      
  UART7_IRQn                  = 82,      
  UART8_IRQn                  = 83,      
  SPI4_IRQn                   = 84,      
  SPI5_IRQn                   = 85,      
  SPI6_IRQn                   = 86,      
  SAI1_IRQn                   = 87,      
  LTDC_IRQn                   = 88,      
  LTDC_ER_IRQn                = 89,      
  DMA2D_IRQn                  = 90,      
  SAI2_IRQn                   = 91,      
  QUADSPI_IRQn                = 92,      
  LPTIM1_IRQn                 = 93,      
  CEC_IRQn                    = 94,      
  I2C4_EV_IRQn                = 95,      
  I2C4_ER_IRQn                = 96,      
  SPDIF_RX_IRQn               = 97,      
  DFSDM1_FLT0_IRQn	          = 99,      
  DFSDM1_FLT1_IRQn	          = 100,     
  DFSDM1_FLT2_IRQn	          = 101,     
  DFSDM1_FLT3_IRQn	          = 102,     
  SDMMC2_IRQn                 = 103,     
  CAN3_TX_IRQn                = 104,     
  CAN3_RX0_IRQn               = 105,     
  CAN3_RX1_IRQn               = 106,     
  CAN3_SCE_IRQn               = 107,     
  JPEG_IRQn                   = 108,     
  MDIOS_IRQn                  = 109      
} IRQn_Type;



 



 
#line 1 "..\\CORE\\core_cm7.h"
 




 

























 











#line 1 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
 
 





 









     
#line 27 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
     











#line 46 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"





 

     

     
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

     
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;
     

     
     
typedef   signed          char int_least8_t;
typedef   signed short     int int_least16_t;
typedef   signed           int int_least32_t;
typedef   signed       __int64 int_least64_t;

     
typedef unsigned          char uint_least8_t;
typedef unsigned short     int uint_least16_t;
typedef unsigned           int uint_least32_t;
typedef unsigned       __int64 uint_least64_t;

     

     
typedef   signed           int int_fast8_t;
typedef   signed           int int_fast16_t;
typedef   signed           int int_fast32_t;
typedef   signed       __int64 int_fast64_t;

     
typedef unsigned           int uint_fast8_t;
typedef unsigned           int uint_fast16_t;
typedef unsigned           int uint_fast32_t;
typedef unsigned       __int64 uint_fast64_t;

     




typedef   signed           int intptr_t;
typedef unsigned           int uintptr_t;


     
typedef   signed     long long intmax_t;
typedef unsigned     long long uintmax_t;




     

     





     





     





     

     





     





     





     

     





     





     





     

     






     






     






     

     


     


     


     

     
#line 215 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     



     






     
    
 



#line 240 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     







     










     











#line 304 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"






 
#line 45 "..\\CORE\\core_cm7.h"

















 




 



 

 













#line 120 "..\\CORE\\core_cm7.h"



 
#line 135 "..\\CORE\\core_cm7.h"

#line 209 "..\\CORE\\core_cm7.h"

#line 1 "..\\CORE\\core_cmInstr.h"
 




 

























 












 



 

 
#line 1 "..\\CORE\\cmsis_armcc.h"
 




 

























 










 



 

 
 





 
static __inline uint32_t __get_CONTROL(void)
{
  register uint32_t __regControl         __asm("control");
  return(__regControl);
}






 
static __inline void __set_CONTROL(uint32_t control)
{
  register uint32_t __regControl         __asm("control");
  __regControl = control;
}






 
static __inline uint32_t __get_IPSR(void)
{
  register uint32_t __regIPSR          __asm("ipsr");
  return(__regIPSR);
}






 
static __inline uint32_t __get_APSR(void)
{
  register uint32_t __regAPSR          __asm("apsr");
  return(__regAPSR);
}






 
static __inline uint32_t __get_xPSR(void)
{
  register uint32_t __regXPSR          __asm("xpsr");
  return(__regXPSR);
}






 
static __inline uint32_t __get_PSP(void)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  return(__regProcessStackPointer);
}






 
static __inline void __set_PSP(uint32_t topOfProcStack)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  __regProcessStackPointer = topOfProcStack;
}






 
static __inline uint32_t __get_MSP(void)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  return(__regMainStackPointer);
}






 
static __inline void __set_MSP(uint32_t topOfMainStack)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  __regMainStackPointer = topOfMainStack;
}






 
static __inline uint32_t __get_PRIMASK(void)
{
  register uint32_t __regPriMask         __asm("primask");
  return(__regPriMask);
}






 
static __inline void __set_PRIMASK(uint32_t priMask)
{
  register uint32_t __regPriMask         __asm("primask");
  __regPriMask = (priMask);
}








 







 







 
static __inline uint32_t  __get_BASEPRI(void)
{
  register uint32_t __regBasePri         __asm("basepri");
  return(__regBasePri);
}






 
static __inline void __set_BASEPRI(uint32_t basePri)
{
  register uint32_t __regBasePri         __asm("basepri");
  __regBasePri = (basePri & 0xFFU);
}







 
static __inline void __set_BASEPRI_MAX(uint32_t basePri)
{
  register uint32_t __regBasePriMax      __asm("basepri_max");
  __regBasePriMax = (basePri & 0xFFU);
}






 
static __inline uint32_t __get_FAULTMASK(void)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  return(__regFaultMask);
}






 
static __inline void __set_FAULTMASK(uint32_t faultMask)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  __regFaultMask = (faultMask & (uint32_t)1);
}










 
static __inline uint32_t __get_FPSCR(void)
{

  register uint32_t __regfpscr         __asm("fpscr");
  return(__regfpscr);



}






 
static __inline void __set_FPSCR(uint32_t fpscr)
{

  register uint32_t __regfpscr         __asm("fpscr");
  __regfpscr = (fpscr);

}





 


 



 




 






 







 






 








 










 










 











 








 

__attribute__((section(".rev16_text"))) static __inline __asm uint32_t __REV16(uint32_t value)
{
  rev16 r0, r0
  bx lr
}







 

__attribute__((section(".revsh_text"))) static __inline __asm int32_t __REVSH(int32_t value)
{
  revsh r0, r0
  bx lr
}









 









 








 
#line 455 "..\\CORE\\cmsis_armcc.h"







 










 












 












 














 














 














 










 









 









 









 

__attribute__((section(".rrx_text"))) static __inline __asm uint32_t __RRX(uint32_t value)
{
  rrx r0, r0
  bx lr
}








 








 








 








 








 








 




   


 



 



#line 720 "..\\CORE\\cmsis_armcc.h"











 


#line 54 "..\\CORE\\core_cmInstr.h"

 
#line 84 "..\\CORE\\core_cmInstr.h"

   

#line 211 "..\\CORE\\core_cm7.h"
#line 1 "..\\CORE\\core_cmFunc.h"
 




 

























 












 



 

 
#line 54 "..\\CORE\\core_cmFunc.h"

 
#line 84 "..\\CORE\\core_cmFunc.h"

 

#line 212 "..\\CORE\\core_cm7.h"
#line 1 "..\\CORE\\core_cmSimd.h"
 




 

























 
















 



 

 
#line 58 "..\\CORE\\core_cmSimd.h"

 
#line 88 "..\\CORE\\core_cmSimd.h"

 






#line 213 "..\\CORE\\core_cm7.h"
















 
#line 271 "..\\CORE\\core_cm7.h"

 






 
#line 287 "..\\CORE\\core_cm7.h"

 




 













 



 






 



 
typedef union
{
  struct
  {
    uint32_t _reserved0:16;               
    uint32_t GE:4;                        
    uint32_t _reserved1:7;                
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
    uint32_t _reserved0:7;                
    uint32_t GE:4;                        
    uint32_t _reserved1:4;                
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

 









 







 



 
typedef struct
{
  volatile uint32_t ISER[8U];                
        uint32_t RESERVED0[24U];
  volatile uint32_t ICER[8U];                
        uint32_t RSERVED1[24U];
  volatile uint32_t ISPR[8U];                
        uint32_t RESERVED2[24U];
  volatile uint32_t ICPR[8U];                
        uint32_t RESERVED3[24U];
  volatile uint32_t IABR[8U];                
        uint32_t RESERVED4[56U];
  volatile uint8_t  IP[240U];                
        uint32_t RESERVED5[644U];
  volatile  uint32_t STIR;                    
}  NVIC_Type;

 



 







 



 
typedef struct
{
  volatile const  uint32_t CPUID;                   
  volatile uint32_t ICSR;                    
  volatile uint32_t VTOR;                    
  volatile uint32_t AIRCR;                   
  volatile uint32_t SCR;                     
  volatile uint32_t CCR;                     
  volatile uint8_t  SHPR[12U];               
  volatile uint32_t SHCSR;                   
  volatile uint32_t CFSR;                    
  volatile uint32_t HFSR;                    
  volatile uint32_t DFSR;                    
  volatile uint32_t MMFAR;                   
  volatile uint32_t BFAR;                    
  volatile uint32_t AFSR;                    
  volatile const  uint32_t ID_PFR[2U];              
  volatile const  uint32_t ID_DFR;                  
  volatile const  uint32_t ID_AFR;                  
  volatile const  uint32_t ID_MFR[4U];              
  volatile const  uint32_t ID_ISAR[5U];             
        uint32_t RESERVED0[1U];
  volatile const  uint32_t CLIDR;                   
  volatile const  uint32_t CTR;                     
  volatile const  uint32_t CCSIDR;                  
  volatile uint32_t CSSELR;                  
  volatile uint32_t CPACR;                   
        uint32_t RESERVED3[93U];
  volatile  uint32_t STIR;                    
        uint32_t RESERVED4[15U];
  volatile const  uint32_t MVFR0;                   
  volatile const  uint32_t MVFR1;                   
  volatile const  uint32_t MVFR2;                   
        uint32_t RESERVED5[1U];
  volatile  uint32_t ICIALLU;                 
        uint32_t RESERVED6[1U];
  volatile  uint32_t ICIMVAU;                 
  volatile  uint32_t DCIMVAC;                 
  volatile  uint32_t DCISW;                   
  volatile  uint32_t DCCMVAU;                 
  volatile  uint32_t DCCMVAC;                 
  volatile  uint32_t DCCSW;                   
  volatile  uint32_t DCCIMVAC;                
  volatile  uint32_t DCCISW;                  
        uint32_t RESERVED7[6U];
  volatile uint32_t ITCMCR;                  
  volatile uint32_t DTCMCR;                  
  volatile uint32_t AHBPCR;                  
  volatile uint32_t CACR;                    
  volatile uint32_t AHBSCR;                  
        uint32_t RESERVED8[1U];
  volatile uint32_t ABFSR;                   
} SCB_Type;

 















 






























 



 





















 









 



























 










































 









 









 















 






 















 





















 






 



 






 






 






 












 












 






 









 









 


















 







 



 
typedef struct
{
        uint32_t RESERVED0[1U];
  volatile const  uint32_t ICTR;                    
  volatile uint32_t ACTLR;                   
} SCnSCB_Type;

 



 















 







 



 
typedef struct
{
  volatile uint32_t CTRL;                    
  volatile uint32_t LOAD;                    
  volatile uint32_t VAL;                     
  volatile const  uint32_t CALIB;                   
} SysTick_Type;

 












 



 



 









 







 



 
typedef struct
{
  volatile  union
  {
    volatile  uint8_t    u8;                  
    volatile  uint16_t   u16;                 
    volatile  uint32_t   u32;                 
  }  PORT [32U];                          
        uint32_t RESERVED0[864U];
  volatile uint32_t TER;                     
        uint32_t RESERVED1[15U];
  volatile uint32_t TPR;                     
        uint32_t RESERVED2[15U];
  volatile uint32_t TCR;                     
        uint32_t RESERVED3[29U];
  volatile  uint32_t IWR;                     
  volatile const  uint32_t IRR;                     
  volatile uint32_t IMCR;                    
        uint32_t RESERVED4[43U];
  volatile  uint32_t LAR;                     
  volatile const  uint32_t LSR;                     
        uint32_t RESERVED5[6U];
  volatile const  uint32_t PID4;                    
  volatile const  uint32_t PID5;                    
  volatile const  uint32_t PID6;                    
  volatile const  uint32_t PID7;                    
  volatile const  uint32_t PID0;                    
  volatile const  uint32_t PID1;                    
  volatile const  uint32_t PID2;                    
  volatile const  uint32_t PID3;                    
  volatile const  uint32_t CID0;                    
  volatile const  uint32_t CID1;                    
  volatile const  uint32_t CID2;                    
  volatile const  uint32_t CID3;                    
} ITM_Type;

 



 



























 



 



 



 









   







 



 
typedef struct
{
  volatile uint32_t CTRL;                    
  volatile uint32_t CYCCNT;                  
  volatile uint32_t CPICNT;                  
  volatile uint32_t EXCCNT;                  
  volatile uint32_t SLEEPCNT;                
  volatile uint32_t LSUCNT;                  
  volatile uint32_t FOLDCNT;                 
  volatile const  uint32_t PCSR;                    
  volatile uint32_t COMP0;                   
  volatile uint32_t MASK0;                   
  volatile uint32_t FUNCTION0;               
        uint32_t RESERVED0[1U];
  volatile uint32_t COMP1;                   
  volatile uint32_t MASK1;                   
  volatile uint32_t FUNCTION1;               
        uint32_t RESERVED1[1U];
  volatile uint32_t COMP2;                   
  volatile uint32_t MASK2;                   
  volatile uint32_t FUNCTION2;               
        uint32_t RESERVED2[1U];
  volatile uint32_t COMP3;                   
  volatile uint32_t MASK3;                   
  volatile uint32_t FUNCTION3;               
        uint32_t RESERVED3[981U];
  volatile  uint32_t LAR;                     
  volatile const  uint32_t LSR;                     
} DWT_Type;

 






















































 



 



 



 



 



 



 



























   







 



 
typedef struct
{
  volatile uint32_t SSPSR;                   
  volatile uint32_t CSPSR;                   
        uint32_t RESERVED0[2U];
  volatile uint32_t ACPR;                    
        uint32_t RESERVED1[55U];
  volatile uint32_t SPPR;                    
        uint32_t RESERVED2[131U];
  volatile const  uint32_t FFSR;                    
  volatile uint32_t FFCR;                    
  volatile const  uint32_t FSCR;                    
        uint32_t RESERVED3[759U];
  volatile const  uint32_t TRIGGER;                 
  volatile const  uint32_t FIFO0;                   
  volatile const  uint32_t ITATBCTR2;               
        uint32_t RESERVED4[1U];
  volatile const  uint32_t ITATBCTR0;               
  volatile const  uint32_t FIFO1;                   
  volatile uint32_t ITCTRL;                  
        uint32_t RESERVED5[39U];
  volatile uint32_t CLAIMSET;                
  volatile uint32_t CLAIMCLR;                
        uint32_t RESERVED7[8U];
  volatile const  uint32_t DEVID;                   
  volatile const  uint32_t DEVTYPE;                 
} TPI_Type;

 



 



 












 






 



 





















 



 





















 



 



 


















 






   








 



 
typedef struct
{
  volatile const  uint32_t TYPE;                    
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

 









 









 



 









 






























 









 



 
typedef struct
{
        uint32_t RESERVED0[1U];
  volatile uint32_t FPCCR;                   
  volatile uint32_t FPCAR;                   
  volatile uint32_t FPDSCR;                  
  volatile const  uint32_t MVFR0;                   
  volatile const  uint32_t MVFR1;                   
  volatile const  uint32_t MVFR2;                   
} FPU_Type;

 



























 



 












 
























 












 

 








 



 
typedef struct
{
  volatile uint32_t DHCSR;                   
  volatile  uint32_t DCRSR;                   
  volatile uint32_t DCRDR;                   
  volatile uint32_t DEMCR;                   
} CoreDebug_Type;

 




































 






 







































 







 






 







 


 







 

 
#line 1749 "..\\CORE\\core_cm7.h"

#line 1758 "..\\CORE\\core_cm7.h"











 










 


 



 





 









 
static __inline void NVIC_SetPriorityGrouping(uint32_t PriorityGroup)
{
  uint32_t reg_value;
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);              

  reg_value  =  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR;                                                    
  reg_value &= ~((uint32_t)((0xFFFFUL << 16U) | (7UL << 8U)));  
  reg_value  =  (reg_value                                   |
                ((uint32_t)0x5FAUL << 16U) |
                (PriorityGroupTmp << 8U)                      );               
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR =  reg_value;
}






 
static __inline uint32_t NVIC_GetPriorityGrouping(void)
{
  return ((uint32_t)((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR & (7UL << 8U)) >> 8U));
}






 
static __inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[(((uint32_t)(int32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}






 
static __inline void NVIC_DisableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[(((uint32_t)(int32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}








 
static __inline uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[(((uint32_t)(int32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
}






 
static __inline void NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[(((uint32_t)(int32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}






 
static __inline void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICPR[(((uint32_t)(int32_t)IRQn) >> 5UL)] = (uint32_t)(1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL));
}








 
static __inline uint32_t NVIC_GetActive(IRQn_Type IRQn)
{
  return((uint32_t)(((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IABR[(((uint32_t)(int32_t)IRQn) >> 5UL)] & (1UL << (((uint32_t)(int32_t)IRQn) & 0x1FUL))) != 0UL) ? 1UL : 0UL));
}








 
static __inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if ((int32_t)(IRQn) < 0)
  {
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHPR[(((uint32_t)(int32_t)IRQn) & 0xFUL)-4UL] = (uint8_t)((priority << (8U - 4)) & (uint32_t)0xFFUL);
  }
  else
  {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[((uint32_t)(int32_t)IRQn)]                = (uint8_t)((priority << (8U - 4)) & (uint32_t)0xFFUL);
  }
}










 
static __inline uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{

  if ((int32_t)(IRQn) < 0)
  {
    return(((uint32_t)((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHPR[(((uint32_t)(int32_t)IRQn) & 0xFUL)-4UL] >> (8U - 4)));
  }
  else
  {
    return(((uint32_t)((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[((uint32_t)(int32_t)IRQn)]                >> (8U - 4)));
  }
}












 
static __inline uint32_t NVIC_EncodePriority (uint32_t PriorityGroup, uint32_t PreemptPriority, uint32_t SubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);    
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7UL - PriorityGroupTmp) > (uint32_t)(4)) ? (uint32_t)(4) : (uint32_t)(7UL - PriorityGroupTmp);
  SubPriorityBits     = ((PriorityGroupTmp + (uint32_t)(4)) < (uint32_t)7UL) ? (uint32_t)0UL : (uint32_t)((PriorityGroupTmp - 7UL) + (uint32_t)(4));

  return (
           ((PreemptPriority & (uint32_t)((1UL << (PreemptPriorityBits)) - 1UL)) << SubPriorityBits) |
           ((SubPriority     & (uint32_t)((1UL << (SubPriorityBits    )) - 1UL)))
         );
}












 
static __inline void NVIC_DecodePriority (uint32_t Priority, uint32_t PriorityGroup, uint32_t* const pPreemptPriority, uint32_t* const pSubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07UL);    
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7UL - PriorityGroupTmp) > (uint32_t)(4)) ? (uint32_t)(4) : (uint32_t)(7UL - PriorityGroupTmp);
  SubPriorityBits     = ((PriorityGroupTmp + (uint32_t)(4)) < (uint32_t)7UL) ? (uint32_t)0UL : (uint32_t)((PriorityGroupTmp - 7UL) + (uint32_t)(4));

  *pPreemptPriority = (Priority >> SubPriorityBits) & (uint32_t)((1UL << (PreemptPriorityBits)) - 1UL);
  *pSubPriority     = (Priority                   ) & (uint32_t)((1UL << (SubPriorityBits    )) - 1UL);
}





 
static __inline void NVIC_SystemReset(void)
{
  do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);                                                          
 
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR  = (uint32_t)((0x5FAUL << 16U)    |
                           (((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR & (7UL << 8U)) |
                            (1UL << 2U)    );          
  do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);                                                           

  for(;;)                                                            
  {
    __nop();
  }
}

 


 





 








 
static __inline uint32_t SCB_GetFPUType(void)
{
  uint32_t mvfr0;

  mvfr0 = ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->MVFR0;
  if        ((mvfr0 & 0x00000FF0UL) == 0x220UL)
  {
    return 2UL;            
  }
  else if ((mvfr0 & 0x00000FF0UL) == 0x020UL)
  {
    return 1UL;            
  }
  else
  {
    return 0UL;            
  }
}


 



 





 

 







 
static __inline void SCB_EnableICache (void)
{

    do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);
    do { __schedule_barrier(); __isb(0xF); __schedule_barrier(); } while (0U);
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->ICIALLU = 0UL;                      
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->CCR |=  (uint32_t)(1UL << 17U);   
    do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);
    do { __schedule_barrier(); __isb(0xF); __schedule_barrier(); } while (0U);

}





 
static __inline void SCB_DisableICache (void)
{

    do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);
    do { __schedule_barrier(); __isb(0xF); __schedule_barrier(); } while (0U);
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->CCR &= ~(uint32_t)(1UL << 17U);   
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->ICIALLU = 0UL;                      
    do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);
    do { __schedule_barrier(); __isb(0xF); __schedule_barrier(); } while (0U);

}





 
static __inline void SCB_InvalidateICache (void)
{

    do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);
    do { __schedule_barrier(); __isb(0xF); __schedule_barrier(); } while (0U);
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->ICIALLU = 0UL;
    do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);
    do { __schedule_barrier(); __isb(0xF); __schedule_barrier(); } while (0U);

}





 
static __inline void SCB_EnableDCache (void)
{

    uint32_t ccsidr;
    uint32_t sets;
    uint32_t ways;

    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->CSSELR = (0U << 1U) | 0U;           
    do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);

    ccsidr = ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->CCSIDR;

                                             
    sets = (uint32_t)((((ccsidr) & (0x7FFFUL << 13U) ) >> 13U ));
    do {
      ways = (uint32_t)((((ccsidr) & (0x3FFUL << 3U)) >> 3U));
      do {
        ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->DCISW = (((sets << 5U) & (0x1FFUL << 5U)) |
                      ((ways << 30U) & (3UL << 30U))  );

          __schedule_barrier();

      } while (ways--);
    } while(sets--);
    do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);

    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->CCR |=  (uint32_t)(1UL << 16U);   

    do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);
    do { __schedule_barrier(); __isb(0xF); __schedule_barrier(); } while (0U);

}





 
static __inline void SCB_DisableDCache (void)
{

    uint32_t ccsidr;
    uint32_t sets;
    uint32_t ways;

    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->CSSELR = (0U << 1U) | 0U;           
    do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);

    ccsidr = ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->CCSIDR;

    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->CCR &= ~(uint32_t)(1UL << 16U);   

                                             
    sets = (uint32_t)((((ccsidr) & (0x7FFFUL << 13U) ) >> 13U ));
    do {
      ways = (uint32_t)((((ccsidr) & (0x3FFUL << 3U)) >> 3U));
      do {
        ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->DCCISW = (((sets << 5U) & (0x1FFUL << 5U)) |
                       ((ways << 30U) & (3UL << 30U))  );

          __schedule_barrier();

      } while (ways--);
    } while(sets--);

    do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);
    do { __schedule_barrier(); __isb(0xF); __schedule_barrier(); } while (0U);

}





 
static __inline void SCB_InvalidateDCache (void)
{

    uint32_t ccsidr;
    uint32_t sets;
    uint32_t ways;

    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->CSSELR = (0U << 1U) | 0U;           
    do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);

    ccsidr = ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->CCSIDR;

                                             
    sets = (uint32_t)((((ccsidr) & (0x7FFFUL << 13U) ) >> 13U ));
    do {
      ways = (uint32_t)((((ccsidr) & (0x3FFUL << 3U)) >> 3U));
      do {
        ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->DCISW = (((sets << 5U) & (0x1FFUL << 5U)) |
                      ((ways << 30U) & (3UL << 30U))  );

          __schedule_barrier();

      } while (ways--);
    } while(sets--);

    do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);
    do { __schedule_barrier(); __isb(0xF); __schedule_barrier(); } while (0U);

}





 
static __inline void SCB_CleanDCache (void)
{

    uint32_t ccsidr;
    uint32_t sets;
    uint32_t ways;

    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->CSSELR = (0U << 1U) | 0U;           
    do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);

    ccsidr = ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->CCSIDR;

                                             
    sets = (uint32_t)((((ccsidr) & (0x7FFFUL << 13U) ) >> 13U ));
    do {
      ways = (uint32_t)((((ccsidr) & (0x3FFUL << 3U)) >> 3U));
      do {
        ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->DCCSW = (((sets << 5U) & (0x1FFUL << 5U)) |
                      ((ways << 30U) & (3UL << 30U))  );

          __schedule_barrier();

      } while (ways--);
    } while(sets--);

    do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);
    do { __schedule_barrier(); __isb(0xF); __schedule_barrier(); } while (0U);

}





 
static __inline void SCB_CleanInvalidateDCache (void)
{

    uint32_t ccsidr;
    uint32_t sets;
    uint32_t ways;

    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->CSSELR = (0U << 1U) | 0U;           
    do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);

    ccsidr = ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->CCSIDR;

                                             
    sets = (uint32_t)((((ccsidr) & (0x7FFFUL << 13U) ) >> 13U ));
    do {
      ways = (uint32_t)((((ccsidr) & (0x3FFUL << 3U)) >> 3U));
      do {
        ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->DCCISW = (((sets << 5U) & (0x1FFUL << 5U)) |
                       ((ways << 30U) & (3UL << 30U))  );

          __schedule_barrier();

      } while (ways--);
    } while(sets--);

    do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);
    do { __schedule_barrier(); __isb(0xF); __schedule_barrier(); } while (0U);

}







 
static __inline void SCB_InvalidateDCache_by_Addr (uint32_t *addr, int32_t dsize)
{

     int32_t op_size = dsize;
    uint32_t op_addr = (uint32_t)addr;
     int32_t linesize = 32U;                 

    do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);

    while (op_size > 0) {
      ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->DCIMVAC = op_addr;
      op_addr += linesize;
      op_size -= linesize;
    }

    do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);
    do { __schedule_barrier(); __isb(0xF); __schedule_barrier(); } while (0U);

}







 
static __inline void SCB_CleanDCache_by_Addr (uint32_t *addr, int32_t dsize)
{

     int32_t op_size = dsize;
    uint32_t op_addr = (uint32_t) addr;
     int32_t linesize = 32U;                 

    do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);

    while (op_size > 0) {
      ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->DCCMVAC = op_addr;
      op_addr += linesize;
      op_size -= linesize;
    }

    do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);
    do { __schedule_barrier(); __isb(0xF); __schedule_barrier(); } while (0U);

}







 
static __inline void SCB_CleanInvalidateDCache_by_Addr (uint32_t *addr, int32_t dsize)
{

     int32_t op_size = dsize;
    uint32_t op_addr = (uint32_t) addr;
     int32_t linesize = 32U;                 

    do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);

    while (op_size > 0) {
      ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->DCCIMVAC = op_addr;
      op_addr += linesize;
      op_size -= linesize;
    }

    do { __schedule_barrier(); __dsb(0xF); __schedule_barrier(); } while (0U);
    do { __schedule_barrier(); __isb(0xF); __schedule_barrier(); } while (0U);

}


 



 





 













 
static __inline uint32_t SysTick_Config(uint32_t ticks)
{
  if ((ticks - 1UL) > (0xFFFFFFUL ))
  {
    return (1UL);                                                    
  }

  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD  = (uint32_t)(ticks - 1UL);                          
  NVIC_SetPriority (SysTick_IRQn, (1UL << 4) - 1UL);  
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL   = 0UL;                                              
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL  = (1UL << 2U) |
                   (1UL << 1U)   |
                   (1UL );                          
  return (0UL);                                                      
}



 



 





 

extern volatile int32_t ITM_RxBuffer;                     










 
static __inline uint32_t ITM_SendChar (uint32_t ch)
{
  if (((((ITM_Type *) (0xE0000000UL) )->TCR & (1UL )) != 0UL) &&       
      ((((ITM_Type *) (0xE0000000UL) )->TER & 1UL               ) != 0UL)   )      
  {
    while (((ITM_Type *) (0xE0000000UL) )->PORT[0U].u32 == 0UL)
    {
      __nop();
    }
    ((ITM_Type *) (0xE0000000UL) )->PORT[0U].u8 = (uint8_t)ch;
  }
  return (ch);
}







 
static __inline int32_t ITM_ReceiveChar (void)
{
  int32_t ch = -1;                            

  if (ITM_RxBuffer != 0x5AA55AA5U)
  {
    ch = ITM_RxBuffer;
    ITM_RxBuffer = 0x5AA55AA5U;        
  }

  return (ch);
}







 
static __inline int32_t ITM_CheckChar (void)
{

  if (ITM_RxBuffer == 0x5AA55AA5U)
  {
    return (0);                               
  }
  else
  {
    return (1);                               
  }
}

 










#line 204 "..\\USER\\stm32f767xx.h"
  
  
#line 1 "..\\USER\\system_stm32f7xx.h"



































 



 



   
  


 









 



 




 
  






 
extern uint32_t SystemCoreClock;           

extern const uint8_t  AHBPrescTable[16];     
extern const uint8_t  APBPrescTable[8];      




 



 



 



 



 



 
  
extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);


 









 
  


   
 
#line 207 "..\\USER\\stm32f767xx.h"
#line 208 "..\\USER\\stm32f767xx.h"



    



 

typedef struct
{
  volatile uint32_t SR;      
  volatile uint32_t CR1;           
  volatile uint32_t CR2;     
  volatile uint32_t SMPR1;   
  volatile uint32_t SMPR2;   
  volatile uint32_t JOFR1;   
  volatile uint32_t JOFR2;   
  volatile uint32_t JOFR3;   
  volatile uint32_t JOFR4;   
  volatile uint32_t HTR;     
  volatile uint32_t LTR;     
  volatile uint32_t SQR1;    
  volatile uint32_t SQR2;    
  volatile uint32_t SQR3;    
  volatile uint32_t JSQR;    
  volatile uint32_t JDR1;    
  volatile uint32_t JDR2;    
  volatile uint32_t JDR3;    
  volatile uint32_t JDR4;    
  volatile uint32_t DR;      
} ADC_TypeDef;

typedef struct
{
  volatile uint32_t CSR;     
  volatile uint32_t CCR;     
  volatile uint32_t CDR;    
 
} ADC_Common_TypeDef;




 

typedef struct
{
  volatile uint32_t TIR;   
  volatile uint32_t TDTR;  
  volatile uint32_t TDLR;  
  volatile uint32_t TDHR;  
} CAN_TxMailBox_TypeDef;



 
  
typedef struct
{
  volatile uint32_t RIR;   
  volatile uint32_t RDTR;  
  volatile uint32_t RDLR;  
  volatile uint32_t RDHR;  
} CAN_FIFOMailBox_TypeDef;



 
  
typedef struct
{
  volatile uint32_t FR1;  
  volatile uint32_t FR2;  
} CAN_FilterRegister_TypeDef;



 
  
typedef struct
{
  volatile uint32_t              MCR;                  
  volatile uint32_t              MSR;                  
  volatile uint32_t              TSR;                  
  volatile uint32_t              RF0R;                 
  volatile uint32_t              RF1R;                 
  volatile uint32_t              IER;                  
  volatile uint32_t              ESR;                  
  volatile uint32_t              BTR;                  
  uint32_t                   RESERVED0[88];        
  CAN_TxMailBox_TypeDef      sTxMailBox[3];        
  CAN_FIFOMailBox_TypeDef    sFIFOMailBox[2];      
  uint32_t                   RESERVED1[12];        
  volatile uint32_t              FMR;                  
  volatile uint32_t              FM1R;                 
  uint32_t                   RESERVED2;            
  volatile uint32_t              FS1R;                 
  uint32_t                   RESERVED3;            
  volatile uint32_t              FFA1R;                
  uint32_t                   RESERVED4;            
  volatile uint32_t              FA1R;                 
  uint32_t                   RESERVED5[8];          
  CAN_FilterRegister_TypeDef sFilterRegister[28];  
} CAN_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;            
  volatile uint32_t CFGR;          
  volatile uint32_t TXDR;          
  volatile uint32_t RXDR;          
  volatile uint32_t ISR;           
  volatile uint32_t IER;           
}CEC_TypeDef;




 

typedef struct
{
  volatile uint32_t  DR;           
  volatile uint8_t   IDR;          
  uint8_t        RESERVED0;    
  uint16_t       RESERVED1;    
  volatile uint32_t  CR;           
  uint32_t       RESERVED2;    
  volatile uint32_t  INIT;         
  volatile uint32_t  POL;          
} CRC_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;        
  volatile uint32_t SWTRIGR;   
  volatile uint32_t DHR12R1;   
  volatile uint32_t DHR12L1;   
  volatile uint32_t DHR8R1;    
  volatile uint32_t DHR12R2;   
  volatile uint32_t DHR12L2;   
  volatile uint32_t DHR8R2;    
  volatile uint32_t DHR12RD;   
  volatile uint32_t DHR12LD;   
  volatile uint32_t DHR8RD;    
  volatile uint32_t DOR1;      
  volatile uint32_t DOR2;      
  volatile uint32_t SR;        
} DAC_TypeDef;



 
typedef struct
{
  volatile uint32_t FLTCR1;          
  volatile uint32_t FLTCR2;          
  volatile uint32_t FLTISR;          
  volatile uint32_t FLTICR;          
  volatile uint32_t FLTJCHGR;        
  volatile uint32_t FLTFCR;          
  volatile uint32_t FLTJDATAR;       
  volatile uint32_t FLTRDATAR;       
  volatile uint32_t FLTAWHTR;        
  volatile uint32_t FLTAWLTR;        
  volatile uint32_t FLTAWSR;         
  volatile uint32_t FLTAWCFR;        
  volatile uint32_t FLTEXMAX;        
  volatile uint32_t FLTEXMIN;        
  volatile uint32_t FLTCNVTIMR;      
} DFSDM_Filter_TypeDef;



 
typedef struct
{
  volatile uint32_t CHCFGR1;      
  volatile uint32_t CHCFGR2;      
  volatile uint32_t CHAWSCDR;    
 
  volatile uint32_t CHWDATAR;     
  volatile uint32_t CHDATINR;     
} DFSDM_Channel_TypeDef;



 

typedef struct
{
  volatile uint32_t IDCODE;   
  volatile uint32_t CR;       
  volatile uint32_t APB1FZ;   
  volatile uint32_t APB2FZ;   
}DBGMCU_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;        
  volatile uint32_t SR;        
  volatile uint32_t RISR;      
  volatile uint32_t IER;       
  volatile uint32_t MISR;      
  volatile uint32_t ICR;       
  volatile uint32_t ESCR;      
  volatile uint32_t ESUR;      
  volatile uint32_t CWSTRTR;   
  volatile uint32_t CWSIZER;   
  volatile uint32_t DR;        
} DCMI_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;      
  volatile uint32_t NDTR;    
  volatile uint32_t PAR;     
  volatile uint32_t M0AR;    
  volatile uint32_t M1AR;    
  volatile uint32_t FCR;     
} DMA_Stream_TypeDef;

typedef struct
{
  volatile uint32_t LISR;    
  volatile uint32_t HISR;    
  volatile uint32_t LIFCR;   
  volatile uint32_t HIFCR;   
} DMA_TypeDef;




 

typedef struct
{
  volatile uint32_t CR;             
  volatile uint32_t ISR;            
  volatile uint32_t IFCR;           
  volatile uint32_t FGMAR;          
  volatile uint32_t FGOR;           
  volatile uint32_t BGMAR;          
  volatile uint32_t BGOR;           
  volatile uint32_t FGPFCCR;        
  volatile uint32_t FGCOLR;         
  volatile uint32_t BGPFCCR;        
  volatile uint32_t BGCOLR;         
  volatile uint32_t FGCMAR;         
  volatile uint32_t BGCMAR;         
  volatile uint32_t OPFCCR;         
  volatile uint32_t OCOLR;          
  volatile uint32_t OMAR;           
  volatile uint32_t OOR;            
  volatile uint32_t NLR;            
  volatile uint32_t LWR;            
  volatile uint32_t AMTCR;          
  uint32_t      RESERVED[236];  
  volatile uint32_t FGCLUT[256];    
  volatile uint32_t BGCLUT[256];    
} DMA2D_TypeDef;




 

typedef struct
{
  volatile uint32_t MACCR;
  volatile uint32_t MACFFR;
  volatile uint32_t MACHTHR;
  volatile uint32_t MACHTLR;
  volatile uint32_t MACMIIAR;
  volatile uint32_t MACMIIDR;
  volatile uint32_t MACFCR;
  volatile uint32_t MACVLANTR;              
  uint32_t      RESERVED0[2];
  volatile uint32_t MACRWUFFR;              
  volatile uint32_t MACPMTCSR;
  uint32_t      RESERVED1[2];
  volatile uint32_t MACSR;                  
  volatile uint32_t MACIMR;
  volatile uint32_t MACA0HR;
  volatile uint32_t MACA0LR;
  volatile uint32_t MACA1HR;
  volatile uint32_t MACA1LR;
  volatile uint32_t MACA2HR;
  volatile uint32_t MACA2LR;
  volatile uint32_t MACA3HR;
  volatile uint32_t MACA3LR;                
  uint32_t      RESERVED2[40];
  volatile uint32_t MMCCR;                  
  volatile uint32_t MMCRIR;
  volatile uint32_t MMCTIR;
  volatile uint32_t MMCRIMR;
  volatile uint32_t MMCTIMR;                
  uint32_t      RESERVED3[14];
  volatile uint32_t MMCTGFSCCR;             
  volatile uint32_t MMCTGFMSCCR;
  uint32_t      RESERVED4[5];
  volatile uint32_t MMCTGFCR;
  uint32_t      RESERVED5[10];
  volatile uint32_t MMCRFCECR;
  volatile uint32_t MMCRFAECR;
  uint32_t      RESERVED6[10];
  volatile uint32_t MMCRGUFCR;
  uint32_t      RESERVED7[334];
  volatile uint32_t PTPTSCR;
  volatile uint32_t PTPSSIR;
  volatile uint32_t PTPTSHR;
  volatile uint32_t PTPTSLR;
  volatile uint32_t PTPTSHUR;
  volatile uint32_t PTPTSLUR;
  volatile uint32_t PTPTSAR;
  volatile uint32_t PTPTTHR;
  volatile uint32_t PTPTTLR;
  volatile uint32_t RESERVED8;
  volatile uint32_t PTPTSSR;
  uint32_t      RESERVED9[565];
  volatile uint32_t DMABMR;
  volatile uint32_t DMATPDR;
  volatile uint32_t DMARPDR;
  volatile uint32_t DMARDLAR;
  volatile uint32_t DMATDLAR;
  volatile uint32_t DMASR;
  volatile uint32_t DMAOMR;
  volatile uint32_t DMAIER;
  volatile uint32_t DMAMFBOCR;
  volatile uint32_t DMARSWTR;
  uint32_t      RESERVED10[8];
  volatile uint32_t DMACHTDR;
  volatile uint32_t DMACHRDR;
  volatile uint32_t DMACHTBAR;
  volatile uint32_t DMACHRBAR;
} ETH_TypeDef;



 

typedef struct
{
  volatile uint32_t IMR;     
  volatile uint32_t EMR;     
  volatile uint32_t RTSR;    
  volatile uint32_t FTSR;    
  volatile uint32_t SWIER;   
  volatile uint32_t PR;      
} EXTI_TypeDef;



 

typedef struct
{
  volatile uint32_t ACR;       
  volatile uint32_t KEYR;      
  volatile uint32_t OPTKEYR;   
  volatile uint32_t SR;        
  volatile uint32_t CR;        
  volatile uint32_t OPTCR;     
  volatile uint32_t OPTCR1;    
} FLASH_TypeDef;





 

typedef struct
{
  volatile uint32_t BTCR[8];        
} FMC_Bank1_TypeDef; 



 
  
typedef struct
{
  volatile uint32_t BWTR[7];     
} FMC_Bank1E_TypeDef;



 
  
typedef struct
{
  volatile uint32_t PCR;         
  volatile uint32_t SR;          
  volatile uint32_t PMEM;        
  volatile uint32_t PATT;        
  uint32_t      RESERVED0;   
  volatile uint32_t ECCR;        
} FMC_Bank3_TypeDef;
 


 
  
typedef struct
{
  volatile uint32_t SDCR[2];         
  volatile uint32_t SDTR[2];         
  volatile uint32_t SDCMR;        
  volatile uint32_t SDRTR;        
  volatile uint32_t SDSR;         
} FMC_Bank5_6_TypeDef; 




 

typedef struct
{
  volatile uint32_t MODER;     
  volatile uint32_t OTYPER;    
  volatile uint32_t OSPEEDR;   
  volatile uint32_t PUPDR;     
  volatile uint32_t IDR;       
  volatile uint32_t ODR;       
  volatile uint32_t BSRR;      
  volatile uint32_t LCKR;      
  volatile uint32_t AFR[2];    
} GPIO_TypeDef;



 
  
typedef struct
{
  volatile uint32_t MEMRMP;        
  volatile uint32_t PMC;           
  volatile uint32_t EXTICR[4];     
  uint32_t      RESERVED;      
  volatile uint32_t CBR;           
  volatile uint32_t CMPCR;         
} SYSCFG_TypeDef;



 

typedef struct
{
  volatile uint32_t CR1;       
  volatile uint32_t CR2;         
  volatile uint32_t OAR1;      
  volatile uint32_t OAR2;      
  volatile uint32_t TIMINGR;   
  volatile uint32_t TIMEOUTR;  
  volatile uint32_t ISR;       
  volatile uint32_t ICR;       
  volatile uint32_t PECR;      
  volatile uint32_t RXDR;      
  volatile uint32_t TXDR;        
} I2C_TypeDef;



 

typedef struct
{
  volatile uint32_t KR;    
  volatile uint32_t PR;    
  volatile uint32_t RLR;   
  volatile uint32_t SR;    
  volatile uint32_t WINR;  
} IWDG_TypeDef;




 
  
typedef struct
{
  uint32_t      RESERVED0[2];   
  volatile uint32_t SSCR;           
  volatile uint32_t BPCR;           
  volatile uint32_t AWCR;           
  volatile uint32_t TWCR;           
  volatile uint32_t GCR;            
  uint32_t      RESERVED1[2];   
  volatile uint32_t SRCR;           
  uint32_t      RESERVED2[1];   
  volatile uint32_t BCCR;           
  uint32_t      RESERVED3[1];   
  volatile uint32_t IER;            
  volatile uint32_t ISR;            
  volatile uint32_t ICR;            
  volatile uint32_t LIPCR;          
  volatile uint32_t CPSR;           
  volatile uint32_t CDSR;          
} LTDC_TypeDef;  



 
  
typedef struct
{  
  volatile uint32_t CR;             
  volatile uint32_t WHPCR;          
  volatile uint32_t WVPCR;          
  volatile uint32_t CKCR;           
  volatile uint32_t PFCR;           
  volatile uint32_t CACR;           
  volatile uint32_t DCCR;           
  volatile uint32_t BFCR;           
  uint32_t      RESERVED0[2];   
  volatile uint32_t CFBAR;          
  volatile uint32_t CFBLR;          
  volatile uint32_t CFBLNR;         
  uint32_t      RESERVED1[3];   
  volatile uint32_t CLUTWR;         

} LTDC_Layer_TypeDef;



 

typedef struct
{
  volatile uint32_t CR1;    
  volatile uint32_t CSR1;   
  volatile uint32_t CR2;    
  volatile uint32_t CSR2;   
} PWR_TypeDef;




 

typedef struct
{
  volatile uint32_t CR;             
  volatile uint32_t PLLCFGR;        
  volatile uint32_t CFGR;           
  volatile uint32_t CIR;            
  volatile uint32_t AHB1RSTR;       
  volatile uint32_t AHB2RSTR;       
  volatile uint32_t AHB3RSTR;       
  uint32_t      RESERVED0;      
  volatile uint32_t APB1RSTR;       
  volatile uint32_t APB2RSTR;       
  uint32_t      RESERVED1[2];   
  volatile uint32_t AHB1ENR;        
  volatile uint32_t AHB2ENR;        
  volatile uint32_t AHB3ENR;        
  uint32_t      RESERVED2;      
  volatile uint32_t APB1ENR;        
  volatile uint32_t APB2ENR;        
  uint32_t      RESERVED3[2];   
  volatile uint32_t AHB1LPENR;      
  volatile uint32_t AHB2LPENR;      
  volatile uint32_t AHB3LPENR;      
  uint32_t      RESERVED4;      
  volatile uint32_t APB1LPENR;      
  volatile uint32_t APB2LPENR;      
  uint32_t      RESERVED5[2];   
  volatile uint32_t BDCR;           
  volatile uint32_t CSR;            
  uint32_t      RESERVED6[2];   
  volatile uint32_t SSCGR;          
  volatile uint32_t PLLI2SCFGR;     
  volatile uint32_t PLLSAICFGR;     
  volatile uint32_t DCKCFGR1;       
  volatile uint32_t DCKCFGR2;       

} RCC_TypeDef;



 

typedef struct
{
  volatile uint32_t TR;          
  volatile uint32_t DR;          
  volatile uint32_t CR;                                                                                                      
  volatile uint32_t ISR;         
  volatile uint32_t PRER;        
  volatile uint32_t WUTR;        
       uint32_t reserved;    
  volatile uint32_t ALRMAR;      
  volatile uint32_t ALRMBR;      
  volatile uint32_t WPR;         
  volatile uint32_t SSR;         
  volatile uint32_t SHIFTR;      
  volatile uint32_t TSTR;        
  volatile uint32_t TSDR;        
  volatile uint32_t TSSSR;       
  volatile uint32_t CALR;        
  volatile uint32_t TAMPCR;      
  volatile uint32_t ALRMASSR;    
  volatile uint32_t ALRMBSSR;    
  volatile uint32_t OR;          
  volatile uint32_t BKP0R;       
  volatile uint32_t BKP1R;       
  volatile uint32_t BKP2R;       
  volatile uint32_t BKP3R;       
  volatile uint32_t BKP4R;       
  volatile uint32_t BKP5R;       
  volatile uint32_t BKP6R;       
  volatile uint32_t BKP7R;       
  volatile uint32_t BKP8R;       
  volatile uint32_t BKP9R;       
  volatile uint32_t BKP10R;      
  volatile uint32_t BKP11R;      
  volatile uint32_t BKP12R;      
  volatile uint32_t BKP13R;      
  volatile uint32_t BKP14R;      
  volatile uint32_t BKP15R;      
  volatile uint32_t BKP16R;      
  volatile uint32_t BKP17R;      
  volatile uint32_t BKP18R;      
  volatile uint32_t BKP19R;      
  volatile uint32_t BKP20R;      
  volatile uint32_t BKP21R;      
  volatile uint32_t BKP22R;      
  volatile uint32_t BKP23R;      
  volatile uint32_t BKP24R;      
  volatile uint32_t BKP25R;      
  volatile uint32_t BKP26R;      
  volatile uint32_t BKP27R;      
  volatile uint32_t BKP28R;      
  volatile uint32_t BKP29R;      
  volatile uint32_t BKP30R;      
  volatile uint32_t BKP31R;      
} RTC_TypeDef;




 
  
typedef struct
{
  volatile uint32_t GCR;       
} SAI_TypeDef;

typedef struct
{
  volatile uint32_t CR1;       
  volatile uint32_t CR2;       
  volatile uint32_t FRCR;      
  volatile uint32_t SLOTR;     
  volatile uint32_t IMR;       
  volatile uint32_t SR;        
  volatile uint32_t CLRFR;     
  volatile uint32_t DR;        
} SAI_Block_TypeDef;



 

typedef struct
{
  volatile uint32_t   CR;            
  volatile uint32_t   IMR;             
  volatile uint32_t   SR;            
  volatile uint32_t   IFCR;           
  volatile uint32_t   DR;            
  volatile uint32_t   CSR;           
  volatile uint32_t   DIR;           
} SPDIFRX_TypeDef;




 

typedef struct
{
  volatile uint32_t POWER;           
  volatile uint32_t CLKCR;           
  volatile uint32_t ARG;             
  volatile uint32_t CMD;             
  volatile const uint32_t  RESPCMD;         
  volatile const uint32_t  RESP1;           
  volatile const uint32_t  RESP2;           
  volatile const uint32_t  RESP3;           
  volatile const uint32_t  RESP4;           
  volatile uint32_t DTIMER;          
  volatile uint32_t DLEN;            
  volatile uint32_t DCTRL;           
  volatile const uint32_t  DCOUNT;          
  volatile const uint32_t  STA;             
  volatile uint32_t ICR;             
  volatile uint32_t MASK;            
  uint32_t      RESERVED0[2];    
  volatile const uint32_t  FIFOCNT;         
  uint32_t      RESERVED1[13];   
  volatile uint32_t FIFO;            
} SDMMC_TypeDef;



 

typedef struct
{
  volatile uint32_t CR1;         
  volatile uint32_t CR2;         
  volatile uint32_t SR;          
  volatile uint32_t DR;          
  volatile uint32_t CRCPR;       
  volatile uint32_t RXCRCR;      
  volatile uint32_t TXCRCR;      
  volatile uint32_t I2SCFGR;     
  volatile uint32_t I2SPR;       
} SPI_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;        
  volatile uint32_t DCR;       
  volatile uint32_t SR;        
  volatile uint32_t FCR;       
  volatile uint32_t DLR;       
  volatile uint32_t CCR;       
  volatile uint32_t AR;        
  volatile uint32_t ABR;       
  volatile uint32_t DR;        
  volatile uint32_t PSMKR;     
  volatile uint32_t PSMAR;                       
  volatile uint32_t PIR;       
  volatile uint32_t LPTR;          
} QUADSPI_TypeDef;



 

typedef struct
{
  volatile uint32_t CR1;          
  volatile uint32_t CR2;          
  volatile uint32_t SMCR;         
  volatile uint32_t DIER;         
  volatile uint32_t SR;           
  volatile uint32_t EGR;          
  volatile uint32_t CCMR1;        
  volatile uint32_t CCMR2;        
  volatile uint32_t CCER;         
  volatile uint32_t CNT;          
  volatile uint32_t PSC;          
  volatile uint32_t ARR;          
  volatile uint32_t RCR;          
  volatile uint32_t CCR1;         
  volatile uint32_t CCR2;         
  volatile uint32_t CCR3;         
  volatile uint32_t CCR4;         
  volatile uint32_t BDTR;         
  volatile uint32_t DCR;          
  volatile uint32_t DMAR;         
  volatile uint32_t OR;           
  volatile uint32_t CCMR3;        
  volatile uint32_t CCR5;         
  volatile uint32_t CCR6;         
  volatile uint32_t AF1;          
  volatile uint32_t AF2;          

} TIM_TypeDef;



 
typedef struct
{
  volatile uint32_t ISR;       
  volatile uint32_t ICR;       
  volatile uint32_t IER;       
  volatile uint32_t CFGR;      
  volatile uint32_t CR;        
  volatile uint32_t CMP;       
  volatile uint32_t ARR;       
  volatile uint32_t CNT;       
} LPTIM_TypeDef;




 
 
typedef struct
{
  volatile uint32_t CR1;      
  volatile uint32_t CR2;      
  volatile uint32_t CR3;     
  volatile uint32_t BRR;                                                    
  volatile uint32_t GTPR;    
  volatile uint32_t RTOR;      
  volatile uint32_t RQR;     
  volatile uint32_t ISR;     
  volatile uint32_t ICR;     
  volatile uint32_t RDR;     
  volatile uint32_t TDR;     
} USART_TypeDef;




 

typedef struct
{
  volatile uint32_t CR;    
  volatile uint32_t CFR;   
  volatile uint32_t SR;    
} WWDG_TypeDef;




 
  
typedef struct 
{
  volatile uint32_t CR;   
  volatile uint32_t SR;   
  volatile uint32_t DR;   
} RNG_TypeDef;



 



 
typedef struct
{
 volatile uint32_t GOTGCTL;                
  volatile uint32_t GOTGINT;               
  volatile uint32_t GAHBCFG;               
  volatile uint32_t GUSBCFG;               
  volatile uint32_t GRSTCTL;               
  volatile uint32_t GINTSTS;               
  volatile uint32_t GINTMSK;               
  volatile uint32_t GRXSTSR;               
  volatile uint32_t GRXSTSP;               
  volatile uint32_t GRXFSIZ;               
  volatile uint32_t DIEPTXF0_HNPTXFSIZ;    
  volatile uint32_t HNPTXSTS;              
  uint32_t Reserved30[2];              
  volatile uint32_t GCCFG;                 
  volatile uint32_t CID;                   
  uint32_t  Reserved5[3];              
  volatile uint32_t GHWCFG3;               
  uint32_t  Reserved6;                  
  volatile uint32_t GLPMCFG;               
  volatile uint32_t GPWRDN;                
  volatile uint32_t GDFIFOCFG;             
   volatile uint32_t GADPCTL;              
    uint32_t  Reserved43[39];          
  volatile uint32_t HPTXFSIZ;              
  volatile uint32_t DIEPTXF[0x0F];         
} USB_OTG_GlobalTypeDef;




 
typedef struct 
{
  volatile uint32_t DCFG;             
  volatile uint32_t DCTL;             
  volatile uint32_t DSTS;             
  uint32_t Reserved0C;            
  volatile uint32_t DIEPMSK;          
  volatile uint32_t DOEPMSK;          
  volatile uint32_t DAINT;            
  volatile uint32_t DAINTMSK;         
  uint32_t  Reserved20;           
  uint32_t Reserved9;             
  volatile uint32_t DVBUSDIS;         
  volatile uint32_t DVBUSPULSE;       
  volatile uint32_t DTHRCTL;          
  volatile uint32_t DIEPEMPMSK;       
  volatile uint32_t DEACHINT;         
  volatile uint32_t DEACHMSK;           
  uint32_t Reserved40;            
  volatile uint32_t DINEP1MSK;        
  uint32_t  Reserved44[15];       
  volatile uint32_t DOUTEP1MSK;          
} USB_OTG_DeviceTypeDef;




 
typedef struct 
{
  volatile uint32_t DIEPCTL;            
  uint32_t Reserved04;              
  volatile uint32_t DIEPINT;            
  uint32_t Reserved0C;              
  volatile uint32_t DIEPTSIZ;           
  volatile uint32_t DIEPDMA;            
  volatile uint32_t DTXFSTS;            
  uint32_t Reserved18;              
} USB_OTG_INEndpointTypeDef;




 
typedef struct 
{
  volatile uint32_t DOEPCTL;        
  uint32_t Reserved04;          
  volatile uint32_t DOEPINT;        
  uint32_t Reserved0C;          
  volatile uint32_t DOEPTSIZ;       
  volatile uint32_t DOEPDMA;        
  uint32_t Reserved18[2];       
} USB_OTG_OUTEndpointTypeDef;




 
typedef struct 
{
  volatile uint32_t HCFG;              
  volatile uint32_t HFIR;              
  volatile uint32_t HFNUM;             
  uint32_t Reserved40C;            
  volatile uint32_t HPTXSTS;           
  volatile uint32_t HAINT;             
  volatile uint32_t HAINTMSK;          
} USB_OTG_HostTypeDef;



 
typedef struct
{
  volatile uint32_t HCCHAR;            
  volatile uint32_t HCSPLT;            
  volatile uint32_t HCINT;             
  volatile uint32_t HCINTMSK;          
  volatile uint32_t HCTSIZ;            
  volatile uint32_t HCDMA;             
  uint32_t Reserved[2];            
} USB_OTG_HostChannelTypeDef;


 



 
typedef struct
{
  volatile uint32_t CONFR0;           
  volatile uint32_t CONFR1;           
  volatile uint32_t CONFR2;           
  volatile uint32_t CONFR3;            
  volatile uint32_t CONFR4;            
  volatile uint32_t CONFR5;            
  volatile uint32_t CONFR6;            
  volatile uint32_t CONFR7;           
  uint32_t  Reserved20[4];        
  volatile uint32_t CR;                
  volatile uint32_t SR;                
  volatile uint32_t CFR;               
  uint32_t  Reserved3c;           
  volatile uint32_t DIR;              
  volatile uint32_t DOR;              
  uint32_t  Reserved48[2];        
  volatile uint32_t QMEM0[16];        
  volatile uint32_t QMEM1[16];        
  volatile uint32_t QMEM2[16];        
  volatile uint32_t QMEM3[16];        
  volatile uint32_t HUFFMIN[16];      
  volatile uint32_t HUFFBASE[32];     
  volatile uint32_t HUFFSYMB[84];     
  volatile uint32_t DHTMEM[103];      
  uint32_t  Reserved4FC;          
  volatile uint32_t HUFFENC_AC0[88];  
  volatile uint32_t HUFFENC_AC1[88];  
  volatile uint32_t HUFFENC_DC0[8];   
  volatile uint32_t HUFFENC_DC1[8];   

} JPEG_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;                
  volatile uint32_t WRFR;              
  volatile uint32_t CWRFR;             
  volatile uint32_t RDFR;               
  volatile uint32_t CRDFR;                
  volatile uint32_t SR;                
  volatile uint32_t CLRFR;                	
  uint32_t RESERVED0[57];          	
  volatile uint32_t DINR0;                 
  volatile uint32_t DINR1;                  
  volatile uint32_t DINR2;                 
  volatile uint32_t DINR3;                  
  volatile uint32_t DINR4;                  
  volatile uint32_t DINR5;                  
  volatile uint32_t DINR6;               
  volatile uint32_t DINR7;              
  volatile uint32_t DINR8;              
  volatile uint32_t DINR9;              
  volatile uint32_t DINR10;             
  volatile uint32_t DINR11;             
  volatile uint32_t DINR12;             
  volatile uint32_t DINR13;             
  volatile uint32_t DINR14;             
  volatile uint32_t DINR15;             
  volatile uint32_t DINR16;             
  volatile uint32_t DINR17;             
  volatile uint32_t DINR18;             
  volatile uint32_t DINR19;             
  volatile uint32_t DINR20;             
  volatile uint32_t DINR21;             
  volatile uint32_t DINR22;             
  volatile uint32_t DINR23;             
  volatile uint32_t DINR24;             
  volatile uint32_t DINR25;             
  volatile uint32_t DINR26;             
  volatile uint32_t DINR27;             
  volatile uint32_t DINR28;             
  volatile uint32_t DINR29;             
  volatile uint32_t DINR30;             
  volatile uint32_t DINR31;             
  volatile uint32_t DOUTR0;               
  volatile uint32_t DOUTR1;               
  volatile uint32_t DOUTR2;                
  volatile uint32_t DOUTR3;                
  volatile uint32_t DOUTR4;               
  volatile uint32_t DOUTR5;                
  volatile uint32_t DOUTR6;                
  volatile uint32_t DOUTR7;            
  volatile uint32_t DOUTR8;            
  volatile uint32_t DOUTR9;            
  volatile uint32_t DOUTR10;           
  volatile uint32_t DOUTR11;           
  volatile uint32_t DOUTR12;           
  volatile uint32_t DOUTR13;           
  volatile uint32_t DOUTR14;           
  volatile uint32_t DOUTR15;           
  volatile uint32_t DOUTR16;           
  volatile uint32_t DOUTR17;           
  volatile uint32_t DOUTR18;           
  volatile uint32_t DOUTR19;           
  volatile uint32_t DOUTR20;           
  volatile uint32_t DOUTR21;           
  volatile uint32_t DOUTR22;           
  volatile uint32_t DOUTR23;           
  volatile uint32_t DOUTR24;           
  volatile uint32_t DOUTR25;           
  volatile uint32_t DOUTR26;           
  volatile uint32_t DOUTR27;           
  volatile uint32_t DOUTR28;           
  volatile uint32_t DOUTR29;           
  volatile uint32_t DOUTR30;           
  volatile uint32_t DOUTR31;           
} MDIOS_TypeDef;




 
#line 1324 "..\\USER\\stm32f767xx.h"

 


 





 
#line 1367 "..\\USER\\stm32f767xx.h"

 
#line 1411 "..\\USER\\stm32f767xx.h"
 
#line 1453 "..\\USER\\stm32f767xx.h"
 



 





 


 



#line 1482 "..\\USER\\stm32f767xx.h"



 
  


   
#line 1608 "..\\USER\\stm32f767xx.h"



 



 
  
  

 
    
 
 
 

 
 
 
 
 
 
#line 1637 "..\\USER\\stm32f767xx.h"

 
#line 1663 "..\\USER\\stm32f767xx.h"
  
 
#line 1689 "..\\USER\\stm32f767xx.h"

 
#line 1727 "..\\USER\\stm32f767xx.h"

 
#line 1769 "..\\USER\\stm32f767xx.h"

 


 


 


 


 


 


 
#line 1818 "..\\USER\\stm32f767xx.h"

 
#line 1856 "..\\USER\\stm32f767xx.h"

 
#line 1894 "..\\USER\\stm32f767xx.h"

 
#line 1923 "..\\USER\\stm32f767xx.h"

 


 


 


 


 



 
#line 1959 "..\\USER\\stm32f767xx.h"

 





 
#line 1987 "..\\USER\\stm32f767xx.h"

 



 
 
 
 
 
 
 
#line 2008 "..\\USER\\stm32f767xx.h"
                                                                           
 
#line 2019 "..\\USER\\stm32f767xx.h"

 
#line 2037 "..\\USER\\stm32f767xx.h"











 





 





 
#line 2075 "..\\USER\\stm32f767xx.h"

 












 
#line 2105 "..\\USER\\stm32f767xx.h"

 
 






 




 





 





 






 




 





 





 






   




 





 





 





 




 





 





 





 




 





 





 
 



 
#line 2246 "..\\USER\\stm32f767xx.h"

 
#line 2263 "..\\USER\\stm32f767xx.h"

 
#line 2280 "..\\USER\\stm32f767xx.h"

 
#line 2297 "..\\USER\\stm32f767xx.h"

 
#line 2331 "..\\USER\\stm32f767xx.h"

 
#line 2365 "..\\USER\\stm32f767xx.h"

 
#line 2399 "..\\USER\\stm32f767xx.h"

 
#line 2433 "..\\USER\\stm32f767xx.h"

 
#line 2467 "..\\USER\\stm32f767xx.h"

 
#line 2501 "..\\USER\\stm32f767xx.h"

 
#line 2535 "..\\USER\\stm32f767xx.h"

 
#line 2569 "..\\USER\\stm32f767xx.h"

 
#line 2603 "..\\USER\\stm32f767xx.h"

 
#line 2637 "..\\USER\\stm32f767xx.h"

 
#line 2671 "..\\USER\\stm32f767xx.h"

 
#line 2705 "..\\USER\\stm32f767xx.h"

 
#line 2739 "..\\USER\\stm32f767xx.h"

 
#line 2773 "..\\USER\\stm32f767xx.h"

 
#line 2807 "..\\USER\\stm32f767xx.h"

 
#line 2841 "..\\USER\\stm32f767xx.h"

 
#line 2875 "..\\USER\\stm32f767xx.h"

 
#line 2909 "..\\USER\\stm32f767xx.h"

 
#line 2943 "..\\USER\\stm32f767xx.h"

 
#line 2977 "..\\USER\\stm32f767xx.h"

 
#line 3011 "..\\USER\\stm32f767xx.h"

 
#line 3045 "..\\USER\\stm32f767xx.h"

 
#line 3079 "..\\USER\\stm32f767xx.h"

 
#line 3113 "..\\USER\\stm32f767xx.h"

 
#line 3147 "..\\USER\\stm32f767xx.h"

 
#line 3181 "..\\USER\\stm32f767xx.h"

 
#line 3215 "..\\USER\\stm32f767xx.h"

 
#line 3249 "..\\USER\\stm32f767xx.h"

 
 
 
 
 

 




 
#line 3271 "..\\USER\\stm32f767xx.h"

 


 


 
#line 3292 "..\\USER\\stm32f767xx.h"

 
#line 3307 "..\\USER\\stm32f767xx.h"

 
 
 
 
 
 


 


 
#line 3328 "..\\USER\\stm32f767xx.h"

 


 



 
 
 
 
 
 
#line 3376 "..\\USER\\stm32f767xx.h"

 



 


 


 


 


 


 


 



 



 



 


 


 



 
 
 
 
 

 

 
#line 3449 "..\\USER\\stm32f767xx.h"

 



 
#line 3461 "..\\USER\\stm32f767xx.h"

 


 



 

 
#line 3493 "..\\USER\\stm32f767xx.h"

 
#line 3504 "..\\USER\\stm32f767xx.h"

 
#line 3515 "..\\USER\\stm32f767xx.h"

 





 


 
#line 3532 "..\\USER\\stm32f767xx.h"

 



 




 



 



 



 



 



 



 


 
 
 
 
 

 
 
 
 
 
 
#line 3601 "..\\USER\\stm32f767xx.h"

 




 






 






 







 







 







 





 





 



 



 





 
 
 
 
 
  
#line 3712 "..\\USER\\stm32f767xx.h"

 
#line 3731 "..\\USER\\stm32f767xx.h"

  
#line 3742 "..\\USER\\stm32f767xx.h"

  
#line 3764 "..\\USER\\stm32f767xx.h"

  
#line 3786 "..\\USER\\stm32f767xx.h"

  
#line 3808 "..\\USER\\stm32f767xx.h"

  
#line 3830 "..\\USER\\stm32f767xx.h"

 
 
 
 
 

 

#line 3851 "..\\USER\\stm32f767xx.h"

 

#line 3860 "..\\USER\\stm32f767xx.h"

 

#line 3869 "..\\USER\\stm32f767xx.h"

 
#line 3877 "..\\USER\\stm32f767xx.h"

 



 



 



 



 

#line 3910 "..\\USER\\stm32f767xx.h"

 





 

#line 3933 "..\\USER\\stm32f767xx.h"

 





 



 



 

#line 3956 "..\\USER\\stm32f767xx.h"

 

 






 




 





 





 



 



 




 



 





 
                                                                     
 


 
 
 
 
 
 
#line 4042 "..\\USER\\stm32f767xx.h"

 
#line 4069 "..\\USER\\stm32f767xx.h"



 
#line 4098 "..\\USER\\stm32f767xx.h"

 
#line 4125 "..\\USER\\stm32f767xx.h"


 
#line 4153 "..\\USER\\stm32f767xx.h"

 
#line 4180 "..\\USER\\stm32f767xx.h"

 
#line 4207 "..\\USER\\stm32f767xx.h"

 
#line 4234 "..\\USER\\stm32f767xx.h"

 
 
 
 
 


 


 
#line 4266 "..\\USER\\stm32f767xx.h"

 
#line 4275 "..\\USER\\stm32f767xx.h"

 
#line 4295 "..\\USER\\stm32f767xx.h"

 
#line 4332 "..\\USER\\stm32f767xx.h"

 



 
 
 
 
 
 
#line 4367 "..\\USER\\stm32f767xx.h"

 
#line 4391 "..\\USER\\stm32f767xx.h"

 
#line 4415 "..\\USER\\stm32f767xx.h"

 
#line 4439 "..\\USER\\stm32f767xx.h"

 
#line 4478 "..\\USER\\stm32f767xx.h"

 
#line 4517 "..\\USER\\stm32f767xx.h"

 
#line 4556 "..\\USER\\stm32f767xx.h"

 
#line 4595 "..\\USER\\stm32f767xx.h"

 
#line 4624 "..\\USER\\stm32f767xx.h"

 
#line 4653 "..\\USER\\stm32f767xx.h"

 
#line 4682 "..\\USER\\stm32f767xx.h"

 
#line 4711 "..\\USER\\stm32f767xx.h"

 
#line 4734 "..\\USER\\stm32f767xx.h"

 
#line 4743 "..\\USER\\stm32f767xx.h"

 
#line 4781 "..\\USER\\stm32f767xx.h"

 
#line 4819 "..\\USER\\stm32f767xx.h"

 


 
#line 4845 "..\\USER\\stm32f767xx.h"

 
#line 4868 "..\\USER\\stm32f767xx.h"

 
#line 4901 "..\\USER\\stm32f767xx.h"

 
#line 4934 "..\\USER\\stm32f767xx.h"

 
#line 4948 "..\\USER\\stm32f767xx.h"

 




 
#line 4963 "..\\USER\\stm32f767xx.h"

 
 
 
 
 
 
#line 5018 "..\\USER\\stm32f767xx.h"

 
#line 5036 "..\\USER\\stm32f767xx.h"

 
#line 5086 "..\\USER\\stm32f767xx.h"

 
#line 5136 "..\\USER\\stm32f767xx.h"

 
#line 5154 "..\\USER\\stm32f767xx.h"

 
#line 5172 "..\\USER\\stm32f767xx.h"

 
#line 5206 "..\\USER\\stm32f767xx.h"

 
#line 5225 "..\\USER\\stm32f767xx.h"


 
 
 
 
 
 
#line 5252 "..\\USER\\stm32f767xx.h"


 
#line 5266 "..\\USER\\stm32f767xx.h"

 




 
#line 5284 "..\\USER\\stm32f767xx.h"

 






 






 
#line 5317 "..\\USER\\stm32f767xx.h"

 
#line 5328 "..\\USER\\stm32f767xx.h"

 


 


 



 
 
 
 
 
 


 





 


 




 


 
 
 
 
 

 




 




 




 




 

#line 5401 "..\\USER\\stm32f767xx.h"


 




 





 






 






 






 



 




 






 





 




 




 





 



 



 





                                
 




 



 




 



 






 
 
 
 
 
 
#line 5527 "..\\USER\\stm32f767xx.h"

 
#line 5550 "..\\USER\\stm32f767xx.h"

 
#line 5562 "..\\USER\\stm32f767xx.h"


 
#line 5577 "..\\USER\\stm32f767xx.h"

 
#line 5591 "..\\USER\\stm32f767xx.h"

 
 
 
 
 
 
#line 5627 "..\\USER\\stm32f767xx.h"

 
#line 5640 "..\\USER\\stm32f767xx.h"

 
#line 5654 "..\\USER\\stm32f767xx.h"

 





 


 
#line 5704 "..\\USER\\stm32f767xx.h"
 


 


 


 


 


 


 


 
 
 
 
 
 
#line 5758 "..\\USER\\stm32f767xx.h"

 
#line 5788 "..\\USER\\stm32f767xx.h"






 
 
#line 5802 "..\\USER\\stm32f767xx.h"

 
#line 5810 "..\\USER\\stm32f767xx.h"

 






#line 5827 "..\\USER\\stm32f767xx.h"

 











 











 
#line 5859 "..\\USER\\stm32f767xx.h"

 




















 
#line 5905 "..\\USER\\stm32f767xx.h"

 
#line 5924 "..\\USER\\stm32f767xx.h"

 





 




 
#line 5967 "..\\USER\\stm32f767xx.h"

 
#line 5989 "..\\USER\\stm32f767xx.h"

 
#line 6014 "..\\USER\\stm32f767xx.h"

 





 



 
#line 6057 "..\\USER\\stm32f767xx.h"

 
#line 6081 "..\\USER\\stm32f767xx.h"

 
#line 6110 "..\\USER\\stm32f767xx.h"

 





 


 
#line 6152 "..\\USER\\stm32f767xx.h"

 
#line 6176 "..\\USER\\stm32f767xx.h"

 
#line 6189 "..\\USER\\stm32f767xx.h"

 
#line 6201 "..\\USER\\stm32f767xx.h"

 





 
#line 6231 "..\\USER\\stm32f767xx.h"

 
#line 6255 "..\\USER\\stm32f767xx.h"

 
#line 6263 "..\\USER\\stm32f767xx.h"

#line 6270 "..\\USER\\stm32f767xx.h"

















 
#line 6331 "..\\USER\\stm32f767xx.h"

 
 
 
 
 
 



 






 
 
 
 
 
 
#line 6381 "..\\USER\\stm32f767xx.h"

 
#line 6411 "..\\USER\\stm32f767xx.h"

 
#line 6439 "..\\USER\\stm32f767xx.h"

 
#line 6459 "..\\USER\\stm32f767xx.h"

 



 


 
#line 6508 "..\\USER\\stm32f767xx.h"

 
#line 6550 "..\\USER\\stm32f767xx.h"

 


 


 



 
#line 6589 "..\\USER\\stm32f767xx.h"

 
#line 6609 "..\\USER\\stm32f767xx.h"

 


 
#line 6627 "..\\USER\\stm32f767xx.h"

 
#line 6657 "..\\USER\\stm32f767xx.h"


 
#line 6666 "..\\USER\\stm32f767xx.h"

 
#line 6674 "..\\USER\\stm32f767xx.h"

 





 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 



 
 
 
 
 
 



                                             




 



                                             



                                             




                                             


                                             



                                             





                                             






 




                                             




                                             
#line 6846 "..\\USER\\stm32f767xx.h"
                                             

                                             




 
#line 6863 "..\\USER\\stm32f767xx.h"
                                                        
#line 6872 "..\\USER\\stm32f767xx.h"
                                                        



                                             
                           


 
#line 6887 "..\\USER\\stm32f767xx.h"
                                            



                                            





                                            


 
#line 6908 "..\\USER\\stm32f767xx.h"

 
#line 6917 "..\\USER\\stm32f767xx.h"
                                             





 
#line 6931 "..\\USER\\stm32f767xx.h"

 


 
 
 
 
 
 
#line 6954 "..\\USER\\stm32f767xx.h"

 
#line 6963 "..\\USER\\stm32f767xx.h"

 
#line 6975 "..\\USER\\stm32f767xx.h"

 





 
#line 6989 "..\\USER\\stm32f767xx.h"

 
#line 6997 "..\\USER\\stm32f767xx.h"

 



 




 




 
 
 
 
 
 




 




         



         



 


 

         



         





 


 


 


 


 


 


 


 


 
















 


 
#line 7117 "..\\USER\\stm32f767xx.h"

 
#line 7130 "..\\USER\\stm32f767xx.h"

 
#line 7154 "..\\USER\\stm32f767xx.h"

 


 


 
 
 
 
 
 
#line 7184 "..\\USER\\stm32f767xx.h"

 
#line 7202 "..\\USER\\stm32f767xx.h"

 
#line 7219 "..\\USER\\stm32f767xx.h"

 


 


 


 


 
#line 7248 "..\\USER\\stm32f767xx.h"

 





 
 
 
 
 
   








 
#line 7278 "..\\USER\\stm32f767xx.h"








 






  
#line 7305 "..\\USER\\stm32f767xx.h"



  
#line 7320 "..\\USER\\stm32f767xx.h"



  
#line 7335 "..\\USER\\stm32f767xx.h"



  
#line 7350 "..\\USER\\stm32f767xx.h"

 






  
#line 7370 "..\\USER\\stm32f767xx.h"



  
#line 7385 "..\\USER\\stm32f767xx.h"



  
#line 7400 "..\\USER\\stm32f767xx.h"



  
#line 7415 "..\\USER\\stm32f767xx.h"

 




           


  
#line 7435 "..\\USER\\stm32f767xx.h"



  
#line 7449 "..\\USER\\stm32f767xx.h"



  
#line 7463 "..\\USER\\stm32f767xx.h"



  
#line 7477 "..\\USER\\stm32f767xx.h"


 






  
#line 7497 "..\\USER\\stm32f767xx.h"



  
#line 7511 "..\\USER\\stm32f767xx.h"



  
#line 7525 "..\\USER\\stm32f767xx.h"



  
#line 7539 "..\\USER\\stm32f767xx.h"

   



   



 
 
 
 
 
 

















 


















#line 7598 "..\\USER\\stm32f767xx.h"

 
#line 7606 "..\\USER\\stm32f767xx.h"





















 
#line 7643 "..\\USER\\stm32f767xx.h"

 
#line 7658 "..\\USER\\stm32f767xx.h"

 
#line 7669 "..\\USER\\stm32f767xx.h"

 






























 





















 
































 





















 
#line 7798 "..\\USER\\stm32f767xx.h"


 


 


 


 


 


 


 


 


 
#line 7834 "..\\USER\\stm32f767xx.h"





#line 7849 "..\\USER\\stm32f767xx.h"

 
#line 7857 "..\\USER\\stm32f767xx.h"

#line 7864 "..\\USER\\stm32f767xx.h"

 


 
#line 7875 "..\\USER\\stm32f767xx.h"

 






















 





 


 



 



 



 



 
 
 
 
 
 
#line 7937 "..\\USER\\stm32f767xx.h"

 
#line 7946 "..\\USER\\stm32f767xx.h"

 
#line 7955 "..\\USER\\stm32f767xx.h"

 




























#line 7991 "..\\USER\\stm32f767xx.h"

 




 


 


 

 
 
 
 
 
 
#line 8042 "..\\USER\\stm32f767xx.h"

 
#line 8066 "..\\USER\\stm32f767xx.h"

 
#line 8088 "..\\USER\\stm32f767xx.h"


 



 




 



 






 
#line 8133 "..\\USER\\stm32f767xx.h"


 
#line 8148 "..\\USER\\stm32f767xx.h"

 


 


 
 
 
 
 
 
#line 8169 "..\\USER\\stm32f767xx.h"




 
#line 8182 "..\\USER\\stm32f767xx.h"









 


 
 
 
 
 
 



 









 
#line 8232 "..\\USER\\stm32f767xx.h"

 






 
 
 
 
 
 
#line 8274 "..\\USER\\stm32f767xx.h"

 
#line 8290 "..\\USER\\stm32f767xx.h"

 


 


 
#line 8308 "..\\USER\\stm32f767xx.h"
  
 


 
#line 8324 "..\\USER\\stm32f767xx.h"

 



  


 








 

  
#line 8351 "..\\USER\\stm32f767xx.h"

 






 



 


 


 
#line 8380 "..\\USER\\stm32f767xx.h"

 


 
#line 8395 "..\\USER\\stm32f767xx.h"

 


 
#line 8410 "..\\USER\\stm32f767xx.h"

 


 
 
 

 
#line 8425 "..\\USER\\stm32f767xx.h"

 




 




 




 




 


 


 


 


 


 


 
 
 

 
#line 8478 "..\\USER\\stm32f767xx.h"

#line 8485 "..\\USER\\stm32f767xx.h"

 


 


 



 


 



 


 


 


 



 
 
 

 
#line 8560 "..\\USER\\stm32f767xx.h"

 


 


 


 


 




   
#line 8611 "..\\USER\\stm32f767xx.h"

 
#line 8637 "..\\USER\\stm32f767xx.h"

 
#line 8654 "..\\USER\\stm32f767xx.h"

 





 


 


 


 


 
 
 
 
 
 
#line 8697 "..\\USER\\stm32f767xx.h"

 





 





#line 8718 "..\\USER\\stm32f767xx.h"









 




 
#line 8740 "..\\USER\\stm32f767xx.h"

 





#line 8756 "..\\USER\\stm32f767xx.h"

 


 



 








 
#line 8783 "..\\USER\\stm32f767xx.h"

 
#line 8810 "..\\USER\\stm32f767xx.h"

 
#line 8825 "..\\USER\\stm32f767xx.h"

 
#line 8835 "..\\USER\\stm32f767xx.h"

 
#line 8847 "..\\USER\\stm32f767xx.h"

#line 8857 "..\\USER\\stm32f767xx.h"

 


 
#line 8870 "..\\USER\\stm32f767xx.h"

 
#line 8900 "..\\USER\\stm32f767xx.h"

 
#line 8930 "..\\USER\\stm32f767xx.h"

 



 


 





 



 

#line 8956 "..\\USER\\stm32f767xx.h"























 

#line 8987 "..\\USER\\stm32f767xx.h"























 


 


 





 


 


#line 9037 "..\\USER\\stm32f767xx.h"

#line 9046 "..\\USER\\stm32f767xx.h"

 



#line 9062 "..\\USER\\stm32f767xx.h"

#line 9074 "..\\USER\\stm32f767xx.h"

 


 



 



 



 


 
 


 
#line 9113 "..\\USER\\stm32f767xx.h"

 
#line 9124 "..\\USER\\stm32f767xx.h"

 
#line 9135 "..\\USER\\stm32f767xx.h"
















 
#line 9163 "..\\USER\\stm32f767xx.h"

 



 










#line 9190 "..\\USER\\stm32f767xx.h"

 


#line 9201 "..\\USER\\stm32f767xx.h"









#line 9221 "..\\USER\\stm32f767xx.h"

 

#line 9232 "..\\USER\\stm32f767xx.h"

#line 9241 "..\\USER\\stm32f767xx.h"







 
#line 9260 "..\\USER\\stm32f767xx.h"

 
#line 9273 "..\\USER\\stm32f767xx.h"

 
#line 9286 "..\\USER\\stm32f767xx.h"

 




 
#line 9299 "..\\USER\\stm32f767xx.h"

 


 


 


 



 
#line 9328 "..\\USER\\stm32f767xx.h"

 
#line 9337 "..\\USER\\stm32f767xx.h"

 







 




 
 
 
 
 
 


 
#line 9373 "..\\USER\\stm32f767xx.h"

 


 



 
#line 9402 "..\\USER\\stm32f767xx.h"

 
#line 9424 "..\\USER\\stm32f767xx.h"

 
#line 9446 "..\\USER\\stm32f767xx.h"

 
#line 9468 "..\\USER\\stm32f767xx.h"

 
#line 9481 "..\\USER\\stm32f767xx.h"

 
#line 9490 "..\\USER\\stm32f767xx.h"

 



 


 


 
 
 
 
 
 
#line 9518 "..\\USER\\stm32f767xx.h"

 


 


 


 


 




 






 



 



 

 




 



 


 


 


 





#line 9585 "..\\USER\\stm32f767xx.h"

 


 
#line 9606 "..\\USER\\stm32f767xx.h"

 
#line 9619 "..\\USER\\stm32f767xx.h"
										
#line 9631 "..\\USER\\stm32f767xx.h"

 


 


                                        
 





 




 


 


 


 


 


 


 




 


 



 

                                     
 
#line 9690 "..\\USER\\stm32f767xx.h"

 
#line 9706 "..\\USER\\stm32f767xx.h"

 
#line 9720 "..\\USER\\stm32f767xx.h"

 
#line 9730 "..\\USER\\stm32f767xx.h"

 
#line 9738 "..\\USER\\stm32f767xx.h"

 
#line 9746 "..\\USER\\stm32f767xx.h"
                                       
 




 
#line 9759 "..\\USER\\stm32f767xx.h"

 
#line 9768 "..\\USER\\stm32f767xx.h"
 
 
#line 9777 "..\\USER\\stm32f767xx.h"
 




 




                                       
 



 



   
 
#line 9804 "..\\USER\\stm32f767xx.h"

 
#line 9814 "..\\USER\\stm32f767xx.h"

 
#line 9822 "..\\USER\\stm32f767xx.h"

 
#line 9830 "..\\USER\\stm32f767xx.h"

 
#line 9838 "..\\USER\\stm32f767xx.h"

 
#line 9850 "..\\USER\\stm32f767xx.h"

 
#line 9860 "..\\USER\\stm32f767xx.h"

 



 
#line 9872 "..\\USER\\stm32f767xx.h"

 




 
#line 9935 "..\\USER\\stm32f767xx.h"

 
#line 9947 "..\\USER\\stm32f767xx.h"

 




 
#line 9963 "..\\USER\\stm32f767xx.h"
      
 





 
#line 9979 "..\\USER\\stm32f767xx.h"

 
#line 9989 "..\\USER\\stm32f767xx.h"

 
#line 9999 "..\\USER\\stm32f767xx.h"

 





 
#line 10015 "..\\USER\\stm32f767xx.h"

 


 



 
 
 
 
 
 
 

 


 




 



 



 
  







 
#line 135 "..\\USER\\stm32f7xx.h"
#line 144 "..\\USER\\stm32f7xx.h"



 



  
typedef enum 
{
  RESET = 0, 
  SET = !RESET
} FlagStatus, ITStatus;

typedef enum 
{
  DISABLE = 0, 
  ENABLE = !DISABLE
} FunctionalState;


typedef enum 
{
  ERROR = 0, 
  SUCCESS = !ERROR
} ErrorStatus;



 
  


 


















 

#line 1 "..\\USER\\stm32f7xx_hal_conf.h"



































  

 
#line 444 "..\\USER\\stm32f7xx_hal_conf.h"
 

 
#line 200 "..\\USER\\stm32f7xx.h"










 

  

 

 
#line 49 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_def.h"
#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"




































 

 







 
 
 



 








 
  


 
#line 107 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 115 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 
  


  
  




    
   


 
#line 146 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 202 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 



 



 



 
  





 



 

#line 238 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 



 
#line 261 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"
  



  
  
  


 



 
  
#line 347 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 
  


 
  
#line 364 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 
  



 
#line 383 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 



 
  




 



 






















#line 428 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 435 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"










 



 
#line 459 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"
   
#line 468 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 



 
#line 491 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 



 





 



 






 



 















 
 






 



 














 
   


 










 



 





                                              















                                                                      



                                                        


 



 






 



 

 
#line 636 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

 












 
  


 
  









#line 673 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"
















 

  


 















 

  


 
#line 727 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 
  


 











 
  


 


  
#line 780 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 790 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 809 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 



 




 



 























 

  


 








 



 




 



 
#line 887 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 
  


 

#line 904 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 916 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"
 
#line 947 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 
  


 











   
  




 






#line 990 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


     




 
  


 

 



 



   



  
#line 1022 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

 













 
  


 
#line 1057 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 



 
#line 1071 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

 

 



 






 

 



 
#line 1108 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 1116 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"




#line 1130 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 
 

   
  


 





 



 



   



 






 
   


  



 
  


  



   
   
  


 
  


 

 



 





   
  


 
#line 1219 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"









 

   


 
#line 1247 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 1268 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 1279 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 1288 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 1302 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 1311 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 



 







 
   


 
#line 1347 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 1362 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


#line 1388 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 



 
#line 1555 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



#line 1565 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 

#line 1579 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 




  


 



 

#line 1602 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 
  


 
  
#line 1626 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 
  


 
  





 



 
  













 




 




 




 







 
  
  


 
#line 1700 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 




 
#line 1744 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 1758 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


   
  
  


 
  






#line 2273 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 2422 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

 



#line 2449 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 2472 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 2589 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 2606 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 2621 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"
























#line 2667 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



#line 2678 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 2711 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 2729 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"












#line 2747 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 2764 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"



 



 




 
  


 
  




#line 2813 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 2828 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"






 



 




#line 2868 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 2890 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"
 





 



 

#line 2908 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"








 



 
#line 2929 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 



 







 
  


 













 




 











 



 












#line 3002 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 3011 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"

#line 3020 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"








 



 








#line 3053 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"




 



 
  
#line 3070 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"






 



 



 



 
#line 3103 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\Legacy/stm32_hal_legacy.h"


 




 
  


 







 

#line 50 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_def.h"
#line 1 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"
 
 
 





 






 







 




  
 








#line 47 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"


  



    typedef unsigned int size_t;    









 
 

 



    typedef struct __va_list __va_list;






   




 




typedef struct __fpos_t_struct {
    unsigned __int64 __pos;
    



 
    struct {
        unsigned int __state1, __state2;
    } __mbstate;
} fpos_t;
   


 


   

 

typedef struct __FILE FILE;
   






 

#line 136 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"


extern FILE __stdin, __stdout, __stderr;
extern FILE *__aeabi_stdin, *__aeabi_stdout, *__aeabi_stderr;

#line 166 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"
    

    

    





     



   


 


   


 

   



 

   


 




   


 





    


 






extern __declspec(__nothrow) int remove(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int rename(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) FILE *tmpfile(void);
   




 
extern __declspec(__nothrow) char *tmpnam(char *  );
   











 

extern __declspec(__nothrow) int fclose(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) int fflush(FILE *  );
   







 
extern __declspec(__nothrow) FILE *fopen(const char * __restrict  ,
                           const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   








































 
extern __declspec(__nothrow) FILE *freopen(const char * __restrict  ,
                    const char * __restrict  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(2,3)));
   








 
extern __declspec(__nothrow) void setbuf(FILE * __restrict  ,
                    char * __restrict  ) __attribute__((__nonnull__(1)));
   




 
extern __declspec(__nothrow) int setvbuf(FILE * __restrict  ,
                   char * __restrict  ,
                   int  , size_t  ) __attribute__((__nonnull__(1)));
   















 
#pragma __printf_args
extern __declspec(__nothrow) int fprintf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   


















 
#pragma __printf_args
extern __declspec(__nothrow) int _fprintf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   




 
#pragma __printf_args
extern __declspec(__nothrow) int _printf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __printf_args
extern __declspec(__nothrow) int sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






 
#pragma __printf_args
extern __declspec(__nothrow) int _sprintf(char * __restrict  , const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

#pragma __printf_args
extern __declspec(__nothrow) int __ARM_snprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(3)));


#pragma __printf_args
extern __declspec(__nothrow) int snprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   















 

#pragma __printf_args
extern __declspec(__nothrow) int _snprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , ...) __attribute__((__nonnull__(3)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int fscanf(FILE * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   






























 
#pragma __scanf_args
extern __declspec(__nothrow) int _fscanf(FILE * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   






 
#pragma __scanf_args
extern __declspec(__nothrow) int _scanf(const char * __restrict  , ...) __attribute__((__nonnull__(1)));
   



 
#pragma __scanf_args
extern __declspec(__nothrow) int sscanf(const char * __restrict  ,
                    const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   








 
#pragma __scanf_args
extern __declspec(__nothrow) int _sscanf(const char * __restrict  ,
                     const char * __restrict  , ...) __attribute__((__nonnull__(1,2)));
   



 

 
extern __declspec(__nothrow) int vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int _vfscanf(FILE * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int _vscanf(const char * __restrict  , __va_list) __attribute__((__nonnull__(1)));
extern __declspec(__nothrow) int _vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) int __ARM_vsscanf(const char * __restrict  , const char * __restrict  , __va_list) __attribute__((__nonnull__(1,2)));

extern __declspec(__nothrow) int vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int _vprintf(const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) int vfprintf(FILE * __restrict  ,
                    const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int vsprintf(char * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int __ARM_vsnprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));

extern __declspec(__nothrow) int vsnprintf(char * __restrict  , size_t  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   







 

extern __declspec(__nothrow) int _vsprintf(char * __restrict  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vfprintf(FILE * __restrict  ,
                     const char * __restrict  , __va_list  ) __attribute__((__nonnull__(1,2)));
   



 
extern __declspec(__nothrow) int _vsnprintf(char * __restrict  , size_t  ,
                      const char * __restrict  , __va_list  ) __attribute__((__nonnull__(3)));
   



 

#pragma __printf_args
extern __declspec(__nothrow) int asprintf(char **  , const char * __restrict  , ...) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) int vasprintf(char **  , const char * __restrict  , __va_list  ) __attribute__((__nonnull__(2)));

#pragma __printf_args
extern __declspec(__nothrow) int __ARM_asprintf(char **  , const char * __restrict  , ...) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) int __ARM_vasprintf(char **  , const char * __restrict  , __va_list  ) __attribute__((__nonnull__(2)));
   








 

extern __declspec(__nothrow) int fgetc(FILE *  ) __attribute__((__nonnull__(1)));
   







 
extern __declspec(__nothrow) char *fgets(char * __restrict  , int  ,
                    FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   










 
extern __declspec(__nothrow) int fputc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   







 
extern __declspec(__nothrow) int fputs(const char * __restrict  , FILE * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) int getc(FILE *  ) __attribute__((__nonnull__(1)));
   







 




    extern __declspec(__nothrow) int (getchar)(void);

   





 
extern __declspec(__nothrow) char *gets(char *  ) __attribute__((__nonnull__(1)));
   









 
extern __declspec(__nothrow) int putc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   





 




    extern __declspec(__nothrow) int (putchar)(int  );

   



 
extern __declspec(__nothrow) int puts(const char *  ) __attribute__((__nonnull__(1)));
   





 
extern __declspec(__nothrow) int ungetc(int  , FILE *  ) __attribute__((__nonnull__(2)));
   






















 

extern __declspec(__nothrow) size_t fread(void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   











 

extern __declspec(__nothrow) size_t __fread_bytes_avail(void * __restrict  ,
                    size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,3)));
   











 

extern __declspec(__nothrow) size_t fwrite(const void * __restrict  ,
                    size_t  , size_t  , FILE * __restrict  ) __attribute__((__nonnull__(1,4)));
   







 

extern __declspec(__nothrow) int fgetpos(FILE * __restrict  , fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   








 
extern __declspec(__nothrow) int fseek(FILE *  , long int  , int  ) __attribute__((__nonnull__(1)));
   














 
extern __declspec(__nothrow) int fsetpos(FILE * __restrict  , const fpos_t * __restrict  ) __attribute__((__nonnull__(1,2)));
   










 
extern __declspec(__nothrow) long int ftell(FILE *  ) __attribute__((__nonnull__(1)));
   











 
extern __declspec(__nothrow) void rewind(FILE *  ) __attribute__((__nonnull__(1)));
   





 

extern __declspec(__nothrow) void clearerr(FILE *  ) __attribute__((__nonnull__(1)));
   




 

extern __declspec(__nothrow) int feof(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) int ferror(FILE *  ) __attribute__((__nonnull__(1)));
   


 
extern __declspec(__nothrow) void perror(const char *  );
   









 

extern __declspec(__nothrow) int _fisatty(FILE *   ) __attribute__((__nonnull__(1)));
    
 

extern __declspec(__nothrow) void __use_no_semihosting_swi(void);
extern __declspec(__nothrow) void __use_no_semihosting(void);
    





 











#line 1021 "D:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdio.h"



 

#line 51 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_def.h"
 



   
typedef enum 
{
  HAL_OK       = 0x00U,
  HAL_ERROR    = 0x01U,
  HAL_BUSY     = 0x02U,
  HAL_TIMEOUT  = 0x03U
} HAL_StatusTypeDef;



 
typedef enum 
{
  HAL_UNLOCKED = 0x00,
  HAL_LOCKED   = 0x01  
} HAL_LockTypeDef;

 



























 


#line 119 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_def.h"







#line 134 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_def.h"


 
#line 156 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_def.h"




  









 


#line 189 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_def.h"



  



 


#line 206 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_def.h"







 
#line 48 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc.h"
   
 
 
#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"



































  

 







 
#line 48 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"



 



  

 


 
   


 
typedef struct
{
  uint32_t PLLState;   
 

  uint32_t PLLSource;  
            

  uint32_t PLLM;       
         

  uint32_t PLLN;       
 

  uint32_t PLLP;       
 

  uint32_t PLLQ;       
 

  uint32_t PLLR;       
 


}RCC_PLLInitTypeDef;   



 
typedef struct
{
  uint32_t PLLI2SN;    

 

  uint32_t PLLI2SR;    

 

  uint32_t PLLI2SQ;    

 

  uint32_t PLLI2SP;    

 
}RCC_PLLI2SInitTypeDef;



 
typedef struct
{
  uint32_t PLLSAIN;    

  
                                 
  uint32_t PLLSAIQ;    

 
                              
  uint32_t PLLSAIR;    

 

  uint32_t PLLSAIP;    

 
}RCC_PLLSAIInitTypeDef;



 
typedef struct
{
  uint32_t PeriphClockSelection; 
 

  RCC_PLLI2SInitTypeDef PLLI2S;  
 

  RCC_PLLSAIInitTypeDef PLLSAI;  
 

  uint32_t PLLI2SDivQ;           

 

  uint32_t PLLSAIDivQ;           

 

  uint32_t PLLSAIDivR;           
 

  uint32_t RTCClockSelection;      
 
                                        
  uint32_t I2sClockSelection;      
 

  uint32_t TIMPresSelection;      
 
  
  uint32_t Sai1ClockSelection;     
 

  uint32_t Sai2ClockSelection;     
 
  
  uint32_t Usart1ClockSelection; 
 
  
  uint32_t Usart2ClockSelection; 
 

  uint32_t Usart3ClockSelection; 
                                 
  
  uint32_t Uart4ClockSelection;  
 
  
  uint32_t Uart5ClockSelection;  
 
  
  uint32_t Usart6ClockSelection;  
 
  
  uint32_t Uart7ClockSelection;  
 
  
  uint32_t Uart8ClockSelection;  
 
  
  uint32_t I2c1ClockSelection;   
 

  uint32_t I2c2ClockSelection;   
 

  uint32_t I2c3ClockSelection;   
 
  
  uint32_t I2c4ClockSelection;   
 
  
  uint32_t Lptim1ClockSelection;   
 
  
  uint32_t CecClockSelection;      
 
  
  uint32_t Clk48ClockSelection;    
 
  
  uint32_t Sdmmc1ClockSelection;     
 
                                          

  uint32_t Sdmmc2ClockSelection;     
 
  
  uint32_t Dfsdm1ClockSelection;     
 
                                          
  uint32_t Dfsdm1AudioClockSelection; 
 

}RCC_PeriphCLKInitTypeDef;


 

 


 



 
#line 277 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"
    


 



 






 
  


 






 



 






 



 





  
  
  


 
#line 335 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"


  



 
#line 348 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"


  



 




 



 






 



 






 



 






 



 






 



 






 



 






 



 






 



 






 



 





 



 






 



 





 



 





 



 







 



 




 



 




 



 




 




 




  



 




 



 




 


#line 568 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"



 
     
 


 






 
 




 
#line 597 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"
                                      
#line 605 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"
                                      
#line 613 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 621 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 629 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 637 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 645 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 653 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 661 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 669 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 677 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 685 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 693 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 701 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 709 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 717 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 725 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 743 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"


 
#line 753 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 761 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 769 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 777 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"
                                      







 
#line 795 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"
                                     




 
#line 808 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 819 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 827 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 836 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"
                                      



#line 861 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"
                                        




 
#line 874 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 882 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"








 
#line 898 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 906 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 914 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 922 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 930 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 938 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 946 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 954 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 962 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 970 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 979 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"
                                        
#line 988 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"
                                        
#line 996 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 1004 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 1012 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 1020 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 1028 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 1036 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 1044 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 1052 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 1060 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 1068 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 1076 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 1084 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 1092 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 1100 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 1108 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 1116 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 1124 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 1156 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"





 
#line 1169 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 1177 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 1185 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 1193 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 1203 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 1211 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 1219 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 1227 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 1235 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 1243 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 1251 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 1259 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 1267 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 1275 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 1283 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 1291 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 1299 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 1307 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 1317 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"
                                        
#line 1327 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"
                                        
#line 1336 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 1345 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"
                                        
#line 1376 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"
                                        


 








 
 




 
#line 1412 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 1430 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"


 
#line 1440 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"



 
#line 1451 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"





 



                                   




#line 1471 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"










   










 
#line 1524 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 1556 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"





 
#line 1618 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"


   




 
  

   
#line 1644 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 1660 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"
 

 















#line 1684 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"


  







 

  
#line 1727 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 1758 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"


 
#line 1779 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 1798 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

















  








  
  

  
#line 1853 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 1879 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"






 














#line 1907 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"






 











   
#line 1957 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 1989 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"






  
#line 2016 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 2045 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"







 








 
  





 
#line 2094 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 2120 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"






 







                                         






#line 2148 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"






 











   
#line 2198 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 2230 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"






  
#line 2265 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 2294 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"


 

 





























 
#line 2369 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"
 












      






 

















    






















 





    





 







 








    

















 














 

















 















 




 



 



 




 




 






 









 








 









 








 









 








 









 








 










 









 










 









 










 









 


 







 









 


 







 









 


 







 









 


 







 









 










 









 










 









 








 







 








 







 








 







 








 







 

                    





 







 







 







 



#line 2941 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"


 

 


 
HAL_StatusTypeDef HAL_RCCEx_PeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit);
void HAL_RCCEx_GetPeriphCLKConfig(RCC_PeriphCLKInitTypeDef  *PeriphClkInit);
uint32_t HAL_RCCEx_GetPeriphCLKFreq(uint32_t PeriphClk);



  
 


 


 
#line 3075 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 3083 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"





#line 3094 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"




#line 3105 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 3116 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

























#line 3154 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 3174 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"

#line 3183 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc_ex.h"
                 


                 
























                 


  



 



  



   






 
#line 52 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc.h"



 



 

  



 



 
typedef struct
{
  uint32_t OscillatorType;       
 

  uint32_t HSEState;             
 

  uint32_t LSEState;             
 
                                          
  uint32_t HSIState;             
 

  uint32_t HSICalibrationValue;   
 
                               
  uint32_t LSIState;             
 

  RCC_PLLInitTypeDef PLL;               

}RCC_OscInitTypeDef;



 
typedef struct
{
  uint32_t ClockType;             
 
  
  uint32_t SYSCLKSource;          
 

  uint32_t AHBCLKDivider;         
 

  uint32_t APB1CLKDivider;        
 

  uint32_t APB2CLKDivider;        
 

}RCC_ClkInitTypeDef;



 

 


 



 







 



 





 



 





 



 






 



 




 



 





 



 






 



 




 



 






 
  


 





 




 





 



 
#line 251 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc.h"


  
  


 







  



 
#line 302 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc.h"


 





 




 



 






 



 






 



 







 



 
#line 362 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc.h"


 
  








 
 






 


 
#line 394 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc.h"


  



 






 
  


 
   
 


 







 
#line 432 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc.h"
									  
#line 440 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc.h"






 







 
#line 462 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc.h"
									  
#line 470 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc.h"





 







 
#line 491 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc.h"
									  




 
  






 







 
  






 







   







 




   
  



   









 




 









 




 








 







 











 











 





 
  







 







 








 







 








 




   



  
                                      













 








 




 



  








 




 



  





















 
#line 771 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc.h"


 



 


















 
#line 819 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc.h"


 



 



 























 


                                                   








 




 



 







 


                            







 










 



 



 








 






 




 



 







 








      















 








 



 
  


  
  














 



                














 





 
  



 











 












 













 













 




 



















 





 
     


 

 
#line 1125 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc.h"

 
 

 



                              
 
void HAL_RCC_DeInit(void);
HAL_StatusTypeDef HAL_RCC_OscConfig(RCC_OscInitTypeDef *RCC_OscInitStruct);
HAL_StatusTypeDef HAL_RCC_ClockConfig(RCC_ClkInitTypeDef *RCC_ClkInitStruct, uint32_t FLatency);


 



 
 
void     HAL_RCC_MCOConfig(uint32_t RCC_MCOx, uint32_t RCC_MCOSource, uint32_t RCC_MCODiv);
void     HAL_RCC_EnableCSS(void);
void     HAL_RCC_DisableCSS(void);
uint32_t HAL_RCC_GetSysClockFreq(void);
uint32_t HAL_RCC_GetHCLKFreq(void);
uint32_t HAL_RCC_GetPCLK1Freq(void);
uint32_t HAL_RCC_GetPCLK2Freq(void);
void     HAL_RCC_GetOscConfig(RCC_OscInitTypeDef *RCC_OscInitStruct);
void     HAL_RCC_GetClockConfig(RCC_ClkInitTypeDef *RCC_ClkInitStruct, uint32_t *pFLatency);

 
void HAL_RCC_NMI_IRQHandler(void);

  
void HAL_RCC_CSSCallback(void);


 



 

 
 
 


 









 
 


 






 


 

 


 
    


   






















































#line 1278 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rcc.h"








 



 



  



 







 
#line 247 "..\\USER\\stm32f7xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_gpio.h"



































  

 







 
#line 48 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_gpio.h"



 



  

 


 



  
typedef struct
{
  uint32_t Pin;       
 

  uint32_t Mode;      
 

  uint32_t Pull;      
 

  uint32_t Speed;     
 

  uint32_t Alternate;  
 
}GPIO_InitTypeDef;



 
typedef enum
{
  GPIO_PIN_RESET = 0,
  GPIO_PIN_SET
}GPIO_PinState;


 

 



  



 
#line 121 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_gpio.h"




 










  







    



 





 




   






 

 


   





 
  


 

 


 






 







 







 







 







 



 

 
#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_gpio_ex.h"



































  

 







 
#line 48 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_gpio_ex.h"



 



  

 

 


 
  


   



  







  
#line 85 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_gpio_ex.h"



  






  
#line 105 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_gpio_ex.h"


  
#line 116 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_gpio_ex.h"



  
#line 126 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_gpio_ex.h"



  
#line 136 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_gpio_ex.h"



  
#line 151 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_gpio_ex.h"



  
#line 165 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_gpio_ex.h"




  
#line 182 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_gpio_ex.h"


  
#line 193 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_gpio_ex.h"



  
#line 203 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_gpio_ex.h"
   


  
#line 214 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_gpio_ex.h"
   


  
#line 224 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_gpio_ex.h"
   


 




  





  



 

 


 


 

  


 


 
 
 
 


 



 
 
#line 282 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_gpio_ex.h"



 

 


 


 
#line 304 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_gpio_ex.h"


 

#line 320 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_gpio_ex.h"


 
#line 480 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_gpio_ex.h"


  



 

 


 



 



  



  
  






 
#line 233 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_gpio.h"

 


 



 
 
void  HAL_GPIO_Init(GPIO_TypeDef  *GPIOx, GPIO_InitTypeDef *GPIO_Init);
void  HAL_GPIO_DeInit(GPIO_TypeDef  *GPIOx, uint32_t GPIO_Pin);


 



 
 
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
void HAL_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
HAL_StatusTypeDef HAL_GPIO_LockPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void HAL_GPIO_EXTI_IRQHandler(uint16_t GPIO_Pin);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);



  



  
 
 
 


 



 

 


 
#line 300 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_gpio.h"


 

 


 



 



  



 







 
#line 251 "..\\USER\\stm32f7xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_dma.h"



































  

 







 
#line 48 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_dma.h"



 



  

 




 
   


 
typedef struct
{
  uint32_t Channel;              
 

  uint32_t Direction;            

 

  uint32_t PeriphInc;            
 

  uint32_t MemInc;               
 

  uint32_t PeriphDataAlignment;  
 

  uint32_t MemDataAlignment;     
 

  uint32_t Mode;                 


 

  uint32_t Priority;             
 

  uint32_t FIFOMode;             


 

  uint32_t FIFOThreshold;        
 

  uint32_t MemBurst;             



 

  uint32_t PeriphBurst;          



 
}DMA_InitTypeDef;



 
typedef enum
{
  HAL_DMA_STATE_RESET             = 0x00U,   
  HAL_DMA_STATE_READY             = 0x01U,   
  HAL_DMA_STATE_BUSY              = 0x02U,   
  HAL_DMA_STATE_TIMEOUT           = 0x03U,   
  HAL_DMA_STATE_ERROR             = 0x04U,   
  HAL_DMA_STATE_ABORT             = 0x05U,   
}HAL_DMA_StateTypeDef;



 
typedef enum
{
  HAL_DMA_FULL_TRANSFER      = 0x00U,     
  HAL_DMA_HALF_TRANSFER      = 0x01U,     
}HAL_DMA_LevelCompleteTypeDef;



 
typedef enum
{
  HAL_DMA_XFER_CPLT_CB_ID          = 0x00U,     
  HAL_DMA_XFER_HALFCPLT_CB_ID      = 0x01U,     
  HAL_DMA_XFER_M1CPLT_CB_ID        = 0x02U,     
  HAL_DMA_XFER_M1HALFCPLT_CB_ID    = 0x03U,     
  HAL_DMA_XFER_ERROR_CB_ID         = 0x04U,     
  HAL_DMA_XFER_ABORT_CB_ID         = 0x05U,     
  HAL_DMA_XFER_ALL_CB_ID           = 0x06U      
}HAL_DMA_CallbackIDTypeDef;



 
typedef struct __DMA_HandleTypeDef
{
  DMA_Stream_TypeDef         *Instance;                                                     

  DMA_InitTypeDef            Init;                                                           

  HAL_LockTypeDef            Lock;                                                            

  volatile HAL_DMA_StateTypeDef  State;                                                         

  void                       *Parent;                                                        

  void                       (* XferCpltCallback)( struct __DMA_HandleTypeDef * hdma);      

  void                       (* XferHalfCpltCallback)( struct __DMA_HandleTypeDef * hdma);  

  void                       (* XferM1CpltCallback)( struct __DMA_HandleTypeDef * hdma);    
  
  void                       (* XferM1HalfCpltCallback)( struct __DMA_HandleTypeDef * hdma);    
  
  void                       (* XferErrorCallback)( struct __DMA_HandleTypeDef * hdma);     
  
  void                       (* XferAbortCallback)( struct __DMA_HandleTypeDef * hdma);       

 volatile uint32_t               ErrorCode;                                                     
  
 uint32_t                    StreamBaseAddress;                                             

 uint32_t                    StreamIndex;                                                   
 
}DMA_HandleTypeDef;



 


 




 




  
#line 212 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_dma.h"


 




  





 
        



  




  




  




 




  





  




 





 




  





 




 






  




 




  




 






  




  






  




  






 




 







 




  
#line 375 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_dma.h"


 



 
 
 




 













 






 






 


 





 
#line 440 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_dma.h"





       
#line 460 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_dma.h"





 
#line 480 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_dma.h"





 
#line 500 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_dma.h"





 
#line 520 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_dma.h"













 

















 
















 














 














 




















 







 



 
#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_dma_ex.h"



































 

 







 
#line 48 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_dma_ex.h"



 



  

 



 
   


  
typedef enum
{
  MEMORY0      = 0x00U,     
  MEMORY1      = 0x01U,     

}HAL_DMA_MemoryTypeDef;



 
  
 




 




  
#line 106 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_dma_ex.h"



 
  


   

 



 




 

 
HAL_StatusTypeDef HAL_DMAEx_MultiBufferStart(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t SecondMemAddress, uint32_t DataLength);
HAL_StatusTypeDef HAL_DMAEx_MultiBufferStart_IT(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t SecondMemAddress, uint32_t DataLength);
HAL_StatusTypeDef HAL_DMAEx_ChangeMemory(DMA_HandleTypeDef *hdma, uint32_t Address, HAL_DMA_MemoryTypeDef memory);



 


 
  
 



 
#line 170 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_dma_ex.h"


   
         
 



 


 



 



 







 
#line 633 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_dma.h"

 




 




 
HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *hdma); 
HAL_StatusTypeDef HAL_DMA_DeInit(DMA_HandleTypeDef *hdma);


 




 
HAL_StatusTypeDef HAL_DMA_Start (DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);
HAL_StatusTypeDef HAL_DMA_Start_IT(DMA_HandleTypeDef *hdma, uint32_t SrcAddress, uint32_t DstAddress, uint32_t DataLength);
HAL_StatusTypeDef HAL_DMA_Abort(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_Abort_IT(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_PollForTransfer(DMA_HandleTypeDef *hdma, HAL_DMA_LevelCompleteTypeDef CompleteLevel, uint32_t Timeout);
void              HAL_DMA_IRQHandler(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_CleanCallbacks(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_RegisterCallback(DMA_HandleTypeDef *hdma, HAL_DMA_CallbackIDTypeDef CallbackID, void (* pCallback)(DMA_HandleTypeDef *_hdma));
HAL_StatusTypeDef HAL_DMA_UnRegisterCallback(DMA_HandleTypeDef *hdma, HAL_DMA_CallbackIDTypeDef CallbackID);



  




 
HAL_DMA_StateTypeDef HAL_DMA_GetState(DMA_HandleTypeDef *hdma);
uint32_t             HAL_DMA_GetError(DMA_HandleTypeDef *hdma);


  


  
 



 


  

 



 
















































  

 



 


 



  



 







 
#line 255 "..\\USER\\stm32f7xx_hal_conf.h"

   
#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_cortex.h"



































  

 







 
#line 48 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_cortex.h"



 



  
 


 





 
typedef struct
{
  uint8_t                Enable;                
 
  uint8_t                Number;                
 
  uint32_t               BaseAddress;            
  uint8_t                Size;                  
 
  uint8_t                SubRegionDisable;      
          
  uint8_t                TypeExtField;          
                  
  uint8_t                AccessPermission;      
 
  uint8_t                DisableExec;           
 
  uint8_t                IsShareable;           
 
  uint8_t                IsCacheable;           
 
  uint8_t                IsBufferable;          
 
}MPU_Region_InitTypeDef;


 




 

 



 



 
#line 118 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_cortex.h"


 



 





 




 






 



 




 



 




 



 




 



 




 



 




 



 





 



 
#line 230 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_cortex.h"


 
   


 
#line 243 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_cortex.h"


 



 
#line 258 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_cortex.h"


 




 


 

 


 
  


 
 
void HAL_NVIC_SetPriorityGrouping(uint32_t PriorityGroup);
void HAL_NVIC_SetPriority(IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority);
void HAL_NVIC_EnableIRQ(IRQn_Type IRQn);
void HAL_NVIC_DisableIRQ(IRQn_Type IRQn);
void HAL_NVIC_SystemReset(void);
uint32_t HAL_SYSTICK_Config(uint32_t TicksNumb);


 



 
 

void HAL_MPU_ConfigRegion(MPU_Region_InitTypeDef *MPU_Init);

uint32_t HAL_NVIC_GetPriorityGrouping(void);
void HAL_NVIC_GetPriority(IRQn_Type IRQn, uint32_t PriorityGroup, uint32_t* pPreemptPriority, uint32_t* pSubPriority);
uint32_t HAL_NVIC_GetPendingIRQ(IRQn_Type IRQn);
void HAL_NVIC_SetPendingIRQ(IRQn_Type IRQn);
void HAL_NVIC_ClearPendingIRQ(IRQn_Type IRQn);
uint32_t HAL_NVIC_GetActive(IRQn_Type IRQn);
void HAL_SYSTICK_CLKSourceConfig(uint32_t CLKSource);
void HAL_SYSTICK_IRQHandler(void);
void HAL_SYSTICK_Callback(void);


 



 

  
 
 
 


 



































#line 361 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_cortex.h"

#line 370 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_cortex.h"

#line 399 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_cortex.h"






                                                                             
                                                                                   
    



 





 
static __inline void HAL_MPU_Disable(void)
{
   
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHCSR &= ~(1UL << 16U);
  
   
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->CTRL  &= ~(1UL );
}











 
static __inline void HAL_MPU_Enable(uint32_t MPU_Control)
{
   
  ((MPU_Type *) ((0xE000E000UL) + 0x0D90UL) )->CTRL   = MPU_Control | (1UL );
  
   
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHCSR |= (1UL << 16U);
}




 



  



 
  





 

 
#line 259 "..\\USER\\stm32f7xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_adc.h"



































 

 







 
#line 48 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_adc.h"



 



  

 


 













 
typedef struct
{
  uint32_t ClockPrescaler;        

 
  uint32_t Resolution;            
 
  uint32_t DataAlign;             

 
  uint32_t ScanConvMode;          





 
  uint32_t EOCSelection;          





 
  uint32_t ContinuousConvMode;    

 
  uint32_t NbrOfConversion;       

 
  uint32_t DiscontinuousConvMode; 


 
  uint32_t NbrOfDiscConversion;   

 
  uint32_t ExternalTrigConv;      


 
  uint32_t ExternalTrigConvEdge;  

 
  uint32_t DMAContinuousRequests; 



 
}ADC_InitTypeDef;







  
typedef struct 
{
  uint32_t Channel;                
 
  uint32_t Rank;                   
 
  uint32_t SamplingTime;           







 
  uint32_t Offset;                  
}ADC_ChannelConfTypeDef;



  
typedef struct
{
  uint32_t WatchdogMode;      
 
  uint32_t HighThreshold;     
      
  uint32_t LowThreshold;      
 
  uint32_t Channel;           

       
  uint32_t ITMode;            

 
  uint32_t WatchdogNumber;     
}ADC_AnalogWDGConfTypeDef;



  
 





 




 





 




 




 





  
typedef struct
{
  ADC_TypeDef                   *Instance;                    

  ADC_InitTypeDef               Init;                         

  volatile uint32_t                 NbrOfCurrentConversionRank;   

  DMA_HandleTypeDef             *DMA_Handle;                  

  HAL_LockTypeDef               Lock;                         

  volatile uint32_t                 State;                        

  volatile uint32_t                 ErrorCode;                    
}ADC_HandleTypeDef;


 

 


 



 







 




  






  



  
#line 276 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_adc.h"


  



  






  



  






  



 
 
 


#line 323 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_adc.h"






  



  




  



  
#line 362 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_adc.h"





  



  
#line 380 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_adc.h"


  

  

  





  



  




 



  
#line 413 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_adc.h"


  
    


  






  
    


  
#line 437 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_adc.h"


  



  





 



 

 


 
	



 






 






 







 







 






 







 







 




 

 
#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_adc_ex.h"



































 

 







 
#line 48 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_adc_ex.h"



 



  

 


 
   











 
typedef struct 
{
  uint32_t InjectedChannel;               

 
  uint32_t InjectedRank;                  

 
  uint32_t InjectedSamplingTime;          







 
  uint32_t InjectedOffset;                


 
  uint32_t InjectedNbrOfConversion;       



 
  uint32_t InjectedDiscontinuousConvMode; 





 
  uint32_t AutoInjectedConv;              






 
  uint32_t ExternalTrigInjecConv;         






 
  uint32_t ExternalTrigInjecConvEdge;     



 
}ADC_InjectionConfTypeDef; 


 



  
typedef struct
{
  uint32_t Mode;              
 
  uint32_t DMAAccessMode;     
 
  uint32_t TwoSamplingDelay;  
 
}ADC_MultiModeTypeDef;



  

 


 



 
#line 171 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_adc_ex.h"


  



  






  



 






  



 
#line 206 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_adc_ex.h"

#line 216 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_adc_ex.h"


  



  






  



 



 
  


  

 


 


 

 


 
	


 

 
HAL_StatusTypeDef HAL_ADCEx_InjectedStart(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef HAL_ADCEx_InjectedStop(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef HAL_ADCEx_InjectedPollForConversion(ADC_HandleTypeDef* hadc, uint32_t Timeout);
HAL_StatusTypeDef HAL_ADCEx_InjectedStart_IT(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef HAL_ADCEx_InjectedStop_IT(ADC_HandleTypeDef* hadc);
uint32_t HAL_ADCEx_InjectedGetValue(ADC_HandleTypeDef* hadc, uint32_t InjectedRank);
HAL_StatusTypeDef HAL_ADCEx_MultiModeStart_DMA(ADC_HandleTypeDef* hadc, uint32_t* pData, uint32_t Length);
HAL_StatusTypeDef HAL_ADCEx_MultiModeStop_DMA(ADC_HandleTypeDef* hadc);
uint32_t HAL_ADCEx_MultiModeGetValue(ADC_HandleTypeDef* hadc);
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc);

 
HAL_StatusTypeDef HAL_ADCEx_InjectedConfigChannel(ADC_HandleTypeDef* hadc,ADC_InjectionConfTypeDef* sConfigInjected);
HAL_StatusTypeDef HAL_ADCEx_MultiModeConfigChannel(ADC_HandleTypeDef* hadc, ADC_MultiModeTypeDef* multimode);



  



 
 
 
 


 



 
	
 


 


                                     
#line 339 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_adc_ex.h"







 



 
	
 


 



 



 
	


 








 
#line 525 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_adc.h"

 


 



 
 
HAL_StatusTypeDef HAL_ADC_Init(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef HAL_ADC_DeInit(ADC_HandleTypeDef *hadc);
void       HAL_ADC_MspInit(ADC_HandleTypeDef* hadc);
void       HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc);


 



 
 
HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef* hadc, uint32_t Timeout);

HAL_StatusTypeDef HAL_ADC_PollForEvent(ADC_HandleTypeDef* hadc, uint32_t EventType, uint32_t Timeout);

HAL_StatusTypeDef HAL_ADC_Start_IT(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef HAL_ADC_Stop_IT(ADC_HandleTypeDef* hadc);

void              HAL_ADC_IRQHandler(ADC_HandleTypeDef* hadc);

HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef* hadc, uint32_t* pData, uint32_t Length);
HAL_StatusTypeDef HAL_ADC_Stop_DMA(ADC_HandleTypeDef* hadc);

uint32_t          HAL_ADC_GetValue(ADC_HandleTypeDef* hadc);

void       HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
void       HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc);
void       HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef* hadc);
void       HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc);


 



 
 
HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef* hadc, ADC_ChannelConfTypeDef* sConfig);
HAL_StatusTypeDef HAL_ADC_AnalogWDGConfig(ADC_HandleTypeDef* hadc, ADC_AnalogWDGConfTypeDef* AnalogWDGConfig);


 



 
 
uint32_t HAL_ADC_GetState(ADC_HandleTypeDef* hadc);
uint32_t HAL_ADC_GetError(ADC_HandleTypeDef *hadc);


 



  

 
 
 


 
 
 
 

 
 
 



 

 


 

 





 









 








 









 






 
#line 709 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_adc.h"
                                      									
#line 742 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_adc.h"





 







 







 







 







 







 






 






 






 






 






 






 

																


 
	
 


 



 
	


 
	


 








 
#line 263 "..\\USER\\stm32f7xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_can.h"



































 

 







 
#line 48 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_can.h"



 



 

 


 



 
typedef enum
{
  HAL_CAN_STATE_RESET             = 0x00U,   
  HAL_CAN_STATE_READY             = 0x01U,   
  HAL_CAN_STATE_BUSY              = 0x02U,   
  HAL_CAN_STATE_BUSY_TX           = 0x12U,   
  HAL_CAN_STATE_BUSY_RX           = 0x22U,   
  HAL_CAN_STATE_BUSY_TX_RX        = 0x32U,   
  HAL_CAN_STATE_TIMEOUT           = 0x03U,   
  HAL_CAN_STATE_ERROR             = 0x04U    

}HAL_CAN_StateTypeDef;



 
typedef struct
{
  uint32_t Prescaler;  
 

  uint32_t Mode;       
 

  uint32_t SJW;        


 

  uint32_t BS1;        
 

  uint32_t BS2;        
 

  uint32_t TTCM;       
 

  uint32_t ABOM;       
 

  uint32_t AWUM;       
 

  uint32_t NART;       
 

  uint32_t RFLM;       
 

  uint32_t TXFP;       
 
}CAN_InitTypeDef;



 
typedef struct
{
  uint32_t FilterIdHigh;          

 

  uint32_t FilterIdLow;           

 

  uint32_t FilterMaskIdHigh;      


 

  uint32_t FilterMaskIdLow;       


 

  uint32_t FilterFIFOAssignment;  
 

  uint32_t FilterNumber;          
 

  uint32_t FilterMode;            
 

  uint32_t FilterScale;           
 

  uint32_t FilterActivation;      
 

  uint32_t BankNumber;            
 

}CAN_FilterConfTypeDef;



 
typedef struct
{
  uint32_t StdId;    
 

  uint32_t ExtId;    
 

  uint32_t IDE;      
 

  uint32_t RTR;      
 

  uint32_t DLC;      
 

  uint8_t Data[8];  
 

}CanTxMsgTypeDef;



 
typedef struct
{
  uint32_t StdId;       
 

  uint32_t ExtId;       
 

  uint32_t IDE;         
 

  uint32_t RTR;         
 

  uint32_t DLC;         
 

  uint8_t Data[8];      
 

  uint32_t FMI;         
 

  uint32_t FIFONumber;  
 

}CanRxMsgTypeDef;



 
typedef struct
{
  CAN_TypeDef                 *Instance;   

  CAN_InitTypeDef             Init;        

  CanTxMsgTypeDef*            pTxMsg;      

  CanRxMsgTypeDef*            pRxMsg;      

  volatile HAL_CAN_StateTypeDef   State;       

  HAL_LockTypeDef             Lock;        

  volatile uint32_t               ErrorCode;   

}CAN_HandleTypeDef;



 

 


 



 
#line 261 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_can.h"


 



 




 



 






 



 






 



 
#line 315 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_can.h"


 



 
#line 330 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_can.h"


 



 




 



 




 



 




 



 




 



 




 



 




 



 

 

 

 
#line 406 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_can.h"

 






 







 

 





 



 


 
#line 444 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_can.h"

 



 







 



 





 



 

 


 




 







 







 







 





























 
#line 542 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_can.h"
























 














 







 










 








 













 





 

 


 



 
 
HAL_StatusTypeDef HAL_CAN_Init(CAN_HandleTypeDef* hcan);
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef* hcan, CAN_FilterConfTypeDef* sFilterConfig);
HAL_StatusTypeDef HAL_CAN_DeInit(CAN_HandleTypeDef* hcan);
void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan);
void HAL_CAN_MspDeInit(CAN_HandleTypeDef* hcan);


 



 
 
HAL_StatusTypeDef HAL_CAN_Transmit(CAN_HandleTypeDef *hcan, uint32_t Timeout);
HAL_StatusTypeDef HAL_CAN_Transmit_IT(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef HAL_CAN_Receive(CAN_HandleTypeDef *hcan, uint8_t FIFONumber, uint32_t Timeout);
HAL_StatusTypeDef HAL_CAN_Receive_IT(CAN_HandleTypeDef *hcan, uint8_t FIFONumber);
HAL_StatusTypeDef HAL_CAN_Sleep(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef HAL_CAN_WakeUp(CAN_HandleTypeDef *hcan);
void HAL_CAN_IRQHandler(CAN_HandleTypeDef* hcan);
void HAL_CAN_TxCpltCallback(CAN_HandleTypeDef* hcan);
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan);
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan);


 



 
 
uint32_t HAL_CAN_GetError(CAN_HandleTypeDef *hcan);
HAL_CAN_StateTypeDef HAL_CAN_GetState(CAN_HandleTypeDef* hcan);


 



 

 


 



 

 


 



  

 


 




 

 


 
#line 730 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_can.h"













 

 


 



 


 



 








 
#line 267 "..\\USER\\stm32f7xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_cec.h"



































 

 







 
#line 48 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_cec.h"



 



 
  
  


 
  


  
typedef struct
{
  uint32_t SignalFreeTime;               



 

  uint32_t Tolerance;                    

 

  uint32_t BRERxStop;                    

 

  uint32_t BREErrorBitGen;               


 
                                              
  uint32_t LBPEErrorBitGen;              


   
                                              
  uint32_t BroadcastMsgNoErrorBitGen;    













 
 
  uint32_t SignalFreeTimeOption;         

 
  
  uint32_t ListenMode;                   







 

  uint16_t  OwnAddress;                 
 
  
  uint8_t  *RxBuffer;                     
  

}CEC_InitTypeDef;







































  
typedef enum
{
  HAL_CEC_STATE_RESET             = 0x00U,    
 
  HAL_CEC_STATE_READY             = 0x20U,    
 
  HAL_CEC_STATE_BUSY              = 0x24U,    
 
  HAL_CEC_STATE_BUSY_RX           = 0x22U,    
 
  HAL_CEC_STATE_BUSY_TX           = 0x21U,    
                                                   
  HAL_CEC_STATE_BUSY_RX_TX        = 0x23U,    
 
  HAL_CEC_STATE_ERROR             = 0x60U      
}HAL_CEC_StateTypeDef;



   
typedef struct
{
  CEC_TypeDef             *Instance;       
  
  CEC_InitTypeDef         Init;            
  
  uint8_t                 *pTxBuffPtr;     
  
  uint16_t                TxXferCount;     
  
  uint16_t                RxXferSize;      
  
  HAL_LockTypeDef         Lock;            

  HAL_CEC_StateTypeDef    gState;         

 
  
  HAL_CEC_StateTypeDef    RxState;        
 
  
  uint32_t                ErrorCode;      
     
}CEC_HandleTypeDef;


 

 


 



  
#line 234 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_cec.h"


 
       


 
#line 249 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_cec.h"


 



 




  



 




             
             


  




  
                        


  




     



  




 
  


  




 
  


  




 
  


 



 
  


 



 



 
#line 351 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_cec.h"


 
    


 
#line 371 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_cec.h"


 



 
#line 391 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_cec.h"


 
  


 




 



 



 
  


 



 
  


   
  
 


 




 






















 




















 




















 




















    




















 





 





 





 






 





 





 





 







 




                        

 


 



 
 
HAL_StatusTypeDef HAL_CEC_Init(CEC_HandleTypeDef *hcec);
HAL_StatusTypeDef HAL_CEC_DeInit(CEC_HandleTypeDef *hcec);
HAL_StatusTypeDef HAL_CEC_SetDeviceAddress(CEC_HandleTypeDef *hcec, uint16_t CEC_OwnAddress);
void HAL_CEC_MspInit(CEC_HandleTypeDef *hcec);
void HAL_CEC_MspDeInit(CEC_HandleTypeDef *hcec);


 



 
 
HAL_StatusTypeDef HAL_CEC_Transmit_IT(CEC_HandleTypeDef *hcec, uint8_t InitiatorAddress,uint8_t DestinationAddress, uint8_t *pData, uint32_t Size);
uint32_t HAL_CEC_GetLastReceivedFrameSize(CEC_HandleTypeDef *hcec);
void HAL_CEC_ChangeRxBuffer(CEC_HandleTypeDef *hcec, uint8_t* Rxbuffer);
void HAL_CEC_IRQHandler(CEC_HandleTypeDef *hcec);
void HAL_CEC_TxCpltCallback(CEC_HandleTypeDef *hcec);
void HAL_CEC_RxCpltCallback(CEC_HandleTypeDef *hcec, uint32_t RxFrameSize);
void HAL_CEC_ErrorCallback(CEC_HandleTypeDef *hcec);


 



 
 
HAL_CEC_StateTypeDef HAL_CEC_GetState(CEC_HandleTypeDef *hcec);
uint32_t HAL_CEC_GetError(CEC_HandleTypeDef *hcec);


 



 
  
 


 



  

 


 
  


  

 


 



  

 


 
  




                                            


                                           





                                                 


                                                                       


                                          









 

                                                 




 






 



 
 


 
  


 
  


  



  
  






 
#line 271 "..\\USER\\stm32f7xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_crc.h"



































 

 







 
#line 48 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_crc.h"



 




 

 


 



 
typedef enum
{
  HAL_CRC_STATE_RESET     = 0x00U,   
  HAL_CRC_STATE_READY     = 0x01U,   
  HAL_CRC_STATE_BUSY      = 0x02U,   
  HAL_CRC_STATE_TIMEOUT   = 0x03U,   
  HAL_CRC_STATE_ERROR     = 0x04U    
}HAL_CRC_StateTypeDef;


 



 
typedef struct
{
  uint8_t DefaultPolynomialUse;       



 

  uint8_t DefaultInitValueUse;        


 

  uint32_t GeneratingPolynomial;      


                                                 

  uint32_t CRCLength;                 




 
                                              
  uint32_t InitValue;                 
                                                 
  
  uint32_t InputDataInversionMode;    




   
                                              
  uint32_t OutputDataInversionMode;   


 
}CRC_InitTypeDef;


 
  


 
typedef struct
{
  CRC_TypeDef                 *Instance;     
  
  CRC_InitTypeDef             Init;         
  
  HAL_LockTypeDef             Lock;         
    
  volatile HAL_CRC_StateTypeDef   State;        
  
  uint32_t InputDataFormat;                





  
}CRC_HandleTypeDef;


 



  
  
 


 
  


 




 



 




 



 






 
 


                                       





 



 






 



 







   



 



    






    



  
 



 




 






 







 







 






 



 


 
#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_crc_ex.h"



































  

 







 
#line 48 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_crc_ex.h"



 



  

  
 



 



 











 



 







 




 
 



 
    




 






 







 




 




 



 
 
 
HAL_StatusTypeDef HAL_CRCEx_Polynomial_Set(CRC_HandleTypeDef *hcrc, uint32_t Pol, uint32_t PolyLength);
HAL_StatusTypeDef HAL_CRCEx_Input_Data_Reverse(CRC_HandleTypeDef *hcrc, uint32_t InputReverseMode);
HAL_StatusTypeDef HAL_CRCEx_Output_Data_Reverse(CRC_HandleTypeDef *hcrc, uint32_t OutputReverseMode);

 
 



 



 




  



 
  






 
#line 287 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_crc.h"

 


 



 
 
HAL_StatusTypeDef HAL_CRC_Init(CRC_HandleTypeDef *hcrc);
HAL_StatusTypeDef HAL_CRC_DeInit (CRC_HandleTypeDef *hcrc);
void HAL_CRC_MspInit(CRC_HandleTypeDef *hcrc);
void HAL_CRC_MspDeInit(CRC_HandleTypeDef *hcrc);


 

 





 
 
uint32_t HAL_CRC_Accumulate(CRC_HandleTypeDef *hcrc, uint32_t pBuffer[], uint32_t BufferLength);
uint32_t HAL_CRC_Calculate(CRC_HandleTypeDef *hcrc, uint32_t pBuffer[], uint32_t BufferLength);


 



 
 
HAL_CRC_StateTypeDef HAL_CRC_GetState(CRC_HandleTypeDef *hcrc);


 



 


 


 



  

 


 



  

 


 



  

 


 



  

 


 
#line 384 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_crc.h"




 

 


 



 

 


 



 



  



 
  






 

#line 275 "..\\USER\\stm32f7xx_hal_conf.h"






#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_dma2d.h"



































  

 







 
#line 48 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_dma2d.h"



 




 

 


 




 
typedef struct
{
  uint32_t Blue;               
 

  uint32_t Green;              
 

  uint32_t Red;                
 
} DMA2D_ColorTypeDef;



 
typedef struct
{
  uint32_t *pCLUT;                   

  uint32_t CLUTColorMode;           
 

  uint32_t Size;                    
 
} DMA2D_CLUTCfgTypeDef;



 
typedef struct
{
  uint32_t             Mode;               
 

  uint32_t             ColorMode;          
 

  uint32_t             OutputOffset;       
 

  uint32_t             AlphaInverted;     
 



  uint32_t             RedBlueSwap;       

  

  
} DMA2D_InitTypeDef;




 
typedef struct
{
  uint32_t             InputOffset;       
 

  uint32_t             InputColorMode;    
 

  uint32_t             AlphaMode;         
 

  uint32_t             InputAlpha;        






 


  uint32_t             AlphaInverted;     


 
  



  uint32_t             RedBlueSwap;       


   


  
} DMA2D_LayerCfgTypeDef;



 
typedef enum
{
  HAL_DMA2D_STATE_RESET             = 0x00U,     
  HAL_DMA2D_STATE_READY             = 0x01U,     
  HAL_DMA2D_STATE_BUSY              = 0x02U,     
  HAL_DMA2D_STATE_TIMEOUT           = 0x03U,     
  HAL_DMA2D_STATE_ERROR             = 0x04U,     
  HAL_DMA2D_STATE_SUSPEND           = 0x05U      
}HAL_DMA2D_StateTypeDef;



 
typedef struct __DMA2D_HandleTypeDef
{
  DMA2D_TypeDef               *Instance;                                                     
                                                                                                                                          
  DMA2D_InitTypeDef           Init;                                                          

  void                        (* XferCpltCallback)(struct __DMA2D_HandleTypeDef * hdma2d);   
                                                                                                                                           
  void                        (* XferErrorCallback)(struct __DMA2D_HandleTypeDef * hdma2d);                                                                                                                                               

  DMA2D_LayerCfgTypeDef       LayerCfg[2];                                       

  HAL_LockTypeDef             Lock;                                                            
                                                                                                                                           
  volatile HAL_DMA2D_StateTypeDef State;                                                         
                                                                                                                                           
  volatile uint32_t               ErrorCode;                                                       
} DMA2D_HandleTypeDef;


 

 


 



 







 



 






 



 







 



 
#line 253 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_dma2d.h"


 



 






     




 




 





 




  




 




 




 
#line 309 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_dma2d.h"


                                                         
                                                            


                                                         
#line 322 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_dma2d.h"


 
  


 



 
  
  


 
 


 




 






 



 












 














 














 














 














 

     


 

   


 



   
  
 
HAL_StatusTypeDef HAL_DMA2D_Init(DMA2D_HandleTypeDef *hdma2d); 
HAL_StatusTypeDef HAL_DMA2D_DeInit (DMA2D_HandleTypeDef *hdma2d);
void              HAL_DMA2D_MspInit(DMA2D_HandleTypeDef* hdma2d);
void              HAL_DMA2D_MspDeInit(DMA2D_HandleTypeDef* hdma2d);



 




 
  
 
HAL_StatusTypeDef HAL_DMA2D_Start(DMA2D_HandleTypeDef *hdma2d, uint32_t pdata, uint32_t DstAddress, uint32_t Width, uint32_t Height);
HAL_StatusTypeDef HAL_DMA2D_BlendingStart(DMA2D_HandleTypeDef *hdma2d, uint32_t SrcAddress1, uint32_t SrcAddress2, uint32_t DstAddress, uint32_t Width,  uint32_t Height);
HAL_StatusTypeDef HAL_DMA2D_Start_IT(DMA2D_HandleTypeDef *hdma2d, uint32_t pdata, uint32_t DstAddress, uint32_t Width, uint32_t Height);
HAL_StatusTypeDef HAL_DMA2D_BlendingStart_IT(DMA2D_HandleTypeDef *hdma2d, uint32_t SrcAddress1, uint32_t SrcAddress2, uint32_t DstAddress, uint32_t Width, uint32_t Height);
HAL_StatusTypeDef HAL_DMA2D_Suspend(DMA2D_HandleTypeDef *hdma2d);
HAL_StatusTypeDef HAL_DMA2D_Resume(DMA2D_HandleTypeDef *hdma2d);
HAL_StatusTypeDef HAL_DMA2D_Abort(DMA2D_HandleTypeDef *hdma2d);
HAL_StatusTypeDef HAL_DMA2D_EnableCLUT(DMA2D_HandleTypeDef *hdma2d, uint32_t LayerIdx);
HAL_StatusTypeDef HAL_DMA2D_CLUTLoad(DMA2D_HandleTypeDef *hdma2d, DMA2D_CLUTCfgTypeDef CLUTCfg, uint32_t LayerIdx);
HAL_StatusTypeDef HAL_DMA2D_CLUTLoad_IT(DMA2D_HandleTypeDef *hdma2d, DMA2D_CLUTCfgTypeDef CLUTCfg, uint32_t LayerIdx);
HAL_StatusTypeDef HAL_DMA2D_CLUTLoading_Abort(DMA2D_HandleTypeDef *hdma2d, uint32_t LayerIdx);
HAL_StatusTypeDef HAL_DMA2D_CLUTLoading_Suspend(DMA2D_HandleTypeDef *hdma2d, uint32_t LayerIdx);
HAL_StatusTypeDef HAL_DMA2D_CLUTLoading_Resume(DMA2D_HandleTypeDef *hdma2d, uint32_t LayerIdx);
HAL_StatusTypeDef HAL_DMA2D_PollForTransfer(DMA2D_HandleTypeDef *hdma2d, uint32_t Timeout);
void              HAL_DMA2D_IRQHandler(DMA2D_HandleTypeDef *hdma2d);
void              HAL_DMA2D_LineEventCallback(DMA2D_HandleTypeDef *hdma2d);
void              HAL_DMA2D_CLUTLoadingCpltCallback(DMA2D_HandleTypeDef *hdma2d);



 



 

 
HAL_StatusTypeDef HAL_DMA2D_ConfigLayer(DMA2D_HandleTypeDef *hdma2d, uint32_t LayerIdx);
HAL_StatusTypeDef HAL_DMA2D_ConfigCLUT(DMA2D_HandleTypeDef *hdma2d, DMA2D_CLUTCfgTypeDef CLUTCfg, uint32_t LayerIdx);
HAL_StatusTypeDef HAL_DMA2D_ProgramLineEvent(DMA2D_HandleTypeDef *hdma2d, uint32_t Line);
HAL_StatusTypeDef HAL_DMA2D_EnableDeadTime(DMA2D_HandleTypeDef *hdma2d);
HAL_StatusTypeDef HAL_DMA2D_DisableDeadTime(DMA2D_HandleTypeDef *hdma2d);
HAL_StatusTypeDef HAL_DMA2D_ConfigDeadTime(DMA2D_HandleTypeDef *hdma2d, uint8_t DeadTime);



 



 

 
HAL_DMA2D_StateTypeDef HAL_DMA2D_GetState(DMA2D_HandleTypeDef *hdma2d);
uint32_t               HAL_DMA2D_GetError(DMA2D_HandleTypeDef *hdma2d);



 



 

   
  


                          



 



 
  


 



       



   



  
  


 



  
  


 




     
  


 



    
    


  


 


 
#line 599 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_dma2d.h"







#line 615 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_dma2d.h"


 



  



 







 

 
#line 283 "..\\USER\\stm32f7xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_dac.h"



































 

 







 
#line 48 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_dac.h"



 



 

 


 



 
typedef enum
{
  HAL_DAC_STATE_RESET             = 0x00U,   
  HAL_DAC_STATE_READY             = 0x01U,   
  HAL_DAC_STATE_BUSY              = 0x02U,   
  HAL_DAC_STATE_TIMEOUT           = 0x03U,   
  HAL_DAC_STATE_ERROR             = 0x04U    
}HAL_DAC_StateTypeDef;
 


 
typedef struct
{
  DAC_TypeDef                 *Instance;      

  volatile HAL_DAC_StateTypeDef   State;          

  HAL_LockTypeDef             Lock;           

  DMA_HandleTypeDef           *DMA_Handle1;   

  DMA_HandleTypeDef           *DMA_Handle2;   

  volatile uint32_t               ErrorCode;      

}DAC_HandleTypeDef;



 
typedef struct
{
  uint32_t DAC_Trigger;       
 

  uint32_t DAC_OutputBuffer;  
 
}DAC_ChannelConfTypeDef;


 

 


 



 






 



 

#line 136 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_dac.h"





 



 




 



 




 



 





 



  




 



  




 



 

 


 




 






 







 








 






 









 









 









 



 

 
#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_dac_ex.h"



































 

 







 
#line 48 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_dac_ex.h"



 



 

 
 


 
   


 
#line 90 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_dac_ex.h"


 



 
 
 


 



 
 
uint32_t HAL_DACEx_DualGetValue(DAC_HandleTypeDef* hdac);
HAL_StatusTypeDef HAL_DACEx_TriangleWaveGenerate(DAC_HandleTypeDef* hdac, uint32_t Channel, uint32_t Amplitude);
HAL_StatusTypeDef HAL_DACEx_NoiseWaveGenerate(DAC_HandleTypeDef* hdac, uint32_t Channel, uint32_t Amplitude);
HAL_StatusTypeDef HAL_DACEx_DualSetValue(DAC_HandleTypeDef* hdac, uint32_t Alignment, uint32_t Data1, uint32_t Data2);

void HAL_DACEx_ConvCpltCallbackCh2(DAC_HandleTypeDef* hdac);
void HAL_DACEx_ConvHalfCpltCallbackCh2(DAC_HandleTypeDef* hdac);
void HAL_DACEx_ErrorCallbackCh2(DAC_HandleTypeDef* hdac);
void HAL_DACEx_DMAUnderrunCallbackCh2(DAC_HandleTypeDef* hdac);


 



 
 
 
 


 



 

 


 
#line 162 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_dac_ex.h"


 

 


 
void DAC_DMAConvCpltCh2(DMA_HandleTypeDef *hdma);
void DAC_DMAErrorCh2(DMA_HandleTypeDef *hdma);
void DAC_DMAHalfConvCpltCh2(DMA_HandleTypeDef *hdma); 


 



 



 
  






 
#line 270 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_dac.h"

 


 



 
 
HAL_StatusTypeDef HAL_DAC_Init(DAC_HandleTypeDef* hdac);
HAL_StatusTypeDef HAL_DAC_DeInit(DAC_HandleTypeDef* hdac);
void HAL_DAC_MspInit(DAC_HandleTypeDef* hdac);
void HAL_DAC_MspDeInit(DAC_HandleTypeDef* hdac);


 



 
 
HAL_StatusTypeDef HAL_DAC_Start(DAC_HandleTypeDef* hdac, uint32_t Channel);
HAL_StatusTypeDef HAL_DAC_Stop(DAC_HandleTypeDef* hdac, uint32_t Channel);
HAL_StatusTypeDef HAL_DAC_Start_DMA(DAC_HandleTypeDef* hdac, uint32_t Channel, uint32_t* pData, uint32_t Length, uint32_t Alignment);
HAL_StatusTypeDef HAL_DAC_Stop_DMA(DAC_HandleTypeDef* hdac, uint32_t Channel);
uint32_t HAL_DAC_GetValue(DAC_HandleTypeDef* hdac, uint32_t Channel);


 



 
 
HAL_StatusTypeDef HAL_DAC_ConfigChannel(DAC_HandleTypeDef* hdac, DAC_ChannelConfTypeDef* sConfig, uint32_t Channel);
HAL_StatusTypeDef HAL_DAC_SetValue(DAC_HandleTypeDef* hdac, uint32_t Channel, uint32_t Alignment, uint32_t Data);


 



 
 
HAL_DAC_StateTypeDef HAL_DAC_GetState(DAC_HandleTypeDef* hdac);
void HAL_DAC_IRQHandler(DAC_HandleTypeDef* hdac);
uint32_t HAL_DAC_GetError(DAC_HandleTypeDef *hdac);

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef* hdac);
void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef* hdac);
void HAL_DAC_ErrorCallbackCh1(DAC_HandleTypeDef *hdac);
void HAL_DAC_DMAUnderrunCallbackCh1(DAC_HandleTypeDef *hdac);


 



 
 
 
 


 



 

 


 
#line 353 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_dac.h"

#line 363 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_dac.h"




 





 





 




 

 


 


 



 



 
  






 
#line 287 "..\\USER\\stm32f7xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_dcmi.h"



































  

 







 
#line 48 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_dcmi.h"
 


 




   

 


 


  
typedef enum
{
  HAL_DCMI_STATE_RESET             = 0x00U,   
  HAL_DCMI_STATE_READY             = 0x01U,   
  HAL_DCMI_STATE_BUSY              = 0x02U,   
  HAL_DCMI_STATE_TIMEOUT           = 0x03U,   
  HAL_DCMI_STATE_ERROR             = 0x04U,   
  HAL_DCMI_STATE_SUSPENDED         = 0x05U        
}HAL_DCMI_StateTypeDef;



  
typedef struct
{
  uint8_t FrameStartCode;  
  uint8_t LineStartCode;   
  uint8_t LineEndCode;     
  uint8_t FrameEndCode;    
}DCMI_CodesInitTypeDef;



   
typedef struct
{
  uint32_t  SynchroMode;                
 

  uint32_t  PCKPolarity;                
 

  uint32_t  VSPolarity;                 
 

  uint32_t  HSPolarity;                 
 

  uint32_t  CaptureRate;                
 

  uint32_t  ExtendedDataMode;           
 

  DCMI_CodesInitTypeDef SyncroCode;     
 

  uint32_t JPEGMode;                    
 

  uint32_t ByteSelectMode;              
 
                                            
  uint32_t ByteSelectStart;             
 

  uint32_t LineSelectMode;              
 
                                            
  uint32_t LineSelectStart;             
 
}DCMI_InitTypeDef;



 
typedef struct
{
  DCMI_TypeDef                  *Instance;            

  DCMI_InitTypeDef              Init;                 

  HAL_LockTypeDef               Lock;                 

  volatile HAL_DCMI_StateTypeDef    State;                

  volatile uint32_t                 XferCount;            

  volatile uint32_t                 XferSize;             

  uint32_t                      XferTransferNumber;   

  uint32_t                      pBuffPtr;             

  DMA_HandleTypeDef             *DMA_Handle;          

  volatile uint32_t                 ErrorCode;            

}DCMI_HandleTypeDef;


 
 



 



 







 



  






 



  







 



 





 



 





 



  





 



 





 



 






 



 







 



 




 



  




 



 







 



 



  





  







  







  



 







 



  





 



 





 



  





 
    


 
 
 


 
  



 






 






 


 



















 















 













 













 













 




 
  
 


 



 
 
HAL_StatusTypeDef HAL_DCMI_Init(DCMI_HandleTypeDef *hdcmi);
HAL_StatusTypeDef HAL_DCMI_DeInit(DCMI_HandleTypeDef *hdcmi);
void       HAL_DCMI_MspInit(DCMI_HandleTypeDef* hdcmi);
void       HAL_DCMI_MspDeInit(DCMI_HandleTypeDef* hdcmi);


 
  


 
 
HAL_StatusTypeDef HAL_DCMI_Start_DMA(DCMI_HandleTypeDef* hdcmi, uint32_t DCMI_Mode, uint32_t pData, uint32_t Length);
HAL_StatusTypeDef HAL_DCMI_Stop(DCMI_HandleTypeDef* hdcmi);
HAL_StatusTypeDef HAL_DCMI_Suspend(DCMI_HandleTypeDef* hdcmi);
HAL_StatusTypeDef HAL_DCMI_Resume(DCMI_HandleTypeDef* hdcmi);
void       HAL_DCMI_ErrorCallback(DCMI_HandleTypeDef *hdcmi);
void       HAL_DCMI_LineEventCallback(DCMI_HandleTypeDef *hdcmi);
void       HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi);
void       HAL_DCMI_VsyncEventCallback(DCMI_HandleTypeDef *hdcmi);
void       HAL_DCMI_VsyncCallback(DCMI_HandleTypeDef *hdcmi);
void       HAL_DCMI_HsyncCallback(DCMI_HandleTypeDef *hdcmi);
void       HAL_DCMI_IRQHandler(DCMI_HandleTypeDef *hdcmi);


 
  


 
 
HAL_StatusTypeDef     HAL_DCMI_ConfigCrop(DCMI_HandleTypeDef *hdcmi, uint32_t X0, uint32_t Y0, uint32_t XSize, uint32_t YSize);
HAL_StatusTypeDef     HAL_DCMI_EnableCrop(DCMI_HandleTypeDef *hdcmi);
HAL_StatusTypeDef     HAL_DCMI_DisableCrop(DCMI_HandleTypeDef *hdcmi);



 
  


 
 
HAL_DCMI_StateTypeDef HAL_DCMI_GetState(DCMI_HandleTypeDef *hdcmi);
uint32_t              HAL_DCMI_GetError(DCMI_HandleTypeDef *hdcmi);


 



 

 
 
 


 




    
 


 


																			 


																	


																					


																				 


																				 


																				 



																				




																				








                                                                                                


                              


                                      





 

 


 
  


 
      


 


  







 
#line 291 "..\\USER\\stm32f7xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_eth.h"



































  

 







 
#line 48 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_eth.h"



 



  
  


 
#line 306 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_eth.h"




 



 
 


 



 


 


 


 


 


 



 


 


 


 


 


 

 

 

  


 



  
typedef enum
{
  HAL_ETH_STATE_RESET             = 0x00U,     
  HAL_ETH_STATE_READY             = 0x01U,     
  HAL_ETH_STATE_BUSY              = 0x02U,     
  HAL_ETH_STATE_BUSY_TX           = 0x12U,     
  HAL_ETH_STATE_BUSY_RX           = 0x22U,     
  HAL_ETH_STATE_BUSY_TX_RX        = 0x32U,     
  HAL_ETH_STATE_BUSY_WR           = 0x42U,     
  HAL_ETH_STATE_BUSY_RD           = 0x82U,     
  HAL_ETH_STATE_TIMEOUT           = 0x03U,     
  HAL_ETH_STATE_ERROR             = 0x04U      
}HAL_ETH_StateTypeDef;



 

typedef struct
{
  uint32_t             AutoNegotiation;           


 

  uint32_t             Speed;                     
 

  uint32_t             DuplexMode;                
 
  
  uint16_t             PhyAddress;                
 
  
  uint8_t             *MACAddr;                    
  
  uint32_t             RxMode;                    
 
  
  uint32_t             ChecksumMode;              
 
  
  uint32_t             MediaInterface    ;               
 

} ETH_InitTypeDef;


 

 

typedef struct
{
  uint32_t             Watchdog;                  


   

  uint32_t             Jabber;                    


 

  uint32_t             InterFrameGap;             
    

  uint32_t             CarrierSense;              
 

  uint32_t             ReceiveOwn;                


   

  uint32_t             LoopbackMode;              
   

  uint32_t             ChecksumOffload;           
     

  uint32_t             RetryTransmission;         

 

  uint32_t             AutomaticPadCRCStrip;      
  

  uint32_t             BackOffLimit;              
 

  uint32_t             DeferralCheck;             
                                                                                                         

  uint32_t             ReceiveAll;                
    

  uint32_t             SourceAddrFilter;          
                   

  uint32_t             PassControlFrames;         
  

  uint32_t             BroadcastFramesReception;  
 

  uint32_t             DestinationAddrFilter;     
  

  uint32_t             PromiscuousMode;           
 

  uint32_t             MulticastFramesFilter;     
  

  uint32_t             UnicastFramesFilter;       
  

  uint32_t             HashTableHigh;             
 

  uint32_t             HashTableLow;              
     

  uint32_t             PauseTime;                 
 

  uint32_t             ZeroQuantaPause;           
   

  uint32_t             PauseLowThreshold;         

 
                                                           
  uint32_t             UnicastPauseFrameDetect;   

   

  uint32_t             ReceiveFlowControl;        

 

  uint32_t             TransmitFlowControl;       

      

  uint32_t             VLANTagComparison;         

  

  uint32_t             VLANTagIdentifier;          

} ETH_MACInitTypeDef;




 

typedef struct
{
 uint32_t              DropTCPIPChecksumErrorFrame; 
  

  uint32_t             ReceiveStoreForward;         
  

  uint32_t             FlushReceivedFrame;          
  

  uint32_t             TransmitStoreForward;        
  

  uint32_t             TransmitThresholdControl;    
 

  uint32_t             ForwardErrorFrames;          
 

  uint32_t             ForwardUndersizedGoodFrames; 

 

  uint32_t             ReceiveThresholdControl;     
 

  uint32_t             SecondFrameOperate;          

 

  uint32_t             AddressAlignedBeats;         
 

  uint32_t             FixedBurst;                  
 
                       
  uint32_t             RxDMABurstLength;            
  

  uint32_t             TxDMABurstLength;            
 
  
  uint32_t             EnhancedDescriptorFormat;    
 

  uint32_t             DescriptorSkipLength;        
                                                              

  uint32_t             DMAArbitration;              
   
} ETH_DMAInitTypeDef;




  

typedef struct  
{
  volatile uint32_t   Status;            
  
  uint32_t   ControlBufferSize;      
  
  uint32_t   Buffer1Addr;            
  
  uint32_t   Buffer2NextDescAddr;    
  
   
  uint32_t   ExtendedStatus;         
  
  uint32_t   Reserved1;              
  
  uint32_t   TimeStampLow;           
  
  uint32_t   TimeStampHigh;          

} ETH_DMADescTypeDef;




  
typedef struct  
{
  ETH_DMADescTypeDef *FSRxDesc;           
  
  ETH_DMADescTypeDef *LSRxDesc;           
  
  uint32_t  SegCount;                     
  
  uint32_t length;                        
  
  uint32_t buffer;                        

} ETH_DMARxFrameInfos;




 
  
typedef struct
{
  ETH_TypeDef                *Instance;      
  
  ETH_InitTypeDef            Init;           
  
  uint32_t                   LinkStatus;     
  
  ETH_DMADescTypeDef         *RxDesc;        
  
  ETH_DMADescTypeDef         *TxDesc;        
  
  ETH_DMARxFrameInfos        RxFrameInfos;   
  
  volatile HAL_ETH_StateTypeDef  State;          
  
  HAL_LockTypeDef            Lock;           

} ETH_HandleTypeDef;

 

 

 


 



  
#line 671 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_eth.h"

 












  


 




  





 












  


 




  




 

 



 












 



  
#line 778 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_eth.h"



  





  




  


  



 

 


 




  


 












 



  
#line 849 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_eth.h"



  








  




  










 

 
#line 899 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_eth.h"

 


 



 
 

  





 


  





 


  




 


  




 



  




 



  




 



  




 



  




 



  
#line 990 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_eth.h"


 



  




 



  




 



  




 



  




 



  




 



  




 



  






 



 




 



  




 



  





 



  





 



  




 



  




 



  




 



  






 



  





 



  




 



  






 



  




 



  




 



  




 



  




 



  






 



  




 



  
#line 1230 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_eth.h"


 



  




 



  




 



  




 



  




 



  
#line 1281 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_eth.h"


 



  




 



  




 



  






 



  




 



  




 



  




 



  
#line 1356 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_eth.h"


 



  
#line 1375 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_eth.h"


 



   




 



  







 



  




 



  






 



  




 



  





 



  





 



 





 



  







 



  
#line 1495 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_eth.h"


 



  







 



  
#line 1532 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_eth.h"


 



  
#line 1545 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_eth.h"



  




  
#line 1560 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_eth.h"



 



  




  



  




 



 

 



 
 



 







 







 






 






 






 






 






 






 






 












 






 






 






 






 











 











 






 






 






 






 













 








 








 







 







 







 










 







 







 







 






 






 






 






 






 






 











 






 






 







 






 






 






 






 






 






 











 










 










 











 





 





 





 





 





 





 





 

                                                            



 





                                                       





 





 






 






 




 
 



 

 



 
HAL_StatusTypeDef HAL_ETH_Init(ETH_HandleTypeDef *heth);
HAL_StatusTypeDef HAL_ETH_DeInit(ETH_HandleTypeDef *heth);
void HAL_ETH_MspInit(ETH_HandleTypeDef *heth);
void HAL_ETH_MspDeInit(ETH_HandleTypeDef *heth);
HAL_StatusTypeDef HAL_ETH_DMATxDescListInit(ETH_HandleTypeDef *heth, ETH_DMADescTypeDef *DMATxDescTab, uint8_t* TxBuff, uint32_t TxBuffCount);
HAL_StatusTypeDef HAL_ETH_DMARxDescListInit(ETH_HandleTypeDef *heth, ETH_DMADescTypeDef *DMARxDescTab, uint8_t *RxBuff, uint32_t RxBuffCount);



 
 



 
HAL_StatusTypeDef HAL_ETH_TransmitFrame(ETH_HandleTypeDef *heth, uint32_t FrameLength);
HAL_StatusTypeDef HAL_ETH_GetReceivedFrame(ETH_HandleTypeDef *heth);
 
HAL_StatusTypeDef HAL_ETH_ReadPHYRegister(ETH_HandleTypeDef *heth, uint16_t PHYReg, uint32_t *RegValue);
HAL_StatusTypeDef HAL_ETH_WritePHYRegister(ETH_HandleTypeDef *heth, uint16_t PHYReg, uint32_t RegValue);
 
HAL_StatusTypeDef HAL_ETH_GetReceivedFrame_IT(ETH_HandleTypeDef *heth);
void HAL_ETH_IRQHandler(ETH_HandleTypeDef *heth);
 
void HAL_ETH_TxCpltCallback(ETH_HandleTypeDef *heth);
void HAL_ETH_RxCpltCallback(ETH_HandleTypeDef *heth);
void HAL_ETH_ErrorCallback(ETH_HandleTypeDef *heth);


 

 



 

HAL_StatusTypeDef HAL_ETH_Start(ETH_HandleTypeDef *heth);
HAL_StatusTypeDef HAL_ETH_Stop(ETH_HandleTypeDef *heth);
HAL_StatusTypeDef HAL_ETH_ConfigMAC(ETH_HandleTypeDef *heth, ETH_MACInitTypeDef *macconf);
HAL_StatusTypeDef HAL_ETH_ConfigDMA(ETH_HandleTypeDef *heth, ETH_DMAInitTypeDef *dmaconf);


  

 



 
HAL_ETH_StateTypeDef HAL_ETH_GetState(ETH_HandleTypeDef *heth);


 



 



 



 








 
#line 295 "..\\USER\\stm32f7xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_flash.h"



































  

 







 
#line 48 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_flash.h"



 



  

  


 
 


 
typedef enum 
{
  FLASH_PROC_NONE = 0U, 
  FLASH_PROC_SECTERASE,
  FLASH_PROC_MASSERASE,
  FLASH_PROC_PROGRAM
} FLASH_ProcedureTypeDef;




 
typedef struct
{
  volatile FLASH_ProcedureTypeDef ProcedureOnGoing;    
  
  volatile uint32_t               NbSectorsToErase;    
  
  volatile uint8_t                VoltageForErase;     
  
  volatile uint32_t               Sector;              

  volatile uint32_t               Address;             
  
  HAL_LockTypeDef             Lock;                

  volatile uint32_t               ErrorCode;           

}FLASH_ProcessTypeDef;



 

 


   




  
#line 114 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_flash.h"


 
  


  






 




  
#line 140 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_flash.h"


 




 




 



 







  



  






 



 
#line 188 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_flash.h"


  



  
  
 


 





 







  





  





  






  





  







 









   









   














 













 



 

 
#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_flash_ex.h"



































 

 







 
#line 48 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_flash_ex.h"



 



 

  


 



 
typedef struct
{
  uint32_t TypeErase;   
 


  uint32_t Banks;       
 

  
  uint32_t Sector;      
 

  uint32_t NbSectors;   
 

  uint32_t VoltageRange;
 

} FLASH_EraseInitTypeDef;



 
typedef struct
{
  uint32_t OptionType;   
 

  uint32_t WRPState;     
 

  uint32_t WRPSector;    
 

  uint32_t RDPLevel;     
 

  uint32_t BORLevel;     
 

  uint32_t USERConfig;   

 
 
  uint32_t BootAddr0;    
 

  uint32_t BootAddr1;    
 

} FLASH_OBProgramInitTypeDef;



 
 



 



  




 
  


  






 
  


  




 
  


  
#line 165 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_flash_ex.h"


 
  


 






  
  


  




  
  



  




  



  




  



                                




 



 




 



 




 



 






 




                                





   





 




 




 
#line 279 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_flash_ex.h"


 
  


 
#line 302 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_flash_ex.h"


 




 





 




 







 



 
#line 351 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_flash_ex.h"


  










 
 
#line 379 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_flash_ex.h"

 
#line 406 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_flash_ex.h"


 

    
#line 428 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_flash_ex.h"



  
  
 


 





 

 

 
                    
 


 



 
 
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *pEraseInit, uint32_t *SectorError);
HAL_StatusTypeDef HAL_FLASHEx_Erase_IT(FLASH_EraseInitTypeDef *pEraseInit);
HAL_StatusTypeDef HAL_FLASHEx_OBProgram(FLASH_OBProgramInitTypeDef *pOBInit);
void              HAL_FLASHEx_OBGetConfig(FLASH_OBProgramInitTypeDef *pOBInit);



 



 
 
 
 
 


 



 




































#line 532 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_flash_ex.h"





#line 545 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_flash_ex.h"

#line 559 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_flash_ex.h"




















 



 

 


 
void FLASH_Erase_Sector(uint32_t Sector, uint8_t VoltageRange);


  



  



 







 
#line 303 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_flash.h"

 


 


 
 
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t TypeProgram, uint32_t Address, uint64_t Data);
HAL_StatusTypeDef HAL_FLASH_Program_IT(uint32_t TypeProgram, uint32_t Address, uint64_t Data);
 
void HAL_FLASH_IRQHandler(void);
  
void HAL_FLASH_EndOfOperationCallback(uint32_t ReturnValue);
void HAL_FLASH_OperationErrorCallback(uint32_t ReturnValue);


 



 
 
HAL_StatusTypeDef HAL_FLASH_Unlock(void);
HAL_StatusTypeDef HAL_FLASH_Lock(void);
HAL_StatusTypeDef HAL_FLASH_OB_Unlock(void);
HAL_StatusTypeDef HAL_FLASH_OB_Lock(void);
 
HAL_StatusTypeDef HAL_FLASH_OB_Launch(void);


 



 
 
uint32_t HAL_FLASH_GetError(void);
HAL_StatusTypeDef FLASH_WaitForLastOperation(uint32_t Timeout);


 



 
 
 


 



 
 


 



  




 

 


 



 






 



 

 


 



 



  



 







 
#line 299 "..\\USER\\stm32f7xx_hal_conf.h"

 
#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_sram.h"



































  

 







 
#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_ll_fmc.h"



































  

 







 
#line 48 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_ll_fmc.h"



 



 



 
































#line 98 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_ll_fmc.h"
								   










										   


									   




#line 124 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_ll_fmc.h"




						   


 



 



 



 



 



 



 



 



 



 



 



 
























 



 










 



 



 



 



 



 



 



 



 



 



 



 
  


 



  
     


   



 
  


   



   
  


   



          
  


   



  
  


   



 
  


   



 



 



 



 



 
  


 



 



 



 
  


 



 



 



 































 

 


 












  
typedef struct
{
  uint32_t NSBank;                       
 

  uint32_t DataAddressMux;               

 

  uint32_t MemoryType;                   

 

  uint32_t MemoryDataWidth;              
 

  uint32_t BurstAccessMode;              

 

  uint32_t WaitSignalPolarity;           

 

  uint32_t WaitSignalActive;             


 

  uint32_t WriteOperation;               
 

  uint32_t WaitSignal;                   

 

  uint32_t ExtendedMode;                 
 

  uint32_t AsynchronousWait;             

 

  uint32_t WriteBurst;                   
 

  uint32_t ContinuousClock;              


 

  uint32_t WriteFifo;                    


 

  uint32_t PageSize;                     
 

}FMC_NORSRAM_InitTypeDef;



 
typedef struct
{
  uint32_t AddressSetupTime;             


 

  uint32_t AddressHoldTime;              


 

  uint32_t DataSetupTime;                



 

  uint32_t BusTurnAroundDuration;        


 

  uint32_t CLKDivision;                  


 

  uint32_t DataLatency;                  





 

  uint32_t AccessMode;                   
 
}FMC_NORSRAM_TimingTypeDef;



  
typedef struct
{
  uint32_t NandBank;               
 

  uint32_t Waitfeature;            
 

  uint32_t MemoryDataWidth;        
 

  uint32_t EccComputation;         
 

  uint32_t ECCPageSize;            
 

  uint32_t TCLRSetupTime;          

 

  uint32_t TARSetupTime;           

 
}FMC_NAND_InitTypeDef;



 
typedef struct
{
  uint32_t SetupTime;            



 

  uint32_t WaitSetupTime;        



 

  uint32_t HoldSetupTime;        




 

  uint32_t HiZSetupTime;         



 
}FMC_NAND_PCC_TimingTypeDef;



   
typedef struct
{
  uint32_t SDBank;                      
 

  uint32_t ColumnBitsNumber;            
 

  uint32_t RowBitsNumber;               
 

  uint32_t MemoryDataWidth;             
 

  uint32_t InternalBankNumber;          
 

  uint32_t CASLatency;                  
 

  uint32_t WriteProtection;             
 

  uint32_t SDClockPeriod;               

 

  uint32_t ReadBurst;                   

 

  uint32_t ReadPipeDelay;               
 
}FMC_SDRAM_InitTypeDef;



 
typedef struct
{
  uint32_t LoadToActiveDelay;            

 

  uint32_t ExitSelfRefreshDelay;         

 

  uint32_t SelfRefreshTime;              

 

  uint32_t RowCycleDelay;                


 

  uint32_t WriteRecoveryTime;            
 

  uint32_t RPDelay;                      

 

  uint32_t RCDDelay;                     

  
}FMC_SDRAM_TimingTypeDef;



 
typedef struct
{
  uint32_t CommandMode;                  
 

  uint32_t CommandTarget;                
 

  uint32_t AutoRefreshNumber;            

 
  uint32_t ModeRegisterDefinition;        
}FMC_SDRAM_CommandTypeDef;


 

 


 



 



 






 



 




 



 





 



 





 



 




 



 




 



 




 



 




 



 




 



 




 



 




 



 




   



 







   



 




 
  


 




  



 




 
	


 






 
    


  



 


 



 



 




 



 



 



 




 



 




 



 
#line 900 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_ll_fmc.h"


 
  


  



 


 




 



 






 



 





 



 





 



 




 



 





 



 




 



 





 



 




 
  


 





 



 
#line 1018 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_ll_fmc.h"


 



 





 



 





 



  



   






 
    


  
#line 1067 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_ll_fmc.h"


 


 



 

 


 




 
 





  







  




  




 
 




   






 




  
    



  










   











 

                                                                                                                           











 












 









 









 











 









 



 



  

 


 



 


 
HAL_StatusTypeDef  FMC_NORSRAM_Init(FMC_Bank1_TypeDef *Device, FMC_NORSRAM_InitTypeDef *Init);
HAL_StatusTypeDef  FMC_NORSRAM_Timing_Init(FMC_Bank1_TypeDef *Device, FMC_NORSRAM_TimingTypeDef *Timing, uint32_t Bank);
HAL_StatusTypeDef  FMC_NORSRAM_Extended_Timing_Init(FMC_Bank1E_TypeDef *Device, FMC_NORSRAM_TimingTypeDef *Timing, uint32_t Bank, uint32_t ExtendedMode);
HAL_StatusTypeDef  FMC_NORSRAM_DeInit(FMC_Bank1_TypeDef *Device, FMC_Bank1E_TypeDef *ExDevice, uint32_t Bank);


  



 
HAL_StatusTypeDef  FMC_NORSRAM_WriteOperation_Enable(FMC_Bank1_TypeDef *Device, uint32_t Bank);
HAL_StatusTypeDef  FMC_NORSRAM_WriteOperation_Disable(FMC_Bank1_TypeDef *Device, uint32_t Bank);


 


 



 


 
HAL_StatusTypeDef  FMC_NAND_Init(FMC_Bank3_TypeDef *Device, FMC_NAND_InitTypeDef *Init);
HAL_StatusTypeDef  FMC_NAND_CommonSpace_Timing_Init(FMC_Bank3_TypeDef *Device, FMC_NAND_PCC_TimingTypeDef *Timing, uint32_t Bank);
HAL_StatusTypeDef  FMC_NAND_AttributeSpace_Timing_Init(FMC_Bank3_TypeDef *Device, FMC_NAND_PCC_TimingTypeDef *Timing, uint32_t Bank);
HAL_StatusTypeDef  FMC_NAND_DeInit(FMC_Bank3_TypeDef *Device, uint32_t Bank);


 



 
HAL_StatusTypeDef  FMC_NAND_ECC_Enable(FMC_Bank3_TypeDef *Device, uint32_t Bank);
HAL_StatusTypeDef  FMC_NAND_ECC_Disable(FMC_Bank3_TypeDef *Device, uint32_t Bank);
HAL_StatusTypeDef  FMC_NAND_GetECC(FMC_Bank3_TypeDef *Device, uint32_t *ECCval, uint32_t Bank, uint32_t Timeout);


 



 


 
HAL_StatusTypeDef  FMC_SDRAM_Init(FMC_Bank5_6_TypeDef *Device, FMC_SDRAM_InitTypeDef *Init);
HAL_StatusTypeDef  FMC_SDRAM_Timing_Init(FMC_Bank5_6_TypeDef *Device, FMC_SDRAM_TimingTypeDef *Timing, uint32_t Bank);
HAL_StatusTypeDef  FMC_SDRAM_DeInit(FMC_Bank5_6_TypeDef *Device, uint32_t Bank);



 



 
HAL_StatusTypeDef  FMC_SDRAM_WriteProtection_Enable(FMC_Bank5_6_TypeDef *Device, uint32_t Bank);
HAL_StatusTypeDef  FMC_SDRAM_WriteProtection_Disable(FMC_Bank5_6_TypeDef *Device, uint32_t Bank);
HAL_StatusTypeDef  FMC_SDRAM_SendCommand(FMC_Bank5_6_TypeDef *Device, FMC_SDRAM_CommandTypeDef *Command, uint32_t Timeout);
HAL_StatusTypeDef  FMC_SDRAM_ProgramRefreshRate(FMC_Bank5_6_TypeDef *Device, uint32_t RefreshRate);
HAL_StatusTypeDef  FMC_SDRAM_SetAutoRefreshNumber(FMC_Bank5_6_TypeDef *Device, uint32_t AutoRefreshNumber);
uint32_t           FMC_SDRAM_GetModeStatus(FMC_Bank5_6_TypeDef *Device, uint32_t Bank);


 



 



 



 



 






 
#line 48 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_sram.h"



 


  

 



 


  
typedef enum
{
  HAL_SRAM_STATE_RESET     = 0x00U,   
  HAL_SRAM_STATE_READY     = 0x01U,   
  HAL_SRAM_STATE_BUSY      = 0x02U,   
  HAL_SRAM_STATE_ERROR     = 0x03U,   
  HAL_SRAM_STATE_PROTECTED = 0x04U    
  
}HAL_SRAM_StateTypeDef;



  
typedef struct
{
  FMC_Bank1_TypeDef           *Instance;    
  
  FMC_Bank1E_TypeDef  *Extended;   
  
  FMC_NORSRAM_InitTypeDef       Init;        

  HAL_LockTypeDef               Lock;         
  
  volatile HAL_SRAM_StateTypeDef    State;       
  
  DMA_HandleTypeDef             *hdma;       
  
}SRAM_HandleTypeDef; 



 

 
 



 




 




 

 


 



 

 
HAL_StatusTypeDef HAL_SRAM_Init(SRAM_HandleTypeDef *hsram, FMC_NORSRAM_TimingTypeDef *Timing, FMC_NORSRAM_TimingTypeDef *ExtTiming);
HAL_StatusTypeDef HAL_SRAM_DeInit(SRAM_HandleTypeDef *hsram);
void HAL_SRAM_MspInit(SRAM_HandleTypeDef *hsram);
void HAL_SRAM_MspDeInit(SRAM_HandleTypeDef *hsram);



 



 

 
HAL_StatusTypeDef HAL_SRAM_Read_8b(SRAM_HandleTypeDef *hsram, uint32_t *pAddress, uint8_t *pDstBuffer, uint32_t BufferSize);
HAL_StatusTypeDef HAL_SRAM_Write_8b(SRAM_HandleTypeDef *hsram, uint32_t *pAddress, uint8_t *pSrcBuffer, uint32_t BufferSize);
HAL_StatusTypeDef HAL_SRAM_Read_16b(SRAM_HandleTypeDef *hsram, uint32_t *pAddress, uint16_t *pDstBuffer, uint32_t BufferSize);
HAL_StatusTypeDef HAL_SRAM_Write_16b(SRAM_HandleTypeDef *hsram, uint32_t *pAddress, uint16_t *pSrcBuffer, uint32_t BufferSize);
HAL_StatusTypeDef HAL_SRAM_Read_32b(SRAM_HandleTypeDef *hsram, uint32_t *pAddress, uint32_t *pDstBuffer, uint32_t BufferSize);
HAL_StatusTypeDef HAL_SRAM_Write_32b(SRAM_HandleTypeDef *hsram, uint32_t *pAddress, uint32_t *pSrcBuffer, uint32_t BufferSize);
HAL_StatusTypeDef HAL_SRAM_Read_DMA(SRAM_HandleTypeDef *hsram, uint32_t *pAddress, uint32_t *pDstBuffer, uint32_t BufferSize);
HAL_StatusTypeDef HAL_SRAM_Write_DMA(SRAM_HandleTypeDef *hsram, uint32_t *pAddress, uint32_t *pSrcBuffer, uint32_t BufferSize);

void HAL_SRAM_DMA_XferCpltCallback(DMA_HandleTypeDef *hdma);
void HAL_SRAM_DMA_XferErrorCallback(DMA_HandleTypeDef *hdma);



 
  


 

 
HAL_StatusTypeDef HAL_SRAM_WriteOperation_Enable(SRAM_HandleTypeDef *hsram);
HAL_StatusTypeDef HAL_SRAM_WriteOperation_Disable(SRAM_HandleTypeDef *hsram);



 



 

 
HAL_SRAM_StateTypeDef HAL_SRAM_GetState(SRAM_HandleTypeDef *hsram);



  



 
  


  



 
  






 
#line 303 "..\\USER\\stm32f7xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_nor.h"



































  

 







 
#line 48 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_nor.h"




 



  

 


 



  
typedef enum
{  
  HAL_NOR_STATE_RESET             = 0x00U,   
  HAL_NOR_STATE_READY             = 0x01U,   
  HAL_NOR_STATE_BUSY              = 0x02U,   
  HAL_NOR_STATE_ERROR             = 0x03U,   
  HAL_NOR_STATE_PROTECTED         = 0x04U    
}HAL_NOR_StateTypeDef;



 
typedef enum
{
  HAL_NOR_STATUS_SUCCESS  = 0U,
  HAL_NOR_STATUS_ONGOING,
  HAL_NOR_STATUS_ERROR,
  HAL_NOR_STATUS_TIMEOUT
}HAL_NOR_StatusTypeDef;



 
typedef struct
{
  uint16_t Manufacturer_Code;   

  uint16_t Device_Code1;

  uint16_t Device_Code2;

  uint16_t Device_Code3;       


 
}NOR_IDTypeDef;



 
typedef struct
{
  

 

  uint16_t CFI_1;

  uint16_t CFI_2;

  uint16_t CFI_3;

  uint16_t CFI_4;
}NOR_CFITypeDef;



  
typedef struct
{
  FMC_Bank1_TypeDef           *Instance;     

  FMC_Bank1E_TypeDef  *Extended;     

  FMC_NORSRAM_InitTypeDef       Init;          

  HAL_LockTypeDef               Lock;          

  volatile HAL_NOR_StateTypeDef     State;         

}NOR_HandleTypeDef;


 
  
 
 


 



 



 

 


 



 

 
HAL_StatusTypeDef HAL_NOR_Init(NOR_HandleTypeDef *hnor, FMC_NORSRAM_TimingTypeDef *Timing, FMC_NORSRAM_TimingTypeDef *ExtTiming);
HAL_StatusTypeDef HAL_NOR_DeInit(NOR_HandleTypeDef *hnor);
void HAL_NOR_MspInit(NOR_HandleTypeDef *hnor);
void HAL_NOR_MspDeInit(NOR_HandleTypeDef *hnor);
void HAL_NOR_MspWait(NOR_HandleTypeDef *hnor, uint32_t Timeout);


 



 

 
HAL_StatusTypeDef HAL_NOR_Read_ID(NOR_HandleTypeDef *hnor, NOR_IDTypeDef *pNOR_ID);
HAL_StatusTypeDef HAL_NOR_ReturnToReadMode(NOR_HandleTypeDef *hnor);
HAL_StatusTypeDef HAL_NOR_Read(NOR_HandleTypeDef *hnor, uint32_t *pAddress, uint16_t *pData);
HAL_StatusTypeDef HAL_NOR_Program(NOR_HandleTypeDef *hnor, uint32_t *pAddress, uint16_t *pData);

HAL_StatusTypeDef HAL_NOR_ReadBuffer(NOR_HandleTypeDef *hnor, uint32_t uwAddress, uint16_t *pData, uint32_t uwBufferSize);
HAL_StatusTypeDef HAL_NOR_ProgramBuffer(NOR_HandleTypeDef *hnor, uint32_t uwAddress, uint16_t *pData, uint32_t uwBufferSize);

HAL_StatusTypeDef HAL_NOR_Erase_Block(NOR_HandleTypeDef *hnor, uint32_t BlockAddress, uint32_t Address);
HAL_StatusTypeDef HAL_NOR_Erase_Chip(NOR_HandleTypeDef *hnor, uint32_t Address);
HAL_StatusTypeDef HAL_NOR_Read_CFI(NOR_HandleTypeDef *hnor, NOR_CFITypeDef *pNOR_CFI);


 
  


 

 
HAL_StatusTypeDef HAL_NOR_WriteOperation_Enable(NOR_HandleTypeDef *hnor);
HAL_StatusTypeDef HAL_NOR_WriteOperation_Disable(NOR_HandleTypeDef *hnor);


 
  


 

 
HAL_NOR_StateTypeDef  HAL_NOR_GetState(NOR_HandleTypeDef *hnor);
HAL_NOR_StatusTypeDef HAL_NOR_GetStatus(NOR_HandleTypeDef *hnor, uint32_t Address, uint32_t Timeout);


 
    


 
  
 
 
 


 
 





 





 

   
 



 






 

 


 






 




 





 







 



  



 







 
#line 307 "..\\USER\\stm32f7xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_nand.h"



































 

 







 
#line 48 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_nand.h"



 



  

 
 


 



 
typedef enum
{
  HAL_NAND_STATE_RESET     = 0x00U,   
  HAL_NAND_STATE_READY     = 0x01U,   
  HAL_NAND_STATE_BUSY      = 0x02U,   
  HAL_NAND_STATE_ERROR     = 0x03U    
}HAL_NAND_StateTypeDef;
   


 
typedef struct
{
   

  uint8_t Maker_Id; 

  uint8_t Device_Id;

  uint8_t Third_Id;

  uint8_t Fourth_Id;
}NAND_IDTypeDef;



 
typedef struct 
{
  uint16_t Page;    

  uint16_t Zone;    

  uint16_t Block;   

}NAND_AddressTypeDef;



  
typedef struct
{
  uint32_t PageSize;        

  uint32_t SpareAreaSize;   

  uint32_t BlockSize;       

  uint32_t BlockNbr;        

  uint32_t ZoneSize;        
}NAND_InfoTypeDef;



    
typedef struct
{
  FMC_Bank3_TypeDef             *Instance;   
  
  FMC_NAND_InitTypeDef         Init;        

  HAL_LockTypeDef              Lock;        

  volatile HAL_NAND_StateTypeDef   State;       

  NAND_InfoTypeDef             Info;        
}NAND_HandleTypeDef;


 

 
 


  




 




 

 


 
    


 

 
HAL_StatusTypeDef  HAL_NAND_Init(NAND_HandleTypeDef *hnand, FMC_NAND_PCC_TimingTypeDef *ComSpace_Timing, FMC_NAND_PCC_TimingTypeDef *AttSpace_Timing);
HAL_StatusTypeDef  HAL_NAND_DeInit(NAND_HandleTypeDef *hnand);
void               HAL_NAND_MspInit(NAND_HandleTypeDef *hnand);
void               HAL_NAND_MspDeInit(NAND_HandleTypeDef *hnand);
void               HAL_NAND_IRQHandler(NAND_HandleTypeDef *hnand);
void               HAL_NAND_ITCallback(NAND_HandleTypeDef *hnand);



 
  


 

 
HAL_StatusTypeDef  HAL_NAND_Read_ID(NAND_HandleTypeDef *hnand, NAND_IDTypeDef *pNAND_ID);
HAL_StatusTypeDef  HAL_NAND_Reset(NAND_HandleTypeDef *hnand);

HAL_StatusTypeDef  HAL_NAND_Read_Page_8b(NAND_HandleTypeDef *hnand, NAND_AddressTypeDef *pAddress, uint8_t *pBuffer, uint32_t NumPageToRead);
HAL_StatusTypeDef  HAL_NAND_Read_Page_16b(NAND_HandleTypeDef *hnand, NAND_AddressTypeDef *pAddress, uint16_t *pBuffer, uint32_t NumPageToRead);
HAL_StatusTypeDef  HAL_NAND_Write_Page_8b(NAND_HandleTypeDef *hnand, NAND_AddressTypeDef *pAddress, uint8_t *pBuffer, uint32_t NumPageToWrite);
HAL_StatusTypeDef  HAL_NAND_Write_Page_16b(NAND_HandleTypeDef *hnand, NAND_AddressTypeDef *pAddress, uint16_t *pBuffer, uint32_t NumPageToWrite);
HAL_StatusTypeDef  HAL_NAND_Read_SpareArea_8b(NAND_HandleTypeDef *hnand, NAND_AddressTypeDef *pAddress, uint8_t *pBuffer, uint32_t NumSpareAreaToRead);
HAL_StatusTypeDef  HAL_NAND_Read_SpareArea_16b(NAND_HandleTypeDef *hnand, NAND_AddressTypeDef *pAddress, uint16_t *pBuffer, uint32_t NumSpareAreaToRead);
HAL_StatusTypeDef  HAL_NAND_Write_SpareArea_8b(NAND_HandleTypeDef *hnand, NAND_AddressTypeDef *pAddress, uint8_t *pBuffer, uint32_t NumSpareAreaTowrite);
HAL_StatusTypeDef  HAL_NAND_Write_SpareArea_16b(NAND_HandleTypeDef *hnand, NAND_AddressTypeDef *pAddress, uint16_t *pBuffer, uint32_t NumSpareAreaTowrite);
HAL_StatusTypeDef  HAL_NAND_Erase_Block(NAND_HandleTypeDef *hnand, NAND_AddressTypeDef *pAddress);
uint32_t           HAL_NAND_Read_Status(NAND_HandleTypeDef *hnand);
uint32_t           HAL_NAND_Address_Inc(NAND_HandleTypeDef *hnand, NAND_AddressTypeDef *pAddress);



 



 

 
HAL_StatusTypeDef  HAL_NAND_ECC_Enable(NAND_HandleTypeDef *hnand);
HAL_StatusTypeDef  HAL_NAND_ECC_Disable(NAND_HandleTypeDef *hnand);
HAL_StatusTypeDef  HAL_NAND_GetECC(NAND_HandleTypeDef *hnand, uint32_t *ECCval, uint32_t Timeout);



 
    


 
 
HAL_NAND_StateTypeDef HAL_NAND_GetState(NAND_HandleTypeDef *hnand);
uint32_t              HAL_NAND_Read_Status(NAND_HandleTypeDef *hnand);


 



 
 
 
 


 











#line 250 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_nand.h"

 
#line 258 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_nand.h"


 

 


 






 







 






 
    


 


  



  







 
#line 311 "..\\USER\\stm32f7xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_sdram.h"



































  

 







 
#line 48 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_sdram.h"



 



  

    



 
	 


  
typedef enum
{
  HAL_SDRAM_STATE_RESET             = 0x00U,   
  HAL_SDRAM_STATE_READY             = 0x01U,   
  HAL_SDRAM_STATE_BUSY              = 0x02U,   
  HAL_SDRAM_STATE_ERROR             = 0x03U,   
  HAL_SDRAM_STATE_WRITE_PROTECTED   = 0x04U,   
  HAL_SDRAM_STATE_PRECHARGED        = 0x05U    
  
}HAL_SDRAM_StateTypeDef;



  
typedef struct
{
  FMC_Bank5_6_TypeDef             *Instance;   
  
  FMC_SDRAM_InitTypeDef         Init;        
  
  volatile HAL_SDRAM_StateTypeDef   State;       
  
  HAL_LockTypeDef               Lock;         

  DMA_HandleTypeDef             *hdma;       
  
}SDRAM_HandleTypeDef;


 

 
 



 




 




 

 



 



 

 
HAL_StatusTypeDef HAL_SDRAM_Init(SDRAM_HandleTypeDef *hsdram, FMC_SDRAM_TimingTypeDef *Timing);
HAL_StatusTypeDef HAL_SDRAM_DeInit(SDRAM_HandleTypeDef *hsdram);
void HAL_SDRAM_MspInit(SDRAM_HandleTypeDef *hsdram);
void HAL_SDRAM_MspDeInit(SDRAM_HandleTypeDef *hsdram);

void HAL_SDRAM_IRQHandler(SDRAM_HandleTypeDef *hsdram);
void HAL_SDRAM_RefreshErrorCallback(SDRAM_HandleTypeDef *hsdram);
void HAL_SDRAM_DMA_XferCpltCallback(DMA_HandleTypeDef *hdma);
void HAL_SDRAM_DMA_XferErrorCallback(DMA_HandleTypeDef *hdma);



 



 
 
HAL_StatusTypeDef HAL_SDRAM_Read_8b(SDRAM_HandleTypeDef *hsdram, uint32_t *pAddress, uint8_t *pDstBuffer, uint32_t BufferSize);
HAL_StatusTypeDef HAL_SDRAM_Write_8b(SDRAM_HandleTypeDef *hsdram, uint32_t *pAddress, uint8_t *pSrcBuffer, uint32_t BufferSize);
HAL_StatusTypeDef HAL_SDRAM_Read_16b(SDRAM_HandleTypeDef *hsdram, uint32_t *pAddress, uint16_t *pDstBuffer, uint32_t BufferSize);
HAL_StatusTypeDef HAL_SDRAM_Write_16b(SDRAM_HandleTypeDef *hsdram, uint32_t *pAddress, uint16_t *pSrcBuffer, uint32_t BufferSize);
HAL_StatusTypeDef HAL_SDRAM_Read_32b(SDRAM_HandleTypeDef *hsdram, uint32_t *pAddress, uint32_t *pDstBuffer, uint32_t BufferSize);
HAL_StatusTypeDef HAL_SDRAM_Write_32b(SDRAM_HandleTypeDef *hsdram, uint32_t *pAddress, uint32_t *pSrcBuffer, uint32_t BufferSize);

HAL_StatusTypeDef HAL_SDRAM_Read_DMA(SDRAM_HandleTypeDef *hsdram, uint32_t * pAddress, uint32_t *pDstBuffer, uint32_t BufferSize);
HAL_StatusTypeDef HAL_SDRAM_Write_DMA(SDRAM_HandleTypeDef *hsdram, uint32_t *pAddress, uint32_t *pSrcBuffer, uint32_t BufferSize);



 
  


 
 
HAL_StatusTypeDef HAL_SDRAM_WriteProtection_Enable(SDRAM_HandleTypeDef *hsdram);
HAL_StatusTypeDef HAL_SDRAM_WriteProtection_Disable(SDRAM_HandleTypeDef *hsdram);
HAL_StatusTypeDef HAL_SDRAM_SendCommand(SDRAM_HandleTypeDef *hsdram, FMC_SDRAM_CommandTypeDef *Command, uint32_t Timeout);
HAL_StatusTypeDef HAL_SDRAM_ProgramRefreshRate(SDRAM_HandleTypeDef *hsdram, uint32_t RefreshRate);
HAL_StatusTypeDef HAL_SDRAM_SetAutoRefreshNumber(SDRAM_HandleTypeDef *hsdram, uint32_t AutoRefreshNumber);
uint32_t          HAL_SDRAM_GetModeStatus(SDRAM_HandleTypeDef *hsdram);



 



 
 
HAL_SDRAM_StateTypeDef  HAL_SDRAM_GetState(SDRAM_HandleTypeDef *hsdram);


 



 



  



 







 
#line 315 "..\\USER\\stm32f7xx_hal_conf.h"






#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_i2c.h"



































 

 







 
#line 48 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_i2c.h"



 



  

 


 




 
typedef struct
{
  uint32_t Timing;              

 

  uint32_t OwnAddress1;         
 

  uint32_t AddressingMode;      
 

  uint32_t DualAddressMode;     
 

  uint32_t OwnAddress2;         
 

  uint32_t OwnAddress2Masks;    
 

  uint32_t GeneralCallMode;     
 

  uint32_t NoStretchMode;       
 

}I2C_InitTypeDef;



 



























  

typedef enum
{
  HAL_I2C_STATE_RESET             = 0x00U,    
  HAL_I2C_STATE_READY             = 0x20U,    
  HAL_I2C_STATE_BUSY              = 0x24U,    
  HAL_I2C_STATE_BUSY_TX           = 0x21U,     
  HAL_I2C_STATE_BUSY_RX           = 0x22U,    
  HAL_I2C_STATE_LISTEN            = 0x28U,    
  HAL_I2C_STATE_BUSY_TX_LISTEN    = 0x29U,   
 
  HAL_I2C_STATE_BUSY_RX_LISTEN    = 0x2AU,   
 
  HAL_I2C_STATE_ABORT             = 0x60,     
  HAL_I2C_STATE_TIMEOUT           = 0xA0U,    
  HAL_I2C_STATE_ERROR             = 0xE0U      

}HAL_I2C_StateTypeDef;



 


















 
typedef enum
{
  HAL_I2C_MODE_NONE               = 0x00U,    
  HAL_I2C_MODE_MASTER             = 0x10U,    
  HAL_I2C_MODE_SLAVE              = 0x20U,    
  HAL_I2C_MODE_MEM                = 0x40U     

}HAL_I2C_ModeTypeDef;



 




 
#line 193 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_i2c.h"


 




 
typedef struct __I2C_HandleTypeDef
{
  I2C_TypeDef                *Instance;       

  I2C_InitTypeDef            Init;            

  uint8_t                    *pBuffPtr;       

  uint16_t                   XferSize;        

  volatile uint16_t              XferCount;       

  volatile uint32_t              XferOptions;    
 

  volatile uint32_t              PreviousState;   

  HAL_StatusTypeDef (*XferISR)(struct __I2C_HandleTypeDef *hi2c, uint32_t ITFlags, uint32_t ITSources);  

  DMA_HandleTypeDef          *hdmatx;         

  DMA_HandleTypeDef          *hdmarx;         

  HAL_LockTypeDef            Lock;            

  volatile HAL_I2C_StateTypeDef  State;           

  volatile HAL_I2C_ModeTypeDef   Mode;            

  volatile uint32_t              ErrorCode;       

  volatile uint32_t              AddrEventCount;  
}I2C_HandleTypeDef;


 



   
 



 



 







 



 




 



 




 



 
#line 288 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_i2c.h"


 



 




 



 




 



 




 
  


 




 



 





 



 






 






 
#line 362 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_i2c.h"


 



  
#line 385 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_i2c.h"


 



 

 
  


 




 















 















 

 













 
























 


















 


 



 





 





 



 

 
#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_i2c_ex.h"



































 

 







 
#line 48 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_i2c_ex.h"



 



  

 
 



 



 




 



 















 
  


  
  
 
 

 
HAL_StatusTypeDef HAL_I2CEx_ConfigAnalogFilter(I2C_HandleTypeDef *hi2c, uint32_t AnalogFilter);
HAL_StatusTypeDef HAL_I2CEx_ConfigDigitalFilter(I2C_HandleTypeDef *hi2c, uint32_t DigitalFilter);

void HAL_I2CEx_EnableFastModePlus(uint32_t ConfigFastModePlus);
void HAL_I2CEx_DisableFastModePlus(uint32_t ConfigFastModePlus);


 


 



 

 


 





#line 157 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_i2c_ex.h"


 


 

 


 
 


 



 



 








 
#line 521 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_i2c.h"

 


 



 
 
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef HAL_I2C_DeInit (I2C_HandleTypeDef *hi2c);
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MspDeInit(I2C_HandleTypeDef *hi2c);


 



 
 
  
HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Slave_Transmit(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Slave_Receive(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint32_t Trials, uint32_t Timeout);

  
HAL_StatusTypeDef HAL_I2C_Master_Transmit_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Master_Receive_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Slave_Transmit_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Slave_Receive_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Mem_Write_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Mem_Read_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);

HAL_StatusTypeDef HAL_I2C_Master_Sequential_Transmit_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_Master_Sequential_Receive_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_Slave_Sequential_Transmit_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_Slave_Sequential_Receive_IT(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size, uint32_t XferOptions);
HAL_StatusTypeDef HAL_I2C_EnableListen_IT(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef HAL_I2C_DisableListen_IT(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef HAL_I2C_Master_Abort_IT(I2C_HandleTypeDef *hi2c, uint16_t DevAddress);

  
HAL_StatusTypeDef HAL_I2C_Master_Transmit_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Master_Receive_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Slave_Transmit_DMA(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Slave_Receive_DMA(I2C_HandleTypeDef *hi2c, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Mem_Write_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2C_Mem_Read_DMA(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size);


 



 
 
void HAL_I2C_EV_IRQHandler(I2C_HandleTypeDef *hi2c);
void HAL_I2C_ER_IRQHandler(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode);
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c);
void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c);


  



 
 
HAL_I2C_StateTypeDef HAL_I2C_GetState(I2C_HandleTypeDef *hi2c);
HAL_I2C_ModeTypeDef  HAL_I2C_GetMode(I2C_HandleTypeDef *hi2c);
uint32_t             HAL_I2C_GetError(I2C_HandleTypeDef *hi2c);



  



  

 


 



  

 


 







#line 643 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_i2c.h"










































  

 


 
 


  



  



  








 
#line 323 "..\\USER\\stm32f7xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_i2s.h"



































  

 







 
#line 48 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_i2s.h"



 



  

  


 



 
typedef struct
{
  uint32_t Mode;                
 

  uint32_t Standard;            
 

  uint32_t DataFormat;          
 

  uint32_t MCLKOutput;          
 

  uint32_t AudioFreq;           
 

  uint32_t CPOL;                
 
   
  uint32_t ClockSource;         
 
}I2S_InitTypeDef;



  
typedef enum
{
  HAL_I2S_STATE_RESET      = 0x00U,   
  HAL_I2S_STATE_READY      = 0x01U,   
  HAL_I2S_STATE_BUSY       = 0x02U,      
  HAL_I2S_STATE_BUSY_TX    = 0x03U,    
  HAL_I2S_STATE_BUSY_RX    = 0x04U,   
  HAL_I2S_STATE_BUSY_TX_RX = 0x05U,   
  HAL_I2S_STATE_TIMEOUT    = 0x06U,     
  HAL_I2S_STATE_ERROR      = 0x07U          
                                                                        
}HAL_I2S_StateTypeDef;



 
typedef struct
{
  SPI_TypeDef                *Instance;     

  I2S_InitTypeDef            Init;          
  
  uint16_t                   *pTxBuffPtr;   
  
  volatile uint16_t              TxXferSize;    
  
  volatile uint16_t              TxXferCount;   
  
  uint16_t                   *pRxBuffPtr;   
  
  volatile uint16_t              RxXferSize;    
  
  volatile uint16_t              RxXferCount;  




 

  DMA_HandleTypeDef          *hdmatx;       

  DMA_HandleTypeDef          *hdmarx;       
  
  volatile HAL_LockTypeDef       Lock;          
  
  volatile HAL_I2S_StateTypeDef  State;         

  volatile uint32_t  ErrorCode;                 

}I2S_HandleTypeDef;


 

 


 




 
#line 161 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_i2s.h"



 


 




 



 






 
  


 







 
  


 






 



 




 



 
#line 230 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_i2s.h"


 
            



 




 



 





 



  











 



  
  
 


 




 





 











   


 









 














 





 
#line 342 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_i2s.h"
    



 
#line 353 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_i2s.h"


  

 


 
                                                


 

 
HAL_StatusTypeDef HAL_I2S_Init(I2S_HandleTypeDef *hi2s);
HAL_StatusTypeDef HAL_I2S_DeInit (I2S_HandleTypeDef *hi2s);
void HAL_I2S_MspInit(I2S_HandleTypeDef *hi2s);
void HAL_I2S_MspDeInit(I2S_HandleTypeDef *hi2s);


 



 
 
  
HAL_StatusTypeDef HAL_I2S_Transmit(I2S_HandleTypeDef *hi2s, uint16_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2S_Receive(I2S_HandleTypeDef *hi2s, uint16_t *pData, uint16_t Size, uint32_t Timeout);

  
HAL_StatusTypeDef HAL_I2S_Transmit_IT(I2S_HandleTypeDef *hi2s, uint16_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2S_Receive_IT(I2S_HandleTypeDef *hi2s, uint16_t *pData, uint16_t Size);
void HAL_I2S_IRQHandler(I2S_HandleTypeDef *hi2s);

 
HAL_StatusTypeDef HAL_I2S_Transmit_DMA(I2S_HandleTypeDef *hi2s, uint16_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_I2S_Receive_DMA(I2S_HandleTypeDef *hi2s, uint16_t *pData, uint16_t Size);

HAL_StatusTypeDef HAL_I2S_DMAPause(I2S_HandleTypeDef *hi2s);
HAL_StatusTypeDef HAL_I2S_DMAResume(I2S_HandleTypeDef *hi2s);
HAL_StatusTypeDef HAL_I2S_DMAStop(I2S_HandleTypeDef *hi2s);

 
void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s);
void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef *hi2s);
void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s);
void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s);
void HAL_I2S_ErrorCallback(I2S_HandleTypeDef *hi2s);


 



 
 
HAL_I2S_StateTypeDef HAL_I2S_GetState(I2S_HandleTypeDef *hi2s);
uint32_t HAL_I2S_GetError(I2S_HandleTypeDef *hi2s);


 



 


 
 
 


 



 

 


 


								   




                           













                                    



								 




 



  



   
	







 
#line 327 "..\\USER\\stm32f7xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_iwdg.h"



































 

 







 
#line 48 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_iwdg.h"



 



 

 


 



 
typedef struct
{
  uint32_t Prescaler;  
 

  uint32_t Reload;     
 

  uint32_t Window;     
 

} IWDG_InitTypeDef;



 
typedef struct
{
  IWDG_TypeDef                 *Instance;   

  IWDG_InitTypeDef             Init;        

}IWDG_HandleTypeDef;



 

 


 



 
#line 108 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_iwdg.h"


 



 



 



 

 


 





 







 




 

 


 



 
 
HAL_StatusTypeDef HAL_IWDG_Init(IWDG_HandleTypeDef *hiwdg);


 



 
 
HAL_StatusTypeDef HAL_IWDG_Refresh(IWDG_HandleTypeDef *hiwdg);


 



 

 


 



 







 

 


 





 






 






 
#line 223 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_iwdg.h"





 






 




 



 



 








 
#line 331 "..\\USER\\stm32f7xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_lptim.h"



































  

 







 
#line 48 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_lptim.h"



 




 
  
  


 



 



 



 
typedef struct
{
  uint32_t Source;         
 

  uint32_t Prescaler;      
 
  
}LPTIM_ClockConfigTypeDef;



 
typedef struct
{
  uint32_t Polarity;      




  
  
  uint32_t SampleTime;     

   
  
}LPTIM_ULPClockConfigTypeDef;



 
typedef struct
{
  uint32_t Source;        
 
  
  uint32_t ActiveEdge;    

 
  
  uint32_t SampleTime;    

   
}LPTIM_TriggerConfigTypeDef;



 
typedef struct
{                                                    
  LPTIM_ClockConfigTypeDef     Clock;                
                                                    
  LPTIM_ULPClockConfigTypeDef  UltraLowPowerClock;   
                                                    
  LPTIM_TriggerConfigTypeDef   Trigger;              
                                                    
  uint32_t                     OutputPolarity;      
 
                                                    
  uint32_t                     UpdateMode;          

 

  uint32_t                     CounterSource;       

   
  
}LPTIM_InitTypeDef;



  
typedef enum __HAL_LPTIM_StateTypeDef
{
  HAL_LPTIM_STATE_RESET            = 0x00U,     
  HAL_LPTIM_STATE_READY            = 0x01U,     
  HAL_LPTIM_STATE_BUSY             = 0x02U,         
  HAL_LPTIM_STATE_TIMEOUT          = 0x03U,       
  HAL_LPTIM_STATE_ERROR            = 0x04U                                                                                   
}HAL_LPTIM_StateTypeDef;



  
typedef struct
{
      LPTIM_TypeDef              *Instance;          
      
      LPTIM_InitTypeDef           Init;              
  
      HAL_StatusTypeDef           Status;              
  
      HAL_LockTypeDef             Lock;              
  
   volatile  HAL_LPTIM_StateTypeDef   State;             
  
}LPTIM_HandleTypeDef;



  

 


 



 




 



 
#line 201 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_lptim.h"


  



 





 



 






 



 






 



 
#line 247 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_lptim.h"


 



 





 



 






 



 





 



 





 
 


 

#line 303 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_lptim.h"


 



 

#line 318 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_lptim.h"


 



 

 


 




 






 







 


 
    





 







 















 















 















 


 












 


    












 
    





 





 





 





 





 





 





 





 





 








 







 





 





 




 
   
 


 

 
HAL_StatusTypeDef HAL_LPTIM_Init(LPTIM_HandleTypeDef *hlptim);
HAL_StatusTypeDef HAL_LPTIM_DeInit(LPTIM_HandleTypeDef *hlptim);

 
void HAL_LPTIM_MspInit(LPTIM_HandleTypeDef *hlptim);
void HAL_LPTIM_MspDeInit(LPTIM_HandleTypeDef *hlptim);

 
 
 
HAL_StatusTypeDef HAL_LPTIM_PWM_Start(LPTIM_HandleTypeDef *hlptim, uint32_t Period, uint32_t Pulse);
HAL_StatusTypeDef HAL_LPTIM_PWM_Stop(LPTIM_HandleTypeDef *hlptim);
 
HAL_StatusTypeDef HAL_LPTIM_PWM_Start_IT(LPTIM_HandleTypeDef *hlptim, uint32_t Period, uint32_t Pulse);
HAL_StatusTypeDef HAL_LPTIM_PWM_Stop_IT(LPTIM_HandleTypeDef *hlptim);

 
 
HAL_StatusTypeDef HAL_LPTIM_OnePulse_Start(LPTIM_HandleTypeDef *hlptim, uint32_t Period, uint32_t Pulse);
HAL_StatusTypeDef HAL_LPTIM_OnePulse_Stop(LPTIM_HandleTypeDef *hlptim);
 
HAL_StatusTypeDef HAL_LPTIM_OnePulse_Start_IT(LPTIM_HandleTypeDef *hlptim, uint32_t Period, uint32_t Pulse);
HAL_StatusTypeDef HAL_LPTIM_OnePulse_Stop_IT(LPTIM_HandleTypeDef *hlptim);

 
 
HAL_StatusTypeDef HAL_LPTIM_SetOnce_Start(LPTIM_HandleTypeDef *hlptim, uint32_t Period, uint32_t Pulse);
HAL_StatusTypeDef HAL_LPTIM_SetOnce_Stop(LPTIM_HandleTypeDef *hlptim);
 
HAL_StatusTypeDef HAL_LPTIM_SetOnce_Start_IT(LPTIM_HandleTypeDef *hlptim, uint32_t Period, uint32_t Pulse);
HAL_StatusTypeDef HAL_LPTIM_SetOnce_Stop_IT(LPTIM_HandleTypeDef *hlptim);

 
 
HAL_StatusTypeDef HAL_LPTIM_Encoder_Start(LPTIM_HandleTypeDef *hlptim, uint32_t Period);
HAL_StatusTypeDef HAL_LPTIM_Encoder_Stop(LPTIM_HandleTypeDef *hlptim);
 
HAL_StatusTypeDef HAL_LPTIM_Encoder_Start_IT(LPTIM_HandleTypeDef *hlptim, uint32_t Period);
HAL_StatusTypeDef HAL_LPTIM_Encoder_Stop_IT(LPTIM_HandleTypeDef *hlptim);

 
 
HAL_StatusTypeDef HAL_LPTIM_TimeOut_Start(LPTIM_HandleTypeDef *hlptim, uint32_t Period, uint32_t Timeout);
HAL_StatusTypeDef HAL_LPTIM_TimeOut_Stop(LPTIM_HandleTypeDef *hlptim);
 
HAL_StatusTypeDef HAL_LPTIM_TimeOut_Start_IT(LPTIM_HandleTypeDef *hlptim, uint32_t Period, uint32_t Timeout);
HAL_StatusTypeDef HAL_LPTIM_TimeOut_Stop_IT(LPTIM_HandleTypeDef *hlptim);

 
 
HAL_StatusTypeDef HAL_LPTIM_Counter_Start(LPTIM_HandleTypeDef *hlptim, uint32_t Period);
HAL_StatusTypeDef HAL_LPTIM_Counter_Stop(LPTIM_HandleTypeDef *hlptim);
 
HAL_StatusTypeDef HAL_LPTIM_Counter_Start_IT(LPTIM_HandleTypeDef *hlptim, uint32_t Period);
HAL_StatusTypeDef HAL_LPTIM_Counter_Stop_IT(LPTIM_HandleTypeDef *hlptim);

 
uint32_t HAL_LPTIM_ReadCounter(LPTIM_HandleTypeDef *hlptim);
uint32_t HAL_LPTIM_ReadAutoReload(LPTIM_HandleTypeDef *hlptim);
uint32_t HAL_LPTIM_ReadCompare(LPTIM_HandleTypeDef *hlptim);

 
void HAL_LPTIM_IRQHandler(LPTIM_HandleTypeDef *hlptim);

 
void HAL_LPTIM_CompareMatchCallback(LPTIM_HandleTypeDef *hlptim);
void HAL_LPTIM_AutoReloadMatchCallback(LPTIM_HandleTypeDef *hlptim);
void HAL_LPTIM_TriggerCallback(LPTIM_HandleTypeDef *hlptim);
void HAL_LPTIM_CompareWriteCallback(LPTIM_HandleTypeDef *hlptim);
void HAL_LPTIM_AutoReloadWriteCallback(LPTIM_HandleTypeDef *hlptim);
void HAL_LPTIM_DirectionUpCallback(LPTIM_HandleTypeDef *hlptim);
void HAL_LPTIM_DirectionDownCallback(LPTIM_HandleTypeDef *hlptim);

 
HAL_LPTIM_StateTypeDef HAL_LPTIM_GetState(LPTIM_HandleTypeDef *hlptim);



 
  
 


 



  

 


 
  


  

 


 



  

 


 
  


													 
#line 668 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_lptim.h"



													 









#line 688 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_lptim.h"



















  






  

 


 
  


 
  


  



  
  






 
#line 335 "..\\USER\\stm32f7xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_ltdc.h"



































  

 









 
#line 50 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_ltdc.h"




 




 

 


 




 
typedef struct
{
  uint8_t Blue;                    
 

  uint8_t Green;                   
 

  uint8_t Red;                     
 

  uint8_t Reserved;                 
} LTDC_ColorTypeDef;



 
typedef struct
{
  uint32_t            HSPolarity;                
 

  uint32_t            VSPolarity;                
 

  uint32_t            DEPolarity;                
 

  uint32_t            PCPolarity;                
 

  uint32_t            HorizontalSync;            
 

  uint32_t            VerticalSync;              
 

  uint32_t            AccumulatedHBP;            
 

  uint32_t            AccumulatedVBP;            
 

  uint32_t            AccumulatedActiveW;        
 

  uint32_t            AccumulatedActiveH;        
 

  uint32_t            TotalWidth;                
 

  uint32_t            TotalHeigh;                
 

  LTDC_ColorTypeDef   Backcolor;                  
} LTDC_InitTypeDef;



 
typedef struct
{
  uint32_t WindowX0;                   
 

  uint32_t WindowX1;                   
 

  uint32_t WindowY0;                   
 

  uint32_t WindowY1;                   
 

  uint32_t PixelFormat;                
 

  uint32_t Alpha;                      
 

  uint32_t Alpha0;                     
 

  uint32_t BlendingFactor1;            
 

  uint32_t BlendingFactor2;            
 

  uint32_t FBStartAdress;               

  uint32_t ImageWidth;                 
 

  uint32_t ImageHeight;                
 

  LTDC_ColorTypeDef   Backcolor;        
} LTDC_LayerCfgTypeDef;



 
typedef enum
{
  HAL_LTDC_STATE_RESET             = 0x00U,     
  HAL_LTDC_STATE_READY             = 0x01U,     
  HAL_LTDC_STATE_BUSY              = 0x02U,     
  HAL_LTDC_STATE_TIMEOUT           = 0x03U,     
  HAL_LTDC_STATE_ERROR             = 0x04U      
}HAL_LTDC_StateTypeDef;



 
typedef struct
{
  LTDC_TypeDef                *Instance;                 

  LTDC_InitTypeDef            Init;                      

  LTDC_LayerCfgTypeDef        LayerCfg[2];       

  HAL_LockTypeDef             Lock;                      

  volatile HAL_LTDC_StateTypeDef  State;                     

  volatile uint32_t               ErrorCode;                 

} LTDC_HandleTypeDef;


 

 


 



 






 



 




 



 




 
  


 




 



 




 



 




 



 



 
      


 




 



 




 
      


 
#line 303 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_ltdc.h"


 



 



 



 







 



 






 
      


 






 



 




 



   

 


 




 






 






 








 








 






 


 










 












 












 












 












 



 






 


 


 
 
HAL_StatusTypeDef HAL_LTDC_Init(LTDC_HandleTypeDef *hltdc);
HAL_StatusTypeDef HAL_LTDC_DeInit(LTDC_HandleTypeDef *hltdc);
void HAL_LTDC_MspInit(LTDC_HandleTypeDef* hltdc);
void HAL_LTDC_MspDeInit(LTDC_HandleTypeDef* hltdc);
void HAL_LTDC_ErrorCallback(LTDC_HandleTypeDef *hltdc);
void HAL_LTDC_LineEventCallback(LTDC_HandleTypeDef *hltdc);
void HAL_LTDC_ReloadEventCallback(LTDC_HandleTypeDef *hltdc);


 



 
 
void  HAL_LTDC_IRQHandler(LTDC_HandleTypeDef *hltdc);


 



 
 
HAL_StatusTypeDef HAL_LTDC_ConfigLayer(LTDC_HandleTypeDef *hltdc, LTDC_LayerCfgTypeDef *pLayerCfg, uint32_t LayerIdx);
HAL_StatusTypeDef HAL_LTDC_SetWindowSize(LTDC_HandleTypeDef *hltdc, uint32_t XSize, uint32_t YSize, uint32_t LayerIdx);
HAL_StatusTypeDef HAL_LTDC_SetWindowPosition(LTDC_HandleTypeDef *hltdc, uint32_t X0, uint32_t Y0, uint32_t LayerIdx);
HAL_StatusTypeDef HAL_LTDC_SetPixelFormat(LTDC_HandleTypeDef *hltdc, uint32_t Pixelformat, uint32_t LayerIdx);
HAL_StatusTypeDef HAL_LTDC_SetAlpha(LTDC_HandleTypeDef *hltdc, uint32_t Alpha, uint32_t LayerIdx);
HAL_StatusTypeDef HAL_LTDC_SetAddress(LTDC_HandleTypeDef *hltdc, uint32_t Address, uint32_t LayerIdx);
HAL_StatusTypeDef HAL_LTDC_SetPitch(LTDC_HandleTypeDef *hltdc, uint32_t LinePitchInPixels, uint32_t LayerIdx);
HAL_StatusTypeDef HAL_LTDC_ConfigColorKeying(LTDC_HandleTypeDef *hltdc, uint32_t RGBValue, uint32_t LayerIdx);
HAL_StatusTypeDef HAL_LTDC_ConfigCLUT(LTDC_HandleTypeDef *hltdc, uint32_t *pCLUT, uint32_t CLUTSize, uint32_t LayerIdx);
HAL_StatusTypeDef HAL_LTDC_EnableColorKeying(LTDC_HandleTypeDef *hltdc, uint32_t LayerIdx);
HAL_StatusTypeDef HAL_LTDC_DisableColorKeying(LTDC_HandleTypeDef *hltdc, uint32_t LayerIdx);
HAL_StatusTypeDef HAL_LTDC_EnableCLUT(LTDC_HandleTypeDef *hltdc, uint32_t LayerIdx);
HAL_StatusTypeDef HAL_LTDC_DisableCLUT(LTDC_HandleTypeDef *hltdc, uint32_t LayerIdx);
HAL_StatusTypeDef HAL_LTDC_ProgramLineEvent(LTDC_HandleTypeDef *hltdc, uint32_t Line);
HAL_StatusTypeDef HAL_LTDC_EnableDither(LTDC_HandleTypeDef *hltdc);
HAL_StatusTypeDef HAL_LTDC_DisableDither(LTDC_HandleTypeDef *hltdc);
HAL_StatusTypeDef HAL_LTDC_Reload(LTDC_HandleTypeDef *hltdc, uint32_t ReloadType);
HAL_StatusTypeDef HAL_LTDC_ConfigLayer_NoReload(LTDC_HandleTypeDef *hltdc, LTDC_LayerCfgTypeDef *pLayerCfg, uint32_t LayerIdx);
HAL_StatusTypeDef HAL_LTDC_SetWindowSize_NoReload(LTDC_HandleTypeDef *hltdc, uint32_t XSize, uint32_t YSize, uint32_t LayerIdx);
HAL_StatusTypeDef HAL_LTDC_SetWindowPosition_NoReload(LTDC_HandleTypeDef *hltdc, uint32_t X0, uint32_t Y0, uint32_t LayerIdx);
HAL_StatusTypeDef HAL_LTDC_SetPixelFormat_NoReload(LTDC_HandleTypeDef *hltdc, uint32_t Pixelformat, uint32_t LayerIdx);
HAL_StatusTypeDef HAL_LTDC_SetAlpha_NoReload(LTDC_HandleTypeDef *hltdc, uint32_t Alpha, uint32_t LayerIdx);
HAL_StatusTypeDef HAL_LTDC_SetAddress_NoReload(LTDC_HandleTypeDef *hltdc, uint32_t Address, uint32_t LayerIdx);
HAL_StatusTypeDef HAL_LTDC_SetPitch_NoReload(LTDC_HandleTypeDef *hltdc, uint32_t LinePitchInPixels, uint32_t LayerIdx);
HAL_StatusTypeDef HAL_LTDC_ConfigColorKeying_NoReload(LTDC_HandleTypeDef *hltdc, uint32_t RGBValue, uint32_t LayerIdx);
HAL_StatusTypeDef HAL_LTDC_EnableColorKeying_NoReload(LTDC_HandleTypeDef *hltdc, uint32_t LayerIdx);
HAL_StatusTypeDef HAL_LTDC_DisableColorKeying_NoReload(LTDC_HandleTypeDef *hltdc, uint32_t LayerIdx);
HAL_StatusTypeDef HAL_LTDC_EnableCLUT_NoReload(LTDC_HandleTypeDef *hltdc, uint32_t LayerIdx);
HAL_StatusTypeDef HAL_LTDC_DisableCLUT_NoReload(LTDC_HandleTypeDef *hltdc, uint32_t LayerIdx);



 



 
 
HAL_LTDC_StateTypeDef HAL_LTDC_GetState(LTDC_HandleTypeDef *hltdc);
uint32_t              HAL_LTDC_GetError(LTDC_HandleTypeDef *hltdc);


 



 
 


 



  

 


 



  

 


 



  

 


 
#line 635 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_ltdc.h"


  

 


 



 



  



 







 
#line 339 "..\\USER\\stm32f7xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_pwr.h"



































  

 







 
#line 48 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_pwr.h"



 



  

 



 
   


 
typedef struct
{
  uint32_t PVDLevel;   
 

  uint32_t Mode;      
 
}PWR_PVDTypeDef;



 

 


 



  
#line 96 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_pwr.h"



    
 


 
#line 111 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_pwr.h"


 



 




 
    


 




 



 




 



 





 



 







 



  
  
 


 










 
#line 190 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_pwr.h"



















 






 





 





 





 





 





 





 





 






 






 





 





 





 





 




 

 
#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_pwr_ex.h"



































  

 







 
#line 48 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_pwr_ex.h"



 



  

  
 


 


 
#line 83 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_pwr_ex.h"



 
	


 




  
  


 





 
	


 
#line 116 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_pwr_ex.h"


 



  
  
 


 

 




 












 













 



 











 











 



 
 


 
 


 
uint32_t HAL_PWREx_GetVoltageRange(void);
HAL_StatusTypeDef HAL_PWREx_ControlVoltageScaling(uint32_t VoltageScaling);

void HAL_PWREx_EnableFlashPowerDown(void);
void HAL_PWREx_DisableFlashPowerDown(void); 
HAL_StatusTypeDef HAL_PWREx_EnableBkUpReg(void);
HAL_StatusTypeDef HAL_PWREx_DisableBkUpReg(void); 

void HAL_PWREx_EnableMainRegulatorLowVoltage(void);
void HAL_PWREx_DisableMainRegulatorLowVoltage(void);
void HAL_PWREx_EnableLowRegulatorLowVoltage(void);
void HAL_PWREx_DisableLowRegulatorLowVoltage(void);

HAL_StatusTypeDef HAL_PWREx_EnableOverDrive(void);
HAL_StatusTypeDef HAL_PWREx_DisableOverDrive(void);
HAL_StatusTypeDef HAL_PWREx_EnterUnderDriveSTOPMode(uint32_t Regulator, uint8_t STOPEntry);



 



 
 
 
 
 


 



 
#line 257 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_pwr_ex.h"


 



 



  



 
  







 
#line 305 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_pwr.h"

 


 
  


 
 
void HAL_PWR_DeInit(void);
void HAL_PWR_EnableBkUpAccess(void);
void HAL_PWR_DisableBkUpAccess(void);


 



 
 
 
void HAL_PWR_ConfigPVD(PWR_PVDTypeDef *sConfigPVD);
void HAL_PWR_EnablePVD(void);
void HAL_PWR_DisablePVD(void);

 
void HAL_PWR_EnableWakeUpPin(uint32_t WakeUpPinPolarity);
void HAL_PWR_DisableWakeUpPin(uint32_t WakeUpPinx);

 
void HAL_PWR_EnterSTOPMode(uint32_t Regulator, uint8_t STOPEntry);
void HAL_PWR_EnterSLEEPMode(uint32_t Regulator, uint8_t SLEEPEntry);
void HAL_PWR_EnterSTANDBYMode(void);

 
void HAL_PWR_PVD_IRQHandler(void);
void HAL_PWR_PVDCallback(void);

 
void HAL_PWR_EnableSleepOnExit(void);
void HAL_PWR_DisableSleepOnExit(void);
void HAL_PWR_EnableSEVOnPend(void);
void HAL_PWR_DisableSEVOnPend(void);


 



 

 
 
 


 



 



 



 
 


 



 
#line 398 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_pwr.h"



 



 



  



 
  







 
#line 343 "..\\USER\\stm32f7xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_qspi.h"



































 

 







 
#line 48 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_qspi.h"



 



  

  


 
  


 

typedef struct
{
  uint32_t ClockPrescaler;     
  
                                  
  uint32_t FifoThreshold;      
 
                                  
  uint32_t SampleShifting;     

 
                                  
  uint32_t FlashSize;          



 
                                  
  uint32_t ChipSelectHighTime; 

  
                                    
  uint32_t ClockMode;          
 
                                 
  uint32_t FlashID;            
 
                                 
  uint32_t DualFlash;          
                                                
}QSPI_InitTypeDef;



  
typedef enum
{
  HAL_QSPI_STATE_RESET             = 0x00U,     
  HAL_QSPI_STATE_READY             = 0x01U,     
  HAL_QSPI_STATE_BUSY              = 0x02U,      
  HAL_QSPI_STATE_BUSY_INDIRECT_TX  = 0x12U,      
  HAL_QSPI_STATE_BUSY_INDIRECT_RX  = 0x22U,     
  HAL_QSPI_STATE_BUSY_AUTO_POLLING = 0x42U,     
  HAL_QSPI_STATE_BUSY_MEM_MAPPED   = 0x82U,     
  HAL_QSPI_STATE_ABORT             = 0x08U,     
  HAL_QSPI_STATE_ERROR             = 0x04U      
}HAL_QSPI_StateTypeDef;



   
typedef struct
{
  QUADSPI_TypeDef            *Instance;         
  QSPI_InitTypeDef           Init;              
  uint8_t                    *pTxBuffPtr;       
  volatile uint16_t              TxXferSize;        
  volatile uint16_t              TxXferCount;       
  uint8_t                    *pRxBuffPtr;       
  volatile uint16_t              RxXferSize;        
  volatile uint16_t              RxXferCount;       
  DMA_HandleTypeDef          *hdma;             
  volatile HAL_LockTypeDef       Lock;              
  volatile HAL_QSPI_StateTypeDef State;             
  volatile uint32_t              ErrorCode;         
  uint32_t                   Timeout;            
}QSPI_HandleTypeDef;



 
typedef struct
{
  uint32_t Instruction;        
 
  uint32_t Address;            
 
  uint32_t AlternateBytes;     
 
  uint32_t AddressSize;        
 
  uint32_t AlternateBytesSize; 
 
  uint32_t DummyCycles;        
 
  uint32_t InstructionMode;    
 
  uint32_t AddressMode;        
 
  uint32_t AlternateByteMode;  
 
  uint32_t DataMode;           
 
  uint32_t NbData;             

 
  uint32_t DdrMode;            
 
  uint32_t DdrHoldHalfCycle;   

 
  uint32_t SIOOMode;          
 
}QSPI_CommandTypeDef;



 
typedef struct
{
  uint32_t Match;              
 
  uint32_t Mask;               
 
  uint32_t Interval;           
 
  uint32_t StatusBytesSize;    
 
  uint32_t MatchMode;          
 
  uint32_t AutomaticStop;      
 
}QSPI_AutoPollingTypeDef;
                           


 
typedef struct
{
  uint32_t TimeOutPeriod;      
 
  uint32_t TimeOutActivation;  
 
}QSPI_MemoryMappedTypeDef;                                     


 

 


 


  







  
  


 




  



 
#line 240 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_qspi.h"


 



 




 
  


 




   

  

 




  



 






   



 






 



 






 



 






   



 






   



 






   



 




 



 




 



 




 



 




   



 




   



 




   



 
#line 400 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_qspi.h"


 



   







 



  



   
    


 

 


 




 





  





 












 













 












 














 











 



 
  
 


 



 
 
HAL_StatusTypeDef     HAL_QSPI_Init     (QSPI_HandleTypeDef *hqspi);
HAL_StatusTypeDef     HAL_QSPI_DeInit   (QSPI_HandleTypeDef *hqspi);
void                  HAL_QSPI_MspInit  (QSPI_HandleTypeDef *hqspi);
void                  HAL_QSPI_MspDeInit(QSPI_HandleTypeDef *hqspi);


 



   
 
 
void                  HAL_QSPI_IRQHandler(QSPI_HandleTypeDef *hqspi);

 
HAL_StatusTypeDef     HAL_QSPI_Command      (QSPI_HandleTypeDef *hqspi, QSPI_CommandTypeDef *cmd, uint32_t Timeout);
HAL_StatusTypeDef     HAL_QSPI_Transmit     (QSPI_HandleTypeDef *hqspi, uint8_t *pData, uint32_t Timeout);
HAL_StatusTypeDef     HAL_QSPI_Receive      (QSPI_HandleTypeDef *hqspi, uint8_t *pData, uint32_t Timeout);
HAL_StatusTypeDef     HAL_QSPI_Command_IT   (QSPI_HandleTypeDef *hqspi, QSPI_CommandTypeDef *cmd);
HAL_StatusTypeDef     HAL_QSPI_Transmit_IT  (QSPI_HandleTypeDef *hqspi, uint8_t *pData);
HAL_StatusTypeDef     HAL_QSPI_Receive_IT   (QSPI_HandleTypeDef *hqspi, uint8_t *pData);
HAL_StatusTypeDef     HAL_QSPI_Transmit_DMA (QSPI_HandleTypeDef *hqspi, uint8_t *pData);
HAL_StatusTypeDef     HAL_QSPI_Receive_DMA  (QSPI_HandleTypeDef *hqspi, uint8_t *pData);

 
HAL_StatusTypeDef     HAL_QSPI_AutoPolling   (QSPI_HandleTypeDef *hqspi, QSPI_CommandTypeDef *cmd, QSPI_AutoPollingTypeDef *cfg, uint32_t Timeout);
HAL_StatusTypeDef     HAL_QSPI_AutoPolling_IT(QSPI_HandleTypeDef *hqspi, QSPI_CommandTypeDef *cmd, QSPI_AutoPollingTypeDef *cfg);

 
HAL_StatusTypeDef     HAL_QSPI_MemoryMapped(QSPI_HandleTypeDef *hqspi, QSPI_CommandTypeDef *cmd, QSPI_MemoryMappedTypeDef *cfg);


 



   
 
void                  HAL_QSPI_ErrorCallback        (QSPI_HandleTypeDef *hqspi);
void                  HAL_QSPI_AbortCpltCallback    (QSPI_HandleTypeDef *hqspi);
void                  HAL_QSPI_FifoThresholdCallback(QSPI_HandleTypeDef *hqspi);

 
void                  HAL_QSPI_CmdCpltCallback      (QSPI_HandleTypeDef *hqspi);
void                  HAL_QSPI_RxCpltCallback       (QSPI_HandleTypeDef *hqspi);
void                  HAL_QSPI_TxCpltCallback       (QSPI_HandleTypeDef *hqspi);
void                  HAL_QSPI_RxHalfCpltCallback   (QSPI_HandleTypeDef *hqspi);
void                  HAL_QSPI_TxHalfCpltCallback   (QSPI_HandleTypeDef *hqspi);

 
void                  HAL_QSPI_StatusMatchCallback  (QSPI_HandleTypeDef *hqspi);

 
void                  HAL_QSPI_TimeOutCallback      (QSPI_HandleTypeDef *hqspi);


 



   
 
HAL_QSPI_StateTypeDef HAL_QSPI_GetState        (QSPI_HandleTypeDef *hqspi);
uint32_t              HAL_QSPI_GetError        (QSPI_HandleTypeDef *hqspi);
HAL_StatusTypeDef     HAL_QSPI_Abort           (QSPI_HandleTypeDef *hqspi);
HAL_StatusTypeDef     HAL_QSPI_Abort_IT        (QSPI_HandleTypeDef *hqspi);
void                  HAL_QSPI_SetTimeout      (QSPI_HandleTypeDef *hqspi, uint32_t Timeout);
HAL_StatusTypeDef     HAL_QSPI_SetFifoThreshold(QSPI_HandleTypeDef *hqspi, uint32_t Threshold);
uint32_t              HAL_QSPI_GetFifoThreshold(QSPI_HandleTypeDef *hqspi);


 



 

 


 


  



 



 



 
  





 



 
  
#line 647 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_qspi.h"






                                  


                                          
  


 



  














 



 
































 



 



 



 


                                             








 



 

#line 752 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_qspi.h"




 

 


 



 



  



  
  






 
#line 347 "..\\USER\\stm32f7xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rng.h"



































 

 







 
#line 48 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rng.h"



 




 

  



 



 
typedef enum
{
  HAL_RNG_STATE_RESET     = 0x00U,   
  HAL_RNG_STATE_READY     = 0x01U,   
  HAL_RNG_STATE_BUSY      = 0x02U,    
  HAL_RNG_STATE_TIMEOUT   = 0x03U,   
  HAL_RNG_STATE_ERROR     = 0x04U    
    
}HAL_RNG_StateTypeDef;



 



  
typedef struct
{
  RNG_TypeDef                 *Instance;     

  uint32_t                    RandomNumber;  	
  
  HAL_LockTypeDef             Lock;          
  
  volatile HAL_RNG_StateTypeDef   State;         
  
}RNG_HandleTypeDef;



 



  
   
 



 



 





 



 






 



  
  
 



 




 






 






 











 









 








 

    




 











 











 




  

 


 



   
HAL_StatusTypeDef HAL_RNG_Init(RNG_HandleTypeDef *hrng);
HAL_StatusTypeDef HAL_RNG_DeInit (RNG_HandleTypeDef *hrng);
void HAL_RNG_MspInit(RNG_HandleTypeDef *hrng);
void HAL_RNG_MspDeInit(RNG_HandleTypeDef *hrng);



  



 
uint32_t HAL_RNG_GetRandomNumber(RNG_HandleTypeDef *hrng);     
uint32_t HAL_RNG_GetRandomNumber_IT(RNG_HandleTypeDef *hrng);  

HAL_StatusTypeDef HAL_RNG_GenerateRandomNumber(RNG_HandleTypeDef *hrng, uint32_t *random32bit);
HAL_StatusTypeDef HAL_RNG_GenerateRandomNumber_IT(RNG_HandleTypeDef *hrng);
uint32_t HAL_RNG_ReadLastRandomNumber(RNG_HandleTypeDef *hrng);

void HAL_RNG_IRQHandler(RNG_HandleTypeDef *hrng);
void HAL_RNG_ErrorCallback(RNG_HandleTypeDef *hrng);
void HAL_RNG_ReadyDataCallback(RNG_HandleTypeDef* hrng, uint32_t random32bit);



  



 
HAL_RNG_StateTypeDef HAL_RNG_GetState(RNG_HandleTypeDef *hrng);



 
  


  

 


 



  

 


 



  
          
 


 



  

 


 



  

 


 









  

 


 



 

 


 



 



  



  







 
#line 351 "..\\USER\\stm32f7xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rtc.h"



































  

 







 
#line 48 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rtc.h"



 



  

 


 



  
typedef enum
{
  HAL_RTC_STATE_RESET             = 0x00U,   
  HAL_RTC_STATE_READY             = 0x01U,   
  HAL_RTC_STATE_BUSY              = 0x02U,        
  HAL_RTC_STATE_TIMEOUT           = 0x03U,     
  HAL_RTC_STATE_ERROR             = 0x04U          
                                                                        
}HAL_RTCStateTypeDef;



 
typedef struct
{
  uint32_t HourFormat;      
          

  uint32_t AsynchPrediv;    
         
                               
  uint32_t SynchPrediv;     
    
  
  uint32_t OutPut;          
       
  
  uint32_t OutPutPolarity;  
  
  
  uint32_t OutPutType;      
              
}RTC_InitTypeDef;



 
typedef struct
{
  uint8_t Hours;            

 

  uint8_t Minutes;          
 
  
  uint8_t Seconds;          
 
  
  uint32_t SubSeconds;      

 
  
  uint32_t SecondFraction;  



 

  uint8_t TimeFormat;       
  
  
  uint32_t DayLightSaving;  
 
  
  uint32_t StoreOperation;  

 
}RTC_TimeTypeDef; 
  


 
typedef struct
{
  uint8_t WeekDay;  
 
  
  uint8_t Month;    
 

  uint8_t Date;     
 
  
  uint8_t Year;     
 
                        
}RTC_DateTypeDef;



 
typedef struct
{
  RTC_TimeTypeDef AlarmTime;      
    
  uint32_t AlarmMask;            
 
  
  uint32_t AlarmSubSecondMask;   
                                    

  uint32_t AlarmDateWeekDaySel;  
 
  
  uint8_t AlarmDateWeekDay;      

 
                                                                     
  uint32_t Alarm;                
                             
}RTC_AlarmTypeDef;



  
typedef struct
{
  RTC_TypeDef                 *Instance;   
   
  RTC_InitTypeDef             Init;         
  
  HAL_LockTypeDef             Lock;        
  
  volatile HAL_RTCStateTypeDef    State;       
    
}RTC_HandleTypeDef;



 

 


 
 


  




  




  




  



  




 



  




  



  





 



  




 



  




  



 
 
#line 283 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rtc.h"


  



    
#line 297 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rtc.h"


                                  



  




  



  
#line 319 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rtc.h"


  



  




 



 
#line 368 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rtc.h"


    



  
#line 383 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rtc.h"


 



  
#line 407 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rtc.h"


 



  
  
 


 




 






 










 




 




 






 






 






 










    










 










 












 










 

                                       








 





 





 





 





 





 





 





 





 





 





 





 





 





 



 

 
#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rtc_ex.h"



































  

 







 
#line 48 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rtc_ex.h"



 



  

  


 



 
typedef struct 
{
  uint32_t Tamper;                      
 
  
  uint32_t Interrupt;                   
                                   
                                             
  uint32_t Trigger;                     
 
                                             
  uint32_t NoErase;                     
 

  uint32_t MaskFlag;                     
 

  uint32_t Filter;                      
 
  
  uint32_t SamplingFrequency;           
 
                                      
  uint32_t PrechargeDuration;           
  
 
  uint32_t TamperPullUp;                
            
 
  uint32_t TimeStampOnTamperDetection;  
                       
}RTC_TamperTypeDef;


 

 


 



  






  
  


 
#line 152 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rtc_ex.h"


  



  




 
  


  





 



 






 



  





  



  






   

  

 




 



 




 
  


  


#line 236 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rtc_ex.h"


 



 
#line 259 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rtc_ex.h"


 



  
#line 274 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rtc_ex.h"


 
  


  




 
  


  




 



  
#line 305 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rtc_ex.h"


  



  
#line 318 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rtc_ex.h"


  



  







 



  




 

 

  




  
  


  
  
 


 





 






 









 









 









 









 










 









 






 






 






 






 






 






 












 














 











 











 






 






 









 









 









 









 










 










 






 






 









 









 






 






 






 






 









 





 





 





 





 





 





 





 





 





 






 





 





 





 





 





 





 





 





 





 





 





 





 






 





 





 





 




 

 


 



 

 
HAL_StatusTypeDef HAL_RTCEx_SetTimeStamp(RTC_HandleTypeDef *hrtc, uint32_t TimeStampEdge, uint32_t RTC_TimeStampPin);
HAL_StatusTypeDef HAL_RTCEx_SetTimeStamp_IT(RTC_HandleTypeDef *hrtc, uint32_t TimeStampEdge, uint32_t RTC_TimeStampPin);
HAL_StatusTypeDef HAL_RTCEx_DeactivateTimeStamp(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_SetInternalTimeStamp(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_DeactivateInternalTimeStamp(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_GetTimeStamp(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTimeStamp, RTC_DateTypeDef *sTimeStampDate, uint32_t Format);

HAL_StatusTypeDef HAL_RTCEx_SetTamper(RTC_HandleTypeDef *hrtc, RTC_TamperTypeDef* sTamper);
HAL_StatusTypeDef HAL_RTCEx_SetTamper_IT(RTC_HandleTypeDef *hrtc, RTC_TamperTypeDef* sTamper);
HAL_StatusTypeDef HAL_RTCEx_DeactivateTamper(RTC_HandleTypeDef *hrtc, uint32_t Tamper);
void              HAL_RTCEx_TamperTimeStampIRQHandler(RTC_HandleTypeDef *hrtc);

void              HAL_RTCEx_Tamper1EventCallback(RTC_HandleTypeDef *hrtc);
void              HAL_RTCEx_Tamper2EventCallback(RTC_HandleTypeDef *hrtc);
void              HAL_RTCEx_Tamper3EventCallback(RTC_HandleTypeDef *hrtc);
void              HAL_RTCEx_TimeStampEventCallback(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_PollForTimeStampEvent(RTC_HandleTypeDef *hrtc, uint32_t Timeout);
HAL_StatusTypeDef HAL_RTCEx_PollForTamper1Event(RTC_HandleTypeDef *hrtc, uint32_t Timeout);
HAL_StatusTypeDef HAL_RTCEx_PollForTamper2Event(RTC_HandleTypeDef *hrtc, uint32_t Timeout);
HAL_StatusTypeDef HAL_RTCEx_PollForTamper3Event(RTC_HandleTypeDef *hrtc, uint32_t Timeout);


 



 
 
HAL_StatusTypeDef HAL_RTCEx_SetWakeUpTimer(RTC_HandleTypeDef *hrtc, uint32_t WakeUpCounter, uint32_t WakeUpClock);
HAL_StatusTypeDef HAL_RTCEx_SetWakeUpTimer_IT(RTC_HandleTypeDef *hrtc, uint32_t WakeUpCounter, uint32_t WakeUpClock);
uint32_t          HAL_RTCEx_DeactivateWakeUpTimer(RTC_HandleTypeDef *hrtc);
uint32_t          HAL_RTCEx_GetWakeUpTimer(RTC_HandleTypeDef *hrtc);
void              HAL_RTCEx_WakeUpTimerIRQHandler(RTC_HandleTypeDef *hrtc);
void              HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_PollForWakeUpTimerEvent(RTC_HandleTypeDef *hrtc, uint32_t Timeout);


 



 
 
void              HAL_RTCEx_BKUPWrite(RTC_HandleTypeDef *hrtc, uint32_t BackupRegister, uint32_t Data);
uint32_t          HAL_RTCEx_BKUPRead(RTC_HandleTypeDef *hrtc, uint32_t BackupRegister);

HAL_StatusTypeDef HAL_RTCEx_SetSmoothCalib(RTC_HandleTypeDef *hrtc, uint32_t SmoothCalibPeriod, uint32_t SmoothCalibPlusPulses, uint32_t SmouthCalibMinusPulsesValue);
HAL_StatusTypeDef HAL_RTCEx_SetSynchroShift(RTC_HandleTypeDef *hrtc, uint32_t ShiftAdd1S, uint32_t ShiftSubFS);
HAL_StatusTypeDef HAL_RTCEx_SetCalibrationOutPut(RTC_HandleTypeDef *hrtc, uint32_t CalibOutput);
HAL_StatusTypeDef HAL_RTCEx_DeactivateCalibrationOutPut(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_SetRefClock(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_DeactivateRefClock(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_EnableBypassShadow(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTCEx_DisableBypassShadow(RTC_HandleTypeDef *hrtc);


 



 
 
void              HAL_RTCEx_AlarmBEventCallback(RTC_HandleTypeDef *hrtc); 
HAL_StatusTypeDef HAL_RTCEx_PollForAlarmBEvent(RTC_HandleTypeDef *hrtc, uint32_t Timeout);


  



  
  
 
 


 




 

 


 
 




 
  
    


 



 
#line 960 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rtc_ex.h"



#line 1000 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rtc_ex.h"

#line 1013 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rtc_ex.h"


 



 



  



 







 
#line 626 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rtc.h"

 


 



 
 
HAL_StatusTypeDef HAL_RTC_Init(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef HAL_RTC_DeInit(RTC_HandleTypeDef *hrtc);
void       HAL_RTC_MspInit(RTC_HandleTypeDef *hrtc);
void       HAL_RTC_MspDeInit(RTC_HandleTypeDef *hrtc);


 



 
 
HAL_StatusTypeDef HAL_RTC_SetTime(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTime, uint32_t Format);
HAL_StatusTypeDef HAL_RTC_GetTime(RTC_HandleTypeDef *hrtc, RTC_TimeTypeDef *sTime, uint32_t Format);
HAL_StatusTypeDef HAL_RTC_SetDate(RTC_HandleTypeDef *hrtc, RTC_DateTypeDef *sDate, uint32_t Format);
HAL_StatusTypeDef HAL_RTC_GetDate(RTC_HandleTypeDef *hrtc, RTC_DateTypeDef *sDate, uint32_t Format);


 



 
 
HAL_StatusTypeDef HAL_RTC_SetAlarm(RTC_HandleTypeDef *hrtc, RTC_AlarmTypeDef *sAlarm, uint32_t Format);
HAL_StatusTypeDef HAL_RTC_SetAlarm_IT(RTC_HandleTypeDef *hrtc, RTC_AlarmTypeDef *sAlarm, uint32_t Format);
HAL_StatusTypeDef HAL_RTC_DeactivateAlarm(RTC_HandleTypeDef *hrtc, uint32_t Alarm);
HAL_StatusTypeDef HAL_RTC_GetAlarm(RTC_HandleTypeDef *hrtc, RTC_AlarmTypeDef *sAlarm, uint32_t Alarm, uint32_t Format);
void                HAL_RTC_AlarmIRQHandler(RTC_HandleTypeDef *hrtc);
HAL_StatusTypeDef   HAL_RTC_PollForAlarmAEvent(RTC_HandleTypeDef *hrtc, uint32_t Timeout);
void         HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc);


 



 
 
HAL_StatusTypeDef   HAL_RTC_WaitForSynchro(RTC_HandleTypeDef* hrtc);


 



 
 
HAL_RTCStateTypeDef HAL_RTC_GetState(RTC_HandleTypeDef *hrtc);


 



 

 
 
 


 
 










 

 


 



 
#line 749 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rtc.h"

#line 779 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_rtc.h"



 



 

 


 
HAL_StatusTypeDef  RTC_EnterInitMode(RTC_HandleTypeDef* hrtc);
uint8_t            RTC_ByteToBcd2(uint8_t Value);
uint8_t            RTC_Bcd2ToByte(uint8_t Value);


 



 



  
  






 
#line 355 "..\\USER\\stm32f7xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_sai.h"



































 

 








 
#line 49 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_sai.h"



 



 

 


 



 
typedef enum
{
  HAL_SAI_STATE_RESET    = 0x00U,   
  HAL_SAI_STATE_READY    = 0x01U,   
  HAL_SAI_STATE_BUSY     = 0x02U,   
  HAL_SAI_STATE_BUSY_TX  = 0x12U,   
  HAL_SAI_STATE_BUSY_RX  = 0x22U,   
}HAL_SAI_StateTypeDef;



 
typedef void (*SAIcallback)(void);




 
typedef struct
{
  uint32_t AudioMode;           
 

  uint32_t Synchro;             
 

  uint32_t SynchroExt;          



 

  uint32_t OutputDrive;         


 

  uint32_t NoDivider;           






 

  uint32_t FIFOThreshold;       
 

  uint32_t AudioFrequency;      
 

  uint32_t Mckdiv;              

 

  uint32_t MonoStereoMode;      
 

  uint32_t CompandingMode;      
 

  uint32_t TriState;            
 

  
 

  uint32_t Protocol;        
 

  uint32_t DataSize;        
 

  uint32_t FirstBit;        
 

  uint32_t ClockStrobing;   
 
}SAI_InitTypeDef;


 




 
typedef struct
{

  uint32_t FrameLength;        



 

  uint32_t ActiveFrameLength;  


 

  uint32_t FSDefinition;       
 

  uint32_t FSPolarity;         
 

  uint32_t FSOffset;           
 

}SAI_FrameInitTypeDef;


 




 
typedef struct
{
  uint32_t FirstBitOffset;  
 

  uint32_t SlotSize;        
 

  uint32_t SlotNumber;      
 

  uint32_t SlotActive;      
 
}SAI_SlotInitTypeDef;


 




 
typedef struct __SAI_HandleTypeDef
{
  SAI_Block_TypeDef         *Instance;     

  SAI_InitTypeDef           Init;          

  SAI_FrameInitTypeDef      FrameInit;     

  SAI_SlotInitTypeDef       SlotInit;      

  uint8_t                  *pBuffPtr;      

  uint16_t                  XferSize;      

  uint16_t                  XferCount;     

  DMA_HandleTypeDef         *hdmatx;       

  DMA_HandleTypeDef         *hdmarx;       

  SAIcallback               mutecallback;  

  void (*InterruptServiceRoutine)(struct __SAI_HandleTypeDef *hsai);  

  HAL_LockTypeDef           Lock;          

  volatile HAL_SAI_StateTypeDef State;         

  volatile uint32_t             ErrorCode;     
}SAI_HandleTypeDef;


 



 

 



 



 
#line 264 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_sai.h"


 



 





 



 







 



 






 



 
#line 314 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_sai.h"


 



 







 



 





 



 
#line 349 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_sai.h"


 



 




 



 




 



 






 



 




 



 




 
  



 




 



 




 



 




 
  

  

 





 



 
#line 460 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_sai.h"


 



 




 



 




 



 







 



 







 



 




 



 
#line 525 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_sai.h"


 



 
#line 539 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_sai.h"


 



 
#line 552 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_sai.h"


 



 

 




 




 














 















 














 















 





 

 

 



 

 



 
HAL_StatusTypeDef HAL_SAI_InitProtocol(SAI_HandleTypeDef *hsai, uint32_t protocol, uint32_t datasize, uint32_t nbslot);
HAL_StatusTypeDef HAL_SAI_Init(SAI_HandleTypeDef *hsai);
HAL_StatusTypeDef HAL_SAI_DeInit (SAI_HandleTypeDef *hsai);
void HAL_SAI_MspInit(SAI_HandleTypeDef *hsai);
void HAL_SAI_MspDeInit(SAI_HandleTypeDef *hsai);



 

 



 
 
HAL_StatusTypeDef HAL_SAI_Transmit(SAI_HandleTypeDef *hsai, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SAI_Receive(SAI_HandleTypeDef *hsai, uint8_t *pData, uint16_t Size, uint32_t Timeout);

 
HAL_StatusTypeDef HAL_SAI_Transmit_IT(SAI_HandleTypeDef *hsai, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SAI_Receive_IT(SAI_HandleTypeDef *hsai, uint8_t *pData, uint16_t Size);

 
HAL_StatusTypeDef HAL_SAI_Transmit_DMA(SAI_HandleTypeDef *hsai, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SAI_Receive_DMA(SAI_HandleTypeDef *hsai, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SAI_DMAPause(SAI_HandleTypeDef *hsai);
HAL_StatusTypeDef HAL_SAI_DMAResume(SAI_HandleTypeDef *hsai);
HAL_StatusTypeDef HAL_SAI_DMAStop(SAI_HandleTypeDef *hsai);

 
HAL_StatusTypeDef HAL_SAI_Abort(SAI_HandleTypeDef *hsai);

 
HAL_StatusTypeDef HAL_SAI_EnableTxMuteMode(SAI_HandleTypeDef *hsai, uint16_t val);
HAL_StatusTypeDef HAL_SAI_DisableTxMuteMode(SAI_HandleTypeDef *hsai);
HAL_StatusTypeDef HAL_SAI_EnableRxMuteMode(SAI_HandleTypeDef *hsai, SAIcallback callback, uint16_t counter);
HAL_StatusTypeDef HAL_SAI_DisableRxMuteMode(SAI_HandleTypeDef *hsai);

 
void HAL_SAI_IRQHandler(SAI_HandleTypeDef *hsai);
void HAL_SAI_TxHalfCpltCallback(SAI_HandleTypeDef *hsai);
void HAL_SAI_TxCpltCallback(SAI_HandleTypeDef *hsai);
void HAL_SAI_RxHalfCpltCallback(SAI_HandleTypeDef *hsai);
void HAL_SAI_RxCpltCallback(SAI_HandleTypeDef *hsai);
void HAL_SAI_ErrorCallback(SAI_HandleTypeDef *hsai);


 



 
 
HAL_SAI_StateTypeDef HAL_SAI_GetState(SAI_HandleTypeDef *hsai);
uint32_t HAL_SAI_GetError(SAI_HandleTypeDef *hsai);


 



 

 


 






























#line 757 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_sai.h"




































































 

 


 



 



 



 







 
#line 359 "..\\USER\\stm32f7xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_sd.h"



































  

 







 
#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_ll_sdmmc.h"



































  

 







 
#line 48 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_ll_sdmmc.h"



 



  

  


 
  


 
typedef struct
{
  uint32_t ClockEdge;            
 

  uint32_t ClockBypass;          

 

  uint32_t ClockPowerSave;       

 

  uint32_t BusWide;              
 

  uint32_t HardwareFlowControl;  
 

  uint32_t ClockDiv;             
   
  
}SDMMC_InitTypeDef;
  



 
typedef struct                                                                                            
{
  uint32_t Argument;            


 

  uint32_t CmdIndex;            
 

  uint32_t Response;            
 

  uint32_t WaitForInterrupt;    

 

  uint32_t CPSM;                

 
}SDMMC_CmdInitTypeDef;




 
typedef struct
{
  uint32_t DataTimeOut;          

  uint32_t DataLength;           
 
  uint32_t DataBlockSize;       
 
 
  uint32_t TransferDir;         

 
 
  uint32_t TransferMode;        
 
 
  uint32_t DPSM;                

 
}SDMMC_DataInitTypeDef;



 
  
 


 



 







 



 







  



 







 



 









 



 







 
  


 



   
    


 



 



 









 



 









 



 







   



 











 



 



 



 
#line 309 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_ll_sdmmc.h"

#line 325 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_ll_sdmmc.h"


 



 







 



 







 



 







 
  


 







   



 
#line 402 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_ll_sdmmc.h"


  



 
#line 431 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_ll_sdmmc.h"


 



 
  
 


 
  



 
 
 
  




 
 



 
 




 


 




 




 
 




  






 






  





 

 




























 






























 






























 




















 






























 


















 






   






   






   






   






   






   






   






   

      


 



   

 


 
  
 


 
HAL_StatusTypeDef SDMMC_Init(SDMMC_TypeDef *SDMMCx, SDMMC_InitTypeDef Init);


 
  
 


 
 
uint32_t          SDMMC_ReadFIFO(SDMMC_TypeDef *SDMMCx);
HAL_StatusTypeDef SDMMC_WriteFIFO(SDMMC_TypeDef *SDMMCx, uint32_t *pWriteData);


 
  
 


 
HAL_StatusTypeDef SDMMC_PowerState_ON(SDMMC_TypeDef *SDMMCx);
HAL_StatusTypeDef SDMMC_PowerState_OFF(SDMMC_TypeDef *SDMMCx);
uint32_t          SDMMC_GetPowerState(SDMMC_TypeDef *SDMMCx);

 
HAL_StatusTypeDef SDMMC_SendCommand(SDMMC_TypeDef *SDMMCx, SDMMC_CmdInitTypeDef *Command);
uint8_t           SDMMC_GetCommandResponse(SDMMC_TypeDef *SDMMCx);
uint32_t          SDMMC_GetResponse(SDMMC_TypeDef *SDMMCx, uint32_t Response);

 
HAL_StatusTypeDef SDMMC_DataConfig(SDMMC_TypeDef *SDMMCx, SDMMC_DataInitTypeDef* Data);
uint32_t          SDMMC_GetDataCounter(SDMMC_TypeDef *SDMMCx);
uint32_t          SDMMC_GetFIFOCount(SDMMC_TypeDef *SDMMCx);

 
HAL_StatusTypeDef SDMMC_SetSDMMCReadWaitMode(SDMMC_TypeDef *SDMMCx, uint32_t SDMMC_ReadWaitMode);



 
  


 
  


  



 







 
#line 48 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_sd.h"



 




  

  


 



 



typedef struct
{
  SDMMC_TypeDef                   *Instance;         
  
  SDMMC_InitTypeDef               Init;              
  
  HAL_LockTypeDef              Lock;              
  
  uint32_t                     CardType;          
  
  uint32_t                     RCA;               
  
  uint32_t                     CSD[4];            
  
  uint32_t                     CID[4];            
  
  volatile uint32_t                SdTransferCplt;    
  
  volatile uint32_t                SdTransferErr;     
  
  volatile uint32_t                DmaTransferCplt;   
  
  volatile uint32_t                SdOperation;       
  
  DMA_HandleTypeDef            *hdmarx;           
  
  DMA_HandleTypeDef            *hdmatx;           
  
}SD_HandleTypeDef;


 



  
typedef struct
{
  volatile uint8_t  CSDStruct;             
  volatile uint8_t  SysSpecVersion;        
  volatile uint8_t  Reserved1;             
  volatile uint8_t  TAAC;                  
  volatile uint8_t  NSAC;                  
  volatile uint8_t  MaxBusClkFrec;         
  volatile uint16_t CardComdClasses;       
  volatile uint8_t  RdBlockLen;            
  volatile uint8_t  PartBlockRead;         
  volatile uint8_t  WrBlockMisalign;       
  volatile uint8_t  RdBlockMisalign;       
  volatile uint8_t  DSRImpl;               
  volatile uint8_t  Reserved2;             
  volatile uint32_t DeviceSize;            
  volatile uint8_t  MaxRdCurrentVDDMin;    
  volatile uint8_t  MaxRdCurrentVDDMax;    
  volatile uint8_t  MaxWrCurrentVDDMin;    
  volatile uint8_t  MaxWrCurrentVDDMax;    
  volatile uint8_t  DeviceSizeMul;         
  volatile uint8_t  EraseGrSize;           
  volatile uint8_t  EraseGrMul;            
  volatile uint8_t  WrProtectGrSize;       
  volatile uint8_t  WrProtectGrEnable;     
  volatile uint8_t  ManDeflECC;            
  volatile uint8_t  WrSpeedFact;           
  volatile uint8_t  MaxWrBlockLen;         
  volatile uint8_t  WriteBlockPaPartial;   
  volatile uint8_t  Reserved3;             
  volatile uint8_t  ContentProtectAppli;   
  volatile uint8_t  FileFormatGrouop;      
  volatile uint8_t  CopyFlag;              
  volatile uint8_t  PermWrProtect;         
  volatile uint8_t  TempWrProtect;         
  volatile uint8_t  FileFormat;            
  volatile uint8_t  ECC;                   
  volatile uint8_t  CSD_CRC;               
  volatile uint8_t  Reserved4;             

}HAL_SD_CSDTypedef;


 



 
typedef struct
{
  volatile uint8_t  ManufacturerID;   
  volatile uint16_t OEM_AppliID;      
  volatile uint32_t ProdName1;        
  volatile uint8_t  ProdName2;        
  volatile uint8_t  ProdRev;          
  volatile uint32_t ProdSN;           
  volatile uint8_t  Reserved1;        
  volatile uint16_t ManufactDate;     
  volatile uint8_t  CID_CRC;          
  volatile uint8_t  Reserved2;        

}HAL_SD_CIDTypedef;


 



 
typedef struct
{
  volatile uint8_t  DAT_BUS_WIDTH;            
  volatile uint8_t  SECURED_MODE;             
  volatile uint16_t SD_CARD_TYPE;             
  volatile uint32_t SIZE_OF_PROTECTED_AREA;   
  volatile uint8_t  SPEED_CLASS;              
  volatile uint8_t  PERFORMANCE_MOVE;         
  volatile uint8_t  AU_SIZE;                  
  volatile uint16_t ERASE_SIZE;               
  volatile uint8_t  ERASE_TIMEOUT;            
  volatile uint8_t  ERASE_OFFSET;             

}HAL_SD_CardStatusTypedef;


 



 
typedef struct
{
  HAL_SD_CSDTypedef   SD_csd;          
  HAL_SD_CIDTypedef   SD_cid;          
  uint64_t            CardCapacity;    
  uint32_t            CardBlockSize;   
  uint16_t            RCA;             
  uint8_t             CardType;        

}HAL_SD_CardInfoTypedef;


 



 
typedef enum
{


    
  SD_CMD_CRC_FAIL                    = (1),    
  SD_DATA_CRC_FAIL                   = (2),    
  SD_CMD_RSP_TIMEOUT                 = (3),    
  SD_DATA_TIMEOUT                    = (4),    
  SD_TX_UNDERRUN                     = (5),    
  SD_RX_OVERRUN                      = (6),    
  SD_START_BIT_ERR                   = (7),    
  SD_CMD_OUT_OF_RANGE                = (8),    
  SD_ADDR_MISALIGNED                 = (9),    
  SD_BLOCK_LEN_ERR                   = (10),   
  SD_ERASE_SEQ_ERR                   = (11),   
  SD_BAD_ERASE_PARAM                 = (12),   
  SD_WRITE_PROT_VIOLATION            = (13),   
  SD_LOCK_UNLOCK_FAILED              = (14),   
  SD_COM_CRC_FAILED                  = (15),   
  SD_ILLEGAL_CMD                     = (16),   
  SD_CARD_ECC_FAILED                 = (17),   
  SD_CC_ERROR                        = (18),   
  SD_GENERAL_UNKNOWN_ERROR           = (19),   
  SD_STREAM_READ_UNDERRUN            = (20),   
  SD_STREAM_WRITE_OVERRUN            = (21),   
  SD_CID_CSD_OVERWRITE               = (22),   
  SD_WP_ERASE_SKIP                   = (23),   
  SD_CARD_ECC_DISABLED               = (24),   
  SD_ERASE_RESET                     = (25),   
  SD_AKE_SEQ_ERROR                   = (26),   
  SD_INVALID_VOLTRANGE               = (27),
  SD_ADDR_OUT_OF_RANGE               = (28),
  SD_SWITCH_ERROR                    = (29),
  SD_SDMMC_DISABLED                  = (30),
  SD_SDMMC_FUNCTION_BUSY             = (31),
  SD_SDMMC_FUNCTION_FAILED           = (32),
  SD_SDMMC_UNKNOWN_FUNCTION          = (33),



  
  SD_INTERNAL_ERROR                  = (34),
  SD_NOT_CONFIGURED                  = (35),
  SD_REQUEST_PENDING                 = (36),
  SD_REQUEST_NOT_APPLICABLE          = (37),
  SD_INVALID_PARAMETER               = (38),
  SD_UNSUPPORTED_FEATURE             = (39),
  SD_UNSUPPORTED_HW                  = (40),
  SD_ERROR                           = (41),
  SD_OK                              = (0) 

}HAL_SD_ErrorTypedef;


 



    
typedef enum
{
  SD_TRANSFER_OK    = 0,   
  SD_TRANSFER_BUSY  = 1,   
  SD_TRANSFER_ERROR = 2    

}HAL_SD_TransferStateTypedef;


 



    
typedef enum
{
  SD_CARD_READY                  = ((uint32_t)0x00000001U),   
  SD_CARD_IDENTIFICATION         = ((uint32_t)0x00000002U),   
  SD_CARD_STANDBY                = ((uint32_t)0x00000003U),   
  SD_CARD_TRANSFER               = ((uint32_t)0x00000004U),     
  SD_CARD_SENDING                = ((uint32_t)0x00000005U),   
  SD_CARD_RECEIVING              = ((uint32_t)0x00000006U),   
  SD_CARD_PROGRAMMING            = ((uint32_t)0x00000007U),   
  SD_CARD_DISCONNECTED           = ((uint32_t)0x00000008U),   
  SD_CARD_ERROR                  = ((uint32_t)0x000000FFU)    

}HAL_SD_CardStateTypedef;


 



    
typedef enum
{
  SD_READ_SINGLE_BLOCK    = 0U,   
  SD_READ_MULTIPLE_BLOCK  = 1U,   
  SD_WRITE_SINGLE_BLOCK   = 2U,   
  SD_WRITE_MULTIPLE_BLOCK = 3U    

}HAL_SD_OperationTypedef;


 



 

 


 



 
#line 382 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_sd.h"




 
#line 398 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_sd.h"




 
#line 414 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_sd.h"



 
#line 426 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_sd.h"


 
  
 



 
 



  





 





  





 

 




























 






























 






























 



















 






























 


















 



 
  
 


 
  


 
HAL_SD_ErrorTypedef HAL_SD_Init(SD_HandleTypeDef *hsd, HAL_SD_CardInfoTypedef *SDCardInfo);
HAL_StatusTypeDef   HAL_SD_DeInit (SD_HandleTypeDef *hsd);
void HAL_SD_MspInit(SD_HandleTypeDef *hsd);
void HAL_SD_MspDeInit(SD_HandleTypeDef *hsd);


 
  


 
 
HAL_SD_ErrorTypedef HAL_SD_ReadBlocks(SD_HandleTypeDef *hsd, uint32_t *pReadBuffer, uint64_t ReadAddr, uint32_t BlockSize, uint32_t NumberOfBlocks);
HAL_SD_ErrorTypedef HAL_SD_WriteBlocks(SD_HandleTypeDef *hsd, uint32_t *pWriteBuffer, uint64_t WriteAddr, uint32_t BlockSize, uint32_t NumberOfBlocks);
HAL_SD_ErrorTypedef HAL_SD_Erase(SD_HandleTypeDef *hsd, uint64_t startaddr, uint64_t endaddr);

 
void HAL_SD_IRQHandler(SD_HandleTypeDef *hsd);

 
void HAL_SD_DMA_RxCpltCallback(DMA_HandleTypeDef *hdma);
void HAL_SD_DMA_RxErrorCallback(DMA_HandleTypeDef *hdma);
void HAL_SD_DMA_TxCpltCallback(DMA_HandleTypeDef *hdma);
void HAL_SD_DMA_TxErrorCallback(DMA_HandleTypeDef *hdma);
void HAL_SD_XferCpltCallback(SD_HandleTypeDef *hsd);
void HAL_SD_XferErrorCallback(SD_HandleTypeDef *hsd);

 
HAL_SD_ErrorTypedef HAL_SD_ReadBlocks_DMA(SD_HandleTypeDef *hsd, uint32_t *pReadBuffer, uint64_t ReadAddr, uint32_t BlockSize, uint32_t NumberOfBlocks);
HAL_SD_ErrorTypedef HAL_SD_WriteBlocks_DMA(SD_HandleTypeDef *hsd, uint32_t *pWriteBuffer, uint64_t WriteAddr, uint32_t BlockSize, uint32_t NumberOfBlocks);
HAL_SD_ErrorTypedef HAL_SD_CheckWriteOperation(SD_HandleTypeDef *hsd, uint32_t Timeout);
HAL_SD_ErrorTypedef HAL_SD_CheckReadOperation(SD_HandleTypeDef *hsd, uint32_t Timeout);


 
  


 
HAL_SD_ErrorTypedef HAL_SD_Get_CardInfo(SD_HandleTypeDef *hsd, HAL_SD_CardInfoTypedef *pCardInfo);
HAL_SD_ErrorTypedef HAL_SD_WideBusOperation_Config(SD_HandleTypeDef *hsd, uint32_t WideMode);
HAL_SD_ErrorTypedef HAL_SD_StopTransfer(SD_HandleTypeDef *hsd);
HAL_SD_ErrorTypedef HAL_SD_HighSpeed (SD_HandleTypeDef *hsd);


 
  
 


 
HAL_SD_ErrorTypedef HAL_SD_SendSDStatus(SD_HandleTypeDef *hsd, uint32_t *pSDstatus);
HAL_SD_ErrorTypedef HAL_SD_GetCardStatus(SD_HandleTypeDef *hsd, HAL_SD_CardStatusTypedef *pCardStatus);
HAL_SD_TransferStateTypedef HAL_SD_GetStatus(SD_HandleTypeDef *hsd);


 
  


 
    
 


 



  

 


 



  
          
 


 



  

 


 



  

 


 



 

 


 



 

 


 



 



  



 








 
#line 363 "..\\USER\\stm32f7xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_spdifrx.h"



































  

 







 
#line 48 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_spdifrx.h"



 



  

  


 



 
typedef struct
{
  uint32_t InputSelection;           
 

  uint32_t Retries;                  
 

  uint32_t WaitForActivity;          
 

  uint32_t ChannelSelection;         
 

  uint32_t DataFormat;               
 
                                               
  uint32_t StereoMode;               
 

    uint32_t PreambleTypeMask;          
 

    uint32_t ChannelStatusMask;        
 
    
    uint32_t ValidityBitMask;          
                                                                                 
                                                                                
    uint32_t ParityErrorMask;          
 
    
}SPDIFRX_InitTypeDef;



 
typedef struct
{
  uint32_t DataFormat;               
 
                                               
  uint32_t StereoMode;               
 

  uint32_t PreambleTypeMask;          
 

  uint32_t ChannelStatusMask;        
 
    
  uint32_t ValidityBitMask;          
                                                                                 
                                                                                
  uint32_t ParityErrorMask;          
 
    
}SPDIFRX_SetDataFormatTypeDef;



  
typedef enum
{
  HAL_SPDIFRX_STATE_RESET      = 0x00U,   
  HAL_SPDIFRX_STATE_READY      = 0x01U,   
  HAL_SPDIFRX_STATE_BUSY       = 0x02U,    
  HAL_SPDIFRX_STATE_BUSY_RX    = 0x03U,     
  HAL_SPDIFRX_STATE_BUSY_CX    = 0x04U,       
  HAL_SPDIFRX_STATE_ERROR      = 0x07U          
}HAL_SPDIFRX_StateTypeDef;



 
typedef struct
{
  SPDIFRX_TypeDef            *Instance;     

  SPDIFRX_InitTypeDef        Init;          
                            
  uint32_t                   *pRxBuffPtr;   
    
    uint32_t                   *pCsBuffPtr;   
  
  volatile uint16_t              RxXferSize;    
  
  volatile uint16_t              RxXferCount;  




 
    
  volatile uint16_t              CsXferSize;    
  
  volatile uint16_t              CsXferCount;  




 
                                                                                             
  DMA_HandleTypeDef          *hdmaCsRx;     

  DMA_HandleTypeDef          *hdmaDrRx;     
  
  volatile HAL_LockTypeDef       Lock;          
  
  volatile HAL_SPDIFRX_StateTypeDef  State;     

  volatile uint32_t  ErrorCode;                 

}SPDIFRX_HandleTypeDef;


 

 


 


  
#line 196 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_spdifrx.h"


 
  


 






 



 






 



 




 
    


 




 



 




 



 




 



 




 



 




 



 





  



 




  



 






 
    


 
#line 316 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_spdifrx.h"


 
    


 
#line 332 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_spdifrx.h"


 



  
  
 


 




 





 





 






 















   


 












 
















 












 

  


 
  
 


 
                                                


 
 
HAL_StatusTypeDef HAL_SPDIFRX_Init(SPDIFRX_HandleTypeDef *hspdif);
HAL_StatusTypeDef HAL_SPDIFRX_DeInit (SPDIFRX_HandleTypeDef *hspdif);
void HAL_SPDIFRX_MspInit(SPDIFRX_HandleTypeDef *hspdif);
void HAL_SPDIFRX_MspDeInit(SPDIFRX_HandleTypeDef *hspdif);
HAL_StatusTypeDef HAL_SPDIFRX_SetDataFormat(SPDIFRX_HandleTypeDef *hspdif, SPDIFRX_SetDataFormatTypeDef  sDataFormat);


 



 
 
  
HAL_StatusTypeDef HAL_SPDIFRX_ReceiveDataFlow(SPDIFRX_HandleTypeDef *hspdif, uint32_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPDIFRX_ReceiveControlFlow(SPDIFRX_HandleTypeDef *hspdif, uint32_t *pData, uint16_t Size, uint32_t Timeout);

  
HAL_StatusTypeDef HAL_SPDIFRX_ReceiveControlFlow_IT(SPDIFRX_HandleTypeDef *hspdif, uint32_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPDIFRX_ReceiveDataFlow_IT(SPDIFRX_HandleTypeDef *hspdif, uint32_t *pData, uint16_t Size);
void HAL_SPDIFRX_IRQHandler(SPDIFRX_HandleTypeDef *hspdif);

 
HAL_StatusTypeDef HAL_SPDIFRX_ReceiveControlFlow_DMA(SPDIFRX_HandleTypeDef *hspdif, uint32_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPDIFRX_ReceiveDataFlow_DMA(SPDIFRX_HandleTypeDef *hspdif, uint32_t *pData, uint16_t Size);

HAL_StatusTypeDef HAL_SPDIFRX_DMAStop(SPDIFRX_HandleTypeDef *hspdif);

 
void HAL_SPDIFRX_RxHalfCpltCallback(SPDIFRX_HandleTypeDef *hspdif);
void HAL_SPDIFRX_RxCpltCallback(SPDIFRX_HandleTypeDef *hspdif);
void HAL_SPDIFRX_ErrorCallback(SPDIFRX_HandleTypeDef *hspdif);
void HAL_SPDIFRX_CxHalfCpltCallback(SPDIFRX_HandleTypeDef *hspdif);
void HAL_SPDIFRX_CxCpltCallback(SPDIFRX_HandleTypeDef *hspdif);


 



 
 
HAL_SPDIFRX_StateTypeDef HAL_SPDIFRX_GetState(SPDIFRX_HandleTypeDef *hspdif);
uint32_t HAL_SPDIFRX_GetError(SPDIFRX_HandleTypeDef *hspdif);


 



  
 
 
 
 


 
#line 526 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_spdifrx.h"
                                             




 

 


 


 
 


 



 
    







 
#line 367 "..\\USER\\stm32f7xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_spi.h"
 


































 

 







 
#line 48 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_spi.h"



 



 

 


 



 
typedef struct
{
  uint32_t Mode;                
 

  uint32_t Direction;           
 

  uint32_t DataSize;            
 

  uint32_t CLKPolarity;         
 

  uint32_t CLKPhase;            
 

  uint32_t NSS;                 

 

  uint32_t BaudRatePrescaler;   



 

  uint32_t FirstBit;            
 

  uint32_t TIMode;              
 

  uint32_t CRCCalculation;      
 

  uint32_t CRCPolynomial;       
 

  uint32_t CRCLength;           

 

  uint32_t NSSPMode;            




 
} SPI_InitTypeDef;



 
typedef enum
{
  HAL_SPI_STATE_RESET      = 0x00U,     
  HAL_SPI_STATE_READY      = 0x01U,     
  HAL_SPI_STATE_BUSY       = 0x02U,     
  HAL_SPI_STATE_BUSY_TX    = 0x03U,     
  HAL_SPI_STATE_BUSY_RX    = 0x04U,     
  HAL_SPI_STATE_BUSY_TX_RX = 0x05U,     
  HAL_SPI_STATE_ERROR      = 0x06U      
}HAL_SPI_StateTypeDef;



 
typedef struct __SPI_HandleTypeDef
{
  SPI_TypeDef             *Instance;       

  SPI_InitTypeDef         Init;            

  uint8_t                 *pTxBuffPtr;     

  uint16_t                TxXferSize;      

  uint16_t                TxXferCount;     

  uint8_t                 *pRxBuffPtr;     

  uint16_t                RxXferSize;      

  uint16_t                RxXferCount;     

  uint32_t                CRCSize;         

  void (*RxISR)(struct __SPI_HandleTypeDef *hspi);  

  void (*TxISR)(struct __SPI_HandleTypeDef *hspi);  

  DMA_HandleTypeDef       *hdmatx;         

  DMA_HandleTypeDef       *hdmarx;         

  HAL_LockTypeDef         Lock;            

  volatile HAL_SPI_StateTypeDef    State;           

  volatile uint32_t                ErrorCode;       

}SPI_HandleTypeDef;



 

 



 



 
#line 190 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_spi.h"


 




 




 



 





 



 
#line 230 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_spi.h"


 



 




 



 




 



 





 



 




 



 
#line 282 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_spi.h"


 



 




 



 




 



 




 







 





 








 






 






 





 








 
#line 373 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_spi.h"


 



 







 



 






 



 

 


 




 











 












 

















 






 







 
#line 478 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_spi.h"






 
#line 492 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_spi.h"






 
#line 505 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_spi.h"





 






 




 

 


 





 






 






 















#line 576 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_spi.h"














#line 598 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_spi.h"



















 

 


 



 

 
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef HAL_SPI_DeInit (SPI_HandleTypeDef *hspi);
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi);
void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi);


 



 

 
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_Transmit_IT(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_Receive_IT(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_TransmitReceive_IT(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_Transmit_DMA(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_Receive_DMA(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_DMAPause(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef HAL_SPI_DMAResume(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef HAL_SPI_DMAStop(SPI_HandleTypeDef *hspi);

void HAL_SPI_IRQHandler(SPI_HandleTypeDef *hspi);
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_TxHalfCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_TxRxHalfCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi);


 



 

 
HAL_SPI_StateTypeDef HAL_SPI_GetState(SPI_HandleTypeDef *hspi);
uint32_t             HAL_SPI_GetError(SPI_HandleTypeDef *hspi);


 



 



 



 







 
#line 371 "..\\USER\\stm32f7xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_tim.h"



































  

 







 
#line 48 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_tim.h"



 



 

 


 
  


 
typedef struct
{
  uint32_t Prescaler;         
 

  uint32_t CounterMode;       
 

  uint32_t Period;            

 

  uint32_t ClockDivision;     
 

  uint32_t RepetitionCounter;  






 
} TIM_Base_InitTypeDef;



 

typedef struct
{
  uint32_t OCMode;        
 

  uint32_t Pulse;         
 

  uint32_t OCPolarity;    
 

  uint32_t OCNPolarity;   

 
  
  uint32_t OCFastMode;   

 


  uint32_t OCIdleState;   

 

  uint32_t OCNIdleState;  

 
} TIM_OC_InitTypeDef;  



 
typedef struct
{
  uint32_t OCMode;        
 

  uint32_t Pulse;         
 

  uint32_t OCPolarity;    
 

  uint32_t OCNPolarity;   

 

  uint32_t OCIdleState;   

 

  uint32_t OCNIdleState;  

 

  uint32_t ICPolarity;    
 

  uint32_t ICSelection;   
 

  uint32_t ICFilter;      
   
} TIM_OnePulse_InitTypeDef;  




 

typedef struct
{
  uint32_t  ICPolarity;   
 

  uint32_t ICSelection;  
 

  uint32_t ICPrescaler;  
 

  uint32_t ICFilter;     
 
} TIM_IC_InitTypeDef;



 

typedef struct
{
  uint32_t EncoderMode;   
 
                                  
  uint32_t IC1Polarity;   
 

  uint32_t IC1Selection;  
 

  uint32_t IC1Prescaler;  
 

  uint32_t IC1Filter;     
 
                                  
  uint32_t IC2Polarity;   
 

  uint32_t IC2Selection;  
 

  uint32_t IC2Prescaler;  
 

  uint32_t IC2Filter;     
 
} TIM_Encoder_InitTypeDef;



  
typedef struct
{
  uint32_t ClockSource;     
  
  uint32_t ClockPolarity;   
 
  uint32_t ClockPrescaler;  
 
  uint32_t ClockFilter;    
 
}TIM_ClockConfigTypeDef;



  
typedef struct
{ 
  uint32_t ClearInputState;      
   
  uint32_t ClearInputSource;     
  
  uint32_t ClearInputPolarity;   
 
  uint32_t ClearInputPrescaler;  
 
  uint32_t ClearInputFilter;    
 
}TIM_ClearInputConfigTypeDef;



  
typedef struct {
  uint32_t  SlaveMode;         
  
  uint32_t  InputTrigger;      
 
  uint32_t  TriggerPolarity;   
 
  uint32_t  TriggerPrescaler;  
 
  uint32_t  TriggerFilter;     
   

}TIM_SlaveConfigTypeDef;



  
typedef enum
{
  HAL_TIM_STATE_RESET             = 0x00U,     
  HAL_TIM_STATE_READY             = 0x01U,     
  HAL_TIM_STATE_BUSY              = 0x02U,     
  HAL_TIM_STATE_TIMEOUT           = 0x03U,     
  HAL_TIM_STATE_ERROR             = 0x04U      
}HAL_TIM_StateTypeDef;



  
typedef enum
{
  HAL_TIM_ACTIVE_CHANNEL_1        = 0x01U,     
  HAL_TIM_ACTIVE_CHANNEL_2        = 0x02U,     
  HAL_TIM_ACTIVE_CHANNEL_3        = 0x04U,     
  HAL_TIM_ACTIVE_CHANNEL_4        = 0x08U,     
  HAL_TIM_ACTIVE_CHANNEL_CLEARED  = 0x00U      
}HAL_TIM_ActiveChannel;



  
typedef struct
{
  TIM_TypeDef                 *Instance;      
  TIM_Base_InitTypeDef        Init;           
  HAL_TIM_ActiveChannel       Channel;        
  DMA_HandleTypeDef           *hdma[7];      
 
  HAL_LockTypeDef             Lock;           
  volatile HAL_TIM_StateTypeDef   State;          
}TIM_HandleTypeDef;


 

 


 



 





 



 




 



 






 



 







 



 





 



 





 



 




 



 




  



 




 



 




 



 




  



 




  



 





 



 








 



 






  



 




 



 






 



  
#line 489 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_tim.h"


 
  


   




 



 
#line 512 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_tim.h"


 



 
#line 528 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_tim.h"


 



 
#line 548 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_tim.h"


 



 
#line 565 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_tim.h"


 



 







 



 






 



 




 



 






 



   




 
  


 




 
  


 






   


                          




 
  


 




 
  


 




   
  


   
#line 678 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_tim.h"


  



 




  
  


 
#line 703 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_tim.h"


   



 







 



 






 




 




  



 
#line 763 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_tim.h"


  



 
#line 788 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_tim.h"


 



 
#line 802 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_tim.h"


  



 






  



    
  
 


 



 






 






 






 




 







 
#line 873 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_tim.h"
                        




 




 




 
#line 899 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_tim.h"

#line 906 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_tim.h"






























 





 






 








 









 












 
#line 988 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_tim.h"




 



















 
















 





  















 





											 


 

 
#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_tim_ex.h"



































  

 







 
#line 48 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_tim_ex.h"



 



  

  


 
  


 

typedef struct
{
                                  
  uint32_t IC1Polarity;            
 
                                                                   
  uint32_t IC1Prescaler;        
 
                                  
  uint32_t IC1Filter;           
   
  uint32_t Commutation_Delay;  
                               
} TIM_HallSensor_InitTypeDef;



  
typedef struct {
  uint32_t  MasterOutputTrigger;   
  
  uint32_t  MasterOutputTrigger2;  
 
  uint32_t  MasterSlaveMode;       
 
}TIM_MasterConfigTypeDef;





  
typedef struct
{
  uint32_t OffStateRunMode;	    
 
  uint32_t OffStateIDLEMode;	    
 
  uint32_t LockLevel;	 	        
                              
  uint32_t DeadTime;	 	        
 
  uint32_t BreakState;	 	        
 
  uint32_t BreakPolarity;           
 
  uint32_t BreakFilter;             
   
  uint32_t Break2State;	 	        
 
  uint32_t Break2Polarity;          
 
  uint32_t Break2Filter;            
   
  uint32_t AutomaticOutput;         
            
} TIM_BreakDeadTimeConfigTypeDef;




 
typedef struct {
  uint32_t Source;         
 
  uint32_t Enable;         
 
} TIMEx_BreakInputConfigTypeDef;



   
 


 
  


 

#line 154 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_tim_ex.h"
                                 


  
    


 
#line 170 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_tim_ex.h"

#line 177 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_tim_ex.h"


 
      


 
#line 196 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_tim_ex.h"


 	



 





 
  


                          




 
    


 




 
 


 






 
	


   
#line 258 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_tim_ex.h"


  
    


 
#line 271 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_tim_ex.h"


 



 




 



 




  



 




  



  



 

 


   















 
#line 337 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_tim_ex.h"













 
#line 358 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_tim_ex.h"



  

 


 



 
 
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Init(TIM_HandleTypeDef* htim, TIM_HallSensor_InitTypeDef* sConfig);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_DeInit(TIM_HandleTypeDef* htim);

void HAL_TIMEx_HallSensor_MspInit(TIM_HandleTypeDef* htim);
void HAL_TIMEx_HallSensor_MspDeInit(TIM_HandleTypeDef* htim);

  
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Start(TIM_HandleTypeDef* htim);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Stop(TIM_HandleTypeDef* htim);
 
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Start_IT(TIM_HandleTypeDef* htim);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Stop_IT(TIM_HandleTypeDef* htim);
 
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Start_DMA(TIM_HandleTypeDef* htim, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIMEx_HallSensor_Stop_DMA(TIM_HandleTypeDef* htim);


 



 
 
 
HAL_StatusTypeDef HAL_TIMEx_OCN_Start(TIM_HandleTypeDef* htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_OCN_Stop(TIM_HandleTypeDef* htim, uint32_t Channel);

 
HAL_StatusTypeDef HAL_TIMEx_OCN_Start_IT(TIM_HandleTypeDef* htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_OCN_Stop_IT(TIM_HandleTypeDef* htim, uint32_t Channel);

 
HAL_StatusTypeDef HAL_TIMEx_OCN_Start_DMA(TIM_HandleTypeDef* htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIMEx_OCN_Stop_DMA(TIM_HandleTypeDef* htim, uint32_t Channel);


 



 
 
 
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start(TIM_HandleTypeDef* htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop(TIM_HandleTypeDef* htim, uint32_t Channel);

 
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start_IT(TIM_HandleTypeDef* htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop_IT(TIM_HandleTypeDef* htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIMEx_PWMN_Start_DMA(TIM_HandleTypeDef* htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIMEx_PWMN_Stop_DMA(TIM_HandleTypeDef* htim, uint32_t Channel);


 



 
 
 
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Start(TIM_HandleTypeDef* htim, uint32_t OutputChannel);
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Stop(TIM_HandleTypeDef* htim, uint32_t OutputChannel);

 
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Start_IT(TIM_HandleTypeDef* htim, uint32_t OutputChannel);
HAL_StatusTypeDef HAL_TIMEx_OnePulseN_Stop_IT(TIM_HandleTypeDef* htim, uint32_t OutputChannel);


 



 
 
HAL_StatusTypeDef HAL_TIMEx_ConfigCommutationEvent(TIM_HandleTypeDef* htim, uint32_t  InputTrigger, uint32_t  CommutationSource);
HAL_StatusTypeDef HAL_TIMEx_ConfigCommutationEvent_IT(TIM_HandleTypeDef* htim, uint32_t  InputTrigger, uint32_t  CommutationSource);
HAL_StatusTypeDef HAL_TIMEx_ConfigCommutationEvent_DMA(TIM_HandleTypeDef* htim, uint32_t  InputTrigger, uint32_t  CommutationSource);
HAL_StatusTypeDef HAL_TIMEx_MasterConfigSynchronization(TIM_HandleTypeDef* htim, TIM_MasterConfigTypeDef * sMasterConfig);
HAL_StatusTypeDef HAL_TIMEx_ConfigBreakDeadTime(TIM_HandleTypeDef* htim, TIM_BreakDeadTimeConfigTypeDef *sBreakDeadTimeConfig);

HAL_StatusTypeDef HAL_TIMEx_ConfigBreakInput(TIM_HandleTypeDef *htim, uint32_t BreakInput, TIMEx_BreakInputConfigTypeDef *sBreakInputConfig);

HAL_StatusTypeDef HAL_TIMEx_RemapConfig(TIM_HandleTypeDef* htim, uint32_t Remap);
HAL_StatusTypeDef HAL_TIMEx_GroupChannel5(TIM_HandleTypeDef *htim, uint32_t OCRef);


 



  
 
void HAL_TIMEx_CommutationCallback(TIM_HandleTypeDef* htim);
void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef* htim);
void TIMEx_DMACommutationCplt(DMA_HandleTypeDef *hdma);


 



 
 
HAL_TIM_StateTypeDef HAL_TIMEx_HallSensor_GetState(TIM_HandleTypeDef* htim);


  



  

 
 
 
 


 
#line 499 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_tim_ex.h"
                                 


                                      



#line 515 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_tim_ex.h"
                              
#line 569 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_tim_ex.h"




                                            





                                                   



   

 


 
  


 



  



 
    






 
#line 1064 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_tim.h"

 


 



 

 
HAL_StatusTypeDef HAL_TIM_Base_Init(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_Base_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_Base_Start(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_Base_Stop(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_Base_Stop_IT(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_Base_Start_DMA(TIM_HandleTypeDef *htim, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIM_Base_Stop_DMA(TIM_HandleTypeDef *htim);


 



 
 
HAL_StatusTypeDef HAL_TIM_OC_Init(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_OC_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_OC_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_OC_MspDeInit(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_OC_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_OC_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_OC_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_OC_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_OC_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIM_OC_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);



 



 
 
HAL_StatusTypeDef HAL_TIM_PWM_Init(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_PWM_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_PWM_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_PWM_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_PWM_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIM_PWM_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);



 



 
 
HAL_StatusTypeDef HAL_TIM_IC_Init(TIM_HandleTypeDef *htim);
HAL_StatusTypeDef HAL_TIM_IC_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_IC_MspDeInit(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_IC_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_IC_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_IC_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_IC_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_IC_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData, uint16_t Length);
HAL_StatusTypeDef HAL_TIM_IC_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);



 



 
 
HAL_StatusTypeDef HAL_TIM_OnePulse_Init(TIM_HandleTypeDef *htim, uint32_t OnePulseMode);
HAL_StatusTypeDef HAL_TIM_OnePulse_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_OnePulse_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_OnePulse_MspDeInit(TIM_HandleTypeDef *htim);
 
HAL_StatusTypeDef HAL_TIM_OnePulse_Start(TIM_HandleTypeDef *htim, uint32_t OutputChannel);
HAL_StatusTypeDef HAL_TIM_OnePulse_Stop(TIM_HandleTypeDef *htim, uint32_t OutputChannel);

 
HAL_StatusTypeDef HAL_TIM_OnePulse_Start_IT(TIM_HandleTypeDef *htim, uint32_t OutputChannel);
HAL_StatusTypeDef HAL_TIM_OnePulse_Stop_IT(TIM_HandleTypeDef *htim, uint32_t OutputChannel);



 



 
 
HAL_StatusTypeDef HAL_TIM_Encoder_Init(TIM_HandleTypeDef *htim,  TIM_Encoder_InitTypeDef* sConfig);
HAL_StatusTypeDef HAL_TIM_Encoder_DeInit(TIM_HandleTypeDef *htim);
void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *htim);
void HAL_TIM_Encoder_MspDeInit(TIM_HandleTypeDef *htim);
  
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_Encoder_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_Encoder_Start_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_Encoder_Stop_IT(TIM_HandleTypeDef *htim, uint32_t Channel);
 
HAL_StatusTypeDef HAL_TIM_Encoder_Start_DMA(TIM_HandleTypeDef *htim, uint32_t Channel, uint32_t *pData1, uint32_t *pData2, uint16_t Length);
HAL_StatusTypeDef HAL_TIM_Encoder_Stop_DMA(TIM_HandleTypeDef *htim, uint32_t Channel);



 



 
 
void HAL_TIM_IRQHandler(TIM_HandleTypeDef *htim);



 



 
 
HAL_StatusTypeDef HAL_TIM_OC_ConfigChannel(TIM_HandleTypeDef *htim, TIM_OC_InitTypeDef* sConfig, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_PWM_ConfigChannel(TIM_HandleTypeDef *htim, TIM_OC_InitTypeDef* sConfig, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_IC_ConfigChannel(TIM_HandleTypeDef *htim, TIM_IC_InitTypeDef* sConfig, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_OnePulse_ConfigChannel(TIM_HandleTypeDef *htim, TIM_OnePulse_InitTypeDef* sConfig, uint32_t OutputChannel,  uint32_t InputChannel);
HAL_StatusTypeDef HAL_TIM_ConfigOCrefClear(TIM_HandleTypeDef *htim, TIM_ClearInputConfigTypeDef * sClearInputConfig, uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_ConfigClockSource(TIM_HandleTypeDef *htim, TIM_ClockConfigTypeDef * sClockSourceConfig);    
HAL_StatusTypeDef HAL_TIM_ConfigTI1Input(TIM_HandleTypeDef *htim, uint32_t TI1_Selection);
HAL_StatusTypeDef HAL_TIM_SlaveConfigSynchronization(TIM_HandleTypeDef *htim, TIM_SlaveConfigTypeDef * sSlaveConfig);
HAL_StatusTypeDef HAL_TIM_SlaveConfigSynchronization_IT(TIM_HandleTypeDef *htim, TIM_SlaveConfigTypeDef * sSlaveConfig);
HAL_StatusTypeDef HAL_TIM_DMABurst_WriteStart(TIM_HandleTypeDef *htim, uint32_t BurstBaseAddress, uint32_t BurstRequestSrc,                                               uint32_t  *BurstBuffer, uint32_t  BurstLength);

HAL_StatusTypeDef HAL_TIM_DMABurst_WriteStop(TIM_HandleTypeDef *htim, uint32_t BurstRequestSrc);
HAL_StatusTypeDef HAL_TIM_DMABurst_ReadStart(TIM_HandleTypeDef *htim, uint32_t BurstBaseAddress, uint32_t BurstRequestSrc,                                               uint32_t  *BurstBuffer, uint32_t  BurstLength);

HAL_StatusTypeDef HAL_TIM_DMABurst_ReadStop(TIM_HandleTypeDef *htim, uint32_t BurstRequestSrc);
HAL_StatusTypeDef HAL_TIM_GenerateEvent(TIM_HandleTypeDef *htim, uint32_t EventSource);
uint32_t HAL_TIM_ReadCapturedValue(TIM_HandleTypeDef *htim, uint32_t Channel);



 



 
 
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_TriggerCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_ErrorCallback(TIM_HandleTypeDef *htim);



 



 
 
HAL_TIM_StateTypeDef HAL_TIM_Base_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_OC_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_PWM_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_IC_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_OnePulse_GetState(TIM_HandleTypeDef *htim);
HAL_TIM_StateTypeDef HAL_TIM_Encoder_GetState(TIM_HandleTypeDef *htim);



 
  


 
  
 


 



 






















































#line 1340 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_tim.h"





#line 1358 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_tim.h"

#line 1369 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_tim.h"












































#line 1421 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_tim.h"




#line 1433 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_tim.h"




























#line 1481 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_tim.h"

#line 1500 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_tim.h"






 
  


 

 


 
void TIM_Base_SetConfig(TIM_TypeDef *TIMx, TIM_Base_InitTypeDef *Structure);
void TIM_TI1_SetConfig(TIM_TypeDef *TIMx, uint32_t TIM_ICPolarity, uint32_t TIM_ICSelection, uint32_t TIM_ICFilter);
void TIM_OC1_SetConfig(TIM_TypeDef *TIMx, TIM_OC_InitTypeDef *OC_Config);
void TIM_OC2_SetConfig(TIM_TypeDef *TIMx, TIM_OC_InitTypeDef *OC_Config);
void TIM_OC3_SetConfig(TIM_TypeDef *TIMx, TIM_OC_InitTypeDef *OC_Config);
void TIM_OC4_SetConfig(TIM_TypeDef *TIMx, TIM_OC_InitTypeDef *OC_Config);
void TIM_ETR_SetConfig(TIM_TypeDef* TIMx, uint32_t TIM_ExtTRGPrescaler, uint32_t TIM_ExtTRGPolarity, uint32_t ExtTRGFilter);

void TIM_DMADelayPulseCplt(DMA_HandleTypeDef *hdma);
void TIM_DMAError(DMA_HandleTypeDef *hdma);
void TIM_DMACaptureCplt(DMA_HandleTypeDef *hdma);
void TIM_CCxChannelCmd(TIM_TypeDef* TIMx, uint32_t Channel, uint32_t ChannelState);


  
     


  



  
  






 
#line 375 "..\\USER\\stm32f7xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_uart.h"



































 

 







 
#line 48 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_uart.h"



 



 

 


 



 
typedef struct
{
  uint32_t BaudRate;                  






 

  uint32_t WordLength;                
 

  uint32_t StopBits;                  
 

  uint32_t Parity;                    




 

  uint32_t Mode;                      
 

  uint32_t HwFlowCtl;                 

 

  uint32_t OverSampling;              
 

  uint32_t OneBitSampling;            

 
}UART_InitTypeDef;



 
typedef struct
{
  uint32_t AdvFeatureInit;        

 

  uint32_t TxPinLevelInvert;      
 

  uint32_t RxPinLevelInvert;      
 

  uint32_t DataInvert;            

 

  uint32_t Swap;                  
 

  uint32_t OverrunDisable;        
 

  uint32_t DMADisableonRxError;   
 

  uint32_t AutoBaudRateEnable;    
 

  uint32_t AutoBaudRateMode;      

 

  uint32_t MSBFirst;              
 
} UART_AdvFeatureInitTypeDef;









































 
typedef enum
{
  HAL_UART_STATE_RESET             = 0x00U,   
 
  HAL_UART_STATE_READY             = 0x20U,   
 
  HAL_UART_STATE_BUSY              = 0x24U,   
 
  HAL_UART_STATE_BUSY_TX           = 0x21U,   
 
  HAL_UART_STATE_BUSY_RX           = 0x22U,   
 
  HAL_UART_STATE_BUSY_TX_RX        = 0x23U,   

 
  HAL_UART_STATE_TIMEOUT           = 0xA0U,   
 
  HAL_UART_STATE_ERROR             = 0xE0U    
 
}HAL_UART_StateTypeDef;



 
typedef enum
{
  UART_CLOCKSOURCE_PCLK1      = 0x00U,     
  UART_CLOCKSOURCE_PCLK2      = 0x01U,     
  UART_CLOCKSOURCE_HSI        = 0x02U,     
  UART_CLOCKSOURCE_SYSCLK     = 0x04U,     
  UART_CLOCKSOURCE_LSE        = 0x08U,     
  UART_CLOCKSOURCE_UNDEFINED  = 0x10U      
}UART_ClockSourceTypeDef;



 
typedef struct
{
  USART_TypeDef            *Instance;         

  UART_InitTypeDef         Init;              

  UART_AdvFeatureInitTypeDef AdvancedInit;    

  uint8_t                  *pTxBuffPtr;       

  uint16_t                 TxXferSize;        

  uint16_t                 TxXferCount;       

  uint8_t                  *pRxBuffPtr;       

  uint16_t                 RxXferSize;        

  uint16_t                 RxXferCount;       

  uint16_t                 Mask;              

  DMA_HandleTypeDef        *hdmatx;           

  DMA_HandleTypeDef        *hdmarx;           

  HAL_LockTypeDef           Lock;             

  volatile HAL_UART_StateTypeDef    gState;      

 

  volatile HAL_UART_StateTypeDef    RxState;     
 

  volatile uint32_t             ErrorCode;    

}UART_HandleTypeDef;



 

 


 


 
#line 277 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_uart.h"


 


 




 



 





 



 






 



 





 

 

 




 



 




 



 




 



 






 



 




 



 




 



 




 



 




 



 




 



 




 



 




 



 







 



 
#line 445 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_uart.h"


 



 




 



 




 



 




 



 




 



 




 



 




 



 




 



 




 



 




 



 



 



 




 



 



 



 



 



 



 



 



 





 
#line 603 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_uart.h"


 










 
#line 625 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_uart.h"







 




 





 



 
#line 659 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_uart.h"


 




 

 


 




 







 























 





 





 





 





 





 




























 

















 



















 




















 

















 













 





      





       





 





 














 


















 


















 


















 








 

 


 




 






 






 







 





 





 


























































#line 1075 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_uart.h"

































 
 
#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_uart_ex.h"



































  

 







 
#line 48 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_uart_ex.h"



 



 

 
 


 
  


 
#line 73 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_uart_ex.h"


 

  


 






   

  


   
  
 



 
           




 
#line 268 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_uart_ex.h"









 
#line 314 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_uart_ex.h"



 

 



 



 

 
HAL_StatusTypeDef HAL_RS485Ex_Init(UART_HandleTypeDef *huart, uint32_t Polarity, uint32_t AssertionTime, uint32_t DeassertionTime);



 
  


  



 

 
HAL_StatusTypeDef HAL_MultiProcessorEx_AddressLength_Set(UART_HandleTypeDef *huart, uint32_t AddressLength);



 
  


 



 







 
#line 1111 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_uart.h"
 


 



 

 
HAL_StatusTypeDef HAL_UART_Init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_HalfDuplex_Init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_LIN_Init(UART_HandleTypeDef *huart, uint32_t BreakDetectLength);
HAL_StatusTypeDef HAL_MultiProcessor_Init(UART_HandleTypeDef *huart, uint8_t Address, uint32_t WakeUpMethod);
HAL_StatusTypeDef HAL_RS485Ex_Init(UART_HandleTypeDef *huart, uint32_t Polarity, uint32_t AssertionTime, uint32_t DeassertionTime);
HAL_StatusTypeDef HAL_UART_DeInit (UART_HandleTypeDef *huart);
void HAL_UART_MspInit(UART_HandleTypeDef *huart);
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart);



 



 

 
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_UART_DMAPause(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_DMAResume(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_UART_DMAStop(UART_HandleTypeDef *huart);

void HAL_UART_IRQHandler(UART_HandleTypeDef *huart);
void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);



 



 

 
HAL_StatusTypeDef HAL_LIN_SendBreak(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_MultiProcessorEx_AddressLength_Set(UART_HandleTypeDef *huart, uint32_t AddressLength);
HAL_StatusTypeDef HAL_MultiProcessor_EnableMuteMode(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_MultiProcessor_DisableMuteMode(UART_HandleTypeDef *huart);
void HAL_MultiProcessor_EnterMuteMode(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_HalfDuplex_EnableTransmitter(UART_HandleTypeDef *huart);
HAL_StatusTypeDef HAL_HalfDuplex_EnableReceiver(UART_HandleTypeDef *huart);



 



 

 
HAL_UART_StateTypeDef HAL_UART_GetState(UART_HandleTypeDef *huart);
uint32_t HAL_UART_GetError(UART_HandleTypeDef *huart);



 



 

 


 

HAL_StatusTypeDef UART_SetConfig(UART_HandleTypeDef *huart);
HAL_StatusTypeDef UART_CheckIdleState(UART_HandleTypeDef *huart);
HAL_StatusTypeDef UART_WaitOnFlagUntilTimeout(UART_HandleTypeDef *huart, uint32_t Flag, FlagStatus Status, uint32_t Tickstart, uint32_t Timeout);
void UART_AdvFeatureConfig(UART_HandleTypeDef *huart);



 



 



 







 
#line 379 "..\\USER\\stm32f7xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_usart.h"



































  

 







 
#line 48 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_usart.h"



 



 

 


 



 
typedef struct
{
  uint32_t BaudRate;                  

 

  uint32_t WordLength;                
 

  uint32_t StopBits;                  
 

  uint32_t Parity;                   




 

  uint32_t Mode;                      
 

  uint32_t OverSampling;              
                                                                                         

  uint32_t CLKPolarity;               
 

  uint32_t CLKPhase;                  
 

  uint32_t CLKLastBit;                

 
}USART_InitTypeDef;



 
typedef enum
{
  HAL_USART_STATE_RESET             = 0x00U,     
  HAL_USART_STATE_READY             = 0x01U,     
  HAL_USART_STATE_BUSY              = 0x02U,     
  HAL_USART_STATE_BUSY_TX           = 0x12U,     
  HAL_USART_STATE_BUSY_RX           = 0x22U,     
  HAL_USART_STATE_BUSY_TX_RX        = 0x32U,     
  HAL_USART_STATE_TIMEOUT           = 0x03U,     
  HAL_USART_STATE_ERROR             = 0x04U      
}HAL_USART_StateTypeDef;




 
typedef enum
{
  USART_CLOCKSOURCE_PCLK1      = 0x00U,     
  USART_CLOCKSOURCE_PCLK2      = 0x01U,     
  USART_CLOCKSOURCE_HSI        = 0x02U,     
  USART_CLOCKSOURCE_SYSCLK     = 0x04U,     
  USART_CLOCKSOURCE_LSE        = 0x08U,     
  USART_CLOCKSOURCE_UNDEFINED  = 0x10U      
}USART_ClockSourceTypeDef;




 
typedef struct
{
  USART_TypeDef                 *Instance;         

  USART_InitTypeDef             Init;              

  uint8_t                       *pTxBuffPtr;       

  uint16_t                      TxXferSize;        

  uint16_t                      TxXferCount;       

  uint8_t                       *pRxBuffPtr;       

  uint16_t                      RxXferSize;        

  uint16_t                      RxXferCount;       

  uint16_t                      Mask;              

  DMA_HandleTypeDef             *hdmatx;           

  DMA_HandleTypeDef             *hdmarx;           

  HAL_LockTypeDef               Lock;             

  HAL_USART_StateTypeDef        State;            

  volatile uint32_t                 ErrorCode;        

}USART_HandleTypeDef;


 

 


 




  
#line 184 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_usart.h"


 



 





 



 





 



 





 



 




 


 




 



 




 



 




 



 




 



 




 





 
#line 290 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_usart.h"


 










 

#line 311 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_usart.h"






 



 
#line 329 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_usart.h"


 



 

 


 




 



















 














 















 


















 















 


















 










 





 





 




 
 
#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_usart_ex.h"



































 

 







 
#line 48 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_usart_ex.h"



 



 

 
 


 



 





 



 

 

 


 









 
#line 129 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_usart_ex.h"






 

 
 
 
 
 




 



 







 
#line 481 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_usart.h"

 


 



 
 
HAL_StatusTypeDef HAL_USART_Init(USART_HandleTypeDef *husart);
HAL_StatusTypeDef HAL_USART_DeInit(USART_HandleTypeDef *husart);
void HAL_USART_MspInit(USART_HandleTypeDef *husart);
void HAL_USART_MspDeInit(USART_HandleTypeDef *husart);
HAL_StatusTypeDef HAL_USART_CheckIdleState(USART_HandleTypeDef *husart);


 



 
 
HAL_StatusTypeDef HAL_USART_Transmit(USART_HandleTypeDef *husart, uint8_t *pTxData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_USART_Receive(USART_HandleTypeDef *husart, uint8_t *pRxData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_USART_TransmitReceive(USART_HandleTypeDef *husart, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_USART_Transmit_IT(USART_HandleTypeDef *husart, uint8_t *pTxData, uint16_t Size);
HAL_StatusTypeDef HAL_USART_Receive_IT(USART_HandleTypeDef *husart, uint8_t *pRxData, uint16_t Size);
HAL_StatusTypeDef HAL_USART_TransmitReceive_IT(USART_HandleTypeDef *husart, uint8_t *pTxData, uint8_t *pRxData,  uint16_t Size);
HAL_StatusTypeDef HAL_USART_Transmit_DMA(USART_HandleTypeDef *husart, uint8_t *pTxData, uint16_t Size);
HAL_StatusTypeDef HAL_USART_Receive_DMA(USART_HandleTypeDef *husart, uint8_t *pRxData, uint16_t Size);
HAL_StatusTypeDef HAL_USART_TransmitReceive_DMA(USART_HandleTypeDef *husart, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size);
HAL_StatusTypeDef HAL_USART_DMAPause(USART_HandleTypeDef *husart);
HAL_StatusTypeDef HAL_USART_DMAResume(USART_HandleTypeDef *husart);
HAL_StatusTypeDef HAL_USART_DMAStop(USART_HandleTypeDef *husart);
void HAL_USART_IRQHandler(USART_HandleTypeDef *husart);
void HAL_USART_TxHalfCpltCallback(USART_HandleTypeDef *husart);
void HAL_USART_TxCpltCallback(USART_HandleTypeDef *husart);
void HAL_USART_RxCpltCallback(USART_HandleTypeDef *husart);
void HAL_USART_RxHalfCpltCallback(USART_HandleTypeDef *husart);
void HAL_USART_TxRxCpltCallback(USART_HandleTypeDef *husart);
void HAL_USART_ErrorCallback(USART_HandleTypeDef *husart);



  



 
 
HAL_USART_StateTypeDef HAL_USART_GetState(USART_HandleTypeDef *husart);
uint32_t               HAL_USART_GetError(USART_HandleTypeDef *husart);



 



 
 
 
 


 


  




 
 


 




 
#line 648 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_usart.h"
  

#line 668 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_usart.h"



 

 


 



 



  



 
  






 
#line 383 "..\\USER\\stm32f7xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_irda.h"



































  

 







 
#line 48 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_irda.h"



 



  

  


 


  
typedef struct
{
  uint32_t BaudRate;                  

 

  uint32_t WordLength;                
 

  uint32_t Parity;                    




 
 
  uint16_t Mode;                      
 
  
  uint8_t  Prescaler;                 

 
  
  uint16_t PowerMode;                 
 
}IRDA_InitTypeDef;







































  
typedef enum
{
  HAL_IRDA_STATE_RESET             = 0x00U,    
 
  HAL_IRDA_STATE_READY             = 0x20U,    
 
  HAL_IRDA_STATE_BUSY              = 0x24U,    
 
  HAL_IRDA_STATE_BUSY_TX           = 0x21U,    
 
  HAL_IRDA_STATE_BUSY_RX           = 0x22U,    
 
  HAL_IRDA_STATE_BUSY_TX_RX        = 0x23U,    

 
  HAL_IRDA_STATE_TIMEOUT           = 0xA0U,    
 
  HAL_IRDA_STATE_ERROR             = 0xE0U     
 
}HAL_IRDA_StateTypeDef;



 
typedef enum
{
  IRDA_CLOCKSOURCE_PCLK1      = 0x00U,     
  IRDA_CLOCKSOURCE_PCLK2      = 0x01U,     
  IRDA_CLOCKSOURCE_HSI        = 0x02U,     
  IRDA_CLOCKSOURCE_SYSCLK     = 0x04U,     
  IRDA_CLOCKSOURCE_LSE        = 0x08U      
}IRDA_ClockSourceTypeDef;



 
typedef struct
{
  USART_TypeDef            *Instance;         

  IRDA_InitTypeDef         Init;              

  uint8_t                  *pTxBuffPtr;       

  uint16_t                 TxXferSize;        

  uint16_t                 TxXferCount;       

  uint8_t                  *pRxBuffPtr;       

  uint16_t                 RxXferSize;        

  uint16_t                 RxXferCount;       

  uint16_t                 Mask;              

  DMA_HandleTypeDef        *hdmatx;           

  DMA_HandleTypeDef        *hdmarx;           

  HAL_LockTypeDef          Lock;              

  volatile HAL_IRDA_StateTypeDef  gState;           

 

  volatile HAL_IRDA_StateTypeDef  RxState;          
 

  volatile uint32_t    ErrorCode;    

}IRDA_HandleTypeDef;



  



 

 


 



  

#line 226 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_irda.h"


 



  





  




  





 



 




 
    
 

  




 

 

  




 



 




   
  


 




   
  


 




 
  




 
#line 322 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_irda.h"


  










   







                                






 




 





 
  


 







  





 





 
  


 

  
 


 





 






















 















 
















 


















 
















 















 












 






 






 




 

 
#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_irda_ex.h"



































 

 







 
#line 48 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_irda_ex.h"



 



  

 
 


 
  


 





 
  
  


   
  
 

 



 




 
#line 173 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_irda_ex.h"





     
#line 215 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_irda_ex.h"






 

 



 



 







 
#line 545 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_irda.h"

 


 



 

 
HAL_StatusTypeDef HAL_IRDA_Init(IRDA_HandleTypeDef *hirda);
HAL_StatusTypeDef HAL_IRDA_DeInit(IRDA_HandleTypeDef *hirda);
void HAL_IRDA_MspInit(IRDA_HandleTypeDef *hirda);
void HAL_IRDA_MspDeInit(IRDA_HandleTypeDef *hirda);


 



 

 
HAL_StatusTypeDef HAL_IRDA_Transmit(IRDA_HandleTypeDef *hirda, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_IRDA_Receive(IRDA_HandleTypeDef *hirda, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_IRDA_Transmit_IT(IRDA_HandleTypeDef *hirda, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_IRDA_Receive_IT(IRDA_HandleTypeDef *hirda, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_IRDA_Transmit_DMA(IRDA_HandleTypeDef *hirda, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_IRDA_Receive_DMA(IRDA_HandleTypeDef *hirda, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_IRDA_DMAPause(IRDA_HandleTypeDef *hirda);
HAL_StatusTypeDef HAL_IRDA_DMAResume(IRDA_HandleTypeDef *hirda);
HAL_StatusTypeDef HAL_IRDA_DMAStop(IRDA_HandleTypeDef *hirda);
void HAL_IRDA_IRQHandler(IRDA_HandleTypeDef *hirda);
void HAL_IRDA_TxCpltCallback(IRDA_HandleTypeDef *hirda);
void HAL_IRDA_RxCpltCallback(IRDA_HandleTypeDef *hirda);
void HAL_IRDA_TxHalfCpltCallback(IRDA_HandleTypeDef *hirda);
void HAL_IRDA_RxHalfCpltCallback(IRDA_HandleTypeDef *hirda);
void HAL_IRDA_ErrorCallback(IRDA_HandleTypeDef *hirda);


 



 
 
HAL_IRDA_StateTypeDef HAL_IRDA_GetState(IRDA_HandleTypeDef *hirda);
uint32_t HAL_IRDA_GetError(IRDA_HandleTypeDef *hirda);


 



 

 
 
 


 



  



 


 

 


 




    





   





								




									 


								  


								  
















 

 


 



 



  



  







 
#line 387 "..\\USER\\stm32f7xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_smartcard.h"



































 

 







 
#line 48 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_smartcard.h"



 



 

  


 



 
typedef struct
{
  uint32_t BaudRate;                  

 
                                           
  uint32_t WordLength;                
 

  uint32_t StopBits;                  
 

  uint32_t Parity;                    



 
 
  uint32_t Mode;                      
 

  uint32_t CLKPolarity;               
 

  uint32_t CLKPhase;                  
 

  uint32_t CLKLastBit;                

 
                                             
  uint32_t OneBitSampling;            

 

  uint32_t  Prescaler;                  
  
  uint32_t  GuardTime;                  
  
  uint32_t NACKState;                  

  
                                           
  uint32_t TimeOutEnable;              
 
  
  uint32_t TimeOutValue;               

  
                                           
  uint32_t BlockLength;                
  
                                           
  uint32_t AutoRetryCount;              


   

}SMARTCARD_InitTypeDef;







































 
typedef struct
{
  uint32_t AdvFeatureInit;            

 

  uint32_t TxPinLevelInvert;          
 

  uint32_t RxPinLevelInvert;          
 

  uint32_t DataInvert;                

 

  uint32_t Swap;                      
 

  uint32_t OverrunDisable;            
 

  uint32_t DMADisableonRxError;       
 

  uint32_t MSBFirst;                  
 
}SMARTCARD_AdvFeatureInitTypeDef;



  
typedef enum
{
  HAL_SMARTCARD_STATE_RESET             = 0x00U,    
 
  HAL_SMARTCARD_STATE_READY             = 0x20U,    
 
  HAL_SMARTCARD_STATE_BUSY              = 0x24U,    
 
  HAL_SMARTCARD_STATE_BUSY_TX           = 0x21U,    
 
  HAL_SMARTCARD_STATE_BUSY_RX           = 0x22U,    
 
  HAL_SMARTCARD_STATE_BUSY_TX_RX        = 0x23U,    

 
  HAL_SMARTCARD_STATE_TIMEOUT           = 0xA0U,    
 
  HAL_SMARTCARD_STATE_ERROR             = 0xE0U     
 
}HAL_SMARTCARD_StateTypeDef;




 
typedef enum
{
  SMARTCARD_CLOCKSOURCE_PCLK1      = 0x00U,     
  SMARTCARD_CLOCKSOURCE_PCLK2      = 0x01U,     
  SMARTCARD_CLOCKSOURCE_HSI        = 0x02U,     
  SMARTCARD_CLOCKSOURCE_SYSCLK     = 0x04U,     
  SMARTCARD_CLOCKSOURCE_LSE        = 0x08U      
}SMARTCARD_ClockSourceTypeDef;



 
typedef struct
{
  USART_TypeDef                       *Instance;         

  SMARTCARD_InitTypeDef               Init;              

  SMARTCARD_AdvFeatureInitTypeDef     AdvancedInit;      

  uint8_t                             *pTxBuffPtr;       

  uint16_t                            TxXferSize;        

  uint16_t                            TxXferCount;       

  uint8_t                             *pRxBuffPtr;       

  uint16_t                            RxXferSize;        

  uint16_t                            RxXferCount;       

  DMA_HandleTypeDef                   *hdmatx;           

  DMA_HandleTypeDef                   *hdmarx;           

  HAL_LockTypeDef                     Lock;              

  volatile HAL_SMARTCARD_StateTypeDef    gState;      

 

  volatile HAL_SMARTCARD_StateTypeDef    RxState;     
 

  volatile uint32_t                       ErrorCode;         

}SMARTCARD_HandleTypeDef;



 
 


 



  
#line 288 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_smartcard.h"


 



 



 



 



 



 




 



 





 



 




  



 




 



 




 



 




   




 




 



 




 
  


 






 



 
#line 404 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_smartcard.h"


 



 




 



 




 



 




  
  


 




  



 




   



 




   



 




   





 
#line 489 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_smartcard.h"


 










 
  
#line 513 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_smartcard.h"





  




 
#line 532 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_smartcard.h"


 



         




 
  
  


 



 
  


 



  
  


 



     
 


  



 
    


     
    
 


 





 






 




















 















 
















 



















 

















 



















 











  






 






 









 




 

 
#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_smartcard_ex.h"



































 

 







 
#line 48 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_smartcard_ex.h"



 



  

 
 
 
   




 
#line 149 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_smartcard_ex.h"

 
 
 
 
void HAL_SMARTCARDEx_BlockLength_Config(SMARTCARD_HandleTypeDef *hsc, uint8_t BlockLength);
void HAL_SMARTCARDEx_TimeOut_Config(SMARTCARD_HandleTypeDef *hsc, uint32_t TimeOutValue);
HAL_StatusTypeDef HAL_SMARTCARDEx_EnableReceiverTimeOut(SMARTCARD_HandleTypeDef *hsc);
HAL_StatusTypeDef HAL_SMARTCARDEx_DisableReceiverTimeOut(SMARTCARD_HandleTypeDef *hsc);

 



  



 
  






 
#line 755 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_smartcard.h"
 


 
  


 
 
HAL_StatusTypeDef HAL_SMARTCARD_Init(SMARTCARD_HandleTypeDef *hsc);
HAL_StatusTypeDef HAL_SMARTCARD_DeInit(SMARTCARD_HandleTypeDef *hsc);
void HAL_SMARTCARD_MspInit(SMARTCARD_HandleTypeDef *hsc);
void HAL_SMARTCARD_MspDeInit(SMARTCARD_HandleTypeDef *hsc);


 



 
 
HAL_StatusTypeDef HAL_SMARTCARD_Transmit(SMARTCARD_HandleTypeDef *hsc, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SMARTCARD_Receive(SMARTCARD_HandleTypeDef *hsc, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SMARTCARD_Transmit_IT(SMARTCARD_HandleTypeDef *hsc, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SMARTCARD_Receive_IT(SMARTCARD_HandleTypeDef *hsc, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SMARTCARD_Transmit_DMA(SMARTCARD_HandleTypeDef *hsc, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SMARTCARD_Receive_DMA(SMARTCARD_HandleTypeDef *hsc, uint8_t *pData, uint16_t Size);
void HAL_SMARTCARD_IRQHandler(SMARTCARD_HandleTypeDef *hsc);
void HAL_SMARTCARD_TxCpltCallback(SMARTCARD_HandleTypeDef *hsc);
void HAL_SMARTCARD_RxCpltCallback(SMARTCARD_HandleTypeDef *hsc);
void HAL_SMARTCARD_ErrorCallback(SMARTCARD_HandleTypeDef *hsc);


 



 
 
HAL_SMARTCARD_StateTypeDef HAL_SMARTCARD_GetState(SMARTCARD_HandleTypeDef *hsc);
uint32_t HAL_SMARTCARD_GetError(SMARTCARD_HandleTypeDef *hsc);



  



 
 
 
 


 

#line 854 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_smartcard.h"




 

 


 



 



  



 







 
#line 391 "..\\USER\\stm32f7xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_wwdg.h"



































 

 







 
#line 48 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_wwdg.h"



 



 

 



 



 
typedef struct
{
  uint32_t Prescaler;     
 

  uint32_t Window;        
 

  uint32_t Counter;       
 

  uint32_t EWIMode ;      
 

}WWDG_InitTypeDef;



 
typedef struct
{
  WWDG_TypeDef                 *Instance;   

  WWDG_InitTypeDef             Init;        

}WWDG_HandleTypeDef;


 

 



 



 



 




 



 



 






 



 




 



 

 



 













 


 



 





 










 









 








 









 









 








 




 

 



 



 
 
HAL_StatusTypeDef     HAL_WWDG_Init(WWDG_HandleTypeDef *hwwdg);
void                  HAL_WWDG_MspInit(WWDG_HandleTypeDef *hwwdg);


 



 
 
HAL_StatusTypeDef     HAL_WWDG_Refresh(WWDG_HandleTypeDef *hwwdg);
void                  HAL_WWDG_IRQHandler(WWDG_HandleTypeDef *hwwdg);
void                  HAL_WWDG_EarlyWakeupCallback(WWDG_HandleTypeDef* hwwdg);


 



 



 



 







 
#line 395 "..\\USER\\stm32f7xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_pcd.h"



































  

 







 
#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_ll_usb.h"



































  

 







 
#line 48 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_ll_usb.h"



 



  

  



   
typedef enum 
{
   USB_OTG_DEVICE_MODE  = 0U,
   USB_OTG_HOST_MODE    = 1U,
   USB_OTG_DRD_MODE     = 2U
   
}USB_OTG_ModeTypeDef;



  
typedef enum {
  URB_IDLE = 0U,
  URB_DONE,
  URB_NOTREADY,
  URB_NYET,
  URB_ERROR,
  URB_STALL
    
}USB_OTG_URBStateTypeDef;



  
typedef enum {
  HC_IDLE = 0U,
  HC_XFRC,
  HC_HALTED,
  HC_NAK,
  HC_NYET,
  HC_STALL,
  HC_XACTERR,  
  HC_BBLERR,   
  HC_DATATGLERR
    
}USB_OTG_HCStateTypeDef;



 
typedef struct
{
  uint32_t dev_endpoints;        

     
  
  uint32_t Host_channels;        

        

  uint32_t speed;                
         
                               
  uint32_t dma_enable;                        

  uint32_t ep0_mps;              
               
                       
  uint32_t phy_itface;           
  
                                
  uint32_t Sof_enable;                 
                               
  uint32_t low_power_enable;      
  
  uint32_t lpm_enable;            
                          
  uint32_t vbus_sensing_enable;    

  uint32_t use_dedicated_ep1;           
  
  uint32_t use_external_vbus;        
  
}USB_OTG_CfgTypeDef;

typedef struct
{
  uint8_t   num;            
  
                                
  uint8_t   is_in;          
  
  
  uint8_t   is_stall;       
  
  
  uint8_t   type;           
  
                                
  uint8_t   data_pid_start; 
 
                                
  uint8_t   even_odd_frame; 
 
                                
  uint16_t  tx_fifo_num;    
 
                                
  uint32_t  maxpacket;      
 

  uint8_t   *xfer_buff;      
                                
  uint32_t  dma_addr;        
  
  uint32_t  xfer_len;        
  
  uint32_t  xfer_count;      

}USB_OTG_EPTypeDef;

typedef struct
{
  uint8_t   dev_addr ;     
  

  uint8_t   ch_num;        
  
                                
  uint8_t   ep_num;        
  
                                
  uint8_t   ep_is_in;      
  
                                
  uint8_t   speed;         
 
                                
  uint8_t   do_ping;        
  
  uint8_t   process_ping;   

  uint8_t   ep_type;       
 
                                
  uint16_t  max_packet;    
 
                                
  uint8_t   data_pid;      
 
                                
  uint8_t   *xfer_buff;     
  
  uint32_t  xfer_len;       
  
  uint32_t  xfer_count;     
  
  uint8_t   toggle_in;     
 
                                
  uint8_t   toggle_out;    
 
  
  uint32_t  dma_addr;       
  
  uint32_t  ErrCnt;         
  
  USB_OTG_URBStateTypeDef  urb_state;  
  
  
  USB_OTG_HCStateTypeDef   state;     
  
                                             
}USB_OTG_HCTypeDef;
  
 



 



 





 



   






 
  


    




 
  


 





 



 






 
  


   






 



 






 



 





 



 







 



 







 



   





 
    


     





   
   




       









    












 
 


    



 
HAL_StatusTypeDef USB_CoreInit(USB_OTG_GlobalTypeDef *USBx, USB_OTG_CfgTypeDef Init);
HAL_StatusTypeDef USB_DevInit(USB_OTG_GlobalTypeDef *USBx, USB_OTG_CfgTypeDef Init);
HAL_StatusTypeDef USB_EnableGlobalInt(USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_DisableGlobalInt(USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_SetCurrentMode(USB_OTG_GlobalTypeDef *USBx , USB_OTG_ModeTypeDef mode);
HAL_StatusTypeDef USB_SetDevSpeed(USB_OTG_GlobalTypeDef *USBx , uint8_t speed);
HAL_StatusTypeDef USB_FlushRxFifo (USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_FlushTxFifo (USB_OTG_GlobalTypeDef *USBx, uint32_t num );
HAL_StatusTypeDef USB_ActivateEndpoint(USB_OTG_GlobalTypeDef *USBx, USB_OTG_EPTypeDef *ep);
HAL_StatusTypeDef USB_DeactivateEndpoint(USB_OTG_GlobalTypeDef *USBx, USB_OTG_EPTypeDef *ep);
HAL_StatusTypeDef USB_ActivateDedicatedEndpoint(USB_OTG_GlobalTypeDef *USBx, USB_OTG_EPTypeDef *ep);
HAL_StatusTypeDef USB_DeactivateDedicatedEndpoint(USB_OTG_GlobalTypeDef *USBx, USB_OTG_EPTypeDef *ep);
HAL_StatusTypeDef USB_EPStartXfer(USB_OTG_GlobalTypeDef *USBx , USB_OTG_EPTypeDef *ep, uint8_t dma);
HAL_StatusTypeDef USB_EP0StartXfer(USB_OTG_GlobalTypeDef *USBx , USB_OTG_EPTypeDef *ep, uint8_t dma);
HAL_StatusTypeDef USB_WritePacket(USB_OTG_GlobalTypeDef *USBx, uint8_t *src, uint8_t ch_ep_num, uint16_t len, uint8_t dma);
void *            USB_ReadPacket(USB_OTG_GlobalTypeDef *USBx, uint8_t *dest, uint16_t len);
HAL_StatusTypeDef USB_EPSetStall(USB_OTG_GlobalTypeDef *USBx , USB_OTG_EPTypeDef *ep);
HAL_StatusTypeDef USB_EPClearStall(USB_OTG_GlobalTypeDef *USBx , USB_OTG_EPTypeDef *ep);
HAL_StatusTypeDef USB_SetDevAddress (USB_OTG_GlobalTypeDef *USBx, uint8_t address);
HAL_StatusTypeDef USB_DevConnect (USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_DevDisconnect (USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_StopDevice(USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_ActivateSetup (USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_EP0_OutStart(USB_OTG_GlobalTypeDef *USBx, uint8_t dma, uint8_t *psetup);
uint8_t           USB_GetDevSpeed(USB_OTG_GlobalTypeDef *USBx);
uint32_t          USB_GetMode(USB_OTG_GlobalTypeDef *USBx);
uint32_t          USB_ReadInterrupts (USB_OTG_GlobalTypeDef *USBx);
uint32_t          USB_ReadDevAllOutEpInterrupt (USB_OTG_GlobalTypeDef *USBx);
uint32_t          USB_ReadDevOutEPInterrupt (USB_OTG_GlobalTypeDef *USBx , uint8_t epnum);
uint32_t          USB_ReadDevAllInEpInterrupt (USB_OTG_GlobalTypeDef *USBx);
uint32_t          USB_ReadDevInEPInterrupt (USB_OTG_GlobalTypeDef *USBx , uint8_t epnum);
void              USB_ClearInterrupts (USB_OTG_GlobalTypeDef *USBx, uint32_t interrupt);

HAL_StatusTypeDef USB_HostInit (USB_OTG_GlobalTypeDef *USBx, USB_OTG_CfgTypeDef cfg);
HAL_StatusTypeDef USB_InitFSLSPClkSel(USB_OTG_GlobalTypeDef *USBx , uint8_t freq);
HAL_StatusTypeDef USB_ResetPort(USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_DriveVbus (USB_OTG_GlobalTypeDef *USBx, uint8_t state);
uint32_t          USB_GetHostSpeed (USB_OTG_GlobalTypeDef *USBx);
uint32_t          USB_GetCurrentFrame (USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_HC_Init(USB_OTG_GlobalTypeDef *USBx,  
                                  uint8_t ch_num,
                                  uint8_t epnum,
                                  uint8_t dev_address,
                                  uint8_t speed,
                                  uint8_t ep_type,
                                  uint16_t mps);
HAL_StatusTypeDef USB_HC_StartXfer(USB_OTG_GlobalTypeDef *USBx, USB_OTG_HCTypeDef *hc, uint8_t dma);
uint32_t          USB_HC_ReadInterrupt (USB_OTG_GlobalTypeDef *USBx);
HAL_StatusTypeDef USB_HC_Halt(USB_OTG_GlobalTypeDef *USBx , uint8_t hc_num);
HAL_StatusTypeDef USB_DoPing(USB_OTG_GlobalTypeDef *USBx , uint8_t ch_num);
HAL_StatusTypeDef USB_StopHost(USB_OTG_GlobalTypeDef *USBx);



  



 
  







 
#line 48 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_pcd.h"
   


 



  

  


 
   


  
typedef enum 
{
  HAL_PCD_STATE_RESET   = 0x00U,
  HAL_PCD_STATE_READY   = 0x01U,
  HAL_PCD_STATE_ERROR   = 0x02U,
  HAL_PCD_STATE_BUSY    = 0x03U,
  HAL_PCD_STATE_TIMEOUT = 0x04U
} PCD_StateTypeDef;

 
typedef enum  
{
  LPM_L0 = 0x00U,  
  LPM_L1 = 0x01U,  
  LPM_L2 = 0x02U,  
  LPM_L3 = 0x03U,  
}PCD_LPM_StateTypeDef;

typedef USB_OTG_GlobalTypeDef  PCD_TypeDef;
typedef USB_OTG_CfgTypeDef     PCD_InitTypeDef;
typedef USB_OTG_EPTypeDef      PCD_EPTypeDef ;                          



  
typedef struct
{
  PCD_TypeDef             *Instance;     
  PCD_InitTypeDef         Init;        
  PCD_EPTypeDef           IN_ep[15];   
  PCD_EPTypeDef           OUT_ep[15];   
  HAL_LockTypeDef         Lock;        
  volatile PCD_StateTypeDef   State;       
  uint32_t                Setup[12];   
  PCD_LPM_StateTypeDef    LPM_State;     
  uint32_t                BESL;
  uint32_t                lpm_active;   
 
  void                    *pData;          
} PCD_HandleTypeDef;



 
    
 
#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_pcd_ex.h"



































  

 







 
#line 48 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_pcd_ex.h"
   


 



 
 
typedef enum  
{
  PCD_LPM_L0_ACTIVE = 0x00U,  
  PCD_LPM_L1_ACTIVE = 0x01U,  
}PCD_LPM_MsgTypeDef;

 
 
 


 


 
HAL_StatusTypeDef HAL_PCDEx_SetTxFiFo(PCD_HandleTypeDef *hpcd, uint8_t fifo, uint16_t size);
HAL_StatusTypeDef HAL_PCDEx_SetRxFiFo(PCD_HandleTypeDef *hpcd, uint16_t size);
HAL_StatusTypeDef HAL_PCDEx_ActivateLPM(PCD_HandleTypeDef *hpcd);
HAL_StatusTypeDef HAL_PCDEx_DeActivateLPM(PCD_HandleTypeDef *hpcd);
void HAL_PCDEx_LPM_Callback(PCD_HandleTypeDef *hpcd, PCD_LPM_MsgTypeDef msg);



  



  



  



 








 
#line 112 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_pcd.h"

 


 



 





 
  


 




 



 
#line 146 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_pcd.h"



 



  
  
 



 


   









                                                      

                                                         


















                                                      









                                                                                                                    








                                                      







                                                         



 

 


 

 


 
HAL_StatusTypeDef HAL_PCD_Init(PCD_HandleTypeDef *hpcd);
HAL_StatusTypeDef HAL_PCD_DeInit (PCD_HandleTypeDef *hpcd);
void HAL_PCD_MspInit(PCD_HandleTypeDef *hpcd);
void HAL_PCD_MspDeInit(PCD_HandleTypeDef *hpcd);


 

 
 


 
HAL_StatusTypeDef HAL_PCD_Start(PCD_HandleTypeDef *hpcd);
HAL_StatusTypeDef HAL_PCD_Stop(PCD_HandleTypeDef *hpcd);
void HAL_PCD_IRQHandler(PCD_HandleTypeDef *hpcd);

void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum);
void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum);
void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef *hpcd);
void HAL_PCD_SOFCallback(PCD_HandleTypeDef *hpcd);
void HAL_PCD_ResetCallback(PCD_HandleTypeDef *hpcd);
void HAL_PCD_SuspendCallback(PCD_HandleTypeDef *hpcd);
void HAL_PCD_ResumeCallback(PCD_HandleTypeDef *hpcd);
void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum);
void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef *hpcd, uint8_t epnum);
void HAL_PCD_ConnectCallback(PCD_HandleTypeDef *hpcd);
void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef *hpcd);


 

 


 
HAL_StatusTypeDef HAL_PCD_DevConnect(PCD_HandleTypeDef *hpcd);
HAL_StatusTypeDef HAL_PCD_DevDisconnect(PCD_HandleTypeDef *hpcd);
HAL_StatusTypeDef HAL_PCD_SetAddress(PCD_HandleTypeDef *hpcd, uint8_t address);
HAL_StatusTypeDef HAL_PCD_EP_Open(PCD_HandleTypeDef *hpcd, uint8_t ep_addr, uint16_t ep_mps, uint8_t ep_type);
HAL_StatusTypeDef HAL_PCD_EP_Close(PCD_HandleTypeDef *hpcd, uint8_t ep_addr);
HAL_StatusTypeDef HAL_PCD_EP_Receive(PCD_HandleTypeDef *hpcd, uint8_t ep_addr, uint8_t *pBuf, uint32_t len);
HAL_StatusTypeDef HAL_PCD_EP_Transmit(PCD_HandleTypeDef *hpcd, uint8_t ep_addr, uint8_t *pBuf, uint32_t len);
uint16_t          HAL_PCD_EP_GetRxCount(PCD_HandleTypeDef *hpcd, uint8_t ep_addr);
HAL_StatusTypeDef HAL_PCD_EP_SetStall(PCD_HandleTypeDef *hpcd, uint8_t ep_addr);
HAL_StatusTypeDef HAL_PCD_EP_ClrStall(PCD_HandleTypeDef *hpcd, uint8_t ep_addr);
HAL_StatusTypeDef HAL_PCD_EP_Flush(PCD_HandleTypeDef *hpcd, uint8_t ep_addr);
HAL_StatusTypeDef HAL_PCD_ActivateRemoteWakeup(PCD_HandleTypeDef *hpcd);
HAL_StatusTypeDef HAL_PCD_DeActivateRemoteWakeup(PCD_HandleTypeDef *hpcd);


 

 


 
PCD_StateTypeDef HAL_PCD_GetState(PCD_HandleTypeDef *hpcd);


  



  

 


 


 




  



  



  



  








 
#line 399 "..\\USER\\stm32f7xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_hcd.h"



































  

 







 
#line 48 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_hcd.h"
   


 




  

 


  



 
typedef enum 
{
  HAL_HCD_STATE_RESET    = 0x00U,
  HAL_HCD_STATE_READY    = 0x01U,
  HAL_HCD_STATE_ERROR    = 0x02U,
  HAL_HCD_STATE_BUSY     = 0x03U,
  HAL_HCD_STATE_TIMEOUT  = 0x04U
} HCD_StateTypeDef;

typedef USB_OTG_GlobalTypeDef   HCD_TypeDef;
typedef USB_OTG_CfgTypeDef      HCD_InitTypeDef;
typedef USB_OTG_HCTypeDef       HCD_HCTypeDef ;   
typedef USB_OTG_URBStateTypeDef HCD_URBStateTypeDef ;
typedef USB_OTG_HCStateTypeDef  HCD_HCStateTypeDef ;


 



  
typedef struct
{
  HCD_TypeDef               *Instance;    
  HCD_InitTypeDef           Init;        
  HCD_HCTypeDef             hc[15];      
  HAL_LockTypeDef           Lock;        
  volatile HCD_StateTypeDef     State;       
  void                      *pData;      
} HCD_HandleTypeDef;


 



  
  
 


 



 





 
  


 




 
  


  
  
 



 














 

 


 



 
HAL_StatusTypeDef      HAL_HCD_Init(HCD_HandleTypeDef *hhcd);
HAL_StatusTypeDef      HAL_HCD_DeInit (HCD_HandleTypeDef *hhcd);
HAL_StatusTypeDef      HAL_HCD_HC_Init(HCD_HandleTypeDef *hhcd,  
                                  uint8_t ch_num,
                                  uint8_t epnum,
                                  uint8_t dev_address,
                                  uint8_t speed,
                                  uint8_t ep_type,
                                  uint16_t mps);

HAL_StatusTypeDef   HAL_HCD_HC_Halt(HCD_HandleTypeDef *hhcd, uint8_t ch_num);
void                HAL_HCD_MspInit(HCD_HandleTypeDef *hhcd);
void                HAL_HCD_MspDeInit(HCD_HandleTypeDef *hhcd);


 

 


 
HAL_StatusTypeDef       HAL_HCD_HC_SubmitRequest(HCD_HandleTypeDef *hhcd,
                                                 uint8_t pipe, 
                                                 uint8_t direction ,
                                                 uint8_t ep_type,  
                                                 uint8_t token, 
                                                 uint8_t* pbuff, 
                                                 uint16_t length,
                                                 uint8_t do_ping);

  
void                    HAL_HCD_IRQHandler(HCD_HandleTypeDef *hhcd);
void             HAL_HCD_SOF_Callback(HCD_HandleTypeDef *hhcd);
void             HAL_HCD_Connect_Callback(HCD_HandleTypeDef *hhcd);
void             HAL_HCD_Disconnect_Callback(HCD_HandleTypeDef *hhcd);
void             HAL_HCD_HC_NotifyURBChange_Callback(HCD_HandleTypeDef *hhcd, 
                                                            uint8_t chnum, 
                                                            HCD_URBStateTypeDef urb_state);


 

 


 
HAL_StatusTypeDef       HAL_HCD_ResetPort(HCD_HandleTypeDef *hhcd);
HAL_StatusTypeDef       HAL_HCD_Start(HCD_HandleTypeDef *hhcd);
HAL_StatusTypeDef       HAL_HCD_Stop(HCD_HandleTypeDef *hhcd);


 

 


 
HCD_StateTypeDef        HAL_HCD_GetState(HCD_HandleTypeDef *hhcd);
HCD_URBStateTypeDef     HAL_HCD_HC_GetURBState(HCD_HandleTypeDef *hhcd, uint8_t chnum);
uint32_t                HAL_HCD_HC_GetXferCount(HCD_HandleTypeDef *hhcd, uint8_t chnum);
HCD_HCStateTypeDef      HAL_HCD_HC_GetState(HCD_HandleTypeDef *hhcd, uint8_t chnum);
uint32_t                HAL_HCD_GetCurrentFrame(HCD_HandleTypeDef *hhcd);
uint32_t                HAL_HCD_GetCurrentSpeed(HCD_HandleTypeDef *hhcd);


 



 

 


 


 




 



 

 


 



 

 


 



 



 



  







 
#line 403 "..\\USER\\stm32f7xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_dfsdm.h"



































 

 








 
#line 49 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_dfsdm.h"



 



  

 


 



  
typedef enum
{
  HAL_DFSDM_CHANNEL_STATE_RESET = 0x00U,  
  HAL_DFSDM_CHANNEL_STATE_READY = 0x01U,  
  HAL_DFSDM_CHANNEL_STATE_ERROR = 0xFFU   
}HAL_DFSDM_Channel_StateTypeDef;



   
typedef struct
{
  FunctionalState Activation;  
  uint32_t        Selection;  
 
  uint32_t        Divider;    
 
}DFSDM_Channel_OutputClockTypeDef;



   
typedef struct
{
  uint32_t Multiplexer; 
 
  uint32_t DataPacking; 
 
  uint32_t Pins;        
 
}DFSDM_Channel_InputTypeDef;



   
typedef struct
{
  uint32_t Type;     
 
  uint32_t SpiClock; 
 
}DFSDM_Channel_SerialInterfaceTypeDef;



   
typedef struct
{
  uint32_t FilterOrder;  
 
  uint32_t Oversampling; 
 
}DFSDM_Channel_AwdTypeDef;



   
typedef struct
{
  DFSDM_Channel_OutputClockTypeDef     OutputClock;      
  DFSDM_Channel_InputTypeDef           Input;            
  DFSDM_Channel_SerialInterfaceTypeDef SerialInterface;  
  DFSDM_Channel_AwdTypeDef             Awd;              
  int32_t                              Offset;          
 
  uint32_t                             RightBitShift;   
 
}DFSDM_Channel_InitTypeDef;



   
typedef struct
{
  DFSDM_Channel_TypeDef          *Instance;  
  DFSDM_Channel_InitTypeDef      Init;       
  HAL_DFSDM_Channel_StateTypeDef State;      
}DFSDM_Channel_HandleTypeDef;



  
typedef enum
{
  HAL_DFSDM_FILTER_STATE_RESET   = 0x00U,  
  HAL_DFSDM_FILTER_STATE_READY   = 0x01U,  
  HAL_DFSDM_FILTER_STATE_REG     = 0x02U,  
  HAL_DFSDM_FILTER_STATE_INJ     = 0x03U,  
  HAL_DFSDM_FILTER_STATE_REG_INJ = 0x04U,  
  HAL_DFSDM_FILTER_STATE_ERROR   = 0xFFU   
}HAL_DFSDM_Filter_StateTypeDef;



   
typedef struct
{
  uint32_t        Trigger;  
 
  FunctionalState FastMode;  
  FunctionalState DmaMode;   
}DFSDM_Filter_RegularParamTypeDef;



   
typedef struct
{
  uint32_t        Trigger;        
 
  FunctionalState ScanMode;        
  FunctionalState DmaMode;         
  uint32_t        ExtTrigger;     
 
  uint32_t        ExtTriggerEdge; 
 
}DFSDM_Filter_InjectedParamTypeDef;



   
typedef struct
{
  uint32_t SincOrder;       
 
  uint32_t Oversampling;    
 
  uint32_t IntOversampling; 
 
}DFSDM_Filter_FilterParamTypeDef;



   
typedef struct
{
  DFSDM_Filter_RegularParamTypeDef  RegularParam;   
  DFSDM_Filter_InjectedParamTypeDef InjectedParam;  
  DFSDM_Filter_FilterParamTypeDef   FilterParam;    
}DFSDM_Filter_InitTypeDef;



   
typedef struct
{
  DFSDM_Filter_TypeDef          *Instance;            
  DFSDM_Filter_InitTypeDef      Init;                 
  DMA_HandleTypeDef             *hdmaReg;             
  DMA_HandleTypeDef             *hdmaInj;             
  uint32_t                      RegularContMode;      
  uint32_t                      RegularTrigger;       
  uint32_t                      InjectedTrigger;      
  uint32_t                      ExtTriggerEdge;       
  FunctionalState               InjectedScanMode;     
  uint32_t                      InjectedChannelsNbr;  
  uint32_t                      InjConvRemaining;     
  HAL_DFSDM_Filter_StateTypeDef State;                
  uint32_t                      ErrorCode;              
}DFSDM_Filter_HandleTypeDef;



   
typedef struct
{
  uint32_t DataSource;      
 
  uint32_t Channel;         
 
  int32_t  HighThreshold;   
 
  int32_t  LowThreshold;    
 
  uint32_t HighBreakSignal; 
 
  uint32_t LowBreakSignal;  
 
}DFSDM_Filter_AwdParamTypeDef;



  
 

 


 



 




 



 




 



 





 



 




 



 






 



 






 



 






 



 





 



 
#line 354 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_dfsdm.h"


  



 





 



 
#line 377 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_dfsdm.h"


 



 




 



  






 



 







 



 
 






 
#line 432 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_dfsdm.h"


 



 




 



 




 



  
 

   


 




 





 




 
 
  
   


 



 
 
HAL_StatusTypeDef HAL_DFSDM_ChannelInit(DFSDM_Channel_HandleTypeDef *hdfsdm_channel);
HAL_StatusTypeDef HAL_DFSDM_ChannelDeInit(DFSDM_Channel_HandleTypeDef *hdfsdm_channel);
void HAL_DFSDM_ChannelMspInit(DFSDM_Channel_HandleTypeDef *hdfsdm_channel);
void HAL_DFSDM_ChannelMspDeInit(DFSDM_Channel_HandleTypeDef *hdfsdm_channel);


 



 
 
HAL_StatusTypeDef HAL_DFSDM_ChannelCkabStart(DFSDM_Channel_HandleTypeDef *hdfsdm_channel);
HAL_StatusTypeDef HAL_DFSDM_ChannelCkabStart_IT(DFSDM_Channel_HandleTypeDef *hdfsdm_channel);
HAL_StatusTypeDef HAL_DFSDM_ChannelCkabStop(DFSDM_Channel_HandleTypeDef *hdfsdm_channel);
HAL_StatusTypeDef HAL_DFSDM_ChannelCkabStop_IT(DFSDM_Channel_HandleTypeDef *hdfsdm_channel);

HAL_StatusTypeDef HAL_DFSDM_ChannelScdStart(DFSDM_Channel_HandleTypeDef *hdfsdm_channel, uint32_t Threshold, uint32_t BreakSignal);
HAL_StatusTypeDef HAL_DFSDM_ChannelScdStart_IT(DFSDM_Channel_HandleTypeDef *hdfsdm_channel, uint32_t Threshold, uint32_t BreakSignal);
HAL_StatusTypeDef HAL_DFSDM_ChannelScdStop(DFSDM_Channel_HandleTypeDef *hdfsdm_channel);
HAL_StatusTypeDef HAL_DFSDM_ChannelScdStop_IT(DFSDM_Channel_HandleTypeDef *hdfsdm_channel);

int16_t           HAL_DFSDM_ChannelGetAwdValue(DFSDM_Channel_HandleTypeDef *hdfsdm_channel);
HAL_StatusTypeDef HAL_DFSDM_ChannelModifyOffset(DFSDM_Channel_HandleTypeDef *hdfsdm_channel, int32_t Offset);

HAL_StatusTypeDef HAL_DFSDM_ChannelPollForCkab(DFSDM_Channel_HandleTypeDef *hdfsdm_channel, uint32_t Timeout);
HAL_StatusTypeDef HAL_DFSDM_ChannelPollForScd(DFSDM_Channel_HandleTypeDef *hdfsdm_channel, uint32_t Timeout);

void HAL_DFSDM_ChannelCkabCallback(DFSDM_Channel_HandleTypeDef *hdfsdm_channel);
void HAL_DFSDM_ChannelScdCallback(DFSDM_Channel_HandleTypeDef *hdfsdm_channel);


 



 
 
HAL_DFSDM_Channel_StateTypeDef HAL_DFSDM_ChannelGetState(DFSDM_Channel_HandleTypeDef *hdfsdm_channel);


 



 
 
HAL_StatusTypeDef HAL_DFSDM_FilterInit(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
HAL_StatusTypeDef HAL_DFSDM_FilterDeInit(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
void HAL_DFSDM_FilterMspInit(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
void HAL_DFSDM_FilterMspDeInit(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);


 



 
 
HAL_StatusTypeDef HAL_DFSDM_FilterConfigRegChannel(DFSDM_Filter_HandleTypeDef *hdfsdm_filter,
                                                   uint32_t                    Channel,
                                                   uint32_t                    ContinuousMode);
HAL_StatusTypeDef HAL_DFSDM_FilterConfigInjChannel(DFSDM_Filter_HandleTypeDef *hdfsdm_filter,
                                                   uint32_t                    Channel);


 



 
 
HAL_StatusTypeDef HAL_DFSDM_FilterRegularStart(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
HAL_StatusTypeDef HAL_DFSDM_FilterRegularStart_IT(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
HAL_StatusTypeDef HAL_DFSDM_FilterRegularStart_DMA(DFSDM_Filter_HandleTypeDef *hdfsdm_filter, int32_t *pData, uint32_t Length);
HAL_StatusTypeDef HAL_DFSDM_FilterRegularMsbStart_DMA(DFSDM_Filter_HandleTypeDef *hdfsdm_filter, int16_t *pData, uint32_t Length);
HAL_StatusTypeDef HAL_DFSDM_FilterRegularStop(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
HAL_StatusTypeDef HAL_DFSDM_FilterRegularStop_IT(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
HAL_StatusTypeDef HAL_DFSDM_FilterRegularStop_DMA(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
HAL_StatusTypeDef HAL_DFSDM_FilterInjectedStart(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
HAL_StatusTypeDef HAL_DFSDM_FilterInjectedStart_IT(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
HAL_StatusTypeDef HAL_DFSDM_FilterInjectedStart_DMA(DFSDM_Filter_HandleTypeDef *hdfsdm_filter, int32_t *pData, uint32_t Length);
HAL_StatusTypeDef HAL_DFSDM_FilterInjectedMsbStart_DMA(DFSDM_Filter_HandleTypeDef *hdfsdm_filter, int16_t *pData, uint32_t Length);
HAL_StatusTypeDef HAL_DFSDM_FilterInjectedStop(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
HAL_StatusTypeDef HAL_DFSDM_FilterInjectedStop_IT(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
HAL_StatusTypeDef HAL_DFSDM_FilterInjectedStop_DMA(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
HAL_StatusTypeDef HAL_DFSDM_FilterAwdStart_IT(DFSDM_Filter_HandleTypeDef *hdfsdm_filter,
                                              DFSDM_Filter_AwdParamTypeDef* awdParam);
HAL_StatusTypeDef HAL_DFSDM_FilterAwdStop_IT(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
HAL_StatusTypeDef HAL_DFSDM_FilterExdStart(DFSDM_Filter_HandleTypeDef *hdfsdm_filter, uint32_t Channel);
HAL_StatusTypeDef HAL_DFSDM_FilterExdStop(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);

int32_t  HAL_DFSDM_FilterGetRegularValue(DFSDM_Filter_HandleTypeDef *hdfsdm_filter, uint32_t* Channel);
int32_t  HAL_DFSDM_FilterGetInjectedValue(DFSDM_Filter_HandleTypeDef *hdfsdm_filter, uint32_t* Channel);
int32_t  HAL_DFSDM_FilterGetExdMaxValue(DFSDM_Filter_HandleTypeDef *hdfsdm_filter, uint32_t* Channel);
int32_t  HAL_DFSDM_FilterGetExdMinValue(DFSDM_Filter_HandleTypeDef *hdfsdm_filter, uint32_t* Channel);
uint32_t HAL_DFSDM_FilterGetConvTimeValue(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);

void HAL_DFSDM_IRQHandler(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);

HAL_StatusTypeDef HAL_DFSDM_FilterPollForRegConversion(DFSDM_Filter_HandleTypeDef *hdfsdm_filter, uint32_t Timeout);
HAL_StatusTypeDef HAL_DFSDM_FilterPollForInjConversion(DFSDM_Filter_HandleTypeDef *hdfsdm_filter, uint32_t Timeout);

void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
void HAL_DFSDM_FilterRegConvHalfCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
void HAL_DFSDM_FilterInjConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
void HAL_DFSDM_FilterInjConvHalfCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
void HAL_DFSDM_FilterAwdCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter, uint32_t Channel, uint32_t Threshold);
void HAL_DFSDM_FilterErrorCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);


 



 
 
HAL_DFSDM_Filter_StateTypeDef HAL_DFSDM_FilterGetState(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);
uint32_t                      HAL_DFSDM_FilterGetError(DFSDM_Filter_HandleTypeDef *hdfsdm_filter);


 



 
 

 


 
#line 691 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_dfsdm.h"


  
 



  



 







 
#line 407 "..\\USER\\stm32f7xx_hal_conf.h"






#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_jpeg.h"



































  

 







 
#line 48 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_jpeg.h"



 



 

 


 




 
typedef struct
{
  uint8_t  ColorSpace;                
 
  
  uint8_t  ChromaSubsampling;         
 
  
  uint32_t ImageHeight;                
  
  uint32_t ImageWidth;                 
  
  uint8_t  ImageQuality;                

}JPEG_ConfTypeDef;


 




 
typedef enum
{
  HAL_JPEG_STATE_RESET              = 0x00U,   
  HAL_JPEG_STATE_READY              = 0x01U,   
  HAL_JPEG_STATE_BUSY               = 0x02U,   
  HAL_JPEG_STATE_BUSY_ENCODING      = 0x03U,   
  HAL_JPEG_STATE_BUSY_DECODING      = 0x04U,     
  HAL_JPEG_STATE_TIMEOUT            = 0x05U,   
  HAL_JPEG_STATE_ERROR              = 0x06U    
}HAL_JPEG_STATETypeDef;



 





 
typedef struct
{
  JPEG_TypeDef             *Instance;         
            
  JPEG_ConfTypeDef         Conf;              

  uint8_t                  *pJpegInBuffPtr;   

  uint8_t                  *pJpegOutBuffPtr;  

  volatile uint32_t            JpegInCount;       

  volatile uint32_t            JpegOutCount;      
    
  uint32_t                 InDataLength;      

  uint32_t                 OutDataLength;       

  DMA_HandleTypeDef        *hdmain;           

  DMA_HandleTypeDef        *hdmaout;          

  uint8_t                  CustomQuanTable;   
      
  uint8_t                  *QuantTable0;      

  uint8_t                  *QuantTable1;      
      
  uint8_t                  *QuantTable2;      
      
  uint8_t                  *QuantTable3;            
      
  HAL_LockTypeDef          Lock;              
      
  volatile  HAL_JPEG_STATETypeDef State;          
      
  volatile  uint32_t           ErrorCode;         
  
  volatile uint32_t Context;                      

}JPEG_HandleTypeDef;



 



 

 



 




  









 




 



 

  



 







 





 






  




 





      
  



 
#line 237 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_jpeg.h"


   




  
#line 252 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_jpeg.h"




 




   





 



 
 



 




 







 






 



















 












 

















 

















 
















 




 

 


 



     
 
HAL_StatusTypeDef HAL_JPEG_Init(JPEG_HandleTypeDef *hjpeg);
HAL_StatusTypeDef HAL_JPEG_DeInit(JPEG_HandleTypeDef *hjpeg);
void HAL_JPEG_MspInit(JPEG_HandleTypeDef *hjpeg);
void HAL_JPEG_MspDeInit(JPEG_HandleTypeDef *hjpeg);



 



  
 
HAL_StatusTypeDef HAL_JPEG_ConfigEncoding(JPEG_HandleTypeDef *hjpeg, JPEG_ConfTypeDef *pConf);
HAL_StatusTypeDef HAL_JPEG_GetInfo(JPEG_HandleTypeDef *hjpeg, JPEG_ConfTypeDef *pInfo);
HAL_StatusTypeDef HAL_JPEG_EnableHeaderParsing(JPEG_HandleTypeDef *hjpeg);
HAL_StatusTypeDef HAL_JPEG_DisableHeaderParsing(JPEG_HandleTypeDef *hjpeg);
HAL_StatusTypeDef HAL_JPEG_SetUserQuantTables(JPEG_HandleTypeDef *hjpeg, uint8_t *QTable0, uint8_t *QTable1, uint8_t *QTable2, uint8_t *QTable3);



 



  
 
HAL_StatusTypeDef  HAL_JPEG_Encode(JPEG_HandleTypeDef *hjpeg, uint8_t *pDataInMCU, uint32_t InDataLength, uint8_t *pDataOut, uint32_t OutDataLength, uint32_t Timeout);
HAL_StatusTypeDef  HAL_JPEG_Decode(JPEG_HandleTypeDef *hjpeg ,uint8_t *pDataIn ,uint32_t InDataLength ,uint8_t *pDataOutMCU ,uint32_t OutDataLength, uint32_t Timeout);
HAL_StatusTypeDef  HAL_JPEG_Encode_IT(JPEG_HandleTypeDef *hjpeg, uint8_t *pDataInMCU, uint32_t InDataLength, uint8_t *pDataOut, uint32_t OutDataLength);
HAL_StatusTypeDef  HAL_JPEG_Decode_IT(JPEG_HandleTypeDef *hjpeg ,uint8_t *pDataIn ,uint32_t InDataLength ,uint8_t *pDataOutMCU ,uint32_t OutDataLength);
HAL_StatusTypeDef  HAL_JPEG_Encode_DMA(JPEG_HandleTypeDef *hjpeg, uint8_t *pDataInMCU, uint32_t InDataLength, uint8_t *pDataOut, uint32_t OutDataLength);
HAL_StatusTypeDef  HAL_JPEG_Decode_DMA(JPEG_HandleTypeDef *hjpeg ,uint8_t *pDataIn ,uint32_t InDataLength ,uint8_t *pDataOutMCU ,uint32_t OutDataLength);
HAL_StatusTypeDef  HAL_JPEG_Pause(JPEG_HandleTypeDef *hjpeg, uint32_t XferSelection);
HAL_StatusTypeDef  HAL_JPEG_Resume(JPEG_HandleTypeDef *hjpeg, uint32_t XferSelection);
void HAL_JPEG_ConfigInputBuffer(JPEG_HandleTypeDef *hjpeg, uint8_t *pNewInputBuffer, uint32_t InDataLength);
void HAL_JPEG_ConfigOutputBuffer(JPEG_HandleTypeDef *hjpeg, uint8_t *pNewOutputBuffer, uint32_t OutDataLength);
HAL_StatusTypeDef HAL_JPEG_Abort(JPEG_HandleTypeDef *hjpeg);



 



  
 
void HAL_JPEG_InfoReadyCallback(JPEG_HandleTypeDef *hjpeg,JPEG_ConfTypeDef *pInfo);
void HAL_JPEG_EncodeCpltCallback(JPEG_HandleTypeDef *hjpeg);
void HAL_JPEG_DecodeCpltCallback(JPEG_HandleTypeDef *hjpeg);
void HAL_JPEG_ErrorCallback(JPEG_HandleTypeDef *hjpeg);
void HAL_JPEG_GetDataCallback(JPEG_HandleTypeDef *hjpeg, uint32_t NbDecodedData);
void HAL_JPEG_DataReadyCallback (JPEG_HandleTypeDef *hjpeg, uint8_t *pDataOut, uint32_t OutDataLength);



 



  
 
void HAL_JPEG_IRQHandler(JPEG_HandleTypeDef *hjpeg);



 



  
 
HAL_JPEG_STATETypeDef  HAL_JPEG_GetState(JPEG_HandleTypeDef *hjpeg);
uint32_t               HAL_JPEG_GetError(JPEG_HandleTypeDef *hjpeg);



 



  

 


 



  

 


 



  
          
 


 



  

 


 



  

 


 



 

















  



  

 


 



 

 


 



 



 



 








 
#line 415 "..\\USER\\stm32f7xx_hal_conf.h"


#line 1 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_mdios.h"



































 

 








   
 
#line 50 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_mdios.h"



 



  

 


 
   


 

typedef enum
{
  HAL_MDIOS_STATE_RESET             = 0x00U,     
  HAL_MDIOS_STATE_READY             = 0x01U,     
  HAL_MDIOS_STATE_BUSY              = 0x02U,     
  HAL_MDIOS_STATE_ERROR             = 0x04U      
}HAL_MDIOS_StateTypeDef;



 



 

typedef struct
{
  uint32_t PortAddress;           
 
  uint32_t PreambleCheck;         
    
}MDIOS_InitTypeDef;



 



 

typedef struct
{
  MDIOS_TypeDef                *Instance;      
  
  MDIOS_InitTypeDef            Init;           
  
  volatile HAL_MDIOS_StateTypeDef  State;          
  
  HAL_LockTypeDef              Lock;           
}MDIOS_HandleTypeDef;



 



 

 


 



 




 



 
#line 168 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_mdios.h"


  



 
#line 208 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal_mdios.h"


 



 





 



 





 

 

 



 
  


 
 


 




 






 













 











 






 






 










 










 











 





 





 





 





 





 





 





 

                                                            



 





                                                       





 





 






 





 




 

 


 



 
HAL_StatusTypeDef HAL_MDIOS_Init(MDIOS_HandleTypeDef *hmdios);
HAL_StatusTypeDef HAL_MDIOS_DeInit(MDIOS_HandleTypeDef *hmdios);
void HAL_MDIOS_MspInit(MDIOS_HandleTypeDef *hmdios);
void  HAL_MDIOS_MspDeInit(MDIOS_HandleTypeDef *hmdios);


 



 
HAL_StatusTypeDef HAL_MDIOS_WriteReg(MDIOS_HandleTypeDef *hmdios,  uint32_t RegNum, uint16_t Data);
HAL_StatusTypeDef HAL_MDIOS_ReadReg(MDIOS_HandleTypeDef *hmdios,  uint32_t RegNum, uint16_t *pData);

uint32_t HAL_MDIOS_GetWrittenRegAddress(MDIOS_HandleTypeDef *hmdios);
uint32_t HAL_MDIOS_GetReadRegAddress(MDIOS_HandleTypeDef *hmdios);
HAL_StatusTypeDef HAL_MDIOS_ClearWriteRegAddress(MDIOS_HandleTypeDef *hmdios, uint32_t RegNum);
HAL_StatusTypeDef HAL_MDIOS_ClearReadRegAddress(MDIOS_HandleTypeDef *hmdios, uint32_t RegNum);

HAL_StatusTypeDef HAL_MDIOS_EnableEvents(MDIOS_HandleTypeDef *hmdios);
void HAL_MDIOS_IRQHandler(MDIOS_HandleTypeDef *hmdios);
void HAL_MDIOS_WriteCpltCallback(MDIOS_HandleTypeDef *hmdios);
void HAL_MDIOS_ReadCpltCallback(MDIOS_HandleTypeDef *hmdios);
void HAL_MDIOS_ErrorCallback(MDIOS_HandleTypeDef *hmdios);
void HAL_MDIOS_WakeUpCallback(MDIOS_HandleTypeDef *hmdios);


 



 
uint32_t HAL_MDIOS_GetError(MDIOS_HandleTypeDef *hmdios);
HAL_MDIOS_StateTypeDef HAL_MDIOS_GetState(MDIOS_HandleTypeDef *hmdios);


 



 

 


 



  

 


 



 

 


 



 

 


 








 

 
  
 


 



 




 



 









 
#line 419 "..\\USER\\stm32f7xx_hal_conf.h"

   
 
#line 437 "..\\USER\\stm32f7xx_hal_conf.h"







 

 
#line 49 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal.h"



 



  

 
 


 



 




 



 
   
 


 
  

 
#line 108 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal.h"

#line 133 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Inc\\stm32f7xx_hal.h"



 

                                       


 









 






 





 





 
  
 


 


 
 
HAL_StatusTypeDef HAL_Init(void);
HAL_StatusTypeDef HAL_DeInit(void);
void HAL_MspInit(void);
void HAL_MspDeInit(void);
HAL_StatusTypeDef HAL_InitTick (uint32_t TickPriority);


 
 


  
 
void HAL_IncTick(void);
void HAL_Delay(volatile uint32_t Delay);
uint32_t HAL_GetTick(void);
void HAL_SuspendTick(void);
void HAL_ResumeTick(void);
uint32_t HAL_GetHalVersion(void);
uint32_t HAL_GetREVID(void);
uint32_t HAL_GetDEVID(void);
void HAL_DBGMCU_EnableDBGSleepMode(void);
void HAL_DBGMCU_DisableDBGSleepMode(void);
void HAL_DBGMCU_EnableDBGStopMode(void);
void HAL_DBGMCU_DisableDBGStopMode(void);
void HAL_DBGMCU_EnableDBGStandbyMode(void);
void HAL_DBGMCU_DisableDBGStandbyMode(void);
void HAL_EnableCompensationCell(void);
void HAL_DisableCompensationCell(void);
void HAL_EnableFMCMemorySwapping(void);
void HAL_DisableFMCMemorySwapping(void);

void HAL_EnableMemorySwappingBank(void);
void HAL_DisableMemorySwappingBank(void);



 



   
 
 


 


 
 


 


 
 
 


  



  
  






 
#line 67 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Src\\stm32f7xx_hal_timebase_rtc_wakeup_template.c"


 



  

 
 







 

 
 

#line 97 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Src\\stm32f7xx_hal_timebase_rtc_wakeup_template.c"

 
 
RTC_HandleTypeDef        hRTC_Handle;

 
void RTC_WKUP_IRQHandler(void);

 













 
HAL_StatusTypeDef HAL_InitTick (uint32_t TickPriority)
{
  volatile uint32_t counter = 0U;

  RCC_OscInitTypeDef        RCC_OscInitStruct;
  RCC_PeriphCLKInitTypeDef  PeriphClkInitStruct;

#line 140 "..\\HALLIB\\STM32F7xx_HAL_Driver\\Src\\stm32f7xx_hal_timebase_rtc_wakeup_template.c"
   
  RCC_OscInitStruct.OscillatorType = ((uint32_t)0x00000001U);
  RCC_OscInitStruct.PLL.PLLState = ((uint32_t)0x00000000U);
  RCC_OscInitStruct.HSEState = 0x00010000U;
   
  PeriphClkInitStruct.RTCClockSelection = ((uint32_t)((uint32_t)0x00000300U | (uint32_t)((((uint32_t)8000000U)/1000000U) << 16U)));




  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) == HAL_OK)
  { 
    PeriphClkInitStruct.PeriphClockSelection = ((uint32_t)0x00000020U);
    if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) == HAL_OK)
    {
       
      (((RCC_TypeDef *) ((0x40000000U + 0x00020000U) + 0x3800U))->BDCR |= (0x00008000U));
      










 
      hRTC_Handle.Instance = ((RTC_TypeDef *) (0x40000000U + 0x2800U));
      hRTC_Handle.Init.HourFormat = ((uint32_t)0x00000000U);
      hRTC_Handle.Init.AsynchPrediv = 99U;
      hRTC_Handle.Init.SynchPrediv = 9U;
      hRTC_Handle.Init.OutPut = ((uint32_t)0x00000000U);
      hRTC_Handle.Init.OutPutPolarity = ((uint32_t)0x00000000U);
      hRTC_Handle.Init.OutPutType = ((uint32_t)0x00000000U);
      HAL_RTC_Init(&hRTC_Handle);

       
      do{ (&hRTC_Handle)->Instance ->WPR = 0xCA; (&hRTC_Handle)->Instance ->WPR = 0x53; } while(0);

       
      ((&hRTC_Handle)->Instance ->CR &= ~(0x00000400U));

        
      ((&hRTC_Handle)->Instance ->CR &= ~(((uint32_t)0x00004000U)));

       
      while((((((&hRTC_Handle)->Instance ->ISR) & (((uint32_t)0x00000004U))) != RESET) ? SET : RESET) == RESET)
      {
        if(counter++ == (SystemCoreClock /48U)) 
        {
          return HAL_ERROR;
        }
      }

       
      (((PWR_TypeDef *) (0x40000000U + 0x7000U))->CR1 |= (0x00000001U) << 2);

       
      ((&hRTC_Handle)->Instance ->ISR) = (~((((uint32_t)0x00000400U)) | 0x00000080U)|((&hRTC_Handle)->Instance ->ISR & 0x00000080U));

       
      hRTC_Handle.Instance->WUTR = (uint32_t)0U;

       
      hRTC_Handle.Instance->CR &= (uint32_t)~0x00000007U;

       
      hRTC_Handle.Instance->CR |= (uint32_t)((uint32_t)0x00000004U);

       
      (((EXTI_TypeDef *) ((0x40000000U + 0x00010000U) + 0x3C00U))->IMR |= ((uint32_t)0x00400000U));

      (((EXTI_TypeDef *) ((0x40000000U + 0x00010000U) + 0x3C00U))->RTSR |= ((uint32_t)0x00400000U));

       
      ((&hRTC_Handle)->Instance ->CR |= (((uint32_t)0x00004000U)));

       
      ((&hRTC_Handle)->Instance ->CR |= (0x00000400U));

       
      do{ (&hRTC_Handle)->Instance ->WPR = 0xFF; } while(0);

      HAL_NVIC_SetPriority(RTC_WKUP_IRQn, TickPriority, 0U);
      HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn); 
      return HAL_OK;
    }
  }
  return HAL_ERROR;
}






 
void HAL_SuspendTick(void)
{
   
  do{ (&hRTC_Handle)->Instance ->WPR = 0xCA; (&hRTC_Handle)->Instance ->WPR = 0x53; } while(0);
   
  ((&hRTC_Handle)->Instance ->CR &= ~(((uint32_t)0x00004000U)));
   
  do{ (&hRTC_Handle)->Instance ->WPR = 0xFF; } while(0);
}






 
void HAL_ResumeTick(void)
{
   
  do{ (&hRTC_Handle)->Instance ->WPR = 0xCA; (&hRTC_Handle)->Instance ->WPR = 0x53; } while(0);
   
  ((&hRTC_Handle)->Instance ->CR |= (((uint32_t)0x00004000U)));
   
  do{ (&hRTC_Handle)->Instance ->WPR = 0xFF; } while(0);
}








 
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
  HAL_IncTick();
}





 
void RTC_WKUP_IRQHandler(void)
{
  HAL_RTCEx_WakeUpTimerIRQHandler(&hRTC_Handle);
}



 



 

 
