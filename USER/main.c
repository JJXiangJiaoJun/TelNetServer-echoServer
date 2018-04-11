#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "key.h"
#include "lcd.h"
#include "sdram.h"
#include "lan8720.h"
#include "pcf8574.h"
#include "mpu.h"
#include "usmart.h"
#include "malloc.h"
#include "malloc.h"
#include "lwip/netif.h"
#include "lwip_comm.h"
#include "lwipopts.h"
#include "includes.h"
#include "echo_server.h"
#include "telnet_server.h"
/************************************************
 ALIENTEK ������STM32F7������ ����ʵ��1
 LWIP�޲���ϵͳ��ֲ-HAL�⺯����
 ����֧�֣�www.openedv.com
 �Ա����̣�http://eboard.taobao.com 
 ��ע΢�Ź���ƽ̨΢�źţ�"����ԭ��"����ѻ�ȡSTM32���ϡ�
 ������������ӿƼ����޹�˾  
 ���ߣ�����ԭ�� @ALIENTEK
************************************************/


//�������ȼ�
#define START_TASK_PRIO		3
//�����ջ��С	
#define START_STK_SIZE 		512
//������ƿ�
OS_TCB StartTaskTCB;
//�����ջ	
CPU_STK START_TASK_STK[START_STK_SIZE];
//������
void start_task(void *p_arg);

//�������ȼ�
#define LED0_TASK_PRIO		11
//�����ջ��С	
#define LED0_STK_SIZE 		128
//������ƿ�
OS_TCB Led0TaskTCB;
//�����ջ	
CPU_STK LED0_TASK_STK[LED0_STK_SIZE];
void led0_task(void *p_arg);

//�������ȼ�
#define LED1_TASK_PRIO		12
//�����ջ��С	
#define LED1_STK_SIZE       128
//������ƿ�
OS_TCB Led1TaskTCB;
//�����ջ	
CPU_STK LED1_TASK_STK[LED1_STK_SIZE];
//������
void led1_task(void *p_arg);


int main(void)
{

		OS_ERR err;
		CPU_SR_ALLOC();
    Write_Through();                //����ǿ��͸д��
    MPU_Memory_Protection();        //������ش洢����
    Cache_Enable();                 //��L1-Cache
    
    HAL_Init();				        //��ʼ��HAL��
    Stm32_Clock_Init(432,25,2,9);   //����ʱ��,216Mhz 
    delay_init(216);                //��ʱ��ʼ��
	uart_init(115200);		        //���ڳ�ʼ��
	usmart_dev.init(108); 		    //��ʼ��USMART
    LED_Init();                     //��ʼ��LED
    KEY_Init();                     //��ʼ������
    SDRAM_Init();                   //��ʼ��SDRAM
    LCD_Init();                     //��ʼ��LCD
    PCF8574_Init();                 //��ʼ��PCF8574
    my_mem_init(SRAMIN);		    //��ʼ���ڲ��ڴ��
	my_mem_init(SRAMEX);		    //��ʼ���ⲿ�ڴ��
	my_mem_init(SRAMDTCM);		    //��ʼ��DTCM�ڴ��
    POINT_COLOR = RED; 

   OSInit(&err);		            //��ʼ��UCOSIII
 
	LCD_ShowString(30,30,200,16,16,"Telnet+Echo Server");
	LCD_ShowString(30,50,200,16,16,"Ethernet lwIP est");
	LCD_ShowString(30,70,200,16,16,"By XiangJiaoJun ");
	LCD_ShowString(30,90,200,16,16,"HUN"); 

	while(lwip_comm_init())         //lwip��ʼ��
	{
		LCD_ShowString(30,110,200,20,16,"LWIP Init Falied! ");
		delay_ms(500);
		LCD_ShowString(30,110,200,16,16,"Retrying...       ");
        delay_ms(500);
	}
	while(telnet_server_init()) 									//��ʼ��tcp_server(����tcp_server��)
	{
		LCD_ShowString(30,150,200,20,16,"telnet Server failed!!"); //TCP Server����ʧ��
		delay_ms(500);
		LCD_Fill(30,150,230,170,WHITE);
		delay_ms(500);
	}
	while(echo_server_init()) 									//��ʼ��tcp_server(����tcp_server��)
	{
	  LCD_ShowString(30,130,200,20,16,"echo Server failed!!"); //TCP Server����ʧ��
		delay_ms(500);
		LCD_Fill(30,130,250,130,WHITE);
		delay_ms(500);
	}
	LCD_ShowString(30,130,200,20,16,"echo   Server Success!"); 	//TCP Server�����ɹ�
	LCD_ShowString(30,150,200,20,16,"telnet Server Success!"); 	//TCP Server�����ɹ�
	
	LCD_Fill(30,200,270,200,WHITE);
	LCD_Fill(30,180,270,180,WHITE);
	LCD_ShowString(30,180,200,20,16,"echo 	Server Break!"); 	//TCP Server�����ɹ�
	LCD_ShowString(30,200,200,20,16,"telnet Server Break!"); 	//TCP Server�����ɹ�
	
	OS_CRITICAL_ENTER();//�����ٽ���x
	//������ʼ����
	OSTaskCreate((OS_TCB 	* )&StartTaskTCB,		//������ƿ�
								(CPU_CHAR	* )"start task", 		//��������
                 (OS_TASK_PTR )start_task, 			//������
                 (void		* )0,					//���ݸ��������Ĳ���
                 (OS_PRIO	  )START_TASK_PRIO,     //�������ȼ�
                 (CPU_STK   * )&START_TASK_STK[0],	//�����ջ����ַ
                 (CPU_STK_SIZE)START_STK_SIZE/10,	//�����ջ�����λ
                 (CPU_STK_SIZE)START_STK_SIZE,		//�����ջ��С
                 (OS_MSG_QTY  )0,					//�����ڲ���Ϣ�����ܹ����յ������Ϣ��Ŀ,Ϊ0ʱ��ֹ������Ϣ
                 (OS_TICK	  )0,					//��ʹ��ʱ��Ƭ��תʱ��ʱ��Ƭ���ȣ�Ϊ0ʱΪĬ�ϳ��ȣ�
                 (void   	* )0,					//�û�����Ĵ洢��
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, //����ѡ��
                 (OS_ERR 	* )&err);				//��Ÿú�������ʱ�ķ���ֵ
	OS_CRITICAL_EXIT();	//�˳��ٽ���	 
	OSStart(&err); 		//����UCOS						  	
								  	       
}

//��ʼ������
void start_task(void *p_arg)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	p_arg = p_arg;

	CPU_Init();
#if OS_CFG_STAT_TASK_EN > 0u
   OSStatTaskCPUUsageInit(&err);  	//ͳ������                
#endif
	
#ifdef CPU_CFG_INT_DIS_MEAS_EN		//���ʹ���˲����жϹر�ʱ��
    CPU_IntDisMeasMaxCurReset();	
#endif

#if	OS_CFG_SCHED_ROUND_ROBIN_EN  //��ʹ��ʱ��Ƭ��ת��ʱ��
	 //ʹ��ʱ��Ƭ��ת���ȹ���,����Ĭ�ϵ�ʱ��Ƭ����s
	OSSchedRoundRobinCfg(DEF_ENABLED,10,&err);  
#endif		
	
	OS_CRITICAL_ENTER();	//�����ٽ���
	//����LED0����
	OSTaskCreate((OS_TCB 	* )&Led0TaskTCB,		
				 (CPU_CHAR	* )"led0 task", 		
                 (OS_TASK_PTR )led0_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )LED0_TASK_PRIO,     
                 (CPU_STK   * )&LED0_TASK_STK[0],	
                 (CPU_STK_SIZE)LED0_STK_SIZE/10,	
                 (CPU_STK_SIZE)LED0_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,					
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR|OS_OPT_TASK_SAVE_FP,
                 (OS_ERR 	* )&err);				
				 
	//����LED1����
	OSTaskCreate((OS_TCB 	* )&Led1TaskTCB,		
				 (CPU_CHAR	* )"led1 task", 		
                 (OS_TASK_PTR )led1_task, 			
                 (void		* )0,					
                 (OS_PRIO	  )LED1_TASK_PRIO,     	
                 (CPU_STK   * )&LED1_TASK_STK[0],	
                 (CPU_STK_SIZE)LED1_STK_SIZE/10,	
                 (CPU_STK_SIZE)LED1_STK_SIZE,		
                 (OS_MSG_QTY  )0,					
                 (OS_TICK	  )0,					
                 (void   	* )0,				
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR|OS_OPT_TASK_SAVE_FP, 
                 (OS_ERR 	* )&err);
				 

	OS_CRITICAL_EXIT();	//�����ٽ���				 
	OS_TaskSuspend((OS_TCB*)&StartTaskTCB,&err);		//����ʼ����			 
}

//led0������
void led0_task(void *p_arg)
{
	OS_ERR err;
	p_arg = p_arg;
	while(1)
	{
		LED0(0);    //LED0��
		OSTimeDlyHMSM(0,0,0,200,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ200ms
		LED0(1);    //LED0�ر�
		OSTimeDlyHMSM(0,0,0,500,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ500ms
	}
}

//led1������
void led1_task(void *p_arg)
{
	p_arg = p_arg;
	while(1)
	{
				LED1_Toggle;
        delay_ms(500);//��ʱ500ms
	}
}


