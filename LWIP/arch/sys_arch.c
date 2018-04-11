/*
 * Copyright (c) 2001-2003 Swedish Institute of Computer Science.
 * All rights reserved. 
 * 
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission. 
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED 
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 * 
 * Author: Adam Dunkels <adam@sics.se>
 *
 */
/* lwIP includes. */
#define SYS_ARCH_GLOBALS

#include "lwip/debug.h"
#include "lwip/def.h"
#include "lwip/lwip_sys.h"
#include "lwip/mem.h"
#include "includes.h"
#include "os_cfg_app.h"
#include "sys.h"
#include "delay.h"
#include "lwip_comm.h"


const void * const pvNULLPointer = (mem_ptr_t*) 0xFFFFFFFF;

//����ϵͳģ����ʼ���������ú���ʲôҲ����
void sys_init(void)
{
   printf("Init sys_arch");
}

/************************************************
��������������һ���ź���
�������: *sem  �ź���ָ��   count  �ź�����ʼֵ
����ֵ:   ERR_OK �ź��������ɹ�   
*************************************************/
err_t sys_sem_new(sys_sem_t *sem, u8_t count)
{
   OS_ERR err;
	 OSSemCreate((OS_SEM*) 			sem,
	             (CPU_CHAR*)		"lwip sem",
							 (OS_SEM_CTR)		count,
	              (OS_ERR*)     &err);
	if(err != OS_ERR_NONE)
	{
	   return  ERR_MEM ;
	
	}
  LWIP_ASSERT("OSSemCreate ",sem != NULL );
	 
	return ERR_OK;
}


/************************************************
����������ɾ��һ���ź���
�������: *sem  �ź���ָ��   
����ֵ:  �� 
*************************************************/

void sys_sem_free(sys_sem_t *sem)
{
   OS_ERR err;
	 OSSemDel((OS_SEM*) 			sem,
							OS_OPT_DEL_ALWAYS,
						(OS_ERR*)     &err);
	sem = NULL;
}


/************************************************
�����������ͷ�һ���ź���
�������: *sem  �ź���ָ��   
����ֵ:  �� 
*************************************************/
void sys_sem_signal(sys_sem_t *sem)
{
    OS_ERR err;
	  OSSemPost((OS_SEM*) 			sem,
										OS_OPT_POST_1,
							 (OS_ERR*)     &err);

}

/************************************************
�����������ȴ�һ���ź���
�������: *sem  �ź���ָ��  timeout �ȴ�ʱ�䣬��Ϊ0��һֱ����  
����ֵ:  ����ȴ�ʱ�䲻Ϊ0���򷵻����ȴ���ʱ�䣬
				 ����ʱ�����򷵻�SYS_ARCH_TIMEOUT 
*************************************************/
u32_t sys_arch_sem_wait(sys_sem_t *sem, u32_t timeout)
{
   OS_ERR err;
	 u32_t  os_time_pre,os_time_now;
	 u32_t  wait_time;
	//����ʱ��Ϊ0,��ת��ΪUCOS��ʱ����
	 if(timeout!=0)
	 {
	   timeout = (timeout*OS_CFG_TICK_RATE_HZ)/1000;
		 if(timeout<1)
			 timeout =0; 
	 }
	 //��ȡϵͳ��ǰʱ��
	 os_time_pre = OSTimeGet(&err);
	//��������ʱ��ȴ��ź��� 
  OSSemPend((OS_SEM*)sem,
	            timeout,OS_OPT_PEND_BLOCKING,NULL,&err);
	//���ȴ���ʱ,����ʧ��
	if(err != OS_ERR_NONE)
	{
	
	  return SYS_ARCH_TIMEOUT;
	
	}
	//��ȡ�ȴ�ʱ��
  os_time_now = OSTimeGet(&err);
  if(os_time_now > os_time_pre)
	{
	   wait_time = ((os_time_pre - os_time_now)*1000)/OS_CFG_TICK_RATE_HZ ;
	}
	else
	{
	   wait_time = ((0xFFFFFFFFUL - os_time_pre + os_time_now)*1000)/OS_CFG_TICK_RATE_HZ ;
	   
	}
	return wait_time;
}


/************************************************
��������������ź�����Ч
�������: *sem  �ź���ָ��  
����ֵ:  1 �ź�����Ч  0�ź�����Ч
*************************************************/

int sys_sem_valid(sys_sem_t *sem)
{
 	if(sem->NamePtr)
		return (strcmp(sem->NamePtr,"?SEM"))? 1:0;
	else
		return 0; 
}

/************************************************
��������������һ���ź�����Ч
�������: *sem  �ź���ָ��  
����ֵ:  ��
*************************************************/
void sys_sem_set_invalid(sys_sem_t *sem)
{
	if(sys_sem_valid(sem))
     sys_sem_free(sem);	
}


/************************************************
��������������һ���µ���Ϣ����
�������: *mbox  ��Ϣ����ָ��  size  ��Ϣ����������Ŀ 
����ֵ:  ��
*************************************************/

err_t sys_mbox_new(sys_mbox_t *mbox, int size)
{
   OS_ERR err;
	 //����һ����Ϣ����
   OSQCreate((OS_Q*)mbox,
							(CPU_CHAR*)"LwIP Queue",
							size ,&err);
	if(err != OS_ERR_NONE)
		return ERR_MEM;
  return ERR_OK;
}

/************************************************
����������ɾ��һ����Ϣ����
�������: *mbox  ��Ϣ����ָ�� 
����ֵ:  ��
*************************************************/

void sys_mbox_free(sys_mbox_t *mbox)
{
	OS_ERR err;
	
#if OS_CFG_Q_FLUSH_EN > 0u  
	OSQFlush(mbox,&err);
#endif
	
	OSQDel((OS_Q*	)mbox,
           (OS_OPT	)OS_OPT_DEL_ALWAYS,
           (OS_ERR*	)&err);
	LWIP_ASSERT( "OSQDel ",err == OS_ERR_NONE ); 
}


/************************************************
��������������Ϣ����Ͷ��һ����Ϣ
�������: *mbox  ��Ϣ����ָ��     *msg  ��Ϣָ��
����ֵ:  ��
*************************************************/

void sys_mbox_post(sys_mbox_t *mbox, void *msg)
{
	u8 i=0;
	OS_ERR err;
	//�ж��Ƿ�Ϊ��ָ��
	if(msg == NULL)
		msg =(void *) &pvNULLPointer;
 while(i<10)	//��10��
	{
		OSQPost((OS_Q*		)mbox,		
			    (void*		)msg,
			    (OS_MSG_SIZE)strlen(msg),
			    (OS_OPT		)OS_OPT_POST_FIFO,
			    (OS_ERR*	)&err);
		if(err==OS_ERR_NONE) break;
		i++;
		OSTimeDlyHMSM(0,0,0,5,OS_OPT_TIME_HMSM_STRICT,&err); //��ʱ5ms
	}
	LWIP_ASSERT( "sys_mbox_post error!\n", i !=10 );  	
}

/************************************************
������������������Ϣ����Ͷ��һ����Ϣ
�������: *mbox  ��Ϣ����ָ��     *msg  ��Ϣָ��
����ֵ:  ��
*************************************************/
err_t sys_mbox_trypost(sys_mbox_t *mbox, void *msg)
{
   OS_ERR err;
		//�ж��Ƿ�Ϊ��ָ��
	if(msg == NULL)
		msg =(void *) &pvNULLPointer;
   OSQPost((OS_Q*		)mbox,		
			    (void*		)msg,
			    (OS_MSG_SIZE)strlen(msg),
			    (OS_OPT		)OS_OPT_POST_FIFO,
			    (OS_ERR*	)&err);
	if(err==OS_ERR_NONE) return ERR_OK;
	else return ERR_MEM;
}

/************************************************
��������������Ϣ�����ȡһ����Ϣ
�������: *mbox  ��Ϣ����ָ��     **msg  ��ȡ������Ϣָ��
          timeout  ��ʱʱ��
����ֵ:  ��
*************************************************/

u32_t sys_arch_mbox_fetch(sys_mbox_t *mbox, void **msg, u32_t timeout)
{
	 OS_ERR err;
	 OS_MSG_SIZE  msg_size;
	 u32_t  os_time_pre,os_time_now;
	 u32_t  wait_time;
	//����ʱ��Ϊ0,��ת��ΪUCOS��ʱ����
	 if(timeout!=0)
	 {
	   timeout = (timeout*OS_CFG_TICK_RATE_HZ)/1000;
		 if(timeout<1)
			 timeout =0; 
	 }
	 //��ȡϵͳ��ǰʱ��
	 os_time_pre = OSTimeGet(&err);
	//��������ʱ���ȡ��Ϣ 
   *msg = OSQPend ((OS_Q *)	mbox,
									(OS_TICK) timeout,
									(OS_OPT) OS_OPT_PEND_BLOCKING,
									(OS_MSG_SIZE *)&msg_size,
									(CPU_TS *)NULL,
									(OS_ERR *)&err);
	//���ȴ���ʱ,����ʧ��
	if(err==OS_ERR_TIMEOUT)
	{
	
	  return SYS_ARCH_TIMEOUT;
	
	}
	//��ȡ�ȴ�ʱ��
  os_time_now = OSTimeGet(&err);
	LWIP_ASSERT("OSQPend ",err==OS_ERR_NONE); 
  if(os_time_now > os_time_pre)
	{
	   wait_time = ((os_time_pre - os_time_now)*1000)/OS_CFG_TICK_RATE_HZ ;
	}
	else
	{
	   wait_time = ((0xFFFFFFFFUL - os_time_pre + os_time_now)*1000)/OS_CFG_TICK_RATE_HZ ;
	   
	}
	//��Ϊ��ָ��
	if(*msg == (void * )& pvNULLPointer)
				*msg = NULL ;
	return wait_time;

}

/************************************************
��������������Ϣ���䳢�Ի�ȡһ����Ϣ
�������: *mbox  ��Ϣ����ָ��     **msg  ��ȡ������Ϣָ��
����ֵ:  ��
*************************************************/

u32_t sys_arch_mbox_tryfetch(sys_mbox_t *mbox, void **msg)
{
	return sys_arch_mbox_fetch(mbox,msg,1);//���Ի�ȡһ����Ϣ

}



/************************************************
�������������һ����Ϣ������Ч
�������: *mbox  ��Ϣ����ָ��  
����ֵ:  1  ��Ϣ������Ч    0��Ϣ������Ч
*************************************************/
int sys_mbox_valid(sys_mbox_t *mbox)
{
	if(mbox->NamePtr)  
		return (strcmp(mbox->NamePtr,"?Q"))? 1:0;
	else
		return 0;
}


/************************************************
��������������һ����Ϣ������Ч
�������: *mbox  ��Ϣ����ָ��  
����ֵ:  1  ��Ϣ������Ч    0��Ϣ������Ч
*************************************************/

void sys_mbox_set_invalid(sys_mbox_t *mbox)
{
   if(sys_mbox_valid(mbox))
    sys_mbox_free(mbox);

}


extern CPU_STK * TCPIP_THREAD_TASK_STK;//TCP IP�ں������ջ,��lwip_comm��������
//LWIP�ں������������ƿ�
OS_TCB TcpipthreadTaskTCB;


/************************************************
��������������һ������
�������:
����ֵ:  
*************************************************/
sys_thread_t sys_thread_new(const char *name, lwip_thread_fn thread, void *arg, int stacksize, int prio)
{
  OS_ERR err;
	CPU_SR_ALLOC();
	if(strcmp(name,TCPIP_THREAD_NAME) == 0)
	{
		OS_CRITICAL_ENTER();	//�����ٽ���			 
		//������ʼ����
		OSTaskCreate(		(OS_TCB 	* )&TcpipthreadTaskTCB,			//������ƿ�
										(CPU_CHAR	* )"TCPIPThread task", 			//��������
                     (OS_TASK_PTR )thread, 						//������
                     (void		* )0,							//���ݸ��������Ĳ���
                     (OS_PRIO	  )prio,     					//�������ȼ�
                     (CPU_STK   * )&TCPIP_THREAD_TASK_STK[0],	//�����ջ����ַ
                     (CPU_STK_SIZE)stacksize/10,				//�����ջ�����λ
                     (CPU_STK_SIZE)stacksize,					//�����ջ��С
                     (OS_MSG_QTY  )0,							//�����ڲ���Ϣ�����ܹ����յ������Ϣ��Ŀ,Ϊ0ʱ��ֹ������Ϣ
                     (OS_TICK	  )0,							//��ʹ��ʱ��Ƭ��תʱ��ʱ��Ƭ���ȣ�Ϊ0ʱΪĬ�ϳ��ȣ�
                     (void   	* )0,							//�û�����Ĵ洢��
                     (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, //����ѡ��
                     (OS_ERR 	* )&err);					//��Ÿú�������ʱ�ķ���ֵ
		OS_CRITICAL_EXIT();	//�˳��ٽ���
	}
  return 0;
}


/************************************************
������������ȡϵͳʱ��
�������: ��
����ֵ:   ϵͳ�ĵ�ǰʱ��(����)
*************************************************/

u32_t sys_now(void)
{
  OS_ERR err; 
	u32 os_time_now;
	os_time_now = OSTimeGet(&err);
	os_time_now = (os_time_now*1000)/OS_CFG_TICK_RATE_HZ;
	
	return os_time_now;
}


//lwip��ʱ����
//ms:Ҫ��ʱ��ms��
void sys_msleep(u32_t ms)
{
	delay_ms(ms);
}
