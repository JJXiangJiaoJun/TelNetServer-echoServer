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

//操作系统模拟层初始化函数，该函数什么也不做
void sys_init(void)
{
   printf("Init sys_arch");
}

/************************************************
函数描述：创建一个信号量
输入参数: *sem  信号量指针   count  信号量初始值
返回值:   ERR_OK 信号量创建成功   
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
函数描述：删除一个信号量
输入参数: *sem  信号量指针   
返回值:  无 
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
函数描述：释放一个信号量
输入参数: *sem  信号量指针   
返回值:  无 
*************************************************/
void sys_sem_signal(sys_sem_t *sem)
{
    OS_ERR err;
	  OSSemPost((OS_SEM*) 			sem,
										OS_OPT_POST_1,
							 (OS_ERR*)     &err);

}

/************************************************
函数描述：等待一个信号量
输入参数: *sem  信号量指针  timeout 等待时间，若为0则一直阻塞  
返回值:  如果等待时间不为0，则返回所等待的时间，
				 若超时发生则返回SYS_ARCH_TIMEOUT 
*************************************************/
u32_t sys_arch_sem_wait(sys_sem_t *sem, u32_t timeout)
{
   OS_ERR err;
	 u32_t  os_time_pre,os_time_now;
	 u32_t  wait_time;
	//若延时不为0,则转换为UCOS延时节拍
	 if(timeout!=0)
	 {
	   timeout = (timeout*OS_CFG_TICK_RATE_HZ)/1000;
		 if(timeout<1)
			 timeout =0; 
	 }
	 //获取系统当前时间
	 os_time_pre = OSTimeGet(&err);
	//根据设置时间等待信号量 
  OSSemPend((OS_SEM*)sem,
	            timeout,OS_OPT_PEND_BLOCKING,NULL,&err);
	//若等待超时,返回失败
	if(err != OS_ERR_NONE)
	{
	
	  return SYS_ARCH_TIMEOUT;
	
	}
	//获取等待时间
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
函数描述：检查信号量有效
输入参数: *sem  信号量指针  
返回值:  1 信号量有效  0信号量无效
*************************************************/

int sys_sem_valid(sys_sem_t *sem)
{
 	if(sem->NamePtr)
		return (strcmp(sem->NamePtr,"?SEM"))? 1:0;
	else
		return 0; 
}

/************************************************
函数描述：设置一个信号量无效
输入参数: *sem  信号量指针  
返回值:  无
*************************************************/
void sys_sem_set_invalid(sys_sem_t *sem)
{
	if(sys_sem_valid(sem))
     sys_sem_free(sem);	
}


/************************************************
函数描述：创建一个新的消息邮箱
输入参数: *mbox  消息邮箱指针  size  消息邮箱的最大数目 
返回值:  无
*************************************************/

err_t sys_mbox_new(sys_mbox_t *mbox, int size)
{
   OS_ERR err;
	 //创建一个消息邮箱
   OSQCreate((OS_Q*)mbox,
							(CPU_CHAR*)"LwIP Queue",
							size ,&err);
	if(err != OS_ERR_NONE)
		return ERR_MEM;
  return ERR_OK;
}

/************************************************
函数描述：删除一个消息邮箱
输入参数: *mbox  消息邮箱指针 
返回值:  无
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
函数描述：向消息邮箱投递一个消息
输入参数: *mbox  消息邮箱指针     *msg  消息指针
返回值:  无
*************************************************/

void sys_mbox_post(sys_mbox_t *mbox, void *msg)
{
	u8 i=0;
	OS_ERR err;
	//判断是否为空指针
	if(msg == NULL)
		msg =(void *) &pvNULLPointer;
 while(i<10)	//试10次
	{
		OSQPost((OS_Q*		)mbox,		
			    (void*		)msg,
			    (OS_MSG_SIZE)strlen(msg),
			    (OS_OPT		)OS_OPT_POST_FIFO,
			    (OS_ERR*	)&err);
		if(err==OS_ERR_NONE) break;
		i++;
		OSTimeDlyHMSM(0,0,0,5,OS_OPT_TIME_HMSM_STRICT,&err); //延时5ms
	}
	LWIP_ASSERT( "sys_mbox_post error!\n", i !=10 );  	
}

/************************************************
函数描述：尝试向消息邮箱投递一个消息
输入参数: *mbox  消息邮箱指针     *msg  消息指针
返回值:  无
*************************************************/
err_t sys_mbox_trypost(sys_mbox_t *mbox, void *msg)
{
   OS_ERR err;
		//判断是否为空指针
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
函数描述：从消息邮箱获取一个消息
输入参数: *mbox  消息邮箱指针     **msg  获取到的消息指针
          timeout  超时时间
返回值:  无
*************************************************/

u32_t sys_arch_mbox_fetch(sys_mbox_t *mbox, void **msg, u32_t timeout)
{
	 OS_ERR err;
	 OS_MSG_SIZE  msg_size;
	 u32_t  os_time_pre,os_time_now;
	 u32_t  wait_time;
	//若延时不为0,则转换为UCOS延时节拍
	 if(timeout!=0)
	 {
	   timeout = (timeout*OS_CFG_TICK_RATE_HZ)/1000;
		 if(timeout<1)
			 timeout =0; 
	 }
	 //获取系统当前时间
	 os_time_pre = OSTimeGet(&err);
	//根据设置时间获取消息 
   *msg = OSQPend ((OS_Q *)	mbox,
									(OS_TICK) timeout,
									(OS_OPT) OS_OPT_PEND_BLOCKING,
									(OS_MSG_SIZE *)&msg_size,
									(CPU_TS *)NULL,
									(OS_ERR *)&err);
	//若等待超时,返回失败
	if(err==OS_ERR_TIMEOUT)
	{
	
	  return SYS_ARCH_TIMEOUT;
	
	}
	//获取等待时间
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
	//若为空指针
	if(*msg == (void * )& pvNULLPointer)
				*msg = NULL ;
	return wait_time;

}

/************************************************
函数描述：从消息邮箱尝试获取一个消息
输入参数: *mbox  消息邮箱指针     **msg  获取到的消息指针
返回值:  无
*************************************************/

u32_t sys_arch_mbox_tryfetch(sys_mbox_t *mbox, void **msg)
{
	return sys_arch_mbox_fetch(mbox,msg,1);//尝试获取一个消息

}



/************************************************
函数描述：检查一个消息邮箱有效
输入参数: *mbox  消息邮箱指针  
返回值:  1  消息邮箱有效    0消息邮箱无效
*************************************************/
int sys_mbox_valid(sys_mbox_t *mbox)
{
	if(mbox->NamePtr)  
		return (strcmp(mbox->NamePtr,"?Q"))? 1:0;
	else
		return 0;
}


/************************************************
函数描述：设置一个消息邮箱无效
输入参数: *mbox  消息邮箱指针  
返回值:  1  消息邮箱有效    0消息邮箱无效
*************************************************/

void sys_mbox_set_invalid(sys_mbox_t *mbox)
{
   if(sys_mbox_valid(mbox))
    sys_mbox_free(mbox);

}


extern CPU_STK * TCPIP_THREAD_TASK_STK;//TCP IP内核任务堆栈,在lwip_comm函数定义
//LWIP内核任务的任务控制块
OS_TCB TcpipthreadTaskTCB;


/************************************************
函数描述：创建一个进程
输入参数:
返回值:  
*************************************************/
sys_thread_t sys_thread_new(const char *name, lwip_thread_fn thread, void *arg, int stacksize, int prio)
{
  OS_ERR err;
	CPU_SR_ALLOC();
	if(strcmp(name,TCPIP_THREAD_NAME) == 0)
	{
		OS_CRITICAL_ENTER();	//进入临界区			 
		//创建开始任务
		OSTaskCreate(		(OS_TCB 	* )&TcpipthreadTaskTCB,			//任务控制块
										(CPU_CHAR	* )"TCPIPThread task", 			//任务名字
                     (OS_TASK_PTR )thread, 						//任务函数
                     (void		* )0,							//传递给任务函数的参数
                     (OS_PRIO	  )prio,     					//任务优先级
                     (CPU_STK   * )&TCPIP_THREAD_TASK_STK[0],	//任务堆栈基地址
                     (CPU_STK_SIZE)stacksize/10,				//任务堆栈深度限位
                     (CPU_STK_SIZE)stacksize,					//任务堆栈大小
                     (OS_MSG_QTY  )0,							//任务内部消息队列能够接收的最大消息数目,为0时禁止接收消息
                     (OS_TICK	  )0,							//当使能时间片轮转时的时间片长度，为0时为默认长度，
                     (void   	* )0,							//用户补充的存储区
                     (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, //任务选项
                     (OS_ERR 	* )&err);					//存放该函数错误时的返回值
		OS_CRITICAL_EXIT();	//退出临界区
	}
  return 0;
}


/************************************************
函数描述：获取系统时间
输入参数: 无
返回值:   系统的当前时间(毫秒)
*************************************************/

u32_t sys_now(void)
{
  OS_ERR err; 
	u32 os_time_now;
	os_time_now = OSTimeGet(&err);
	os_time_now = (os_time_now*1000)/OS_CFG_TICK_RATE_HZ;
	
	return os_time_now;
}


//lwip延时函数
//ms:要延时的ms数
void sys_msleep(u32_t ms)
{
	delay_ms(ms);
}
