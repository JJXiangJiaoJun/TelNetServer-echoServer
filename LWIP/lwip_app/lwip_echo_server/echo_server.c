#include "echo_server.h"
#include "lcd.h"


OS_TCB  	EchoServerTaskTCB;
CPU_STK  	Echo_Server_Taks_STK[ECHO__SERVER_STK_SIZE];

const char * const Send_Msg = "time is out\r\n";

u8 echo_server_init(void)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	
	OS_CRITICAL_ENTER();//进入临界区
	//创建开始任务
	OSTaskCreate(	(OS_TCB 	* )&EchoServerTaskTCB,		//任务控制块
								(CPU_CHAR	* )"echoserver task", 			//任务名字
                 (OS_TASK_PTR )echo_server_task, 			//任务函数
                 (void		* )0,						//传递给任务函数的参数
                 (OS_PRIO	  )ECHO_SERVER_TASK_PRIO, 	//任务优先级
                 (CPU_STK   * )&Echo_Server_Taks_STK[0],	//任务堆栈基地址
                 (CPU_STK_SIZE)ECHO__SERVER_STK_SIZE/10,	//任务堆栈深度限位
                 (CPU_STK_SIZE)ECHO__SERVER_STK_SIZE,		//任务堆栈大小
                 (OS_MSG_QTY  )0,						//任务内部消息队列能够接收的最大消息数目,为0时禁止接收消息
                 (OS_TICK	  )0,						//当使能时间片轮转时的时间片长度，为0时为默认长度，
                 (void   	* )0,						//用户补充的存储区
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, //任务选项
                 (OS_ERR 	* )&err);					//存放该函数错误时的返回值
	OS_CRITICAL_EXIT();	//退出临界区

  if(err != OS_ERR_NONE)
	{
	   printf("echoserver task creat failed!");
		 return 1;
	}		
  return 0;
}



static void echo_server_task(void * arg)
{
     
	 struct netconn *conn,*newconn;
	 struct netbuf  *databuf;
	 err_t  err;
	 char *buf;
	 u16_t  buflen;
	 //创建一个新的NETCONN接口
	 conn = netconn_new(NETCONN_TCP);
	 //为服务器绑定熟知端口号
	 netconn_bind(conn,NULL,ECHO_SERVER_PORT);
	 //服务器置为侦听状态
	 netconn_listen(conn);
	  while(1)
		{ 
	    //接受新连接
			err = netconn_accept(conn,&newconn);
		  //连接正常建立
			if(err == ERR_OK)
			{
			  newconn->recv_timeout = 500 ;
				LCD_Fill(30,180,270,180,WHITE);
				LCD_ShowString(30,180,200,20,16,"Echo Server Connect!"); 	//TCP Server创建成功
				while(1)
				{
					//读取消息
					err = netconn_recv(newconn,&databuf); 
				 //若正常接受数据
					if(err == ERR_OK)
					{
           netbuf_data(databuf,(void**)&buf,&buflen);
					//回显数据
           netconn_write(newconn,buf,buflen,NETCONN_NOCOPY);
            
           netbuf_delete(databuf);   					
         
					}
				//若等待超时
					else if(err ==	ERR_TIMEOUT)
					{
						netconn_write(newconn,Send_Msg,strlen(Send_Msg),NETCONN_NOCOPY);
					}
					else if(err == ERR_CLSD)
					{
					  //连接已关闭
					  LCD_Fill(30,180,270,180,WHITE);
				    LCD_ShowString(30,180,200,20,16,"Echo Server Break!"); 	//TCP Server创建成功
						netconn_close(newconn);
						netconn_delete(newconn);
						break;
					}
			
				}
			
			}
		
		}
  

}


