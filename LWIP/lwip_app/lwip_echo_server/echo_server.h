#ifndef  __ECHO_SERVER_H
#define  __ECHO_SERVER_H


#include "lwip_comm.h"
#include "lwip/tcpip.h"
#include "includes.h"


//任务序号:3
//任务功能介绍:回显服务器任务
//任务优先级:8
//任务发送消息:无
//任务接受消息：无
//任务类型(时间阻塞型/消息阻塞型):无

//服务器任务优先级
#define  ECHO_SERVER_TASK_PRIO     8
//服务器任务堆栈大小
#define  ECHO__SERVER_STK_SIZE     256


#define  ECHO_SERVER_PORT           7


/*****外部调用函数*******/
u8 echo_server_init(void);

/*****内部调用函数*******/
static void echo_server_task(void * arg);

#endif