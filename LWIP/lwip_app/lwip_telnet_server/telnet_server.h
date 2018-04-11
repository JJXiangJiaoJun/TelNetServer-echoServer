#ifndef  __TELNET_SERVER_H
#define  __TELNET_SERVER_H


#include "lwip_comm.h"
#include "lwip/tcpip.h"
#include "includes.h"
#include "malloc.h"


//任务序号:4
//任务功能介绍:TELNET服务器
//任务优先级:7
//任务发送消息:无
//任务接受消息：无
//任务类型(时间阻塞型/消息阻塞型):无

//服务器任务优先级
#define  TELNET_SERVER_TASK_PRIO     7
//服务器任务堆栈大小
#define  TELNET_SERVER_STK_SIZE     512

#define  MAX_INFO_LEN  30
#define  LOGIN_INFO  "Please input Password to login:"
#define  PASSWORD    "20180101"
#define  TELNET_SERVER_PORT  23

enum TELNET_STATE
{
  TELNET_SETUP,
	TELNET_CONNETCTED,
};


typedef struct
{
   int 	state;       		//连接状态
	 u16  client_port; 	  //客户端端口号
	 u16  bytes_len;			//缓存字符的索引
	 char   bytes[MAX_INFO_LEN];
	 struct netconn *conn;
	 u8   break_flag;
}telnet_conn_arg;

/*****外部调用函数*******/
u8 telnet_server_init(void);

/*****内部调用函数*******/
static void telnet_server_task(void * arg);
static u8 telnet_input(telnet_conn_arg *arg,char *buf,u16 len);
static void  telnet_command(telnet_conn_arg *arg);
static void  telnet_process(telnet_conn_arg *arg,struct netbuf *newbuf);
#endif