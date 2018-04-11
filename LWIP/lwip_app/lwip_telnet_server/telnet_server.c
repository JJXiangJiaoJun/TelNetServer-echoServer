#include "telnet_server.h"
#include "lcd.h"


OS_TCB  	TelnetServerTaskTCB;
CPU_STK  	Telnet_Server_Taks_STK[TELNET_SERVER_STK_SIZE];

char *command[] ={
	"greeting",
	"date",
	"systick",
	"version",
	"quit",
	"help",
};

u8 telnet_server_init(void)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	
	OS_CRITICAL_ENTER();//进入临界区
	//创建开始任务
	OSTaskCreate(	(OS_TCB 	* )&TelnetServerTaskTCB,		//任务控制块
								(CPU_CHAR	* )"TelNetServer task", 			//任务名字
                 (OS_TASK_PTR )telnet_server_task, 			//任务函数
                 (void		* )0,						//传递给任务函数的参数
                 (OS_PRIO	  )TELNET_SERVER_TASK_PRIO, 	//任务优先级
                 (CPU_STK   * )&Telnet_Server_Taks_STK[0],	//任务堆栈基地址
                 (CPU_STK_SIZE)TELNET_SERVER_STK_SIZE/10,	//任务堆栈深度限位
                 (CPU_STK_SIZE)TELNET_SERVER_STK_SIZE,		//任务堆栈大小
                 (OS_MSG_QTY  )0,						//任务内部消息队列能够接收的最大消息数目,为0时禁止接收消息
                 (OS_TICK	  )0,						//当使能时间片轮转时的时间片长度，为0时为默认长度，
                 (void   	* )0,						//用户补充的存储区
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, //任务选项
                 (OS_ERR 	* )&err);					//存放该函数错误时的返回值
	OS_CRITICAL_EXIT();	//退出临界区

  if(err != OS_ERR_NONE)
	{
	   printf("telnetserver task creat failed!");
		 return 1;
	}		
  return 0;
}



static void telnet_server_task(void * arg)
{
     
	 struct netconn *conn,*newconn;
	 struct netbuf  *databuf;
	 telnet_conn_arg  *telnet_arg;
	 static ip_addr_t ipaddr;
	 static u16_t port;
	 err_t  err;
	
	 char greet[100];
	 u8 remot_addr[4];
	
	 //创建一个新的NETCONN接口
	 conn = netconn_new(NETCONN_TCP);
	 //为服务器绑定熟知端口号
	 netconn_bind(conn,NULL,TELNET_SERVER_PORT);
	 //服务器置为侦听状态
	 netconn_listen(conn);
	  while(1)
		{ 
	    //接受新连接
			err = netconn_accept(conn,&newconn);
		  //连接正常建立
			if(err == ERR_OK)
			{
        LCD_Fill(30,200,230,200,WHITE);
				LCD_ShowString(30,200,200,20,16,"Telnet Server Connect!");
				//TCP Server创建成功,获取连接端口号
				netconn_getaddr(newconn,&ipaddr,&port,0);
				
				remot_addr[3] = (u8)(ipaddr.addr>>24); 
				remot_addr[2] = (u8)(ipaddr.addr>>16);
				remot_addr[1] = (u8)(ipaddr.addr>>8);
				remot_addr[0] = (u8)(ipaddr.addr);
				sprintf(greet,"Welcome to Telnet! your Info --> [%d.%d.%d.%d:%d]\r\n",\
	                                      remot_addr[0],remot_addr[1],remot_addr[2],remot_addr[3],port);
				
				LCD_Fill(30,220,230,220,WHITE);
				LCD_ShowString(30,220,200,20,16,(u8*)greet);
				
				//发送登录信息
				netconn_write(newconn,greet,strlen(greet),NETCONN_NOCOPY);
				netconn_write(newconn,LOGIN_INFO,strlen(LOGIN_INFO),NETCONN_NOCOPY);
		    //申请内存
				telnet_arg =(telnet_conn_arg  *)mymalloc(SRAMIN,sizeof(telnet_conn_arg));
				//初始化连接结构
				mymemset(telnet_arg->bytes,0,MAX_INFO_LEN);
				telnet_arg->bytes_len = 0;
				telnet_arg->client_port = port;
				telnet_arg->state = TELNET_SETUP;
				telnet_arg->conn 	= newconn;
				telnet_arg->break_flag = 0;
				for(;;)
				{
				    //阻塞接受数据
  				err = netconn_recv(newconn,&databuf);
				   //若正常接受 
					if(err == ERR_OK)
						{
					    telnet_process(telnet_arg,databuf);
						  netbuf_delete(databuf);
              if(telnet_arg->break_flag)
							{
									LCD_Fill(30,200,230,200,WHITE);
									LCD_ShowString(30,200,200,20,16,"Telnet Client Break!");
									//关闭连接
								  myfree(SRAMIN,telnet_arg);
									netconn_close(newconn);
									netconn_delete(newconn);
									break;
							}								
					}
						//若客户端断开连接
					else
					{
					
						LCD_Fill(30,200,230,200,WHITE);
						LCD_ShowString(30,200,200,20,16,"Telnet Server Break!");
					  //关闭连接
					  myfree(SRAMIN,telnet_arg);
						netconn_close(newconn);
						netconn_delete(newconn);
						break;
					}
				  
				
				}
			
			}
			
		}
		
  
}

static u8 telnet_input(telnet_conn_arg *conn_arg,char *datab,u16 len)
{
   
  char buf[20];
  int  buflen;	
	//收到回车键
	if((len == 2)&&(*datab==0x0d)&&(*(datab+1)==0x0a))
		{
	     conn_arg->bytes[conn_arg->bytes_len]	=	'\0';
	     return 1;
		}
   //收到普通字符
	else if((len==1)&&(*datab>=0x20)&&(*datab<=0x7e))
		{
	     conn_arg->bytes[conn_arg->bytes_len]	=	*datab;
		   if(conn_arg->bytes_len<(MAX_INFO_LEN-1))
				 conn_arg->bytes_len++;
		}
	//收到退格键
	else if((len==1)&&(*datab==0x08)&&(conn_arg->bytes_len>0))
	{
	  conn_arg->bytes_len--;
	  buflen = sprintf(buf,"\b");
		netconn_write(conn_arg->conn,buf,buflen,NETCONN_NOCOPY);
	}
	else if((len==1)&&(*datab==0x08))
	{
		conn_arg->bytes_len = 0;
		buflen = sprintf(buf,">");
		netconn_write(conn_arg->conn,buf,buflen,NETCONN_NOCOPY);
	}
	
	return 0;
}

static void telnet_command(telnet_conn_arg *arg)
{
	 u16   send_len;
	 char res_buffer[100];

	 if(strcmp(arg->bytes,command[0]) == 0)
	 {
	   send_len = sprintf(res_buffer,"Hi,I am a LwIP Telnet Server..\r\n");
	 
	 }else if(strcmp(arg->bytes,command[1]) == 0)
	 {
	   send_len = sprintf(res_buffer,"Time Now is : 2018/1/9\r\n");
	 }else if(strcmp(arg->bytes,command[2]) == 0)
	 {
	   send_len = sprintf(res_buffer,"Current systick is : %ld\r\n",sys_now());
	 }else if(strcmp(arg->bytes,command[3]) == 0)
	 {
	   send_len = sprintf(res_buffer,"STM32F767 demo V1.0 by 香蕉君\r\n");
	 }else if(strcmp(arg->bytes,command[4]) == 0)
	 {
	   send_len = sprintf(res_buffer,"The connection will shutdown..\r\n");
		 arg->break_flag = 1;
	 }else if(strcmp(arg->bytes,command[5]) == 0)
	 {
	   send_len = sprintf(res_buffer,"Supprted Command : date greeting systick versiong help quit..\r\n");
	 }else
	 {
	    send_len = sprintf(res_buffer,"Command not support..\r\n");
	 
	 }
	 
	 netconn_write(arg->conn,res_buffer,send_len,NETCONN_COPY);
	 send_len = sprintf(res_buffer,"LwIP Shell>");
   netconn_write(arg->conn,res_buffer,send_len,NETCONN_COPY);
}

static void telnet_process(telnet_conn_arg *arg,struct netbuf *newbuf)
{
    char 	*comm;
	  u16   buflen;
		u8    ret;
    u16   send_len;
    char sndbuf[100];	
	  //获取接受数据
	  netbuf_data(newbuf,(void **)&comm,&buflen);
	  //处理接收到字符
	  ret = telnet_input(arg,comm,buflen);
    //判断是否进行处理
	 if(ret)
	 {
	    switch(arg->state)
			{
				case TELNET_SETUP :
					//校验登录密码，若正确
				  if(strcmp(arg->bytes,PASSWORD) == 0)
					{
							send_len=sprintf(sndbuf,"##Welcome to demo TELNET based on LwIP##\r\n");
							netconn_write(arg->conn,sndbuf,send_len,NETCONN_COPY);

							 
							send_len=sprintf(sndbuf,"##Create by XiangJiaoJun..............##\r\n");
							 netconn_write(arg->conn,sndbuf,send_len,NETCONN_COPY);
						
							 send_len=sprintf(sndbuf,"##quit:退出         help:帮助信息        ##\r\n");
							 netconn_write(arg->conn,sndbuf,send_len,NETCONN_COPY);
							 
							 send_len=sprintf(sndbuf,"LwIP Shell>");
							 netconn_write(arg->conn,sndbuf,send_len,NETCONN_COPY);
						
						   arg->state = TELNET_CONNETCTED;		
					}
					else
					{
						send_len=sprintf(sndbuf,"##PASSWORD  ERROR!!!!!Try again:##\r\n");
						netconn_write(arg->conn,sndbuf,send_len,NETCONN_COPY);
					
					}
					mymemset(arg->bytes,0,MAX_INFO_LEN);
					arg->bytes_len = 0;
					break;
       //解析命令,并返回数据	     
				case  TELNET_CONNETCTED:
						telnet_command(arg);
						mymemset(arg->bytes,0,MAX_INFO_LEN);
						arg->bytes_len = 0;
			   break;
				default:
					break;
			}
	 
	 
	 }
	  

}


