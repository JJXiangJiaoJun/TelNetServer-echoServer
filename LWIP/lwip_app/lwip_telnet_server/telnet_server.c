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
	
	OS_CRITICAL_ENTER();//�����ٽ���
	//������ʼ����
	OSTaskCreate(	(OS_TCB 	* )&TelnetServerTaskTCB,		//������ƿ�
								(CPU_CHAR	* )"TelNetServer task", 			//��������
                 (OS_TASK_PTR )telnet_server_task, 			//������
                 (void		* )0,						//���ݸ��������Ĳ���
                 (OS_PRIO	  )TELNET_SERVER_TASK_PRIO, 	//�������ȼ�
                 (CPU_STK   * )&Telnet_Server_Taks_STK[0],	//�����ջ����ַ
                 (CPU_STK_SIZE)TELNET_SERVER_STK_SIZE/10,	//�����ջ�����λ
                 (CPU_STK_SIZE)TELNET_SERVER_STK_SIZE,		//�����ջ��С
                 (OS_MSG_QTY  )0,						//�����ڲ���Ϣ�����ܹ����յ������Ϣ��Ŀ,Ϊ0ʱ��ֹ������Ϣ
                 (OS_TICK	  )0,						//��ʹ��ʱ��Ƭ��תʱ��ʱ��Ƭ���ȣ�Ϊ0ʱΪĬ�ϳ��ȣ�
                 (void   	* )0,						//�û�����Ĵ洢��
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, //����ѡ��
                 (OS_ERR 	* )&err);					//��Ÿú�������ʱ�ķ���ֵ
	OS_CRITICAL_EXIT();	//�˳��ٽ���

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
	
	 //����һ���µ�NETCONN�ӿ�
	 conn = netconn_new(NETCONN_TCP);
	 //Ϊ����������֪�˿ں�
	 netconn_bind(conn,NULL,TELNET_SERVER_PORT);
	 //��������Ϊ����״̬
	 netconn_listen(conn);
	  while(1)
		{ 
	    //����������
			err = netconn_accept(conn,&newconn);
		  //������������
			if(err == ERR_OK)
			{
        LCD_Fill(30,200,230,200,WHITE);
				LCD_ShowString(30,200,200,20,16,"Telnet Server Connect!");
				//TCP Server�����ɹ�,��ȡ���Ӷ˿ں�
				netconn_getaddr(newconn,&ipaddr,&port,0);
				
				remot_addr[3] = (u8)(ipaddr.addr>>24); 
				remot_addr[2] = (u8)(ipaddr.addr>>16);
				remot_addr[1] = (u8)(ipaddr.addr>>8);
				remot_addr[0] = (u8)(ipaddr.addr);
				sprintf(greet,"Welcome to Telnet! your Info --> [%d.%d.%d.%d:%d]\r\n",\
	                                      remot_addr[0],remot_addr[1],remot_addr[2],remot_addr[3],port);
				
				LCD_Fill(30,220,230,220,WHITE);
				LCD_ShowString(30,220,200,20,16,(u8*)greet);
				
				//���͵�¼��Ϣ
				netconn_write(newconn,greet,strlen(greet),NETCONN_NOCOPY);
				netconn_write(newconn,LOGIN_INFO,strlen(LOGIN_INFO),NETCONN_NOCOPY);
		    //�����ڴ�
				telnet_arg =(telnet_conn_arg  *)mymalloc(SRAMIN,sizeof(telnet_conn_arg));
				//��ʼ�����ӽṹ
				mymemset(telnet_arg->bytes,0,MAX_INFO_LEN);
				telnet_arg->bytes_len = 0;
				telnet_arg->client_port = port;
				telnet_arg->state = TELNET_SETUP;
				telnet_arg->conn 	= newconn;
				telnet_arg->break_flag = 0;
				for(;;)
				{
				    //������������
  				err = netconn_recv(newconn,&databuf);
				   //���������� 
					if(err == ERR_OK)
						{
					    telnet_process(telnet_arg,databuf);
						  netbuf_delete(databuf);
              if(telnet_arg->break_flag)
							{
									LCD_Fill(30,200,230,200,WHITE);
									LCD_ShowString(30,200,200,20,16,"Telnet Client Break!");
									//�ر�����
								  myfree(SRAMIN,telnet_arg);
									netconn_close(newconn);
									netconn_delete(newconn);
									break;
							}								
					}
						//���ͻ��˶Ͽ�����
					else
					{
					
						LCD_Fill(30,200,230,200,WHITE);
						LCD_ShowString(30,200,200,20,16,"Telnet Server Break!");
					  //�ر�����
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
	//�յ��س���
	if((len == 2)&&(*datab==0x0d)&&(*(datab+1)==0x0a))
		{
	     conn_arg->bytes[conn_arg->bytes_len]	=	'\0';
	     return 1;
		}
   //�յ���ͨ�ַ�
	else if((len==1)&&(*datab>=0x20)&&(*datab<=0x7e))
		{
	     conn_arg->bytes[conn_arg->bytes_len]	=	*datab;
		   if(conn_arg->bytes_len<(MAX_INFO_LEN-1))
				 conn_arg->bytes_len++;
		}
	//�յ��˸��
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
	   send_len = sprintf(res_buffer,"STM32F767 demo V1.0 by �㽶��\r\n");
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
	  //��ȡ��������
	  netbuf_data(newbuf,(void **)&comm,&buflen);
	  //������յ��ַ�
	  ret = telnet_input(arg,comm,buflen);
    //�ж��Ƿ���д���
	 if(ret)
	 {
	    switch(arg->state)
			{
				case TELNET_SETUP :
					//У���¼���룬����ȷ
				  if(strcmp(arg->bytes,PASSWORD) == 0)
					{
							send_len=sprintf(sndbuf,"##Welcome to demo TELNET based on LwIP##\r\n");
							netconn_write(arg->conn,sndbuf,send_len,NETCONN_COPY);

							 
							send_len=sprintf(sndbuf,"##Create by XiangJiaoJun..............##\r\n");
							 netconn_write(arg->conn,sndbuf,send_len,NETCONN_COPY);
						
							 send_len=sprintf(sndbuf,"##quit:�˳�         help:������Ϣ        ##\r\n");
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
       //��������,����������	     
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


