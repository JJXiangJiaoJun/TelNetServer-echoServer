#include "echo_server.h"
#include "lcd.h"


OS_TCB  	EchoServerTaskTCB;
CPU_STK  	Echo_Server_Taks_STK[ECHO__SERVER_STK_SIZE];

const char * const Send_Msg = "time is out\r\n";

u8 echo_server_init(void)
{
	OS_ERR err;
	CPU_SR_ALLOC();
	
	OS_CRITICAL_ENTER();//�����ٽ���
	//������ʼ����
	OSTaskCreate(	(OS_TCB 	* )&EchoServerTaskTCB,		//������ƿ�
								(CPU_CHAR	* )"echoserver task", 			//��������
                 (OS_TASK_PTR )echo_server_task, 			//������
                 (void		* )0,						//���ݸ��������Ĳ���
                 (OS_PRIO	  )ECHO_SERVER_TASK_PRIO, 	//�������ȼ�
                 (CPU_STK   * )&Echo_Server_Taks_STK[0],	//�����ջ����ַ
                 (CPU_STK_SIZE)ECHO__SERVER_STK_SIZE/10,	//�����ջ�����λ
                 (CPU_STK_SIZE)ECHO__SERVER_STK_SIZE,		//�����ջ��С
                 (OS_MSG_QTY  )0,						//�����ڲ���Ϣ�����ܹ����յ������Ϣ��Ŀ,Ϊ0ʱ��ֹ������Ϣ
                 (OS_TICK	  )0,						//��ʹ��ʱ��Ƭ��תʱ��ʱ��Ƭ���ȣ�Ϊ0ʱΪĬ�ϳ��ȣ�
                 (void   	* )0,						//�û�����Ĵ洢��
                 (OS_OPT      )OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, //����ѡ��
                 (OS_ERR 	* )&err);					//��Ÿú�������ʱ�ķ���ֵ
	OS_CRITICAL_EXIT();	//�˳��ٽ���

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
	 //����һ���µ�NETCONN�ӿ�
	 conn = netconn_new(NETCONN_TCP);
	 //Ϊ����������֪�˿ں�
	 netconn_bind(conn,NULL,ECHO_SERVER_PORT);
	 //��������Ϊ����״̬
	 netconn_listen(conn);
	  while(1)
		{ 
	    //����������
			err = netconn_accept(conn,&newconn);
		  //������������
			if(err == ERR_OK)
			{
			  newconn->recv_timeout = 500 ;
				LCD_Fill(30,180,270,180,WHITE);
				LCD_ShowString(30,180,200,20,16,"Echo Server Connect!"); 	//TCP Server�����ɹ�
				while(1)
				{
					//��ȡ��Ϣ
					err = netconn_recv(newconn,&databuf); 
				 //��������������
					if(err == ERR_OK)
					{
           netbuf_data(databuf,(void**)&buf,&buflen);
					//��������
           netconn_write(newconn,buf,buflen,NETCONN_NOCOPY);
            
           netbuf_delete(databuf);   					
         
					}
				//���ȴ���ʱ
					else if(err ==	ERR_TIMEOUT)
					{
						netconn_write(newconn,Send_Msg,strlen(Send_Msg),NETCONN_NOCOPY);
					}
					else if(err == ERR_CLSD)
					{
					  //�����ѹر�
					  LCD_Fill(30,180,270,180,WHITE);
				    LCD_ShowString(30,180,200,20,16,"Echo Server Break!"); 	//TCP Server�����ɹ�
						netconn_close(newconn);
						netconn_delete(newconn);
						break;
					}
			
				}
			
			}
		
		}
  

}


