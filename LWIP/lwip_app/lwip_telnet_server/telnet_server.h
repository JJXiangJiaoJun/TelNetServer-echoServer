#ifndef  __TELNET_SERVER_H
#define  __TELNET_SERVER_H


#include "lwip_comm.h"
#include "lwip/tcpip.h"
#include "includes.h"
#include "malloc.h"


//�������:4
//�����ܽ���:TELNET������
//�������ȼ�:7
//��������Ϣ:��
//���������Ϣ����
//��������(ʱ��������/��Ϣ������):��

//�������������ȼ�
#define  TELNET_SERVER_TASK_PRIO     7
//�����������ջ��С
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
   int 	state;       		//����״̬
	 u16  client_port; 	  //�ͻ��˶˿ں�
	 u16  bytes_len;			//�����ַ�������
	 char   bytes[MAX_INFO_LEN];
	 struct netconn *conn;
	 u8   break_flag;
}telnet_conn_arg;

/*****�ⲿ���ú���*******/
u8 telnet_server_init(void);

/*****�ڲ����ú���*******/
static void telnet_server_task(void * arg);
static u8 telnet_input(telnet_conn_arg *arg,char *buf,u16 len);
static void  telnet_command(telnet_conn_arg *arg);
static void  telnet_process(telnet_conn_arg *arg,struct netbuf *newbuf);
#endif