#ifndef  __ECHO_SERVER_H
#define  __ECHO_SERVER_H


#include "lwip_comm.h"
#include "lwip/tcpip.h"
#include "includes.h"


//�������:3
//�����ܽ���:���Է���������
//�������ȼ�:8
//��������Ϣ:��
//���������Ϣ����
//��������(ʱ��������/��Ϣ������):��

//�������������ȼ�
#define  ECHO_SERVER_TASK_PRIO     8
//�����������ջ��С
#define  ECHO__SERVER_STK_SIZE     256


#define  ECHO_SERVER_PORT           7


/*****�ⲿ���ú���*******/
u8 echo_server_init(void);

/*****�ڲ����ú���*******/
static void echo_server_task(void * arg);

#endif