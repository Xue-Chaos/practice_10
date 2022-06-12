/* ����ͷ�ļ� */
#include "ioCC2530.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


/* LED�ƺͰ����ĺ궨�� */
#define D3 P1_0    
#define D4 P1_1
#define D5 P1_3
#define D6 P1_4
#define SW1 P1_2

#define uint16 unsigned short
#define uint unsigned int  //4Bytes
#define uint32 unsigned long

/* �������� */
uint16 adc_val = 0;//�ɼ��Ĺ���ֵ
uint t1_flag = 0;//��ʱʱ�䵽1s�ı�־λ
uint t1_count = 0;//��ʱ�жϴ���
uint key_count = 0;//�����Ĵ���
char outputSensorStr[100]={0};//���ת������ַ�����ʽ������

/*************** ��ʱ���� ***************/
void delay(int delaytime)
{
  int i=0,j=0;
  for(i=0; i<300; i++)
    for(j=0; j<delaytime; j++);
}
/*************** LED�Ƴ�ʼ�� ***************/
void initLed()
{
    P1SEL &= ~(0X1 << 0 | 0x1 << 1 | 0x1 << 3 | 0x1 << 4);  //����P1_0 P1_1 P1_3 P1_4ΪGPIO�� 
    P1DIR |= 0X1 << 0 | 0x1 << 1 | 0x1 << 3 | 0x1 << 4;  //����P1_0 P1_1 P1_3 P1_4Ϊ���
    D3 = 0;//����LED�ĳ�ʼ״̬
    D4 = 0;
    D5 = 0;
    D6 = 0;
}
/*************** ������ʼ�� ***************/
void initSW()
{
    P1SEL &= ~(0x01 << 2); //P1_2Ϊ��ͨIO��
    P1DIR &= ~0x04;//P1_2 �˿�Ϊ����
    P1INP &= ~0x04;//P1_2 �˿�Ϊ��������ģʽ
    P2INP &= ~0x04;//P1���ж˿�����Ϊ����
}
/*************** �����ж�����***************/
void initial_interrupt()
{
    EA = 1;
    IEN2 |= 0x10;//ʹ��P1�˿��ж�Դ
    P1IEN |= 0x04;//ʹ��P1_2 
    PICTL |= 0x02;//������ʽΪ�½��ش��� 
}
/*************** ��ʱ��1���� ***************/
void initial_t1()
{
   /***********������4��ʼ ��ʱ������************/
    T1CTL = 0x0B;// 32��Ƶ ģģʽ
    T1CC0H = (10000 & 0xFF00) >> 8;//T1CC0 10000(0x2710) 0.01s  ��10000�ĸ�8λд��T1CC0H  
    T1CC0L = 10000 & 0xFF;// ��10000�ĵ�8λд��T1CC0L

  IEN0 |=0X80;//EA���ж� 7λΪ1
  IEN1 |=0X02;//T0�ж�ʹ�� 1λΪ1
  T1IF=0; //���־λ 
}

/**********����ͨ�ų�ʼ��************************/
void initUART0(void)
{
  CLKCONCMD &= ~0X7F;                 //��������Ϊ32MHz  
  while(CLKCONSTA & 0X40);            //�ȴ������ȶ�  
  CLKCONCMD &= ~0X47;                 //����ϵͳ��ʱ��Ƶ��Ϊ32MHz   
  
  PERCFG|= 0x00;
  P0SEL |=0X0C;//P0_2 P0_3
  P2DIR &= ~0XC0;//P0������ΪUART��ʽ 
  U0CSR |=0XC0;//����ģʽ������ʹ�� 
  U0GCR = 11;
  U0BAUD = 216;//115200  
  URX0IE = 1;
  EA = 1; 
}
uint16 get_adc(void)
{
  uint32 value;
   // ADC��ʼ��
  APCFG  |=1;   
  P0SEL  |= (1 << (0));	
  P0DIR  &= ~(1 << (0));
  ADCIF = 0;   //��ADC �жϱ�־
  //���û�׼��ѹavdd5:3.3V��ͨ��0������ADת��
  ADCCON3 = (0x80 | 0x10 | 0x00);
  while ( !ADCIF )
  {
    ;  //�ȴ�ADת������
  }
  value = ADCL;				//ADCת������ĵ�λ���ִ���value��
  value |= (((uint16)ADCH)<< 8);	//ȡ������ת���������value��
  value = value * 330;
  value = value >> 15;  		//���ݼ��㹫ʽ������ֵ	
  return (uint16)value;
}

#pragma vector = P1INT_VECTOR
__interrupt void P1_ISR(void)
{
  if(P1IFG==0x04)//�Ƿ���P1_2�����ж�
  {
     key_count++;
    /***********������6��ʼ ������������ÿ�ΰ���SW1������D3��D4��D5 ��������������****/
   
    
    
    
    /***********������6���� *************/
  }
  P1IF = 0x00;//�����־λ
  P1IFG = 0x00;
}

#pragma vector = T1_VECTOR
__interrupt void TI_VEC(void)
{
  T1IF=0;//�жϱ�־λ T1IF
  
  t1_count++;
   /***********������7��ʼ 1s��ʱ�䵽����t1_flag��־λ��1 *******/
   
  
  
  
  /***********������7����   ******/
}


/*************** �����ڷ���ָ�����ȵ�����  ***************/
void uart_tx_string(char *data_tx,int len)  
{   
  uint16 int j;  
  for(j=0;j<len;j++)  
  {   
    U0DBUF = *data_tx++;  
    while(UTX0IF == 0);  
    UTX0IF = 0;
  }
}
/******************************************
* �������ƣ�main
* ��    �ܣ�main�������
* ��ڲ�������
* ���ڲ�������
* �� �� ֵ����
**************************************************/
void main(void)
{
  initLed();
  initSW();
  initial_interrupt();
  initial_t1();
  initUART0();  // UART0��ʼ��
  while(1)
  {
    if(t1_flag == 1)//1s��־λΪ1ʱ
    {
        adc_val = get_adc();//���� �ɼ����մ���������
        sprintf(outputSensorStr, "����ǿ�ȣ� %d Lux\r\n", adc_val);
        uart_tx_string(outputSensorStr, sizeof(outputSensorStr));//���������ݷ��͵�����
        memset(outputSensorStr, 0, sizeof(outputSensorStr));
        
        if (adc_val < 110)//
        {
            D6 = 1;
        }
        else
        {
            D6 = 0;
        }
      t1_flag = 0;//t1_flag �����־
    }
  }
}

