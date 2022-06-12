/* 包含头文件 */
#include "ioCC2530.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


/* LED灯和按键的宏定义 */
#define D3 P1_0    
#define D4 P1_1
#define D5 P1_3
#define D6 P1_4
#define SW1 P1_2

#define uint16 unsigned short
#define uint unsigned int  //4Bytes
#define uint32 unsigned long

/* 变量定义 */
uint16 adc_val = 0;//采集的光照值
uint t1_flag = 0;//定时时间到1s的标志位
uint t1_count = 0;//定时中断次数
uint key_count = 0;//按键的次数
char outputSensorStr[100]={0};//存放转化后的字符串形式的数据

/*************** 延时函数 ***************/
void delay(int delaytime)
{
  int i=0,j=0;
  for(i=0; i<300; i++)
    for(j=0; j<delaytime; j++);
}
/*************** LED灯初始化 ***************/
void initLed()
{
    P1SEL &= ~(0X1 << 0 | 0x1 << 1 | 0x1 << 3 | 0x1 << 4);  //设置P1_0 P1_1 P1_3 P1_4为GPIO口 
    P1DIR |= 0X1 << 0 | 0x1 << 1 | 0x1 << 3 | 0x1 << 4;  //设置P1_0 P1_1 P1_3 P1_4为输出
    D3 = 0;//设置LED的初始状态
    D4 = 0;
    D5 = 0;
    D6 = 0;
}
/*************** 按键初始化 ***************/
void initSW()
{
    P1SEL &= ~(0x01 << 2); //P1_2为普通IO口
    P1DIR &= ~0x04;//P1_2 端口为输入
    P1INP &= ~0x04;//P1_2 端口为上拉下拉模式
    P2INP &= ~0x04;//P1所有端口设置为上拉
}
/*************** 按键中断设置***************/
void initial_interrupt()
{
    EA = 1;
    IEN2 |= 0x10;//使能P1端口中断源
    P1IEN |= 0x04;//使能P1_2 
    PICTL |= 0x02;//触发方式为下降沿触发 
}
/*************** 定时器1设置 ***************/
void initial_t1()
{
   /***********答题区4开始 定时器配置************/
    T1CTL = 0x0B;// 32分频 模模式
    T1CC0H = (10000 & 0xFF00) >> 8;//T1CC0 10000(0x2710) 0.01s  把10000的高8位写入T1CC0H  
    T1CC0L = 10000 & 0xFF;// 把10000的低8位写入T1CC0L

  IEN0 |=0X80;//EA总中断 7位为1
  IEN1 |=0X02;//T0中断使能 1位为1
  T1IF=0; //清标志位 
}

/**********串口通信初始化************************/
void initUART0(void)
{
  CLKCONCMD &= ~0X7F;                 //晶振设置为32MHz  
  while(CLKCONSTA & 0X40);            //等待晶振稳定  
  CLKCONCMD &= ~0X47;                 //设置系统主时钟频率为32MHz   
  
  PERCFG|= 0x00;
  P0SEL |=0X0C;//P0_2 P0_3
  P2DIR &= ~0XC0;//P0优先作为UART方式 
  U0CSR |=0XC0;//串口模式，接收使能 
  U0GCR = 11;
  U0BAUD = 216;//115200  
  URX0IE = 1;
  EA = 1; 
}
uint16 get_adc(void)
{
  uint32 value;
   // ADC初始化
  APCFG  |=1;   
  P0SEL  |= (1 << (0));	
  P0DIR  &= ~(1 << (0));
  ADCIF = 0;   //清ADC 中断标志
  //采用基准电压avdd5:3.3V，通道0，启动AD转化
  ADCCON3 = (0x80 | 0x10 | 0x00);
  while ( !ADCIF )
  {
    ;  //等待AD转化结束
  }
  value = ADCL;				//ADC转换结果的低位部分存入value中
  value |= (((uint16)ADCH)<< 8);	//取得最终转换结果存入value中
  value = value * 330;
  value = value >> 15;  		//根据计算公式算出结果值	
  return (uint16)value;
}

#pragma vector = P1INT_VECTOR
__interrupt void P1_ISR(void)
{
  if(P1IFG==0x04)//是否是P1_2产生中断
  {
     key_count++;
    /***********答题区6开始 按键处理函数：每次按下SW1按键，D3、D4、D5 三个灯依次亮灭。****/
   
    
    
    
    /***********答题区6结束 *************/
  }
  P1IF = 0x00;//清除标志位
  P1IFG = 0x00;
}

#pragma vector = T1_VECTOR
__interrupt void TI_VEC(void)
{
  T1IF=0;//中断标志位 T1IF
  
  t1_count++;
   /***********答题区7开始 1s定时间到设置t1_flag标志位置1 *******/
   
  
  
  
  /***********答题区7结束   ******/
}


/*************** 往串口发送指定长度的数据  ***************/
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
* 函数名称：main
* 功    能：main函数入口
* 入口参数：无
* 出口参数：无
* 返 回 值：无
**************************************************/
void main(void)
{
  initLed();
  initSW();
  initial_interrupt();
  initial_t1();
  initUART0();  // UART0初始化
  while(1)
  {
    if(t1_flag == 1)//1s标志位为1时
    {
        adc_val = get_adc();//考生 采集光照传感器数据
        sprintf(outputSensorStr, "光照强度： %d Lux\r\n", adc_val);
        uart_tx_string(outputSensorStr, sizeof(outputSensorStr));//考生把数据发送到串口
        memset(outputSensorStr, 0, sizeof(outputSensorStr));
        
        if (adc_val < 110)//
        {
            D6 = 1;
        }
        else
        {
            D6 = 0;
        }
      t1_flag = 0;//t1_flag 清除标志
    }
  }
}

