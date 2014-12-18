#include "SPWM.h"
#include "timer.h"
#include "math.h"

u16 MODULATION_FREQ_MIN=1000;//最小频率
u16 MODULATION_FREQ_MAX=5000;//最大频率
u16 SINWAVE_DATA[600];//载波比最大为600
vu16 N;//载波比
vu16 N120;//在三角函数表中B相落后于A相的位置，滞后120°
vu16 N240;//在三角函数表中C相落后于A相的位置，滞后120°
void SPWM_Init()
{
  //1.从eeprom读取以前设置的值
	//2.初始化COS_Table;1/4周期
}


/***********************************************************************************
FunctionName: u16 Search_Carrier_Rat(u16 TGT_FREQ)
Description:  根据目标正弦波的频率获取到载波比=载波/目标正弦波；载波比，必须能够被3整除，且除以2后是偶数
Input:        TGA_FREQ 类型：unsigned short 包含两位小数,目标正弦波频率
Output:       无
Return:       载波比
***********************************************************************************/
u16 Caculate_Carrier_Rat(u16 TGT_FREQ)
{
	u16 Carrier_Rat;
//  //1.载波频率固定
//	Carrier_Rat=CARR_FRQC*100/TGT_FREQ;
	//2.载波比固定
	//3.载波比根据目标频率分段改变
	 if(TGT_FREQ<1000)            
	 {
		 Carrier_Rat=600;//CARR_FRQC*100/1000=600;  
	 }
	 else if(TGT_FREQ<3000)
	 {
	   Carrier_Rat=180;//CARR_FRQC*100/3000≈180;  
	 }
	 	 else if(TGT_FREQ<5000)
	 {
	   Carrier_Rat=120;//CARR_FRQC*100/5000≈120;  
	 }
	 return Carrier_Rat;
	
}
/************************************************************************************************************/
//计算SPWM函数（等效面积法，双极性输出）
//参数：M 类型：unsigned short 三位小数        调制度，取值范围：大于等于0，小于等于1
//      TGA_FREQ 类型：unsigned short 两位小数 目标正弦波频率，取值范围：20~300HZ
//返回值：0：计算输出数据正常
//        1：输入的调制度数值超范围
//        2：载波频率越限
//        3：目标正弦波频率数值超范围
//        4: 周期寄存器数值超范围
//说明：1、定时器1工作在中央对齐模式1，PWM输出工作在模式2
//      2、计算出来的数组内容是定时器1比较寄存器的数值
//      3、被分割的每一分正弦波的面积Sk = M*[Cos(2*k*PI/N) - Cos(2*(k+1)*PI)/N]  其中：k(0,1,2,......N-1)
//      4、对应一份正弦波面积Sk的方波高电平持续时间t = 1/(2*N*Fr) + Sk/(4*PI*Fr) 其中：Fr为正弦波频率（TGA_FREQ）
//      5、设定时器1计数频率为Fs，则方波高电平需要的计数个数Con = t*Fs
//      6、定时器1比较寄存器的值CCRx = TIM1->ARR - Con
//      7、本计算程序只计算正弦波的前1/4周波，后续数据按照对称法计算
/************************************************************************************************************/
//用示波器监测，计算306点数据需要6.7ms(余弦计算法)
//用示波器监测，计算300点数据需要6.5ms(余弦计算法)
//用示波器监测，计算300点数据需要972us(余弦查表法)
u8 Caculate_Control_SPWM(u16 M,u16 TGT_FREQ)
{
u8  err = 0;
u16 i;//载波比，必须能够被3整除，且除以2后是偶数
static u16 tim1_arr_new;
static u16 tim1_arr_old;
double n5,n6,m_f,tgt_freq_f;//_f：double float型的调制度和载波比
/*----------------------------------------------------------------------------*/
  N = Caculate_Carrier_Rat(TGT_FREQ);//根据频率设定载波比，可以固定载波比，也可以分段
	N120 = 2*N/3;//计算相对A相滞后120度的点位置
	N240 = N120 >> 1;      //N240 = N120/2；计算相对A相滞后240度的点位置
m_f = (double)M/1000;
tgt_freq_f = (double)TGT_FREQ/100;
tim1_arr_new = CPU_FREQ/(2*N*tgt_freq_f) + 0.5;//计算定时器1周期寄存器ARR的值，4舍5入
if(M > 1000)//调制度为大于等于0且小于等于1的数值，调制度等于0时关闭输出 
  {
   err = 1;
   return err;
  }                  
if(N * tgt_freq_f > 20000)//载波频率不能超过20KHZ 
  {
   err = 2;
   return err;
  }             
if(TGT_FREQ < MODULATION_FREQ_MIN || TGT_FREQ > MODULATION_FREQ_MAX)//目标正弦波频率范围20~300HZ 
  {
   err = 3;
   return err;
  }
if(tim1_arr_new > 0xffff)//周期寄存器数值超范围
  {
   err = 4;
   return err;
  }  
/*-------------------------------------------------------*/
if(err == 0)//入参没有错误开始计算过程
  {
   //n1 = 1 / (2*N*TGT_FREQ);
   //n2 = 2*PI/N;   
   //n3 = 4*PI*TGT_FREQ;
   //n4 = CPU_FREQ / 2;
   //n5 = n1*n4;
   //n6 = M*n4/n3;
//   n5 = CPU_FREQ / (4*N*tgt_freq_f);
   n6 = m_f*CPU_FREQ / (8*PI*tgt_freq_f);
   for(i=0;i<N;i++)
    {
     if(i < N/4)
       SINWAVE_DATA[i] = (unsigned short)(TIMER1_ARR- (n5 + n6*(cos(i*PI/N)-cos((i+1)*PI/N))) + 0.5);//(unsigned short)(tim1_arr_new - (n5 + n6*(COS_TABLE[i]-COS_TABLE[i+1])) + 0.5);//4舍5入
     else if(i < N/2)
       SINWAVE_DATA[i] = SINWAVE_DATA[N/2-i-1];
     else
       SINWAVE_DATA[i] = TIMER1_ARR - SINWAVE_DATA[i-N/2];
    }
//   //如果调制比发生变化，则重新设置TIM1的自动重装载周期寄存器，已得到预期的调制频率
   if(tim1_arr_old != tim1_arr_new)
    {
  tim1_arr_old = tim1_arr_new;//记录新的调制比
  TIM1->ARR = tim1_arr_new;   //由于TIM1_ARR是预装载的，所以新的周期值会在下一次更新事件后起作用
    }
   if(M == 0)//如果调制度为0，则关闭输出
     TIM1->BDTR &= 0x7fff;//禁止PWM输出，清零MOE位
   else if((TIM1->BDTR & 0x8000) == 0)//如果调制度大于0且MOE位为禁止状态则置位MOE位
     TIM1->BDTR |= 0x8000;//使能PWM输出，设置MOE位
  }
  return err;
}