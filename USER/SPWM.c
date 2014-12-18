#include "SPWM.h"
#include "timer.h"
#include "math.h"

u16 MODULATION_FREQ_MIN=1000;//��СƵ��
u16 MODULATION_FREQ_MAX=5000;//���Ƶ��
u16 SINWAVE_DATA[600];//�ز������Ϊ600
vu16 N;//�ز���
vu16 N120;//�����Ǻ�������B�������A���λ�ã��ͺ�120��
vu16 N240;//�����Ǻ�������C�������A���λ�ã��ͺ�120��
void SPWM_Init()
{
  //1.��eeprom��ȡ��ǰ���õ�ֵ
	//2.��ʼ��COS_Table;1/4����
}


/***********************************************************************************
FunctionName: u16 Search_Carrier_Rat(u16 TGT_FREQ)
Description:  ����Ŀ�����Ҳ���Ƶ�ʻ�ȡ���ز���=�ز�/Ŀ�����Ҳ����ز��ȣ������ܹ���3�������ҳ���2����ż��
Input:        TGA_FREQ ���ͣ�unsigned short ������λС��,Ŀ�����Ҳ�Ƶ��
Output:       ��
Return:       �ز���
***********************************************************************************/
u16 Caculate_Carrier_Rat(u16 TGT_FREQ)
{
	u16 Carrier_Rat;
//  //1.�ز�Ƶ�ʹ̶�
//	Carrier_Rat=CARR_FRQC*100/TGT_FREQ;
	//2.�ز��ȹ̶�
	//3.�ز��ȸ���Ŀ��Ƶ�ʷֶθı�
	 if(TGT_FREQ<1000)            
	 {
		 Carrier_Rat=600;//CARR_FRQC*100/1000=600;  
	 }
	 else if(TGT_FREQ<3000)
	 {
	   Carrier_Rat=180;//CARR_FRQC*100/3000��180;  
	 }
	 	 else if(TGT_FREQ<5000)
	 {
	   Carrier_Rat=120;//CARR_FRQC*100/5000��120;  
	 }
	 return Carrier_Rat;
	
}
/************************************************************************************************************/
//����SPWM��������Ч�������˫���������
//������M ���ͣ�unsigned short ��λС��        ���ƶȣ�ȡֵ��Χ�����ڵ���0��С�ڵ���1
//      TGA_FREQ ���ͣ�unsigned short ��λС�� Ŀ�����Ҳ�Ƶ�ʣ�ȡֵ��Χ��20~300HZ
//����ֵ��0�����������������
//        1������ĵ��ƶ���ֵ����Χ
//        2���ز�Ƶ��Խ��
//        3��Ŀ�����Ҳ�Ƶ����ֵ����Χ
//        4: ���ڼĴ�����ֵ����Χ
//˵����1����ʱ��1�������������ģʽ1��PWM���������ģʽ2
//      2��������������������Ƕ�ʱ��1�ȽϼĴ�������ֵ
//      3�����ָ��ÿһ�����Ҳ������Sk = M*[Cos(2*k*PI/N) - Cos(2*(k+1)*PI)/N]  ���У�k(0,1,2,......N-1)
//      4����Ӧһ�����Ҳ����Sk�ķ����ߵ�ƽ����ʱ��t = 1/(2*N*Fr) + Sk/(4*PI*Fr) ���У�FrΪ���Ҳ�Ƶ�ʣ�TGA_FREQ��
//      5���趨ʱ��1����Ƶ��ΪFs���򷽲��ߵ�ƽ��Ҫ�ļ�������Con = t*Fs
//      6����ʱ��1�ȽϼĴ�����ֵCCRx = TIM1->ARR - Con
//      7�����������ֻ�������Ҳ���ǰ1/4�ܲ����������ݰ��նԳƷ�����
/************************************************************************************************************/
//��ʾ������⣬����306��������Ҫ6.7ms(���Ҽ��㷨)
//��ʾ������⣬����300��������Ҫ6.5ms(���Ҽ��㷨)
//��ʾ������⣬����300��������Ҫ972us(���Ҳ��)
u8 Caculate_Control_SPWM(u16 M,u16 TGT_FREQ)
{
u8  err = 0;
u16 i;//�ز��ȣ������ܹ���3�������ҳ���2����ż��
static u16 tim1_arr_new;
static u16 tim1_arr_old;
double n5,n6,m_f,tgt_freq_f;//_f��double float�͵ĵ��ƶȺ��ز���
/*----------------------------------------------------------------------------*/
  N = Caculate_Carrier_Rat(TGT_FREQ);//����Ƶ���趨�ز��ȣ����Թ̶��ز��ȣ�Ҳ���Էֶ�
	N120 = 2*N/3;//�������A���ͺ�120�ȵĵ�λ��
	N240 = N120 >> 1;      //N240 = N120/2���������A���ͺ�240�ȵĵ�λ��
m_f = (double)M/1000;
tgt_freq_f = (double)TGT_FREQ/100;
tim1_arr_new = CPU_FREQ/(2*N*tgt_freq_f) + 0.5;//���㶨ʱ��1���ڼĴ���ARR��ֵ��4��5��
if(M > 1000)//���ƶ�Ϊ���ڵ���0��С�ڵ���1����ֵ�����ƶȵ���0ʱ�ر���� 
  {
   err = 1;
   return err;
  }                  
if(N * tgt_freq_f > 20000)//�ز�Ƶ�ʲ��ܳ���20KHZ 
  {
   err = 2;
   return err;
  }             
if(TGT_FREQ < MODULATION_FREQ_MIN || TGT_FREQ > MODULATION_FREQ_MAX)//Ŀ�����Ҳ�Ƶ�ʷ�Χ20~300HZ 
  {
   err = 3;
   return err;
  }
if(tim1_arr_new > 0xffff)//���ڼĴ�����ֵ����Χ
  {
   err = 4;
   return err;
  }  
/*-------------------------------------------------------*/
if(err == 0)//���û�д���ʼ�������
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
       SINWAVE_DATA[i] = (unsigned short)(TIMER1_ARR- (n5 + n6*(cos(i*PI/N)-cos((i+1)*PI/N))) + 0.5);//(unsigned short)(tim1_arr_new - (n5 + n6*(COS_TABLE[i]-COS_TABLE[i+1])) + 0.5);//4��5��
     else if(i < N/2)
       SINWAVE_DATA[i] = SINWAVE_DATA[N/2-i-1];
     else
       SINWAVE_DATA[i] = TIMER1_ARR - SINWAVE_DATA[i-N/2];
    }
//   //������Ʊȷ����仯������������TIM1���Զ���װ�����ڼĴ������ѵõ�Ԥ�ڵĵ���Ƶ��
   if(tim1_arr_old != tim1_arr_new)
    {
  tim1_arr_old = tim1_arr_new;//��¼�µĵ��Ʊ�
  TIM1->ARR = tim1_arr_new;   //����TIM1_ARR��Ԥװ�صģ������µ�����ֵ������һ�θ����¼���������
    }
   if(M == 0)//������ƶ�Ϊ0����ر����
     TIM1->BDTR &= 0x7fff;//��ֹPWM���������MOEλ
   else if((TIM1->BDTR & 0x8000) == 0)//������ƶȴ���0��MOEλΪ��ֹ״̬����λMOEλ
     TIM1->BDTR |= 0x8000;//ʹ��PWM���������MOEλ
  }
  return err;
}