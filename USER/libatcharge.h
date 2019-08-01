
#ifndef _H_LIBAT_
#define _H_LIBAT_


#define LIBAT_LOG_OUT	1
/********************************************************************************************************
ADC���
********************************************************************************************************/
//ADC���������ֵ
#define LIBAT_ADCDIGI_MAX	4095

//������ͨ��,���������û���õ�

#if 0
#define LIBAT_CHARGECURRENT_CH	ADC_Channel_8//������
#define LIBAT_CHARGEVOLT_CH		ADC_Channel_7//����ѹ
#define LIBAT_LIBATTEMPER_CH	ADC_Channel_15//����¶�
#else   
#define LIBAT_CHARGECURRENT_CHNO	9//������
#define LIBAT_CHARGEVOLT_CHNO		8//����ѹ
#define LIBAT_LIBATTEMPER_CHNO	7//����¶�
#endif

//�ο���ѹֵ���Ŵ�1000������λΪmv,�ο���ѹһ�㲻�ᳬ��10V,��10000mv
#define LIBAT_ADCVOLREF	3250
//��������ת��Ϊ��ѹ��ĵ�ѹ�Ŵ���
#define LIBAT_ADCCURRENT_GAIN_MULTI	10//����
#define LIBAT_ADCCURRENT_GAIN_DIV	1//��ĸ
//��������������ֵ,�Ŵ�1000��,��λΪ��ŷ
#define LIBAT_ADCCURRENT_RES	100
//��ѹ�Ŵ���
#define LIBAT_ADCVOLT_GAIN_MULTI	1//����
#define LIBAT_ADCVOLT_GAIN_DIV	11//��ĸ

//NTC���϶˷�ѹ����ֵ,��λŷķ,����ʹ��Ϊ10K 1%
#define LIBAT_NTCRESABOVE	10000
/********************************************************************************************************
end ADC���
********************************************************************************************************/

/********************************************************************************************************
PID���
********************************************************************************************************/
//PID��ȱʡϵ��
#define LIBAT_CURRENT_KP_DEF	350
#define LIBAT_CURRENT_KI_DEF	0//600
#define LIBAT_CURRENT_KD_DEF	50

//����û���õ���ѹ���,ֻ���ں������ʱ������ѹ
#define LIBAT_VOLT_KP_DEF	1
#define LIBAT_VOLT_KI_DEF	12
#define LIBAT_VOLT_KD_DEF	16
/********************************************************************************************************
end PID���
********************************************************************************************************/

/********************************************************************************************************
����IO���
********************************************************************************************************/
//�����Դ����
#define LIBAT_MOTOR_PWR_PORT	MT_PWR_PORT
#define LIBAT_MOTOR_PWR_PIN		MT_PWR_PIN

//��༰������⿪��
#define LIBAT_IRDIST_PWR_PORT	IR_DIST_PWR_PORT
#define LIBAT_IRDIST_PWR_PIN	IR_DIST_PWR_PIN

//��س��IO
#define LIBAT_CHPWM_PORT	GPIOE
#define LIBAT_CHPWM_PIN		GPIO_Pin_13

#define LIBAT_CHPWM_TIM		TIM1
#define LIBAT_CHPWM_CH		3
/********************************************************************************************************
end ����IO���
********************************************************************************************************/

/********************************************************************************************************
���������
********************************************************************************************************/

//����������
#define LIBAT_CHARGECURRENT_SET	500

//�������������ѹ��λ:mv
//#define LIBAT_CHARGEVOLT_MAX	16500//16500
#define LIBAT_CHARGEVOLT_MAX	16750//16500

#define LIBAT_CHARGECURRENT_OFFSET	12
//����ѹ���ֵ
#define LIBAT_CHARGEVOLT_SET	

//PWM��������ֵ
#define LIBAT_PWM_MAX	255 // 4095->255


//�ﵽ�����ѹ��,�������������С�ڴ˵���ֵ,��Ϊ����ѳ���
//����������
#define LIBAT_CHARGESTOP_CURRENT	100

//��ѹ���ʱ,��������С��LIBAT_CHARGECURRENT_END������������LIBAT_CHARGESTOP_TIMES����������
#define LIBAT_CHARGESTOP_TIMES	1000

//�ڷǳ��״̬��(ǰ�����ѽӳ����(��)),�������ѳ������Ͽ�,��ôÿ��һ��ʱ����Ҫȥ����ص�ѹ,
//�������������LIBAT_CHARGECURRENT_SET,��ָ����,��ʱ������λΪ100us
#define LIBAT_CHARGERESTART_DELAY	6000
//�����ɺ��ֵ�ѹ�½�,�ٻָ����ʱ�ĵ�ѹ����ֵ
#define LIBAT_CHARGERESTART_VOLT	16500//16000
/********************************************************************************************************
end ���������
********************************************************************************************************/
//PID����ķ���ֵ
enum __LiBat_PID_Errors
{
	LB_ERROR_NONE = 0,
	LB_ERR_CURROVER,//��������(���δʹ��)
	LB_ERR_VOLTOVER,//��ѹ����
	LB_ERR_TEMPOVER,//�¶ȳ���
	LB_ERR_NOBAT,//��ز�����
	LB_ERR_RETRY//��س������ʧ��,ֻ���ڲ���ģʽʱ,�������ʧ�����Դ�������3��,���ش˴���
};

#if 1
enum __LiBat_ChargeStatus
{
	LB_CHS_NOCHARGE = 0,//δ��ʼ��������ֹͣ
	LB_CHS_NORMAL,//�������,��״̬����������缰��ѹ���(��ʵ���Ǽ�����ѹ���Ƶ�PID����)
	LB_CHS_STANDBY,//����ģʽ,����ÿ��һ��ʱ�����ص�ѹ�Ա��ָ����
	LB_CHS_ERROR//��繤���쳣,�������ʧ��,�������״̬,pwm��0(���ر�),ֱ����һ�����½�����״̬
};

#endif

void LiBat_SetPwrChargePwm(uint16_t pwm);
#if 1
uint8_t LiBat_CurrentPid(uint16_t Current,uint8_t Cycles);
#else
uint8_t LiBat_CurrentPid(uint16_t Current);
#endif
uint8_t LiBat_VoltPid(uint16_t Volt, uint16_t NowVolt);
void LiBat_HalInit(void);

uint16_t LiBat_GetChargeCurrent(void);
uint16_t LiBat_GetBatVolt(void);

extern volatile uint16_t LiBat_VoltValue;
char libat_charge_task(void);
void LiBat_ExitChargeMode(void);

int get_state_libat_percent(uint8_t	sState,int volt);

int get_libat_percent(int volt);

int16_t LiBat_GetBatTemper(void);


#endif
