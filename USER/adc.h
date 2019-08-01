#ifndef  _ADC_H_
#define  _ADC_H_

#define	MAX_ADC_VALUE	4
#define ADC_CHANNEL_NUM 16
#define ADC_SAMPLE_NUM  1

//������
#define LEFT1_ADC()			adc_converted_value[0]
//ǰײ�����
#define LEFT2_ADC()			adc_converted_value[2]
//ǰײ�м����
#define MID_ADC()			adc_converted_value[3]
//ǰײ�Һ���
#define RIGHT2_ADC()		adc_converted_value[4]
//�Ҳ����
#define RIGHT1_ADC()		adc_converted_value[6]
//�Ե������
#define BOTTOM_L_ADC()		adc_converted_value[7]
//�Ե��Һ���
#define BOTTOM_R_ADC()		adc_converted_value[9]

void adc1_init(void);
extern __IO uint16_t adc_converted_value[ADC_CHANNEL_NUM+1];
#endif

