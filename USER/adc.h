#ifndef  _ADC_H_
#define  _ADC_H_

#define	MAX_ADC_VALUE	4
#define ADC_CHANNEL_NUM 16
#define ADC_SAMPLE_NUM  1

//左侧红外
#define LEFT1_ADC()			adc_converted_value[0]
//前撞左红外
#define LEFT2_ADC()			adc_converted_value[2]
//前撞中间红外
#define MID_ADC()			adc_converted_value[3]
//前撞右红外
#define RIGHT2_ADC()		adc_converted_value[4]
//右侧红外
#define RIGHT1_ADC()		adc_converted_value[6]
//对地左红外
#define BOTTOM_L_ADC()		adc_converted_value[7]
//对地右红外
#define BOTTOM_R_ADC()		adc_converted_value[9]

void adc1_init(void);
extern __IO uint16_t adc_converted_value[ADC_CHANNEL_NUM+1];
#endif

