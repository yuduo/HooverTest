
#if !USE_ON_COMPUTER
#include "sys.h"
#include "task_rx.h"
#include "libatcharge.h"
#endif

#include "test_mode.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

uint8_t tm_mode = 0;//����ģʽѡ��,0Ϊ���ģʽ,�������ڲ��Լ��ϵĳ���,1Ϊ��������ģʽ,������װ�����Բ����


//���԰��ϴ������ݴ�ŵ���ʱ����
struct __testbd_dat{
	uint8_t unread_flag;//δ����־,������µ������ϱ�,��ֵ��1,�ڱ���ȡ����,����0
	
	uint8_t pwr_sw;//��Դ�����Ƿ�� 1Ϊ�Ѵ� 0Ϊδ��
	uint8_t mtoc_whl_l;//����ֵ���Ķ�·�����Ƿ�ͨ�� 1Ϊ�Ѷ�· 0Ϊδ��·
	uint8_t mtoc_whl_r;//�Һ��ֵ���Ķ�·�����Ƿ�ͨ�� 1Ϊ�Ѷ�· 0Ϊδ��·
	uint8_t mtoc_smt_l;//���ˢ����Ķ�·�����Ƿ�ͨ��
	uint8_t mtoc_smt_r;//�ұ�ˢ����Ķ�·�����Ƿ�ͨ��
	uint8_t mtoc_mt_fan;//����Ķ�·�����Ƿ��(δʹ��)
	uint8_t mtoc_mt_mid;//���ˢ����Ķ�·�����Ƿ�ͨ��

	uint8_t bum_sw;//ǰ��ײ����ģ������,bit0Ϊ�� bit1Ϊ��,1��ʾ�ر�(�յ�Ϊ�͵�ƽ),0��ʾ��(�յ�Ϊ�ߵ�ƽ)
	uint8_t bum_sw_led;//ǰ��ײ�����LED��Դ���,����Ϊ0,������Ϊ1,bit0Ϊ��,bit1Ϊ��
	uint8_t sensors_sw;//��������ʽ��������ģ�������ź�bit0-bit5����ΪĨ��/����ǽ/����/��������/������/������
	uint8_t sensors_sw_led;//��������ʽ������LED��Դ���bit0-bit3����ΪĨ��/��/��/��������/��/��

	uint8_t bat_info;//��صĽ��������Ϣ:0�������԰�Ͽ� 1��ؽ�������,24V�ӵ�DC�� 2��ؽ�������,24V�ӵ���紥Ƭ
					//3��ؽ�������,24Vδ��������
	uint8_t nc;//δʹ��

	uint16_t cd_smt_l;//���ˢ������
	uint16_t cd_smt_r;//�ұ�ˢ������
	uint16_t cd_mt_mid;//��ɨ������
	
};
struct __testbd_dat tmd_rxbd_dat;

void tx_dp_msg(uint8_t *buff,int len);
void proc_bd_test_mode(uint8_t *buf,uint8_t len);
void proc_com_test_mode(uint8_t *buf,uint8_t len);
void tx_com_dat(uint8_t code, uint8_t progress, uint8_t error_code, uint8_t ret_value);
void tx_com_beep(uint8_t progress, uint8_t error_code, uint8_t ret_value,uint16_t *irvalue,uint16_t *irbuttom);
//void tx_dp_dat(uint8_t code, uint8_t para, uint8_t res_out, uint8_t res_in);
void tx_dp_dat(uint8_t code, uint8_t para, uint8_t para1);
void tx_dp_pc_dat(uint8_t code, uint8_t para, uint8_t para1);
void data_print(uint8_t * pdat,uint16_t len);
uint8_t get_chksum(uint8_t *pdata, uint16_t length);

void tx_com_burnin(uint8_t side_dir, uint16_t timer,uint32_t bum_left_ct,uint32_t ir_left_ct,uint32_t bum_right_ct,uint32_t ir_right_ct);
void tx_sn(uint8_t code ,uint8_t *sn);

#define TMOD_COMRX_BUFFSIZE	64

static uint8_t tmd_buf[TMOD_COMRX_BUFFSIZE];
static uint8_t tmd_bd_buf[TMOD_COMRX_BUFFSIZE];

static uint8_t tmod_rx_sta = 0;
static uint8_t tmod_rx_len = 0;
static uint8_t tmod_bd_rx_sta = 0;
static uint8_t tmod_bd_rx_len = 0;

//UART���շ�����
//com_dp: ����ͬʱ�������ӿڵ���,������λ��ͨ�ŵ�UART1�Լ�����԰�ͨ�ŵ�UART4(ԭDP��),��Ҫ���Ӵ˱�������Ӧ
//        ��ͬ��Э���������,����λ��ͨ��,ֵΪ0,����԰�ͨ��ֵΪ1
void tmod_rx_uart(uint8_t com_dp, uint8_t chr)
{
	uint8_t *n_pbuf;
	uint8_t *n_prxsta;
	uint8_t *n_polen;
		//log_printf("%02X ",chr);

	if(!com_dp)
	{
		n_pbuf = tmd_buf;
		n_prxsta = &tmod_rx_sta;
		n_polen = &tmod_rx_len;
	}
	else
	{
		n_pbuf = tmd_bd_buf;
		n_prxsta = &tmod_bd_rx_sta;
		n_polen = &tmod_bd_rx_len;
	}
	
	switch(*n_prxsta)
	{
		case 0:
			if(chr == 0x7E)
			{
				*n_prxsta=1;
			}
			break;
		case 1:
			if(chr ==0x5D)
			{
				*n_prxsta=2;
				memset(n_pbuf,0x00,TMOD_COMRX_BUFFSIZE);
				*n_polen=0;
			}
			else
			 *n_prxsta =0;
			break;
		case 2:
			n_pbuf[(*n_polen)++]=chr;
			if(*n_polen >=TMOD_COMRX_BUFFSIZE)
			{
				log_printf("rx error\r\n");
				*n_polen=0;
				*n_prxsta=0;
				break;
			}
			//7E 5D 05 00 6D 7D 
			if(chr == 0x7D && n_pbuf[*n_polen-2]==0x6D)	//�������
			{
				//log_printf("\r\nfjd\r\n\r\n");
				if(!com_dp)//����λ����ͨ��
					proc_com_test_mode(n_pbuf,*n_polen);
				else
					proc_bd_test_mode(n_pbuf,*n_polen);
				*n_polen=0;
				*n_prxsta=0;
			}
			break;

		default:
				*n_polen=0;
				*n_prxsta=0;
				break;
			
		
	}
}

#define BDTEST_LOGPRINT	0

#define BDTEST_SMT_PWMVALUE	900//��ˢPWMת��ֵ,����,ֵԽ��ת��ԽС,��ΧΪ1000-0

uint8_t bdtest_fantest = 0;//����ģʽ�·������ʹ��,�����ת�ټ��ź�ǰ�����̵�A������жϺ��ص�,���Դ���Ϊ���
uint16_t tmod_fancd = 0;//��������̼�����,���Ƶ�30000,�����򱣳�30000 
static uint8_t bdtest_sta = TPC_STOP_ALL;//�˱���������ǲ��Թ����е�ĳ����������,����ָ��,�˱����봮��ָ���޹�,ֻ��ʹ������֮��ͬ�ĺ궨��
static uint8_t bdtest_sta_child = 0;//���Բ����С״̬,0Ϊ��ʼ

#define BDTEST_CHG_SUCC_MIN	20//��������������Χ�ڵļ�����Сֵ,��������������ֵ,��Ϊ�������
#define BDTEST_CHG_FAIL_MAX	10//���������޵ļ������ֵ,������ֵ,��Ϊ���ʧ��
#define BDTEST_CHG_START_MAX	200//��ʼ��絽�ﵽԤ���������޶�ʱ��,����������ʱ�仹δ�ﵽԤ��ֵ,��ʾ���ʧ��
uint16_t bdtest_chgcurr = 0;//�������ļ�¼,���������ﵽԤ��ֵ��100maʱ,����20�����ϵķ�Χ�ڼ�����Ϊ�������
									//������ܼƳ���10�����ϵĵ������޼���,��Ϊ���ʧ��
									//������30��ʱ,���޼�������������������������Ϊ�������
static uint8_t bdtest_chgsucceed_counter = 0xff;//�����������
static uint8_t bdtest_chgfailed_counter = 0xff;//��糬�޼���
static uint16_t bdtest_chgstart_counter = 0;//��翪ʼ����������ֵ֮ǰ�ļ���,��ֵ�������BDTEST_CHG_START_MAX,��ʾһֱ��δ�ﵽԤ��ֵ,���ʧ��

#if 0
//��¼��ǰ�ĳ�����,�ⲿ����ʹ��
void bdtest_set_curr(uint16_t curr)
{bdtest_chgcurr = curr;}
#endif
//�Եغ�������õ���ʱ����
static uint8_t tm_ir_bot_cc;//,tm_ir_bot_j;//,tm_ir_bot_i
///static int32_t tm_ir_bot_k1;//,tm_ir_bot_k2;

void proc_bdtest_task(void)
{
	static uint8_t bd_timer = 0xff;
	static uint8_t bd_heart_timer = 0;
	static uint8_t bd_heart_ct = 0;
//	static uint8_t bdtest_sta_old = TPC_STOP_ALL;
	static uint16_t bd_batcharge_timer = 0;
	static uint8_t bd_stopmode_trigger = TPC_STOP_ALL;//TPC_STOP_ALL״̬���´�����־,�����ж�������TPC_STOP_ALL״̬
	
	if(TIM5->CNT < 50000)//50ms��ʱ
		return;
	TIM5->CNT = 0;

	if(bd_heart_timer ++ >= 20)//����������,ÿ��һ��
	{
		bd_heart_timer = 0;
		tx_com_dat(TPC_HEART_BEEP, bd_heart_ct ++, TPC_ERR_NOERROR, 0);
	}
	//return;
	if(bdtest_sta != TPC_STOP_ALL)
		bd_stopmode_trigger = bdtest_sta;
	switch(bdtest_sta)
	{
		case TPC_STOP_ALL:
			//if(bdtest_sta_old != bdtest_sta)
			if(bd_stopmode_trigger != TPC_STOP_ALL)
			{
				//if(bd_stopmode_trigger == TPC_IDLE)
				{
					tx_com_dat(TPC_STOP_ALL, 100 , TPC_ERR_NOERROR, 0);//�Ϸ��������
					tx_dp_dat(TBD_STOP_ALL, 0, 0);//��24V��DC�����ӿ�
				}
				bd_stopmode_trigger = TPC_STOP_ALL;
				//TargetSysReset();
				sys->sState = SYS_IDLE;
			}
			MOTOR_POWER_OFF_NPRI();//�رյ����Դ
			break;
		case TPC_START_ALL://����Ҫ����������Ƿ�����/����԰��ͨ���Ƿ�����
			//MOTOR_CTRL(1000, 1000, 0, 0);
			MOTOR_POWER_OFF_NPRI();
			tx_dp_dat(TBD_START_ALL, 0, 0);//��24V��DC�����ӿ�
			bdtest_sta = TPC_WHL_L;//TPC_WHL_L;////TPC_WHL_L;//TPC_GYRO_TST;//TPC_WHL_L;//TPC_SMT_L;;
			bdtest_sta_child = 0;
			break;
		case TPC_WHL_L://���ּ��
			switch(bdtest_sta_child&0x0f)
			{
				case 0://��ʼ��������
					bdtest_sta_child = 1;
					MOTOR_POWER_ON();
					motor.c_left_hw = 0;
					motor_wheel_stop(RIGHT_WHEEL);
					motor_wheel_forward(LEFT_WHEEL,600);
					
					tx_com_dat(TPC_WHL_L, 25 , TPC_ERR_NOERROR, 0);//�ϱ�����
					tx_dp_dat(TBD_MTOC_WHL_L, 0, 0);//�·�����ֹ��ز���,����·�ĵ���Ͽ�
					bd_timer = 0;
					break;
				case 1://��ǰת
					if(bd_timer ++ < 20)break;
					motor_wheel_stop(LEFT_WHEEL);
					#if BDTEST_LOGPRINT
					log_printf("forward:%d\r\n",motor.c_left_hw);
					#endif
#if HW_AB_OLD					
					if(motor.c_left_hw < 200)//����������
#else
					if(motor.c_left_hw > -200)//����������
#endif
					{
						MOTOR_POWER_OFF_NPRI();
						//motor_wheel_forward(LEFT_WHEEL,GO_FORWARD,600);
						tx_com_dat(TPC_WHL_L, 50, TPC_ERR_FORWARD, 0);
						bdtest_sta = TPC_WHL_R;//������һ������
						bdtest_sta_child = 0;
						break;
					}
					tx_com_dat(TPC_WHL_L, 40, TPC_ERR_NOERROR, 0);
					bdtest_sta_child = 2;
					//MOTOR_POWER_ON();
					motor.c_left_hw = 0;
					motor_wheel_backward(LEFT_WHEEL,600);
					bd_timer = 0;
					break;
				case 2://���ת
					if(bd_timer ++ < 20)break;
					motor_wheel_stop(LEFT_WHEEL);

					#if BDTEST_LOGPRINT
					log_printf("back:%d\r\n",motor.c_left_hw);
					#endif
#if HW_AB_OLD					
					if(motor.c_left_hw > -200)
#else
					if(motor.c_left_hw < 200)//����������
#endif
					{
						MOTOR_POWER_OFF_NPRI();
						//motor_wheel_forward(LEFT_WHEEL,GO_FORWARD,600);
						tx_com_dat(TPC_WHL_L, 75, TPC_ERR_BACK, 0);//�ϱ��������ת
						
						bdtest_sta = TPC_WHL_R;//������һ������
						bdtest_sta_child = 0;
						break;
					}
					tx_com_dat(TPC_WHL_L, 75, TPC_ERR_NOERROR, 0);//�ϱ�����
					
					bdtest_sta_child = 3;
					//MOTOR_POWER_ON();
					motor.c_left_hw = 0;
					
					tx_dp_dat(TBD_MTOC_WHL_L, 1, 0);//�·�����ֹ��ز���,����·�ĵ������
					bd_timer = 0;
					tmd_rxbd_dat.unread_flag = 0;//�����ݽ��յĻ���δ����־��0
					break;
				case 3:
					if(bd_timer ++ < 20 && !tmd_rxbd_dat.unread_flag)break;//��ʱδ�յ�����
					bd_timer = 0;
					if(!tmd_rxbd_dat.unread_flag)//��ʱδ�յ�����,����԰�(�Կض˿�)ͨ��ʧ��
					{
						tx_com_dat(TPC_DPPORT_TST, 0, TPC_ERR_COMM, 0);//����ͨ�Ŵ���
						tx_dp_dat(TBD_MTOC_WHL_L, 1, 0);//�·�����ֹ��ز���,����·�ĵ������
						bdtest_sta_child = (bdtest_sta_child & 0xf0) + 3;
						bdtest_sta_child += 0x10;
						if(bdtest_sta_child > 0x50)
							bdtest_sta_child = 4;
						#if BDTEST_LOGPRINT
						log_printf("no rep:TBD_MTOC_WHL_L\r\n");
						#endif
						break;
					}
					else
					{
						tmd_rxbd_dat.unread_flag = 0;
					}
					tmd_rxbd_dat.unread_flag = 0;
					if(tmd_rxbd_dat.mtoc_whl_l)
					{
						uint8_t i;
						uint16_t n_mt_curr;
						
						motor_wheel_forward(LEFT_WHEEL,600);
						delay_ms(100);
						for(i = 0;i < 18;i ++)
						{
							delay_ms(5);
							n_mt_curr = I_MOTER_LEFT();
						#if BDTEST_LOGPRINT
							log_printf("curr:%d-%d-%d\r\n",n_mt_curr,MAX_MOTOR_I,i);
						#endif
							//if(I_MOTER_LEFT() >= MAX_MOTOR_I)
							//if(n_mt_curr >= 100)
							if(n_mt_curr >= 60)
							{
								motor_wheel_stop(LEFT_WHEEL);
								break;
							}
						}
						
						tx_com_dat(TPC_WHL_L, 100, i < 18 ? TPC_ERR_NOERROR:TPC_ERR_OC, 0);//�ϱ����Ȼ��ϱ�����������

						if(i < 5)
						{
						#if BDTEST_LOGPRINT
							bdtest_sta_child = 4;
							log_printf("oc succeeded\r\n");
						#else
							bdtest_sta = TPC_WHL_R;//������һ������
							bdtest_sta_child = 0;
						#endif
						}
						#if BDTEST_LOGPRINT
						else
						{
							bdtest_sta_child = 4;
							log_printf("\r\n11111\r\n");
							tx_dp_dat(TBD_MTOC_WHL_L, 0, 0);//�·�����ֹ��ز���,����·�ĵ���Ͽ�
							break;
						}
						#endif
					}
					
					tx_dp_dat(TBD_MTOC_WHL_L, 0, 0);//�·�����ֹ��ز���,����·�ĵ���Ͽ�
					//bdtest_sta = TPC_WHL_R;
					//bdtest_sta_child = 0;
					//MOTOR_POWER_OFF_NPRI();
				#if BDTEST_LOGPRINT
					break;
				case 4:
					log_printf("$");
					motor_wheel_stop(LEFT_WHEEL);
					MOTOR_POWER_OFF_NPRI();
					break;
				#endif
				default:
					bdtest_sta = TPC_WHL_R;
					bdtest_sta_child = 0;
					motor_wheel_stop(LEFT_WHEEL);
					MOTOR_POWER_OFF_NPRI();
					break;
			}
			break;
		case TPC_WHL_R://���ּ��
			switch(bdtest_sta_child&0x0f)
			{
				case 0://��ʼ��������
					bdtest_sta_child = 1;
					MOTOR_POWER_ON();
					motor.c_right_hw = 0;
					motor_wheel_stop(LEFT_WHEEL);
					motor_wheel_forward(RIGHT_WHEEL,600);
					
					tx_com_dat(TPC_WHL_R, 25 , TPC_ERR_NOERROR, 0);//�ϱ�����
					tx_dp_dat(TBD_MTOC_WHL_R, 0, 0);//�·��Һ��ֹ��ز���,����·�ĵ���Ͽ�
					bd_timer = 0;
					break;
				case 1://��ǰת
					if(bd_timer ++ < 20)break;
					motor_wheel_stop(RIGHT_WHEEL);
					#if BDTEST_LOGPRINT
					log_printf("forward:%d\r\n",motor.c_right_hw);
					#endif
#if HW_AB_OLD					
					if(motor.c_right_hw < 200)//����������
#else
					if(motor.c_right_hw > -200)//����������
#endif
					{
						MOTOR_POWER_OFF_NPRI();
						//motor_wheel_forward(LEFT_WHEEL,GO_FORWARD,600);
						tx_com_dat(TPC_WHL_R, 50, TPC_ERR_FORWARD, 0);
						#if BDTEST_LOGPRINT
						bdtest_sta_child = 4;
						#else
						bdtest_sta = TPC_SMT_L;//������һ������
						bdtest_sta_child = 0;
						#endif
						break;
					}
					tx_com_dat(TPC_WHL_R, 40, TPC_ERR_NOERROR, 0);
					bdtest_sta_child = 2;
					//MOTOR_POWER_ON();
					motor.c_right_hw = 0;
					motor_wheel_backward(RIGHT_WHEEL,600);
					bd_timer = 0;
					break;
				case 2://���ת
					if(bd_timer ++ < 20)break;
					motor_wheel_stop(RIGHT_WHEEL);

					#if BDTEST_LOGPRINT
					log_printf("back:%d\r\n",motor.c_right_hw);
					#endif
					//if(motor.c_right_hw > -200)
#if HW_AB_OLD					
					if(motor.c_right_hw > -200)
#else
					if(motor.c_right_hw < 200)//����������
#endif
					{
						MOTOR_POWER_OFF_NPRI();
						//motor_wheel_forward(LEFT_WHEEL,GO_FORWARD,600);
						tx_com_dat(TPC_WHL_R, 75, TPC_ERR_BACK, 0);//�ϱ��������ת
						
						#if BDTEST_LOGPRINT
						bdtest_sta_child = 4;
						#else
						bdtest_sta = TPC_SMT_L;//������һ������
						bdtest_sta_child = 0;
						#endif
						break;
					}
					tx_com_dat(TPC_WHL_R, 75, TPC_ERR_NOERROR, 0);//�ϱ�����
					
					bdtest_sta_child = 3;
					//MOTOR_POWER_ON();
					motor.c_left_hw = 0;
					
					tx_dp_dat(TBD_MTOC_WHL_R, 1, 0);//�·�����ֹ��ز���,����·�ĵ������
					bd_timer = 0;
					tmd_rxbd_dat.unread_flag = 0;//�����ݽ��յĻ���δ����־��0
					break;
				case 3:
					if(bd_timer ++ < 20 && !tmd_rxbd_dat.unread_flag)break;//��ʱδ�յ�����
					bd_timer = 0;
					if(!tmd_rxbd_dat.unread_flag)//��ʱδ�յ�����,����԰�(�Կض˿�)ͨ��ʧ��
					{
						tx_com_dat(TPC_DPPORT_TST, 0, TPC_ERR_COMM, 0);//����ͨ�Ŵ���
						tx_dp_dat(TBD_MTOC_WHL_R, 1, 0);//�·�����ֹ��ز���,����·�ĵ������
						//bdtest_sta_child = 3;
						bdtest_sta_child = (bdtest_sta_child & 0xf0) + 3;
						bdtest_sta_child += 0x10;
						if(bdtest_sta_child > 0x50)
							bdtest_sta_child = 4;
						#if BDTEST_LOGPRINT
						log_printf("no rep:TBD_MTOC_WHL_L\r\n");
						#endif
						break;
					}
					else
					{
						tmd_rxbd_dat.unread_flag = 0;
					}
					tmd_rxbd_dat.unread_flag = 0;
					if(tmd_rxbd_dat.mtoc_whl_r)
					{
						uint8_t i;
						uint16_t n_mt_curr;
						
						motor_wheel_forward(RIGHT_WHEEL,600);
						delay_ms(100);
						for(i = 0;i < 18;i ++)
						{
							delay_ms(5);
							n_mt_curr = I_MOTER_RIGHT();
						#if BDTEST_LOGPRINT
							log_printf("curr:%d-%d-%d\r\n",n_mt_curr,MAX_MOTOR_I,i);
						#endif
							//if(I_MOTER_LEFT() >= MAX_MOTOR_I)
							//if(n_mt_curr >= 100)
							if(n_mt_curr >= 60)
							{
								motor_wheel_stop(RIGHT_WHEEL);
								break;
							}
						}
						
						tx_com_dat(TPC_WHL_R, 100, i < 18 ? TPC_ERR_NOERROR:TPC_ERR_OC, 0);//�ϱ����Ȼ��ϱ�����������

						//log_printf("\r\n 55555\r\n");
						if(i < 8)
						{
						#if BDTEST_LOGPRINT
							bdtest_sta_child = 4;
							log_printf("oc succeeded\r\n");
						#else
							bdtest_sta = TPC_SMT_L;//������һ������
							bdtest_sta_child = 0;
						#endif
						}
						#if BDTEST_LOGPRINT
						else
						{
							bdtest_sta_child = 4;
							log_printf("\r\n11111\r\n");
							tx_dp_dat(TBD_MTOC_WHL_R, 0, 0);//�·�����ֹ��ز���,����·�ĵ���Ͽ�
							break;
						}
						#endif
					}
					
					tx_dp_dat(TBD_MTOC_WHL_R, 0, 0);//�·�����ֹ��ز���,����·�ĵ���Ͽ�
					//bdtest_sta = TPC_WHL_R;
					//bdtest_sta_child = 0;
					//MOTOR_POWER_OFF_NPRI();
				#if BDTEST_LOGPRINT
					break;
				case 4:
					log_printf("$");
					motor_wheel_stop(RIGHT_WHEEL);
					MOTOR_POWER_OFF_NPRI();
					break;
				#endif
				default:
					bdtest_sta = TPC_SMT_L;
					bdtest_sta_child = 0;
					motor_wheel_stop(DOUBLE_WHEEL);
					MOTOR_POWER_OFF_NPRI();
					break;
			}
			break;
		case TPC_SMT_L://�������ˢ���
			switch(bdtest_sta_child&0x0f)
			{
				case 0://��ʼ��������
					bdtest_sta_child = 1;
					MOTOR_POWER_ON();
					SET_RSIDE_MOTER(1000);
					SET_LSIDE_MOTER(BDTEST_SMT_PWMVALUE);
					tx_com_dat(TPC_SMT_L, 25 , TPC_ERR_NOERROR, 0);//�ϱ�����
					tx_dp_dat(TBD_SMT_L, 1, 0);//�·����ˢ��ʼ��������,��ԭ��������0
					tx_dp_dat(TBD_MTOC_SMT_L, 0, 0);//�·�����ֹ��ز���,����·�ĵ���Ͽ�
					bd_timer = 0;
					break;
				case 1://��ȡ������
					if(bd_timer ++ < 20)break;
					tx_com_dat(TPC_SMT_L, 50 , TPC_ERR_NOERROR, 0);//�ϱ�����
					tx_dp_dat(TBD_SMT_L, 0, 0);//�·����ˢֹͣ��������,��ԭ����������0
					tx_dp_dat(TBD_SMT_L, 3, 0);//�·����ˢֹͣ��������,��ԭ����������0
					bd_timer = 0;
					bdtest_sta_child = 2;
					break;
				case 2:
					if(bd_timer ++ < 20 && !tmd_rxbd_dat.unread_flag)break;//��ʱδ�յ�����
					bd_timer = 0;
					if(!tmd_rxbd_dat.unread_flag)//��ʱδ�յ�����,����԰�(�Կض˿�)ͨ��ʧ��
					{
						SET_LSIDE_MOTER(BDTEST_SMT_PWMVALUE);//ͣ��ˢ
						tx_com_dat(TPC_DPPORT_TST, 0, TPC_ERR_COMM, 0);//����ͨ�Ŵ���
						tx_dp_dat(TBD_SMT_L, 3, 0);//�·����ˢֹͣ��������,��ԭ����������0
						//bdtest_sta_child = 2;
						bdtest_sta_child = (bdtest_sta_child & 0xf0) + 2;
						bdtest_sta_child += 0x10;
						if(bdtest_sta_child > 0x50)
							bdtest_sta_child = 4;
						break;
					}
					else
					{
						tmd_rxbd_dat.unread_flag = 0;
					}
					tmd_rxbd_dat.unread_flag = 0;
					
					tx_com_dat(TPC_SMT_L, 75, tmd_rxbd_dat.cd_smt_l < 100 ? TPC_ERR_FORWARD:TPC_ERR_NOERROR, 0);//�ϱ����Ȼ��ϱ������ת

					//log_printf("get SMTL cd:%d\r\n",tmd_rxbd_dat.cd_smt_l);
					if(tmd_rxbd_dat.cd_smt_l < 100)
					{
					#if 0
						//bdtest_sta = TPC_SMT_R;//������һ������
						bdtest_sta_child = 4;
					#else
						bdtest_sta = TPC_SMT_R;//������һ������
						bdtest_sta_child = 0;
					#endif
						break;
					}
					

					//log_printf("get2 SMTL cd:%d\r\n",tmd_rxbd_dat.cd_smt_l);
					tx_dp_dat(TBD_MTOC_SMT_L, 1, 0);//�·����ˢ��ʼ�������
					SET_LSIDE_MOTER(BDTEST_SMT_PWMVALUE);
					bdtest_sta_child = 3;
					
					break;
				case 3:
					if(bd_timer ++ < 20 && !tmd_rxbd_dat.unread_flag)break;//��ʱδ�յ�����
					bd_timer = 0;
					if(!tmd_rxbd_dat.unread_flag)//��ʱδ�յ�����,����԰�(�Կض˿�)ͨ��ʧ��
					{
						tx_com_dat(TPC_DPPORT_TST, 0, TPC_ERR_COMM, 0);//����ͨ�Ŵ���
						tx_dp_dat(TBD_MTOC_SMT_L, 1, 0);//�·����ˢ��ʼ�������
						bdtest_sta_child = (bdtest_sta_child & 0xf0) + 3;
						bdtest_sta_child += 0x10;
						if(bdtest_sta_child > 0x50)
							bdtest_sta_child = 4;
						break;
					}
					else
					{
						tmd_rxbd_dat.unread_flag = 0;
					}
					//log_printf("get mtoc_smt_l:%d\r\n",tmd_rxbd_dat.mtoc_smt_l);
					if(tmd_rxbd_dat.mtoc_smt_l)
					{
						uint8_t i;//, j = 0;
						uint16_t n_mt_curr;
						
						SET_LSIDE_MOTER(BDTEST_SMT_PWMVALUE);
						delay_ms(10);
						for(i = 0;i < 8;i ++)
						{
							delay_ms(5);
							n_mt_curr = I_SMTL_ADC();
						#if BDTEST_LOGPRINT
							log_printf("curr:%d-%d\r\n",n_mt_curr,i);
						#endif
							if(n_mt_curr > 600)
							{
								//j ++;
								break;
							}
						}

						tx_com_dat(TPC_SMT_L, 100, i < 5 ? TPC_ERR_NOERROR:TPC_ERR_OC, 0);//�ϱ����Ȼ��ϱ�����������
						if(i < 5)
						{
						#if 0
							//bdtest_sta = TPC_SMT_R;//������һ������
							bdtest_sta_child = 4;
							log_printf("*****\r\n");
						#else
							bdtest_sta = TPC_SMT_R;//������һ������
							bdtest_sta_child = 0;
						#endif
							//break;
						}
					}
					else
						tx_com_dat(TPC_SMT_L, 100, TPC_ERR_OC, 0);//�ϱ����Ȼ��ϱ�����������
					
					tx_dp_dat(TBD_MTOC_SMT_L, 0, 0);//�·�����ֹ��ز���,����·�ĵ���Ͽ�
					//bdtest_sta = TPC_WHL_R;
					//bdtest_sta_child = 0;
					//MOTOR_POWER_OFF_NPRI();
				#if 0
					break;
				case 4:
					log_printf("^");
					SET_LSIDE_MOTER(1000);
					MOTOR_POWER_OFF_NPRI();
					break;
				#endif
				default:
					bdtest_sta = TPC_SMT_R;
					bdtest_sta_child = 0;
					SET_LSIDE_MOTER(1000);
					MOTOR_POWER_OFF_NPRI();
					break;
			}
			break;
		case TPC_SMT_R://���Դ��ұ�ˢ���
			switch(bdtest_sta_child&0x0f)
			{
				case 0://��ʼ��������
					bdtest_sta_child = 1;
					MOTOR_POWER_ON();
					SET_LSIDE_MOTER(1000);
					SET_RSIDE_MOTER(BDTEST_SMT_PWMVALUE);
					tx_com_dat(TPC_SMT_R, 25 , TPC_ERR_NOERROR, 0);//�ϱ�����
					tx_dp_dat(TBD_MTOC_SMT_R, 0, 0);//�·�����ֹ��ز���,����·�ĵ���Ͽ�
					tx_dp_dat(TBD_SMT_R, 1, 0);//�·����ˢ��ʼ��������,��ԭ��������0
					bd_timer = 0;
					break;
				case 1://��ȡ������
					if(bd_timer ++ < 20)break;
					tx_com_dat(TPC_SMT_R, 50 , TPC_ERR_NOERROR, 0);//�ϱ�����
					tx_dp_dat(TBD_SMT_R, 3, 0);//�·����ˢֹͣ��������,��ԭ����������0
					bd_timer = 0;
					bdtest_sta_child = 2;
					break;
				case 2:
					if(bd_timer ++ < 20 && !tmd_rxbd_dat.unread_flag)break;//��ʱδ�յ�����
					bd_timer = 0;
					if(!tmd_rxbd_dat.unread_flag)//��ʱδ�յ�����,����԰�(�Կض˿�)ͨ��ʧ��
					{
						SET_RSIDE_MOTER(BDTEST_SMT_PWMVALUE);//ͣ��ˢ
						tx_com_dat(TPC_DPPORT_TST, 0, TPC_ERR_COMM, 0);//����ͨ�Ŵ���
						tx_dp_dat(TBD_SMT_R, 3, 0);//�·����ˢֹͣ��������,��ԭ����������0
						bdtest_sta_child = (bdtest_sta_child & 0xf0) + 2;
						bdtest_sta_child += 0x10;
						if(bdtest_sta_child > 0x50)
							bdtest_sta_child = 0x0f;
						break;
					}
					else
					{
						tmd_rxbd_dat.unread_flag = 0;
					}
					tmd_rxbd_dat.unread_flag = 0;
					
					tx_com_dat(TPC_SMT_R, 75, tmd_rxbd_dat.cd_smt_r < 100 ? TPC_ERR_FORWARD:TPC_ERR_NOERROR, 0);//�ϱ����Ȼ��ϱ������ת

					if(tmd_rxbd_dat.cd_smt_r < 100)
					{
					#if 0
						log_printf("\r\n tmd_rxbd_dat.cd_smt_r:%d\r\n",tmd_rxbd_dat.cd_smt_r);
						//bdtest_sta = TPC_SMT_R;//������һ������
						bdtest_sta_child = 4;
					#else
						bdtest_sta = TPC_MT_MID;//������һ������
						bdtest_sta_child = 0;
					#endif
						break;
					}
					
					tx_dp_dat(TBD_MTOC_SMT_R, 1, 0);//�·����ˢ��ʼ�������
					SET_RSIDE_MOTER(BDTEST_SMT_PWMVALUE);
					bdtest_sta_child = 3;
					
					break;
				case 3:
					if(bd_timer ++ < 20 && !tmd_rxbd_dat.unread_flag)break;//��ʱδ�յ�����
					bd_timer = 0;
					if(!tmd_rxbd_dat.unread_flag)//��ʱδ�յ�����,����԰�(�Կض˿�)ͨ��ʧ��
					{
						tx_com_dat(TPC_DPPORT_TST, 0, TPC_ERR_COMM, 0);//����ͨ�Ŵ���
						tx_dp_dat(TBD_MTOC_SMT_R, 1, 0);//�·����ˢ��ʼ�������
						bdtest_sta_child = (bdtest_sta_child & 0xf0) + 3;
						bdtest_sta_child += 0x10;
						if(bdtest_sta_child > 0x50)
							bdtest_sta_child = 0x0f;
						break;
					}
					else
					{
						tmd_rxbd_dat.unread_flag = 0;
					}
					if(tmd_rxbd_dat.mtoc_smt_r)
					{
						uint8_t i;//, j = 0;
						uint16_t n_mt_curr;
						
						SET_RSIDE_MOTER(BDTEST_SMT_PWMVALUE);
						delay_ms(10);
						for(i = 0;i < 8;i ++)
						{
							delay_ms(5);
							n_mt_curr = I_SMTR_ADC();
						#if BDTEST_LOGPRINT
							log_printf("curr:%d-%d\r\n",n_mt_curr,i);
						#endif
							if(n_mt_curr > 600)
							{
								//j ++;
								break;
							}
							//log_printf("i:%d\r\n",i);
						}

						tx_com_dat(TPC_SMT_R, 100, i < 5 ? TPC_ERR_NOERROR:TPC_ERR_OC, 0);//�ϱ����Ȼ��ϱ�����������
						if(i < 5)
						{
						#if 0
							//bdtest_sta = TPC_SMT_R;//������һ������
							bdtest_sta_child = 4;
							log_printf("**6**%d\r\n",i);
						#else
							bdtest_sta = TPC_MT_MID;//������һ������
							bdtest_sta_child = 0;
						#endif
							//break;
						}
						#if 0
						else
						{
							//bdtest_sta = TPC_SMT_R;//������һ������
							bdtest_sta_child = 5;
							log_printf("666666\r\n");
						}
						#endif
					}
					else
						tx_com_dat(TPC_SMT_R, 100, TPC_ERR_OC, 0);//�ϱ����Ȼ��ϱ�����������
					
					tx_dp_dat(TBD_MTOC_SMT_R, 0, 0);//�·�����ֹ��ز���,����·�ĵ���Ͽ�
					//bdtest_sta = TPC_WHL_R;
					//bdtest_sta_child = 0;
					//MOTOR_POWER_OFF_NPRI();
				#if 0
					break;
				case 4:
					log_printf("^");
					SET_RSIDE_MOTER(1000);
					MOTOR_POWER_OFF_NPRI();
					break;
				case 5:
					log_printf("%");
					SET_RSIDE_MOTER(1000);
					MOTOR_POWER_OFF_NPRI();
					break;
				#endif
				default:
					bdtest_sta = TPC_MT_MID;
					bdtest_sta_child = 0;
					SET_RSIDE_MOTER(1000);
					MOTOR_POWER_OFF_NPRI();
					break;
			}
			break;
		case TPC_MT_MID://������ɨ���
			switch(bdtest_sta_child&0x0f)
			{
				case 0://��ʼ��������
					bdtest_sta_child = 1;
					MOTOR_POWER_ON();
					SET_MID_MOTER(800);
					tx_com_dat(TPC_MT_MID, 25 , TPC_ERR_NOERROR, 0);//�ϱ�����
					tx_dp_dat(TBD_MT_MID, 1, 0);//�·����ˢ��ʼ��������,��ԭ��������0
					tx_dp_dat(TBD_MTOC_MT_MID, 0, 0);//�·�����ֹ��ز���,����·�ĵ���Ͽ�
					bd_timer = 0;
					break;
				case 1://��ȡ������
					if(bd_timer ++ < 20)break;
					tx_com_dat(TPC_MT_MID, 50 , TPC_ERR_NOERROR, 0);//�ϱ�����
					//tx_dp_dat(TBD_MT_MID, 0, 0);//�·����ˢֹͣ��������,��ԭ����������0
					tx_dp_dat(TBD_MT_MID, 3, 0);//�·����ˢֹͣ��������,��ԭ����������0
					bd_timer = 0;
					bdtest_sta_child = 2;
					break;
				case 2:
					if(bd_timer ++ < 20 && !tmd_rxbd_dat.unread_flag)break;//��ʱδ�յ�����
					bd_timer = 0;
					if(!tmd_rxbd_dat.unread_flag)//��ʱδ�յ�����,����԰�(�Կض˿�)ͨ��ʧ��
					{
						SET_MID_MOTER(0);
						tx_com_dat(TPC_DPPORT_TST, 0, TPC_ERR_COMM, 0);//����ͨ�Ŵ���
						tx_dp_dat(TBD_MT_MID, 3, 0);//�·����ˢֹͣ��������,��ԭ����������0
						bdtest_sta_child = (bdtest_sta_child & 0xf0) + 2;
						bdtest_sta_child += 0x10;
						if(bdtest_sta_child > 0x50)
							bdtest_sta_child = 0x0f;
						break;
					}
					else
					{
						tmd_rxbd_dat.unread_flag = 0;
					}
					tmd_rxbd_dat.unread_flag = 0;
					
					tx_com_dat(TPC_MT_MID, 75, tmd_rxbd_dat.cd_mt_mid < 400 ? TPC_ERR_FORWARD:TPC_ERR_NOERROR, 0);//�ϱ����Ȼ��ϱ������ת

					//delay_ms(10);
					//log_printf("\r\n tmd_rxbd_dat.cd_mt_mid:%d\r\n",tmd_rxbd_dat.cd_mt_mid);
					if(tmd_rxbd_dat.cd_mt_mid< 400)
					{
					#if 0
						//bdtest_sta = TPC_MT_MID;//������һ������
						bdtest_sta_child = 4;
					#else
						bdtest_sta = TPC_MT_FAN;//������һ������
						bdtest_sta_child = 0;
						MOTOR_POWER_OFF_NPRI();
					#endif
						break;
					}
					
					tx_dp_dat(TBD_MTOC_MT_MID, 1, 0);//�·����ˢ��ʼ�������
					SET_MID_MOTER(800);
					bdtest_sta_child = 3;
					
					break;
				case 3:
					if(bd_timer ++ < 20 && !tmd_rxbd_dat.unread_flag)break;//��ʱδ�յ�����
					bd_timer = 0;
					if(!tmd_rxbd_dat.unread_flag)//��ʱδ�յ�����,����԰�(�Կض˿�)ͨ��ʧ��
					{
						tx_com_dat(TPC_DPPORT_TST, 0, TPC_ERR_COMM, 0);//����ͨ�Ŵ���
						tx_dp_dat(TBD_MTOC_MT_MID, 1, 0);//�·����ˢ��ʼ�������
						bdtest_sta_child = (bdtest_sta_child & 0xf0) + 3;
						bdtest_sta_child += 0x10;
						if(bdtest_sta_child > 0x50)
							bdtest_sta_child = 0x0f;
						break;
					}
					else
					{
						tmd_rxbd_dat.unread_flag = 0;
					}
					if(tmd_rxbd_dat.mtoc_mt_mid)
					{
						uint8_t i;//, j = 0;
						uint16_t n_mt_curr;
						
						SET_MID_MOTER(800);
						delay_ms(10);
						for(i = 0;i < 8;i ++)
						{
							delay_ms(5);
							n_mt_curr = READ_MAIN_MOTOR();
						#if BDTEST_LOGPRINT
							log_printf("curr:%d-%d\r\n",n_mt_curr,i);
						#endif
							if(n_mt_curr > 150)
							{
								//j ++;
								SET_MID_MOTER(0);
								MOTOR_POWER_OFF_NPRI();
								break;
							}
							//log_printf("i:%d\r\n",i);
						}

						tx_com_dat(TPC_MT_MID, 100, i < 8 ? TPC_ERR_NOERROR:TPC_ERR_OC, 0);//�ϱ����Ȼ��ϱ�����������
						if(i < 8)
						{
						#if 0
							//bdtest_sta = TPC_SMT_R;//������һ������
							bdtest_sta_child = 4;
							log_printf("**6**%d\r\n",i);
						#else
							tx_dp_dat(TBD_MTOC_MT_MID, 0, 0);//�·�����ֹ��ز���,����·�ĵ���Ͽ�
							bdtest_sta = TPC_MT_FAN;//������һ������
							bdtest_sta_child = 0;
							SET_MID_MOTER(0);
							MOTOR_POWER_OFF_NPRI();
							break;
						#endif
						}
						#if 0
						else
						{
							//bdtest_sta = TPC_SMT_R;//������һ������
							bdtest_sta_child = 5;
							log_printf("666666\r\n");
						}
						#endif
					}
					else
					{
						//log_printf("\r\ntmd_rxbd_dat.mtoc_mt_mid=0\r\n");
						//tx_com_dat(TPC_MT_MID, 100, TPC_ERR_OC, 0);//�ϱ����Ȼ��ϱ�����������
						//MOTOR_POWER_OFF_NPRI();
						tx_dp_dat(TBD_MTOC_MT_MID, 1, 0);//�·����ˢ��ʼ�������
						bdtest_sta_child = 3;
						tmd_rxbd_dat.unread_flag = 0;
						break;
					}
					
					tx_com_dat(TPC_MT_MID, 100, TPC_ERR_OC, 0);//�ϱ����Ȼ��ϱ�����������
					
					tx_dp_dat(TBD_MTOC_MT_MID, 0, 0);//�·�����ֹ��ز���,����·�ĵ���Ͽ�
					//bdtest_sta = TPC_WHL_R;
					//bdtest_sta_child = 0;
					//MOTOR_POWER_OFF_NPRI();
				#if 0
					break;
				case 4:
					log_printf("^");
					//SET_MID_MOTER(0);
					//MOTOR_POWER_OFF_NPRI();
					break;
				case 5:
					log_printf("%");
					//SET_MID_MOTER(0);
					//MOTOR_POWER_OFF_NPRI();
					break;
				#endif
				default:
					bdtest_sta = TPC_MT_FAN;
					bdtest_sta_child = 0;
					SET_MID_MOTER(0);
					//SET_DST_MOTER(0);
					MOTOR_POWER_OFF_NPRI();
					break;
			}
			break;
		case TPC_MT_FAN:
			switch(bdtest_sta_child)
			{
				case 0://��ʼ��������
					bdtest_sta_child = 1;
					MOTOR_POWER_OFF_NPRI();
					delay_ms(500);
					SET_DST_MOTER(300);//�������
					delay_ms(100);
					//��ʼ�����ת���жϵ�IO��
					GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource13);   

					bdtest_fantest = 1;
					tmod_fancd = 0;
					
					tx_com_dat(TPC_MT_MID, 25 , TPC_ERR_NOERROR, 0);//�ϱ�����
					tx_dp_dat(TBD_MT_MID, 1, 0);//�·����ˢ��ʼ��������,��ԭ��������0
					bd_timer = 0;
					MOTOR_POWER_ON();
					break;
				case 1://��ȡ������
					if(bd_timer ++ < 20)break;
					MOTOR_POWER_OFF_NPRI();
					SET_DST_MOTER(0);//�رշ��
					bdtest_fantest = 0;
					GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource13);//�ָ��ֳ�
					#if 0
					if(tmod_fancd < 200)//�������С��20,�������������,����
					{
						log_printf("fan cd = %d\r\n",tmod_fancd);
					}
					delay_ms(200);
					#endif
					tx_com_dat(TPC_MT_FAN, 100 , tmod_fancd < 200 ? TPC_ERR_FORWARD:TPC_ERR_NOERROR, 0);//�ϱ����ȼ�������Ϣ
					
					bd_timer = 0;
				#if 0
					bdtest_sta_child = 2;
					break;
				case 2:
					log_printf("^");
					//SET_MID_MOTER(0);
					//SET_DST_MOTER(0)
					MOTOR_POWER_OFF_NPRI();
					break;
				#endif
				default:
					bdtest_sta = TPC_IR_FRT;
					bdtest_sta_child = 0;
					MOTOR_POWER_OFF_NPRI();
					bdtest_fantest = 0;
					break;
			}
			break;
		case TPC_IR_FRT://ǰײ���(��ײ)������,�Եغ���������һ����
			switch(bdtest_sta_child)
			{
				case 0:
					{
						uint8_t cc,bot;
						int32_t k1,k2;
						uint8_t i,j;
						
						sys->c_ir_adc = 0;
						for(cc = 0;cc < 9;cc ++)
						{
							for(i=0;i<10;i++)
							{
								sys->m_sta[i][cc] = adc_converted_value[i];
								//log_printf(",%d",sys->m_sta[i][sys->c_ir_adc] );
							}
							//log_printf("\r\n");

							if(cc <=3 )
							{
								GPIO_ResetBits(PORT_IR_CTRL,PIN_IR_CTRL);	//�͵�ƽ���ص�
								//GPIO_ResetBits(PORT_IR_CTRL2,PIN_IR_CTRL2); //�͵�ƽ���ص�
								//�ż�
								NEAR_LAN_ON();
								FAR_LAN_OFF();
							}
							else 
							{

								GPIO_SetBits(PORT_IR_CTRL,PIN_IR_CTRL);
								//GPIO_SetBits(PORT_IR_CTRL2,PIN_IR_CTRL2);
								NEAR_LAN_OFF();
								FAR_LAN_ON();
							}
							delay_ms(5);
						}

						cc = bot = 0;//cc������¼ǰײ����ļ����,j������¼�Եغ���ļ����
						for(i=0;i<10;i++)
						{
							k1 = k2 = 0;
							for(j=0;j<4;j++)
								k1 +=sys->m_sta[i][j];
							for(j=4;j<8;j++)
								k2 +=sys->m_sta[i][j];
							
							
							if(i >=7)
							{
								sys->g_buton[0][i-7] = k2 / 4; //sys->g_sta[i];
								if(sys->g_buton[0][i-7] > 300)//���˺���
								{
									bot |= 1 << (i - 7);
								}
								sys->g_buton[1][i-7] = k1 / 4;
								if(sys->g_buton[1][i-7] > 1500)//Զ�˺���
								{
									bot |= 1 << (i - 7 + 3);
								}
								//log_printf("\r\n%d\t%d\r\n",sys->g_buton[0][i-7],sys->g_buton[1][i-7]);
							}else
							{
								if(i == 1 || i == 5)
									continue;
								if(i == 2 || i == 4)
								{
									if(k1 > k2 && ((k1 - k2) /4) < 5000)
										sys->g_sta[i] = (k1 - k2) >> 2;
									else
										sys->g_sta[i] = 0;
									if(sys->g_sta[i] < 3300 || sys->g_sta[i] > 4030)//���˺���
									{
										cc |= 1 << i;
										//log_printf("err:%d-%d\r\n",i,sys->g_sta[i]);
									}
								}
								else
								{
									sys->g_sta[i] = (k1) >> 2;
									if(sys->g_sta[i] < 2800 || sys->g_sta[i] > 3800)//���˺���
									{
										cc |= 1 << i;
										//log_printf("err:%d-%d\r\n",i,sys->g_sta[i]);
									}
								}

							}
						//log_printf("%d,%d,%d,%d\r\n",i,k1 / 4,k2 /4,(k1-k2 ) / 4);
						}
						#if 0 //���ԵĴ�ӡ��Ϣ
						log_printf("\r\nirda:%02x\r\nirbt:%02x\r\n",cc,bot);
						delay_ms(500);
						#endif
						//delay_ms(500);
						#if 1
						#if 0
						delay_ms(500);
						log_printf("\r\nir_frt:\r\n");
						for(i = 0;i < 7;i ++)
						{
							log_printf("\t%d",sys->g_sta[i]);
						}
						log_printf("-%02X\r\nir_bot:\r\n",cc);
						for(i = 0;i < 3;i ++)
						{
							log_printf("\t%d",sys->g_buton[0][i]);
						}
						log_printf("-\t-");
						for(i = 0;i < 3;i ++)
						{
							log_printf("\t%d",sys->g_buton[1][i]);
						}
						log_printf("-%02X\r\n\r\n",bot);
						#else
						
						if(cc)
						{
							tx_com_dat(TPC_IR_FRT, 100 , TPC_ERR_IRLED, cc);//�ϱ����ȼ�������Ϣ
						}
						else
						{
							tx_com_dat(TPC_IR_FRT, 100 , TPC_ERR_NOERROR, 0);//�ϱ����ȼ�������Ϣ
						}

						if(bot)
						{
							tx_com_dat(TPC_IR_BOT, 100 , TPC_ERR_IRLED, bot);//�ϱ����ȼ�������Ϣ
						}
						else
						{
							tx_com_dat(TPC_IR_BOT, 100 , TPC_ERR_NOERROR, 0);//�ϱ����ȼ�������Ϣ
						}
						#endif
						#endif
					}
					//�ر����к���LED,�Խ��͹���
					FAR_LAN_OFF();
					NEAR_LAN_OFF();
					GPIO_ResetBits(PORT_IR_CTRL,PIN_IR_CTRL);
					//GPIO_ResetBits(PORT_IR_CTRL2,PIN_IR_CTRL2); //�͵�ƽ���ص�
					#if 1//�����ܲ���״̬��,��״̬ת����Ҫ�ر�һ��
					bdtest_sta = TPC_IRDA_CHRG;
					bdtest_sta_child = 0;
					#endif
					break;
				case 1:
				default:
					bdtest_sta_child = 0;
					bdtest_sta = TPC_IRDA_CHRG;
					break;
			}
			break;
		case TPC_IRDA_CHRG:
			switch(bdtest_sta_child)
			{
			case 0:
				{
					uint8_t n_irdata[3] = {0};
					
					read_ir_original_data(n_irdata);//��һ�κ���,��֮ǰ��ֵ���
				}
				bdtest_sta_child = 1;
				bd_batcharge_timer = 0;//�ѱ�����ʱ�������һ��,���������ղ��������źŵļ���
				break;
			case 1:
				{
					uint8_t n_irdata[3] = {0};
					uint8_t n_irres;
					//uint8_t n_irknk = 0;
					//uint8_t i,cc;
					read_ir_original_data(n_irdata);

					n_irres = (n_irdata[0] | n_irdata[1] | n_irdata[2]) & 0x01;//ȡ�����ź�
					n_irres ^= 0x01;
					
					if(!n_irdata[0])n_irres |= 0x08;
					if(!n_irdata[1])n_irres |= 0x02;
					if(!n_irdata[2])n_irres |= 0x04;
					
					#if 0
					log_printf("\r\n %02x %02x %02x res:%02x\r\n",n_irdata[0],n_irdata[1],n_irdata[2],n_irres);
					//log_printf("\r\n ---%02x---\r\n",n_irdata[0]);
					delay_ms(200);
					#else
					if(n_irres != 0 && bd_batcharge_timer ++ > 2)
					{
						#if 0
						log_printf("\r\n %02x %02x %02x res:%02x\r\n",n_irdata[0],n_irdata[1],n_irdata[2],n_irres);
						//log_printf("\r\n ---%02x---\r\n",n_irdata[0]);
						delay_ms(200);
						#else
						bd_batcharge_timer = 0;
						bdtest_sta = TPC_BUM_SW;
						tx_com_dat(TPC_IRDA_CHRG, 100 , n_irres!=0 ? TPC_ERR_IRLED:TPC_ERR_NOERROR, n_irres);//�ϱ����ȼ�������Ϣ
						#endif
					}
					else if(n_irres == 0)
					{
						bd_batcharge_timer = 0;
						bdtest_sta = TPC_BUM_SW;
						tx_com_dat(TPC_IRDA_CHRG, 100 , n_irres!=0 ? TPC_ERR_IRLED:TPC_ERR_NOERROR, n_irres);//�ϱ����ȼ�������Ϣ
					}
					#endif
				}
				#if 1//�����ܲ���״̬��,��״̬ת����Ҫ�ر�һ��
				#else
				bdtest_sta_child = 1;
				#endif
				break;
			case 2:
			default:
				break;
			}
			break;
		case TPC_BUM_SW://��ȡ��ײ��������
			switch(bdtest_sta_child&0x0f)
			{
				case 0://��ȡ�����޹�ʱ��GPIO������������,���������״̬��,��ʱӦ������ײ(�ߵ�ƽ)
					bdtest_sta_child = 1;
					{
						uint8_t n_res = 0;
						uint8_t i;
						
						for(i=0;i<20;i++)
						{
							if(((GPIOD->IDR) & MASK_BUM_LEFT)!=0)
								break;
						}
						if(i==20)
							n_res |= 0x01;
							
						for(i=0;i<20;i++)
						{
							if(((GPIOD->IDR) & MASK_BUM_RIGHT) !=0)
								break;
						}
						if(i==20)
							n_res |= 0x02;

						#if 0
						log_printf("\r\n double bum:%02x\r\n",n_res);
						delay_ms(100);
						#endif
						if(n_res)
						{
							tx_com_dat(TPC_BUM_SW, 25 , TPC_ERR_DIGI_IN, n_res);//�ϱ����ȼ�������Ϣ
						}
					}
					
					tx_com_dat(TPC_BUM_SW, 25 , TPC_ERR_NOERROR, 0);//�ϱ�����
					tx_dp_dat(TBD_BUM_SW, 1, 0x01);//����ײ�̽ӵ���
					bd_timer = 0;
					break;
				case 1:
					if(bd_timer ++ < 20 && !tmd_rxbd_dat.unread_flag)break;//��ʱδ�յ�����
					bd_timer = 0;
					if(!tmd_rxbd_dat.unread_flag)//��ʱδ�յ�����,����԰�(�Կض˿�)ͨ��ʧ��
					{
						tx_com_dat(TPC_DPPORT_TST, 0, TPC_ERR_COMM, 0);//����ͨ�Ŵ���
						tx_dp_dat(TBD_BUM_SW, 1, 0x01);//����ײ�̽ӵ���
						//bdtest_sta_child = 1;
						bdtest_sta_child = (bdtest_sta_child & 0xf0) + 1;
						bdtest_sta_child += 0x10;
						if(bdtest_sta_child > 0x50)
							bdtest_sta_child = 0x0f;
						break;
					}
					else
					{
						tmd_rxbd_dat.unread_flag = 0;
					}
					
					if(tmd_rxbd_dat.bum_sw_led)//led���
						tx_com_dat(TPC_BUM_SW, 50 , TPC_ERR_DIGI_OUT, (tmd_rxbd_dat.bum_sw_led));//�ϱ����ȼ�������Ϣ
						
					if(tmd_rxbd_dat.bum_sw == 0x01)
					{
						uint8_t i;
						uint8_t n_res = 0;
						
						for(i=0;i<20;i++)
						{
							if(((GPIOD->IDR) & MASK_BUM_LEFT) ==0)
								break;
						}
						if(i==20)
							n_res |= 0x01;
							
						for(i=0;i<20;i++)
						{
							if(((GPIOD->IDR) & MASK_BUM_RIGHT) !=0)
								break;
						}
						if(i==20)
							n_res |= 0x02;
							
						#if 0
						log_printf("\r\nleft bum:%02x\tled:%d\r\n",n_res,tmd_rxbd_dat.bum_sw_led);
						delay_ms(100);
						#endif
						if(n_res)
						{
							tx_com_dat(TPC_BUM_SW, 50 , TPC_ERR_DIGI_IN, n_res);//�ϱ����ȼ�������Ϣ
						}
						tx_com_dat(TPC_BUM_SW, 50 , TPC_ERR_NOERROR, 0);//�ϱ�����
					}
					//else
					//	tx_com_dat(TPC_BUM_SW, 50, TPC_ERR_TB, 0);//�ϱ����Ȼ��ϱ�����������
					
					tx_dp_dat(TBD_BUM_SW, 1, 0x02);//����ײ�ſ�,����ײ�̽ӵ���
					bdtest_sta_child = 2;
					break;
				case 2:
					if(bd_timer ++ < 20 && !tmd_rxbd_dat.unread_flag)break;//��ʱδ�յ�����
					bd_timer = 0;
					if(!tmd_rxbd_dat.unread_flag)//��ʱδ�յ�����,����԰�(�Կض˿�)ͨ��ʧ��
					{
						tx_com_dat(TPC_DPPORT_TST, 0, TPC_ERR_COMM, 0);//����ͨ�Ŵ���
						tx_dp_dat(TBD_BUM_SW, 1, 0x02);//����ײ�ſ�,����ײ�̽ӵ���
						bdtest_sta_child = (bdtest_sta_child & 0xf0) + 2;
						bdtest_sta_child += 0x10;
						if(bdtest_sta_child > 0x50)
							bdtest_sta_child = 0x0f;
						break;
					}
					else
					{
						tmd_rxbd_dat.unread_flag = 0;
					}
					if(tmd_rxbd_dat.bum_sw == 0x02)
					{
						uint8_t i;
						uint8_t n_res = 0;
						
						for(i=0;i<20;i++)
						{
							if(((GPIOD->IDR) & MASK_BUM_LEFT)!=0)
								break;
						}
						if(i==20)
							n_res |= 0x01;
							
						for(i=0;i<20;i++)
						{
							if(((GPIOD->IDR) & MASK_BUM_RIGHT) ==0)
								break;
						}
						if(i==20)
							n_res |= 0x02;
						#if 0
						log_printf("\r\nright bum:%02x\r\n",n_res);
						delay_ms(100);
						#endif
						if(n_res)
						{
							tx_com_dat(TPC_BUM_SW, 100 , TPC_ERR_DIGI_IN, n_res);//�ϱ����ȼ�������Ϣ
						}
						tx_com_dat(TPC_BUM_SW, 100 , TPC_ERR_NOERROR, 0);//�ϱ�����
					}
					else
						tx_com_dat(TPC_BUM_SW, 100, TPC_ERR_TB, 0);//�ϱ����Ȼ��ϱ�����������
					
					tx_dp_dat(TBD_BUM_SW, 0, 0);//������ײ�ſ�
					//bdtest_sta_child = 2;
				#if 0
					bdtest_sta_child = 3;
					break;
				case 3:
					break;
				#endif
				default:
					bdtest_sta = TPC_SWSENSOR;
					bdtest_sta_child = 0;
					bdtest_fantest = 0;
					break;
			}
			break;
		case TPC_SWSENSOR://��ȡ��������ʽ����������
			switch(bdtest_sta_child&0x0f)
			{
				case 0://��ȡδ���̽ӵ���ʱ�ĸ�����������
					bdtest_sta_child = 1;
					{
						uint8_t n_res = 0;
						uint8_t i;
						
						for(i=0;i<20;i++)//Ĩ�����
						{
							if(READ_MOP_DET() != 0)//�ſ�ʱӦΪ�ߵ�ƽ,�����⵽�͵�ƽ���ж�Ϊ����쳣
								break;
						}
						if(i==20)
							n_res |= 0x01;
						
						for(i=0;i<20;i++)//����ǽ,�궨���Ƿ���...
						{
							if(READ_VWALL_DET() == 0)//�ſ�ʱӦΪ�ߵ�ƽ,�����⵽�͵�ƽ���ж�Ϊ����쳣
								break;
						}
						if(i==20)
							n_res |= 0x02;
						
						for(i=0;i<20;i++)//���м��
						{
							if(READ_DUSTBOX_DET() != 0)//�ſ�ʱӦΪ�ߵ�ƽ,�����⵽�͵�ƽ���ж�Ϊ����쳣
								break;
						}
						if(i==20)
							n_res |= 0x04;
						
						for(i=0;i<20;i++)//�������
						{
							if(READ_DUST_DET() != 0)//�ſ�ʱӦΪ�ߵ�ƽ,�����⵽�͵�ƽ���ж�Ϊ����쳣
								break;
						}
						if(i==20)
							n_res |= 0x08;
						
						for(i=0;i<20;i++)//������
						{
							if(LEFT_MOTOR_LEAVE() != 0)//�ſ�ʱӦΪ�ߵ�ƽ,�����⵽�͵�ƽ���ж�Ϊ����쳣
								break;
						}
						if(i==20)
							n_res |= 0x10;
						
						for(i=0;i<20;i++)//������
						{
							if(RIGHT_MOTOR_LEAVE() != 0)//�ſ�ʱӦΪ�ߵ�ƽ,�����⵽�͵�ƽ���ж�Ϊ����쳣
								break;
						}
						if(i==20)
							n_res |= 0x20;
						if(mcu_get_wifi_work_state() == 0xff)
						{
							n_res |= 0x40;
						}

						#if BDTEST_LOGPRINT
						log_printf("\r\n case 0:%02x\r\n",n_res);
						delay_ms(100);
						#else
						if(n_res)
						{
							tx_com_dat(TPC_SWSENSOR, 25 , TPC_ERR_DIGI_IN, n_res);//�ϱ����ȼ�������Ϣ
						}
						#endif
					}
					
					tx_com_dat(TPC_SWSENSOR, 25 , TPC_ERR_NOERROR, 0);//�ϱ�����
					tx_dp_dat(TBD_SWSENSOR, 1, 0x01);//Ĩ������������
					bd_timer = 0;
					break;
				case 1:
					//bdtest_sta_child = (bdtest_sta_child & 0xf0) + 2;
					if(bd_timer ++ < 20 && !tmd_rxbd_dat.unread_flag)break;//��ʱδ�յ�����
					bd_timer = 0;
					if(!tmd_rxbd_dat.unread_flag)//��ʱδ�յ�����,����԰�(�Կض˿�)ͨ��ʧ��
					{
						tx_com_dat(TPC_DPPORT_TST, 0, TPC_ERR_COMM, 0);//����ͨ�Ŵ���
						tx_dp_dat(TBD_SWSENSOR, 1, 0x01);//Ĩ������������
						
						tx_com_dat(TPC_SWSENSOR, 90, TPC_ERR_TB, tmd_rxbd_dat.sensors_sw);//�ϱ����Ȼ��ϱ�����������
						bdtest_sta_child = (bdtest_sta_child & 0xf0) + 1;
						bdtest_sta_child += 0x10;
						if(bdtest_sta_child > 0x50)
							bdtest_sta_child = 0x0f;
						break;
					}
					else
					{
						tmd_rxbd_dat.unread_flag = 0;
					}
					
					if(tmd_rxbd_dat.sensors_sw_led)//led���
						tx_com_dat(TBD_SWSENSOR, 50 , TPC_ERR_DIGI_OUT, tmd_rxbd_dat.sensors_sw_led);//�ϱ����ȼ�������Ϣ

					#if BDTEST_LOGPRINT
					log_printf("\r\nled:%d\r\n",tmd_rxbd_dat.sensors_sw_led);
					delay_ms(100);
					#endif
						
					if(tmd_rxbd_dat.sensors_sw== 0x01)
					{
						uint8_t n_res = 0;
						uint8_t i;
						
						for(i=0;i<20;i++)//Ĩ�����
						{
							if(READ_MOP_DET() == 0)//�ſ�ʱӦΪ�ߵ�ƽ,�����⵽�͵�ƽ���ж�Ϊ����쳣
								break;
						}
						if(i==20)
							n_res |= 0x01;
						
						for(i=0;i<20;i++)//����ǽ
						{
							if(READ_VWALL_DET() == 0)//�ſ�ʱӦΪ�ߵ�ƽ,�����⵽�͵�ƽ���ж�Ϊ����쳣
								break;
						}
						if(i==20)
							n_res |= 0x02;
						
						for(i=0;i<20;i++)//���м��
						{
							if(READ_DUSTBOX_DET() != 0)//�ſ�ʱӦΪ�ߵ�ƽ,�����⵽�͵�ƽ���ж�Ϊ����쳣
								break;
						}
						if(i==20)
							n_res |= 0x04;
						/*
						for(i=0;i<20;i++)//�������
						{
							if(READ_DUST_DET() != 0)//�ſ�ʱӦΪ�ߵ�ƽ,�����⵽�͵�ƽ���ж�Ϊ����쳣
								break;
						}
						if(i==20)
							n_res |= 0x08;
						*/
						for(i=0;i<20;i++)//������
						{
							if(LEFT_MOTOR_LEAVE() != 0)//�ſ�ʱӦΪ�ߵ�ƽ,�����⵽�͵�ƽ���ж�Ϊ����쳣
								break;
						}
						if(i==20)
							n_res |= 0x10;
						
						for(i=0;i<20;i++)//������
						{
							if(RIGHT_MOTOR_LEAVE() != 0)//�ſ�ʱӦΪ�ߵ�ƽ,�����⵽�͵�ƽ���ж�Ϊ����쳣
								break;
						}
						if(i==20)
							n_res |= 0x20;

						if(mcu_get_wifi_work_state() == 0xff)
						{
							n_res |= 0x40;
						}
						#if BDTEST_LOGPRINT
						log_printf("\r\n case 1:%02x\r\n",n_res);
						delay_ms(100);
						#else
						if(n_res)
						{
							tx_com_dat(TPC_SWSENSOR, 50 , TPC_ERR_DIGI_IN, n_res);//�ϱ����ȼ�������Ϣ
						}
						#endif
					}
					else
						tx_com_dat(TPC_SWSENSOR, 50, TPC_ERR_TB, tmd_rxbd_dat.sensors_sw);//�ϱ����Ȼ��ϱ�����������
					
					tx_com_dat(TPC_SWSENSOR, 50 , TPC_ERR_NOERROR, 0);//�ϱ�����
					tx_dp_dat(TBD_SWSENSOR, 1, 0x02);//����ǽ����������
					bd_timer = 0;
					bdtest_sta_child = 2;
					break;
				case 2:
					//bdtest_sta_child = (bdtest_sta_child & 0xf0) + 3;
					if(bd_timer ++ < 20 && !tmd_rxbd_dat.unread_flag)break;//��ʱδ�յ�����
					bd_timer = 0;
					if(!tmd_rxbd_dat.unread_flag)//��ʱδ�յ�����,����԰�(�Կض˿�)ͨ��ʧ��
					{
						tx_com_dat(TPC_DPPORT_TST, 0, TPC_ERR_COMM, 0);//����ͨ�Ŵ���
						tx_dp_dat(TBD_SWSENSOR, 1, 0x02);//����ǽ����������
						bdtest_sta_child = (bdtest_sta_child & 0xf0) + 2;
						bdtest_sta_child += 0x10;
						if(bdtest_sta_child > 0x50)
							bdtest_sta_child = 0x0f;
						break;
					}
					else
					{
						tmd_rxbd_dat.unread_flag = 0;
					}
					if(tmd_rxbd_dat.sensors_sw== 0x02)
					{
						uint8_t n_res = 0;
						uint8_t i;
						
						for(i=0;i<20;i++)//Ĩ�����
						{
							if(READ_MOP_DET() != 0)//�ſ�ʱӦΪ�ߵ�ƽ,�����⵽�͵�ƽ���ж�Ϊ����쳣
								break;
						}
						if(i==20)
							n_res |= 0x01;
						
						for(i=0;i<20;i++)//����ǽ
						{
							if(READ_VWALL_DET() != 0)//�ſ�ʱӦΪ�ߵ�ƽ,�����⵽�͵�ƽ���ж�Ϊ����쳣
								break;
						}
						if(i==20)
							n_res |= 0x02;
						
						for(i=0;i<20;i++)//���м��
						{
							if(READ_DUSTBOX_DET() != 0)//�ſ�ʱӦΪ�ߵ�ƽ,�����⵽�͵�ƽ���ж�Ϊ����쳣
								break;
						}
						if(i==20)
							n_res |= 0x04;
						/*
						for(i=0;i<20;i++)//�������
						{
							if(READ_DUST_DET() != 0)//�ſ�ʱӦΪ�ߵ�ƽ,�����⵽�͵�ƽ���ж�Ϊ����쳣
								break;
						}
						if(i==20)
							n_res |= 0x08;
						*/
						for(i=0;i<20;i++)//������
						{
							if(LEFT_MOTOR_LEAVE() != 0)//�ſ�ʱӦΪ�ߵ�ƽ,�����⵽�͵�ƽ���ж�Ϊ����쳣
								break;
						}
						if(i==20)
							n_res |= 0x10;
						
						for(i=0;i<20;i++)//������
						{
							if(RIGHT_MOTOR_LEAVE() != 0)//�ſ�ʱӦΪ�ߵ�ƽ,�����⵽�͵�ƽ���ж�Ϊ����쳣
								break;
						}
						if(i==20)
							n_res |= 0x20;

						if(mcu_get_wifi_work_state() == 0xff)
						{
							n_res |= 0x40;
						}
						#if BDTEST_LOGPRINT
						log_printf("\r\n case 2:%02x\r\n",n_res);
						delay_ms(100);
						#else
						if(n_res)
						{
							tx_com_dat(TPC_SWSENSOR, 60 , TPC_ERR_DIGI_IN, n_res);//�ϱ����ȼ�������Ϣ
						}
						#endif
					}
					else
						tx_com_dat(TPC_SWSENSOR, 60, TPC_ERR_TB, tmd_rxbd_dat.sensors_sw);//�ϱ����Ȼ��ϱ�����������
					
					tx_com_dat(TPC_SWSENSOR, 60 , TPC_ERR_NOERROR, 0);//�ϱ�����
					tx_dp_dat(TBD_SWSENSOR, 1, 0x04);//����ǽ����������
					bd_timer = 0;
					bdtest_sta_child = 3;
					break;
				case 3:
					//bdtest_sta_child = (bdtest_sta_child & 0xf0) + 4;
					if(bd_timer ++ < 20 && !tmd_rxbd_dat.unread_flag)break;//��ʱδ�յ�����
					bd_timer = 0;
					if(!tmd_rxbd_dat.unread_flag)//��ʱδ�յ�����,����԰�(�Կض˿�)ͨ��ʧ��
					{
						tx_com_dat(TPC_DPPORT_TST, 0, TPC_ERR_COMM, 0);//����ͨ�Ŵ���
						tx_dp_dat(TBD_SWSENSOR, 1, 0x04);//����ǽ����������
						bdtest_sta_child = (bdtest_sta_child & 0xf0) + 3;
						bdtest_sta_child += 0x10;
						if(bdtest_sta_child > 0x50)
							bdtest_sta_child = 0x0f;
						break;
					}
					else
					{
						tmd_rxbd_dat.unread_flag = 0;
					}
					if(tmd_rxbd_dat.sensors_sw== 0x04)
					{
						uint8_t n_res = 0;
						uint8_t i;
						
						for(i=0;i<20;i++)//Ĩ�����
						{
							if(READ_MOP_DET() != 0)//�ſ�ʱӦΪ�ߵ�ƽ,�����⵽�͵�ƽ���ж�Ϊ����쳣
								break;
						}
						if(i==20)
							n_res |= 0x01;
						
						for(i=0;i<20;i++)//����ǽ
						{
							if(READ_VWALL_DET() == 0)//�ſ�ʱӦΪ�ߵ�ƽ,�����⵽�͵�ƽ���ж�Ϊ����쳣
								break;
						}
						if(i==20)
							n_res |= 0x02;
						
						for(i=0;i<20;i++)//���м��
						{
							if(READ_DUSTBOX_DET() == 0)//�ſ�ʱӦΪ�ߵ�ƽ,�����⵽�͵�ƽ���ж�Ϊ����쳣
								break;
						}
						if(i==20)
							n_res |= 0x04;
						/*
						for(i=0;i<20;i++)//�������
						{
							if(READ_DUST_DET() != 0)//�ſ�ʱӦΪ�ߵ�ƽ,�����⵽�͵�ƽ���ж�Ϊ����쳣
								break;
						}
						if(i==20)
							n_res |= 0x08;
						*/
						for(i=0;i<20;i++)//������
						{
							if(LEFT_MOTOR_LEAVE() != 0)//�ſ�ʱӦΪ�ߵ�ƽ,�����⵽�͵�ƽ���ж�Ϊ����쳣
								break;
						}
						if(i==20)
							n_res |= 0x10;
						
						for(i=0;i<20;i++)//������
						{
							if(RIGHT_MOTOR_LEAVE() != 0)//�ſ�ʱӦΪ�ߵ�ƽ,�����⵽�͵�ƽ���ж�Ϊ����쳣
								break;
						}
						if(i==20)
							n_res |= 0x20;

						if(mcu_get_wifi_work_state() == 0xff)
						{
							n_res |= 0x40;
						}
						#if BDTEST_LOGPRINT
						log_printf("\r\n case 3:%02x\r\n",n_res);
						delay_ms(100);
						#else
						if(n_res)
						{
							tx_com_dat(TPC_SWSENSOR, 70 , TPC_ERR_DIGI_IN, n_res);//�ϱ����ȼ�������Ϣ
						}
						#endif
					}
					else
						tx_com_dat(TPC_SWSENSOR, 70, TPC_ERR_TB, tmd_rxbd_dat.sensors_sw);//�ϱ����Ȼ��ϱ�����������
					
					tx_com_dat(TPC_SWSENSOR, 70 , TPC_ERR_NOERROR, 0);//�ϱ�����
					tx_dp_dat(TBD_SWSENSOR, 1, 0x08);//����ǽ����������
					bd_timer = 0;
					bdtest_sta_child = 4;
					break;
				case 4:
					//bdtest_sta_child = (bdtest_sta_child & 0xf0) + 5;
					if(bd_timer ++ < 20 && !tmd_rxbd_dat.unread_flag)break;//��ʱδ�յ�����
					bd_timer = 0;
					if(!tmd_rxbd_dat.unread_flag)//��ʱδ�յ�����,����԰�(�Կض˿�)ͨ��ʧ��
					{
						tx_com_dat(TPC_DPPORT_TST, 0, TPC_ERR_COMM, 0);//����ͨ�Ŵ���
						tx_dp_dat(TBD_SWSENSOR, 1, 0x08);//����ǽ����������
						bdtest_sta_child = (bdtest_sta_child & 0xf0) + 4;
						bdtest_sta_child += 0x10;
						if(bdtest_sta_child > 0x50)
							bdtest_sta_child = 0x0f;
						break;
					}
					else
					{
						tmd_rxbd_dat.unread_flag = 0;
					}
					if(tmd_rxbd_dat.sensors_sw== 0x08)
					{
						uint8_t n_res = 0;
						uint8_t i;
						
						for(i=0;i<20;i++)//Ĩ�����
						{
							if(READ_MOP_DET() != 0)//�ſ�ʱӦΪ�ߵ�ƽ,�����⵽�͵�ƽ���ж�Ϊ����쳣
								break;
						}
						if(i==20)
							n_res |= 0x01;
						
						for(i=0;i<20;i++)//����ǽ
						{
							if(READ_VWALL_DET() == 0)//�ſ�ʱӦΪ�ߵ�ƽ,�����⵽�͵�ƽ���ж�Ϊ����쳣
								break;
						}
						if(i==20)
							n_res |= 0x02;
						
						for(i=0;i<20;i++)//���м��
						{
							if(READ_DUSTBOX_DET() != 0)//�ſ�ʱӦΪ�ߵ�ƽ,�����⵽�͵�ƽ���ж�Ϊ����쳣
								break;
						}
						if(i==20)
							n_res |= 0x04;
						/*
						for(i=0;i<20;i++)//�������
						{
							if(READ_DUST_DET() == 0)//�ſ�ʱӦΪ�ߵ�ƽ,�����⵽�͵�ƽ���ж�Ϊ����쳣
								break;
						}
						if(i==20)
							n_res |= 0x08;
						*/
						for(i=0;i<20;i++)//������
						{
							if(LEFT_MOTOR_LEAVE() != 0)//�ſ�ʱӦΪ�ߵ�ƽ,�����⵽�͵�ƽ���ж�Ϊ����쳣
								break;
						}
						if(i==20)
							n_res |= 0x10;
						
						for(i=0;i<20;i++)//������
						{
							if(RIGHT_MOTOR_LEAVE() != 0)//�ſ�ʱӦΪ�ߵ�ƽ,�����⵽�͵�ƽ���ж�Ϊ����쳣
								break;
						}
						if(i==20)
							n_res |= 0x20;

						if(mcu_get_wifi_work_state() == 0xff)
						{
							n_res |= 0x40;
						}
						#if BDTEST_LOGPRINT
						log_printf("\r\n case 4:%02x\r\n",n_res);
						delay_ms(100);
						#else
						if(n_res)
						{
							tx_com_dat(TPC_SWSENSOR, 80 , TPC_ERR_DIGI_IN, n_res);//�ϱ����ȼ�������Ϣ
						}
						#endif
					}
					else
						tx_com_dat(TPC_SWSENSOR, 80, TPC_ERR_TB, tmd_rxbd_dat.sensors_sw);//�ϱ����Ȼ��ϱ�����������
					
					tx_com_dat(TPC_SWSENSOR, 80 , TPC_ERR_NOERROR, 0);//�ϱ�����
					tx_dp_dat(TBD_SWSENSOR, 1, 0x10);//����ǽ����������
					bd_timer = 0;
					bdtest_sta_child = 5;
					break;
				case 5:
					//bdtest_sta_child = (bdtest_sta_child & 0xf0) + 6;
					if(bd_timer ++ < 20 && !tmd_rxbd_dat.unread_flag)break;//��ʱδ�յ�����
					bd_timer = 0;
					if(!tmd_rxbd_dat.unread_flag)//��ʱδ�յ�����,����԰�(�Կض˿�)ͨ��ʧ��
					{
						tx_com_dat(TPC_DPPORT_TST, 0, TPC_ERR_COMM, 0);//����ͨ�Ŵ���
						tx_dp_dat(TBD_SWSENSOR, 1, 0x10);//����ǽ����������
						bdtest_sta_child = (bdtest_sta_child & 0xf0) + 5;
						bdtest_sta_child += 0x10;
						if(bdtest_sta_child > 0x50)
							bdtest_sta_child = 0x0f;
						break;
					}
					else
					{
						tmd_rxbd_dat.unread_flag = 0;
					}
					if(tmd_rxbd_dat.sensors_sw== 0x10)
					{
						uint8_t n_res = 0;
						uint8_t i;
						
						for(i=0;i<20;i++)//Ĩ�����
						{
							if(READ_MOP_DET() != 0)//�ſ�ʱӦΪ�ߵ�ƽ,�����⵽�͵�ƽ���ж�Ϊ����쳣
								break;
						}
						if(i==20)
							n_res |= 0x01;
						
						for(i=0;i<20;i++)//����ǽ
						{
							if(READ_VWALL_DET() == 0)//�ſ�ʱӦΪ�ߵ�ƽ,�����⵽�͵�ƽ���ж�Ϊ����쳣
								break;
						}
						if(i==20)
							n_res |= 0x02;
						
						for(i=0;i<20;i++)//���м��
						{
							if(READ_DUSTBOX_DET() != 0)//�ſ�ʱӦΪ�ߵ�ƽ,�����⵽�͵�ƽ���ж�Ϊ����쳣
								break;
						}
						if(i==20)
							n_res |= 0x04;
						/*
						for(i=0;i<20;i++)//�������
						{
							if(READ_DUST_DET() != 0)//�ſ�ʱӦΪ�ߵ�ƽ,�����⵽�͵�ƽ���ж�Ϊ����쳣
								break;
						}
						if(i==20)
							n_res |= 0x08;
						*/
						for(i=0;i<20;i++)//������
						{
							if(LEFT_MOTOR_LEAVE() == 0)//�ſ�ʱӦΪ�ߵ�ƽ,�����⵽�͵�ƽ���ж�Ϊ����쳣
								break;
						}
						if(i==20)
							n_res |= 0x10;
						
						for(i=0;i<20;i++)//������
						{
							if(RIGHT_MOTOR_LEAVE() != 0)//�ſ�ʱӦΪ�ߵ�ƽ,�����⵽�͵�ƽ���ж�Ϊ����쳣
								break;
						}
						if(i==20)
							n_res |= 0x20;

						if(mcu_get_wifi_work_state() == 0xff)
						{
							n_res |= 0x40;
						}
						#if BDTEST_LOGPRINT
						log_printf("\r\n case 5:%02x\r\n",n_res);
						delay_ms(100);
						#else
						if(n_res)
						{
							tx_com_dat(TPC_SWSENSOR, 90 , TPC_ERR_DIGI_IN, n_res);//�ϱ����ȼ�������Ϣ
						}
						#endif
					}
					else
						tx_com_dat(TPC_SWSENSOR, 90, TPC_ERR_TB, tmd_rxbd_dat.sensors_sw);//�ϱ����Ȼ��ϱ�����������
					
					tx_com_dat(TPC_SWSENSOR, 90 , TPC_ERR_NOERROR, 0);//�ϱ�����
					tx_dp_dat(TBD_SWSENSOR, 1, 0x20);//����ǽ����������
					bd_timer = 0;
					bdtest_sta_child = 6;
					break;
				case 6:
					//bdtest_sta_child = (bdtest_sta_child & 0xf0);
					if(bd_timer ++ < 20 && !tmd_rxbd_dat.unread_flag)break;//��ʱδ�յ�����
					bd_timer = 0;
					if(!tmd_rxbd_dat.unread_flag)//��ʱδ�յ�����,����԰�(�Կض˿�)ͨ��ʧ��
					{
						tx_com_dat(TPC_DPPORT_TST, 0, TPC_ERR_COMM, 0);//����ͨ�Ŵ���
						tx_dp_dat(TBD_SWSENSOR, 1, 0x20);//����ǽ����������
						bdtest_sta_child = (bdtest_sta_child & 0xf0) + 6;
						bdtest_sta_child += 0x10;
						if(bdtest_sta_child > 0x50)
							bdtest_sta_child = 0x0f;
						break;
					}
					else
					{
						tmd_rxbd_dat.unread_flag = 0;
					}
					if(tmd_rxbd_dat.sensors_sw== 0x20)
					{
						uint8_t n_res = 0;
						uint8_t i;
						
						for(i=0;i<20;i++)//Ĩ�����
						{
							if(READ_MOP_DET() != 0)//�ſ�ʱӦΪ�ߵ�ƽ,�����⵽�͵�ƽ���ж�Ϊ����쳣
								break;
						}
						if(i==20)
							n_res |= 0x01;
						
						for(i=0;i<20;i++)//����ǽ
						{
							if(READ_VWALL_DET() == 0)//�ſ�ʱӦΪ�ߵ�ƽ,�����⵽�͵�ƽ���ж�Ϊ����쳣
								break;
						}
						if(i==20)
							n_res |= 0x02;
						
						for(i=0;i<20;i++)//���м��
						{
							if(READ_DUSTBOX_DET() != 0)//�ſ�ʱӦΪ�ߵ�ƽ,�����⵽�͵�ƽ���ж�Ϊ����쳣
								break;
						}
						if(i==20)
							n_res |= 0x04;
						/*
						for(i=0;i<20;i++)//�������
						{
							if(READ_DUST_DET() != 0)//�ſ�ʱӦΪ�ߵ�ƽ,�����⵽�͵�ƽ���ж�Ϊ����쳣
								break;
						}
						if(i==20)
							n_res |= 0x08;
						*/
						for(i=0;i<20;i++)//������
						{
							if(LEFT_MOTOR_LEAVE() != 0)//�ſ�ʱӦΪ�ߵ�ƽ,�����⵽�͵�ƽ���ж�Ϊ����쳣
								break;
						}
						if(i==20)
							n_res |= 0x10;
						
						for(i=0;i<20;i++)//������
						{
							if(RIGHT_MOTOR_LEAVE() == 0)//�ſ�ʱӦΪ�ߵ�ƽ,�����⵽�͵�ƽ���ж�Ϊ����쳣
								break;
						}
						if(i==20)
							n_res |= 0x20;

						if(mcu_get_wifi_work_state() == 0xff)
						{
							n_res |= 0x40;
						}
						#if BDTEST_LOGPRINT
						log_printf("\r\n case 6:%02x\r\n",n_res);
						delay_ms(100);
						#else
						if(n_res)
						{
							tx_com_dat(TPC_SWSENSOR, 100 , TPC_ERR_DIGI_IN, n_res);//�ϱ����ȼ�������Ϣ
						}
						#endif
					}
					else
						tx_com_dat(TPC_SWSENSOR, 100, TPC_ERR_TB, tmd_rxbd_dat.sensors_sw);//�ϱ����Ȼ��ϱ�����������
					
					tx_com_dat(TPC_SWSENSOR, 100 , TPC_ERR_NOERROR, 0);//�ϱ�����
					tx_dp_dat(TBD_SWSENSOR, 0, 0);//����ǽ����������
					bd_timer = 0;
					bdtest_sta_child = 0;
				#if BDTEST_LOGPRINT
					bdtest_sta_child = 7;
					break;
				case 7:
					break;
				#endif
				default:
					bdtest_sta = TPC_CD_FRT;
					bdtest_sta_child = 0;
					MOTOR_POWER_OFF_NPRI();
					bdtest_fantest = 0;
					break;
			}
			break;
		case TPC_CD_FRT://����ǰ������
			switch(bdtest_sta_child&0x0f)
			{
				case 0://��ȡδ���̽ӵ���ʱ�ĸ�����������
					bdtest_sta_child = 1;
					#if BDTEST_LOGPRINT
					log_printf("\r\n frt:00\r\n");
					#else
					tx_com_dat(TPC_CD_FRT, 25 , TPC_ERR_NOERROR, 0);//�ϱ�����
					#endif
					tx_dp_dat(TBD_CD_FRT, 1, 0);//������ǰת
					delay_ms(50);
					bd_timer = 0;
					navigat->distance = 0;
					break;
				case 1:
					if(bd_timer ++ < 20)// && !tmd_rxbd_dat.unread_flag)
					{
						//bd_timer = 0;
						break;
					}
					#if BDTEST_LOGPRINT
					log_printf("\r\nfront\r\n");
					#endif
					tx_dp_dat(TBD_CD_FRT, 0, 0);//������
					bdtest_sta_child = 2;
					bd_timer = 0;
					break;
				case 2:
					if(bd_timer ++ < 20 && !tmd_rxbd_dat.unread_flag)
					{
						//bd_timer = 0;
						break;
					}
					bd_timer = 0;
					if(!tmd_rxbd_dat.unread_flag)//��ʱδ�յ�����,����԰�(�Կض˿�)ͨ��ʧ��
					{
						#if 1
						tx_com_dat(TPC_DPPORT_TST, 0, TPC_ERR_COMM, 0);//����ͨ�Ŵ���
						#else
						log_printf("\r\n frt:22\r\n");
						#endif
						tx_dp_dat(TBD_CD_FRT, 1, 0);//������ǰת
						delay_ms(50);
						//bdtest_sta_child = 2;
						bdtest_sta_child = (bdtest_sta_child & 0xf0) + 2;
						bdtest_sta_child += 0x10;
						if(bdtest_sta_child > 0x50)
							bdtest_sta_child = 0x0f;
						break;
					}
					else
					{
						tmd_rxbd_dat.unread_flag = 0;
					}
					bdtest_sta_child = 3;
					
					if(navigat->distance < 50)//����������
					{
						MOTOR_POWER_OFF_NPRI();
						//motor_wheel_forward(LEFT_WHEEL,GO_FORWARD,600);
						#if 1
						tx_com_dat(TBD_CD_FRT, 50, TPC_ERR_FORWARD, 0);
						#endif
						tx_dp_dat(TBD_CD_FRT, 0, 0);//����ͣת
						#if BDTEST_LOGPRINT
						bdtest_sta_child = 5;
						#else
						bdtest_sta = TPC_CHRG_TST;//������һ������
						//tx_dp_dat(TBD_CHRG_TST, 1, 0);
						bdtest_sta_child = 0;
						#endif
						break;
					}
					
					#if BDTEST_LOGPRINT
					log_printf("\r\nFHW:%d\r\n",navigat->distance);
					#else
					tx_com_dat(TPC_CD_FRT, 50 , TPC_ERR_NOERROR, 0);//�ϱ�����
					#endif
					tx_dp_dat(TBD_CD_FRT, 2, 0);//���̷�ת
					navigat->distance = 0;
					bd_timer = 0;
					break;
				case 3:
					if(bd_timer ++ < 20)// && !tmd_rxbd_dat.unread_flag)
					{
						//bd_timer = 0;
						break;
					}
					#if BDTEST_LOGPRINT
					log_printf("\r\nback\r\n");
					#endif
					tx_dp_dat(TBD_CD_FRT, 0, 0);//������
					bdtest_sta_child = 4;
					bd_timer = 0;
					break;
				case 4:
					if(bd_timer ++ < 20)// && !tmd_rxbd_dat.unread_flag)
					{
						//bd_timer = 0;
						//tx_dp_dat(TBD_CD_FRT, 2, 0);//�������ת
						break;//��ʱδ�յ�����
					}
					bd_timer = 0;
					if(!tmd_rxbd_dat.unread_flag)//��ʱδ�յ�����,����԰�(�Կض˿�)ͨ��ʧ��
					{
						#if 1
						tx_com_dat(TPC_DPPORT_TST, 0, TPC_ERR_COMM, 0);//����ͨ�Ŵ���
						#endif
						tx_dp_dat(TBD_CD_FRT, 2, 0);//������ǰת
						bdtest_sta_child = (bdtest_sta_child & 0xf0) + 4;
						bdtest_sta_child += 0x10;
						if(bdtest_sta_child > 0x50)
							bdtest_sta_child = 0x0f;
						break;
					}
					else
					{
						tmd_rxbd_dat.unread_flag = 0;
					}
					bdtest_sta_child = 0;
					
					if(navigat->distance > -50)//����������
					//if(0)//(navigat->distance > -50)//����������
					{
						MOTOR_POWER_OFF_NPRI();
						//motor_wheel_forward(LEFT_WHEEL,GO_FORWARD,600);
						#if 1
						tx_com_dat(TBD_CD_FRT, 100, TPC_ERR_BACK, 0);
						#endif
						tx_dp_dat(TBD_CD_FRT, 0, 0);//����ͣת
						#if BDTEST_LOGPRINT
						bdtest_sta_child = 5;
						#else
						bdtest_sta = TPC_CHRG_TST;//������һ������
						//tx_dp_dat(TBD_CHRG_TST, 1, 0);
						bdtest_sta_child = 0;
						#endif
						break;
					}
					
					#if BDTEST_LOGPRINT
					bdtest_sta_child = 5;
					#else
					tx_com_dat(TPC_CD_FRT, 100 , TPC_ERR_NOERROR, 0);//�ϱ�����
					#endif
					log_printf("\r\nBHW:%d\r\n",navigat->distance);
					tx_dp_dat(TBD_CD_FRT, 0, 0);//����ͣת
					navigat->distance = 0;
					bd_timer = 0;
				#if BDTEST_LOGPRINT
					break;
				case 5:
					break;
				#endif
				default:
					bdtest_sta = TPC_CHRG_TST;
					//tx_dp_dat(TBD_CHRG_TST, 1, 0);
					bdtest_sta_child = 0;
					tx_dp_dat(TBD_CHRG_TST, 3, 0);//��24V��DC�����ӿ�
					MOTOR_POWER_OFF_NPRI();
					bdtest_fantest = 0;
					break;
			}
			break;
		case TPC_CHRG_TST://������
			switch(bdtest_sta_child&0x0f)
			{
				case 0:
					tx_dp_dat(TBD_CHRG_TST, 3, 0);//��24V��DC�����ӿ�
					bdtest_sta_child = 1;
					break;
				case 1://�·������԰�,Ҫ����԰彫��ص���ؽŽ��뵽������
					bdtest_sta_child = 2;
					//�Ȳ�����ص��¶�,������Ƿ���ڻ����,���DC������������źŽ��Ƿ�����
					{
						uint16_t n_bat_temp = LiBat_GetBatTemper();//��ȡ����¶�
						uint8_t n_error_code = TPC_ERR_NOERROR;
						#if 0
						if(n_bat_temp == 1500)//��ز�����,�˳�����ģʽ
						{
							//bdtest_sta_child = 0;
							//bdtest_sta = TPC_STOP_ALL;
							n_error_code = TPC_ERR_BATNE;
							//break;
						}
						else if(/*n_bat_temp < 0 || */n_bat_temp > 400)//��ع���,�˳�����ģʽ
						{
							//bdtest_sta_child = 0;
							//bdtest_sta = TPC_STOP_ALL;
							n_error_code = TPC_ERR_BATOT;
							//break;
						}
						#else
						n_bat_temp = 200;
						#endif
						//���������DC�����ĵ�·�Ƿ�����,δ�����״̬��,��ӦΪ�͵�ƽ
						if(DOCK_DETECT())
						{
							//bdtest_sta_child = 0;
							//bdtest_sta = TPC_STOP_ALL;
							n_error_code = TPC_ERR_DOCK;
							//break;
						}
						if(EXTERAL_AC_DETECT())
						{
							n_error_code = TPC_ERR_DCJACK;
							//break;
						}
						
						//log_printf("\r\nbat:%d\r\n",n_error_code);
						//delay_ms(100);
						tx_com_dat(TPC_CHRG_TST, 10 , n_error_code, 0);
						if(n_error_code != TPC_ERR_NOERROR)
						{
							bdtest_sta_child = 0;
							bdtest_sta = TPC_GYRO_TST;
							//bdtest_sta_child = 4;
							break;
						}
					}
					tmd_rxbd_dat.unread_flag = 1;
					tx_dp_dat(TBD_CHRG_TST, 1, 0);//��24V��DC�����ӿ�
					bd_timer = 0;
					break;
				case 2://�·������԰�,Ҫ����԰彫��ص���ؽŽ��뵽������
					if(bd_timer ++ < 20 && !tmd_rxbd_dat.unread_flag)break;//��ʱδ�յ�����
					bd_timer = 0;
					if(!tmd_rxbd_dat.unread_flag)//��ʱδ�յ�����,����԰�(�Կض˿�)ͨ��ʧ��
					{
						tx_com_dat(TPC_DPPORT_TST, 0, TPC_ERR_COMM, 0);//����ͨ�Ŵ���
						tx_dp_dat(TBD_CHRG_TST, 1, 0);//��24V��DC�����ӿ�
						bdtest_sta_child = (bdtest_sta_child & 0xf0) + 2;
						bdtest_sta_child += 0x10;
						if(bdtest_sta_child > 0x50)
							bdtest_sta_child = 0x0f;
						//tmd_rxbd_dat.unread_flag = 0;
						//bdtest_sta_child = 4;
						break;
					}
					else
					{
//						uint8_t n_error_code = TPC_ERR_NOERROR;

						delay_ms(200);
						tmd_rxbd_dat.unread_flag = 0;
						/*
						if(DOCK_DETECT())//�����źż�����
							n_error_code = TPC_ERR_DOCK;
						if(EXTERAL_AC_DETECT() == 0)
							n_error_code = TPC_ERR_DCJACK;
						*/
						//log_printf("\r\n 1:%d\r\n",n_error_code);
						//delay_ms(100);
						//tx_com_dat(TBD_CHRG_TST, 20 , n_error_code, 0);
					}
					bdtest_sta_child = 3;
					tx_dp_dat(TBD_CHRG_TST, 2, 0);//��24V�򵽳������Ƭ�ӿ�
					bd_timer = 0;
					break;
				case 3://���DC�����źż���Ƿ�����
					if(bd_timer ++ < 20 && !tmd_rxbd_dat.unread_flag)break;//��ʱδ�յ�����
					bd_timer = 0;
					if(!tmd_rxbd_dat.unread_flag)//��ʱδ�յ�����,����԰�(�Կض˿�)ͨ��ʧ��
					{
						tx_com_dat(TPC_DPPORT_TST, 0, TPC_ERR_COMM, 0);//����ͨ�Ŵ���
						tx_dp_dat(TBD_CHRG_TST, 2, 0);//��24V��DC�����ӿ�
						bdtest_sta_child = (bdtest_sta_child & 0xf0) + 3;
						bdtest_sta_child += 0x10;
						if(bdtest_sta_child > 0x50)
							bdtest_sta_child = 0x0f;
						//tmd_rxbd_dat.unread_flag = 0;
						//bdtest_sta_child = 4;
						break;
					}
					else
					{
					//	uint8_t n_error_code = TPC_ERR_NOERROR;
						
						delay_ms(200);
						tmd_rxbd_dat.unread_flag = 0;
						/*
						if(DOCK_DETECT() == 0)//�����źż�����
							n_error_code = TPC_ERR_DOCK;
						if(EXTERAL_AC_DETECT())
							n_error_code = TPC_ERR_DCJACK;
						*/
						//log_printf("\r\n 2:%d\r\n",n_error_code);
						delay_ms(100);
						//tx_com_dat(TBD_CHRG_TST, 40 , n_error_code, 0);
						LiBat_HalInit();
						bdtest_chgfailed_counter = 0xff;
						bdtest_chgsucceed_counter = 0xff;
						bdtest_chgstart_counter = 0;
						bd_batcharge_timer = 0;
						//delay_ms(500);
					}
					
					bdtest_sta_child = 4;
					bd_timer = 0;
					break;
				case 4:
					{
						uint8_t n_charge_res;
				  	 	n_charge_res = LiBat_CurrentPid(LIBAT_CHARGECURRENT_SET,2);

				  	 	if(n_charge_res == LB_ERROR_NONE)
				  	 	{
				  	 		bd_batcharge_timer ++;
				  	 		if(bd_batcharge_timer > 600)//�������,ֹͣ���
				  	 		{
				  	 			bd_batcharge_timer = 0;
				  	 			bdtest_sta_child = 0;//0
								tx_dp_dat(TBD_CHRG_TST, 0, 0);//��24V��DC�����ӿ�
								LiBat_ExitChargeMode();
								bdtest_sta = TPC_GYRO_TST;
								tx_com_dat(TPC_CHRG_TST, 100 , TPC_ERR_NOERROR, 0);
								//log_printf("\r\nLB_CHS_NORMAL\r\n");
								bdtest_chgfailed_counter = 0xff;
								bdtest_chgsucceed_counter = 0xff;
								bdtest_chgstart_counter = 0;
				  	 		}
				  	 		else
				  	 		{
				  	 			if(bdtest_chgsucceed_counter == 0xff)
				  	 			{
				  	 				if(bdtest_chgcurr > (LIBAT_CHARGECURRENT_SET - 100) && 
				  	 				bdtest_chgcurr < (LIBAT_CHARGECURRENT_SET + 100))//����������Ԥ��ֵ��Χ
				  	 					bdtest_chgsucceed_counter = bdtest_chgfailed_counter = 0;
				  	 				else if(LiBat_GetBatVolt() > 16000 && (bdtest_chgcurr > (50) && 
				  	 					bdtest_chgcurr < (LIBAT_CHARGECURRENT_SET + 100)))
				  	 					bdtest_chgsucceed_counter = bdtest_chgfailed_counter = 0;
				  	 				else if(bdtest_chgcurr < (LIBAT_CHARGECURRENT_SET - 100))
				  	 				{
				  	 					if(bdtest_chgstart_counter ++ > BDTEST_CHG_START_MAX)
				  	 					{//�ܾö�û�дﵽԤ������,���ʧ��
											tx_com_dat(TPC_CHRG_TST, 100 , 
												bdtest_chgfailed_counter > BDTEST_CHG_FAIL_MAX ? TPC_ERR_BATCH:TPC_ERR_NOERROR, 0);
							  	 			bd_batcharge_timer = 0;
											tx_dp_dat(TBD_CHRG_TST, 0, 0);//��24V��DC�����ӿ�
											LiBat_ExitChargeMode();
											bdtest_sta = TPC_GYRO_TST;
											bdtest_sta_child = 0;
											MOTOR_POWER_OFF_NPRI();
											bdtest_fantest = 0;
											bdtest_chgfailed_counter = 0xff;
											bdtest_chgsucceed_counter = 0xff;
											break;
				  	 					}
				  	 				}
				  	 			}

								if(bdtest_chgsucceed_counter != 0xff && bdtest_chgfailed_counter != 0xff)
								{
					  	 			if(bdtest_chgcurr < (LIBAT_CHARGECURRENT_SET - 100) || 
					  	 				bdtest_chgcurr > (LIBAT_CHARGECURRENT_SET + 100))//����������
					  	 			{
					  	 				bdtest_chgfailed_counter =  bdtest_chgfailed_counter > BDTEST_CHG_FAIL_MAX? (BDTEST_CHG_FAIL_MAX + 1):(bdtest_chgfailed_counter + 1);
					  	 				bdtest_chgsucceed_counter = 0;
					  	 			}
					  	 			else
					  	 			{
					  	 				bdtest_chgsucceed_counter =  bdtest_chgsucceed_counter > BDTEST_CHG_SUCC_MIN ? (BDTEST_CHG_SUCC_MIN + 1):(bdtest_chgsucceed_counter + 1);
					  	 			}
					  	 			
					  	 			if(bdtest_chgsucceed_counter > BDTEST_CHG_SUCC_MIN
					  	 				|| bdtest_chgfailed_counter > BDTEST_CHG_FAIL_MAX)
					  	 			{
						  	 			bd_batcharge_timer = 0;
										tx_com_dat(TPC_CHRG_TST, 100 , 
											bdtest_chgfailed_counter > BDTEST_CHG_FAIL_MAX ? TPC_ERR_BATCH:TPC_ERR_NOERROR, 0);
										tx_dp_dat(TBD_CHRG_TST, 0, 0);//��24V��DC�����ӿ�
										LiBat_ExitChargeMode();
										bdtest_sta = TPC_GYRO_TST;
										bdtest_sta_child = 0;
										MOTOR_POWER_OFF_NPRI();
										bdtest_fantest = 0;
										bdtest_chgfailed_counter = 0xff;
										bdtest_chgsucceed_counter = 0xff;
										break;
					  	 			}
				  	 			}
				  	 		}
				  	 	}
				  	 	else
				  	 	{
							tx_com_dat(TPC_CHRG_TST, 40 , TPC_ERR_BATCH, 0);
							#if 1
							LiBat_ExitChargeMode();
							tx_dp_dat(TBD_CHRG_TST, 0, 0);//��24V��DC�����ӿ�
							bdtest_sta_child = 0;//0
							//log_printf("\r\nLB_CHS_ERROR:%d\r\n",n_charge_res);
							bdtest_sta = TPC_GYRO_TST;
							#endif
							//log_printf("\r\n ch:%d\r\n",n_charge_res);
							//delay_ms(100);
				  	 	}
			  	 	}
					break;
				#if 0
				case 5:
					//log_printf("^");
					//delay_ms(100);
					break;
				#endif
				default:
					bdtest_sta = TPC_GYRO_TST;
					bdtest_sta_child = 0;
					MOTOR_POWER_OFF_NPRI();
					bdtest_fantest = 0;
					break;
			}
			break;
		case TPC_GYRO_TST:
			switch(bdtest_sta_child)
			{
			case 0:
				#if 0
				{
					uint8_t n_gyro_res = 0;
					uint8_t n_err_res = TPC_ERR_DIGI_IN;
					SPI_CS(0);
					SPI1_ReadWriteByte(0xB5);
					n_gyro_res = SPI1_ReadWriteByte(0);
					SPI_CS(1);

					if(n_gyro_res == 0x70)//��������
					{
						n_err_res = TPC_ERR_NOERROR;
					}
					else if(n_gyro_res == 0)//�����ǰ��쳣��δ��
					{
						n_err_res = TPC_ERR_COMM;//����ͨ���쳣
					}
					else if(n_gyro_res == 0x7A)//������IC�쳣
					{
						n_err_res = TPC_ERR_DIGI_IN;//����ͨ���쳣
					}
					else
					{
						n_err_res = TPC_ERR_DIGI_OUT;//����ͨ���쳣
					}
					#if 0
					bdtest_sta_child = 1;
					log_printf("n_gyro_res:%02x\t err:%d\r\n",n_gyro_res,n_err_res);
					#else
					//bdtest_sta = TPC_STOP_ALL;
					tx_com_dat(TPC_GYRO_TST, 100 , n_err_res, 0);//�Ϸ��������
					tx_com_dat(TPC_STOP_ALL, 100 , TPC_ERR_NOERROR, 0);//�Ϸ��������
					tx_dp_dat(TBD_STOP_ALL, 0, 0);//��24V��DC�����ӿ�
					#endif
				}
				#else
				{
					uint8_t n_gyro_res = 0;
					uint8_t n_err_res = TPC_ERR_DIGI_IN;
					SPI_CS(0);
					SPI1_ReadWriteByte(0xB5);
					n_gyro_res = SPI1_ReadWriteByte(0);
					SPI_CS(1);

					if(n_gyro_res == 0x70)//��������
					{
						n_err_res = TPC_ERR_NOERROR;
					}
					else if(n_gyro_res == 0)//�����ǰ��쳣��δ��
					{
						n_err_res = TPC_ERR_COMM;//����ͨ���쳣
					}
					else if(n_gyro_res == 0x7A)//������IC�쳣
					{
						n_err_res = TPC_ERR_DIGI_IN;//����ͨ���쳣
					}
					else
					{
						n_err_res = TPC_ERR_DIGI_OUT;//����ͨ���쳣
					}
					//bdtest_sta = TPC_STOP_ALL;
					if(n_err_res == TPC_ERR_NOERROR || (tm_ir_bot_cc ++ > 4))//�������ǳɹ����߲��ɹ������Դ�������4
					{
						tm_ir_bot_cc = 0;
					#if BDTEST_LOGPRINT
						bdtest_sta_child = 1;
						log_printf("n_gyro_res end:%02x\t err:%d\r\n",n_gyro_res,n_err_res);
					#else
						tx_com_dat(TPC_GYRO_TST, 100 , n_err_res, 0);//�Ϸ��������
						//tx_com_dat(TPC_STOP_ALL, 100 , TPC_ERR_NOERROR, 0);//�Ϸ��������
						bdtest_sta = TPC_STOP_ALL;
						//delay_ms(100);
						//tx_com_dat(TPC_STOP_ALL, 100 , TPC_ERR_NOERROR, 0);//�Ϸ��������
						//tx_dp_dat(TBD_STOP_ALL, 0, 0);//��24V��DC�����ӿ�
						//TargetSysReset();
					#endif
					}
					#if BDTEST_LOGPRINT
					else
					{
						//bdtest_sta_child = 1;
						log_printf("n_gyro_res:%02x\t err:%d\r\n",n_gyro_res,n_err_res);
					}
					#endif
					//else
					//	bdtest_sta_child = 0;
				}
				#endif
			break;
			case 1:
				break;
			default:
				break;
			}
			break;
		//case TPC_IDLE:
		//	break;
		default:
			bdtest_sta = TPC_STOP_ALL;
			break;
	}
//	bdtest_sta_old = bdtest_sta;
}

//���԰��ͨ�ų���
void proc_bd_test_mode(uint8_t *buf,uint8_t len)
{
	TEST_BD_PROC *proc_dat;

	proc_dat = (TEST_BD_PROC *)buf;

	//data_print(buf,len);
	if(
#if TBD_STOP_ALL > 0
	proc_dat->code >= TBD_STOP_ALL &&
#endif
		proc_dat->code <= TBD_SWSENSOR)
		tmd_rxbd_dat.unread_flag = 1;//����δ����ǩ
	switch(proc_dat->code)
	{
		case TBD_STOP_ALL://�˳�����ģʽ,���԰彫������ĵ�Դ���عر�,��λ����������⵽��������ʱ,����Ϊ������˳���ر�,�������,��������������ϱ�����Ϊ��Դ����������
			tx_msg(buf,sizeof(TEST_PROC));
			TargetSysReset();
			break;
		case TBD_START_ALL://�������ģʽ
			tx_msg(buf,sizeof(TEST_PROC));
			break;
		case TBD_MTOC_WHL_L:
			tmd_rxbd_dat.mtoc_whl_l = proc_dat->t_res_output;
			//log_printf("Get Data TBD_MTOC_WHL_L\r\n");
			break;
		case TBD_MTOC_WHL_R:
			tmd_rxbd_dat.mtoc_whl_r = proc_dat->t_res_output;
			break;
		case TBD_MTOC_SMT_L:
			tmd_rxbd_dat.mtoc_smt_l = proc_dat->t_res_output;
			break;
		case TBD_MTOC_SMT_R:
			tmd_rxbd_dat.mtoc_smt_r = proc_dat->t_res_output;
			break;
		case TBD_MTOC_MT_MID:
			tmd_rxbd_dat.mtoc_mt_mid = proc_dat->t_res_output;
			break;
		case TBD_SMT_L:
			tmd_rxbd_dat.cd_smt_l = proc_dat->t_res_input;
			//log_printf("\r\n cd_smt_l:%d-%d\r\n",tmd_rxbd_dat.cd_smt_l,proc_dat->t_res_input);
			break;
		case TBD_SMT_R:
			tmd_rxbd_dat.cd_smt_r = proc_dat->t_res_input;
			break;
		case TBD_MT_MID:
			tmd_rxbd_dat.cd_mt_mid = proc_dat->t_res_input;
			break;
		case TBD_BUM_SW:
			tmd_rxbd_dat.bum_sw = proc_dat->t_res_output;
			tmd_rxbd_dat.bum_sw_led = proc_dat->t_res_input;
			break;
		case TBD_SWSENSOR:
			tmd_rxbd_dat.sensors_sw = proc_dat->t_res_output;
			tmd_rxbd_dat.sensors_sw_led = proc_dat->t_res_input;
			break;
		default:
			break;
	}
}

//===============================================================================================================
//�ϻ����Գ����

#define BURNIN_LOG_EN	0
#define BURNIN_EX_TIME	600//�����رߵ��л�����,��λΪ��

static uint16_t burnin_work_near_timer;//�����ӱߵĹ���ʱ�������
static uint16_t burnin_work_timer;//����ʱ�������,��λΪ��,ǰ20����Ϊ���ر�,��20����Ϊ���ر�,�ܹ�����40���Ӻ����̽���,���ص�idleģʽ
static uint32_t bummech_ct,bumir_ct;//��е��ײ(���Ӵ�ʽ��ײ)������ ������ײ������
static uint32_t bummech_left_ct,bumir_left_ct;//���ر߼�����
static uint32_t bummech_right_ct,bumir_right_ct;//���ر߼�����

#if 1
//����N����
void burnin_back_off(int hw)
{
	//float dt;
//	int c;
	
	int i,tmr;
//	int dist=0
#if CALE_BY_BKHW
	int distance=motor.c_left_hw - hw * WHELE_HW;
#else
	int distance=navigat->distance - hw;	
#endif

	CHECK_NAVI_STA();
	motor_run(GO_STOP,0,0,0);
	delay_ms(5);
	TIM5->CNT = 0;
	//motor.c_left_hw = 0;
	tmr = hw > 50 ?2000:500;

	motor_run(GO_BACK, BACK_OFF_PWM, 0, 0);
//	navigat->wheel_dir =-1;
	//navigat->is_walk = 1;
	
	i=0;
	
//	log_printf("befor bk,left(%3.1f,%3.1f)right(%3.1f,%3.1f)org(%d,%d)\r\n",navigat->x_org_f,navigat->y_org_f,navigat->x_org_r,,navigat->y_org_r,navigat->x_org,navigat->y_org);
	while(1)
	{
		
		if(TIM5->CNT >=10000)				
		{			
			TIM5->CNT = 0;
		//	www_idleintel_com();	
			//log_printf("k");
			if(WALK_DIST()< distance || i++ >= tmr)
			{
				motor_run(GO_STOP,0,0,0);
				break;
			}		
		}	
	}
	//navigat->is_walk = 0;
	motor_run(GO_STOP,0,0,0);
	www_idleintel_com();	
	//calc_gyro();
	burnin_coordinate_calcu(0);					//�����µ�����
	//log_printf("c2\r\n");
	coord_org2map(navigat->x_org,navigat->y_org,&navigat->tx,&navigat->ty);	//����ϵת��		
	//log_printf("o&");
	//gyro_whlmap();
	//log_printf("\r\n");
	//while(1);
//	log_printf("last bk,left(%3.1f,%3.1f)right(%3.1f,%3.1f)org(%d,%d)\r\n",navigat->x_org_f,navigat->y_org_f,navigat->x_org_r,,navigat->y_org_r,navigat->x_org,navigat->y_org);
	//log_printf("*(%d,%d,%f,0)-[%d,%d,%3.1f]\r\n\r\n",navigat->tx,navigat->ty,navigat->angle,navigat->x_org,navigat->y_org,sys->angle);
	//log_printf("hw=%d,x:%d,y%d,angle:%f,tx:%d,ty:%d\r\n",hw,navigat->x_org,navigat->y_org,navigat->angle,navigat->tx,navigat->ty);
	save_line_xy(navigat->x1_org,navigat->y1_org);
}


//�����Ƿ�360��תȦȦ
uint8_t burnin_near_round_360(float *m_angle,int16_t c_m_angle)
{
	int16_t i;
	uint8_t quadrant_1 = 0,quadrant2=0,quadrant3=0,quadrant4=0;
	uint8_t c_quadrant=0;
	//log_printf("[near_round_360]\r\n");
	for(i=0;i<MAX_C_M_ANGLE;i++)
	{

		if( m_angle[i] == 0)
			continue;
		if(m_angle[i] >=0 && m_angle[i] <=90)
			quadrant_1++;
		if(m_angle[i] >90 && m_angle[i] <=180)
			quadrant2++;
		if(m_angle[i] >180 && m_angle[i] <=270)
			quadrant3++;
		if((m_angle[i] >270 && m_angle[i] <=360) || m_angle[i] < 0)
			quadrant4++;
		//log_printf("quadrant=%3.1f,%d,%d,%d,%d\r\n",m_angle[i],quadrant_1,quadrant2,quadrant3,quadrant4);
		if(c_quadrant >=4 && i > 3)
		{
 			if((get_quadrant(m_angle[0]) == get_quadrant(m_angle[i]) && get_quadrant(m_angle[0]) == get_quadrant(m_angle[i-1])) ||
 			   (get_quadrant(m_angle[1]) == get_quadrant(m_angle[i]) && get_quadrant(m_angle[0]) == get_quadrant(m_angle[i-1]))  ||
 			   (get_quadrant(m_angle[2]) == get_quadrant(m_angle[i]) && get_quadrant(m_angle[0]) == get_quadrant(m_angle[i-1]))  ||
 			    (get_quadrant(m_angle[4]) == get_quadrant(m_angle[i]) && get_quadrant(m_angle[0]) == get_quadrant(m_angle[i-1]))||
 			     (get_quadrant(m_angle[5]) == get_quadrant(m_angle[i]) && get_quadrant(m_angle[0]) == get_quadrant(m_angle[i-1])))
			{
				//log_printf("found round...\r\n");
				for(c_m_angle = 0;c_m_angle<MAX_C_M_ANGLE;c_m_angle++)
					m_angle[c_m_angle] = 0;
				c_m_angle = 0;
				return 1;
			}
		}else
		{
			c_quadrant =count_quadrant(quadrant_1,quadrant2,quadrant3,quadrant4);	//ȡ������
			
		}
		
	}
	//log_printf("quadrant=%d,%d,%d,%d\r\n",quadrant_1,quadrant2,quadrant3,quadrant4);
	/**/
	if(c_quadrant >=4 && (quadrant_1 >=3 || quadrant2 >=3 || quadrant3 >=3 || quadrant4>=3))
	{
		//log_printf("found round2...\r\n");
		for(c_m_angle = 0;c_m_angle<MAX_C_M_ANGLE;c_m_angle++)
				m_angle[c_m_angle] = 0;
		c_m_angle = 0;	
		return 1;
	}
	
	return 0;
}

//????????
extern GRYO_T *gyro;
extern gyro_t	*mgyro;
void burnin_gyro_mapwhl(void)
{

	if(gyro->isOK ==0)
	{
		*(mgyro->angle)  = 0;
		//g_printf("Warning!!No copy allowed.Copyright idleintel\r\n");
		return ;
	}
//	disable_irq();	

	*(mgyro->x_org_f)  = *(mgyro->x_org) - *(mgyro->radius) * gyro->sinA ; // WHELE_HW;
	*(mgyro->y_org_f)  =  *(mgyro->y_org) + *(mgyro->radius) * gyro->cosA ;// WHELE_HW;	

	*(mgyro->x_org_r)  = *(mgyro->x_org) + *(mgyro->radius) * gyro->sinA ; // WHELE_HW;
	*(mgyro->y_org_r)  =  *(mgyro->y_org) - *(mgyro->radius) * gyro->cosA ;// WHELE_HW;
	
	*(mgyro->x_org_t)  = (*(mgyro->x_org) + *(mgyro->radius) * gyro->cosA)*ORG_TO_FRONT ; // WHELE_HW;
	*(mgyro->y_org_t)  =  (*(mgyro->y_org) + *(mgyro->radius) * gyro->sinA)*ORG_TO_FRONT ;// WHELE_HW

	//*(mgyro->x_org_t)  = (navigat->k_x_org + *(mgyro->radius) * gyro->cosA)*ORG_TO_FRONT ; // WHELE_HW;
	//*(mgyro->y_org_t)  =  (navigat->k_y_org + *(mgyro->radius) * gyro->sinA)*ORG_TO_FRONT ;// WHELE_HW
//	enable_irq();

}

void burnin_robot_turn_deg(uint8_t dir,int pwm,float agle)
{
//	int pwm=600;
	//int dg_turn=deg - 2.5;
//	int c=0;
	float dis_agle,b_agle;
	float out_deg;
	int midl=0;
	uint32_t t;
	www_idleintel_com();
	//calc_gyro();
	printf_power_sta();
	//log_printf("turn_charge,dir=%s,agle=%f\r\n",get_motor_sta(dir),agle);
	motor.c_left_hw = motor.c_right_hw = 0;
	//log_printf("1\r\n");
	CHECK_NAVI_STA();

	b_agle = sys->angle;
	//log_printf("2\r\n");
	//burnin_coordinate_calcu();
	//gyro_whlmap();
	navigat->is_walk = 0;
	//log_printf("3\r\n");
	turn_round_pid(0);
	//log_printf("4\r\n");
	motor_run(dir,pwm,0,0);
	sys->right_pwm = sys->left_pwm = pwm;
	TIM5->CNT = 0;
	//���м���������ת����
	if(MIDLE_HAVE_IRDA2() || *(navigat->near.pid->adc2) >= MAX_IR_SIDE2 )
		midl = 1;

	out_deg = NO_ANGLE_V;
	t=0;
	burnin_coordinate_calcu(0);
	while(1)
	{
		get_sensers(&sys->gSta);
		CHECK_NAVI_STA();
		//laser_calc_move(navigat,MAX_LASR_DIST);
#if LASER_SIDE
		laser_scan_dist(navigat,LASER_CALC_GO_FW);
#endif
		if(TIM5->CNT >=5000)
		{
			
			TIM5->CNT = 0;
			turn_round_pid(TURN_SPEED);
			www_idleintel_com();
			//calc_gyro();
			
			dis_agle = b_agle -sys->angle;
			if(dis_agle > 180)
				dis_agle = 360 - dis_agle;
			if(dis_agle <-180)
				dis_agle += 360;
			if(dis_agle >=agle || dis_agle <=-agle)
				break;
			//�м�û�к��⣬���˳�
			if(out_deg == NO_ANGLE_V)
			{
				if(agle == WALL_LOST_DEG && midl && (dis_agle >=20 || dis_agle <=-20))
				{
					if(sys->g_sta[2] < 300 && sys->g_sta[3] < 150 && sys->g_sta[4] < 280 &&
						*(navigat->near.pid->adc2) < MAX_IR_SIDE2)
					{
						
						//out_deg = sys->angle;
						
						if(*(navigat->near.pid->adc2) < 1700)
						{
							//log_printf("found V_angle=%3.1f,%d\r\n",sys->angle,*(navigat->near.pid->adc2));
							break;
						}
						//log_printf("\r\n");
					}
				}
			}else
			{
				if(disfloat(sys->angle,out_deg) > 5)	//��ת10��
				{
					//log_printf("VOK,v_agle=%3.1f\r\n",sys->angle);
					break;
				}
			}
			if(t++>=3000)
			{
				//log_printf("timeout\r\n");
				break;
			}

			
		}
		
		//log_printf("%3.1f,%3.1f\r\n",sys->angle,dis_agle);
	}	
	motor_run(GO_STOP,0,0,0);
	DELAY_MS(100);
	//delay_ms_sensers(200);
#if LASER_SIDE
		calc_scan_laser(navigat,1);
#endif	
//	www_idleintel_com();
	//gyro_whlmap();
	//gyro_mapwhl();
	burnin_gyro_mapwhl();
	//navigat->is_walk =1;
	//log_printf("..OK,c=%d,%d,agl=%3.1f\r\n",motor.c_left_hw,motor.c_right_hw,sys->angle);
	
}
#define BURN_MAX_KXXYY		80
static int16_t b_kxx[BURN_MAX_KXXYY];
static int16_t b_kyy[BURN_MAX_KXXYY];
char burnin_check_round_bum(uint8_t type)
{
	static uint16_t idx=0;
	short i;
	if(type ==0)
	{
		idx = 0;
		return 0;
	}
	if(idx >=BURN_MAX_KXXYY)
		idx = 0;
	b_kxx[idx] = X_NOW;
	b_kyy[idx++] = Y_NOW;
	
	for(i=1;i<BURN_MAX_KXXYY;i++)
	{
		if(dis_xy(b_kxx[0],b_kxx[i]) > 5)
			return 0;
		if(dis_xy(b_kyy[0],b_kyy[i]) > 5)
			return 0;
	}
	return 1;
	
}
#if CALE_BY_FRON
int burnin_coordinate_calcu(uint8_t type)
{
	
	static int cc=0;
//	static int16_t cc=0;
	int32_t x_org1;//,x_org2;//,
	int32_t y_org1;//,y_org2;//,;
#if WALK_PRINTF_CALC	
	static  short xx=0,yy=0;
#endif
	
	int32_t x_org2;	//����
	int32_t y_org2;	//����
	int32_t x_org3,y_org3;	//ǰ��
	float x_org_t,y_org_t;
	

	float x_org_f,y_org_f;
	float x_org_r,y_org_r;
	cc++;
	if(cc <2 && type==1)
		return 1;
	cc = 0;
//	
	disable_irq();	
	x_org_f = navigat->x_org_f ;
	y_org_f = navigat->y_org_f;
	
	x_org_r = navigat->x_org_r;
	y_org_r = navigat->y_org_r;
	
 	x_org_t= navigat->x_org_t;
 	y_org_t= navigat->y_org_t;
 	enable_irq();
 	//����
 	x_org1	=	x_org_calc(x_org_f);
 	y_org1	= 	y_org_calc(y_org_f);
 	//ʹ��ǰ��
	x_org3 = x_org_calc_f(x_org_t);
	y_org3 = y_org_calc_f(y_org_t);

	//ʹ������
	x_org2	=	x_org_calc_r(x_org_r);
	y_org2	= 	y_org_calc_r(y_org_r);
		

	/**/
	if(motor.c_left_hw > 100)
	{
		if(disXY(x_org1,x_org2) > 30 || disXY(y_org1,y_org2) > 30)
		{
			navigat->x_org	= 	x_org1;
			navigat->y_org	=  	y_org1;
			//log_printf("[coor_calcu]err x=%d,%d,y=%d,%d,%d\r\n",x_org1,x_org2,y_org1,y_org2,motor.c_left_hw);
			navigat->x_org	= 	(x_org1+x_org2+x_org3) / 3;
			navigat->y_org	=  	(y_org1+y_org2+y_org3) / 3;	
			gyro_whlmap();
			return 0;
		}
		


		if(disXY(x_org1,x_org3) > 30 || disXY(y_org1,y_org3) > 30)
		{
			//log_printf("[coor_calcu]errX=%d,%d,Y=%d,%d,%d\r\n",x_org1,y_org1,x_org3,y_org3,motor.c_left_hw);
			navigat->x_org	= 	(x_org1+x_org2+x_org3) / 3;
			navigat->y_org	=  	(y_org1+y_org2+y_org3) / 3;	
			gyro_whlmap();
			return 0;
		}
	}
	/**/
	//if(cc++>=100)
	{
				//log_printf("%d %d|%d %d|%d %d\r\n",x_org1,y_org1,x_org2,y_org2,x_org3,y_org3);
				//	cc = 0;
	}

	

	navigat->x_org	= 	x_org1;
	navigat->y_org	=  	y_org1;		
	gyro_whlmap();	

/*
	log_printf("*(%d,%d,%3.1f)[%d,%d|%d,%d|%d,%d]*\r\n",navigat->tx,navigat->ty,sys->angle,
					navigat->x_org,navigat->y_org,x_org1,y_org1,x_org2,y_org2);
	
	log_printf("*%3.3f[%d,%d|%d,%d|%d,%d](%d,%3.1f,%3.1f)(%d,%3.1f,%3.1f)*\r\n",sys->angle,
						navigat->x_org,navigat->y_org,x_org1,y_org1,x_org2,y_org2,
						motor.c_left_hw,navigat->x_org_f,navigat->y_org_f,
						motor.c_right_hw,navigat->x_org_r,navigat->y_org_r);

					
	log_printf("*(%d,%d,%3.3f)[%d,%d|%d,%d](%d,%d,%d)*\r\n",navigat->tx,navigat->ty,sys->angle,
						navigat->x_org,navigat->y_org,x_org2,y_org2,
						motor.c_right_hw,motor.c_right_hw,navigat->distance);
	*/	
#if WALK_PRINTF_CALC
	if(xx != X_NOW || yy != Y_NOW)
	{
		xx 	= X_NOW;
		yy	= Y_NOW;
		//log_printf("*(%d,%d,%3.1f,0)[%d,%d,%d,%d,%d]*\r\n",navigat->tx,navigat->ty,sys->angle,navigat->x_org,navigat->y_org,
		//												x_org2,y_org2,navigat->distance);
	}

#endif



	return 1;



}
#else
int burnin_coordinate_calcu(uint8_t type)
{
	

	int32_t x_org1;//,x_org2;//,
	int32_t y_org1;//,y_org2;//,;
#if WALK_PRINTF_CALC	
	static  short xx=0,yy=0;
#endif
	
#if CALE_ADJ_RIGHT
	int32_t x_org2;//,
	int32_t y_org2;//,;
#endif	
#if CALE_BY_FRONT
//	static int distanc=0;		
	int32_t x_org_front,y_org_front;
	float x_org_t,y_org_t;
	//int ret=0; asd
	
#endif

	float x_org_f,y_org_f;
#if CALE_ADJ_RIGHT	
	float x_org_r,y_org_r;
#endif	
//	
	disable_irq();	
	x_org_f = navigat->x_org_f ;
	y_org_f = navigat->y_org_f;
#if CALE_ADJ_RIGHT	
	x_org_r = navigat->x_org_r;
	y_org_r = navigat->y_org_r;
#endif	
 #if CALE_BY_FRONT
 	x_org_t= navigat->x_org_t;
 	y_org_t= navigat->y_org_t;
 #endif	
 	enable_irq();
 	
 	x_org1	=	x_org_calc(x_org_f);
 	y_org1	= 	y_org_calc(y_org_f);
 	//ʹ��ǰ��
#if CALE_BY_FRONT
	x_org_front = x_org_calc_f(x_org_t);
	y_org_front = y_org_calc_f(y_org_t);

	navigat->k_x_org	= 	x_org_front;
	navigat->k_y_org	=  	y_org_front;	
	coord_org2map(navigat->k_x_org,navigat->k_y_org,&navigat->kx,&navigat->ky);
#endif	 
	//log_printf("%d %d %d %d %d %d\r\n",);
	//ʹ������У׼
#if CALE_ADJ_RIGHT
		if(navigat->adj_run == FALSE)
		{
			x_org2	=	x_org_calc_r(x_org_r);
			y_org2	= 	y_org_calc_r(y_org_r);
			
			//TEST MICONY
			//micony 2017-12-27��֪��Ϊ�Σ������ӱߵ�ʱ���������ô�
			//if(sys->nsta == NO_SIDE_NEAR)
			{
				if(disXY(x_org1,x_org2) > 160 || disXY(y_org1,y_org2) > 160)
				{
					navigat->x_org	= 	x_org1;
					navigat->y_org	=  	y_org1;
					//log_printf("[coor_calcu]err x=%d,%d,y=%d,%d\r\n",x_org1,x_org2,y_org1,y_org2);
					return 0;
				}
			}
		}
	

#if CALE_BY_FRONT		//ʹ��ǰ�ֲ���У׼
		//navigat->x_org	= 	(x_org1 + x_org_front)>> 1;
		//navigat->y_org	=  	(y_org1 + y_org_front)>> 1;
		if(disXY(x_org1,x_org_front) > 20 || disXY(y_org1,y_org_front) > 20)
		{
			//log_printf("[coor_calcu]errX=%d,%d,Y=%d,%d\r\n",x_org1,x_org2,x_org_front,y_org_front);
			return 0;	
		}
		gyro_fwhlmap();
#endif	

		//navigat->x_org	= 	(x_org1 +x_org2)>>1 ;// + (int32_t)((float)(x_org2 - x_org1)*0.6f);
		//navigat->y_org	=  	(y_org1 +y_org2)>>1 ;//+ (int32_t)((float)(y_org2 - y_org1)*0.6f);
		navigat->x_org	= 	x_org1;
		navigat->y_org	=  	y_org1;
	
#else
		navigat->x_org	= 	x_org1;
		navigat->y_org	=  	y_org1;		
		
#endif

/*
	log_printf("*(%d,%d,%3.1f)[%d,%d|%d,%d|%d,%d]*\r\n",navigat->tx,navigat->ty,sys->angle,
					navigat->x_org,navigat->y_org,x_org1,y_org1,x_org2,y_org2);
	
	log_printf("*%3.3f[%d,%d|%d,%d|%d,%d](%d,%3.1f,%3.1f)(%d,%3.1f,%3.1f)*\r\n",sys->angle,
						navigat->x_org,navigat->y_org,x_org1,y_org1,x_org2,y_org2,
						motor.c_left_hw,navigat->x_org_f,navigat->y_org_f,
						motor.c_right_hw,navigat->x_org_r,navigat->y_org_r);

					
	log_printf("*(%d,%d,%3.3f)[%d,%d|%d,%d](%d,%d,%d)*\r\n",navigat->tx,navigat->ty,sys->angle,
						navigat->x_org,navigat->y_org,x_org2,y_org2,
						motor.c_right_hw,motor.c_right_hw,navigat->distance);
	*/	
#if WALK_PRINTF_CALC
	if(xx != X_NOW || yy != Y_NOW)
	{
		xx 	= X_NOW;
		yy	= Y_NOW;
		//log_printf("*(%d,%d,%3.1f,0)[%d,%d,%d,%d,%d]*\r\n",navigat->tx,navigat->ty,sys->angle,navigat->x_org,navigat->y_org,
		//												x_org2,y_org2,navigat->distance);
	}

#endif



	return 1;



}


//�ж��Ƿ�ʱ(���趨��ʱ��) 
//ע�⣺msec�������Ϊ0ʱ�����
uint8_t burnin_mstimeout(uint32_t *timep,uint32_t msec)
{
	uint32_t time,diff;
	uint8_t result=0;
	time=msTmr;
	diff=time- *timep;
	if(msec==0)
	{
		*timep=time;
		result=1;
	}
	else if(diff>=msec)
	{
		//diff += (msec + (msec >> 1));
		result = (uint8_t)(diff/msec);
		*timep=time;
	}
	return result;
}

#endif

int burnin_near_wall_pid(float *agle,int c_lost)
{
	static int c_pid=0;
	NEAR_WALL *near;

	near = &navigat->near;
	if(c_pid ++ >=2)
	{
		c_pid = 0;
		//if(near->n_sta !=NO_SIDE_NEAR && near->pid != NULL)		//ǿ���ӱ�
		{
			return (near_wall_pid(1,near->pid,agle));
		}/*else
		{
			if(near_wall_pid(0,&l_near_pid,agle) ==0)
			{
				return (near_wall_pid(0,&r_near_pid,agle));
			}
			return 1;
		}*/
			
	}
	return 1;

}
#endif
/*****************************************************************************
 * ��������:
 * ��    ��:	type  :
 *						 GO_NEAR_TYPE_NO		0x00		//��ͨ�ӱ�
 *						 GO_NEAR_TYPE_NAVI		0x01		//�м�����ܵ������յ㣬���˳�ȥ����	
 *						 GO_NEAR_TYPE_ADJ	   0x02	
 *��������:	�ӱߵ������ҵط������������Ŀ�ĵأ����˳�
 *			
 *****************************************************************************/
 static uint32_t burninbeep_timer = 0;
char motor_go_burnin(uint8_t n_sta ,short tox,short toy,uint8_t type,uint8_t is_save)
{
//	int xx=0,yy=0,x1=0,y1=0;
	int calue=0;
//	int by = navigat->y_org;
//	uint8_t gSta;
	uint8_t sta;
//	int	lx=0,ly=0;
	short llx=0,lly=0;
//	short lx=X_NOW,ly=Y_NOW;
//	int bx = X_NOW;
	short by = Y_NOW;
//	short bx = X_NOW;
	short nx1,ny1;
	float lagle;		//��¼�����ӱ߽Ƕȣ����ڼ����Ƿ�ת�����
	uint16_t	gSta;
	uint32_t	t_begin;//��ʼ��ʱ��
	int ret_calc=1;
	uint16_t c_dock_data=0;
	int16_t c_lost=0;
	uint16_t c_round=0;		//תȦ�Ĵ���


	short x_begin_line=0,y_begin_line = 0;		//һ���߿�ʼ��X��Y������

	uint8_t sec1s_ct = 0;// 1��ļ�����
	///uint8_t time5ms_flag = 0;//5msʱ���־
	

	u8 irData_bak[6];
	
	NEAR_WALL *near = &navigat->near;
	float m_angle[MAX_C_M_ANGLE];			//���������20����ײ�ĽǶ�
	int16_t c_m_angle=0;
	int16_t c_near_wall = 0;

	for(c_m_angle = 0;c_m_angle<MAX_C_M_ANGLE;c_m_angle++)
		m_angle[c_m_angle] = 0;
	c_m_angle = 0;
	CHECK_NAVI_STA_RT(0);
	pd_gyro_int(GO_NEAR_PWM_FAST);
	navigat->out =navigat->angle;
	navigat->distance = 0;
	motor.c_left_hw = motor.c_right_hw = 0;
	cord_calc_store(0);
	gyro_whlmap();
	robot_whele_stop(0);
	motor_run(GO_FORWARD, GO_NEAR_PWM_FAST, 0, 0);
	if(n_sta & 0x80)
		init_near_wall_navi(n_sta & 0x7F);
	else
		init_near_wall_navi(NO_SIDE_NEAR);
	n_sta &=0x7F;
	navigat->is_walk =1;
#if BURNIN_LOG_EN
	log_printf("[motor_go_edgeways]sta=%d(%d),type=%d,is_save=%d,now=(%d,%d,)to=(%d,%d,)\r\n",navigat->near.n_sta,n_sta,type,is_save,X_NOW,Y_NOW,tox,toy);
#endif	
	//navigat->wheel_dir = 1;
	sta = sys->sState;
	lagle = sys->angle;
	mstimeout(&t_begin,0);
	MOTOR_CTRL(NORM_SIDE_PWM_L,NORM_SIDE_PWM_R,NORM_MID_PWM,sys->dust_pwm_value)
#if BURNIN_LOG_EN
	printf_power_sta();
#endif	
	burnin_mstimeout(&burninbeep_timer,0);
	while(1)
	{

		sec1s_ct = burnin_mstimeout(&burninbeep_timer,1000);
		proc_uart_task();
		get_sensers(&sys->gSta);
		if(sys->sState !=sta)
		{
#if BURNIN_LOG_EN
			log_printf("\r\nmode changed\r\n");
#endif	
			tx_com_burnin(NEAR_BY_IRDA, burnin_work_timer, bummech_left_ct, bumir_left_ct, bummech_right_ct, bumir_right_ct);
			return 0;
			//log_printf("\r\nerr:%d\r\n",sys->work_errcode);
			//sys->sState = SYS_RANDOM;
			//MOTOR_POWER_ON();
			//motor_run(GO_FORWARD, GO_NEAR_PWM_FAST, 0, 0);
			//MOTOR_CTRL(NORM_SIDE_PWM_L,NORM_SIDE_PWM_R,NORM_MID_PWM,sys->dust_pwm_value)
		}

		if(sec1s_ct)
		//if(mstimeout(&sys->t_loop,1000))
		//if(TIM5->CNT >=5000)
		{
			//TIM5->CNT = 0;
			//time5ms_flag = 1;

			//if(sec1s_ct ++ > 199)
			{
				//sec1s_ct = 0;
				if(near->n_sta == LEFT_SIDE_NEAR)
				{
					bummech_left_ct = bummech_ct;
					bumir_left_ct = bumir_ct;
				}
				else if(near->n_sta == RIGHT_SIDE_NEAR)
				{
					bummech_right_ct = bummech_ct;
					bumir_right_ct = bumir_ct;
				}
				//log_printf("--------------------------------------\r\n");
				burnin_work_timer += sec1s_ct;
				burnin_work_near_timer += sec1s_ct;
				if(sys->work_errcode != 0)
					log_printf("\r\nerr:%d\r\n",sys->work_errcode);
				tx_com_burnin(near->n_sta, burnin_work_timer, bummech_left_ct, bumir_left_ct, bummech_right_ct, bumir_right_ct);
				if(burnin_work_near_timer > BURNIN_EX_TIME)//������,�ݶ�5����
				{
					burnin_work_near_timer = 0;
					motor_run(GO_STOP,0,0,0);
					return RET_NEAR_TIMEOUT;
				}
			}
		}
		//if(sys->work_errcode != 0 && sys->c_left==0 && sys->c_right == 0)
		//{
		//}
			//return RET_NEAR_ERROR;
		CHECK_NAVI_STA_RT(0);
		//��ײ
		if(sys->gSta & (MASK_BUM_MIDL) || ret_calc==0 ||  c_lost >=MAX_NEAR_LOST || MIDLE_IRDA())
		{
//			gSta = sys->gSta;	//��¼��ײ��״̬
			
			motor_run(GO_STOP,0,0,0);
#if BURNIN_LOG_EN
			log_printf("BUM\r\n");
#endif			
			//delay_ms(200);
			if(sys->gSta & (MASK_BUM_MIDL))
			{
				if(sys->fall_sta)		//���䣬��
					burnin_back_off(BACK_HW*8);
				else
					burnin_back_off(BACK_HW);
				bummech_ct ++;//��е��ײ�����ļ���

			}
			if(MIDLE_IRDA())
				bumir_ct ++;//��ǰ��������ײ�ļ���
			//delay_ms_sensers(200);
			burnin_coordinate_calcu(0);														//�����ԭʼ������ϵ
			coord_org2map(navigat->x_org,navigat->y_org,&navigat->tx,&navigat->ty); //����ϵת��
			motor_run(GO_STOP,0,0,0);
#if BURNIN_LOG_EN
			log_printf("\r\n-----go_edgeways bum(%d,%d,%d,%f,%f),gsta=%d,irda=(%d,%d,%d,%d)angle=%3.1f,ret=%d,lost=%d,%d\r\n",navigat->tx,navigat->ty,motor.c_left_hw,navigat->x_org_f,navigat->y_org_f,
							sys->gSta,sys->g_sta[0],sys->g_sta[1],sys->g_sta[5],sys->g_sta[6],sys->angle,ret_calc,c_lost,c_m_angle);
#endif							

			if(ret_calc==0)
					burnin_gyro_mapwhl();
			if(c_m_angle >=MAX_C_M_ANGLE)
				c_m_angle = 0;
			m_angle[c_m_angle++] = sys->angle;		//���ֽǶ�

			if(burnin_near_round_360(m_angle,c_m_angle))
			{
#if BURNIN_LOG_EN
				log_printf("found round...go line...\r\n");
#endif				

				init_near_wall_navi(NO_SIDE_NEAR);
				for(c_m_angle = 0;c_m_angle<MAX_C_M_ANGLE;c_m_angle++)
					m_angle[c_m_angle] = 0;
				c_m_angle = 0;
				goto l_mgo_edeways_burnin;
			}

			
			
			if(!(ret_calc==0 || c_lost >=MAX_NEAR_LOST))
				c_round = 0;
			if(ret_calc==0)
					gyro_mapwhl();
			gSta = sys->gSta;
			//motor_run(GO_STOP,0,0,0);
			if(sys->work_mod & MWO_MOP)
				delay_ms(200);
			//www_idleintel_com();
			lagle = sys->angle;
			//init_near_wall_navi(n_sta);		//��ײ�󣬲ſ�ʼһֱ�ر�
		
			navigat->distance = 0;
			motor.c_left_hw = 0;
			
			if( type == GO_NEAR_TYPE_DOCK)
			{
				//�ҳ��׮
				read_ir_data_timeout(irData_bak,0);	
				
				if(irData_bak[IR_L_PIN_NUM] ||  irData_bak[IR_M_PIN_NUM] || irData_bak[IR_R_PIN_NUM] || ir_dock_insight(1))
				{
					//int16_t i;
					//for(i=0;i<3;i++)
					{
						//delay_ms(80);
						//read_ir_data(irData_bak,0);	

#if BURNIN_LOG_EN						
						log_printf("[motor_go_edgeways]bum  dock(%d,%d,%d)%d\r\n",
						irData_bak[IR_L_PIN_NUM] ,irData_bak[IR_M_PIN_NUM],irData_bak[IR_R_PIN_NUM],c_dock_data);
#endif						
						//if(irData_bak[IR_L_PIN_NUM] ||  irData_bak[IR_M_PIN_NUM] || irData_bak[IR_R_PIN_NUM])
						if(1)
						{
							
							c_dock_data++;
#if BURNIN_LOG_EN
							log_printf("ir=%d,%d,%d,c=%d\r\n",irData_bak[IR_L_PIN_NUM] , irData_bak[IR_M_PIN_NUM] , irData_bak[IR_R_PIN_NUM],c_dock_data);
#endif							
							if(c_dock_data >=3)
							{
#if BURNIN_LOG_EN
								log_printf("found dock..\r\n");
#endif								
								goto l_go_out_for_dock_burnin;
							}
						}else
							c_dock_data=0;
					}

				}else
					c_dock_data = 0;
			}


			/**************************************************************************
				���tox��toy���������ã��򵽵��ˣ���ͣ������
			*****************************************************************************/
			if(  (((X_NOW == tox || tox ==0) && (Y_NOW == toy || toy==0)) && tox && toy) || 	//������
				(type == GO_NEAR_TYPE_ADJ &&  ((by > toy && Y_NOW < toy) || (by < toy && Y_NOW > toy)))) //adj��ȥ��Y�ᳬ����
			{
#if BURNIN_LOG_EN
				log_printf("xy ok(%d,%d,%3.1f)\r\n",X_NOW,Y_NOW,sys->angle);
#endif				
				motor_run(GO_STOP,0,0,0);
				//delay_ms_sensers(200);
				//burnin_coordinate_calcu();														//�����ԭʼ������ϵ
				//coord_org2map(navigat->x_org,navigat->y_org,&navigat->tx,&navigat->ty);	//����ϵת��
				return RET_NEAR_OK;				
			}	
			
	
			if(near->n_sta ==NO_SIDE_NEAR)
			{
				init_near_wall_navi(n_sta);
			}

			if( (ret_calc==0 || c_lost >= MAX_NEAR_LOST)&& *(navigat->near.pid->adc) < navigat->near.pid->min_adc )
			{
				c_round++;
#if BURNIN_LOG_EN
				log_printf("lost or calc error(%d,%d),cround=%d\r\n",ret_calc,c_lost,c_round);
#endif				
				/*
				if(c_round >=3)
				{
					log_printf("big round not near\r\n");
					init_near_wall_navi(NO_SIDE_NEAR);
					goto l_go_edeways;
				}
				*/
				if(ret_calc)
				{
					gyro_whlmap();
				}
				if(near->n_sta == RIGHT_SIDE_NEAR)
				{
					burnin_robot_turn_deg(GO_RIGTH,DEG_TURN_PWM,NEAR_LOST_DEG);
					goto l_mgo_edeways_burnin;
				}else if(near->n_sta == LEFT_SIDE_NEAR)
				{
					burnin_robot_turn_deg(GO_LEFT,DEG_TURN_PWM,NEAR_LOST_DEG);
					goto l_mgo_edeways_burnin;
				}
					
			}	

			if(near->n_sta == RIGHT_SIDE_NEAR)
			{
				if(burnin_check_round_bum(1))
					burnin_robot_turn_deg(GO_LEFT,DEG_TURN_PWM,12);
				else
				{
					if(BUM_LEFT(gSta) || MIDLE_HAVE_IRDA2() || RIGHT_IR_BUM2())
					{
						burnin_robot_turn_deg(GO_LEFT,DEG_TURN_PWM,WALL_LOST_DEG);
					}else if(sys->g_sta[6] > 900 && sys->g_sta[5] > 900)//��ǽ�ˣ�������ӱߣ���Ҫת̫��
					{
						burnin_robot_turn_deg(GO_LEFT,DEG_TURN_PWM,12);
					}else
						burnin_robot_turn_deg(GO_LEFT,DEG_TURN_PWM,25);
				}
				
			}else if(near->n_sta == LEFT_SIDE_NEAR)
			{
				if(burnin_check_round_bum(1))
					burnin_robot_turn_deg(GO_RIGTH,DEG_TURN_PWM,12);

				else
				{
					if(BUM_RIGHT(gSta) || MIDLE_HAVE_IRDA2() || LEFT_IR_BUM2())
					{
						burnin_robot_turn_deg(GO_RIGTH,DEG_TURN_PWM,WALL_LOST_DEG);
					}
					else if(sys->g_sta[1] > 900 && sys->g_sta[2] > 900)
					{
						burnin_robot_turn_deg(GO_RIGTH,DEG_TURN_PWM,12);
					}
					else 
					{
						burnin_robot_turn_deg(GO_RIGTH,DEG_TURN_PWM,25);
					}
				}

			}
			else
			{
#if BURNIN_LOG_EN
				log_printf("RET_NEAR_ERROR,nsta=%d\r\n",n_sta);
#endif				
				return RET_NEAR_ERROR;	
			}
l_mgo_edeways_burnin:		

			//תȦ�ˡ���
			c_lost=0;
			//burnin_coordinate_calcu(); 	
			motor_run(GO_STOP,0,0,0);
		//	log_printf("after bk(%d,%d,%d,%3.3f,%3.3f,%3.3f)\r\n==============\r\n",navigat->tx,navigat->ty,motor.c_left_hw,navigat->x_org_f,navigat->y_org_f,sys->angle);
			//if(ccc++ >=5)
			//	while(1);
			navigat->distance = 0;
			navigat->is_walk = 1;
			pd_gyro_int(GO_NEAR_PWM_FAST);
			navigat->out =sys->angle;
			cord_calc_store(0);
			gyro_whlmap();
			motor.c_left_hw = motor.c_right_hw = navigat->distance = 0;
			
			motor_run(GO_FORWARD, GO_NEAR_PWM_FAST, 0, 0);
			navigat->near.pid->c_lost = 0;
			
			navigat->near.pid->c_lost = 0;
			navigat->near.pid->c_lost_flag = 0;			//ʧȥǽ�ı�־     2019 02 15 add 
			sys->g_t_walk = 0; 
			ret_calc = 1;
			//��¼����λ��
			x_begin_line = X_NOW;
			y_begin_line = Y_NOW;
			c_near_wall = 0;
		//	motor_run(GO_RIGHT_RD,0,0,0);
		//	navigat->near_sta = LOST_WALL_RIGHT;
		}

		if(TIM5->CNT >=5000)
		//if(time5ms_flag)
		{
			TIM5->CNT = 0;
			//time5ms_flag = 0;
			navigat->out = format_agle(navigat->out,ANGLE_360);
			proc_line_pid(navigat->out);
			burnin_near_wall_pid(&navigat->out,5);
			//navigat_near_wall_pid(&navigat->out,5);
			if(near->n_sta ==NO_SIDE_NEAR)
			{
				if(RIGHT_ADC() >= cfg->lock_right_adc)
					init_near_wall_navi(RIGHT_SIDE_NEAR);
				else if(LEFT_ADC() >= cfg->lock_left_adc)
					init_near_wall_navi(LEFT_SIDE_NEAR);
			}
			
			if(robot_whele_stop(1))
					continue;			//ֱ�ӳ���������ײ��׼��

			

			if( type == GO_NEAR_TYPE_DOCK )
			{
				//�ҳ��׮
			//	c_dock = 0;
				read_ir_data_timeout(irData_bak,0); 
				if(irData_bak[IR_L_PIN_NUM] ||	irData_bak[IR_M_PIN_NUM] || irData_bak[IR_R_PIN_NUM] || ir_dock_insight(1))
				{
#if BURNIN_LOG_EN
					log_printf("[motor_go_edgeways]found dock(%d,%d,%d)%d\r\n",
						irData_bak[IR_L_PIN_NUM] ,irData_bak[IR_M_PIN_NUM],irData_bak[IR_R_PIN_NUM],c_dock_data);
#endif						
					//if(c_dock_data++ >=3)
					{
#if BURNIN_LOG_EN
						log_printf("[motor_go_edgeways]found dock\r\n");
#endif						
						motor_run(GO_STOP,0,0,0);
						//delay_ms_sensers(200);
						burnin_coordinate_calcu(0); 													//�����ԭʼ������ϵ
						coord_org2map(navigat->x_org,navigat->y_org,&navigat->tx,&navigat->ty); //����ϵת��
						goto	l_go_out_for_dock_burnin;
					}

				}else
					c_dock_data = 0;
			}

			
			if( *(navigat->near.pid->adc) > navigat->near.pid->min_adc) //�ӱ�
			  lagle = sys->angle;
			else		//�ӱ���ʧ��ת�ĽǶȳ���180�ȣ���ʧ���˳�
			{
				if(disfloat( lagle , sys->angle) > 180)
				{
#if BURNIN_LOG_EN
					log_printf("lost over(%d,%d,%3.1f,%3.1f)\r\n",X_NOW,Y_NOW,sys->angle,lagle);
#endif					
					motor_run(GO_STOP,0,0,0);
					//delay_ms_sensers(200);
					burnin_coordinate_calcu(0);														//�����ԭʼ������ϵ
					coord_org2map(navigat->x_org,navigat->y_org,&navigat->tx,&navigat->ty);	//����ϵת��
					return RET_NEAR_ERROR;	
				}
			}
#if CALE_BY_FRON				
				ret_calc = burnin_coordinate_calcu(1);														//�����ԭʼ������ϵ
#endif				
			if(calue++ >=40)
			{

				if(c_near_wall ++ >=30)
				{

					for(c_m_angle = 0;c_m_angle<MAX_C_M_ANGLE;c_m_angle++)
						m_angle[c_m_angle] = 0;
					c_m_angle = 0;
					c_near_wall = 0;
#if BURNIN_LOG_EN
					log_printf("log near\r\n");
#endif					
				}


			
				calue = 0;
#if !CALE_BY_FRON				
				ret_calc = burnin_coordinate_calcu(0);														//�����ԭʼ������ϵ
#endif				
				coord_org2map(navigat->x_org,navigat->y_org,&navigat->tx,&navigat->ty);	//����ϵת��
				//����0�ȣ�90�ȣ�80�ȵķ���,ˢ������
				/**/
				if(x_begin_line != X_NOW && y_begin_line != Y_NOW)
				{
					ajust_xy_by_near_line(x_begin_line,y_begin_line,X_NOW,Y_NOW,sys->angle,LINE_TYPE_LOST,n_sta);
					x_begin_line = X_NOW;
					y_begin_line = Y_NOW;	
				}

				
				if( *(navigat->near.pid->adc) > navigat->near.pid->min_adc) //�ӱ�
				{
					//c_near++;
					c_round = 0;
					
				}
				else if(near->n_sta !=NO_SIDE_NEAR)	//ǿ���ӱߣ��������ʧ�ĸ���
					c_lost ++;	
					
				if(llx!=navigat->tx || lly!=navigat->ty)
				{
#if WALK_PRINTF	&& BURNIN_LOG_EN
					log_printf("*(%d,%d,%3.1f,0)-[%3.1f,%3.1f,0]*\r\n",navigat->tx,navigat->ty,sys->angle,navigat->x_org_f,navigat->y_org_f);
#endif
					llx = navigat->tx;
					lly = navigat->ty;
				}
				/**************************************************************************
					���tox��toy���������ã��򵽵��ˣ���ͣ������
				*****************************************************************************/
				if(  (((X_NOW == tox || tox ==0) && (Y_NOW == toy || toy==0)) && (tox || toy)) || 	//������
				(type == GO_NEAR_TYPE_ADJ &&  ((by > toy && Y_NOW < toy) || (by < toy && Y_NOW > toy)))) //adj��ȥ��Y�ᳬ����
				{
#if BURNIN_LOG_EN
						log_printf("xy ok(%d,%d,%3.1f)\r\n",X_NOW,Y_NOW,sys->angle);
#endif
						motor_run(GO_STOP,0,0,0);
						//delay_ms_sensers(200);
						burnin_coordinate_calcu(0);														//�����ԭʼ������ϵ
						coord_org2map(navigat->x_org,navigat->y_org,&navigat->tx,&navigat->ty);	//����ϵת��
						return RET_NEAR_OK;				
				}

				if(type == GO_NEAR_TYPE_NAVI)		//����
				{
					if(router_p2p(tox,toy,X_NOW,Y_NOW,&nx1,&ny1))		//�ܵ�����ȥ�����˳���
					{
#if BURNIN_LOG_EN
						log_printf("[motor_go_edgeways]can navi out2(%d,%d,)\r\n",X_NOW,Y_NOW);
#endif
						motor_run(GO_STOP,0,0,0);
						//delay_ms_sensers(200);
						burnin_coordinate_calcu(0);														//�����ԭʼ������ϵ
						coord_org2map(navigat->x_org,navigat->y_org,&navigat->tx,&navigat->ty);	//����ϵת��
						return RET_NEAR_OK;
					}
				}

				if(type == GO_NEAR_DRAW_MAP)		//����ͼ
				{
					if(Y_NOW < 100 &&  Y_NOW > 98 && disXY(X_NOW,100) < 3)
					{
#if BURNIN_LOG_EN
						log_printf("[motor_go_edgeways]to begin point2\r\n",X_NOW,Y_NOW);
#endif
						motor_run(GO_STOP,0,0,0);
						//delay_ms_sensers(200);
						burnin_coordinate_calcu(0);														//�����ԭʼ������ϵ
						coord_org2map(navigat->x_org,navigat->y_org,&navigat->tx,&navigat->ty);	//����ϵת��
						return RET_NEAR_OK;
					}
				}
			
			}
		}
		
	}


//	return 0;

l_go_out_for_dock_burnin:	
	if(near->n_sta == RIGHT_SIDE_NEAR)
	{
		burnin_robot_turn_deg(GO_LEFT,DEG_TURN_PWM,60);
	}else if(near->n_sta == LEFT_SIDE_NEAR)
	{
		burnin_robot_turn_deg(GO_RIGTH,DEG_TURN_PWM,60);
	}	
#if BURNIN_LOG_EN
	log_printf("[motor_go_fw]...\r\n");
#endif	
	CHECK_IDLE_STA_RT(0);
	sys->sState = SYS_DOCK;
	motor_go_fw(2000,NO_SIDE,0);
	return 0;
	//delay_ms(1000);
}

//�ϻ����������ʼ������
void proc_burn_in_init(void)
{
	//log_printf("dir = %d,pwm=%d,speed=%d,hw=%d\r\n",dir,ctrl->pwm,ctrl->speed,ctrl->hw);
	sys->shut_down_motor = 0;
	navigat_init(0);

	MOTOR_POWER_ON();
	MOTOR_CTRL(NORM_SIDE_PWM_L,NORM_SIDE_PWM_R,NORM_MID_PWM,sys->dust_pwm_value)
	sys->sState = SYS_RANDOM;
	burnin_work_near_timer = 0;
	burnin_work_timer = 0;
	bummech_ct = bumir_ct = 0;

	bummech_left_ct = bummech_right_ct = 0;
	bumir_left_ct = bumir_right_ct = 0;
}
//�ϻ���������
void proc_burn_in_task(void)
{
	uint8_t near_dir = LEFT_SIDE_NEAR;

	
	tx_com_burnin(near_dir, burnin_work_timer, bummech_left_ct, bumir_left_ct, bummech_right_ct, bumir_right_ct);
	motor_go_forwark(0,NO_SIDE_NEAR,NULL);		//ֱ��
	motor_go_burnin(LEFT_SIDE_NEAR,0,0,GO_NEAR_TYPE_NO,0);
	bummech_left_ct = bummech_ct;
	bummech_ct = 0;
	bumir_left_ct = bumir_ct;
	bumir_ct = 0;

	tx_com_burnin(near_dir, burnin_work_timer, bummech_left_ct, bumir_left_ct, bummech_right_ct, bumir_right_ct);
	near_dir = RIGHT_SIDE_NEAR;
	motor_go_forwark(0,NO_SIDE_NEAR,NULL);		//ֱ��
	motor_go_burnin(RIGHT_SIDE_NEAR,0,0,GO_NEAR_TYPE_NO,0);
	bummech_right_ct = bummech_ct;
	bummech_ct = 0;
	bumir_right_ct = bumir_ct;
	bumir_ct = 0;

	tx_com_burnin(NO_SIDE_NEAR, burnin_work_timer, bummech_left_ct, bumir_left_ct, bummech_right_ct, bumir_right_ct);
	STOP_ALL_MOTOR();
	sys->sState = SYS_IDLE;
	ny3p_play(VOICE_M_FINISHED);
}

//=========================================================================================end


void sn_print(uint8_t * pdat,uint16_t len)
{
	uint16_t i;

	len --;
	log_printf("\r\n[sn %d]:",len);
	for(i = 0;i < len;i ++)
	{
		log_printf("%c",pdat[i]);
	}
	log_printf("\r\n",len);
	for(i = 0;i < len;i ++)
	{
		log_printf("%02X ",pdat[i]);
	}
	log_printf("\r\n%02X\r\n",pdat[i]);
}

void data_print(uint8_t * pdat,uint16_t len)
{
	uint16_t i;
	
	log_printf("\r\n[data_print %d]:",len);
	for(i = 0;i < len;i ++)
	{
		log_printf("%02x ",pdat[i]);
	}
	log_printf("\r\n\r\n");
}

//�ڲ���ģʽ�»�ȡ������������������,�������ݾ���λ��ʾ,
// λ������bit0-bit7�ֱ�Ϊ:Ĩ��/����ǽ/����/��������/������/������/����ײ/����ײ
uint8_t get_mactest_sensors(void)
{
	uint8_t i;
	uint16_t n_res = 0;
	
	for(i=0;i<20;i++)//Ĩ�����
	{
		if(READ_MOP_DET() == 0)//�ſ�ʱӦΪ�ߵ�ƽ,�����⵽�͵�ƽ���ж�Ϊ����쳣
			break;
	}
	if(i==20)
		n_res |= 0x01;
	
	for(i=0;i<20;i++)//����ǽ
	{
		if(READ_VWALL_DET() == 0)//�ſ�ʱӦΪ�ߵ�ƽ,�����⵽�͵�ƽ���ж�Ϊ����쳣
			break;
	}
	if(i==20)
		n_res |= 0x02;
	
	for(i=0;i<20;i++)//���м��
	{
		if(READ_DUSTBOX_DET() != 0)//�ſ�ʱӦΪ�ߵ�ƽ,�����⵽�͵�ƽ���ж�Ϊ����쳣
			break;
	}
	if(i==20)
		n_res |= 0x04;
	/* //������ⲻ��
	for(i=0;i<20;i++)//�������
	{
		if(READ_DUST_DET() != 0)//�ſ�ʱӦΪ�ߵ�ƽ,�����⵽�͵�ƽ���ж�Ϊ����쳣
			break;
	}
	if(i==20)
		n_res |= 0x08;
	*/
	for(i=0;i<20;i++)//������
	{
		if(LEFT_MOTOR_LEAVE() != 0)//�ſ�ʱӦΪ�ߵ�ƽ,�����⵽�͵�ƽ���ж�Ϊ����쳣
			break;
	}
	if(i==20)
		n_res |= 0x10;
	
	for(i=0;i<20;i++)//������
	{
		if(RIGHT_MOTOR_LEAVE() != 0)//�ſ�ʱӦΪ�ߵ�ƽ,�����⵽�͵�ƽ���ж�Ϊ����쳣
			break;
	}
	if(i==20)
		n_res |= 0x20;
		
	for(i=0;i<20;i++)
	{
		if(((GPIOD->IDR) & MASK_BUM_LEFT)!=0)
			break;
	}
	if(i==20)
		n_res |= 0x40;
		
	for(i=0;i<20;i++)
	{
		if(((GPIOD->IDR) & MASK_BUM_RIGHT)!=0)
			break;
	}
	if(i==20)
		n_res |= 0x80;

	return n_res;
}

typedef struct __st_tmirmaxmin
{
	uint16_t min,max;
}tm_irmaxmin;

#if 0
tm_irmaxmin irmaxmin[7] = {{1000,3800},
							{500,2000},
							{1000,3800},
							{1000,3800},
							{1000,3800},
							{500,2000},
							{1000,3800}};
#else
tm_irmaxmin irmaxmin[7] = {{50,3000},
							{0,3800},
							{600,3800},
							{50,3000},
							{600,3800},
							{0,3800},
							{50,3000}};
#endif

uint8_t const pn_number[] = {0x10,0x01,0x18,0x45,0x12,0x34,0x56,0x67,0x78,0x90};

//�����Ĳ��Գ���,ȫ�ֱ�������proc_bdtest_task(),��Ϊ������������ͬʱ��ʹ��
//����������Ҫʵʱ�ϱ����ֿ�������״̬,��������ʾ,�����������ڲ���ģʽ��ר�ż��
void proc_mactest_task(void)
{
	static uint8_t bd_timer = 0xff;
	static uint8_t bd_heart_timer = 0;
	static uint8_t bd_heart_ct = 0;
//	static uint8_t bdtest_sta_old = TPC_STOP_ALL;
	static uint16_t bd_batcharge_timer = 0;
	
	get_irda(&sys->gSta);
	
	if(TIM5->CNT < 50000)//50ms��ʱ
		return;
	TIM5->CNT = 0;

	if(bd_heart_timer ++ >= 20)//����������,ÿ��һ��
	{
		uint8_t n_sensors;
		bd_heart_timer = 0;
		n_sensors = get_mactest_sensors();
		#if BDTEST_LOGPRINT == 0
		tx_com_beep(bd_heart_ct ++, TPC_ERR_NOERROR, n_sensors,sys->g_sta,&sys->g_buton[0][0]);
		#endif
		//log_printf(">>>>>>>>>>>>\r\n");
	}
	
	switch(bdtest_sta)
	{
		case TPC_STOP_ALL:
			MOTOR_POWER_OFF_NPRI();//�رյ����Դ
			//TargetSysReset();
			break;
		case TPC_START_ALL://����Ҫ����������Ƿ�����/����԰��ͨ���Ƿ�����
			MOTOR_CTRL(1000, 1000, 300, 0);
			MOTOR_POWER_OFF_NPRI();
			bdtest_sta = TPC_MT_FAN;//TPC_MT_FAN;//TPC_IR_BOT;//TPC_MT_FAN;//TPC_SMT_L;;//�������ʱ���ԭ����Ҫ�ȿ����
			bdtest_sta_child = 0;
			navigat->distance = 0;
			#if BDTEST_LOGPRINT
			log_printf(">>>>>>>>>>>>\r\n");
			#endif
			//tx_com_dat(TPC_TRANS_PACK, 0, 0x20, 0);//�ϱ�����
			tx_com_dat(TPC_TRANS_PACK, 0 , 0x00, 0);//�ϱ�����
			delay_ms(500);
			break;
		case TPC_MT_FAN:
			switch(bdtest_sta_child)
			{
				case 0://��ʼ���Է��
					bdtest_sta_child = 1;
					GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource13);   
					MOTOR_POWER_OFF_NPRI();
					delay_ms(500);
					SET_DST_MOTER(300);//�������
					//log_printf("fdafas\r\n");
					delay_ms(100);
					MOTOR_POWER_ON();
					//��ʼ�����ת���жϵ�IO��

					bdtest_fantest = 1;
					tmod_fancd = 0;
					
					tx_com_dat(TPC_MT_FAN, 5 , TPC_ERR_NOERROR, 0);//�ϱ�����
					bd_timer = 0;
					break;
				case 1://��ȡ������
					//break;
					if(bd_timer ++ < 20)break;
					SET_DST_MOTER(0);//�رշ��
					bdtest_fantest = 0;
					GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource13);//�ָ��ֳ�
					#if BDTEST_LOGPRINT
					if(tmod_fancd < 200)//�������С��20,�������������,����
					{
						log_printf("fan cd = %d\r\n",tmod_fancd);
					}
					delay_ms(200);
					#endif
					tx_com_dat(TPC_MT_FAN, 8 , tmod_fancd < 200 ? TPC_ERR_FORWARD:TPC_ERR_NOERROR, 1);//�ϱ����ȼ�������Ϣ
					
					bd_timer = 0;
				#if BDTEST_LOGPRINT
					bdtest_sta_child = 2;
					break;
				case 2:
					log_printf("^");
					//SET_MID_MOTER(0);
					//SET_DST_MOTER(0)
					MOTOR_POWER_OFF_NPRI();
					break;
				#endif
				default:
					bdtest_sta = TPC_WHL_L;
					bdtest_sta_child = 0;
					bdtest_fantest = 0;
					break;
			}
			break;
		case TPC_WHL_L://���ּ��
			switch(bdtest_sta_child)
			{
				case 0://��ʼ��������
					bdtest_sta_child = 1;
					MOTOR_POWER_ON();
					motor.c_left_hw = 0;
					motor_wheel_stop(RIGHT_WHEEL);
					motor_wheel_forward(LEFT_WHEEL,600);
					
					tx_com_dat(TPC_WHL_L, 25 , TPC_ERR_NOERROR, 0);//�ϱ�����
					bd_timer = 0;
					break;
				case 1://��ǰת
					if(bd_timer ++ < 20)break;
					motor_wheel_stop(LEFT_WHEEL);
					#if BDTEST_LOGPRINT
					log_printf("forward:%d\r\n",motor.c_left_hw);
					#endif
					if(motor.c_left_hw < 200)//����������
					{
						tx_com_dat(TPC_WHL_L, 50, TPC_ERR_FORWARD, 0);
						#if BDTEST_LOGPRINT
						log_printf("TPC_WHL_L:FAIL\r\n");
						#else
						bdtest_sta = TPC_WHL_R;//������һ������
						bdtest_sta_child = 0;
						break;
						#endif
					}
					tx_com_dat(TPC_WHL_L, 40, TPC_ERR_NOERROR, 0);
					bdtest_sta_child = 2;
					//MOTOR_POWER_ON();
					motor.c_left_hw = 0;
					motor_wheel_backward(LEFT_WHEEL,600);
					bd_timer = 0;
					break;
				case 2://���ת
					if(bd_timer ++ < 20)break;
					motor_wheel_stop(LEFT_WHEEL);

					#if BDTEST_LOGPRINT
					log_printf("back:%d\r\n",motor.c_left_hw);
					#endif
					if(motor.c_left_hw > -200)
					{
						//MOTOR_POWER_OFF_NPRI();
						//motor_wheel_forward(LEFT_WHEEL,GO_FORWARD,600);
						tx_com_dat(TPC_WHL_L, 75, TPC_ERR_BACK, 0);//�ϱ��������ת
						
						#if BDTEST_LOGPRINT
						log_printf("TPC_WHL_L:FAIL\r\n");
						#else
						bdtest_sta = TPC_WHL_R;//������һ������
						bdtest_sta_child = 0;
						break;
						#endif
					}
					tx_com_dat(TPC_WHL_L, 100, TPC_ERR_NOERROR, 0);//�ϱ�����
					
					#if BDTEST_LOGPRINT
					delay_ms(50);
					bdtest_sta_child = 3;
					motor_wheel_stop(LEFT_WHEEL);
					log_printf("TPC_WHL_L STOP!\r\n");
					#else
					bdtest_sta = TPC_WHL_R;//������һ������
					bdtest_sta_child = 0;
					#endif
					//MOTOR_POWER_ON();
					motor.c_left_hw = 0;
					
					bd_timer = 0;
					tmd_rxbd_dat.unread_flag = 0;//�����ݽ��յĻ���δ����־��0
					break;
				#if BDTEST_LOGPRINT
				case 3:
					log_printf("TPC_WHL_L STOP!\r\n");
					motor_wheel_stop(LEFT_WHEEL);
					//MOTOR_POWER_OFF_NPRI();
					break;
				#endif
				default:
					bdtest_sta = TPC_WHL_R;
					bdtest_sta_child = 0;
					motor_wheel_stop(LEFT_WHEEL);
					//MOTOR_POWER_OFF_NPRI();
					break;
			}
			break;
		case TPC_WHL_R://���ּ��
			switch(bdtest_sta_child)
			{
				case 0://��ʼ��������
					bdtest_sta_child = 1;
					MOTOR_POWER_ON();
					motor.c_right_hw = 0;
					motor_wheel_stop(LEFT_WHEEL);
					motor_wheel_forward(RIGHT_WHEEL,600);
					
					tx_com_dat(TPC_WHL_R, 25 , TPC_ERR_NOERROR, 0);//�ϱ�����
					bd_timer = 0;
					break;
				case 1://��ǰת
					if(bd_timer ++ < 20)break;
					motor_wheel_stop(RIGHT_WHEEL);
					#if BDTEST_LOGPRINT
					log_printf("forward:%d\r\n",motor.c_right_hw);
					#endif
					if(motor.c_right_hw < 200)//����������
					{
						tx_com_dat(TPC_WHL_R, 50, TPC_ERR_FORWARD, 0);
						#if BDTEST_LOGPRINT
						log_printf("TPC_WHL_R:FAIL\r\n");
						#else
						bdtest_sta = TPC_IR_FRT;//������һ������
						bdtest_sta_child = 0;
						break;
						#endif
					}
					tx_com_dat(TPC_WHL_R, 40, TPC_ERR_NOERROR, 0);
					bdtest_sta_child = 2;
					//MOTOR_POWER_ON();
					motor.c_right_hw = 0;
					motor_wheel_backward(RIGHT_WHEEL,600);
					bd_timer = 0;
					break;
				case 2://���ת
					if(bd_timer ++ < 20)break;
					//motor_wheel_stop(RIGHT_WHEEL);
					motor_wheel_stop(DOUBLE_WHEEL);

					#if BDTEST_LOGPRINT
					log_printf("back:%d\r\n",motor.c_right_hw);
					#endif
					if(motor.c_right_hw > -200)
					{
						//MOTOR_POWER_OFF_NPRI();
						//motor_wheel_forward(LEFT_WHEEL,GO_FORWARD,600);
						tx_com_dat(TPC_WHL_R, 75, TPC_ERR_BACK, 0);//�ϱ��������ת
						motor_wheel_stop(DOUBLE_WHEEL);
						
						#if BDTEST_LOGPRINT
						log_printf("TPC_WHL_R:FAIL\r\n");
						#else
						bdtest_sta = TPC_IR_FRT;//������һ������
						bdtest_sta_child = 0;
						break;
						#endif
					}
					tx_com_dat(TPC_WHL_R, 100, TPC_ERR_NOERROR, 0);//�ϱ�����
					
					#if BDTEST_LOGPRINT
					delay_ms(50);
					bdtest_sta_child = 3;
					motor_wheel_stop(DOUBLE_WHEEL);
					log_printf("TPC_WHL_L STOP!\r\n");
					#else
					bdtest_sta_child = 0;
					bdtest_sta = TPC_IR_FRT;//������һ������
					#endif
					//MOTOR_POWER_ON();
					motor.c_right_hw = 0;
					
					bd_timer = 0;
					tmd_rxbd_dat.unread_flag = 0;//�����ݽ��յĻ���δ����־��0
					motor_wheel_stop(DOUBLE_WHEEL);
					break;
				#if BDTEST_LOGPRINT
				case 3:
				break;
				#endif
				default:
					bdtest_sta = TPC_IR_FRT;
					bdtest_sta_child = 0;
					motor_wheel_stop(DOUBLE_WHEEL);
					//MOTOR_POWER_OFF_NPRI();
					break;
			}
			break;
		case TPC_IR_FRT://ǰײ���(��ײ)������
			switch(bdtest_sta_child)
			{
				case 0:
					{
						uint8_t cc;
						//#if BDTEST_LOGPRINT
						///uint8_t bot;
						//#endif
						int32_t k1,k2;
						uint8_t i,j;
						
						sys->c_ir_adc = 0;
						for(cc = 0;cc < 9;cc ++)
						{
							for(i=0;i<10;i++)
							{
								sys->m_sta[i][cc] = adc_converted_value[i];
								//log_printf(",%d",sys->m_sta[i][sys->c_ir_adc] );
							}
							//log_printf("\r\n");

							if(cc <=3 )
							{
								GPIO_ResetBits(PORT_IR_CTRL,PIN_IR_CTRL);	//�͵�ƽ���ص�
								//GPIO_ResetBits(PORT_IR_CTRL2,PIN_IR_CTRL2); //�͵�ƽ���ص�
								//�ż�
								NEAR_LAN_ON();
								FAR_LAN_OFF();
							}
							else 
							{

								GPIO_SetBits(PORT_IR_CTRL,PIN_IR_CTRL);
								//GPIO_SetBits(PORT_IR_CTRL2,PIN_IR_CTRL2);
								NEAR_LAN_OFF();
								FAR_LAN_ON();
							}
							delay_ms(5);
						}

						cc /*= bot */= 0;//cc������¼ǰײ����ļ����,j������¼�Եغ���ļ����
						for(i=0;i<10;i++)//�ѶԵغ���Ҳ�����,Ŀǰ���Եغ����������ȡ���ݣ���⻷��Ϊһ�����ߵ��ϰ���
						{
							k1 = k2 = 0;
							for(j=0;j<4;j++)
								k1 +=sys->m_sta[i][j];
							for(j=4;j<8;j++)
								k2 +=sys->m_sta[i][j];
							
							if(i < 7)//ǰײ����
							{
								if(i == 0 || i == 3 || i == 6)
								{
									sys->g_sta[i] = k1 >> 2;
								}
								else
								{
									if(k1 > k2 && ((k1 - k2) /4) < 5000)
										sys->g_sta[i] = (k1 - k2) >> 2;
									else
										sys->g_sta[i] = 0;
								}
								//if(sys->g_sta[i] < 500 || sys->g_sta[i] > 3500)//ǰײ����,�ں춨�����ȡֵ������Χ��Ϊ���ϸ�
								//if(sys->g_sta[i] < 1000 || sys->g_sta[i] > 3800)//ǰײ����,�ں춨�����ȡֵ������Χ��Ϊ���ϸ�
								if(sys->g_sta[i] < irmaxmin[i].min || sys->g_sta[i] > irmaxmin[i].max)
								{
									cc |= 1 << i;
								}

							}
							else
							{//���CASEֻ��ȡ�Եغ���ֵ����TPC_IR_BOT���ж϶Եغ����Ƿ�����
								sys->g_buton[0][i-7] = k2 >> 2; //sys->g_sta[i];
								sys->g_buton[1][i-7] = k1 >> 2;
								/*if(sys->g_buton[0][i-7] > 300)//���˺���
								{
									bot |= 1 << (i - 7);
								}
								if(sys->g_buton[1][i-7] > 1500)//Զ�˺���
								{
									bot |= 1 << (i - 7 + 3);
								}*/
							}
							//log_printf("%d\t%d\r\n",i,sys->g_sta[i]);
						}
						#if BDTEST_LOGPRINT//���ԵĴ�ӡ��Ϣ
						log_printf("\r\nirda:%02x\r\nirbt:%02x\r\n",cc,bot);
						delay_ms(500);
						#endif
						//cc = 0;
						if(cc)
						{
							tx_com_dat(TPC_IR_FRT, 100 , TPC_ERR_IRLED, cc);//�ϱ����ȼ�������Ϣ
						}
						else
						{
							tx_com_dat(TPC_IR_FRT, 100 , TPC_ERR_NOERROR, 0);//�ϱ����ȼ�������Ϣ
						}

						#if BDTEST_LOGPRINT
						delay_ms(500);
						log_printf("\r\nir_frt:\r\n");
						for(i = 0;i < 7;i ++)
						{
							log_printf("\t%d",sys->g_sta[i]);
						}
						log_printf("\r\nir_bot:\r\n");
						for(i = 0;i < 3;i ++)
						{
							log_printf("\t%d",sys->g_buton[0][i]);
						}
						log_printf("\r\n");
						for(i = 0;i < 3;i ++)
						{
							log_printf("\t%d",sys->g_buton[1][i]);
						}
						log_printf("\r\n");
						#endif
					}
					//�ر����к���LED,�Խ��͹���
					FAR_LAN_OFF();
					NEAR_LAN_OFF();
					GPIO_ResetBits(PORT_IR_CTRL,PIN_IR_CTRL);
					//GPIO_ResetBits(PORT_IR_CTRL2,PIN_IR_CTRL2);
					#if BDTEST_LOGPRINT//�����ܲ���״̬��,��״̬ת����Ҫ�ر�һ��
					bdtest_sta_child = 1;
					#else
					bdtest_sta = TPC_IR_BOT;
					#endif
					break;
				case 1:
				default:
					break;
			}
			break;
		case TPC_IR_BOT://ǰײ���(��ײ)������,�Եغ���������һ����
			switch(bdtest_sta_child)
			{
				case 0:
					{
						uint8_t i,bot;
						bdtest_sta_child = 3;
						///tm_ir_bot_k1 /*= tm_ir_bot_k2 */= 0;
						sys->c_ir_adc = 0;
						tm_ir_bot_cc = 4;
						tx_com_dat(TPC_IR_BOT, 50 , TPC_ERR_NOERROR, 0);//�ϱ���ʼ�Եغ������,�ȴ��û�������
																	//ģ���ϰ���ŵ��������沢ȷ��
						bot = 0;
						for(i = 0;i < 3;i ++)
						{
								if(sys->g_buton[0][i] > 1500)//���˺���
								{
									bot |= 1 << (i);
								}
								if(sys->g_buton[1][i] > 3000)//Զ�˺���
								{
									bot |= 1 << (i + 3);
								}
						}
						#if BDTEST_LOGPRINT//���ԵĴ�ӡ��Ϣ
						log_printf("\r\nirbt:%02x\r\n",bot);
						delay_ms(500);
						#endif
						//bot = 1;
						tx_com_dat(TPC_IR_BOT, 100 , bot ? TPC_ERR_IRLED:TPC_ERR_NOERROR, bot);//�ϱ����ȼ�������Ϣ
						
						//�ر����к���LED,�Խ��͹���
						FAR_LAN_OFF();
						NEAR_LAN_OFF();
						GPIO_ResetBits(PORT_IR_CTRL,PIN_IR_CTRL);
						//GPIO_ResetBits(PORT_IR_CTRL2,PIN_IR_CTRL2);
						#if BDTEST_LOGPRINT
						delay_ms(500);
						log_printf("\r\nir_bot:\r\n");
						for(i = 0;i < 3;i ++)
						{
							log_printf("\t%d",sys->g_buton[0][i]);
						}
						log_printf("\r\n");
						for(i = 0;i < 3;i ++)
						{
							log_printf("\t%d",sys->g_buton[1][i]);
						}
						log_printf("\r\n");
						#else
						bdtest_sta = TPC_BUM_SW;
						bdtest_sta_child = 0;
						tx_com_dat(TPC_TRANS_PACK, 30 , 0x00, 0);//�ϱ�����
						delay_ms(50);
						#endif
					}
					break;
				#if 0
				case 1://Ҫ�ȴ���λ��ȷ��֮���ٽ�����һ��
//					tm_ir_bot_i = tm_ir_bot_j = tm_ir_bot_cc = 0;
					tm_ir_bot_k1 /*= tm_ir_bot_k2 */= 0;
					sys->c_ir_adc = 0;
					//bdtest_sta_child = 3;//����ֱ���Ȳ���˺���,���˺�����ʱ����
					tm_ir_bot_cc = 4;
					delay_ms(50);
					log_printf("\r\n >>\r\n");
					delay_ms(50);
					NEAR_LAN_OFF();
					FAR_LAN_ON();
					break;
				case 2://��ʼ����Զ�˺������
					{
					
						uint8_t i;

						for(i=0;i<10;i++)
						{
							sys->m_sta[i][tm_ir_bot_cc] = adc_converted_value[i];
							//log_printf(",%d",sys->m_sta[i][sys->c_ir_adc] );
						}
						if(tm_ir_bot_cc <=3 )//�Ȳ��Խ���,��Ҫ�Ȱѽ��˵��ϰ���ŵ����������ס�Եغ���
						{
							//�ż�
							NEAR_LAN_ON();
							FAR_LAN_OFF();
						}
						else
						{
							bdtest_sta_child = 3;
							tx_com_dat(TPC_IR_BOT, 50 , TPC_ERR_IRLED, 0);//�ϱ���ʼ�Եغ������,�ȴ��û�������
																			//ģ���ϰ������߲�ȷ��
							break;
						}
						tm_ir_bot_cc ++;
					}
					break;
				case 3://����Ҫ�ȴ���λ�����û���Ӧ,��ģ��̨�׵��ϰ�������֮����ܲ�Զ�˵ĺ���
					bdtest_sta_child = 4;
					tm_ir_bot_cc = 0;
					NEAR_LAN_ON();
					FAR_LAN_OFF();
					delay_ms(50);
					break;
				case 4:
					{
						uint8_t i;

						NEAR_LAN_ON();
						FAR_LAN_OFF();
						for(i=7;i<10;i++)
						{
							sys->m_sta[i][tm_ir_bot_cc] = adc_converted_value[i];
							//log_printf(",%d",sys->m_sta[i][tm_ir_bot_cc] );
						}
						//log_printf("\r\n");
						//tm_ir_bot_cc ++;
						if(tm_ir_bot_cc >= 3 )//�Ȳ��Խ���,��Ҫ�Ȱѽ��˵��ϰ���ŵ����������ס�Եغ���
						{
							uint8_t j;
							uint8_t bot = 0;

								//log_printf("\r\n");
							for(i = 7;i < 10;i ++)
							{
								tm_ir_bot_k1 = 0;
								for(j = 0; j < 4;j ++)
								{
									tm_ir_bot_k1 += sys->m_sta[i][j];
								}
								//if(i >=7)//�Եغ���
								{
									sys->g_buton[0][i-7] = tm_ir_bot_k1 / 4; //sys->g_sta[i];
									//if(sys->g_buton[0][i-7] > 1000)//���˺���
									if(sys->g_buton[0][i-7] > 2500)//���˺���
									{
										bot |= 1 << (i - 7);
										//log_printf("err %d\r\n",i);
									}
									//log_printf("\r\n%d\t%d\r\n",sys->g_buton[0][i-7],tm_ir_bot_k1);
									//delay_ms(100);
								}
							}
							tx_com_dat(TPC_IR_BOT, 100 , bot ? TPC_ERR_IRLED:TPC_ERR_NOERROR, bot);//�ϱ����ȼ�������Ϣ
							
							//�ر����к���LED,�Խ��͹���
							FAR_LAN_OFF();
							NEAR_LAN_OFF();
							GPIO_ResetBits(PORT_IR_CTRL,PIN_IR_CTRL);
							GPIO_ResetBits(PORT_IR_CTRL2,PIN_IR_CTRL2);
							bdtest_sta = TPC_BUM_SW;
							bdtest_sta_child = 0;
							tx_com_dat(TPC_TRANS_PACK, 30 , 0x00, 0);//�ϱ�����
							delay_ms(500);
							break;
						}
						tm_ir_bot_cc ++;
					}
					break;
				#endif
				case 5:
					break;
				default:
					bdtest_sta = TPC_BUM_SW;
					bdtest_sta_child = 0;
					tx_com_dat(TPC_TRANS_PACK, 30 , 0x00, 0);//�ϱ�����
					delay_ms(500);
					break;
			}
			break;
		case TPC_BUM_SW:
		
			switch(bdtest_sta_child)
			{
				case 0://��ȡ�����޹�ʱ��GPIO������������,���������״̬��,��ʱӦ������ײ(�ߵ�ƽ)
					bdtest_sta_child = 1;
					{
						uint8_t n_res = 0;
						uint8_t n_err_res = 0;
						uint8_t i;
						
						for(i=0;i<20;i++)
						{
							if(((GPIOD->IDR) & MASK_BUM_LEFT)!=0)
								break;
						}
						if(i==20)
							n_err_res |= 0x01;
						else
							n_res |= 0x01;
							
						for(i=0;i<20;i++)
						{
							if(((GPIOD->IDR) & MASK_BUM_RIGHT) !=0)
								break;
						}
						if(i==20)
							n_err_res |= 0x02;
						else
							n_res |= 0x02;

						#if 0
						log_printf("\r\n double bum:%02x\r\n",n_res);
						delay_ms(100);
						#endif
						tx_com_dat(TPC_TRANS_PACK, 25 , 0x80, 0);//�ϱ�����
						delay_ms(500);
						
						//if(n_res)
						{
							tx_com_dat(TPC_BUM_SW, 25 , TPC_ERR_NOERROR, 0);//�ϱ����ȼ�������Ϣ
						}
						if(n_err_res)
						{
							tx_com_dat(TPC_BUM_SW, 25 , TPC_ERR_DIGI_IN, n_err_res);//�ϱ����ȼ�������Ϣ
							//bdtest_sta_child = 13;
						}
					}
					//������ײ�̵���
					//tx_com_dat(TPC_BUM_SW, 25 , TPC_ERR_NOERROR, 0);//�ϱ�����
						delay_ms(100);
					//tx_dp_dat(TBD_BUM_SW, 1, 0x01);//����ײ�̽ӵ���
					bd_timer = 0;
					break;
				case 1:
					/*if(bd_timer ++ < 20 && !tmd_rxbd_dat.unread_flag)break;//��ʱδ�յ�����
					bd_timer = 0;
					if(!tmd_rxbd_dat.unread_flag)//��ʱδ�յ�����,����԰�(�Կض˿�)ͨ��ʧ��
					{
						tx_com_dat(TPC_DPPORT_TST, 0, TPC_ERR_COMM, 0);//����ͨ�Ŵ���
						//tx_dp_dat(TBD_BUM_SW, 1, 0x01);//����ײ�̽ӵ���
						bdtest_sta_child = 1;
						break;
					}
					else
					{
						tmd_rxbd_dat.unread_flag = 0;
					}*/

					bdtest_sta_child = 2;
					{
					//��ʼ�������ײ�Ƿ񱻰���,ͬʱ����Ҳ���ײ�Ƿ�û�б�����
						uint8_t i;
						uint8_t n_err_res = 0;
						uint8_t n_res = 0;
					
					//��ʱӦ������ײ,�������ʱ�͵�ƽ����ʱ�䲻����û�е͵�ƽ��ʾ����ײ������
						if(( (GPIOD->IDR) & MASK_BUM_LEFT)==0 )

						{
							for(i=0;i<20;i++)
							{
								if(((GPIOD->IDR) & MASK_BUM_LEFT))
									break;
							}
							if(i<20)
								n_err_res |= 0x01;
							else
								n_res |= 0x01;
						}
						else
						{
							n_err_res |= 0x01;
						}
					
						if( (GPIOD->IDR & MASK_BUM_RIGHT) ==0 )
						{
							for(i=0;i<20;i++)
							{
								if(((GPIOD->IDR) & MASK_BUM_RIGHT))
									break;
							}
							if(i>=20)
								n_err_res |= 0x02;
							else
								n_res |= 0x02;
						}
						else
						{
							n_res |= 0x02;
						}
						#if 0
						log_printf("\r\nleft bum:%02x\tled:%d\r\n",n_res,tmd_rxbd_dat.bum_sw_led);
						delay_ms(100);
						#endif
						tx_com_dat(TPC_TRANS_PACK, 26 , 0x20, 0);//�ϱ�����
						delay_ms(500);
						//if(n_res)
						{
							tx_com_dat(TPC_BUM_SW, 26 , TPC_ERR_NOERROR, 0);//�ϱ����ȼ�������Ϣ
						}
						if(n_err_res)
						{
							tx_com_dat(TPC_BUM_SW, 26 , TPC_ERR_DIGI_IN, n_err_res);//�ϱ����ȼ�������Ϣ
							//bdtest_sta_child = 13;
						}
						
						delay_ms(100);
					}
					//else
					//	tx_com_dat(TPC_BUM_SW, 50, TPC_ERR_TB, 0);//�ϱ����Ȼ��ϱ�����������
					break;
				case 2:
					/*if(bd_timer ++ < 20 && !tmd_rxbd_dat.unread_flag)break;//��ʱδ�յ�����
					bd_timer = 0;
					if(!tmd_rxbd_dat.unread_flag)//��ʱδ�յ�����,����԰�(�Կض˿�)ͨ��ʧ��
					{
						tx_com_dat(TPC_DPPORT_TST, 0, TPC_ERR_COMM, 0);//����ͨ�Ŵ���
						//tx_dp_dat(TBD_BUM_SW, 1, 0x02);//����ײ�ſ�,����ײ�̽ӵ���
						bdtest_sta_child = 2;
						break;
					}
					else
					{
						tmd_rxbd_dat.unread_flag = 0;
					}
					*/
					bdtest_sta_child = 3;
					{
					//��ʼ�������ײ�Ƿ񱻰���,ͬʱ����Ҳ���ײ�Ƿ�û�б�����
						uint8_t i;
						uint8_t n_err_res = 0;
						uint8_t n_res = 0;
					
					//��ʱӦ������ײ,�������ʱ�͵�ƽ����ʱ�䲻����û�е͵�ƽ��ʾ����ײ������
						if(( GPIOD->IDR & MASK_BUM_LEFT)==0 )

						{
							for(i=0;i<20;i++)
							{
								if(((GPIOD->IDR) & MASK_BUM_LEFT))
									break;
							}
							if(i>=20)
								n_err_res |= 0x01;
							else
								n_res |= 0x01;
						}
						else
						{
							n_res |= 0x01;
						}
					
						if( (GPIOD->IDR & MASK_BUM_RIGHT) ==0 )
						{
							for(i=0;i<20;i++)
							{
								if(((GPIOD->IDR) & MASK_BUM_RIGHT))
									break;
							}
							if(i<20)
								n_err_res |= 0x02;
							else
								n_res |= 0x02;
						}
						else
						{
							n_err_res |= 0x02;
						}
						#if 0
						log_printf("\r\nleft bum:%02x\tled:%d\r\n",n_res,tmd_rxbd_dat.bum_sw_led);
						#endif
						tx_com_dat(TPC_TRANS_PACK, 27 , 0x40, 0);//�ϱ�����
						delay_ms(500);
						
						//if(n_res)
						{
							tx_com_dat(TPC_BUM_SW, 27 , TPC_ERR_NOERROR, 0);//�ϱ����ȼ�������Ϣ
						}
						if(n_err_res)
						{
							tx_com_dat(TPC_BUM_SW, 27 , TPC_ERR_DIGI_IN, n_err_res);//�ϱ����ȼ�������Ϣ
							//bdtest_sta_child = 13;
						}
						delay_ms(100);
						//delay_ms(50);
					}
					//bdtest_sta_child = 2;
					break;
				case 3:
					/*if(bd_timer ++ < 20 && !tmd_rxbd_dat.unread_flag)break;//��ʱδ�յ�����
					bd_timer = 0;
					if(!tmd_rxbd_dat.unread_flag)//��ʱδ�յ�����,����԰�(�Կض˿�)ͨ��ʧ��
					{
						tx_com_dat(TPC_DPPORT_TST, 0, TPC_ERR_COMM, 0);//����ͨ�Ŵ���
						//tx_dp_dat(TBD_BUM_SW, 1, 0x02);//����ײ�ſ�,����ײ�̽ӵ���
						bdtest_sta_child = 2;
						break;
					}
					else
					{
						tmd_rxbd_dat.unread_flag = 0;
					}
					*/
					{
					//��ʼ�������ײ�Ƿ񱻰���,ͬʱ����Ҳ���ײ�Ƿ�û�б�����
						uint8_t i;
						uint8_t n_err_res = 0;
						uint8_t n_res = 0;
					
					//��ʱӦ������ײ,�������ʱ�͵�ƽ����ʱ�䲻����û�е͵�ƽ��ʾ����ײ������
						if(( GPIOD->IDR & MASK_BUM_LEFT)==0 )

						{
							for(i=0;i<20;i++)
							{
								if(((GPIOD->IDR) & MASK_BUM_LEFT))
									break;
							}
							if(i<20)
								n_err_res |= 0x01;
							else
								n_res |= 0x01;
						}
						else
						{
							n_err_res |= 0x01;
						}
					
						if( (GPIOD->IDR & MASK_BUM_RIGHT) ==0 )
						{
							for(i=0;i<20;i++)
							{
								if(((GPIOD->IDR) & MASK_BUM_RIGHT))
									break;
							}
							if(i<20)
								n_err_res |= 0x02;
							else
								n_res |= 0x02;
						}
						else
						{
							n_err_res |= 0x02;
						}
						#if 0
						log_printf("\r\nleft bum:%02x\tled:%d\r\n",n_res,tmd_rxbd_dat.bum_sw_led);
						#endif
						tx_com_dat(TPC_TRANS_PACK, 30 , 0x00, 0);//�ϱ�����
						delay_ms(500);
						
						if(n_res)
						{
							tx_com_dat(TPC_BUM_SW, 28 , TPC_ERR_NOERROR, n_res);//�ϱ����ȼ�������Ϣ
						}
						if(n_err_res)
						{
							tx_com_dat(TPC_BUM_SW, 28 , TPC_ERR_DIGI_IN, n_err_res);//�ϱ����ȼ�������Ϣ
						}
						//delay_ms(50);
					}
					//bdtest_sta_child = 2;
					//break;
				#if 0
					bdtest_sta_child = 3;
					break;
				case 3:
					break;
				#endif
				default:
					tx_com_dat(TPC_TRANS_PACK, 29 , 0x00, 0);//�ϱ�����
					delay_ms(100);
					bdtest_sta = TPC_IRDA_CHRG;
					bdtest_sta_child = 0;
					bdtest_fantest = 0;
					break;
			}
			break;
		case TPC_IRDA_CHRG:
			switch(bdtest_sta_child)
			{
			case 0:
				{
					uint8_t n_irdata[3] = {0};
					
					read_ir_original_data(n_irdata);//��һ�κ���,��֮ǰ��ֵ���
				}
				bdtest_sta_child = 1;
				bd_batcharge_timer = 0;//�ѱ�����ʱ�������һ��,���������ղ��������źŵļ���
				break;
			case 1:
				{
					uint8_t n_irdata[3] = {0};
					uint8_t n_irres;
					//uint8_t n_irknk = 0;
					//uint8_t i,cc;
					read_ir_original_data(n_irdata);

					n_irres = (n_irdata[0] | n_irdata[1] | n_irdata[2]) & 0x01;//ȡ�����ź�
					
					if(!n_irdata[0])n_irres |= 0x08;
					if(!n_irdata[1])n_irres |= 0x02;
					if(!n_irdata[2])n_irres |= 0x04;
					if(n_irres != 0 && bd_batcharge_timer ++ > 2)
					{
						#if 0
						log_printf("\r\n %02x %02x %02x res:%02x\r\n",n_irdata[0],n_irdata[1],n_irdata[2],n_irres);
						//log_printf("\r\n ---%02x---\r\n",n_irdata[0]);
						delay_ms(200);
						#else
						bdtest_sta_child = 0;
						bd_batcharge_timer = 0;
						bdtest_sta = TPC_GYRO_TST;
						tx_com_dat(TPC_IRDA_CHRG, 100 , n_irres!=0 ? TPC_ERR_IRLED:TPC_ERR_NOERROR, n_irres);//�ϱ����ȼ�������Ϣ
						#endif
					}
					else if(n_irres == 0)
					{
						bdtest_sta_child = 0;
						bdtest_sta = TPC_GYRO_TST;
						tm_ir_bot_cc = 0;//���õ������ǵĲ�����������ͨ��ʧ�ܵ����Լ�����,���ֵ������5
						tx_com_dat(TPC_IRDA_CHRG, 100 , n_irres!=0 ? TPC_ERR_IRLED:TPC_ERR_NOERROR, n_irres);//�ϱ����ȼ�������Ϣ
					}
				}
				#if 1//�����ܲ���״̬��,��״̬ת����Ҫ�ر�һ��
				#else
				bdtest_sta_child = 1;
				#endif
				break;
			case 2:
			default:
				break;
			}
			break;
		case TPC_GYRO_TST:
			switch(bdtest_sta_child)
			{
			case 0:
				{
					uint8_t n_gyro_res = 0;
					uint8_t n_err_res = TPC_ERR_DIGI_IN;
					SPI_CS(0);
					SPI1_ReadWriteByte(0xB5);
					n_gyro_res = SPI1_ReadWriteByte(0);
					SPI_CS(1);

					if(n_gyro_res == 0x70)//��������
					{
						n_err_res = TPC_ERR_NOERROR;
					}
					else if(n_gyro_res == 0)//�����ǰ��쳣��δ��
					{
						n_err_res = TPC_ERR_COMM;//����ͨ���쳣
					}
					else if(n_gyro_res == 0x7A)//������IC�쳣
					{
						n_err_res = TPC_ERR_DIGI_IN;//����ͨ���쳣
					}
					else
					{
						n_err_res = TPC_ERR_DIGI_OUT;//����ͨ���쳣
					}
					#if BDTEST_LOGPRINT
					bdtest_sta_child = 1;
					log_printf("n_gyro_res:%02x\t err:%d\r\n",n_gyro_res,n_err_res);
					#else
					//bdtest_sta = TPC_STOP_ALL;
					if(n_err_res != TPC_ERR_NOERROR)
					{
						delay(5);
						if(test_gyro_org())
							n_err_res = TPC_ERR_NOERROR;
					}
					if(n_err_res == TPC_ERR_NOERROR || (tm_ir_bot_cc ++ > 20))//�������ǳɹ����߲��ɹ������Դ�������4
					{
						
						tx_com_dat(TPC_GYRO_TST, 100 , n_err_res, 0);//�Ϸ��������
						//tx_com_dat(TPC_STOP_ALL, 100 , TPC_ERR_NOERROR, 0);//�Ϸ��������
						bdtest_sta = TPC_CD_FRT;
					}
					//else
					//	bdtest_sta_child = 0;
					#endif
				}
			break;
			case 1:
				break;
			default:
				break;
			}
			break;
		case TPC_CD_FRT://����ǰ������,ǰ�����̵Ĳ��Ե�������ͻ�ת��ֱ�Ӷ�ǰ�����̼������Ϳ���
			{
				int n_cd_frt = navigat->distance;
				if(n_cd_frt < 0)n_cd_frt = -n_cd_frt;
				tx_com_dat(TPC_CD_FRT, 100 , (n_cd_frt > 20) ? TPC_ERR_NOERROR:TPC_ERR_FORWARD, 0);//�ϱ�����
				//tx_com_dat(TPC_CD_FRT, 100 , TPC_ERR_NOERROR, 0);//�ϱ�����
				bdtest_sta = TPC_CHRG_TST;
				bdtest_sta_child = 0;
				bd_batcharge_timer = 0;
			}
			break;
		case TPC_CHRG_TST://������
			switch(bdtest_sta_child)
			{
				case 0://��ͨ���忪ʼ���Գ��,Ҫ�����Ա�������ŵ�������ϻ����Ƚ�������ĵ�Դ���ٰ�ɨ�ػ�����ȥ
					//tx_dp_dat(TBD_CHRG_TST, 3, 0);//��24V��DC�����ӿ�
					#if BDTEST_LOGPRINT
					bdtest_sta_child = 1;
					log_printf("\r\nTPC_CHRG_TST\r\n");
					delay_ms(100);
					#else
					bdtest_sta_child = 1;//�ϱ�PC���ȴ�DC��ͷ����
					#endif
					tx_com_dat(TPC_CHRG_TST, 0 , TPC_ERR_NOERROR, 0);//�ϴ���ʼ������,��λ��������ʾ�������ŵ��������,�ټ�������
					break;
				case 1://�·������԰�,Ҫ����԰彫��ص���ؽŽ��뵽������
					bdtest_sta_child = 2;
					//�Ȳ�����ص��¶�,������Ƿ���ڻ����,���DC������������źŽ��Ƿ�����
					{
						uint16_t n_bat_temp = LiBat_GetBatTemper();//��ȡ����¶�
						uint8_t n_error_code = TPC_ERR_NOERROR;
						#if 1
						if(n_bat_temp == 1500)//��ز�����,�˳�����ģʽ
						{
							//bdtest_sta_child = 0;
							//bdtest_sta = TPC_STOP_ALL;
							n_error_code = TPC_ERR_BATNE;
							//break;
						}
						else if(/*n_bat_temp < 0 ||*/ n_bat_temp > 600)//��ع���,�˳�����ģʽ
						{
							//bdtest_sta_child = 0;
							//bdtest_sta = TPC_STOP_ALL;
							n_error_code = TPC_ERR_BATOT;
							//break;
						}
						#if BDTEST_LOGPRINT
						bdtest_sta_child = 1;
						log_printf("\r\n BAT temp : %d\r\n",n_bat_temp);
						delay_ms(50);
						#endif
						#endif
						//���������DC�����ĵ�·�Ƿ�����,δ�����״̬��,��ӦΪ�͵�ƽ
						//if(DOCK_DETECT())//��������Ƿ��ѽ�������Ƿ��е�
						if(EXTERAL_AC_DETECT())//��������Ƿ��ѽ�������Ƿ��е�
						{
							//bdtest_sta_child = 0;
							//bdtest_sta = TPC_STOP_ALL;
							#if BDTEST_LOGPRINT
							bdtest_sta_child = 1;
							log_printf("\r\EXTERAL_AC_DETECT\r\n");
							delay_ms(50);
							#else
							tx_com_dat(TPC_CHRG_TST, 10 , n_error_code, 0);
							#endif
							//break;
						}
						else
						{
							n_error_code = TPC_ERR_DOCK;
						}

						#if BDTEST_LOGPRINT
						log_printf("\r\nbat:%d\r\n",n_error_code);
						delay_ms(100);
						#endif
						
						if(n_error_code == TPC_ERR_DOCK)
						{
							bdtest_sta_child = 1;
							break;
						}
						else if(n_error_code != TPC_ERR_NOERROR)
						{
							bdtest_sta = TPC_STOP_ALL;
							bdtest_sta_child = 0;
							break;
						}
					}
					bd_timer = 0;
					break;
				case 2://�·������԰�,Ҫ����԰彫��ص���ؽŽ��뵽������
					bdtest_sta_child = 3;
					bd_timer = 0;
					break;
				case 3://���DC�����źż���Ƿ�����
					{
//						uint8_t n_error_code = TPC_ERR_NOERROR;
						
						delay_ms(200);
						tmd_rxbd_dat.unread_flag = 0;
						//if(DOCK_DETECT() == 0)//�����źż�����
						//if(EXTERAL_AC_DETECT() == 0)//�����źż�����
//							n_error_code = TPC_ERR_DOCK;
							
						//log_printf("\r\n 2:%d\r\n",n_error_code);
						delay_ms(100);
						//tx_com_dat(TBD_CHRG_TST, 40 , n_error_code, 0);
						LiBat_HalInit();
						bdtest_chgfailed_counter = 0xff;
						bdtest_chgsucceed_counter = 0xff;
						bdtest_chgstart_counter = 0;
						bd_batcharge_timer = 0;
						//delay_ms(500);
					}
					
					bdtest_sta_child = 4;
					bd_timer = 0;
					break;
				case 4:
					{
						uint8_t n_charge_res;
				  	 	n_charge_res = LiBat_CurrentPid(LIBAT_CHARGECURRENT_SET,2);

				  	 	if(n_charge_res == LB_ERROR_NONE)
				  	 	{
				  	 		bd_batcharge_timer ++;
				  	 		if(bd_batcharge_timer > 600)//�������,ֹͣ���
				  	 		{
				  	 			bd_batcharge_timer = 0;
				  	 			bdtest_sta_child = 0;//0
								LiBat_ExitChargeMode();
								bdtest_sta = TPC_STOP_ALL;
								tx_com_dat(TPC_CHRG_TST, 100 , TPC_ERR_NOERROR, 0);
								#if BDTEST_LOGPRINT
								log_printf("\r\nLB_CHS_NORMAL\r\n");
								#endif
								bdtest_chgfailed_counter = 0xff;
								bdtest_chgsucceed_counter = 0xff;
								bdtest_chgstart_counter = 0;
				  	 		}
				  	 		else
				  	 		{
				  	 			if(bdtest_chgsucceed_counter == 0xff)
				  	 			{
				  	 				if(bdtest_chgcurr > (LIBAT_CHARGECURRENT_SET - 100) && 
				  	 				bdtest_chgcurr < (LIBAT_CHARGECURRENT_SET + 100))//����������Ԥ��ֵ��Χ
				  	 				{
				  	 				#if BDTEST_LOGPRINT
										log_printf("CHG SUCCESS\r\n");
										delay_ms(50);
				  	 				#endif
				  	 					bdtest_chgsucceed_counter = bdtest_chgfailed_counter = 0;
				  	 				}
				  	 				else if(bdtest_chgcurr < (LIBAT_CHARGECURRENT_SET - 100))
				  	 				{
				  	 					if(bdtest_chgstart_counter ++ > BDTEST_CHG_START_MAX)
				  	 					{//�ܾö�û�дﵽԤ������,���ʧ��
							  	 			bd_batcharge_timer = 0;
											LiBat_ExitChargeMode();
											#if BDTEST_LOGPRINT
											bdtest_sta_child = 5;
											log_printf("BDTEST_CHG_FAIL_MAX\r\n");
											delay_ms(50);
											#else
											bdtest_sta = TPC_STOP_ALL;
											bdtest_sta_child = 0;
											#endif
											bdtest_fantest = 0;
											tx_com_dat(TPC_CHRG_TST, 100 , 
												bdtest_chgfailed_counter > BDTEST_CHG_FAIL_MAX ? TPC_ERR_BATCH:TPC_ERR_NOERROR, 0);
											bdtest_chgfailed_counter = 0xff;
											bdtest_chgsucceed_counter = 0xff;
											break;
				  	 					}
				  	 				}
				  	 			}

								if(bdtest_chgsucceed_counter != 0xff && bdtest_chgfailed_counter != 0xff)
								{
					  	 			if(bdtest_chgcurr < (LIBAT_CHARGECURRENT_SET - 100) || 
					  	 				bdtest_chgcurr > (LIBAT_CHARGECURRENT_SET + 100))//����������
					  	 			{
					  	 				bdtest_chgfailed_counter =  bdtest_chgfailed_counter > BDTEST_CHG_FAIL_MAX? (BDTEST_CHG_FAIL_MAX + 1):(bdtest_chgfailed_counter + 1);
					  	 				bdtest_chgsucceed_counter = 0;
					  	 			}
					  	 			else
					  	 			{
					  	 				bdtest_chgsucceed_counter =  bdtest_chgsucceed_counter > BDTEST_CHG_SUCC_MIN ? (BDTEST_CHG_SUCC_MIN + 1):(bdtest_chgsucceed_counter + 1);
					  	 			}
					  	 			
					  	 			if(bdtest_chgsucceed_counter > BDTEST_CHG_SUCC_MIN
					  	 				|| bdtest_chgfailed_counter > BDTEST_CHG_FAIL_MAX)
					  	 			{
						  	 			bd_batcharge_timer = 0;
										LiBat_ExitChargeMode();
										#if BDTEST_LOGPRINT
										log_printf("BDTEST_CHG_FAILED %d\r\n",bdtest_chgfailed_counter);
										delay_ms(50);
										bdtest_sta_child = 0;
										#else
										bdtest_sta = TPC_STOP_ALL;
										bdtest_sta_child = 0;
										#endif
										bdtest_fantest = 0;
										tx_com_dat(TPC_CHRG_TST, 100 , 
											bdtest_chgfailed_counter > BDTEST_CHG_FAIL_MAX ? TPC_ERR_BATCH:TPC_ERR_NOERROR, 1);
										bdtest_chgfailed_counter = 0xff;
										bdtest_chgsucceed_counter = 0xff;
										delay_ms(50);
										#if !BDTEST_LOGPRINT
										tx_com_dat(TPC_STOP_ALL, 100 , 0x00, 0);//�ϱ�����
										#endif
										break;
					  	 			}
				  	 			}
				  	 		}
				  	 	}
				  	 	else
				  	 	{
							//tx_com_dat(TPC_CHRG_TST, 40 , TPC_ERR_BATCH, 0);
							#if 1
							LiBat_ExitChargeMode();
							#if BDTEST_LOGPRINT
							bdtest_sta_child = 5;//0
							log_printf("\r\nLB_CHS_ERROR:%d\r\n",n_charge_res);
							//log_printf("\r\n ch:%d\r\n",n_charge_res);
							delay_ms(50);
							#else
							//tx_com_dat(TPC_STOP_ALL, 100 , 0x00, 0);//�ϱ�����
							bdtest_sta_child = 0;//0
							bdtest_sta = TPC_STOP_ALL;
							#endif
							#endif
							delay_ms(100);
				  	 	}
			  	 	}
					break;
				case 6:
					break;
				#if 1
				case 5:
					log_printf("^");
					delay_ms(100);
					break;
				#endif
				default:
					bdtest_sta = TPC_STOP_ALL;
					bdtest_sta_child = 0;
					MOTOR_POWER_OFF_NPRI();
					bdtest_fantest = 0;
					break;
			}
			break;
		default:
			bdtest_sta = TPC_STOP_ALL;
			break;
	}
//	bdtest_sta_old = bdtest_sta;
}




//��λ����ͨ�ų���
void proc_com_test_mode(uint8_t *buf,uint8_t len)
{
	TEST_PROC *proc_dat;

	proc_dat = (TEST_PROC *)buf;

	//log_printf("\r\n\r\n");
	//data_print(buf,len);
	switch(proc_dat->code)
	{
		case TPC_STOP_ALL://�˳�����ģʽ,ϵͳ����
			tx_msg(buf,sizeof(TEST_PROC));
			bdtest_sta = TPC_STOP_ALL;
			if(tm_mode == TM_MODE_BOARD)
				tx_dp_dat(TBD_STOP_ALL, 0, 0);
			TargetSysReset();
			break;
		case TPC_START_ALL://�������ģʽ
			//if(bdtest_sta == TPC_STOP_ALL)
			{
				bdtest_sta = TPC_START_ALL;
				tx_msg(buf,sizeof(TEST_PROC));
				if(tm_mode == TM_MODE_BOARD)
					tx_dp_dat(TBD_START_ALL, 0, 0);
			}
			break;
		case TPC_RESET://�������ģʽ
			tx_com_dat(TPC_RESET, 0 , TPC_ERR_NOERROR, 0);
			TargetSysReset();
			break;
		case TPC_IR_BOT://�Եغ���
			if(proc_dat->t_ret_value < 5)
				bdtest_sta_child = proc_dat->t_ret_value;
			tx_com_dat(TPC_IR_BOT, proc_dat->t_progress , TPC_ERR_NOERROR, proc_dat->t_ret_value);
			break;
		case TPC_CHRG_TST://������
			if(proc_dat->t_ret_value < 5)
				bdtest_sta_child = proc_dat->t_ret_value;
			tx_com_dat(TPC_IR_BOT, proc_dat->t_progress, TPC_ERR_NOERROR, proc_dat->t_ret_value);
			break;
		case TPC_SETSN://PC������д��SN��
			//delay_ms(100);
			if(len < 20)//������ݳ���С��20������������Ч����
			{
				#if BDTEST_LOGPRINT
				log_printf("error:data len = %d\r\n",len);
				#else
				tx_sn(TPC_SETSN,NULL);
				#endif
				break;
			}
			else
			{
				uint8_t n_chksum;
				n_chksum = get_chksum(buf, SNLENGTH + 1);
				if(n_chksum == buf[22])
				{
					memcpy(cfg->sn,buf+1,21);
					
					//log_printf("OK!\r\n");
					//sn_print(cfg->sn, 21);
					save_cfg();
					tx_sn(TPC_SETSN,cfg->sn);
					delay_ms(500);
					TargetSysReset();
				}
				else
				{
				#if BDTEST_LOGPRINT
					log_printf("\r\nerror:chksum = %02X-%02X\r\n",n_chksum,buf[22]);
					sn_print(buf + 1, 21);
					log_printf("\r\n");
				#else
					tx_sn(TPC_SETSN,NULL);
				#endif
					break;
				}
			}
			break;
		case TPC_GETSN://������PC���������SN��
			//ny3p_play(VOICE_DIDI);
			tx_sn(TPC_GETSN,cfg->sn);
			break;
		default:
			if(proc_dat->code >= TPC_CD_FRT && proc_dat->code <= TPC_MT_FAN)
			{
				bdtest_sta = proc_dat->code;
				bdtest_sta_child = 0;
				tx_msg(buf,sizeof(TEST_PROC));
				if(tm_mode == TM_MODE_BOARD)
					tx_dp_dat(TBD_START_ALL, 0, 0);
				if(proc_dat->code != TPC_CHRG_TST)
				{
					LiBat_ExitChargeMode();
					delay_ms(100);
					if(tm_mode == TM_MODE_BOARD)
						tx_dp_dat(TBD_CHRG_TST, 0, 0);//��24V��DC�����ӿ�
				}
			}
			break;
	}
}

//����԰�Ķ˿�(���Կض˿�)usart2��������
void tx_dp_msg(uint8_t *buff,int len)
{
	uint8_t tmp[2];

	//������֮ǰ�Ȱ�δ��������,���Ȿ�ζ�����һ�εı��
	tmd_rxbd_dat.unread_flag = 0;
	
	tmp[0] = 0x7E;
	tmp[1] = 0x5D;
	usart2_write(tmp,2);
	usart2_write(buff, len);
	tmp[0] = 0x6D;
	tmp[1] = 0x7D;
	usart2_write(tmp,2);
	
}


//����λ�������ϻ�����,�˺������ڻ����ϻ�������,����һֱ�����ر����ߣ����ϱ���������ײ���ݼ�һЩ���ϴ���
void tx_com_burnin(uint8_t side_dir, uint16_t timer,uint32_t bum_left_ct,uint32_t ir_left_ct,uint32_t bum_right_ct,uint32_t ir_right_ct)
{
	uint8_t i = 0;
	uint8_t tmp[2];
	TEST_BURNIN_PROC test_proc_dat;

	test_proc_dat.code = TPC_HEART_BEEP;
	test_proc_dat.para = side_dir;
	test_proc_dat.worktimer = timer;
	test_proc_dat.bum_ct[i ++] = bum_left_ct;
	test_proc_dat.bum_ct[i ++] = ir_left_ct;
	test_proc_dat.bum_ct[i ++] = bum_right_ct;
	test_proc_dat.bum_ct[i ++] = ir_right_ct;
	test_proc_dat.fw_ver = FW_VERSION;
	test_proc_dat.bat_volt= LiBat_GetBatVolt();
	test_proc_dat.errorcode = sys->work_errcode;
	
	
	tmp[0] = 0x7E;
	tmp[1] = 0x5D;
	uasrt_write(tmp,2);
	uasrt_write((uint8_t *)&test_proc_dat, sizeof(TEST_BURNIN_PROC));
	tmp[0] = 0x6D;
	tmp[1] = 0x7D;
	uasrt_write(tmp,2);	
}

//����λ��������������Ϣ
void tx_com_beep(uint8_t progress, uint8_t error_code, uint8_t ret_value,uint16_t *irvalue,uint16_t *irbuttom)
{
	uint8_t i;
	uint8_t tmp[2];
	TEST_BEEP_PROC test_proc_dat;

	test_proc_dat.code = TPC_HEART_BEEP;
	test_proc_dat.t_progress = progress;
	test_proc_dat.t_res = error_code;
	test_proc_dat.t_ret_value = ret_value;
	test_proc_dat.bat_volt= LiBat_GetBatVolt();
	test_proc_dat.fw_ver = FW_VERSION;
	if(EXTERAL_AC_DETECT())
	{
		test_proc_dat.bat_chstate = 1;
		test_proc_dat.bat_curr = LiBat_GetChargeCurrent();
	}
	else
	{
		test_proc_dat.bat_chstate = 0;
		test_proc_dat.bat_curr = 0;
	}
	for(i = 0;i < 7;i ++)
	{
		test_proc_dat.ir_frt_value[i] = irvalue[i];
	}
	for(;i < 10;i ++)
	{
		test_proc_dat.ir_frt_value[i] = irbuttom[i - 7];
	}

	
	tmp[0] = 0x7E;
	tmp[1] = 0x5D;
	uasrt_write(tmp,2);
	uasrt_write((uint8_t *)&test_proc_dat, sizeof(TEST_BEEP_PROC));
	tmp[0] = 0x6D;
	tmp[1] = 0x7D;
	uasrt_write(tmp,2);	
}

//����λ��������Ϣ
void tx_com_dat(uint8_t code, uint8_t progress, uint8_t error_code, uint8_t ret_value)
{
	uint8_t tmp[2];
	TEST_PROC test_proc_dat;

	test_proc_dat.code = code;
	test_proc_dat.t_progress = progress;
	test_proc_dat.t_res = error_code;
	test_proc_dat.t_ret_value = ret_value;
	//test_proc_dat.fw_ver = FW_VERSION;
	
	tmp[0] = 0x7E;
	tmp[1] = 0x5D;
	uasrt_write(tmp,2);
	uasrt_write((uint8_t *)&test_proc_dat, sizeof(TEST_PROC));
	tmp[0] = 0x6D;
	tmp[1] = 0x7D;
	uasrt_write(tmp,2);	
}

//����԰巢����Ϣ
void tx_dp_dat(uint8_t code, uint8_t para, uint8_t para1)
{
	uint8_t tmp[2];
	TEST_BD_PROC test_bd_dat;

	test_bd_dat.code = code;
	test_bd_dat.para = para;
	test_bd_dat.para1 = para1;
	test_bd_dat.t_res_input = 0;
	test_bd_dat.t_res_output = 0;

	//data_print((uint8_t *)&test_bd_dat, sizeof(TEST_BD_PROC));

	tmp[0] = 0x7E;
	tmp[1] = 0x5D;
	usart4_write(tmp,2);
	usart4_write((uint8_t *)&test_bd_dat, sizeof(TEST_BD_PROC));
	tmp[0] = 0x6D;
	tmp[1] = 0x7D;
	usart4_write(tmp,2);
}


//����԰巢����Ϣ,�����������PC��ת�����Թ�װ��,���������������������
void tx_dp_pc_dat(uint8_t code, uint8_t para, uint8_t para1)
{
	uint8_t tmp[2];
	TEST_BD_PROC test_bd_dat;

	test_bd_dat.code = code;
	test_bd_dat.para = para;
	test_bd_dat.para1 = para1;
	test_bd_dat.t_res_input = 0;
	test_bd_dat.t_res_output = 0;

	//data_print((uint8_t *)&test_bd_dat, sizeof(TEST_BD_PROC));

	tmp[0] = 0x7E;
	tmp[1] = 0x5D;
	uasrt_write(tmp,2);
	uasrt_write((uint8_t *)&test_bd_dat, sizeof(TEST_BD_PROC));
	tmp[0] = 0x6D;
	tmp[1] = 0x7D;
	uasrt_write(tmp,2);
}

uint8_t get_chksum(uint8_t *pdata, uint16_t length)
{
	uint16_t i;
	uint8_t ret = 0;

	for(i = 0;i < length;i ++)
	{
		ret += pdata[i];
	}
	return ret;
}

//��PC����SN,ʹ�õ�ָ����ΪTPC_GETSN �� TPC_SETSN
//���SNΪNULL����ʾ���͵�SN��Ч
void tx_sn(uint8_t code ,uint8_t *sn)
{
	uint8_t tmp[2];
	TEST_SN_PROC sn_proc;

	sn_proc.code = code;
	if(sn)
	{
		memcpy(sn_proc.sn,sn,SNLENGTH);
	}
	else
	{
		sn_proc.sn[0] = 0xff;
	}

	sn_proc.chksum = get_chksum((uint8_t *)&sn_proc, sizeof(TEST_SN_PROC) - 1);
	//data_print((uint8_t *)&sn_proc, sizeof(TEST_SN_PROC));

	tmp[0] = 0x7E;
	tmp[1] = 0x5D;
	uasrt_write(tmp,2);
	uasrt_write((uint8_t *)&sn_proc, sizeof(TEST_SN_PROC));
	tmp[0] = 0x6D;
	tmp[1] = 0x7D;
	uasrt_write(tmp,2);
}
#define DIS_S 50
void walk_tesk_for_whele(void)
{
	uint8_t err=0;
	sys->shut_down_motor = 1;		//�ص��
	navigat_init(0);				//��ʼ��
    navigat->angle = 0;
	sys->sState = SYS_NAVIGAT;		//����״̬������ֱ�к������˳�
	navigat->distance = motor.c_left_hw = motor.c_right_hw = 0;	//����
	motor_go_forwark(0,NO_SIDE_NEAR,NULL);	//ֱ��

	
	float dis = motor.c_left_hw * 0.87f;			//�����ǰ��ֵ
   
	if( disXY(navigat->distance,(int)dis) >= DIS_S)		//ǰ�ֺ����ֶԱ�
	{
		err = 1;
		log_printf( "Errdis0=%d,%d\r\n",navigat->distance,(int)dis);
	}
	if( disXY(motor.c_left_hw,motor.c_right_hw) >= DIS_S)	//���ֺ����ֶԱ�
	{
		err = 1;
		log_printf( "Errdis1=%d,%d\r\n",motor.c_left_hw,motor.c_right_hw);
	}
  
	turn_to_deg(180);	
	//delay_ms(800);//��ͷ180��
	 if(err == 1)
	{
		for(int i=0;i<20;i++)
		{
			ny3p_play(VOICE_DIDI);
			delay_ms(50);
		}

	}
	else
	{
     delay_ms(800);
	}
	navigat->distance = motor.c_left_hw = motor.c_right_hw = 0;
	
	motor_go_forwark(0,NO_SIDE_NEAR,NULL);	//ֱ��
	sys->sState = SYS_IDLE;
	 dis = motor.c_left_hw * 0.87f;
	if( disXY(navigat->distance,(int)dis) >= DIS_S)
	{
		err = 1;
		log_printf( "Errdis2=%d,%d\r\n",navigat->distance,(int)dis);
		}
	if( disXY(motor.c_left_hw,motor.c_right_hw) >= DIS_S)
	{
		err = 1;
     log_printf( "Errdis3=%d,%d\r\n",motor.c_left_hw,motor.c_right_hw);
     }
	//�������
	if(err == 1)
	{
		for(int i=0;i<40;i++)
		{
			ny3p_play(VOICE_DIDI);
			delay_ms(50);
		}

	}

	STOP_ALL_MOTOR();
}

