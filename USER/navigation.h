#ifndef H_NAVIGAT_
#define H_NAVIGAT_

#define ROOMA_GYROONLY	1//�����ǰ汾�ĺ�,�˺�Ϊ��0ʱ��ʾΪ�������ǰ汾,��ӦPCBϵ�а��ΪRE811/RE810A Ӳ���汾��>=1.03
						//��RE810ȡ����������PD12,������������PA8 PA15 PB14,����LED PA11 PA12
						//�Կش��ڸ�ΪWIFIģ�鴮��,������9600bps

#define ROOMA_RELEASE				1		// ������ʽ�ͷŰ汾




#if ROOMA_RELEASE

#define EDGE_DRAW_MAP				0		//�ر߻���ͼ	micony2019-03-05
#define JUDGE_STOP_FRONT			1		//wy190615 ����ǰ���ж��Ƿ�ֹͣ������Ϊ1������Ϊ0
#define ADJ_END						1		//����ʹ��	
#define TURN_V_ADJ					1		
#define BOTTOM_IR_EN				1		//�Եغ���ʹ��
#define GYRO_MODUL					0		//���õ�����������ģ��
#define NEAR_DRAW_MAP				0		//�ӱ߻���ͼ
#define PRINTF_BY_BUFFER			0		//�ж�		��ʽΪ0
#define CALE_ADJ_RIGHT				1		//���ֽ�������
#define CALE_BY_FRONT				0
#define MOP_EN						0		//ʪ��ģʽ	��ʽΪ1
#define GYRO_ADJ_ONBOARD			0
#define LASER_IR					1		//����������
#define LASER_IR_ONLY_ONE			0		//ֻ��1��������

#define TURN_CHECK_BUM				0		//wy190528 ��תʱ������ײ�ж�

#define	ADJ_GYRO_IDX				1
                                             //micony20190306 �����ǲ��ܽ���0�ȷ����µ������ǳ�����Խ������µ������ǳ���Ϊ0
                                             
#define THE_ORIGINAL_VALUE          1        //��1,��3 ԭʼֵ   2019 03 11


#define DUST_BOX_CHECK				1		//���ϼ��
#define UNINTERRUPTED_CLEAN			0		//�������ɨ			��ʽΪ0
#define WALK_PRINTF					1		//��ӡ�߹���·
#define WALK_PRINTF_CALC			0
#define PRINTF_SCAN_END				0		//������ʱ���ӡɨ���
#define NEARWALL_PID				0			
#define CLEAN_OK_LOW_LEVEL			1		//�ͼ������ɨ���
#define Z_GO_NEAR_REAL	   			1		//�ӱ߲���
#define TARGE_RUN					0		
#define ADJ_V_90					0		
#define LINEX_EN					1		
#define WALK_1D5_HOUR				0		//ֻ��һ����Сʱ��
#define LAB_DEMO_VERSION			0		//------------------------  1ʵ������ʾ�汾   0���������汾

#if LAB_DEMO_VERSION				//ʵ���Ұ汾��У׼������
#define ADJ_GYRO					0		///У׼������					��ҪУ׼�����ǵ�ʱ����,����ҪУ׼�����ǵ�ʱ��Ϊ0
#else
#define ADJ_GYRO					1		///У׼������					��ҪУ׼�����ǵ�ʱ����,����ҪУ׼�����ǵ�ʱ��Ϊ0
#endif

#define MOTOER_SENSER_CHECK			0	///��ʽҪΪ1
#define LOCKED_MOTOR_PARA 			0		//��ʽҪΪ1   ������
#define JUDGE_PID_CTRL_WHEEL_STOP	1		//ͨ���ж�PID������HWֵ,�õ��������Ƿ���߱�־
#define MID_MOTOR_CUR_OVER_GO_ON_EN	1		//��ɨ������� ͣ��������
#define SIDE_MOTOR_CUR_OVER_GO_ON_EN	1		//��ˢ������� ͣ��������
#define TURN_TIMEOUT_EN				0		//��ת��ʱ�ж�  1ʹ��  0һֱ����ת
#define CONSUMABLES_CHECK_EN		1		//�Ĳĸ澯ʹ��



#if CONSUMABLES_CHECK_EN
#define DUST_BOX_FULL_CHECK			0		//�������ۻ�ʱ�����ʹ��   1���л�澯  0 ���в���澯
#define STRAINER_ALARM_CHECK		0
#define SIDE_RUSH_ALARM_CHECK		0
#define MAIN_RUSH_ALARM_CHECK		0

#endif

#define DP_PANEL_EN					4	//165�Կ�888ʹ��   	2:�Կذ汾��170�汾  3:�����ǰ屾��170�汾   4:�Կذ汾180�汾
#define NEARWALL_SCAN_OBST_RECORD	0		//1,�ر���ɨ���ϰ��㱻��¼��������,   0,����¼ 

#if LASER_IR_ONLY_ONE
#define TURN_90_V					0		//TURN_90_V��LASER_IR_ONLY_ONE״̬Ҫһ��
#else
#define TURN_90_V					0		//TURN_90_V��LASER_IR_ONLY_ONE״̬Ҫһ��
#endif

#define CAPACITY_4400_VER			1


#else

#define JUDGE_STOP_FRONT			0		//����ǰ���ж��Ƿ�ֹͣ��
#define ADJ_END						0		//����ʹ��	
#define BOTTOM_IR_EN				1		//�Եغ���ʹ��
#define GYRO_MODUL					0		//���õ�����������ģ��
#define NEAR_DRAW_MAP				0		//�ӱ߻���ͼ
#define PRINTF_BY_BUFFER			0		//�ж�			//��ʽΪ0�����Կ�����1
#define CALE_ADJ_RIGHT				1		//���ֽ�������
#define CALE_BY_FRONT				0
#define MOP_EN						0		//ʪ��ģʽ
#define GYRO_ADJ_ONBOARD			0
#define LASER_IR					1		//����������
#define LASER_IR_ONLY_ONE			0		//ֻ��1��������
//#define TURN_90_V					1
#define TURN_V_ADJ					0		

#define	ADJ_GYRO_IDX				1		//micony20190306 �����ǲ��ܽ���0�ȷ����µ������ǳ�����Խ������µ������ǳ���Ϊ0


#define DUST_BOX_CHECK				0		//���ϼ��
#define UNINTERRUPTED_CLEAN			0		//�������ɨ
#define WALK_PRINTF					1		//��ӡ�߹���·
#define WALK_PRINTF_CALC			0
#define PRINTF_SCAN_END				0		//������ʱ���ӡɨ���
#define NEARWALL_PID				0			
#define CLEAN_OK_LOW_LEVEL			1		//�ͼ������ɨ���
#define Z_GO_NEAR_REAL	   			1		//�ӱ߲���
#define TARGE_RUN					0
#define ADJ_V_90					0
#define LINEX_EN					1
#define MOTOER_SENSER_CHECK			0		///��ɨ����
#define LOCKED_MOTOR_PARA 			0		//��ʱ���ж��������Ƿ����
#define JUDGE_PID_CTRL_WHEEL_STOP	1		//ͨ���ж�PID������HWֵ,�õ��������Ƿ���߱�־

#define MID_MOTOR_CUR_OVER_GO_ON_EN	1		//��ɨ������� ͣ��������
#define SIDE_MOTOR_CUR_OVER_GO_ON_EN	1		//��ˢ������� ͣ��������
#define TURN_TIMEOUT_EN				0		//��ת��ʱ�ж�  1ʹ��  0һֱ����ת

#define DP_PANEL_EN					2		///1 165�Կ�888ʹ��   2 170

#if LASER_IR_ONLY_ONE
#define TURN_90_V					0		//TURN_90_V��LASER_IR_ONLY_ONE״̬Ҫһ��
#else
#define TURN_90_V					0		//TURN_90_V��LASER_IR_ONLY_ONE״̬Ҫһ��
#endif


#if JUDGE_PID_CTRL_WHEEL_STOP			//ͨ���ж�PID������HWֵ,�õ��������Ƿ���߱�־
#define GET_OUT_OF_TROUBLE_EN		0	//		Get out of trouble

#define GETOUT_TROUBLE_ANGLE   		44.59f
#define INTO_CHECK_SENSER_GETOUT_TROUBLE   0				///��serner����������
#else
#define GET_OUT_OF_TROUBLE_EN		0
#define GETOUT_TROUBLE_ANGLE   		44.59f
#define INTO_CHECK_SENSER_GETOUT_TROUBLE   0				///��serner����������

#endif

#endif

#define DOCK_SIGNAL_UNBLE			1						///1 ����+����һ���жϳ��������  0 �����ж� ��ʽΪ1

#if ROOMA_GYROONLY
#define WIFIAPP_EN					1			//��ʽΪ1

#if SYS_VER == VER_BIGWHITEMACH	|| VER_SMALLBLACKMACH || VER_WHITEBLUEMACH
#define ARC_Z_GO					0			//��ʽΪ1 /��������			״̬������ADJ_END	����һ��
#else
#define ARC_Z_GO					0///1			//��ʽΪ1 /��������			״̬������ADJ_END	����һ��
#endif

#else

#define WIFIAPP_EN 					0
#define ARC_Z_GO					0

#endif


#if SYS_VER == VER_BIGWHITEMACH	
#define BUM_STA_OLD					1		//�ϰ汾����ײ  ��ҡ��Ϊ1  ��ҡ��Ϊ0
#else
#define BUM_STA_OLD					0		//�ϰ汾����ײ  ��ҡ��Ϊ1  ��ҡ��Ϊ0
#endif
#define		FRONT_IR_VERSION_1		1		//����İ汾��		2018-07-11 ��1��Ϊ0
#define  PID_DEBUG					0		///PID����Ϊ1  ��ʽΪ0
#define  GYRO_TO_LIB				0

#define GYRO_VERSION				1			//�����ǰ汾---   0:EPSON	1:BOSIH BMI055
#define MAP_VERSION					1			//��ͼ�汾  ����Ϊ����0�޵�ͼ   ��ͼ��Ϊ1 ���԰汾 
#if MAP_VERSION
#define MAP_VERSION_V				3			//��ͼ�汾1 2    //2Ϊ��UI���Ʋ��Բ�Ʒ   //3�������ǵ�ͼ�汾
#define CURRENT_POINT_UNREPORT		0

#endif
#define LAGER_ROUND_MODE			1			//�ƴ�׮�ж�ģʽ

#define CALE_BY_RIGHT_STOP_EN		0			//HWֵ����,�Ƿ�ͣ���  1 ͣ�� 0 ��ͣ��


#define MAX_CLEAN_FRONT				2		//ǰ������3�����ɼ���
#define MAX_UNCLEAN_IN				4		//�����м���δ��ɨ���Ž�ȥ
#define CALE_BY_FRON				0

#define FORCE_SIDE_NAVI				0		//ǿ��ɨ��һ��
#define EXTEND_ROUTER				1
#define PARTITION					0		//������ɨ
#define ROBOT_TO_WHELE				1		//
#define NAVI_V_AJUST				0		//������ʱ���Ƿ�Ҫ����һ��
#define CALE_BY_BKHW				1		//���ּ�������
#define HW_AB_EN					0		//���ֻ���ʹ��AB�����
#define HW_AB_OLD					0		//���ֻ���AB�࣬���ʽSK1816Ϊ�ϰ汾���̣���ƬʽSK1816Ϊ�°汾���̣��¾����̵�AB�����෴�ģ���Ҫ����һ�� 1Ϊ������ 0Ϊ������
											//��Ҫע�����������Լܵ�����AB�������ϰ汾��ͬ���������������ʱ��Ҫ��������

#define BORD_FRONT_100				1		//20190307 ���Ϻ��칫�ҡ�2019��3�·ݣ�����������100��ǰײ������̫�࣬ǰ��������������⡣1Ϊ��ǰײ�汾��δ�������øð汾



#define NOT_TURN_DEG_V				0			
#define PRINTF_PWR_STA				1		//��ӡ��ѹ

//#define COOR_CALC_TEST				0		//����������

#define WHELE_RESV					1		//�����������Ƿ񷴵ģ��Ժ�ߵ�Ϊ����֮ǰ���Ƿ���

#define LASER_EN					0		//ǰ������汾
#define LASER_SIDE					0		//�������
//#define NEAR_DRAW_MAP				0		//�ӱ߻���ͼ
#define VERIFY_MAP					0		//�Ƿ�У���ͼ
#define WALK_NAV_FOR_AJD			1		//�����ߵ�ʱ�������ͼ
#define NAVI_WALK_NEAR_LINE			0		//�ӱ�ֻ��һ����
#define BORD_VERSION				3		// 3 ���°汾 ��2��һ�汾
#define ADJ_NAER_TURN_V				0		//ADJ�ӱߵ�ʱ��ת����ֱ��λ�ã��ú���д�ˣ�δ���ԣ��ȹر�
#define EN_GO_UNCLR					1		//�Ƿ�ȥδ��ɨ����
#define GYRO_SHORT_ORG				0		//ȡ������ԭʼ����
#define GYRO_OFSET_ONBORD			0		//�ڱ���ʵ��������ƫ�ƽ���

//#define PID_ANGLE_TEST				1		//PID�Ƕȵ���

#define LASER_DRAW			0		//���⻭ͼ

#define WALK_AJUST			0		//���ߵ�ʱ�������ͼ

#define NAVI_NEAR_LINE		0		//���������ӱ߻���

#define LASER_RED			0		//��⼤��
#define GYRO_LASER_V1		0		//�����ǣ�����汾1
#define GRYO_SPI  			1

#define TEST_VERSION		0		//����
#define TEST_FOR_LABO		0 		
#define LASER_FORMART_LINE	0		//�����ڵĲ��԰汾�����ڸ�ʽ���ϰ����һ��ֱ�ߡ������ڷ����ڵ���ʾ��
#define LASER_SCAN_TEST		0		//�����ڵ�ɨ�����


#define DELAY_MS(X)		delay_ms_sensers(X)

#define LASER_PWM			650		//������ת��ʱ����ٶ�

#define AJUST_NEAR_ANGLE	0
#define AJUST_VITAL_ANGLE	1
#define SIGE_UNCLAER		0		//���δ��ɨ����ķ�ʽ
#define BOARD_V1			0
#define ADJ_COR_WHELL		0		//���ֽ�������

#define OS_WINDWOS			0

#define EN_AJUST_BY_LINE	1		//�Ƿ�����Աߵ�ֱ�����жϱ�����ײʱֱ��
#define EN_SIDE_BUM			1		//���ܲ�����ײ���Ƿ����б��ȥ
#define NAVI_LINE_Y			0
#define LAND_IR_EN			1		//�Եغ����Ƿ������ã�Ҫ�µ�ĥ�߲�����

//micony20170726 ��ѹ����       13887

#if SYS_VER	 == VER_ROMMAT
#if CAPACITY_4400_VER
#define CHARGE_POWER		13700//13800//13800//13500
#define CHARGE_POWER_2		13300//12800
#define LOW_POWER			13000//12800

#else
#define CHARGE_POWER		13600//13800//13800//13500
#define CHARGE_POWER_2		13200//12800
#define LOW_POWER			12800//12800
#endif
#else
#define CHARGE_POWER		14100//13500
#define CHARGE_POWER_2		13500//12800
#define LOW_POWER			12800//12800
#endif

#define AJUST_NOW_OBST		0		//��ײ��ʱ�����Ƿ�Խ���ϰ���,���Ŀǰ�����⣬����ײ��ʱ�����ǽ��������������ҵ���������Y���90?270����ײ��ʱ�򣬾��������⡣

#define ANGLE_VALID()		((sys->angle <= 20 && sys->angle >=-20)	 ||		\
							 (sys->angle >= 70 && sys->angle <=110)  ||		\
							 (sys->angle >= 165 && sys->angle <=195) ||		\
							 (sys->angle >= 250 && sys->angle <=290) ||		\
							 (sys->angle >= 340 && sys->angle <=360))
							 

#if ADJ_END
#define AJUST_RUN			1		//������Ա�ʹ��

#define ANGLE_NEAR_RUN_ADJ	1		//�ر��ߵ�ʱ������Ƕ�
#define ANGLE_PEER			30		//�Ƕ�У׼�ı���
#define ANGLE_ADJ_DIS		20.0f

#define TARGER_REC_AJUST	0		//�����ﱻ���ǵĽ�����Ŀǰ���������⣬�������������඼����ɨ���򣬻������⣬Ŀǰ��û���ж�������ķ���
#define NAVI_TO_TARGET		1		//�Ƿ�Ҫ������������
#define FORMART_ADJ_RUN		1		//��ʽ���������߷�ʱ��X������ꡣ����Ƕȱ仯������X�����겻�ö���Ҫ��Ȼ����һ�����߳�����
#define ADJ_Y_BUM			0		//y����ײ��ʱ�򣬸������������		Ŀǰ��ʾ��ʱ�Ȳ�����
#define FORCE_AJD_LONG_TIME	0		//��ʱ���ߺ�ǿ��У׼һ��
#define TO_ADJ_BEFOR_NAV	0		//����ǰ���Ƿ���һ��������
#define NEAR_ADJ_NAV		1		//������Ľ���


#else
#define AJUST_RUN			0		//������Ա�ʹ��

#if ROOMA_GYROONLY
#define ANGLE_NEAR_RUN_ADJ	0		//�ر��ߵ�ʱ������Ƕ�
#else
#define ANGLE_NEAR_RUN_ADJ	0		//�ر��ߵ�ʱ������Ƕ�
#endif
#define ANGLE_PEER			80		//�Ƕ�У׼�ı���

#define ANGLE_ADJ_DIS		20.0f

#define TARGER_REC_AJUST	0		//�����ﱻ���ǵĽ�����Ŀǰ���������⣬�������������඼����ɨ���򣬻������⣬Ŀǰ��û���ж�������ķ���
#define NAVI_TO_TARGET		0		//�Ƿ�Ҫ������������
#define FORMART_ADJ_RUN		0		//��ʽ���������߷�ʱ��X������ꡣ����Ƕȱ仯������X�����겻�ö���Ҫ��Ȼ����һ�����߳�����
#define ADJ_Y_BUM			0		//y����ײ��ʱ�򣬸������������		Ŀǰ��ʾ��ʱ�Ȳ�����
#define FORCE_AJD_LONG_TIME	0		//��ʱ���ߺ�ǿ��У׼һ��
#define TO_ADJ_BEFOR_NAV	0		//����ǰ���Ƿ���һ��������
#define NEAR_ADJ_NAV		0		//������Ľ���

#endif


#define OFFSET_180_PID		(0)		//180���ߵ�ʱ��ƫ��һ�½Ƕȣ�����Y�ᵼ����׼

#define TURN_TO_DEG			2		//0 - ��ת ��1-��ת��2������ת

#define TURN_VERTIC_WALL	0		//��ǽת�Ƕȵ�ʱ���Ƿ�Ҫ��ֱת

#define PRINTF_SCAN			1
#define PRINTF_BUM_DIS		0		//��ӡ��ײʱ�ľ��������

#define NAVI_TYPE_CLEAN		0		//ǰ���Ѿ���ɨ���µĵ���
#define NAVI_TYPE_BUM		1		//��ײ���µĵ���
#define NAVI_TYPE_NJST		2		//������У׼

#define NO_ANGLE_V			1000		//���ǽǶ�


#define SCANE_XY(X,Y)		read_compress_mem(X, Y, navigat->scan)
#define OBST_XY(X,Y)		read_compress_mem(X, Y, navigat->obstacle)


#define BUM_NO				0x00
#define BUM_BUM				0x01
#define BUM_SIDE			0x02

#define ADJ_NAVI_NO_RUN		0x00	//�����ϵ������������������û����û������ȥ
#define ADJ_NAVI_RUN_OK		0x01	//���ϵ�������������������ҵ�����ȥ�������ɹ�
#define ADJ_NAVI_RUN_ERR	0x02	//���ϵ�������������������ҵ�����ȥ�������Ƿ񣬵����Ӷ���

#define NAV_ADJ_TYPE_NO		0x00
#define NAV_ADJ_FORCE_H		0x01	//ǿ��ȥ��һ�����ڶ�����
#define NAV_ADJ_FORCE_ALL	0x02	//ǿ��ȥ��ֻҪ�ܵ�����ȥ����ȥ��һ�㷢����������ʱ��Ҫǿ�ƽ���һ��
#define NAVA_ADJ_NEAR		0x03
#define NAV_ADJ_ANGLE		0x04		//ǿ��ȥ�ӱ߽����Ƕ�


#define NADJ_RET_NO  		0x00	//������У��ı�׼�����������λ�����һ����
#define NADJ_RET_BUM 		0x01	//������У��
#define NADJ_RET_LINE 		0x02	//һ����У��

#define GO_ADJ_ERROR	   	NADJ_RET_NO		//�⻬�ȴ���
#define GO_ADJ_OVER	   		NADJ_RET_NO		//��������
#define GO_ADJ_BUM	   		NADJ_RET_BUM		//��ײ
#define GO_ADJ_LOST	   		NADJ_RET_LINE		//�ӱ߶�ʧ

#if SYS_VER == VER_ROMMAT || VER_SMALLBLACKMACH || VER_BIGWHITEMACH || VER_WHITEBLUEMACH
#define WALL_LOST_DEG		90
#else
#define WALL_LOST_DEG		40

#endif

#define WALL_LOST_DEG2		15

#define GO_TYPE_SAVE	0x00
#define GO_TYPE_NO_SAVE	0x01

#define OUT_TYPE_Y				0x01		//�����͵ķ�ʽ
#define OUT_TYPE_SIDE			0x02		//����б��ȥ	
#define OUT_TYPE_Z				0x03		//��������
#define OUT_NEAR_BACK			0x04		//�ӱ߻�ȥ��δ��ɨ����
#define OUT_TARGET_RUN			0x05		//�������ӱ��߷�
#define OUT_TARGET_BACK			0x06

#define Y_ORG_OFFSET			0			//�����͵�ʱ�򲹳��Ļ�������Ҫ��Ȼ�����ͳ���Y�������ر��,0 �Ͳ�����

#define T_GO_FW_PWM			0
#define T_GO_NEAR_PWM		1
//#define T_GO_SLOW_PWM		2
#define T_GO_TURN_DEG		3

#if SYS_VER == VER_ROMMAT
#define FW_SLOW_PWM			780//sys->pwm//620
#define BACK_OFF_PWM		780//650//(sys->pwm+80)//cfg->go_back_pwm
#define GO_FORWARD_PWM		700//sys->pwm//cfg->go_forward_pwm
#define GO_ROUTINT_PWM		780//sys->pwm//cfg->go_route_pwm
#define TURN_DEG_PWM		780//sys->pwm
#define GO_NEAR_PWM			650
#define GO_NEAR_PWM_FAST	650

#elif SYS_VER == VER_KESMAN

#define FW_SLOW_PWM			580//sys->pwm//620
#define BACK_OFF_PWM		580//650//(sys->pwm+80)//cfg->go_back_pwm
#define GO_FORWARD_PWM		560//sys->pwm//cfg->go_forward_pwm
#define GO_ROUTINT_PWM		580//sys->pwm//cfg->go_route_pwm
#define TURN_DEG_PWM		600//sys->pwm
#define GO_NEAR_PWM			580
#define GO_NEAR_PWM_FAST	580

#elif SYS_VER == VER_SMALLBLACKMACH

#define FW_SLOW_PWM			780//sys->pwm//620
#define BACK_OFF_PWM		780//650//(sys->pwm+80)//cfg->go_back_pwm
#define GO_FORWARD_PWM		700//sys->pwm//cfg->go_forward_pwm
#define GO_ROUTINT_PWM		780//sys->pwm//cfg->go_route_pwm
#define TURN_DEG_PWM		780//sys->pwm
#define GO_NEAR_PWM			650
#define GO_NEAR_PWM_FAST	650

#elif SYS_VER == VER_BIGWHITEMACH
#define FW_SLOW_PWM			780//sys->pwm//620
#define BACK_OFF_PWM		780//650//(sys->pwm+80)//cfg->go_back_pwm
#define GO_FORWARD_PWM		700//sys->pwm//cfg->go_forward_pwm
#define GO_ROUTINT_PWM		780//sys->pwm//cfg->go_route_pwm
#define TURN_DEG_PWM		780//sys->pwm
#define GO_NEAR_PWM			650
#define GO_NEAR_PWM_FAST	650

#elif SYS_VER == VER_WHITEBLUEMACH
#define FW_SLOW_PWM			780//sys->pwm//620
#define BACK_OFF_PWM		780//650//(sys->pwm+80)//cfg->go_back_pwm
#define GO_FORWARD_PWM		700//sys->pwm//cfg->go_forward_pwm
#define GO_ROUTINT_PWM		780//sys->pwm//cfg->go_route_pwm
#define TURN_DEG_PWM		780//sys->pwm
#define GO_NEAR_PWM			650
#define GO_NEAR_PWM_FAST	650


#endif

/*
#define FW_SLOW_PWM			get_pwm(T_GO_FW_PWM)//780//sys->pwm//620
#define BACK_OFF_PWM		get_pwm(T_GO_FW_PWM)//780//650//(sys->pwm+80)//cfg->go_back_pwm
#define GO_FORWARD_PWM		get_pwm(T_GO_FW_PWM)//780//sys->pwm//cfg->go_forward_pwm
#define GO_ROUTINT_PWM		get_pwm(T_GO_FW_PWM)//780//sys->pwm//cfg->go_route_pwm
#define TURN_DEG_PWM		get_pwm(T_GO_TURN_DEG)//700//sys->pwm
#define GO_NEAR_PWM			get_pwm(T_GO_NEAR_PWM)//700
#define GO_NEAR_PWM_FAST	630
*/
#define WAIT_STOP_COUNT		150
#define WAIT_STOP_CNT		2000

#define IDR_PRINTF()	log_printf("    [%d,%d,%d]\r\n",LEFT_ADC(),MIDLE_ADC(),RIGHT_ADC())

#if (1)
#define BACK_HW		16
#else
#define BACK_HW		4
#endif
#if CALE_BY_BKHW		//���ּ���


#if SYS_VER == VER_ROMMAT
	#define CM_PER_HW		29//31			//ÿ30������һ����
	#define HW_HF_GRID		225
	#define HW_GRID			450//30	 15mһ��		//���ٸ�������һ��	 200,7mm����һ��
	#define INC_HW			600//(50*WHELE_HW)
	#define RADIUS			338.0f		// �뾶 ,������ֱ��226mm������Ľ��336.6���ң���������׼����ȥ340
#elif SYS_VER == VER_KESMAN

	#define HW_HF_GRID		600//225
	#define HW_GRID			1200//  //���ٸ�������һ��	 200,7mm����һ��
	#define INC_HW			1500//(50*WHELE_HW)
	#define RADIUS			786//638.0f		// �뾶 ,������ֱ��22CM (22 / 2)*58 = 
	#define CM_PER_HW		71//29//31			//ÿ30������һ����
#elif SYS_VER == VER_SMALLBLACKMACH

	#define CM_PER_HW		36		//-1
	#define RADIUS			347.0f	//-2
	#define INC_HW			670		//-3
	#define HW_HF_GRID		260		//-4
	#define HW_GRID 		521		//-5
#elif SYS_VER == VER_BIGWHITEMACH
	#define CM_PER_HW		37		//-1
	#define RADIUS			383.0f	//-2
	#define INC_HW			725 	//-3
	#define HW_HF_GRID		287 	//-4
	#define HW_GRID 		575 	//-5	
#elif SYS_VER == VER_WHITEBLUEMACH
	#define CM_PER_HW		53		//-1
	#define RADIUS			590.0f	//-2
	#define INC_HW			1033 	//-3
	#define HW_HF_GRID		442 	//-4
	#define HW_GRID 		883 	//-5	

#endif

//165�Կذ�(888)��ʹ�ܿ���,
//1ʱΪֻ���Կذ�,�ǻ���PCB V1.6�ĸ�װ�汾 165
//2ʱ�ǻ���PCB V1.7���°汾,��ɨ����ĵ������ΪAD���� 888
//3ʱΪ��888��PCB V1.7�汾,��װ���ǰ�����,����ΪAD����,������LED��V1.6��ͬ,ǰײ��̤���Ժ���������ؽ�Ҳ����ͬ
//4ʱ�ǻ���PCB V1.8�İ汾����V1.7���������ұ�ˢ����ADC����,ADC����ͨ��Ϊԭǰײ������2����2������2����2�ֱ�ΪIOģʽ��PE9��PE15
#ifndef DP_PANEL_EN
#define DP_PANEL_EN 1
#endif

#define LASER_RES				15		//�ֱ��ʣ�15mmһ��
#define ORG_AJUST_HW	1	

#define MAX_X		7000
#define MAX_Y		7000

#define RC_GRID		0			//25����ÿ����3������
#define WALK_DIST()     motor.c_left_hw//navigat->distance
//����
#else
#define HW_HF_GRID		225
#define HW_GRID			450//30	 15mһ��		//���ٸ�������һ��	 200,7mm����һ��
#define INC_HW			600//(50*WHELE_HW)
#define MAX_X			7000
#define MAX_Y			7000

#define WALK_DIST()     navigat->distance

#endif

#define save_line_xy(x,y)	
#define printf_clr_are()
#define	BLACK			0
#define	BLUE			0xAA0000
#define	GREEN			0x00AA00
#define	CYAN			0xAAAA00
#define	RED				0x0000AA
#define	MAGENTA			0xAA00AA
#define	BROWN			0x0055AA
#define	LIGHTGRAY		0xAAAAAA
#define	DARKGRAY		0x555555
#define	LIGHTBLUE		0xFF5555
#define	LIGHTGREEN		0x55FF55
#define	LIGHTCYAN		0xFFFF55
#define	LIGHTRED		0x5555FF
#define	LIGHTMAGENTA	0xFF55FF
#define	YELLOW			0x55FFFF
#define	WHITE			0xFFFFFF
#define draw_dot(X,Y,K)


#define D2R		0.0174533f

#define CROSS_XY_NAVI		0x00		//Խ��ȥ�ˣ���Ҫ����
#define CROSS_XY_OK			0x01		//������λ�����ٵ���
#define CROSS_XY_ERR		0x02		//��ͼ�����ˣ���Ҫ��������
#define CROSS_XY_POINT_OK	0x03		//���������ˡ�
//typedef 	 int 	ORG_T
//typedef 	short   MAP_T;
#define CHECK_NAVI_STA2()		{if(sys->sState !=SYS_NAVIGAT  && sys->sState != SYS_DOCK && sys->sState != SYS_FOCUS && sys->sState !=SYS_RANDOM) { motor_run(GO_STOP,0,0,0); STOP_ALL_MOTOR() ; log_printf("sta(%d) err ,stop all motor\r\n",sys->sState); return 0;}}
#define CHECK_NAVI_STA()		{if(sys->sState !=SYS_NAVIGAT  && sys->sState != SYS_DOCK && sys->sState != SYS_FOCUS && sys->sState !=SYS_RANDOM) { motor_run(GO_STOP,0,0,0); STOP_ALL_MOTOR() ; log_printf("sta(%d) err ,stop all motor\r\n",sys->sState); return ;}}
#define CHECK_NAVI_STA_RT(X)		{if(sys->sState !=SYS_NAVIGAT && sys->sState != SYS_DOCK && sys->sState != SYS_FOCUS && sys->sState !=SYS_RANDOM) { motor_run(GO_STOP,0,0,0); STOP_ALL_MOTOR() ;log_printf("sta(%d) err2,stop all motor\r\n",sys->sState);return (X);}}

#define CHECK_NAVI_STA_ONLY()		{if(sys->sState !=SYS_NAVIGAT) { motor_run(GO_STOP,0,0,0); /*STOP_ALL_MOTOR() */; return ;}} //����ͣ�����Ե�������س�������

#define CHECK_IDLE_STA()		{if(sys->sState == SYS_IDLE)return;}	
#define CHECK_IDLE_STA_RT(X)		{if(sys->sState == SYS_IDLE)return X;}						
/*

 ÿһ��(15cm)��Լ35�����������200�� 200 * 35 = 7000
*/
#define DONT_SEARCH_UNCLR	0x00		//������δ��ɨ����ֱ�ӵ�����Ŀ�ĵ�
#define SEARCH_UNCLR		0x01		//����δ��ɨ����Ȼ���ٵ�����ȥ

#define NO_SIDE			0x00
#define LEFT_SIDE		0x01
#define RIGHT_SIDE		0x02

#define ROALD_LEN		20
#define NAVI_LEN		20

#define TYPE_ENDPOINT	0x00	//�����յ�,�����յ�ʼ�ղ�ɾ
#define TYPE_TEMPOINT	0x01	//��ʱ�㣬���������У�����ײ����ɾ��
#define TYPE_DOCK_AJUST	0x02	//������
#define SIDE_BUM_LEN	3

#define UC_LEN		30
#define DOOR_LEN		20

#define CROSS_X			0x01
#define CROSS_Y			0x02

#define INC_RAISE		1
#define INC_FAILL		-1

#define AJUST_X			1
#define AJUST_Y			2
#define AJUST_MOVE		4		//�ƶ��ˡ�
//����״̬
#define NAV_STA_NORMORL			0	//���ڹ�����
#define NAV_STA_NAVIGATE		1	//����
#define NAV_STA_REVEV			2	//��������
#define NAV_STA_DOCK_AJUST		3	//90�ȡ�270�ȵĸ������ӵ����ߵ�У׼
//#define NAV_STA_ADJ				4	//�������߷�


#define NO_NEAR					0x00
#define NEAR_WALL_RIGHT			0x01
#define LOST_WALL_RIGHT			0x02

#define NEAR_WALL_LEFT			0x03
#define LOST_WALL_LEFT			0x04

#define NO_SIDE_NEAR		0
#define LEFT_SIDE_NEAR		1
#define RIGHT_SIDE_NEAR		2
#define NEAR_BY_IRDA		20


#define  MAX_OBST			1000			//miconydrawmap		1000	->10
#define  MAX_J_OBST			200

#define NAVIGAT_OK			0x01
#define NAVIGAT_ERR			0x02
#define NAVIGAT_AJUST		0x03
#define NAVIGAT_SERCH_ERR	0x04


#define WHELE_HW			9


#define		DOCK_ERR     	0		
#define 	DOCK_NO_FOUND	1
#define 	DOCK_OK			2
#define 	DOCK_TO_X		3		//����ָ���ĵ�����Xֵ

#define 	MAX_TARGET		50
#define		MAX_ADGE_LEN	1000


#define ADJ_FAIL  			0x01	//�������߷�ʧ��
#define ADJ_MID_BUM			0x02	//�м���ײ,100%У׼
#define ADJ_BUM				0x03	//�������1�������ڣ�100У׼,
#define ADJ_BUM_SIDE		0x04	//
#define ADJ_SIDE_LOST		0x05	//���涪ʧ,����������80%У׼

#define GO_NEAR_TYPE_NO 	   0x00 	   //��ͨ�ӱ�
#define GO_NEAR_TYPE_NAVI	   0x01 	   //�м�����ܵ������յ㣬���˳�ȥ����
#define GO_NEAR_TYPE_ADJ	   0x02			//adj�������ȥ
#define GO_NEAR_DRAW_MAP	   0x03		  	//�ӱ߻���ͼ
#define GO_NEAR_TYPE_DOCK	   0x04			//�س�


#define RET_NEAR_ERROR		0x00
#define RET_NEAR_OK			0x01
#define RET_NEAR_TIMEOUT	0x02
#define RET_NEAR_FITT		0x03	//��ϵ�ͼ�洢����
#define RET_NEAR_ADJ		0x04	//��ϵ�ͼ
#define RET_NEAR_HW_OVER	0x05	//�г�����

#define STORT_XY()		{bx = X_NOW;by = Y_NOW;}
	
	
#define Y_LENE		0X00	// X�᲻�䣬Y��һ����
#define X_LINE		0x01	//Y�᲻�䣬X��һ����

#define LINE_TYPE_BUM		0x00
#define LINE_TYPE_LOST		0x01

#define  NEAR_GO_CLEAN_IN_FRONT		0x10

#define X_NOW				navigat->tx
#define Y_NOW				navigat->ty
#define A_NOW				sys->angle

#define drow_point(X,Y,z)	

#define  FRONT_TO_ORG		1.2//1.17f			
#define  ORG_TO_FRONT		0.83//0.85f			

typedef struct obst_t
{
	short 		x;
	short 		y;
	int 		x_org;		//����汾����1mm����
	int 		y_org;		//����汾����1mm����

	short 		tx;
	short 		ty;
//	unsigned short	ir;		//����ĺ���	

}OBST;
#define MAX_LINEX		50
//һ��X�ߣ�
typedef struct linex_t
{
	signed short tx;
	signed short ty;
	signed short dir;
}LINEX;
//��Ե
typedef struct edge_t
{
	signed short 		x;		
	signed short 		y;
	int 		x_org;		//����汾����1mm����
	int 		y_org;		//����汾����1mm����
	
	/**/
	//unsigned short	ir;		//����ĺ���		��λ 1������ɨ��λ
	//int8_t		x_dir;
	//int8_t		y_dir;
	
}EDGE;


typedef struct tobst_t
{
	//��ʼ�ĵ�
	short bx;
	short by;
	//�����ĵ�
	short ex;
	short ey;
	//���Եĵ������
	//short tx;
	//short ty;
	//��ʼ�ͽ�β��ʱ���״̬
	unsigned char	bret;
	unsigned char eret;
	short 	x_dir;	//X�ķ�����0�ȷ�����adj��x_dir=1,Ҳ����˵�����ϰ����������ɨ��ķ����ϰ���>��ɨ�㣬��Ϊ1
	int 	x_org;
/*
	short x;
	short y;
	int x_org;
	int y_org;
	short x_dir;	//X,���ȥ�ķ��� �Ǹĵ�����ϰ���ķ��� (x-x_dir,y)�����ϰ���ĵ�
	short y_dir;
	unsigned char side;
	unsigned char sta;
	short idx;		//���
//	int dist;
	float  angle;
	*/
}TOBST;

typedef struct majust_t
{
	//��λ
	short x;
	short y;
	//ԭʼ����
	short x_org;
	short y_org;
	//������
	short x_end;
	short y_end;
	
	short x_begin;		
	short y_begin;
//	int dist;
	float  angle;
}MAJST;

#define  MAX_NAVI_ERR		200
#define  MAX_NAVI_REP		10

//�洢����ʧ�ܵĵ�
typedef struct navi_err_t
{
	signed short x;
	signed short y;
	signed short c;
}NVIER;

typedef struct near_wall_t
{
	//unsigned char 	sta;
	unsigned char		n_sta;		//�رߵ�״̬ 0 - ���رߣ�1-���رߣ�2-���ر�
	unsigned char		turn_dir;	//��ײʱ��ת��
	//unsigned char		side_sta;	//
	unsigned char		c_left;
	unsigned char 	c_right;
	unsigned char		c_adc;
	
//	unsigned short 	lost_dist;		//����ʧȥ���ٸ������󣬲�ת��
//	unsigned short 	c_near;		//�رߵĴ���
//	unsigned short 	c_lost;		//����ʧȥǽ��ʱ��
	//float		l_angle;	//��ʧʱ��ĽǶ�

	NEAR_PID 	*pid;
	
//	unsigned short	count;		//�����Ƕȵ��ۼƺ�
	//unsigned short	previ_adc;	//�ϴε�ADC
//	float		angle;
//	float		yaw;
	
}NEAR_WALL;
//һ���������
typedef struct point_t
{
	short x;
	short y;
	int x_org;
	int y_org;
	float angle;
}POINT;

struct laser_point_t
{
	short m_dist;
	float m_deg;
};
//
//��������
#if LASER_EN
#define MAX_LASER_SCAN		500
#else
#define MAX_LASER_SCAN 	1

#endif
typedef struct laser_t
{
	
	//�ϴβ��Ե����ݣ������������ε�����
	short 		last_x_org;
	short		last_y_org; 	//�ϴγ��ĵ�Բ������
	unsigned int 	last_timer;	//�ϴβ���ʱ��
	short 		last_dist;
	float		last_deg;

	short 		c_scan;
	short		c_dist;			//�Ƕȣ�����ļ���
	short		m_dist[MAX_LASER_SCAN];			//����
	float		s_angle[MAX_LASER_SCAN];	
} LASER;

#define MAX_IR_SCAN		100
typedef struct ir_scan_t
{
	int 		len;
	unsigned short	ir[MAX_IR_SCAN][7];
	float		angle[MAX_IR_SCAN][7];
}IR_SCAN;
#define ORG_SAVE_LEN		20
#define MAX_WALK_GRID		50 //30�׵����߾���

//ֱ�е�ʱ����ʱ�Ĳ����ϰ������ʱ�洢�����ڽ��� ��Լ�ķ�5K 14*14 =5600
#define MAX_SIDE_OBST		200
typedef struct side_obst_t
{
	unsigned short  len;
	OBST obst[MAX_SIDE_OBST];
}SIDEOBST;
#define MAX_Z_XLINE		200

/*
��ɨ��·
*/
typedef struct x_line_t
{
	unsigned short bx;
	unsigned short by;
	unsigned short ex;
	unsigned short idx;
	unsigned char	 clean;
	unsigned char  r;
	//struct x_line_t *prev;
	//unsigned short ey;
}XLINE_T;
//�������ͻ���ʱ�Ľ�����ͼ����
typedef struct xline_z_t
{
	unsigned short len;
	XLINE_T  xline[MAX_Z_XLINE];
}Z_XLINE;

//#define MAX_WALK_GRID		30

typedef struct func_arg_t
{
	unsigned char type;
	unsigned char	type1;
	unsigned char	arg8_1;
	signed short	arg16_1;
	signed short arg16_2;
}FUNC_ARG;

typedef struct xy_t
{
	signed short	x;
	signed short y;
}XY_T;


#define MAX_M_EDGE		500

typedef struct h_targe_t
{
	//��Ӧ��XY������Ŀ�ʼ����
	signed short bidx;
	signed short eidx;
	signed short	deg;
	//��Ӧ�Ŀ�ʼ��xline�����
	//signed short xline;
}TARGET_T;
#define MAX_TARGET_T		100

//��ͼ�ı�Ե
typedef struct m_edge_t
{
	unsigned short 	len;
	signed short 		tx[MAX_M_EDGE];		
	signed short 		ty[MAX_M_EDGE];
	signed short 		ox[MAX_M_EDGE];		//�ϰ���
	signed short 		oy[MAX_M_EDGE];		//�ϰ���
}MEDGE;
	


#define MAX_P_NAVI		100		//���洢10��������
typedef struct h_navigat_t
{
	unsigned int gun_cnt;
	unsigned char sta;			//����״̬
	unsigned char is_walk;		//�Ƿ����ƶ���ֻ�����ƶ���ʱ�򣬲ż�������ϵ
	unsigned char	navi_ok;		//��ɨ���
	unsigned char	adj_run;		//�������߷�
	unsigned char	next_side;		//�������һ������
	unsigned char need_back;		//��Ҫ180���ͷ
//	unsigned char navi_to_dock;
//	unsigned char	wet_sta;		//ʪ��ģʽ
	unsigned char clean_ok;		//��ɨ���
	unsigned char	need_adj_dock;	//��Ҫ�������ӣ������ӱȽ�Զ��Ҫ�����ˡ�
	unsigned char navi_to_dock;	//�����س��׮

	unsigned char	whele_stop;		//�����Ƿ�ֹͣ�ˣ�����ٸģ����ô������ˡ�

	unsigned char to_side;		//�����ĵķ���
	unsigned char	m_navi_run;		//������ȥ�󣬵�һ���ߣ���ʱ�����ϰ�����ҪУ׼
	unsigned char	force_z_go;		//ǿ�ƹ����͵Ĵ���

	unsigned char	sheepflod;		//��Ȧ��	
	unsigned char	rever_navi;
	unsigned char navi_near;		//�ӱ���

	unsigned char	c_org_navi;		//������ԭ�صĴ���

	unsigned char can_navi;			//�ܵ�����ȥ
	unsigned char navside;
	unsigned char backsta;		
	unsigned char	arc_z_go;			//���ߵ�ͷ
	

	unsigned short	c_reverse_go;		//�����˶�
	unsigned short	c_z_go;				//�����˶�
	signed short	first_bx;
	signed short	first_ex;
	unsigned char clean_right_record;		//2018-10-27 jzz ��¼�Ƿ���ɨ���Ҳ�

	
	
	
	//unsigned char right_side_cln;	//�ұ��Ƿ���ɨ�ˡ��ұ�ɨ����ٵ��������ȥ������û��δ��ɨ�ġ�
	//unsigned short	gSta;		//ֱ����ײ��״̬������ֱ�к�õ��ղ���ײ��״̬
	
	int x1_org,y1_org;		//��һ���ԭʼ����
	int	x_org,y_org;		//�����ԭʼ����
	int	k_x_org,k_y_org;		//�����ǰ��ԭʼ����
	
	int x_org_l,y_for_l;
	float x_org_f,y_org_f;		//����d����ϵ
	float x_org_r,y_org_r;		//���ּ����
	float x_org_t,y_org_t;		//ǰ�ּ��������
//	float x_orgf1,y_orgf1;		//���ּ���ĸ�������
	//float x_orgf_l,y_orgf_l;	//ǰ��ͣǮ������ʱ�̵����꣬���ڼ���ǰ���Ƿ�򻬶���
	//float x_orgf_pre,y_orgf_pre;	//��һʱ��
	//float x_orgf1,y_orgf1;		//���ּ���ĸ�������
//	int mx,my;				//���ּ��������
	signed short x,y;				//ת����������ϵ
	signed short	tx,ty;
	signed short kx,ky;			// ǰ������
	//int	txx,tyy;		//��ʱ���������
	signed short x_dock;				//��Ҫ�����ӵ�λ��
	signed short dock_x;		//���׮������
	signed short dock_y;
	int l_dist;				//��һ�����ߵľ���
	volatile int	distance;	//
	short	tox,toy;			//������ĵ�����ȥ�ĵص�
	signed short	mtox,mtoy;
	int	toAgl;				//���������Ӧ���ߵĵķ���0��180
	short toxx[4];			//����·��
	short toyy[4];			//����·��
	short x_road[ROALD_LEN];	//����·��
	short y_road[ROALD_LEN];
	short c_road;				//·��
	short	side_obst;		//�����Ƿ�Ҫ�����ϰ���
	short	c_side_obst;
	short x_avr[2];			//����X��Ľ�����
	short c_avr[2];
	short osbt_x;
	short osbt_y;
	short scan_xx;
	short mx,my;				//�ϵ������ĵ�
	short y_min_draw;
	short y_max_draw;

	signed short ntox;	//����Ҫ����ȥ�ĵ�ַ
	signed short ntoy;
	
	//�洢ԭʼ����Ľ���ǰ������
	/*
	short x_org_s[ORG_SAVE_LEN];
	short y_org_s[ORG_SAVE_LEN];
	*/
	short x_walk[MAX_WALK_GRID];		//���ߵ������X
	short y_walk[MAX_WALK_GRID];		//���ߵ������X
	
	//���׮��λ�á�
	short tx_dock[3];
	short ty_dock[3];
	
	short c_dock_l;		//������ߵĴ���
	short c_dock_m;		//�����м�Ĵ���
	short c_dock_r;		//�����ұߵĴ���
//	short org_save_len;
	short walk_len;						//���ߵ�������� 
	signed short dock_adj_y;			//��¼�ϴ��ó��׮�����ĵ�ַ�������ظ��Ľ���
	unsigned short	c_near_0;		//0�ȵ��ӱߴ���
	unsigned short	c_near_180;		//180�ȷ�����ӱߴ���
	unsigned short	c_v_go;			//��ֱ�������ߵľ���
	unsigned short	c_zgo_bum;		//�����ж��ٴι�����ɨ
	unsigned short	c_zgo_err;
	unsigned short	navi_xline_index;		//�������ĵ��XLINE
	signed short		c_target_go;
	//unsigned short 	mapedge_len;

#if TEST_FOR_LABO
	short c_side_obst;
#endif
	//int inc_wheel;		//���ֵ������ٶȡ�
	/*
	//int ajust_widows;		//������������
	short dir_0_bum_x[SIDE_BUM_LEN];	//0�ȷ������ײ
	short dir_0_bum_y[SIDE_BUM_LEN];	//0�ȷ������ײ
	short dir_0_bum_len;
	short dir_180_bum_x[SIDE_BUM_LEN];	//0�ȷ������ײ
	short dir_180_bum_y[SIDE_BUM_LEN];	//0�ȷ������ײ
	short dir_180_bum_len;
	*/
	short walk_dis;		//�ߵľ���
//	int	wheel_dir;		//���ַ���
	//int x_obst,y_obst;	//�ݴ���ϰ���
	unsigned char to_type[ROALD_LEN];		//�м䵼���������
//	int navi_x[NAVI_LEN];		//�洢�ĵ�����
//	int navi_y[NAVI_LEN];
	short tolen;				//��������·��
	//δ��ɨ����
	signed short 	uc_x[UC_LEN];
	signed short 	uc_y[UC_LEN];
	unsigned char 	uc_side[UC_LEN];
	unsigned short	uc_len;	
#if PARTITION
	signed short   m_by;		//���ο�ʼ��Y

#endif	

	short 	i_obst;
	short 	j_obst;			
	short		idx_adj;
	float		adj_x_org;		//�������߷���Y���ԭʼ���꣬ǿ��ִ��
	//short		lst_obst;		//��һ�����������ţ������жϱ����������Ƿ�ֵ��ȡ
	//ת��Ȧ�������ߵĴ���
	unsigned short 	c_turn;
	unsigned short		c_go;
	unsigned short	c_go_navi;
	short		c_go_force;
	short	draw_map_bx;
	unsigned short p_navi_len;		//�����ĵĴ���
	
	/*
	int	door_x[DOOR_LEN];
	int door_y[DOOR_LEN];
	int door_len;
	*/
	//int navilen;
	//int tox1,toy1;			//�м�����ʱȥ�ĵ㣬���ȴ洢�����㡣
	short	nextx,nexty;		//���������һ���ȥ�ĵص�
	short max_ty;
	short max_tx;
	short min_tx;
	short min_ty;
	short scan_x;			//����ɨ�����X��ĸ��������ڼ���������ϰ����ʱ��ֱ�ӵ������������߹�ȥ��
	short ajust_y;		//�洢��б��λ��
	float ajust_agle;

	float radius; 
	//int	before_uclr_y;	//����δ��ɨ����ǰ��y�ᱸ��
	unsigned char	adj_run_ret;		//�洢�������ر��ߵĽ��
	unsigned char side;
	unsigned char old_side;
	unsigned char side_sta;			//�ߵ�״̬��
	unsigned char	c_navi_bum_y;		//ͬһ��Y������ײ�Ĵ���
	unsigned char c_near;				//��ǽ�Ĵ�����Ҫ�����εġ�
	//unsigned char c_side_bum;
	//unsigned char routering;			//����·������
	unsigned char	ret_search;			//����·����� 1-������δ��ɨ����0���������м�����
	unsigned char backClean;
	unsigned char	navi_side;			//�����ķ���
	unsigned char	is_bum;				//ֱ�е�ʱ���Ƿ���ײ
	unsigned char c_side_bum;			//������ײ�Ĵ���
	unsigned char	bum_side;
	unsigned char first_bum;
//	unsigned char	ajust_run;			//У׼���߷���1-�ߣ�0-����
	unsigned char from_charge;		//�ӳ��������
	unsigned char	found_charge;		//���ֳ��׮
	unsigned char is_cross_obst;
	//У׼��Ѱ��������ı���
	unsigned char	adj_nsta;			//�رߵı�
	unsigned char	adj_c_lost;
	unsigned char c_navi_err;
	unsigned char c_navi_repeat;

	unsigned char	mside;
	
	unsigned char	force_obst;		//ǿ�����ϰ���
	unsigned char suspend;		//��ͣ���������ڴ档
	unsigned char draw_map_flage;	//�ӱ߻���ͼ�ı�־����־�Ƿ�ȥ��Y-�ķ���ȥ��Y-�ķ����ٻص�Y��ԭ�㣬���㻭ͼ���
	signed short	arg[4];			//�м�ı�����������ֱ�к�����ʱ���õ���
	
	short adj_walk_dis;		//�ر��ߵ�·��
	short walk_speed;			//�ƶ��ٶ�

	int   	draw_map_bk_angle;		//����ͼ��ʱ�򣬽���ʱ��ȥ�ĽǶ�
	short	draw_map_adj_x;			//����ͼ�������ϰ���

	unsigned short  c_z_bum;			//�����͵���ײ����
	unsigned short  c_m_bum;			//�м���ײ�Ĵ�����

	unsigned short	edge_len;
	unsigned short 	c_adj_navi_near;		//�����ӱ߻����Ĵ���
#if LINEX_EN
	signed short		c_linex;
#endif	

	unsigned int	t_navigat;		//������ʱ��
	unsigned int 	t_gyro_adj;		//��������������ʱ��
#if  TARGE_RUN
	signed short   	i_target;
#endif	
//	unsigned char	near_sta;			//�ر�ģʽ	
	//unsigned char c_snack;			//���εĴ���
	//unsigned char force_side;			//ǿ�����еķ������ڽ��뵽δ��ɨ����ʱ��ǿ�Ƶķ���		
//	unsigned char mask_side;
	float	to_angle;
	float	angle;			//�Ƕ�
	float	lst_angle;		//������й����ĽǶ�
	float  	out;
	float	dis_angle_adj;
	unsigned char obstacle;		
	unsigned char scan;		
	//unsigned char walkmap[MAX_GRID_LIN];			//ɨ���ͼ�ߵ�·��
	OBST 	m_obst[MAX_OBST];			// ����汾 12*1000 = 12K ��ͨ�汾  8 * 10000 = 8K
#if AJUST_RUN	
	//OBST adj_obst[MAX_J_OBST];
	TOBST	adj_obst[MAX_TARGET];		//������ 16K
	TOBST	*l_adj_obst;		//���ȥУ���������
#endif	
	NEAR_WALL	near;
	NVIER		e_navi[MAX_NAVI_ERR];		
	NVIER		r_navi[MAX_NAVI_REP];		//�ظ�������
//#if LASER_EN || LASER_SIDE
//	LASER		laser;
//#endif
	//IR_SCAN 	*ir_scan;
	POINT		poffset;	//������ʱ����ĵ��ֵ����У׼��ʱ�򣬱���һ�£����Ҽ����ƫ�����Ĳ�ֵ�����ڻع�
	SIDEOBST	*g_obst;	//��ϵ�ͼ��ʱ���ݴ���ϰ����ͼ

	FUNC_ARG	func_arg;		//����ʱ��Ĳ�����
#if  TARGE_RUN
	TARGET_T	target[MAX_TARGET_T];
#endif	
	
#if NEAR_DRAW_MAP	
	EDGE		edge[MAX_ADGE_LEN];
#endif
#if LINEX_EN
	LINEX		linex[MAX_LINEX];
#endif	
	
//	NVIER		p_navi[MAX_P_NAVI];		//��󼸸�������
	//unsigned char next_y[MAX_GRID];		//��һ��Y����ϰ���
}navigat_t;

typedef signed short (*m_go_func)(void);

extern navigat_t *navigat;
void navigat_init(char flage);
void coord_org2map(int x,int y,short *xx,short *yy);
void coord_map2org(short xx,short yy,int *x,int *y);
void proc_navigat_task(unsigned short sta);
void motor_turn_deg(char dir,int deg);
void motor_back_off(int hw);
char motor_go_forwark(int hw,unsigned char nsta,m_go_func m_go_check);

char go_to_unclr(char sta,short bx);
void save_map(char clac,char scan,char obst);
char check_back_clr(void);
char parallel_forwark_y(int hw1,int dir);
char search_unclear(short *tx,short *ty,float *agnle);
char search_route(short x,short y,short tox,short toy,short *nx,short *ny);
char always_wak_map_cord_y(short y,float angle,int dir);


void turn_to_deg(int deg);
char walk_map_cord(short x,short y,float angle,int bk_dis);
void cord_calc_store(char reset_dist);
char walk_org_cord(short x,short y,float angle,int bk_dis);
char navigat_robot(char type,char t_search,short tox,short toy,float toagle);
void delay_for_gyro(void);
unsigned char clean_sta(short x,short y);
char walk_map_line(short x,short y,float angle,int bk_dis);
int disxy(short x,short y);
char search_cross_y(short x,short y,short tox,short toy,short *nx,short *ny,char flage);
char osbt_sta_x(short y1,short x1,short y2);
char clean_in_front(int cnt);
char search_bum_route(void);
void save_navigat_point(void);
char osbt_sta_x_noscan(short x1,short y1,short x2);
char osbt_sta_y_noscan(short x1,short y1,short y2);
char search_cross_y_noscan(short x,short y,short tox,short toy,short *nx,short *ny);
char search_route_sub(short x,short y,short tox,short toy,short *nx,short *ny);
char judge_obst(unsigned short gsta,int hw);
char walk_for_navigat(void);
unsigned char proc_z_go_bum(unsigned short sta,short bx,short by);
char vertical_wall(unsigned char side_idx);
void insrt_vertical(unsigned short value);
char vertical_big_than(unsigned short min);
unsigned short get_adc(unsigned char side);
char router_p2p(short bx,short by,short tox,short toy,short *nextx,short *nexty);
char proc_walk_line_task(char sta);
void ajust_angle_after_near_pid(void);	
char motor_go_forwark_ajust(int hw);
char proc_navigat_bum(short tox,short toy,unsigned char unclr);
void set_osbt_side(void);
void navigat_near_turn(void);
char back_bum_unclr(void);
//char navigat_to_point(int tox,int toy,int toangle);
char search_near_clean(void);
void save_obs_side(char obst);
void sign_unclr(void);
void walk_for_calibration(float to_angle);
void navigat_calibration(void);
void init_navigate_gobal(void);
char robot_moves(int hw);
void set_osbt(short x,short y,unsigned char value,unsigned char type,char *str);
char proc_side_bum(int tdir);
char always_walk(short tox,short toy,unsigned char type,float angle,int dir,char turn,int c_snak);
void ajust_xy_map(short x,short y,unsigned char type);
void ajust_xy_obst(unsigned char adj,short bx,short by,int xdir);

void insert_osbt(short x,short y,unsigned short ir);
OBST *get_obst(short x,short y);
unsigned char robot_turn_deg_getout_trouble(unsigned char dir,int pwm,float agle);
unsigned char  robot_turn_deg(unsigned char dir,int pwm,float agle);
char motor_go_near_wall(short ty,unsigned char n_sta,unsigned char type,unsigned char t_out,int dir,int c_lost);
char nearwall_cross_map(int y,float angle,unsigned char  is_save,unsigned char nsta);
void init_near_wall_navi(unsigned char n_sta);
TOBST *get_adj_obst(short x,short y);
char osbt_sta_y(short x1,short y1,short y2);
char walk_for_dock_ajust(int y_org);
char walk_for_dock(short xx,short yy);
int unclear_round(short x,short y);
void go_near_wall(unsigned char side);
char navigat_to_target(unsigned char type);
int ajust_recover_obst(void);
signed short navi_go_forwark_check(void);
void ajust_y_on_move(void);
void printf_scan_obst(void);
int search_navi_err_table(short x,short y);
void calc_timr_upload(void);
NVIER * get_navi_err_stu(short x,short y);
char search_route_depth(short bx,short by,short tox,short toy,short *nx,short *ny);
char router_extend_begin(short bx,short by,short tox,short toy,short *nextx,short *nexty);


int check_now_point(short x,short y,int side_sta);
char navigat_to_point(short tox,short toy,float toagle);
char walk_map_cord_only(short x,short y,float angle,int bk_dis);
char motor_go_forwark_only(int hw,unsigned char nsta);


void save_router(void);
void load_router(void);
int check_adj_line(short nx,short ny,int x_dir,int y_dir,short *xx,short *yy);
int search_adj_line(short nx,short ny,short tox,short toy,short *xxo,short *yyo,short *ax);
int motor_go_for_adj(int hw,unsigned char nsta);
int navi_to_adj_near(short nx,short ny,short tox,short toy);
int search_adj_line_on_road(short nx,short ny,short tox,short toy,short *yyo);
char near_cross_xy(short tox,short toy,unsigned char ctype,unsigned char n_sta);
char motor_go_edgeways(unsigned char n_sta ,short tox,short toy,unsigned char type,unsigned char is_save);
int navi_to_new_adj(short bx,short by,short ex,short ey,short dist);
int check_y_line(short xx,short yy,short *rx,short *ry,short *ox,short *oy,short *deg);
int ajust_y_line(void);
void set_scan(short x,short y,unsigned char value);
void set_scan_point(short x,short y,int scan,int c_grid);

void insert_osbt_l(short x,short y,short x_org,short y_org,short tx_org,short ty_org);
void near_wall_test(void);
//int in_obst_f(struct obst_fifo_t *obst_f,int x,int y);
int ajust_xy_by_near_line(short bx,short by,short ex,short ey,float angle,unsigned char type,unsigned char n_sta);

int proc_walk_road(navigat_t *navi,short bx,short by,unsigned short gsta);

signed short proc_navigat_reverse(void);
unsigned char z_go_for_map(unsigned char side,signed short tx,signed short ty,unsigned short count);
unsigned char insert_navi_err_table(signed short x,signed short y,signed short count);
unsigned char check_xline_sta(signed short bx,signed short by,signed short ex);
unsigned char check_dock(signed short tx,signed short ty,float angle);

unsigned char proc_z_go_navi(unsigned char sta);

unsigned char adj_go_nearwall(signed short tx,signed short ty);
unsigned char i_go_for_navi(unsigned char side);
unsigned char proc_back_unclr(signed short bx,signed short ex);

unsigned char navi_go_dock_adj(unsigned char type);
unsigned char proc_side_near_wall(signed short bx,unsigned char side,float angle);

void init_focus_task(void);
int proc_focus_task(void);
unsigned char navi_to_dock(void);
void go_and_back_test(void);
signed short front_line_unlr(void);

unsigned char walk_to_unclr_line(signed short tox,signed short toy,unsigned char side);
char clean_front_by_walk(void);
unsigned char need_turn_back(signed short tx,signed short ty,float angle);
NVIER * get_navi_repeat_stu(short x,short y);
unsigned char insert_navi_repeat_table(signed short x,signed short y,signed short count);
void clear_navi_repeat_table(void);
int search_navi_repeat_table(short x,short y);
void nearwall_test_12(void);
unsigned char check_go_back(signed short x_dir);
signed short func_nearwall_back_org(void);
char search_unclear_only(void);
void navigat_reset(void);
unsigned char unclr_xline(XLINE_T *xline,signed short *tox,signed short *toy,unsigned char type);
char nearback_for_unclr(signed short bxx,signed short byy,unsigned char type);
char path_navigation(short tox,short toy);


#endif
