#ifndef __NY3P_H__
#define __NY3P_H__



//------------------------------------------modified by wonton2004 2016.12.28
#define NY3P_VOICE_TYPE 1
// ����оƬ�ͺ�
// 0��ԭ(����)����
// 1��������
#ifndef NY3P_VOICE_TYPE
#define NY3P_VOICE_TYPE 0
#endif
#if NY3P_VOICE_TYPE == 1
//������
#define VOICE_DIDI			2		//	��	
#define VOICE_DRIP			3		//ˮ����Ч
#define VOICE_SELECT_M		4		//ƻ��line��ʾ������	
#define VOICE_M_START		5		//	���ڿ�ʼ�Զ���ɨ	
#define VOICE_M_STOP		6		//	������ͣ��ɨ
#define VOICE_M_NAVI		7	//���ڿ�ʼ������ɨ
#define VOICE_M_MOP			8	//���ڿ�ʼʪ����ɨ
#define VOICE_M_EX_MOP		9	//�����л���ʪ��ģʽ	
#define VOICE_M_CLN_HEART	10	//	���ڿ�ʼ������ɨ
#define VOICE_M_NEAR		11	//	���ڿ�ʼ��ǽ��ɨ
#define VOICE_M_SPEED_MODE	12	//	ǿ��ģʽ����	
#define VOICE_MEN_CLEAN		13	//	���ڿ�ʼ�ֶ���ɨ
#define VOICE_MSCH_CLEAN	14	//  ���ڿ�ʼԤԼ��ɨ
#define VOICE_M_FINISHED	15	//	��ɨ�����
#define VOICE_M_Z_CLEAN		16	//	���ڿ�ʼ������ɨ(Zig-zag)
#define VOICE_M_ORDER		17	//	���ڽ���ԤԼģʽ
#define VOICE_M_ORDER_OUT	18	//	�˳�ԤԼģʽ
#define VOICE_M_ORDER_SAVE	19	//	ԤԼ����ɹ�		
#define VOICE_M_SLEEP		20	//	�����˽���˯��
#define VOICE_M_DOCK		21	//	���ڷ��س����
#define VOICE_CHARING		22	//	���ڿ�ʼ��ʼ���
#define VOICE_CHARGE_M		23	//	���ֶ����
#define VOICE_START_CHARGE 	24	//	�����Դ�����Ƿ��
#define VOICE_CHECK_BOX		25	//	����༯������������
#define VOICE_CHECK_M_MTR	26	//	������ɨ�Ƿ����
#define VOICE_BOX_OK		27	//	���鼯�����Ƿ�ŵ�λ
#define VOICE_CHECK_LEAVE	28	//	�����������Ƿ����
#define VOICE_CHECK_LINE	29		//	�����������Ƿ����
#define VOICE_CHECKLEAVE2	30	//	��������Ӧ���Ƿ���ػ���
#define VOICE_CHECK_LINE3	31	//	�����ɨ�Ƿ����
#define VOICE_CHECK_F_LINE	32	//	����ǰ���Ƿ����
#define VOICE_CHECK_M_LINE	33	//	������ɨ��ɨ�Ƿ����
#define VOICE_FAC_MODE		34	//	���ڽ��빤��ģʽ

//��Чָ��
#define VOICE_CHECK_BOX2	0	//	���鳾�и��Ƿ�ǵ�λ						
#define VOICE_M_SEFT_CHECK	0	//	�����˽����Լ�		
#define VOICE_OUT_SEFT_TEST	0	//	�������˳��Լ�						
//30	//	��һ����		
//31	//	�ڶ�����		
//32	//	��������	
#elif NY3P_VOICE_TYPE == 0
//������
#define VOICE_DIDI			3		//	��		
#define VOICE_SELECT_M		4		//��������2�������ѡ��ģʽ		
#define VOICE_M_START		5		//	�����˿�ʼ����		
#define VOICE_M_STOP		6		//	��������ͣ����		
#define VOICE_M_DOCK		7	//	�������Զ��س�		
#define VOICE_M_ORDER		8	//	������ԤԼģʽ		
#define VOICE_M_ORDER_OUT	9	//	�˳�ԤԼģʽ		
#define VOICE_M_ORDER_SAVE	10	//	ԤԼ����ɹ�		
#define VOICE_M_SLEEP		11	//	�����˽���˯��		
#define VOICE_M_CLN_HEART	12	//	�������ص���ɨ		
#define VOICE_M_NEAR		13	//	��������ǽ��ɨ		
#define VOICE_M_SPEED_MODE	14	//	�������ٶ�ģʽ		
#define VOICE_MEN_CLEAN		15	//	�������ֶ���ɨ		
#define VOICE_CHECK_BOX		16	//	����೾����������		
#define VOICE_CHECK_M_MTR	17	//	������ɨ�Ƿ����		
#define VOICE_CHECK_BOX2	18	//	���鳾�и��Ƿ�ǵ�λ		
#define VOICE_BOX_OK		19	//	���鳾���Ƿ�ŵ�λ		
#define VOICE_CHARING		20	//	��ʼ���		
#define VOICE_CHECK_LEAVE	21	//	�����������Ƿ����		
#define VOICE_CHECK_LINE	22		//	�����������Ƿ����		
#define VOICE_CHECKLEAVE2	23	//	����ؼ��Ƿ���ػ���		
#define VOICE_CHECK_LINE3	24	//	�����ɨ�Ƿ����		
#define VOICE_M_SEFT_CHECK	25	//	�����˽����Լ�		
#define VOICE_OUT_SEFT_TEST	26	//	�������˳��Լ�		
#define VOICE_M_Z_CLEAN		27	//	������Z����ɨ		
#define VOICE_CHARGE_M		28//	���ֶ����		
#define VOICE_CHECK_M_LINE	29	//	������ɨ��ɨ�Ƿ����		
//30	//	��һ����		
//31	//	�ڶ�����		
//32	//	��������		
#define VOICE_START_CHARGE 33	//	�뿪�����	
#else
//������
#define VOICE_DIDI			3		//	��		
#define VOICE_SELECT_M		4		//��������2�������ѡ��ģʽ		
#define VOICE_M_START		5		//	�����˿�ʼ����		
#define VOICE_M_STOP		6		//	��������ͣ����		
#define VOICE_M_DOCK		7	//	�������Զ��س�		
#define VOICE_M_ORDER		8	//	������ԤԼģʽ		
#define VOICE_M_ORDER_OUT	9	//	�˳�ԤԼģʽ		
#define VOICE_M_ORDER_SAVE	10	//	ԤԼ����ɹ�		
#define VOICE_M_SLEEP		11	//	�����˽���˯��		
#define VOICE_M_CLN_HEART	12	//	�������ص���ɨ		
#define VOICE_M_NEAR		13	//	��������ǽ��ɨ		
#define VOICE_M_SPEED_MODE	14	//	�������ٶ�ģʽ		
#define VOICE_MEN_CLEAN		15	//	�������ֶ���ɨ		
#define VOICE_CHECK_BOX		16	//	����೾����������		
#define VOICE_CHECK_M_MTR	17	//	������ɨ�Ƿ����		
#define VOICE_CHECK_BOX2	18	//	���鳾�и��Ƿ�ǵ�λ		
#define VOICE_BOX_OK		19	//	���鳾���Ƿ�ŵ�λ		
#define VOICE_CHARING		20	//	��ʼ���		
#define VOICE_CHECK_LEAVE	21	//	�����������Ƿ����		
#define VOICE_CHECK_LINE	22		//	�����������Ƿ����		
#define VOICE_CHECKLEAVE2	23	//	����ؼ��Ƿ���ػ���		
#define VOICE_CHECK_LINE3	24	//	�����ɨ�Ƿ����		
#define VOICE_M_SEFT_CHECK	25	//	�����˽����Լ�		
#define VOICE_OUT_SEFT_TEST	26	//	�������˳��Լ�		
#define VOICE_M_Z_CLEAN		27	//	������Z����ɨ		
#define VOICE_CHARGE_M		28//	���ֶ����		
#define VOICE_CHECK_M_LINE	29	//	������ɨ��ɨ�Ƿ����		
//30	//	��һ����		
//31	//	�ڶ�����		
//32	//	��������		
#define VOICE_START_CHARGE 33	//	�뿪�����	
#endif
//------------------------------------------------------------------------end

//PC13 DATA
//PC12 RESET

#if (1 == ENABLE_VOICE)
#define  NY3P_REST_PORT  GPIOC
#define  NY3P_DATA_PORT  GPIOC
#define  NY3P_REST_PIN	 GPIO_Pin_13  
#define  NY3P_DATA_PIN	 GPIO_Pin_12  

#define NY3P_REST(b)	GPIO_WriteBit(NY3P_REST_PORT, NY3P_REST_PIN, (BitAction)!!b)
#define NY3P_DATA(b)	GPIO_WriteBit(NY3P_DATA_PORT, NY3P_DATA_PIN, (BitAction)!!b)
#define NY3P_MUTE(b)	//GPIO_WriteBit(NY3P_POWER1_PORT, NY3P_POWER1_PIN, (BitAction)!b)
#define NY3P_POWER(b)	//GPIO_WriteBit(NY3P_POWER2_PORT, NY3P_POWER2_PIN, (BitAction)b)
#else
#define BEEP_ON()		{TIM2->CCR4 = 400;}
#define BEEP_OFF()		{TIM2->CCR4 = 0;}
#define BEEP_RING()		{BEEP_ON();delay_ms(500);BEEP_OFF();}
#endif

u16 ny3p_play(u8 num);
void ny3p_init(void);
u16 ny3p_volume_set(u8 num);

#define Voice_Init			ny3p_init
#define voice_output		ny3p_play
#define voice_volume_set	ny3p_volume_set

#endif
