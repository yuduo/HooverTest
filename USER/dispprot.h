#ifndef _DISP_H_
#define _DISP_H_
//����ͨ��Э��

//���ջ����С
#define DP_RXBUFFER_LEN	512

//��ֵ��
#define DP_KEY_PWR	TM_KEY_PWR//0x02
#define DP_KEY_MODE	TM_KEY_MODE//0x01
#define DP_KEY_PLAY	TM_KEY_PLAY//0x04


//#define PD_MSG_TIMER_SYNC		0x81
#define PD_MSG_WORK_STA		0x84
#define DP_MSG_BATY			0x85
#define DP_SYS_STA			0x83



#define ROBOT_STOP	0x00
#define ROBOT_WORK	0x01

//���
//bit7=1Ϊ���巢�͵�����
//bit7=0Ϊ�Կط��͵�����

//�����¼�(�Կ�=>����)
#define DP_CMD_INVALID		0x00	//��Ч����
#define DP_CMD_KEY_POWER	0x01	//���ػ�����,����Ϊ1�ֽ�,0��ʾ�˼����º�Ϊ�ػ�,1��ʾ����
#define DP_CMD_KEY_MODE		0x02	//ģʽ�л�����,����Ϊ������ʱ�Ĺ���ģʽ,��ѡ��,��2�ֽ�,��MachineWorkMode&MachineWorkOptions
#define DP_CMD_KEY_PLAY		0x03	//ִ�л�ֹͣ����,����Ϊ3�ֽ�,byte0:0��ʾ�˼����º�Ϊֹͣ,1��ʾִ�е�ǰ��ģʽ
																//byte1:��ǰ�Ĺ���ģʽ,��MachineWorkMode
																//byte2:��ǰ�Ĺ���ģʽ,��MachineWorkOptions
#define DP_CMD_KEY_MANUAL	0x04	//�ֶ���ɨģʽ,����Ϊ1�ֽ�,byte0:0/1/2/3�ֱ��ʾǰ/��/��/��	
#define DP_CMD_KEY_SETSCHEDULE	0x07	//����ԤԼʱ��,�������������ʾ��,ԤԼ�����Ǵ����Կذ��,���÷���������
#define DP_CMD_VIOC			0x08	//����
#define DP_CMD_KEY_DIDI		0x09	//����������ȶ�һ��,����һЩ��ָ�������������,����û�����	
#define DP_CMD_KEY_STATUS		0x0A	//�Կ��·�����ͷ��������һЩ�����豸��״̬��Ϣ
#define DP_CMD_KEY_SELFTEST		0x0B	//���������Լ�����

#define DP_CMD_KEY_WIFI		0x0C		//WIFI����

//�����¼�
#define DP_CMD_SET_TIME			0x81	//����ʱ��,����3�ֽ� 0-2�ֱ�Ϊ ʱ(24Сʱ��ʽ)/��/����(����Ϊλ�ĸ�ʽ,��bit0����һ,bit6������)
#define DP_CMD_SET_ORDERTIME	0x82	//����ԤԼʱ��,����3�ֽ�,����ͬDP_CMD_SET_TIME,����λʹ��:��bit2=1��ʾ������Ҫ��ɨ
#define DP_CMD_SET_STATUS		0x83	//���ͻ�����Ϣ(��������),����2�ֽ�,byte0: ��Ϣ������,��MachineStatus
																		// byte1: ���������,��1�ŵ�99,���MachineStatus��errorλΪ0,�˲���Ч
#define DP_CMD_SET_MODE			0x84	//����ģʽ,����2�ֽ�,��DP_CMD_KEY_MODE
#define DP_CMD_SET_BTRY			0x85	//���͵�ص���,����1�ֽ�,����Ϊ0-100,��λΪ%


#define DP_PROTHEAD_1	0xaa
#define DP_PROTHEAD_2	0X99

//ͨ�����ݵ��ܳ���
#define DP_PROT_COMMLENGTH	8
#define DP_PROT_DATALENGTH	5

typedef struct __dp_option{
	uint8_t opt1;
	uint8_t opt2;
	uint8_t opt3;
	uint8_t opt4;
	
}DP_OPINT;

typedef struct __dp_work{
	uint8_t go;
	uint8_t sta;
	uint8_t option;
}DP_WORK;

//ʱ�����
typedef struct __dp_time{
	uint8_t hour;
	uint8_t minute;
	uint8_t week;//����,��λ����,bit0����һ,bit1���ܶ�,�Դ�����,����ʱ��ʱ,week��ֻ����һ��λ��1,ԤԼʱ,��1��λΪ��Ҫ��ɨ������
	uint8_t unused;//ռ�ռ���,��ֹ�ڴ����
}DP_PROTTIME;

//����ģʽ��ѡ��
typedef struct __dp_workmode{
	uint8_t workmode;//����ģʽ,��MachineWorkMode
	uint8_t workoptions;//����ѡ��,�ߵ��ż�(�ϵ�ʱ�˳�)/ԤԼ/�ϵ���ɨ(Ĩ����װ�ϲ���ת�����ϵ���ɨ),��MachineWorkOptions
}DP_PROTWMODE;

//����״̬
typedef struct __dp_workstatus{
	uint8_t status;//����״̬������,��MachineStatus
	uint8_t errorcode;//�������,ȡֵ1-99,���status��errorλΪ0,��������Ч
}DP_PROTWSTATUS;

//��Դ(���ػ�)״̬
typedef struct __dp_powerstatus{
	uint8_t power;//����״̬������,��MachineStatus
}DP_POWERSTATUS;

//��ص���ֵ
typedef struct __dp_battery{
	uint8_t bat;//����״̬������,��MachineStatus
}DP_BATTERY;

union __data_union
{
	DP_PROTTIME timedata;
	DP_PROTWMODE workmode;
	DP_PROTWSTATUS morkstatus;
	DP_POWERSTATUS powerstatus;
	DP_BATTERY battery;
	DP_WORK	   work;
	DP_OPINT	opt;
};

typedef struct __dp_protdata{
	uint8_t head1,head2;//��ͷ 0xaa,0x99
	uint8_t chksum;//У���,Ϊ���ֽں�������ֽ�֮��ȡ��8λ
	uint8_t cmd;//����(�������ͷ����--0xaa 0x99)
	union __data_union data;//
}DP_PROTDAT;

enum __dp_parseerrors{
	DP_PARSE_NOERR = 0,
	DP_PARSE_LENTH,//���ݳ��ȴ���
	DP_PARSE_CHKSUM,//У�����
	DP_PARSE_NODAT//����Ч����,��δ�ҵ���ͷ
};

void dp_sendkey(uint8_t keycode,uint8_t pwr, uint8_t workmode, uint8_t workoptions);
void dp_commrxtask(void);

uint8_t dp_weekbit2dec(uint8_t weekbit);
void dp_tx_data(uint8_t msg,uint8_t option1,uint8_t option2,uint8_t option3,uint8_t option4);

extern uint8_t dp_camera_error;

#endif
