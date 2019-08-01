#ifndef _DISP_H_
#define _DISP_H_
//串口通信协议

//接收缓存大小
#define DP_RXBUFFER_LEN	512

//键值宏
#define DP_KEY_PWR	TM_KEY_PWR//0x02
#define DP_KEY_MODE	TM_KEY_MODE//0x01
#define DP_KEY_PLAY	TM_KEY_PLAY//0x04


//#define PD_MSG_TIMER_SYNC		0x81
#define PD_MSG_WORK_STA		0x84
#define DP_MSG_BATY			0x85
#define DP_SYS_STA			0x83



#define ROBOT_STOP	0x00
#define ROBOT_WORK	0x01

//命令集
//bit7=1为主板发送的命令
//bit7=0为显控发送的命令

//按键事件(显控=>主板)
#define DP_CMD_INVALID		0x00	//无效命令
#define DP_CMD_KEY_POWER	0x01	//开关机按键,带参为1字节,0表示此键按下后为关机,1表示开机
#define DP_CMD_KEY_MODE		0x02	//模式切换按键,带参为机器此时的工作模式,及选项,共2字节,见MachineWorkMode&MachineWorkOptions
#define DP_CMD_KEY_PLAY		0x03	//执行或停止按键,带参为3字节,byte0:0表示此键按下后为停止,1表示执行当前的模式
																//byte1:当前的工作模式,见MachineWorkMode
																//byte2:当前的工作模式,见MachineWorkOptions
#define DP_CMD_KEY_MANUAL	0x04	//手动清扫模式,带参为1字节,byte0:0/1/2/3分别表示前/后/左/右	
#define DP_CMD_KEY_SETSCHEDULE	0x07	//设置预约时间,这里就用来放提示音,预约数据是存在显控板的,不用发到主控上
#define DP_CMD_VIOC			0x08	//静音
#define DP_CMD_KEY_DIDI		0x09	//让主板的喇叭叮一声,用于一些无指令按键的声音播放,提高用户体验	
#define DP_CMD_KEY_STATUS		0x0A	//显控下发摄像头或者其他一些板载设备的状态信息
#define DP_CMD_KEY_SELFTEST		0x0B	//触发整机自检命令

#define DP_CMD_KEY_WIFI		0x0C		//WIFI配网

//设置事件
#define DP_CMD_SET_TIME			0x81	//设置时间,带参3字节 0-2分别为 时(24小时格式)/分/星期(星期为位的格式,即bit0表周一,bit6表周日)
#define DP_CMD_SET_ORDERTIME	0x82	//设置预约时间,带参3字节,内容同DP_CMD_SET_TIME,星期位使能:如bit2=1表示周三需要清扫
#define DP_CMD_SET_STATUS		0x83	//发送机器信息(包括错误),带参2字节,byte0: 信息的内容,见MachineStatus
																		// byte1: 错误的内容,从1排到99,如果MachineStatus的error位为0,此参无效
#define DP_CMD_SET_MODE			0x84	//设置模式,带参2字节,见DP_CMD_KEY_MODE
#define DP_CMD_SET_BTRY			0x85	//发送电池电量,带参1字节,电量为0-100,单位为%


#define DP_PROTHEAD_1	0xaa
#define DP_PROTHEAD_2	0X99

//通信数据的总长度
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

//时间相关
typedef struct __dp_time{
	uint8_t hour;
	uint8_t minute;
	uint8_t week;//星期,以位区分,bit0表周一,bit1表周二,以此类推,设置时间时,week中只能有一个位置1,预约时,置1的位为需要清扫的星期
	uint8_t unused;//占空间用,防止内存溢出
}DP_PROTTIME;

//工作模式及选项
typedef struct __dp_workmode{
	uint8_t workmode;//工作模式,见MachineWorkMode
	uint8_t workoptions;//工作选项,高低门槛(拖地时退出)/预约/拖地清扫(抹布已装上才能转换到拖地清扫),见MachineWorkOptions
}DP_PROTWMODE;

//工作状态
typedef struct __dp_workstatus{
	uint8_t status;//工作状态的内容,见MachineStatus
	uint8_t errorcode;//错误代码,取值1-99,如果status的error位为0,此内容无效
}DP_PROTWSTATUS;

//电源(开关机)状态
typedef struct __dp_powerstatus{
	uint8_t power;//工作状态的内容,见MachineStatus
}DP_POWERSTATUS;

//电池电量值
typedef struct __dp_battery{
	uint8_t bat;//工作状态的内容,见MachineStatus
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
	uint8_t head1,head2;//包头 0xaa,0x99
	uint8_t chksum;//校验和,为此字节后的所有字节之和取低8位
	uint8_t cmd;//命令(不能与包头重码--0xaa 0x99)
	union __data_union data;//
}DP_PROTDAT;

enum __dp_parseerrors{
	DP_PARSE_NOERR = 0,
	DP_PARSE_LENTH,//数据长度错误
	DP_PARSE_CHKSUM,//校验错误
	DP_PARSE_NODAT//无有效数据,即未找到包头
};

void dp_sendkey(uint8_t keycode,uint8_t pwr, uint8_t workmode, uint8_t workoptions);
void dp_commrxtask(void);

uint8_t dp_weekbit2dec(uint8_t weekbit);
void dp_tx_data(uint8_t msg,uint8_t option1,uint8_t option2,uint8_t option3,uint8_t option4);

extern uint8_t dp_camera_error;

#endif
