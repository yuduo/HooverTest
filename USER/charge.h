#ifndef CHARGE_H_
#define CHARGE_H_


#define NEAR_DOCK_STA		67

#if 0

#define CHARGE_PWM				630
#define DOCK_TURN_PWM			630		//转弯

#define DOCK_PWM_MDL			650		//找到中线
#define DOCK_PWM_GO				600		//行走
#else//---------------------------------------------add by wonton2004 2017.3.3
#define CHARGE_PWM				720//740//830
#define DOCK_TURN_PWM			760//800//780		//转弯

#define DOCK_PWM_MDL			740		//找到中线
#define DOCK_PWM_GO				720		//行走
#endif



#define CHARGE_ING		0x00	//找座子的的过程，
#define CHARGE_OK		0x01	//找到座子

#define DOCK_STA_NO		0x00
#define DOCK_STA_SIDE	0x01	//侧面过去
#define DOCK_STA_MID	0x02
#define DOCK_STA_TURN	0x03


typedef struct move_list{
	u8	sta;
	u8	dock_sta;
	u8	knock_back;		//后退
	u8	first_run;
	u8	dock_side;
	u16 find_side_flag ;
	u16	c_midle;		//?????òμ??D??μ?′?êy
	u16	c_mm;
	u16	c_edgeways;
	u16	c_lost;			//????ê§è￥?D??μ?′?êy
	u16	c_bum;			//??×2μ?′?êy
	u16 c_cycle;		//
	u16	walk_dist;
	u16	c_top_dock;		//顶灯计数
	u16 c_top_lost;		//顶灯丢失
	u32 t_turn;
	short c_side_walk;
	float out;
}ChargeInfo;

extern ChargeInfo charge_info;

extern u8 IrRevData[6];	
extern u8 IrData_bak[6];
u8 do_charging_work(ChargeInfo *Info);
void motor_turn_charge(uint8_t dir,int pwm,float agle);
void motor_fw_charge(void);
char motor_turn_check_midle(uint8_t dir,int pwm,float agle,uint8_t type,uint8_t c_m_f_out);
char dock_found_midle(uint8_t dir,float agle,uint8_t type,uint8_t c_m_f_out);
void init_charge(uint8_t sta);
void proc_charge_task(void);
u8 do_navi_charge(ChargeInfo *Info);
char dock_found_dc(float agle);
char dc_charge_ok(void);

#ifdef KAILY_ROUND_CC_V12
uint8_t get_dockmode_irsw(void);
void set_dockmode_irsw(uint8_t sw);
#endif
uint8_t dock_go_middle(int16_t by);
void charge_back_off(int hw);
uint16_t read_ir_dock_knk(void);
uint8_t motor_turn_check_nosignal(uint8_t dir,int pwm,int hw,char flage);


#endif
