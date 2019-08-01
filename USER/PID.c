
#include "sys.h"

char pid_printf=0;


int round_pid_calce(pid_t *pid,float set_speed,float now_speed)
{
	float incrementSpeed;
	float	kp,kd,ki;
//	float max,min;
	pid->SetSpeed	= set_speed;
	pid->ActualSpeed = now_speed;
#if R_PID_DEBUG	
	log_printf("%3.1f,",now_speed);
#endif	
	pid->err			= pid->SetSpeed-pid->ActualSpeed;

	pid->integral +=pid->err;
	kp = pid->Kp * pid->err;
	kd = pid->Kd * (pid->err-pid->err_next);
	ki = pid->Ki * pid->integral;

	incrementSpeed	=  kp+ ki + kd;
#if R_PID_DEBUG	
	log_printf("%3.1f,%3.1f,%3.1f,%3.1f\r\n ",kp,ki,kd,incrementSpeed);
#endif	

	if(incrementSpeed < -300)
		incrementSpeed = -300;
	if(incrementSpeed > 300)
		incrementSpeed = 300;
	pid->out			+= incrementSpeed;
	sys->lpwm = sys->pwm  + incrementSpeed;
	sys->rpwm = sys->pwm  - incrementSpeed;
	if(sys->lpwm >=1000)
		sys->lpwm = 999;
	if(sys->lpwm <=50)
		sys->lpwm =50;

	if(sys->rpwm >=1000)
		sys->rpwm = 999;
	if(sys->rpwm <=50)
		sys->rpwm =50;
#if R_PID_DEBUG		
//	log_printf(",%5.1f\r\n",pid->out);
#endif	
	if(pid->out >=1000)
		pid->out=999;
	if(pid->out < 30)
		pid->out=30;
	pid->err_last	= pid->err_next;
	pid->err_next	= pid->err;
//	log_printf("%3.1f,%3.1f\r\n",now_speed,incrementSpeed);
	return (int)pid->out;
}



#define AJUST_AGLE   	0.2
#define AJUST_AGLE_2	0.2
#define AJUST_AGLE_BIG   	0.5

#define AJUST_AGLE_LOST	2.0f

#define OUT_IDR			950
#define MAX_IN_1		3500
#define MAX_IN_2		2100


#define MAX_NEAR_COUNT	3



float angle_buf[]={0.0f,90.0f,180.0f,270.0f};
uint16_t bf_angle[]={0,90,180,270};


NEAR_PID  r_near_pid,l_near_pid;
/*
level 延边等级，等级越低，越严，越高，越宽
level =0 工字型延边
level =1 adj延边
*/
	/*

#define MAX_N_ANGLE		140
#define MAX_N_DIST		1800
#define ANGLE_ADJ_PST	20

*/
#if KWS_VERSION
#define MAX_N_ANGLE		100				///延边次数
#define MAX_N_DIST		2600			///延边矫正距离  KWS:2500    正常的1200    2800
#define ANGLE_ADJ_PST	20
#define MAX_N_SMALL_DIST		450		///延边期间最小的矫正距离

#else
#define MAX_N_ANGLE		100
#define MAX_N_DIST		1200			///延边矫正距离  KWS:2500    正常的1200    
#define ANGLE_ADJ_PST	25
#define MAX_N_SMALL_DIST		200		///延边期间最小的矫正距离

#endif

float 		near_angle[MAX_N_ANGLE];
uint16_t	c_near_a=0;
//uint16_t 	c_near_a;	

#define ADJ_ANGLE_P		0.3f
void init_near_pid(char level)
{
	//延边角度初始化
	uint16_t i;
	for(i=0;i<MAX_N_ANGLE;i++)
		near_angle[i] = 1000;
	//c_near_a = 0;
	//log_printf("K1\r\n");
//右边延边PID 
	if(level == 0)
	{
		r_near_pid.min_adc 	= IRD_RIGHT_PID_MIN;
		r_near_pid.max_adc 	= IRD_RIGHT_PID_MAX;
		r_near_pid.max_adc2 = IRD_RIGHT_PID_MAX2;
		r_near_pid.lock_adc = IRD_RIGHT_PID_LOCK;

		//左边延边PID
		l_near_pid.min_adc 	= IRD_LEFT_PID_MIN;
		l_near_pid.max_adc 	= IRD_LEFT_PID_MAX;
		l_near_pid.max_adc2 = IRD_LEFT_PID_MAX2;
		l_near_pid.lock_adc = IRD_LEFT_PID_LOCK;
		
	}else if(level == 1)
	{
		r_near_pid.min_adc 	= 90;
		r_near_pid.max_adc 	= IRD_RIGHT_PID_MAX;
		r_near_pid.max_adc2 = IRD_RIGHT_PID_MAX2;
		r_near_pid.lock_adc = IRD_RIGHT_PID_LOCK;

		//左边延边PID
		l_near_pid.min_adc 	= 90;
		l_near_pid.max_adc 	= IRD_LEFT_PID_MAX;
		l_near_pid.max_adc2 = IRD_LEFT_PID_MAX2;
		l_near_pid.lock_adc = IRD_LEFT_PID_LOCK;
	}
		
	r_near_pid.adc  = &sys->g_sta[6];		//主红外
	r_near_pid.adc2 = &sys->g_sta[5];		//侧面第二个红外
	
	r_near_pid.a_go_outside = 0.3f;	//往外走的角度
	r_near_pid.a_go_inside  = -0.3f;	//往里走的角度

	r_near_pid.l_angle = 0;	//丢失时候的角度
	r_near_pid.count  = 0;
	r_near_pid.c_near =0;		//沿边的次数
	r_near_pid.c_lost =0;		//计算失去墙的时间
	r_near_pid.c_dis_a = 0;
	r_near_pid.last_adc= 0;
	r_near_pid.level = level;
	r_near_pid.obst		=	0;
	r_near_pid.max_lost = 0;
	r_near_pid.c_angle = 0;
	r_near_pid.real_lock = r_near_pid.real_max = 0;
	r_near_pid.c_big_change = 0;
	r_near_pid.dist = 0;
	r_near_pid.dist_ok = 0;

	r_near_pid.small_dist_ok = 0;		////
	r_near_pid.small_dist = 0;
	r_near_pid.c_lost_flag =0;	////add 2018-11-03

	//左边延边PID

	l_near_pid.adc  = &sys->g_sta[0];		//主红外
	l_near_pid.adc2 = &sys->g_sta[1];		//侧面第二个红外
	
	l_near_pid.a_go_outside = -0.3f;	//往外走的角度
	l_near_pid.a_go_inside  = 0.3f;	//往里走的角度

	l_near_pid.l_angle = 0;	//丢失时候的角度
	l_near_pid.count  = 0;
	l_near_pid.c_near =0;		//沿边的次数
	l_near_pid.c_near2 =0;		//沿边的次数
	l_near_pid.c_lost =0;		//计算失去墙的时间

	l_near_pid.c_dis_a = 0;
	l_near_pid.last_adc= 0;
	l_near_pid.level = level;

	l_near_pid.obst		=	0;
	l_near_pid.max_lost = 0;
	l_near_pid.c_angle = 0;
	l_near_pid.c_big_change = 0;
	l_near_pid.dist = 0;
	l_near_pid.dist_ok = 0;

	l_near_pid.small_dist_ok = 0;			//////
	l_near_pid.small_dist = 0;
	l_near_pid.c_lost_flag =0;			////add 2018-11-03

	l_near_pid.real_lock = l_near_pid.real_max = 0;

	//
	sys->c_near = 0;
	sys->i_c_near = 0;
	sys->dist_ok = 0;
	sys->small_dist_ok = 0;

	sys->setminpwm = 300;//cfg->setminpwm;			//设置为非强制延边做小PWM值
	sys->setmaxpwm = 800;//cfg->setmaxpwm;			//设置为非强制延边做大PWM值	
	log_printf("setminpwm=%d,setminpwm:%d\r\n",sys->setminpwm ,sys->setmaxpwm );
}


/*
根据障碍物做PID。
1.侧面有障碍物，则丢失的时候，就不能再做丢失的PID了。
*/
int pid_obst_around(navigat_t *navi,float angle)
{
/**/
	NEAR_WALL *near;
	NEAR_PID  *npid;
	
	int x,y;
	static int lx=0,ly=0;
	
	near=&navi->near;
	npid=near->pid;
	
	//简单一点，周边有障碍物，就不能取消延边
	if(lx == X_NOW && ly == Y_NOW)
		return 0;
	lx = X_NOW;
	ly = Y_NOW;
	for(x=X_NOW-2;x<=X_NOW+2;x++)
	{
		for(y = Y_NOW -2;y<=Y_NOW+2;y++)
		{
				if(OBST_XY(x,y))
				{
					//短距离内，有障碍物，则不延边
					
						log_printf("nobst(%d,%d,)\r\n",x,y);
						npid->obst = 1;		//取消延边
						return 0;
				}
	
			}
		
	}
	return 0;
	
}


char *strl="l";
char *strr="r";


#define NEAR_PID_DEBUG		0
#define NEAR_PID_OFSET		200

//沿墙的PID
/*
nsta : 	0 - 不强制延边  1--强制延边
*/
int near_wall_pid(uint8_t nsta,NEAR_PID *npid,float *agle)
{
#if ANGLE_NEAR_RUN_ADJ	
	static int dist=0;
#endif	
#if NEAR_PID_DEBUG	
	char *sStr;
	
	if(npid == &l_near_pid)
			sStr = strl;
		else
			sStr = strr;
#endif	
	
	if( *npid->adc > npid->min_adc && npid->c_lost_flag==0) // 有信号
	{
#if NEAR_PID_DEBUG	
		sys->near_debug=1;
		if(sys->near_debug)
			log_printf("N:%3.1f,%3.1f,%d,%d\r\n",sys->angle,*agle,*npid->adc,npid->c_near);
#endif		
		//计数靠近边的次数，如果不是强制延边， 靠近边超过一次次数，才做延边
		if(npid->level > 0) //如果是adj延边等等级，就直接进来
			npid->count = 100;
		else if(*npid->adc >= npid->max_adc || *npid->adc2 >= npid->max_adc2)
				npid->count = MAX_NEAR_COUNT+1;
		
		if(*npid->adc >= npid->lock_adc  ) 
		{
			if(npid->count < 200)
				npid->count++;
		}
		//不是真正的沿边，不做沿边，否则有问题
		if(nsta == 0 && npid->count < MAX_NEAR_COUNT)
		{
			*agle = navigat->angle;
#if NEAR_PID_DEBUG		
			if(sys->near_debug)
				log_printf("out1,%d,%d,%d,%d\r\n",*npid->adc,npid->count,npid->lock_adc);
#endif			
			return 0;
		}
		/*
		if(disfloat(*agle,sys->angle) >=10)
		{
#if NEAR_PID_DEBUG		
			if(sys->near_debug)
				log_printf("angleerr(%3.1f,%d3.1f)\r\n",*npid->adc,npid->count,npid->lock_adc);
#endif		
			*agle = sys->angle;
		}
		*/
		npid->c_lost = 0;	
		if(npid->last_adc ==0)
		{
			npid->last_adc = *npid->adc;
			npid->c_dis_a=0;
		}
		if(*npid->adc >= (npid->lock_adc+100) && *npid->adc <= (npid->max_adc-100))
		{
			//每隔40次，计算一下变化的趋势
			//if(npid->c_dis_a++>=8)
			if(npid->c_dis_a++>=cfg->c_pid_near)// 8 -> 12 modified by wonton2004 20170602 量产的红外EVA值修改
			{
//				float f;
				
#if NEAR_PID_DEBUG		
				if(sys->near_debug)				
					log_printf("[%d,%d]\r\n",npid->c_dis_a , (short)(*npid->adc - npid->last_adc));
#endif				
				npid->c_dis_a=0;
				//变大了100，往外拐
				//if(npid->last_adc > *npid->adc && (npid->last_adc - *npid->adc) > 200)
				if(npid->last_adc > *npid->adc && (npid->last_adc - *npid->adc) > cfg->a_pid_near)// 200 -> 40 modified by wonton2004 20170602 量产的红外EVA值修改
				{
					float f;
					f = (float)(npid->last_adc - *npid->adc) / 50.0f;
					if(f <1)
						f = 1.0f;
					if(f > 4)
						f = 4;
					f = npid->a_go_inside *f;
					*agle+=f;
					//*agle = sys->angle +f;
					if(npid->c_near > 10)
					{
						npid->c_near -=10;		//不让调整角度，否则有问题
					}else
						npid->c_near = 0;
#if NEAR_PID_DEBUG		
					if(sys->near_debug)
						log_printf("%s--:%3.2f->%3.2f,%d(%d),%d,%3.1f\r\n",sStr,sys->angle,*agle,*npid->adc,npid->lock_adc,*npid->adc2,f);
#endif				
				}
				//往里拐了
				if(npid->last_adc < *npid->adc && (*npid->adc - npid->last_adc) > cfg->a_pid_near)
				{
					float f;
					f = (float)(*npid->adc - npid->last_adc) / 50.0f;
					if(f <1)
						f = 1.0f;
					if(f > 4)
						f = 4;
					f = npid->a_go_outside *f;
					*agle+= f;
				//	*agle = sys->angle +f;
					if(f >=1)
					{
						if(npid->c_near > 10)
						{
							npid->c_near -=10;		//不让调整角度，否则有问题
						}else
							npid->c_near = 0;
					}
#if NEAR_PID_DEBUG	
					if(sys->near_debug)
						log_printf("%s++:%3.2f->%3.2f,%d(%d),%d,%3.1f\r\n",sStr,sys->angle,*agle,*npid->adc,npid->max_adc,*npid->adc2,f);
#endif				
				}
				npid->last_adc = *npid->adc;
			}
		}
		//往外拐
		if(*npid->adc >= npid->max_adc || *npid->adc2 >= npid->max_adc2)
		{
			*agle+= npid->a_go_outside;
			//*agle = sys->angle + npid->a_go_outside;
#if NEAR_PID_DEBUG
			if(sys->near_debug)
				log_printf("%s+:%3.2f->%3.2f,%d(%d),%d(%d)\r\n",sStr,sys->angle,*agle,*npid->adc,npid->max_adc,*npid->adc2,npid->max_adc2);
#endif			
		//消失了，往里拐点，
		}
		else if(*npid->adc <= npid->lock_adc)
		{
			*agle+=npid->a_go_inside;
			//*agle = sys->angle +npid->a_go_inside;
#if NEAR_PID_DEBUG		
			if(sys->near_debug)
				log_printf("%s-:%3.2f->%3.2f,%d(%d),%d\r\n",sStr,sys->angle,*agle,*npid->adc,npid->lock_adc,*npid->adc2);
#endif			
		}
		//校准角度
#if ANGLE_NEAR_RUN_ADJ			
		if(*npid->adc >= (npid->lock_adc-100) && *npid->adc <= (npid->max_adc+100))
		{
			//if( disxy(dist ,WALK_DIST()) > 3)
			if( disxy(dist ,WALK_DIST()) > 10)		////3
			{
				npid->c_near++;
				npid->c_near2++;
				sys->c_near = npid->c_near;////
				sys->dist_ok = npid->dist_ok;
				sys->small_dist_ok = npid->small_dist_ok;
				//test20181103
				if(npid->c_near >=50)
					npid->max_lost = cfg->max_near_lost;
				else
					npid->max_lost=3;
						
				if(c_near_a >= MAX_N_ANGLE)
					c_near_a = 0;
				
				near_angle[c_near_a++] = sys->angle;
				//距离超过一定距离，才给矫正
				if(npid->dist == 0)
				{
					npid->dist = motor.c_left_hw;
					log_printf("dist0:%d,%3.3f\r\n",motor.c_left_hw,sys->angle);
				}
				else
				{
					if((motor.c_left_hw - npid->dist) >=MAX_N_DIST) // 1200
					{
						if(npid->dist_ok  == 0)
						{
							npid->small_dist_ok = 1;	
							npid->small_dist = 0;
						}
						npid->dist_ok = 1;
						//npid->small_dist_ok = 1;	
					}
				}
				///////////////////////////////20180410 jzz
				//矫正小的距离超过一定距离，才给矫正				
				if(npid->small_dist == 0 )
				{
					npid->small_dist = motor.c_left_hw;					
				}
				else
				{
					if(motor.c_left_hw - npid->small_dist >=MAX_N_SMALL_DIST) 
						npid->small_dist_ok = 1;					
				}
				//log_printf("s_ok = %d,s_dist%d,motor.c_left_hw =%d\r\n",npid->small_dist_ok,npid->small_dist,motor.c_left_hw);
				/////////////////////////////////end
			}
			dist = WALK_DIST();
			//角度超过，则清零。
			if(!ANGLE_VALID())
				npid->c_near = 0;
			
			
		}else
		{
			npid->c_near = 0;//navigat->distance;
			npid->c_near2 = 0;
			npid->c_angle	= 0;
			//log_printf("unl(%d)\r\n",*npid->adc);
		}
		/*
		if(cc++ > 20)
		{
			cc = 0;
			log_printf("%d,%d\r\n",npid->c_near,npid->c_near2);
		}
		*/
		//角度限制
		//else
		//	near->c_near=0;
		//if(npid->c_near >MAX_N_ANGLE && npid->dist_ok)
		
		if(npid->c_near >MAX_N_ANGLE && npid->dist_ok && npid->small_dist_ok	)
		{
			npid->small_dist_ok = 0;		/////jzz		
			npid->small_dist = 0;			/////jzz
#if GYRO_ADJ_ONBOARD
			npid->c_near -=20;
			if(sys->c_yaw < (MAX_YAW_FIFO-1))
				sys->yaw_fifo[sys->c_yaw++] = sys->yaw;
#else				
			int16_t i;//,c=0,c1=0;
			npid->c_near -=50;
			//npid->dist = motor.c_left_hw;
			
			//float dis=0;
			uint16_t kk=0;
			float disa;
			for(i=0;i<MAX_N_ANGLE;i++)
			{		
				disa = disfloat(sys->angle,near_angle[i]);
				if( disa>=5 && disa <-5)		///在10度以内的超过100个
				{
					log_printf("not vagl(%3.1f,%d)\r\n",near_angle[i],i);
					break;
				}
				if(disa >-2 && disa < 2)		///在6度以内的超过50个
					kk++;
			}
			///////////将i全局变量，打印出来
			sys->i_c_near = i;
			//log_printf("(%3.1f,%d,%d,%d)\r\n",sys->angle,i,kk,motor.c_left_hw);
#if NEAR_PID_DEBUG
			if(sys->near_debug)			
				log_printf("k=%d\r\n",i);
#endif
			if(i >=MAX_N_ANGLE && kk > 45 )
			{
				//micony20190306 - 重要 权宜版本，由于陀螺仪程序的问题，0度方向不能矫正，新的陀螺仪程序可以矫正。
				for(i=ADJ_GYRO_IDX;i<4;i++)
				{
					float dis = disfloat(sys->angle, angle_buf[i]);
					if( dis < 6 || //ok
					   //(dis < ANGLE_ADJ_DIS && ( (motor.c_left_hw - npid->dist) >4500) ))
					   (dis < ANGLE_ADJ_DIS && ( (motor.c_left_hw - npid->dist) >3000) ))
					{
						uint16_t ps; 
						//if(npid->c_near2 >300)
						//	ps = 50;
						//else
							ps = ANGLE_ADJ_PST;
							
						illegal_copy_tell_13757122544_gsend(0x02,bf_angle[i],ps); 
						log_printf("adj_a:%3.1f,%d,%d,%d,%d\r\n",sys->angle,bf_angle[i],ps,motor.c_left_hw,npid->dist);						
						if(dis < 2)		//2度以内，认为是角度偏差不大，
							navigat->t_gyro_adj = sys->t_navi_work;		//刷新时间
						navigat->dis_angle_adj = dis;		//保留最后的差值，用于给上层计算是否校验成功
						
						if(dis > 3.0f)	//大角度，调整较大，则角度重新设置
						{
							*agle = NO_ANGLE_V;
							log_printf("a_out,%3.1f\r\n",*agle);
							
						}
						break;
					}
				}
			}

#endif		
			
		}
		/*
		
		if(npid->c_near >100)
		{
			npid->c_near =0;
			{
				int i;
				for(i=0;i<4;i++)
				{
					if(disfloat(sys->angle, angle_buf[i]) < ANGLE_ADJ_DIS)
					{
						illegal_copy_tell_13757122544_gsend(0x02,bf_angle[i],ANGLE_PEER); 
						log_printf("ajust agle(%3.1f,%3.1f)\r\n",sys->angle,angle_buf[i]);
						break;
					}else if( npid->c_near2 > 400 && disfloat(sys->angle, angle_buf[i]) < 20.0f)
					{
						illegal_copy_tell_13757122544_gsend(0x02,bf_angle[i],90); 
						log_printf("ajust agle2(%3.1f,%3.1f)\r\n",sys->angle,angle_buf[i]);
						npid->c_near2 = 0;
						break;
					}
				}
			}
		}
		*/
#endif	
		return 1;
		
	} else
	{
		//near->c_right = 0;
		npid->c_angle = 0;
		//log_printf("K(%d)\r\n",*npid->adc);
		if(nsta)			///左右延边
		{
			//*agle = sys->angle
			npid->c_lost++;
			//test20181003
			//log_printf("lost:%d,%d\r\n",npid->max_lost,npid->c_lost);
			if(npid->c_lost >=npid->max_lost)
			//if(npid->c_lost >=cfg->max_near_lost)
			{
#if 0			
				*agle+=npid->a_go_inside* 3;
#else
			//	*agle+=2;
				*agle = format_agle(( sys->angle +( npid->a_go_inside*cfg->setlostturnangle) ),ANGLE_360);	//
				//log_printf("k=%3.1f,%3.1f,%d,%3.1f\r\n",sys->angle, npid->a_go_inside,cfg->setlostturnangle,*agle);
				sys->min_pwm = -200;
				sys->max_pwm = 600;
				npid->c_lost_flag=1;
				sys->setminpwm =(int) cfg->setsideminpwm;			//设置强制延边做小PWM值
				sys->setmaxpwm =(int) cfg->setsidemaxpwm;			//设置强制延边做大PWM值
#endif
			}
				npid->c_near = 0;
				npid->c_near2 = 0;
			/*
			pid_obst_around(navigat,sys->angle);		//计算侧面是否有障碍物。
			if(npid->obst ==0)
				*agle+=npid->a_go_inside * 8;
			else
				*agle+=npid->a_go_inside;
			*/
			
			//log_printf("out:%3.1f\r\n",*agle);
			return 0;
		}
		//log_printf("out2:%3.1f\r\n",*agle);
		if(npid->count > MAX_NEAR_COUNT)		//从延边到丢失
		{
			if(npid->c_lost ==0)
			{
				npid->l_angle = sys->angle; //记录下丢失的时间
				//log_printf("first lost angle=%f\r\n",near->l_angle);
			}
			
			if(npid->c_lost < 800)
				npid->c_lost++;			
				
			if(npid->c_lost  >=10)
			{
				//转的角度太大，就不让转了。
				if(disfloat(sys->angle, npid->l_angle) > 150 && npid->c_lost < 800)
				{
					//log_printf("angle=%f,dis near\r\n",sys->angle);
					npid->c_lost = 800;
					npid->l_angle = sys->angle;
				}
				
				npid->c_near	= npid->c_near2 =0;
				npid->c_dis_a = 0;
				npid->last_adc = 0;
				if(nsta  )		//强制延边
				{
					if(npid->c_lost < 800)
						*agle = sys->angle - AJUST_AGLE_LOST;	//
					else
						*agle = npid->l_angle;
				//	log_printf("-:%3.1f\r\n",*agle );
				}
				else			//非强制延边
				{
#if OFFSET_180_PID == 0
					*agle			= navigat->angle ;	
#else
					if(navigat->angle == 180)
						*agle			= navigat->angle + OFFSET_180_PID;				//直接按照导航角度走
					else
						*agle			= navigat->angle;
#endif						
					
				}
			
					
			}

			//return ;
		}

	}
	return 0;

}

#if 0
int near_wall_pid(uint8_t nsta,NEAR_PID *npid,float *agle)
{
	static int dist=0;
#if NEAR_PID_DEBUG	
	char *sStr;
	
	//static int cc=0;
	//static int k=0;
	
	if(npid == &l_near_pid)
			sStr = strl;
		else
			sStr = strr;
	/*
	if(k++ >=5)
	{
		log_printf("[%s:%d,%d]\r\n", sStr,*npid->adc , npid->min_adc);
		k=0;
	}*/
#endif	
	
	if( *npid->adc > npid->min_adc) // 有信号
	{
	/*
		if(npid == &l_near_pid)
			sStr = strl;
		else
			sStr = strr;
		*/	
#if NEAR_PID_DEBUG			
	//	log_printf("%d,%d,%d\r\n",*npid->adc,npid->real_lock,npid->real_max);
#endif		
		//计数靠近边的次数，如果不是强制延边， 靠近边超过一次次数，才做延边
		if(npid->level > 0)	//如果是adj延边等等级，就直接进来
			npid->count = 100;
		else if(*npid->adc >= npid->max_adc || *npid->adc2 >= npid->max_adc2)
				npid->count = MAX_NEAR_COUNT+1;
		
		if(*npid->adc >= npid->lock_adc  ) 
		{
			if(npid->count < 200)
				npid->count++;
		}
		//不是真正的沿边，不做沿边，否则有问题
		if(nsta == 0 && npid->count < MAX_NEAR_COUNT)
		{
			*agle = navigat->angle;
#if NEAR_PID_DEBUG				
			log_printf("out1,%d,%d,%d,%d\r\n",*npid->adc,npid->count,npid->lock_adc);
#endif			
			return 0;
		}
		
		//刚进来，就根据当前的值设置延边PID的上下限
		if(npid->count == MAX_NEAR_COUNT)
		{
			npid->real_lock= *npid->adc - NEAR_PID_OFSET;
			if(*npid->adc < (npid->max_adc-NEAR_PID_OFSET))
				npid->real_max= *npid->adc + NEAR_PID_OFSET;
			else
				npid->real_max= npid->max_adc;
			npid->count+=10;
			npid->c_big_change = 0;
#if NEAR_PID_DEBUG				
			log_printf("set,%d,%d\r\n",npid->real_lock,npid->real_max);
#endif				
		}
		/*	*/
		if(dis_xy(*npid->adc ,npid->real_lock) > 2000)		//突然大跳变重新设置上下限
		{
			if(npid->c_big_change ++ >=30)
			{
#if NEAR_PID_DEBUG				
				log_printf("big change,%d,%d\r\n",*npid->adc,npid->real_lock);
#endif	
				npid->real_lock= *npid->adc - NEAR_PID_OFSET;
				if(*npid->adc < (npid->max_adc-NEAR_PID_OFSET))
					npid->real_max= *npid->adc + NEAR_PID_OFSET;
				else
					npid->real_max= npid->max_adc;

				npid->c_big_change = 0;
			}

		}
	


		npid->c_lost = 0;	
/*		
		//每隔40次，计算一下变化的趋势
		if(*npid->adc <  (npid->max_adc - 200)  && *npid->adc > (npid->lock_adc+200))
		{
			if(npid->last_adc ==0)
			{
				npid->last_adc = *npid->adc;
				npid->c_dis_a=0;
			}
		
			if(npid->c_dis_a++>=10)
			{
				npid->c_dis_a=0;
				//log_printf("[%d,%d]\r\n",npid->last_adc , *npid->adc);
				//变大了100，往外拐
				if(npid->last_adc > *npid->adc && (npid->last_adc - *npid->adc) > 200)
				{
					*agle+=npid->a_go_outside;
#if NEAR_PID_DEBUG					
					log_printf("%s--:%3.2f,%3.2f,%d(%d),%d\r\n",sStr,sys->angle,*agle,*npid->adc,npid->lock_adc,*npid->adc2);
#endif				
				}
				//往里拐了
				if(npid->last_adc < *npid->adc && (*npid->adc - npid->last_adc) > 200)
				{
					*agle+= npid->a_go_inside;
#if NEAR_PID_DEBUG				
					log_printf("%s++:%3.2f,%3.2f,%d(%d),%d(%d)\r\n",sStr,*agle,sys->angle,*npid->adc,npid->max_adc,*npid->adc2,npid->max_adc2);
#endif				
				}
				npid->last_adc = *npid->adc;
			}
		}
	*/	
		//往外拐
		if(*npid->adc >= npid->real_max|| *npid->adc2 >= npid->max_adc2)
		{
			float f;
			f  = (*npid->adc - *npid->adc) / 100.0f;
			if(f < 1.0f)
				f = 1.0f;
			if(f >= 8)
				f = 8;
			if(*npid->adc2 >= npid->max_adc2)
				f = 8;
			*agle+= npid->a_go_outside * f;
#if NEAR_PID_DEBUG				
			log_printf("%s+:%3.2f,%3.2f,%d(%d),%d(%d),%d\r\n",sStr,*agle,sys->angle,*npid->adc,npid->real_max,*npid->adc2,npid->max_adc2,npid->c_near);
#endif			
		//消失了，往里拐点，
		}
		else if(*npid->adc <= npid->real_lock)
		{
			float f;
			f  = (npid->real_lock - *npid->adc) / 100.0f;
			if(f < 1.0f)
				f = 1.0f;
			if(f >= 4)
				f = 4;
			*agle+=npid->a_go_inside * f;
#if NEAR_PID_DEBUG				
			log_printf("%s-:%3.2f,%3.2f,%d(%d),%d,%d\r\n",sStr,*agle,sys->angle,*npid->adc,npid->real_lock,*npid->adc2,npid->c_near);
#endif			
		}
		//校准角度
#if ANGLE_NEAR_RUN_ADJ			
		if(*npid->adc >= (npid->real_lock- 300) && *npid->adc <= (npid->real_max+300))
		{
			if( disxy(dist ,WALK_DIST()) > 3)
			{
				npid->c_near++;
				npid->c_near2++;
			}
			dist = WALK_DIST();
			//角度超过，则清零。
			if(!ANGLE_VALID())
				npid->c_near = 0;
			
		}else
		{
			npid->c_near = 0;//navigat->distance;
			npid->c_near2 = 0;
			npid->c_angle	= 0;
			//log_printf("unl(%d)\r\n",*npid->adc);
		}
		/*
		if(cc++ > 20)
		{
			cc = 0;
			log_printf("%d,%d\r\n",npid->c_near,npid->c_near2);
		}
		*/
		//角度限制
		//else
		//	near->c_near=0;
		if(npid->c_near >30)
		{
			//log_printf("%d,%3.1f\r\n",npid->c_angle,sys->angle);
			npid->c_near = 0;
			//if(npid->c_angle < (MAX_N_ANGLE-1))
			{
			
				near_angle[npid->c_angle++] = sys->angle;
			//	if(npid->c_angle >=10 && (npid->c_angle & 3)==0)
				{

					int16_t i,c=0,c1=0;
				/*	float dis=0;
					for(i=npid->c_angle-2;i > 0;i--)
					{
						dis = disfloat(near_angle[i],near_angle[npid->c_angle-1]); 
						if(dis <= 0.6f && dis >= -0.6f)
							c++;
						if(dis > 5.0f || dis <= -5.0f)
							break;
						c1++;
					}
					
					//log_printf("c=%d,%d\r\n",c1,c);
					if(c1 >=10  )
					*/
					{
				
						for(i=0;i<4;i++)
						{
						
							if(disfloat(sys->angle, angle_buf[i]) < ANGLE_ADJ_DIS)
							{
								if(c > 20)
									illegal_copy_tell_13757122544_gsend(0x02,bf_angle[i],90); 
								else
									illegal_copy_tell_13757122544_gsend(0x02,bf_angle[i],ANGLE_PEER); 
					#if !GYRO_OFSET_ONBORD	
								log_printf("adj_a:%3.1f,%d,%d,%d\r\n",sys->angle,bf_angle[i],c1,c);
					#endif
								break;
							}else if( c >30 && disfloat(sys->angle, angle_buf[i]) < 20.0f)
							{
								illegal_copy_tell_13757122544_gsend(0x02,bf_angle[i],90); 
					#if !GYRO_OFSET_ONBORD			
								log_printf("adj_a2:%3.1f,%d,%d\r\n",sys->angle,bf_angle[i],c1,c);
					#endif
								//npid->c_near2 = 0;
								break;
							}
						}

					}
				}
			}
/*
			else
			{
				npid->c_angle = 0;
				//log_printf("K2\r\n");
			}
			*/
		}
		/*
		
		if(npid->c_near >100)
		{
			npid->c_near =0;
			{
				int i;
				for(i=0;i<4;i++)
				{
					if(disfloat(sys->angle, angle_buf[i]) < ANGLE_ADJ_DIS)
					{
						illegal_copy_tell_13757122544_gsend(0x02,bf_angle[i],ANGLE_PEER);	
						log_printf("ajust agle(%3.1f,%3.1f)\r\n",sys->angle,angle_buf[i]);
						break;
					}else if( npid->c_near2 > 400 && disfloat(sys->angle, angle_buf[i]) < 20.0f)
					{
						illegal_copy_tell_13757122544_gsend(0x02,bf_angle[i],90);	
						log_printf("ajust agle2(%3.1f,%3.1f)\r\n",sys->angle,angle_buf[i]);
						npid->c_near2 = 0;
						break;
					}
				}
			}
		}
		*/
#endif	
		return 1;
		
	} else
	{
		//near->c_right = 0;
		npid->c_angle = 0;
		//log_printf("K(%d)\r\n",*npid->adc);
		if(nsta)
		{
			//*agle = sys->angle
			npid->c_lost++;
			if(npid->c_lost >=npid->max_lost)
			
				*agle+=npid->a_go_inside * 10;
				npid->c_near = 0;
				npid->c_near2 = 0;
			/*
			pid_obst_around(navigat,sys->angle);		//计算侧面是否有障碍物。
			if(npid->obst ==0)
				*agle+=npid->a_go_inside * 8;
			else
				*agle+=npid->a_go_inside;
			*/
			
			//log_printf("out:%3.1f\r\n",*agle);
			return 0;
		}
		//log_printf("out2:%3.1f\r\n",*agle);
		if(npid->count > MAX_NEAR_COUNT)		//从延边到丢失
		{
			if(npid->c_lost ==0)
			{
				npid->l_angle = sys->angle; //记录下丢失的时间
				//log_printf("first lost angle=%f\r\n",near->l_angle);
			}
			
			if(npid->c_lost < 800)
				npid->c_lost++;
	
			
				
			if(npid->c_lost  >=10)
			{
				//转的角度太大，就不让转了。
				if(disfloat(sys->angle, npid->l_angle) > 150 && npid->c_lost < 800)
				{
					//log_printf("angle=%f,dis near\r\n",sys->angle);
					npid->c_lost = 800;
					npid->l_angle = sys->angle;
				}
				
				npid->c_near	= npid->c_near2 =0;
				npid->c_dis_a = 0;
				npid->last_adc = 0;
				if(nsta  )		//强制延边
				{
					if(npid->c_lost < 800)
						*agle = sys->angle - AJUST_AGLE_LOST;	//
					else
						*agle = npid->l_angle;
				//	log_printf("-:%3.1f\r\n",*agle );
				}
				else
				{
#if OFFSET_180_PID == 0
					*agle 			= navigat->angle ;	
#else
					if(navigat->angle == 180)
						*agle 			= navigat->angle + OFFSET_180_PID;				//直接按照导航角度走
					else
						*agle  			= navigat->angle;
#endif						
					
				}
			
					
			}

			//return ;
		}

	}
	return 0;

}
#endif
int navigat_near_wall_pid(float *agle,int c_lost)
{
	static int c_pid=0;
	NEAR_WALL *near;

	near = &navigat->near;			//从navigat->near获取数据
	if(c_pid ++ >=2)
	{
		c_pid = 0;
		if(near->n_sta !=NO_SIDE_NEAR && near->pid != NULL)		//强制延边
		{
			return (near_wall_pid(1,near->pid,agle));
		}else													//不是强制延边
		{
			if(near_wall_pid(0,&l_near_pid,agle) ==0)
			{
				return (near_wall_pid(0,&r_near_pid,agle));
			}
			return 1;
		}			
	}
	return 1;
}



#if 0
/*
根据激光测试出来的延边
*/
#define PID_NER_DEBUG	0
void pid_near_dist(uint16_t set_dist,uint_16 dist ,pid_t *pid)
{
	float incrementSpeed;
	float	kp,kd;//,ki;
	 uint16_t ird_adc=0;
	 
	if(dist ==0 || dist > MAX_LASR_DIST)
	{
		reutrn 0;
	}

	
	//pid->SetSpeed	= set_speed;
	pid->ActualSpeed = ird_adc;
#if PID_NER_DEBUG	
	log_printf("%5.1f,",pid->ActualSpeed);
#endif	
	if(RIGHT_ADC() > 1800)
		pid->err			= pid->ActualSpeed - pid->SetSpeed;
	else if(LEFT_ADC() >=1800)
		pid->err			= pid->SetSpeed-pid->ActualSpeed ;
	else
	{
		pid->out = navigat->angle;
		return 0;
	}
	//if(pid->err < 100)
	//	return 1;
	kp = pid->Kp * pid->err;
	kd = pid->Kd * (pid->err-pid->err_next);
	
	incrementSpeed	=  kp+ kd;
#if PID_NER_DEBUG	
//	log_printf("[%5.3f,%5.3f,%5.3f,%5.1f],",pid->err,kp,kd,incrementSpeed);
#endif		
	if(incrementSpeed < -3)
		incrementSpeed = -3;
	if(incrementSpeed > 3)
		incrementSpeed = 3;
	pid->out			+= incrementSpeed;
	if(pid->out > 340)
		pid->out -= 360;
	if(pid->out > 340)
	{
		log_printf("pid_near pid error\r\n");
		pid->out = navigat->angle;
	}
#if PID_NER_DEBUG		
	log_printf("%3.1f,%5.1f,%d\r\n",pid->out,pid->err,sys->c_near_angle);
#endif

	
	pid->err_last	= pid->err_next;
	pid->err_next	= pid->err;
	//?à′???à′2???D?μ÷?????è ???°ê? 3 * 4 = 12 ??
	//if(++c_rst_angle >=2 )
	{
		pid->SetSpeed = ird_adc;
//		c_rst_angle = 0;
	}
	if(pid->err > -80 && pid->err < 80)
	{
		//log_printf("1\r\n");
		return 1;
	}
	//log_printf("0\r\n");
	return 0;
}

#endif
#if 0
void navigat_near_wall_pid(float *agle,int c_lost)
{
	static int c_pid=0;
//	float angle;
	NEAR_WALL *near;
	static int lx=0,ly=0;
	
	near = &navigat->near;
	/*
	if(sys->c_pid_hw > navigat->distance || (navigat->distance - sys->c_pid_hw )>=3)
		goto l_near_pid;
	return ;
*/

//l_near_pid:
	sys->c_pid_hw = navigat->distance;
	if(c_pid ++ >=2)
	{
		c_pid = 0;
		//右边PID
		if(near ->n_sta == NO_SIDE_NEAR || near->n_sta == RIGHT_SIDE_NEAR)
		{
			//有信号
			
		}


		//左沿边
		if(near ->n_sta == NO_SIDE_NEAR || near->n_sta == LEFT_SIDE_NEAR)
		{
			if(sys->g_sta[0] >= IRD_LEFT_PID_MIN || sys->g_sta[1] >= IRD_LEFT_PID_MIN2  ) //3100
			{
				//真正的延边

					if(sys->g_sta[0] >= IRD_LEFT_PID_LOCK || sys->g_sta[1] >= IRD_LEFT_PID_MAX2  ) 
					{
						if(near->c_left < 200)
							near->c_left++;
					}
					//不是真正的沿边，不做沿边，否则有问题
					if(near->n_sta == NO_SIDE_NEAR && near->c_left < 30)
					{
						*agle = navigat->angle;
						//log_printf("out2\r\n");
						return ;
					}
					near->c_lost = 0;
					if(near->sta == LOST_WALL_LEFT)		//从丢失沿边回来
					{
						log_printf("lost wall but found");
						//motor_run(GO_STOP,0,0,0);
						//delay_ms(200);
						pd_gyro_int(GO_FORWARD_PWM);
						*agle =sys->angle;
						motor_run(GO_FORWARD, GO_FORWARD_PWM, 0, 0);
					}
					if(near->sta != NEAR_WALL_LEFT)
						log_printf("found wall l");
					near->sta = NEAR_WALL_LEFT;
					
					if((sys->g_sta[0] >=1000 && sys->g_sta[0] <=sys->g_sta[1] ) ||sys->g_sta[0] >= IRD_LEFT_PID_MAX/*cfg->max_left_adc|*/|  sys->g_sta[1]>=IRD_LEFT_PID_MAX2)
					{

						log_printf("l-:%f,%d\r\n",*agle,sys->g_sta[1]);
						*agle-= AJUST_AGLE;
					//消失了，往里拐点，
					}
					if(sys->g_sta[0] < IRD_LEFT_PID_LOCK)	 
					{
							log_printf("l+:%f,%d\r\n",*agle,sys->g_sta[1]);
							*agle+=AJUST_AGLE_2;
					}
#if ANGLE_NEAR_RUN_ADJ				
					if(sys->g_sta[0] > 500 && sys->g_sta[0] < cfg->max_left_adc)
						near->c_near++;
				//	else
				//		near->c_near=0;
				
					if(near->c_near > 50)
					{
						near->c_near =0;
					
						if(disfloat(sys->angle, 90) < 20)
							illegal_copy_tell_13757122544_gsend(0x02,90,ANGLE_PEER);	
						if(disfloat(sys->angle, 0) < 20)
							illegal_copy_tell_13757122544_gsend(0x02,0,ANGLE_PEER);	
						if(disfloat(sys->angle, 180) < 20)
							illegal_copy_tell_13757122544_gsend(0x02,180,ANGLE_PEER);
						if(disfloat(sys->angle, 270) < 20)
							illegal_copy_tell_13757122544_gsend(0x02,270,ANGLE_PEER);
						log_printf("ajust agle(%3.1f)\r\n",sys->angle);
					
						
					}
					if(near->c_near)
					{
						if(lx!=navigat->tx || ly!= navigat->ty)
						{
						//	log_printf("    npid(%d.%d,%3.1f)\r\n",navigat->tx,navigat->ty,near->yaw);
							lx =navigat->tx ;
							ly = navigat->ty;
						}
					}
#endif					
					/*
					if(near->count > 10)
					{
						angle = near->angle / (float)near->count;
						log_printf("(%3.3f)",angle);
					}
					*/
						
				//	log_printf("\r\n");
					return ; //
				
			} else
			{
				near->c_left = 0;
				if(near->n_sta !=RIGHT_SIDE_NEAR)		//需要沿边
				{
					
					if(near->c_lost < 100)
						near->c_lost++;
						
					near->c_near =0;

					if(near->c_lost  >=10)
					{
						//log_printf("lost\r\n");
						
						near->sta = LOST_WALL_LEFT;
						if(near->n_sta == LEFT_SIDE_NEAR)
							*agle = sys->angle + AJUST_AGLE_LOST;
						
						else
							*agle = navigat->angle;
					}
				//	return ;
				}
		
			}
		}
		

	}

	//*agle = navigat->angle;

}
#endif
#if 0
int pid_calce_sp(pid_t *pid,float set_speed,float now_speed)
{
	float incrementSpeed;
	float	kp,kd,ki;
	pid->SetSpeed	= set_speed;
	pid->ActualSpeed = now_speed;
#if PID_DEBUG	
	log_printf("%5.3f,",now_speed);
#endif	
	pid->err			= pid->SetSpeed-pid->ActualSpeed;
	pid->integral +=pid->err;
	kp = pid->Kp * pid->err;
	kd = pid->Kd * (pid->err-pid->err_next);
	ki = pid->Ki * pid->integral;
	/*
	kp = pid->Kp*(float)(pid->err-pid->err_next);
	kd = pid->Kd*(float)(pid->err-2*pid->err_next + pid->err_last);
	ki = pid->Ki*pid->err;
	*/
	incrementSpeed	=  kp+ ki + kd;
#if PID_DEBUG	
	log_printf("%5.3f,%5.3f,%5.3f,%5.1f ",kp,ki,kd,incrementSpeed);
#endif		
	if(incrementSpeed < -10)
		incrementSpeed = -10;
	if(incrementSpeed > 10)
		incrementSpeed = 10;
	sys->pwm			+= incrementSpeed;


#if PID_DEBUG		
//	log_printf(",%5.1f,",pid->out);
#endif	
	if(sys->pwm >=800)
		sys->pwm=800;
	if(sys->pwm < 300)
		sys->pwm=300;
	pid->err_last	= pid->err_next;
	pid->err_next	= pid->err;
	return (int)pid->out;
}
#endif
#if 0
void proc_hw_pid(void)
{
	static char c_loop=0;
	static float set_speed=0;
	float l_btmr,l_etmr,lcount;
//	float r_btmr,r_etmr,rcount;
	disable_irq();
		l_btmr = (float)motor.t_begin_left;
		l_etmr = (float)motor.t_end_left;
		lcount = (float)motor.c_left_hw;
		motor.t_begin_left = motor.t_end_left=motor.c_left_hw=0;
/*
		r_btmr = (float)motor.t_begin_right;
		r_etmr = (float)motor.t_end_right;
		rcount = (float)motor.c_right_hw;
		motor.t_begin_right= motor.t_end_right=motor.c_right_hw=0;
		*/
		TIM6->CNT=0;	
	enable_irq();
	sys->lspeed = (l_etmr-l_btmr) / lcount;
	//sys->rspeed = (r_etmr-r_btmr) / rcount;
	if(c_loop >=3)
	{
		L_FORWRK_PWM = MAX_PWM - pid_calce(&lpid,set_speed,sys->lspeed);
		//R_FORWRK_PWM = MAX_PWM - pid_calce(&rpid,set_speed,sys->rspeed);
	}else
	{
		c_loop++;
		set_speed = sys->lspeed;
	}
	//log_printf("lsp=%f,rsp=%f,l:%d,r%d\r\n",lspeed,rspeed,L_FORWRK_PWM,R_FORWRK_PWM);
}


PI rPid,lPid; 

/*==================================================================================================== 
Initialize PID Structure PID参数初始化
=====================================================================================================*/ 
void Speed_PIDInit(void) 
{ 
	lPid.Target=rPid.Target = 0;
	lPid.Out=rPid.Out		= 0;
	lPid.Udk=rPid.Udk    = 0;
	lPid.Out_1=rPid.Out_1   = NOMORL_PWM;
	lPid.ek_0=rPid.ek_0 	= 0;	 //ek=0	  本次误差值
	lPid.ek_1=rPid.ek_1 	= 0;	 //ek-1=0 上次误差值
	lPid.ek_2=rPid.ek_2 	= 0; 	 //ek-2=0
	lPid.P=rPid.P 		= 0.02;  //比例常数 Proportional Const 
	lPid.I=rPid.I 		= 0.05; 	 //积分常数Integral Const
	lPid.b=rPid.b      = 0;

	lPid.maxIn=rPid.maxIn =5000;
	lPid.minIn=rPid.minIn =300;
	lPid.minOut=rPid.minOut =5;
	lPid.maxOut=rPid.maxOut =1000;



}

void pd_gyro_int(PI *s_pid,int n_pwm,int n_speed,int target)
{
	s_pid->Target=target;
	s_pid->Out=s_pid->Out_1=n_pwm;

}
/*==================================================================================================== 
增量式PID计算部分 
=====================================================================================================*/ 
int pid_calc(PI *s_pid ) 
{
	
	
	int mReal;
	int Udk;	

	mReal=s_pid->In;
	if(mReal < s_pid->minIn)
	{
	 	mReal = s_pid->minIn;		
	}
	else if(mReal > s_pid->maxIn)
	{
		mReal = s_pid->maxIn;
	}


	//s_pid->Target = Target;    			  		//目标速度
	s_pid->ek_0= s_pid->Target - mReal; 	//增量计算， ek_0，误差值

//	if(s_pid->ek_0 < 10)

	if(((s_pid->Out_1>=s_pid->maxOut)&&(s_pid->ek_0>=0))||((s_pid->Out_1<=s_pid->minOut)&&(s_pid->ek_0<=0)))
	{
	    s_pid->b=0;
	} 
	else
	{
		s_pid->b=1;
	} 
#if PID_DEBUG
	log_printf("targ=%d,In=%d,e1=%d,e0=%d(%d),b=%d\r\n",s_pid->Target,mReal,s_pid->ek_1,s_pid->ek_0,s_pid->ek_0-s_pid->ek_1,s_pid->b);
#endif

	s_pid->Udk=s_pid->P*(s_pid->ek_0-s_pid->ek_1) + s_pid->b*s_pid->I*s_pid->ek_0 ;
	Udk = s_pid->Udk;
#if PID_DEBUG
	log_printf("U=%f+%f=%d\r\n",s_pid->P*(s_pid->ek_0-s_pid->ek_1),s_pid->b*s_pid->I*s_pid->ek_0,s_pid->Udk);
	
#endif	
    //限幅
	if(s_pid->Udk >10)
	   s_pid->Udk=10;
	if(s_pid->Udk <-10)
		s_pid->Udk=-10;
	
	s_pid->Out = s_pid->Out_1 + s_pid->Udk;
	
  //  log_printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",s_pid->Target,mReal,s_pid->ek_1,s_pid->ek_0,s_pid->ek_0-s_pid->ek_1,s_pid->b,Udk,s_pid->Udk,s_pid->Out_1,s_pid->Out) ;
#if PID_DEBUG
	log_printf("Out_1=%d,Out=%d\r\n",s_pid->Out_1,s_pid->Out);
#endif
	/* 存储误差，用于下次计算 */
	s_pid->ek_2 =	s_pid->ek_1;
	s_pid->ek_1 = s_pid->ek_0;


	
	s_pid->Out_1 = s_pid->Out; 

   	if(s_pid->Out >= s_pid->maxOut)
	{
		s_pid->Out = s_pid->maxOut;
		//log_printf("act out=%d\r\n",s_pid->Out);
	}

	else if(s_pid->Out <= s_pid->minOut)
	{
		s_pid->Out = s_pid->minOut;
		//log_printf("act out=%d\r\n",s_pid->Out);
	} 
	
	s_pid->Out_1 = s_pid->Out; 


      return s_pid->Out;
//	pidTmr=msTmr  ;
///	log_printf("2,0,0,0,%d,%d,%d\r\n",s_pid->In,s_pid->Target,s_pid->Out);
//#endif

}

void calc_pid(PI *s_pid  ) 
{
	
	int mReal;

	mReal=s_pid->In;
	/*
	if(mReal < s_pid->minIn)
	{
	 	mReal = s_pid->minIn;		
	}
	else if(mReal > s_pid->maxIn)
	{
		mReal = s_pid->maxIn;
	}
*/

	//s_pid->Target = Target;    			  		//目标速度
	s_pid->ek_0= s_pid->Target - mReal; 	//增量计算， ek_0，误差值


	//三个阶段PID
	if(s_pid->ek_0>=0)
	{
		if(s_pid->ek_0 >200)
		{
			s_pid->Out = s_pid->Out_1 -2;
		}else if(s_pid->ek_0 >100)
		{
			s_pid->Out = s_pid->Out_1 -1;
		}else 
			s_pid->Out = s_pid->Out_1;
	}else
	{
		if(s_pid->ek_0 <-200)
		{
			s_pid->Out = s_pid->Out_1 + 2;
		}else if(s_pid->ek_0 <-100)
		{
			s_pid->Out = s_pid->Out_1 + 1;
		}else 
			s_pid->Out = s_pid->Out_1;
	}
#if PID_DEBUG
	log_printf("g=%d,I=%d(%d)o1=%d,o=%d\r\n",s_pid->Target,mReal,s_pid->ek_0,s_pid->Out_1,s_pid->Out);
#endif		

	
	s_pid->Out_1 = s_pid->Out; 

   	if(s_pid->Out >= s_pid->maxOut)
	{
		s_pid->Out = s_pid->maxOut;
	}

	else if(s_pid->Out <= s_pid->minOut)
	{
		s_pid->Out = s_pid->minOut;
	} 
	
	s_pid->Out_1 = s_pid->Out; 
	


   //   return s_pid->Out;
//	pidTmr=msTmr  ;
///	log_printf("2,0,0,0,%d,%d,%d\r\n",s_pid->In,s_pid->Target,s_pid->Out);
//#endif

}
#endif

/*
#define NEAR_PID_DEBUG  0

int pid_near_wall(pid_t *pid,int gset,int gnow)
{
	float incrementSpeed;
	float	kp,kd,ki;
	pid->SetSpeed	= gset;
	pid->ActualSpeed = gnow;
#if NEAR_PID_DEBUG	
	log_printf("%d,%d,",gset,gnow);
#endif	
	pid->err			= pid->SetSpeed-pid->ActualSpeed;
	pid->integral +=pid->err;
	kp = pid->Kp * pid->err;
	kd = pid->Kd * (pid->err-pid->err_next);
	ki = pid->Ki * pid->integral;

	incrementSpeed	=  kp+ ki + kd;
#if NEAR_PID_DEBUG	
	log_printf("%5.3f,%5.3f,%5.3f,%5.3f ",kp,ki,kd,incrementSpeed);
#endif		
	if(incrementSpeed < -1.5)
		incrementSpeed = -1.5;
	if(incrementSpeed > 1.5)
		incrementSpeed = 1.5;
	//sys->pwm			+= incrementSpeed;

	pid->out		+=incrementSpeed;
#if PID_DEBUG		
//	log_printf(",%5.1f,",pid->out);
#endif	

	pid->err_last	= pid->err_next;
	pid->err_next	= pid->err;
	return pid->out;
}

*/
void turn_round_pid(int sta)
{
	static int  c =0;
	static int lhw1=0,lhw2=0;
	int	speed = sta;
	static uint8_t count=0,c_ok=0;
#if JUDGE_PID_CTRL_WHEEL_STOP	
			static int idex=0,hw_l=0,hw_r=0;	//,lhw2=0;
#endif
	
	//初始化
	if(sta ==0)
	{
		count =c =  lhw1 = lhw2 = 0;	
		idex=0;
		return ;
	}
	//log_printf("c:%d\r\n",c);
	//循环
	if(c++>4)
	{
		int hw1=0,hw2=0;
		int pid_pwm;
		c=0;
		
		hw1 = motor.c_left_hw - lhw1;
		hw2 = motor.c_right_hw- lhw2;
		
		lhw1 = motor.c_left_hw ;
		lhw2 = motor.c_right_hw;
		if(count >=4)
		{
			//motor.c_left_hw = motor.c_right_hw = 0;
			pid_pwm = (hw1 - speed)*5;
			sys->left_pwm +=pid_pwm;
			if(sys->left_pwm < 450)
				sys->left_pwm = 450;
			if(sys->left_pwm > 800)
				sys->left_pwm = 800;
			*left_pwm = sys->left_pwm;

			pid_pwm = (hw2 - speed)*5;
			sys->right_pwm +=pid_pwm;
			if(sys->right_pwm < 450)
				sys->right_pwm = 450;
			if(sys->right_pwm > 800)
				sys->right_pwm = 800;
			*right_pwm = sys->right_pwm;
				//log_printf("%d,%d,%d,%d\r\n",hw1, hw2,sys->left_pwm,sys->right_pwm);
		}else
			count++;
#if JUDGE_PID_CTRL_WHEEL_STOP
#if GET_OUT_OF_TROUBLE_EN
	if( /*(motor.c_left_hw > 50 || motor.c_right_hw > 50 ) && */ ((disXY(hw_l,motor.c_left_hw)<4 && *left_pwm <701) || ((disXY(hw_r,motor.c_right_hw) <4) && *right_pwm < 701)) &&  sys->motor_primary_locked == 0) 
#else
	if(/*(motor.c_left_hw > 10 || motor.c_right_hw > 10 ) &&*/((disXY(hw_l,motor.c_left_hw)<4&& *left_pwm <901) || (disXY(hw_r,motor.c_right_hw) <4 && *right_pwm <901) ))
#endif	
	{
		c_ok = 0;
		if(idex++ >580)		//3s*7=21s
		{	log_printf("dubug_turn_idex:L(%d,%d)R(%d,%d)%d\r\n",hw_l,motor.c_left_hw,hw_r,motor.c_right_hw,idex);
#if GET_OUT_OF_TROUBLE_EN
			sys->motor_primary_locked = 1;
#else		
			sys->motor_locked_flag = 1;//缠线
#endif			
			idex = 0;
		}	
	}	
	else
	{	if(c_ok++>3)
			idex = 0;
		//sys->motor_locked_flag = 0;//缠线
	}
	hw_l = motor.c_left_hw ;
	hw_r = motor.c_right_hw;
#endif					
	}	
}


void charge_turn_pid(int sta)
{
//	static int  c =0;
	static int lhw1=0,lhw2=0;
	int	speed = sta;
	static uint8_t count=0,c_ok=0;

#if JUDGE_PID_CTRL_WHEEL_STOP	
		static int idex=0,hw_l=0,hw_r=0;	//,lhw2=0;
#endif
	
	//初始化
	if(sta ==0)
	{
		count =  lhw1 = lhw2 = 0;//=c;
		idex=0;
		return ;
	}
	
	//循环
	//if(c++>4)
	{
		int hw1=0,hw2=0;
		int pid_pwm;
//		c=0;
		
		hw1 = motor.c_left_hw - lhw1;
		hw2 = motor.c_right_hw- lhw2;
		
		lhw1 = motor.c_left_hw ;
		lhw2 = motor.c_right_hw;
		if(count >=2)//4)                       
		{
			//motor.c_left_hw = motor.c_right_hw = 0;
			pid_pwm = (hw1 - speed)*5;
			sys->left_pwm +=pid_pwm;
			if(sys->left_pwm < 450)
				sys->left_pwm = 450;
			if(sys->left_pwm > 780)
				sys->left_pwm = 780;
			*left_pwm = sys->left_pwm;

			pid_pwm = (hw2 - speed)*5;
			sys->right_pwm +=pid_pwm;
			if(sys->right_pwm < 450)
				sys->right_pwm = 450;
			if(sys->right_pwm > 780)
				sys->right_pwm = 780;
			*right_pwm = sys->right_pwm;
			//	log_printf("%d,%d,%d,%d\r\n",hw1, hw2,sys->left_pwm,sys->right_pwm);
			//log_printf("%d,%d,%d,%d\r\n",hw1, hw2,*right_pwm,*left_pwm);
		}else
			count++;
		
#if JUDGE_PID_CTRL_WHEEL_STOP         
#if GET_OUT_OF_TROUBLE_EN          //左右轮缠线
	if(/*(motor.c_left_hw > 50 || motor.c_right_hw > 50) &&*/ ((disXY(hw_l,motor.c_left_hw)<4 && *left_pwm <701) || ((disXY(hw_r,motor.c_right_hw) <4) && *right_pwm < 701)) &&  sys->motor_primary_locked == 0) 
#else
	if(/*(motor.c_left_hw > 10 || motor.c_right_hw > 10) &&*/((disXY(hw_l,motor.c_left_hw)<4 && *left_pwm <901) || (disXY(hw_r,motor.c_right_hw) <4 && *left_pwm<901) ))
#endif	
	{
		c_ok = 0;
		if(idex++ >2300)		//
		{	log_printf("change_idex:L(%d,%d)R(%d,%d)%d\r\n",hw_l,motor.c_left_hw,hw_r,motor.c_right_hw,idex);
#if GET_OUT_OF_TROUBLE_EN
			sys->motor_primary_locked = 1;	
#else		
			sys->motor_locked_flag = 1;//缠线
#endif			
			idex = 0;
		}	
	}
	else
	{
		if(c_ok++>3)
			idex = 0;
		//sys->motor_locked_flag = 0;//缠线
	}
	hw_l = motor.c_left_hw ;
	hw_r = motor.c_right_hw;
#endif				
	

	}


}


/*
速度换
*/
void speed_pid_ctrl(uint16_t speed)
{

	static int  c =0;
	static int lhw1=0,lhw2=0;
	//循环 每隔50计算一次
	if(c++>10)
	{
		int hw1=0,hw2=0;
		int pid_pwm;
		c=0;
		
		hw1 = motor.c_left_hw - lhw1;
		hw2 = motor.c_right_hw- lhw2;
		
		lhw1 = motor.c_left_hw ;
		lhw2 = motor.c_right_hw;
		//motor.c_left_hw = motor.c_right_hw = 0;
		pid_pwm = (hw1 - hw2)*5;
		sys->left_pwm +=pid_pwm;
		if(sys->left_pwm < 450)
			sys->left_pwm = 450;
		if(sys->left_pwm > 780)
			sys->left_pwm = 780;
		//l_motor_set_pwm(sys->left_dir,sys->left_pwm);
		*left_pwm = sys->left_pwm;
	//	log_printf("%3.1f,%d,%d,%d,%d\r\n",sys->angle,hw1, hw2,pid_pwm,sys->left_pwm);

	}	


}

/*
直行的时候，简单速度环
*/
#if JUDGE_PID_CTRL_WHEEL_STOP
void m_speed_pid_ctrl(uint16_t speed,uint8_t type,int hw,int c_left_hw_min,uint8_t ird_state,int hw_min,int walk_dist_min)
{

//	static int  c =0;
static int lhw1=0;	//,lhw2=0;
	int hw1=0;////,hw2=0;
	int pid_pwm;

	static int idex=0,hw_l=0,hw_r=0,c_ok=0;	//,lhw2=0;
	
	static int pid_flag = 0;

	if(type ==0)
	{
		lhw1 = motor.c_left_hw;
		idex=0;
		return ;
	}
	if(ird_state == 1)
	{
		if(motor.c_left_hw > c_left_hw_min && !SLOW_IRD_MIDL() &&
		 !(hw > hw_min && WALK_DIST() > (hw-walk_dist_min)))

			pid_flag = 1;
		else
			pid_flag = 0;

	}
	else if(ird_state == 0)
	{
		if(motor.c_left_hw > c_left_hw_min &&/* !SLOW_IRD_MIDL() && */
					!(hw > hw_min && WALK_DIST() > (hw-walk_dist_min))) 
			pid_flag = 1;
		else
			pid_flag = 0;
	}		
	else if(ird_state == 2)
	{

	}
	//循环 每隔50计算一次
	//if(c++>10)
	if(pid_flag)
	{
#if SYS_VER == VER_KESMAN
	#define RUN_SPEED	100	
	if(sys->pwm !=470)
		sys->pwm = 470;
	return ;
#endif
#if SYS_VER == VER_ROMMAT || VER_SMALLBLACKMACH || VER_BIGWHITEMACH	|| VER_WHITEBLUEMACH
	#define RUN_SPEED	220//180//180	

		//c=0;
		
		hw1 = motor.c_left_hw - lhw1;
		lhw1 = motor.c_left_hw ;
		//log_printf("hw=%d,%d,",hw1,sys->pwm);
		//if(hw1 > 100 && hw1 < 300)
		if(hw1 < RUN_SPEED)
		{

			/**/
			pid_pwm = (hw1 - RUN_SPEED) / 2;
			sys->pwm +=pid_pwm;
			if(sys->pwm > 800)
			{
				sys->pwm = 800;
			}	
			if(sys->pwm < 300)
			{
				sys->pwm = 300;	
			}
			
			//log_printf("%d(%d)\r\n",sys->pwm,pid_pwm);
		}	
		//log_printf(",%d\r\n",sys->pwm);
		//hw2 = motor.c_right_hw- lhw2;

		/*
		lhw1 = motor.c_left_hw ;
		lhw2 = motor.c_right_hw;
		//motor.c_left_hw = motor.c_right_hw = 0;
		pid_pwm = (hw1 - hw2)*5;
		sys->left_pwm +=pid_pwm;
		if(sys->left_pwm < 450)
			sys->left_pwm = 450;
		if(sys->left_pwm > 780)
			sys->left_pwm = 780;
		//l_motor_set_pwm(sys->left_dir,sys->left_pwm);
		*left_pwm = sys->left_pwm;
		*/
	//	log_printf("%3.1f,%d,%d,%d,%d\r\n",sys->angle,hw1, hw2,pid_pwm,sys->left_pwm);
#endif
/*
#if SYS_VER == VER_SMALLBLACKMACH
	#define RUN_SPEED	100	
		if(sys->pwm !=470)
			sys->pwm = 470;
		
#endif
*/
	}	
#if GET_OUT_OF_TROUBLE_EN
	if(( /*(motor.c_left_hw > 50 || motor.c_right_hw > 50 ) && */(disXY(hw_l,motor.c_left_hw)<8 && *left_pwm <701) || ((disXY(hw_r,motor.c_right_hw) <8) && *right_pwm < 701)) &&  sys->motor_primary_locked == 0) 
#else
	if(/*(motor.c_left_hw > 10 || motor.c_right_hw > 10 ) &&*/ /*()&&*/ ((disXY(hw_l,motor.c_left_hw)<8 && *left_pwm <901) || ((disXY(hw_r,motor.c_right_hw) <8) && *right_pwm < 901)) )
#endif
	//if((disXY(hw_l,motor.c_left_hw)<8 ) || (disXY(hw_r,motor.c_right_hw) <8)) 
	{
		if(idex++ >68 /*&& */)		//20s
		{	log_printf("dubug_idex:L%d(%d,%d)R%d(%d,%d)%d\r\n",*left_pwm ,hw_l,motor.c_left_hw,*right_pwm ,hw_r,motor.c_right_hw,idex);
#if GET_OUT_OF_TROUBLE_EN
			sys->motor_primary_locked = 1;
#else
			sys->motor_locked_flag = 1;//缠线
#endif			
			idex = 0;
		}	
		c_ok = 0;
	}
	else
	{
		if(c_ok++>3)
			idex = 0;
		//sys->motor_locked_flag = 0;//缠线
	}
	hw_l = motor.c_left_hw ;
	hw_r = motor.c_right_hw;	

	return ;
}

#else

void m_speed_pid_ctrl(uint16_t speed,uint8_t type)
{

//	static int  c =0;
static int lhw1=0;	//,lhw2=0;
	int hw1=0;////,hw2=0;
	int pid_pwm;

	
	if(type ==0)
	{
		lhw1 = motor.c_left_hw;
		return ;
	}
	//循环 每隔50计算一次
	//if(c++>10)
	{
#if SYS_VER == VER_KESMAN
	#define RUN_SPEED	100	
	if(sys->pwm !=470)
		sys->pwm = 470;
	return ;
#endif
#if SYS_VER == VER_ROMMAT || VER_SMALLBLACKMACH || VER_BIGWHITEMACH	|| VER_WHITEBLUEMACH
	#define RUN_SPEED	220//180//180	

		//c=0;
		
		hw1 = motor.c_left_hw - lhw1;
		lhw1 = motor.c_left_hw ;
		//log_printf("hw=%d,%d,",hw1,sys->pwm);
		//if(hw1 > 100 && hw1 < 300)
		if(hw1 < RUN_SPEED)
		{

			/**/
			pid_pwm = (hw1 - RUN_SPEED) / 2;
			sys->pwm +=pid_pwm;
			if(sys->pwm > 800)
			{
				sys->pwm = 800;
			}	
			if(sys->pwm < 300)
			{
				sys->pwm = 300;	
			}
			
			//log_printf("%d(%d)\r\n",sys->pwm,pid_pwm);
		}

		//log_printf(",%d\r\n",sys->pwm);
		//hw2 = motor.c_right_hw- lhw2;

		/*
		lhw1 = motor.c_left_hw ;
		lhw2 = motor.c_right_hw;
		//motor.c_left_hw = motor.c_right_hw = 0;
		pid_pwm = (hw1 - hw2)*5;
		sys->left_pwm +=pid_pwm;
		if(sys->left_pwm < 450)
			sys->left_pwm = 450;
		if(sys->left_pwm > 780)
			sys->left_pwm = 780;
		//l_motor_set_pwm(sys->left_dir,sys->left_pwm);
		*left_pwm = sys->left_pwm;
		*/
	//	log_printf("%3.1f,%d,%d,%d,%d\r\n",sys->angle,hw1, hw2,pid_pwm,sys->left_pwm);
#endif
/*
#if SYS_VER == VER_SMALLBLACKMACH
	#define RUN_SPEED	100	
		if(sys->pwm !=470)
			sys->pwm = 470;
		return ;
#endif
*/
	}	


}

#endif

#if 1


//int near_lost = 0;



typedef struct _mpid_t{
	int16_t c_near_loop;
	int16_t c_near_lost ;
	int16_t c_near_wall; 
	uint8_t m_nsta;
	uint8_t	distok;
	uint16_t	min_adc;
	uint16_t	max_adc;
	uint16_t 	c_near;
	int	dist;
	
	uint16_t 	*adc;		//主红外
	
	float SetSpeed;			//定义设定值
	float ActualSpeed;		//定义实际值
	float err; 				//定义偏差值
	float err_next;	//定义上一个偏差值
	float integral;
	float err_last;//定义最上前的偏差值
	float Kp,Ki,Kd;	//定义比例、积分、微分系数
	float out;
	
//	float max;
	//int	  c_pid;		
}mpid_t;

mpid_t mrpid,mlpid;
#define NEAR_DIS_PID_DEBUG	1

void init_nearpid_differ(int pwm,uint16_t speed,uint8_t msta)
{
	mpid_t *pid;
	
//	near_lost = 0;
	//右边
	pid = &mrpid;
	pid->SetSpeed =(float)speed;
	pid->ActualSpeed=0.0;
	pid->err=0.0;
	pid->err_last=0.0;
	pid->err_next=0.0;
	pid->out = 0;
  	//sys->pwm = pwm;
	pid->c_near_loop = 30;
	pid->c_near_lost = 0;
	pid->m_nsta = msta;

	//pid->Kp = 0.5f;
	//pid->Kd = 0.5f;		
	pid->Kp = cfg->kp1;//0.5f;
	pid->Kd = cfg->kd1;	
	pid->Kp /=100.0f;
	pid->Kd /=100.0f;

	pid->adc =  &sys->g_sta[6];
	pid->min_adc = cfg->min_right_adc;
	pid->max_adc = cfg->max_right_adc;

	//左边
	pid = &mlpid;
	pid->SetSpeed =(float)speed;
	pid->ActualSpeed=0.0;
	pid->err=0.0;
	pid->err_last=0.0;
	pid->err_next=0.0;
	pid->out = 0;
  	//sys->pwm = pwm;
	pid->c_near_loop = 30;
	pid->c_near_lost = 0;
	pid->m_nsta = msta;
	//pid->Kp = 0.5f;
	//pid->Kd = 0.5f;		
	pid->Kp = cfg->kp1;//0.5f;
	pid->Kd = cfg->kd1;	
	pid->Kp /=100.0f;
	pid->Kd /=100.0f;

	pid->adc =  &sys->g_sta[0];
	pid->min_adc = cfg->min_left_adc;
	pid->max_adc = cfg->max_left_adc;

	sys->nsta = NO_SIDE_NEAR;

	
	
	//pid->Kp = (float)cfg->kp2 / PID_PEC;
	//lpid.Ki = (float)cfg->ki2 / PID_PEC;
	//pid->Kd = (float)cfg->kd2 / PID_PEC;	
#if NEAR_DIS_PID_DEBUG	
	log_printf("init_nearpid(%3.2f,%3.2f)\r\n",pid->Kp,pid->Kd);
#endif
	

}


void init_nearmpid(mpid_t *pid,int pwm,uint16_t speed)
{
	
	pid->SetSpeed =(float)speed;
	pid->ActualSpeed=0.0;
	pid->err=0.0;
	pid->err_last=0.0;
	pid->err_next=0.0;
	
	pid->out = 0;
  	sys->pwm = pwm;
	pid->c_near_loop = 30;
	pid->c_near_lost = 0;
	pid->distok= 0;

	pid->Kp = 0.3f;
	pid->Kd = 0.3f;	
	//pid->dist = motor.c_left_hw;
	pid->c_near = 0;
	
#if NEAR_DIS_PID_DEBUG	
	log_printf("rset,speed=%d,adc=%d\r\n",speed,*pid->adc);
#endif
	

}


/*
差速控制的PID
*/
#define MAX_N_ANGLE1		10		
float		near_angle1[MAX_N_ANGLE1];
uint16_t	c_near_a1=0;

int nearwall_pid_differ(mpid_t *pid/*,uint16_t speed*/)
{
		
	float incrementSpeed;
	float	kp,kd;
	float	err;
	int dist ;
	
	if(pid->c_near_loop ++ < 18)
	{
		if(pid->m_nsta ==1)
		{
			www_idleintel_com();
			return 1;
		}
		else
			return 0;
	}
	dist = motor.c_left_hw;
	www_idleintel_com();
	pid->c_near_loop = 0;
	
//	int max,min;
	//log_printf("[%d,%d,%d,%d,%d]\r\n",*pid->adc,pid->min_adc,pid->m_nsta,sys->g_sta[0],sys->g_sta[6]);
	if( *pid->adc > pid->min_adc) // 有信号
	{
		if(*pid->adc > pid->max_adc)
			*pid->adc = pid->max_adc;
		pid->ActualSpeed = *pid->adc;
		

		if(pid->m_nsta ==2 /*|| err >1000 || err <-1000*/)		//从丢失边进来
		{
			//if(*pid->adc < 1000)
			//	init_nearpid_differ(sys->pwm,900,1);
			//else
			if(*pid->adc > pid->max_adc)
				init_nearmpid(pid,sys->pwm,pid->max_adc-1000);
			else if(*pid->adc < (pid->min_adc+200))
				init_nearmpid(pid,sys->pwm,pid->min_adc+1000);
			else
				init_nearmpid(pid,sys->pwm,*pid->adc);
			c_near_a1 = 0;
			pid->dist = dist;
			pid->distok = 0;
		}
		pid->ActualSpeed = *pid->adc;
		err = pid->SetSpeed-pid->ActualSpeed;
		//near_lost = 0;	
		pid->c_near_lost = 0;
		pid->m_nsta = 1;			//延边状态
		if( c_near_a1 >=MAX_N_ANGLE1)
			c_near_a1 = 0;
		near_angle1[c_near_a1++] = sys->angle;
		pid->c_near++;

		
#if NEAR_DIS_PID_DEBUG	
		if(sys->near_debug)
			log_printf("%3.1f,%d,%3.0f,",sys->angle,*pid->adc,pid->SetSpeed);
#endif	
			
			
		pid->err			= err;//pid->SetSpeed-pid->ActualSpeed;
	

		kp = pid->Kp * pid->err;
		kd = pid->Kd * (pid->err-pid->err_next);

		
		incrementSpeed	=  kp+kd;
#if NEAR_DIS_PID_DEBUG	
		if(sys->near_debug)
			log_printf("(%3.1f,%3.1f,%3.1f)",kp,kd,incrementSpeed);
#endif	

		if(incrementSpeed < -200)
			incrementSpeed = -200;
		if(incrementSpeed > 200)
			incrementSpeed = 200;
			/*
		pid->out			+= incrementSpeed;

		if(pid->out < -200)
			pid->out = -200;
		if(pid->out > 200)
			pid->out = 200;
		*/
		if(*pid->adc == sys->g_sta[6])
		{
			L_FORWRK_PWM = sys->pwm  - (int)incrementSpeed;//pid->out;
			R_FORWRK_PWM = sys->pwm  + (int)incrementSpeed;//pid->out;
		}
		else
		{
			L_FORWRK_PWM = sys->pwm  + (int)incrementSpeed;//pid->out;
			R_FORWRK_PWM = sys->pwm  - (int)incrementSpeed;//pid->out;
		}
		if(L_FORWRK_PWM >=900)
			L_FORWRK_PWM = 900;
		if(L_FORWRK_PWM < 400)
			L_FORWRK_PWM = 400;

		if(R_FORWRK_PWM >=900)
			R_FORWRK_PWM = 900;
		if(R_FORWRK_PWM < 400)
			R_FORWRK_PWM = 400;


#if NEAR_DIS_PID_DEBUG		
		if(sys->near_debug)
			log_printf("%3.1f(%d,%d)\r\n",pid->out,L_FORWRK_PWM,R_FORWRK_PWM);
#endif	

		pid->err_last	= pid->err_next;
		pid->err_next	= pid->err;

		//
		if(pid->distok ==0)
		{
			if(dist > pid->dist && (dist-pid->dist) > 3000)	//超过一扇门的距离
				pid->distok = 1;
		}
		//log_printf("%3.1f,%d,%d,%d\r\n",sys->angle,pid->c_near,(dist-pid->dist),pid->distok);
		if(pid->c_near >= 15 && pid->distok)	//根据目前的频率计算，5个一格
		{

			
			int16_t i;//,c=0,c1=0;
			pid->c_near -=2;
			//float dis=0;
			for(i=0;i<MAX_N_ANGLE1;i++)		
			{		
				if(disfloat(sys->angle,near_angle1[i]) >=6)
				{
					log_printf("not vagl(%3.1f)\r\n",near_angle1[i]);
					break;
				}
			}
		//	log_printf("i=%d",i);
			if(i >=MAX_N_ANGLE1)
			{
				for(i=0;i<4;i++)
				{
					float dis = disfloat(sys->angle, angle_buf[i]);
					if( dis < ANGLE_ADJ_DIS)
					{
						uint16_t ps;
						ps = 40;
							
						illegal_copy_tell_13757122544_gsend(0x02,bf_angle[i],ps); 
						log_printf("adj_a:%3.1f,%d,%d\r\n",sys->angle,bf_angle[i],ps);
						if(dis <2)	
							navigat->t_gyro_adj = sys->t_navi_work;		//刷新时间
						navigat->dis_angle_adj = dis;		//保留最后的差值，用于给上层计算是否校验成功
					//	if(dis > 3.0f)	//大角度，调整较大，则角度重新设置
						{
						//	*agle = NO_ANGLE_V;
						//	log_printf("a_out,%3.1f\r\n");
							
						}
						break;
					}
				}
			}

#endif		
			
		}
		//log_printf("\r\n");

			return 1;
	} else
	{

		pid->m_nsta = 2;
	}
	return 0;

}

int  m_nearwall_pid(void)
{
	if(sys->nsta == NO_SIDE_NEAR)
	{ 
		//log_printf("%d,%d\r\n",sys->g_sta[0],sys->g_sta[6]);
		if(sys->g_sta[0] >= cfg->min_left_adc)		//左边延边
			sys->nsta = LEFT_SIDE_NEAR;
		else if(sys->g_sta[6] >= cfg->min_right_adc)
			sys->nsta = RIGHT_SIDE_NEAR;
		else
			return 0;
	}
	if(sys->nsta == LEFT_SIDE_NEAR)		//左边延边
		return nearwall_pid_differ(&mlpid);
	else if(sys->nsta == RIGHT_SIDE_NEAR)
		return nearwall_pid_differ(&mrpid);
	else
		return 0;
}




pid_t rpid,*pid;

#define PID_PEC	100.0f


extern void pd_gyro_int(int out)
{
/*
	pid_t *pid;
	pid = &lpid;
	pid->SetSpeed =0.0;
	pid->ActualSpeed=0.0;
	pid->err=0.0;
	pid->err_last=0.0;
	pid->err_next=0.0;
	//pid->Kp=0.008f;
	//pid->Ki=0.035f;
	//pid->Kd=0.02f;
	pid->out = out;
*/
//	g_printf("1\r\n");
	pid = &rpid;
	pid->SetSpeed =0.0;
	pid->ActualSpeed=0.0;
	pid->err=0.0;
	pid->err_last=0.0;
	pid->err_next=0.0;
	pid->integral = 0;
	
//	g_printf("2\r\n");

	pid->out = out;

//	pid->max = 20;

	
/*
	lpid.Kp = (float)cfg->kp1 / PID_PEC;
	lpid.Ki = (float)cfg->ki1 / PID_PEC;
	lpid.Kd = (float)cfg->kd1 / PID_PEC;
	*/
	
	//g_printf("3\r\n");

#if GYRO_TO_LIB
	rpid.Kp = (float)(*(mgyro->kp2)) / PID_PEC;
	
//	g_printf("4\r\n");
	rpid.Ki = (float)(*(mgyro->ki2))/ PID_PEC;
	rpid.Kd = (float)(*(mgyro->kd2)) / PID_PEC;
	
//	sys->c_pid_loop = 0;
	
	*(mgyro->rpwm)=*(mgyro->lpwm)=out;
	*(mgyro->pwm)= out;
#else
rpid.Kp = (float)(cfg->kp2) / PID_PEC;
	
//	g_printf("4\r\n");
	rpid.Ki = (float)(cfg->ki2)/ PID_PEC;
	rpid.Kd = (float)(cfg->kd2) / PID_PEC;
	
//	sys->c_pid_loop = 0;
	
	sys->rpwm=sys->lpwm=out;
	sys->pwm= out;
#endif
	

	sys->min_pwm = -130;//-160;

	sys->max_pwm = 130;//160;
	
	sys->setminpwm = 300;//cfg->setminpwm;			//设置为非强制延边做小PWM值
	sys->setmaxpwm = 800;//cfg->setmaxpwm;			//设置为非强制延边做大PWM值	

	//rpid->angle = sys->angle;

	//if(pid_printf == 0)
	//g_printf("right p=%f,i=%f,d=%f\r\n",rpid.Kp,rpid.Ki,rpid.Kd);	
	//pid_printf = 1;
	//msyst->c_yaw = 0;
	//sys->setminpwm = 100;//cfg->setminpwm;			//设置为非强制延边做小PWM值
	//sys->setmaxpwm = 900;//cfg->setmaxpwm;			//设置为非强制延边做大PWM值	

}


extern int p_gyro_calce(float set_speed)
{
	float incrementSpeed;
	float	kp,kd,ki;
//	float dis;
	www_idleintel_com();
	


//	sys->walk_pid_debug =1;

	
	pid->SetSpeed = format_agle(set_speed, ANGLE_360);
	pid->ActualSpeed = format_agle(sys->angle, ANGLE_360);
	//g_printf("%3.1f,%3.1f\r\n",set_speed,now_speed);
#if PID_DEBUG	
	//if(sys->walk_pid_debug==10)
	//	sys->walk_pid_debug = 1;
	if(sys->walk_pid_debug == 1)
		log_printf("p(%3.1f,%3.1f)",set_speed,sys->angle);
#endif	


	pid->err			= pid->SetSpeed-pid->ActualSpeed;
		

	if(pid->err > 180)
		pid->err = pid->err - 360;
	if(pid->err <-180)
		pid->err += 360;

#if PID_DEBUG	
	if(sys->walk_pid_debug == 1)
		log_printf("(%3.5f)",pid->err);
#endif

	
	pid->integral +=pid->err;
	kp = pid->Kp * pid->err;
	kd = pid->Kd * (pid->err-pid->err_next);
//	kd = pid->Kd * (pid->err_next - pid->err);
	ki = pid->Ki * pid->integral;

	incrementSpeed	=  kp+ ki + kd;
#if PID_DEBUG	
	if(sys->walk_pid_debug == 1)
		log_printf("(%3.1f,%3.1f,%3.1f,%3.1f) ",kp,ki,kd,incrementSpeed);
#endif	

/*
		if(incrementSpeed < -160)
			incrementSpeed = -160;
		if(incrementSpeed > 160)
			incrementSpeed = 160;
	
*/
	if(incrementSpeed < sys->min_pwm)
			incrementSpeed = sys->min_pwm;
		if(incrementSpeed > sys->max_pwm)
			incrementSpeed = sys->max_pwm;
	
	pid->out			+= incrementSpeed;
		
	sys->lpwm	= sys->pwm  + incrementSpeed;
	sys->rpwm	= sys->pwm  - incrementSpeed;
///强制沿边需要这个条件
//sys->setminpwm = cfg->setminpwm;
	if(sys->lpwm > sys->setmaxpwm)			//sys->setmaxpwm setminpwm  
		sys->lpwm = sys->setmaxpwm;
	if(sys->rpwm > sys->setmaxpwm)
		sys->rpwm = sys->setmaxpwm;
	if(sys->lpwm < sys->setminpwm)			/////2018-11-03add
		sys->lpwm = sys->setminpwm;
	if(sys->rpwm < sys->setminpwm)
		sys->rpwm = sys->setminpwm;
#if PID_DEBUG		
	if(sys->walk_pid_debug == 1)
		log_printf("%3.1f,%d,%d,%d\r\n",incrementSpeed,sys->pwm,sys->rpwm,sys->lpwm);
#endif	

	pid->err_last	= pid->err_next;
	pid->err_next	= pid->err;
//	g_printf("%3.1f,%3.1f\r\n",now_speed,incrementSpeed);
	return (int)pid->out;
}


int16_t dock_speed_pid_ctrl(uint16_t min,uint16_t max,uint8_t type)
{

	static int  c =0;//,c_zero=0;
	static int lhw1=0;//,lhw2=0;
	int hw1=0;//,hw2=0;
	static int count=0;
//	int pid_pwm;
	if(type ==0)
	{
		count = c = lhw1 = motor.c_left_hw=0 ;
		//c_zero = 0;
		log_printf("speed_pid_ctrl reset\r\n");
		return 0;
	}

	if(c++>20)
	{
		
		c=0;
	
		if(charge_info.c_midle >=3)		//·￠???D??
		{
			min = 20;
			max = 50;
		}
		else
		{
			//min = 70;
			//max = 150;
			min = 20;				//2018-10-18 jzz 
			max = 50;
		}
		
		hw1 = motor.c_left_hw - lhw1;
		lhw1 = motor.c_left_hw ;
		
		if(count >=5 && count <=9)
		{
			if(charge_info.c_midle)
			{

				*left_pwm	= sys->left_pwm = 700;
				*right_pwm	= sys->right_pwm = 700;

			}/*else if(charge_info.found_dock == 0)
			{	
				*left_pwm	= sys->left_pwm = 550;
				*right_pwm	= sys->right_pwm = 550;
			}*/else
			{
				*left_pwm	= sys->left_pwm = 620;
				*right_pwm	= sys->right_pwm = 620;
			}
		}
		count++;
		if(count <=10)
			return 0;
//		log_printf("hw=%d,%d,%d\r\n",hw1,motor.c_left_hw,sys->pwm);
			if(hw1 < min)
		{
			sys->pwm -=5;
//			log_printf("-hw=%d,%d,",hw1,sys->pwm);
			
				
		}else if(hw1 > max)
		{
			sys->pwm +=5;
//			log_printf("+hw=%d,%d,",hw1,sys->pwm);

		}else
//			log_printf("hw=%d,%d,%d,",hw1,motor.c_left_hw,sys->pwm);
		if(sys->pwm < 500)
				sys->pwm = 500;
		else if(sys->pwm >870)
			sys->pwm = 870;
		

	}	
	return 0;

}


