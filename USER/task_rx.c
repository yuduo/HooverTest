

#include "sys.h"



void proc_rx_msg(uint8_t *buf,uint8_t len)
{
	ctrl_t *ctrl; 
	pid_set_t  *pid_set;
	rx_cfg_t   *rxCfg;
	int16_t i;
//	float dt;
//	int tmr=0;
	char	dir;
	//for(i = 0;i<len;i++)
	//	log_printf("%02X ",buf[i]);
	sys->t_idle = 0;
	sys->power_on = 1;		//有串口命令就poweron
	switch (buf[0])
	{	//log_printf("buf[0]=%d,dir = %d,pwm=%d,speed=%d,hw=%d\r\n",buf[0],dir,ctrl->pwm,ctrl->speed,ctrl->hw);
		case 0x01:				//速度测试
			//gyro_offset_manage(500);
			
		
			ctrl = (ctrl_t *)buf;
			dir = ctrl->crc;
			if(ctrl->pwm == 551)		//关闭电机
			   sys->shut_down_motor = 1;
			else
				sys->shut_down_motor = 0;
				
			cfg->go_forward_pwm = ctrl->pwm;
			cfg->go_route_pwm	= ctrl->hw;
			log_printf("ctrl msg sta=%d,pwm=%d,router pwm=%d\r\n",dir,cfg->go_forward_pwm,cfg->go_route_pwm);
			//init_sys_sta(dir);
			sys->sState = dir;	

			
			break;
		case 0x02:		//PID设置
			pid_set = (pid_set_t *)buf;
			cfg->kp1 = pid_set->kp1;
			cfg->ki1 = pid_set->ki1;
			cfg->kd1 = pid_set->kd1;

			cfg->kp2 = pid_set->kp2;
			cfg->ki2 = pid_set->ki2;
			cfg->kd2 = pid_set->kd2;
			log_printf("p1=%d,i1=%d,d1=%d,p2=%d,i2=%d,d2=%d\r\n",cfg->kp1,cfg->ki1,cfg->kd1,cfg->kp2,cfg->ki2,cfg->kd2);
			save_cfg();

			break;
		case 0x03	:		//读取配置信息
			cfg->msg = 0x03;
			
			RESET_GYRO();
			usart2_init_info();
			//reset_gyro(0);
			//log_printf("ird_en=%d\r\n",sys->ird_en);
			sys->t_idle = 0;
			tx_msg((uint8_t *)&cfg->msg,128);
			log_printf("\r\n");
		
			break;
		case 4:			//配置信息			
			rxCfg = (rx_cfg_t *)buf;
			if(rxCfg->cfg[0] >5 && rxCfg->cfg[0]<100)
			{
				cfg->ajust_agle = rxCfg->cfg[0];
				sys->ajust_agle = (float)cfg->ajust_agle / 10.0f;
				log_printf("ajs=%d,%f,",cfg->ajust_agle,sys->ajust_agle);
			}
			//midle adc
			if(rxCfg->cfg[1] >100 && rxCfg->cfg[1] < 4000)
			{
				cfg->midle_adc = rxCfg->cfg[1];
				cfg->slow_adc = cfg->midle_adc - 500;
				log_printf("mid adc=%d,",cfg->midle_adc);
			}	
			//side adc
			if(rxCfg->cfg[2] < 500)
			{
				cfg->stop_timr= rxCfg->cfg[2];
				log_printf("stop_timr=%d,",cfg->stop_timr);
			}	

			if(rxCfg->cfg[3] >10 && rxCfg->cfg[3] < 4000)
			{
				cfg->min_left_adc= rxCfg->cfg[3];
				log_printf("min_left_adc=%d,",cfg->min_left_adc);
			}	

			if(rxCfg->cfg[4] >10 && rxCfg->cfg[4] < 4000)
			{
				cfg->lock_left_adc= rxCfg->cfg[4];
				log_printf("lock_left_adc=%d,",cfg->lock_left_adc);
			}	
			if(rxCfg->cfg[5] >10 && rxCfg->cfg[5] < 4000)
			{
				cfg->max_left_adc= rxCfg->cfg[5];
				log_printf("max_left_adc=%d,",cfg->max_left_adc );
			}	
			if(rxCfg->cfg[6] >10 && rxCfg->cfg[6] < 4000)
			{
				cfg->side_left_adc= rxCfg->cfg[6];
				log_printf("side_left_adc=%d,",cfg->side_left_adc );
			}
			if(rxCfg->cfg[7] >10 && rxCfg->cfg[7] < 4000)
			{
				cfg->lost_left_adc= rxCfg->cfg[7];
				log_printf("lost_left_adc=%d,",cfg->lost_left_adc );
			}	
			
			if(rxCfg->cfg[8] >10 && rxCfg->cfg[8] < 4000)
			{
				cfg->min_right_adc= rxCfg->cfg[8];
				log_printf("min_right_adc=%d,",cfg->min_right_adc);
			}	

			if(rxCfg->cfg[9] >10 && rxCfg->cfg[9] < 4000)
			{
				cfg->lock_right_adc= rxCfg->cfg[9];
				log_printf("lock_right_adc=%d,",cfg->lock_right_adc);
			}	
			if(rxCfg->cfg[10] >10 && rxCfg->cfg[10] < 4000)
			{
				cfg->max_right_adc= rxCfg->cfg[10];
				log_printf("max_right_adc=%d,",cfg->max_right_adc );
			}	
			if(rxCfg->cfg[11] >10 && rxCfg->cfg[11] < 4000)
			{
				cfg->side_right_adc= rxCfg->cfg[11];
				log_printf("side_right_adc=%d,",cfg->side_right_adc );
			}
			if(rxCfg->cfg[12] >10 && rxCfg->cfg[12] < 4000)
			{
				cfg->lost_right_adc= rxCfg->cfg[12];
				log_printf("lost_left_adc=%d,",cfg->lost_right_adc );
			}
			
			if(rxCfg->cfg[13] >10 && rxCfg->cfg[13] < 4000)
			{
				cfg->m_left_adc= rxCfg->cfg[13];
				log_printf("m_left_adc=%d,",cfg->m_left_adc );
			}
			if(rxCfg->cfg[14] >10 && rxCfg->cfg[14] < 4000)
			{
				cfg->m_right_adc= rxCfg->cfg[14];
				log_printf("m_right_adc=%d,",cfg->m_right_adc );
			}
			i=15;
			if(rxCfg->cfg[i] >100 && rxCfg->cfg[i] < 4000)
			{
				cfg->max_left2_adc= rxCfg->cfg[i];
				log_printf("max_left2_adc=%d,",cfg->max_left2_adc );
			}
			
			i++;
			if(rxCfg->cfg[i] >100 && rxCfg->cfg[i] < 4000)
			{
				cfg->max_right2_adc= rxCfg->cfg[i];
				log_printf("max_right2_adc=%d,",cfg->max_right2_adc );
			}
			i++;
			int16_t v;
			v = (int16_t)rxCfg->cfg[i];
			if( v < 300)
			{
				cfg->gyro_ofs1= v;
				log_printf("gyro_ofs1=%d,%d,",cfg->gyro_ofs1,rxCfg->cfg[i] );
			}
			i++;
			
			v = (int16_t)rxCfg->cfg[i];
			if(v < 300)
			{
				cfg->gyro_ofs2= v;
				log_printf("gyro_ofs2=%d,%d,set=%d,%d",cfg->gyro_ofs2,rxCfg->cfg[i],cfg->gyro_ofs1,cfg->gyro_ofs2 );
				illegal_copy_tell_13757122544_gsend(GYRO_SET_OFS_MSG,cfg->gyro_ofs1,cfg->gyro_ofs2);
			}
			
			i++;
			if(rxCfg->cfg[i] >2 && rxCfg->cfg[i] < 200)
			{
				cfg->c_pid_near= rxCfg->cfg[i];
				log_printf("c_pid_near=%d,",cfg->c_pid_near );
			}
			i++;
			if(rxCfg->cfg[i] >10 && rxCfg->cfg[i] < 500)
			{
				cfg->a_pid_near= rxCfg->cfg[i];
				log_printf("a_pid_near=%d,",cfg->a_pid_near );
			}
			///
			i++;
			if(rxCfg->cfg[i] >699 && rxCfg->cfg[i] < 1001)			//875
			{
				cfg->setsidemaxpwm= rxCfg->cfg[i];
				log_printf("setsidemaxpwm=%d,",cfg->setsidemaxpwm );
			}	
			i++;
			if(rxCfg->cfg[i] >299 && rxCfg->cfg[i] < 601)			//500
			{
				cfg->setsideminpwm= rxCfg->cfg[i];
				log_printf("setsideminpwm=%d,",cfg->setsideminpwm );			
			}	
			i++;
			if(rxCfg->cfg[i] >0 && rxCfg->cfg[i] < 60)				//20
			{
				cfg->setlostturnangle= rxCfg->cfg[i];
				log_printf("setlostturnangle=%d,",cfg->setlostturnangle );
			}	
			i++;
			if(rxCfg->cfg[i] >0 && rxCfg->cfg[i] < 101)				//7
			{
				cfg->max_near_lost= rxCfg->cfg[i];
				log_printf("max_near_lost=%d,",cfg->max_near_lost );
			}		
			i++;
			if(rxCfg->cfg[i] >0 && rxCfg->cfg[i] < 101)				//7
			{
				cfg->dock_delay = rxCfg->cfg[i];
				log_printf("dock_delay=%d,",cfg->dock_delay );
			}		
			i++;
			if(rxCfg->cfg[i] >0 && rxCfg->cfg[i] < 101)				//7
			{
				cfg->dock_l_count = rxCfg->cfg[i];
				log_printf("dock_l_count=%d,",cfg->dock_l_count );
			}		
			i++;
			if(rxCfg->cfg[i] >0 && rxCfg->cfg[i] < 101)				//7
			{
				cfg->dock_r_count = rxCfg->cfg[i];
				log_printf("dock_r_count=%d,",cfg->dock_r_count );
			}
			
			save_cfg();
			log_printf("\r\n");//ok
			break;
		case 5:
			rxCfg = (rx_cfg_t *)buf;
		
			if( rxCfg->cfg[1] <=100)
			{
				illegal_copy_tell_13757122544_gsend(0x03,cfg->gyro_ajust,cfg->gyro_ajust);	
				cfg->gyro_ajust = rxCfg->cfg[1];
				log_printf("gyro_ajs=%d\r\n",cfg->gyro_ajust);
				save_cfg();	
				
			}
			break;
				
		case 6:
			memcpy((char *)&sys_debug,(char *)buf,SYS_DEBUG_LEN);

			break;			
		case 7:		//控制命令
			ctrl = (ctrl_t *)buf;
			dir = ctrl->crc+1;
			log_printf("buf[0]=%d,dir = %d,pwm=%d,speed=%d,hw=%d\r\n",buf[0],dir,ctrl->pwm,ctrl->speed,ctrl->hw);

			if(dir == 6)		//自测
			{
				if(sys->sState == SYS_TEST_SEFT)
					sys->sState = SYS_IDLE;
				else
				{
					sys->sState = SYS_TEST_SEFT;
					test_seft_task(0);
					ny3p_play(VOICE_DIDI);
					delay_ms(500);
					ny3p_play(VOICE_DIDI);
					delay_ms(500);
					ny3p_play(VOICE_DIDI);
					delay_ms(500);
					
				}
				break;
			}	
			if(ctrl->speed ==1)
			{
				log_printf("dir = %d,pwm=%d,speed=%d,hw=%d\r\n",dir,ctrl->pwm,ctrl->speed,ctrl->hw);
				MOTOR_POWER_ON();
				MOTOR_CTRL1(NORM_SIDE_PWM_L,NORM_SIDE_PWM_R,NORM_MID_PWM,sys->dust_pwm_value)
				sys->sState = SYS_NAVIGAT;
				
				navigat->angle = 0;
				motor_go_forwark(0,NO_SIDE_NEAR,navi_go_forwark_check/*NULL*/);		
				log_printf("%d,%d,%d\r\n",sys->g_sta[2],sys->g_sta[3],sys->g_sta[4]);
				turn_to_deg_v(ctrl->hw);
				sys->sState = SYS_IDLE;
				break;
			}
			if(ctrl->speed ==2)
			{
				if(ctrl->pwm == 600)
					sys->near_debug = 1;
				else
					sys->near_debug = 0;
				go_and_back_test();
				break;
			}
			if(ctrl->speed == 3 || ctrl->speed == 4)
			{
				turn_deg_test(ctrl->speed,ctrl->hw);
				break;
			}
			if(ctrl->speed == 6)		//延边测试
			{
			   sys->sState_auxi = 0;
			//	proc_random_task();      //随机模式
			  // walk_tesk_for_whele();
				//nearwall_adj_test();
				break;
			}
			if(ctrl->speed == 7)		//对地红外测试
			{
				sys->bottom_ir = 1;
				break;
			}
			if(ctrl->speed == 8)		//直行测试，测试坐标
			{
				log_printf("dir = %d,pwm=%d,speed=%d,hw=%d\r\n",dir,ctrl->pwm,ctrl->speed,ctrl->hw);
				sys->shut_down_motor = 1;
				navigat_init(0);
	
				MOTOR_POWER_ON();
				MOTOR_CTRL1(NORM_SIDE_PWM_L,NORM_SIDE_PWM_R,NORM_MID_PWM,sys->dust_pwm_value)
				sys->sState = SYS_NAVIGAT;
				//www_idleintel_com();
				//navigat->angle = sys->angle;
				//
				//motor_go_forwark(1000,NO_SIDE_NEAR,NULL);		
				robot_coord_test(ctrl->pwm,ctrl->hw);
				//log_printf("%d,%d,%d\r\n",sys->g_sta[2],sys->g_sta[3],sys->g_sta[4]);
				sys->sState = SYS_IDLE;
				break;
			}
			if(ctrl->speed == 9)			//延边测试
			{
				#if 1
				log_printf("dir = %d,pwm=%d,speed=%d,hw=%d\r\n",dir,ctrl->pwm,ctrl->speed,ctrl->hw);
				if (ctrl->pwm == 600)			///600关电机
				{
					//MOTOR_POWER_ON();
					sys->shut_down_motor = 1;
					//MOTOR_CTRL1(NORM_SIDE_PWM_L,NORM_SIDE_PWM_R,NORM_MID_PWM,sys->dust_pwm_value);
				}
				else
				{					
					sys->shut_down_motor = 0;
					MOTOR_POWER_ON();
					//MOTOR_CTRL(NORM_SIDE_PWM_L,NORM_SIDE_PWM_R,NORM_MID_PWM,sys->dust_pwm_value);
				}
				navigat_init(0);

					
				sys->sState = SYS_NAVIGAT;
				//robot_turn_deg(GO_RIGTH,DEG_TURN_PWM,12);
				if (ctrl->pwm == 601)
				{
					sys->near_debug = 0;
#if PID_DEBUG					
				sys->walk_pid_debug = 0;
#endif					
				}else
				{
				sys->near_debug = 1;
#if PID_DEBUG									
				sys->walk_pid_debug = 1;
#endif						
				}
				motor_go_edgeways(RIGHT_SIDE_NEAR,0,0,GO_NEAR_TYPE_NO,0);
				if(sys->sState == SYS_IDLE)//2018-05-26 jzz
					return;

				
				STOP_ALL_MOTOR();
				sys->sState = SYS_IDLE;
				#else
				sys->sState = SYS_RANDOM;
				#endif
				break;
			}
			if(ctrl->speed == 10)			//延边坐标测试
			{
				sys->shut_down_motor = 0;
				navigat_init(0);
	
				//MOTOR_POWER_ON();
				//MOTOR_CTRL1(NORM_SIDE_PWM_L,NORM_SIDE_PWM_R,NORM_MID_PWM,sys->dust_pwm_value)
				sys->sState = SYS_NAVIGAT;
				motor_coord_run_test(LEFT_SIDE_NEAR);
				//robot_coord_test(ctrl->pwm,ctrl->hw);
				//log_printf("%d,%d,%d\r\n",sys->g_sta[2],sys->g_sta[3],sys->g_sta[4]);
				sys->sState = SYS_IDLE;
					break;			
				
			}
			if(ctrl->speed == 11)			//延边坐标测试
			{
#if INTO_CHECK_SENSER_GETOUT_TROUBLE
#if GET_OUT_OF_TROUBLE_EN					//脱困处理
				uint8_t tempsta_l,tempsta_r,mState_b;
				uint16_t left_pwm_b,right_pwm_b;
				float agle_b;
				sys->sState = SYS_NAVIGAT;
				MOTOR_POWER_ON();
				sys->motor_primary_locked = 1;
				if(sys->motor_primary_locked == 1)		//左右轮好像锁住了
				{
					sys->motor_primary_locked = 0;
					////备份之前的角度 PWM 方向
					log_printf("[senser]backup--mState:%d,left_pwm_b:%d,right_pwm_b:%d,agle_b:%5.3f\r\n",sys->mState,sys->left_pwm,sys->right_pwm,sys->angle);
					mState_b = sys->mState;				//电机状态
					left_pwm_b = sys->left_pwm;
					right_pwm_b = sys->right_pwm;		//
					agle_b = sys->angle;
					////		
					log_printf("[senser]get out of trouble start\r\n");
					//左旋转45度,有旋转45度
					tempsta_l = robot_turn_deg_getout_trouble(GO_LEFT,DEG_TURN_PWM,GETOUT_TROUBLE_ANGLE);
					if (tempsta_l == 0)		//失败
					{
						log_printf("[senser]robot_turn_left_err\r\n");

						tempsta_r = robot_turn_deg_getout_trouble(GO_RIGTH,DEG_TURN_PWM,GETOUT_TROUBLE_ANGLE);
						if (tempsta_r == 0)
						{
							log_printf("[senser]robot_turn_right_err\r\n");
							log_printf("[senser]motor_locked_flag = 1 \r\n");
							sys->motor_locked_flag = 1;
						}	
						else if(tempsta_r == 1)
						{
							log_printf("[senser]robot_turn_right_ok\r\n");
							sys->mState = mState_b; 			//电机状态
							sys->left_pwm = left_pwm_b; 		//
							sys->right_pwm = right_pwm_b;		//		
							sys->angle = agle_b ;				//
							log_printf("[senser]recovery--mState:%d,left_pwm_b:%d,right_pwm_b:%d\r\n",sys->mState,sys->left_pwm,sys->right_pwm);
						}			
					}
					else if (tempsta_l == 1)
					{
						log_printf("[senser]robot_turn_left_ok\r\n");
						sys->mState = mState_b;				//电机状态
						sys->left_pwm = left_pwm_b;			//
						sys->right_pwm = right_pwm_b;		//		
						sys->angle = agle_b ;				//
						log_printf("[senser]recovery--mState:%d,left_pwm_b:%d,right_pwm_b:%d\r\n",sys->mState,sys->left_pwm,sys->right_pwm);
					}
					///恢复之前的角度 PWM 和方向		
				}	
#endif	

#endif			
				//MOTOR_POWER_ON();
				sys->sState = SYS_IDLE;

					break;			
				
			}			
			if(ctrl->speed == 12)			//延边测试
			{
				sys->sState = SYS_RANDOM;
				break;
			}
			if(dir == 5)		//电机测试
			{
				MOTOR_POWER_ON();
				timer1_init();					//时钟1，控制两个边刷
				timer2_init();	
				MOTOR_CTRL(ctrl->pwm,ctrl->hw,0,ctrl->speed);	
				break;
			}
			/*
			init_sys_status(SYS_NAVIGAT,SYS_IDLE);
			//robot_coord_test(ctrl->pwm);
			go_and_back_test();
			sys->sState = SYS_IDLE;
			break;
			//测试电机
			
			timer1_init();					//时钟1，控制两个边刷
			timer2_init();					//控制中扫电机和吸尘电机
			if(dir == GO_BACK)
			{
				log_printf("stop all motor\r\n");
				MOTOR_CTRL(0,0,0,0);	
			}
			else if(dir == GO_RIGTH)
			{
				//MOTOR_CTRL(NORM_SIDE_PWM,NORM_SIDE_PWM,NORM_MID_PWM,FOCUS_DUST_PWM);	
				log_printf("set midle=%d\r\n",500);
				MOTOR_CTRL(0,0,500,0);	
			}else if(dir == GO_LEFT)
			{
				log_printf("set dust=%d\r\n",500);
				MOTOR_CTRL(0,0,0,500);	
			}else if(dir == GO_FORWARD)
			{
				log_printf("motor ctl,left=%d,right=%d,mid=%d\r\n",ctrl->pwm,ctrl->hw,ctrl->speed);
				MOTOR_CTRL(ctrl->pwm,ctrl->hw,0,ctrl->speed);	
			}
		
			break ;
			*/	
			//illegal_copy_tell_13757122544_gsend(0x02,90,ANGLE_PEER);	
			//log_printf("set ok\r\n");
			//break;
			//测试激光
			MOTOR_POWER_ON();
			//MOTOR_CTRL(DOCK_SIDE_PWM,DOCK_SIDE_PWM,DOCK_DUST_PWM,DOCK_MID_PWM);	
			
			/*sys->sState = SYS_NAVIGAT;


			navigat->x_org = navigat->y_org = 0;
			navigat->x_org_f = navigat->y_org_f = 0;

			navigat->y_org_f = RADIUS;

			navigat->tx = navigat->ty = 100;
			
			
			log_printf("----------(%d,%d,)\r\n",X_NOW,Y_NOW);
			turn_to_deg(90);
			sys->sState = SYS_IDLE;
			break;
			*/
			log_printf("ctlr=7,dir=%d\r\n",dir);

			

			
			if(dir  > 4)
			{
				log_printf("run sta error,sta=%d\r\n",ctrl->crc);
				break;
			}
			if(ctrl->pwm > 1000)
			{
				log_printf("pwm error,pwm=%d\r\n",ctrl->pwm);
				break;
			}
			
			if(ctrl->pwm == 0)
			{
				  u8 irData_bak[6];
					motor_run(GO_STOP, 0, 0, 0);
					sys->sState =SYS_DOCK;
					while(1)
					{
						delay(600);
						read_ir_data(irData_bak,0);	
						log_printf("(%d,%d,%d),%d,%d\r\n\r\n",irData_bak[IR_L_PIN_NUM], irData_bak[IR_M_PIN_NUM], irData_bak[IR_R_PIN_NUM],DOCK_DETECT(),0);
					}
			}
			else
			{

		
				if(dir == GO_FORWARD || dir == GO_BACK)
				{
					if(dir == GO_FORWARD)
					{
						pd_gyro_int(ctrl->pwm);
						gyro_offset_manage(10);
//						sys->yaw = 0;
					}
					motor_run(dir, ctrl->pwm, 0, 0);
					//gyro_offset_manage(500);
				//	motor.c_front_hw=motor.c_left_hw=motor.c_right_hw=0;
				//	sys->yaw = 0;
					
				}
				/*else if(dir == GO_LEFT || dir == GO_RIGTH)
				{
					//motor_turn(dir,180);
					
				}
				*/
				else if(dir == GO_LEFT || dir == GO_RIGTH)
				{
					//motor_turn_360(dir);
					//motor_turn(dir,ctrl->speed);
					if(ctrl->pwm == 100)
						gyro_move_test(dir,ctrl->hw);
					else
					if(ctrl->pwm == 200)
					{	
						int i;
						MOTOR_POWER_ON();
						MOTOR_CTRL(NORM_SIDE_PWM_L,NORM_SIDE_PWM_R,NORM_MID_PWM,sys->dust_pwm_value)
						sys->sState =SYS_NAVIGAT;
						if(dir == GO_LEFT)
						{
							for(i=0;i<ctrl->hw;i++)
							{
								log_printf("==========i=%d==========\r\n",i);
								turn_to_deg(90);
								delay_ms(100);
								turn_to_deg(180);
								delay_ms(100);
								turn_to_deg(90);
								
								delay_ms(100);
								turn_to_deg(0);
								
							}
							MOTOR_POWER_OFF();

						}else 
						{
							for(i=0;i<ctrl->hw;i++)
							{
								log_printf("==========i=%d==========\r\n",i);
								turn_to_deg(270);
								delay_ms(100);
								turn_to_deg(180);
								delay_ms(100);
								turn_to_deg(270);
								
								delay_ms(100);
								turn_to_deg(0);
								
							}
							MOTOR_POWER_OFF();

						}
						sys->sState = SYS_IDLE;
					}else	
					
					if(ctrl->speed >0 && ctrl->speed <=360)
					{
						log_printf("turn to deg,ajust=%d,deg=%d\r\n",sys->ajust_agle,ctrl->speed);
						
						//MOTOR_POWER_ON();
						MOTOR_CTRL(0,0,0,0);
						turn_to_deg(ctrl->speed);
					}
					else
					{

						
						{
							if(ctrl->pwm == 600)		//关闭电机
							   sys->shut_down_motor = 1;
							else
								sys->shut_down_motor = 0;
							motor_turn_circle(dir,ctrl->pwm,ctrl->hw);
							MOTOR_POWER_OFF();
						}
					}
					
						
				}
				
				
			
			}

			/*
			if((ctrl->crc+1 ) == GO_LEFT || (ctrl->crc+1) == GO_RIGTH)
			{
				motor_turn_deg(ctrl->crc+1, 90);
			}else
			{
				motor_run(ctrl->crc+1, ctrl->pwm, 0, 0);
			}
			*/
			break;	
		case 8://重启
			log_printf("system reset!\r\n");
			TargetSysReset();
			break;	
		case 9://????
			log_printf("Entering testing mode!\r\n");
			for(uint8_t i = 1;i < 4;i ++)
			{
				if(buf[i] != i)
				{
					log_printf("Code data error! Ignored!");
					return;
				}
			}
			tm_mode = buf[4];//μú5??×??úê??￡ê?????0?a°?2a?￡ê?,1?a???ú2aê??￡ê?
			if(tm_mode != 1)tm_mode = 0;
			sys->sState = SYS_TEST_BD;
			ny3p_play(VOICE_FAC_MODE);
			break;	
		case TPC_SETSN:
			if(len < 20)//如果数据长度小于20，将被视作无效数据
			{
				tx_sn(TPC_SETSN,NULL);
				log_printf("error:data len = %d\r\n",len);
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
					tx_sn(TPC_SETSN,NULL);
					//log_printf("\r\nerror:chksum = %02X-%02X\r\n",n_chksum,buf[22]);
					//sn_print(buf + 1, 21);
					//log_printf("\r\n");
					break;
				}
			}
			break;
		case TPC_GETSN://主板向PC发送主板的SN码
			//ny3p_play(VOICE_DIDI);
			tx_sn(TPC_GETSN,cfg->sn);
			break;
		default:
			log_printf("error cmd\r\n");
			break;
	}
}

#define MAX_RX_BUFF	256
uint8_t rx_sta=0;
uint8_t rx_len;
uint8_t rx_buf[MAX_RX_BUFF];

void rx_usart(uint8_t chr)
{
		//log_printf("%02X ",chr);
	switch(rx_sta)
	{
		case 0:
			if(chr == 0x7E)
			{
				rx_sta=1;
			}
			break;
		case 1:
			if(chr ==0x5D)
			{
				rx_sta=2;
				memset(rx_buf,0x00,MAX_RX_BUFF);
				rx_len=0;
			}
			else
			 rx_sta =0;
			break;
		case 2:
			rx_buf[rx_len++]=chr;
			if(rx_len >=MAX_RX_BUFF)
			{
				log_printf("rx error\r\n");
				rx_len=0;
				rx_sta=0;
				break;
			}
			//7E 5D 05 00 6D 7D 
			if(chr == 0x7D && rx_buf[rx_len-2]==0x6D)	//接受完毕
			{
				proc_rx_msg(rx_buf,rx_len);
				rx_len=0;
				rx_sta=0;
			}
			break;

		default:
				rx_len=0;
				rx_sta=0;
				break;
			
		
	}
}
void tx_robot_msg(void)
{
	struct robot_msg_t rmsg;

	//rmsg.magic 	= MAGIC;
	rmsg.msg 	= 0x0A;

	//红外的值
	rmsg.data[0] = sys->g_sta[0];
	rmsg.data[1] = sys->g_sta[1];
	rmsg.data[2] = sys->g_sta[2];
	rmsg.data[3] = sys->g_sta[3];
	rmsg.data[4] = sys->g_sta[4];
	rmsg.data[5] = sys->g_sta[5];
	rmsg.data[6] = sys->g_sta[6];
	
	rmsg.data[7] = sys->g_buton[0][0];
	rmsg.data[8] = sys->g_buton[0][1];
	rmsg.data[9] = sys->g_buton[0][2];
	rmsg.data[10] = sys->g_buton[1][0];
	rmsg.data[11] = sys->g_buton[1][1];
	rmsg.data[12] = sys->g_buton[1][2];

	tx_msg((uint8_t *)&rmsg.msg,64);

}

void test_seft_task(uint8_t type)
{
	static int32_t	nt=0;
	static int32_t	kt = 0;
	if(type ==0)
	{
		nt = kt=0;
		return ;
	}

	if(TIM5->CNT >=20000)
	{
		tx_robot_msg();
		TIM5->CNT = 0;
		if(kt >0)
			kt --;
		
		if(sys->g_sta[0] > 700 || sys->g_sta[3]>700 || sys->g_sta[6] >  700)
		{
			nt = 	3000;		//叫1分钟
		}
		if(nt >0 && nt < 8000)
		{
			nt -- ;
			
			if(kt  ==0)
			{
				ny3p_play(VOICE_DIDI);
				kt = 5;
			}
			
		}else
			nt = 0;
	}
	
	
}

void tx_msg(uint8_t *buff,int len)
{
	uint8_t tmp[2];
	tmp[0] = 0x7E;
	tmp[1] = 0x5D;
	uasrt_write(tmp,2);
	uasrt_write(buff, len);
	tmp[0] = 0x6D;
	tmp[1] = 0x7D;
	uasrt_write(tmp,2);	
	
	
}
