
/*
�������߷��ͽ���
*/

#include "sys.h"

#if TARGE_RUN

XY_T xy[MAX_TARGE_XY];
#endif
//����Ƿ���Ҫ�������߷�

//�������߷����
int16_t run_ajust_check(void)
{
//	int idx;
//	TOBST *t_obst;

//	xy[0].x = 1;
	//�ر���ʧ�������˳�����ʧ5�Σ�������һ�����ӣ����˳�
	if(navigat->adj_nsta == LEFT_SIDE_NEAR )		//���ر�
	{
		if(LEFT_ADC() < IRD_LEFT_ADJ_MIN)		
		{
			//log_printf("adj lost l:%d\r\n",LEFT_ADC());
			if(navigat->adj_c_lost++ > LOST_NUBUER && WALK_DIST() >= HW_GRID)
			{
				motor_run(GO_STOP,0,0,0);
				if(LEFT_ADC() < SIDE_LOST_LEFT)	
					navigat->adj_run_ret = ADJ_SIDE_LOST;	//�������߷���ȷ
				else
					navigat->adj_run_ret = ADJ_FAIL;
				log_printf("[run_ajust_check]lost12,ret=%d,ir=%d\r\n",navigat->adj_run_ret,LEFT_ADC());
				return 0;
			}
		}else
			navigat->adj_c_lost = 0;
	}
	if(navigat->adj_nsta == RIGHT_SIDE_NEAR )		//���ر�
	{
		if(RIGHT_ADC() < IRD_RIGHT_ADJ_MIN)		
		{
		//	log_printf("adj lost r:%d\r\n",RIGHT_ADC());
			if(navigat->adj_c_lost++ > LOST_NUBUER && WALK_DIST()  >= HW_GRID)
			{
				motor_run(GO_STOP,0,0,0);
				if(RIGHT_ADC() < SIDE_LOST_RIGHT)	
					navigat->adj_run_ret = ADJ_SIDE_LOST;	//�������߷���ȷ
				else
					navigat->adj_run_ret = ADJ_FAIL;
				log_printf("[run_ajust_check]lost13,ret=%d,ir=%d\r\n",navigat->adj_run_ret,RIGHT_ADC());
				return 0;
			}
		}else
		  navigat->adj_c_lost = 0;
	}
							
	
	return 1;
}
#if  AJUST_RUN

//��ǽ��һ�Σ���������
/**********************************************************************************
 *��������:robot_ajust_run
 			type - TYPE_ADJ_FIRST_RUN �����͵�ʱ���������߷�������߷����жϣ��Ҷ̾���᲻�ٻ�ͷѰ��������������ǻ�ͷ
 			       TYPE_ADJ_CHECK_AGAIN  ����������У��������
 *�������:gSta �����봫��ײ״̬������������sys->gsta������  
 *		   x_dir X�ᾭ���ķ�����������X��������߹������ж����������û����ɨ��
 *��������:������ײ��ʱ����ǽ��һ�飬ֱ����ײ������ǽ��ʧ���������������
 *********************************************************************************/
char robot_ajust_run(uint8_t type,int x_dir,TOBST *g_obst)
{
//	int y_dir;
	int next_deg=0;
	int begin_deg=0;	//��ʼ�ĽǶ�
	//extern int begin_deg=0;
//	int i;
//	uint8_t sta;
	uint8_t n_sta_back;		//������ʱ���ӱߵķ���
	short bx,toy;//,by;
	int dist1;
	TOBST tobst;
	///uint16_t n_adc=0;	//��¼ת������ADC 	 
	uint16_t state;			///2018-07-16 jzz
	state = sys->sState;	//���ǳ�ʼ����
	
    log_printf("===robot_ajust_run start:%d\r\n",type);
	//��һ����������ż������
	if(type == TYPE_ADJ_FIRST_RUN)
	{	
#if EDGE_DRAW_MAP
		//micony20190305 ֻҪ�к��⣬�Ϳ���
		
		/*if( sys->g_sta[3] < 3000 )
		//if(sys->g_sta[2] < 1500  && sys->g_sta[3] < 3000 &&  sys->g_sta[4] <  1500)
		{
			log_printf("physical bum,don't draw map(%d,%d,%d)\r\n",sys->g_sta[2] ,sys->g_sta[3] ,  sys->g_sta[4] );
			return 0;
		}*/
		
		
		//����Ѿ��߹��ˣ�Ҳ���������ˡ��ж�((X_NOW,Y_NOW)) (X_NOW+1,Y_NOW)(X_NOW-1,Y_NOW)(X_NOW+2,Y_NOW)(X_NOW-2,Y_NOW) ���ϰ�����˳�
		/*if((point_sta(X_NOW,Y_NOW)) == OBST_STA ||(point_sta(X_NOW+1,Y_NOW)) == OBST_STA ||(point_sta(X_NOW-1,Y_NOW)) == OBST_STA ||(point_sta(X_NOW+2,Y_NOW)) == OBST_STA||(point_sta(X_NOW-2,Y_NOW)) == OBST_STA)
		//if(((X_NOW,Y_NOW)||(X_NOW+1,Y_NOW)||(X_NOW-1,Y_NOW) ||(X_NOW+2,Y_NOW)||(X_NOW-2,Y_NOW))==OBST_XY(navigat->tx, navigat->ty))
		//if((X_NOW,Y_NOW)==OBST_XY(X,Y)||(X_NOW+1,Y_NOW)==OBST_XY(X,Y)||(X_NOW-1,Y_NOW)==OBST_XY(X,Y) ||(X_NOW+2,Y_NOW)==OBST_XY(X,Y)||(X_NOW-2,Y_NOW)==OBST_XY(X,Y))
		{
		  log_printf(" The second edge\r\n");
		  return 0;
		}
		 
	    if((begin_deg==navigat->angle)&&(point_sta(X_NOW,Y_NOW)) == OBST_STA )
	    //if((point_sta(X_NOW,Y_NOW)) == OBST_STA ||(point_sta(X_NOW+1,Y_NOW)) == OBST_STA ||(point_sta(X_NOW-1,Y_NOW)) == OBST_STA ||(point_sta(X_NOW+2,Y_NOW)) == OBST_STA||(point_sta(X_NOW-2,Y_NOW)) == OBST_STA)
	    {


        
         log_printf(" The second edge\r\n");
          return 0;
	    }
		*/
#else
		if(navigat->j_obst>=(MAX_TARGET-2))
		{
			log_printf("[adj_run]num over\r\n");
			return 0;
		}	
		//�������ײ����
		//if((sys->g_sta[2] < MIN_IRDA || sys->g_sta[3] < MIN_IRDA || sys->g_sta[4] < MIN_IRDA))
		if(!MIDLE_HAVE_IRDA2())
		{
			log_printf("[adj_run]not mid\r\n");
			return 0;
		}

		if(RIGHT_ADC() >= cfg->side_right_adc || LEFT_ADC() >= cfg->side_left_adc ||
			LEFT_ADC1() >=1800 || RIGHT_ADC1() >=1800)
		{
			log_printf("side adc,not adj\r\n");
			return 0;
		}

		if(navigat->from_charge)		//�ӳ��׮����
		{
			if(navigat->angle == 0 && dis_xy(X_NOW,navigat->dock_x) < 3 && dis_xy(Y_NOW,navigat->dock_y) < 20)
			{
				log_printf("near dock ,not adj\r\n");
				return 0;
			}
		}		
			
		//�Ѿ����Թ����Ͳ�������,�����Ϻ�ߵ�������������ʱ���ظ�����
		if(get_adj_obst(navigat->tx, navigat->ty) || 
			get_adj_obst(navigat->tx+1, navigat->ty) || get_adj_obst(navigat->tx-1, navigat->ty) ||
			get_adj_obst(navigat->tx+2, navigat->ty) || get_adj_obst(navigat->tx-2, navigat->ty) ||
			get_adj_obst(navigat->tx+3, navigat->ty) || get_adj_obst(navigat->tx-3, navigat->ty) ||
			get_adj_obst(navigat->tx+4, navigat->ty) || get_adj_obst(navigat->tx-4, navigat->ty))
		{
			log_printf("have adj()\r\n");
			return 0;
		}
#endif		
		log_printf("[adj_run]type=%d,dir=%d,c=%d\r\n",type,x_dir,navigat->j_obst);
		//������رߵķ����Ҫת���ĽǶ�
		if(navigat->side == LEFT_SIDE)
		{
			turn_to_deg_v(90);
			//�رߵķ���
			navigat->adj_nsta 	= navigat->lst_angle == 0?RIGHT_SIDE_NEAR:LEFT_SIDE_NEAR;
			n_sta_back			= navigat->lst_angle == 0?LEFT_SIDE_NEAR:RIGHT_SIDE_NEAR;
			next_deg = 270;		//�����ͷ��ȥ�ķ���һ����������ķ�������֮�󣬿���Ҫ��ͷ�߻�ȥ�ص�ԭ����λ��
			begin_deg = 90;
			toy = navigat->ty +1;		//������Y����
		}
		else
		{
			turn_to_deg_v(270);
			navigat->adj_nsta 	= navigat->lst_angle == 180?RIGHT_SIDE_NEAR:LEFT_SIDE_NEAR;
			n_sta_back			= navigat->lst_angle == 180?LEFT_SIDE_NEAR:RIGHT_SIDE_NEAR;
			next_deg = 90;
			begin_deg = 270;
			toy = navigat->ty -1;
		}
		if(state != sys->sState)			///2018-07-17 jzz
			return 0;
		tobst.x_dir = navigat->lst_angle == 0?1:-1;
	}
    else
    
	{
		if(g_obst == NULL)
			return 0;
		//������رߵķ����Ҫת���ĽǶ�
		//if(navigat->angle ==0 || navigat->angle == 180)
		//{
			if(g_obst->bret == ADJ_MID_BUM)	//���Сͷ����ײ��������ͷ��
			{
				//�رߵķ���
				navigat->adj_nsta	= navigat->angle == 0?RIGHT_SIDE_NEAR:LEFT_SIDE_NEAR;
				n_sta_back			= navigat->angle == 0?LEFT_SIDE_NEAR:RIGHT_SIDE_NEAR;
				next_deg = 270; 	//�����ͷ��ȥ�ķ���һ����������ķ�������֮�󣬿���Ҫ��ͷ�߻�ȥ�ص�ԭ����λ��
				begin_deg = 90;
			}
			else
			{
				navigat->adj_nsta	= navigat->angle == 180?RIGHT_SIDE_NEAR:LEFT_SIDE_NEAR;
				n_sta_back			= navigat->angle == 180?LEFT_SIDE_NEAR:RIGHT_SIDE_NEAR;
				next_deg = 90;
				begin_deg = 270;
			}
			// navigat->begin_deg;
          // navigat->begin_deg =next_deg;
		/* 
		//��ֱ����
		}else if(navigat->angle ==90 || navigat->angle == 270)
		{
			navigat->adj_nsta	= navigat->angle == 90?RIGHT_SIDE_NEAR:LEFT_SIDE_NEAR;
			n_sta_back			= navigat->angle == 90?LEFT_SIDE_NEAR:RIGHT_SIDE_NEAR;
			next_deg = 180; 	//�����ͷ��ȥ�ķ���һ����������ķ�������֮�󣬿���Ҫ��ͷ�߻�ȥ�ص�ԭ����λ��
			begin_deg = 0;


		}else
		{
			log_printf("angle errr=3.1f\r\n",navigat->angle);
			return 0;
		}
		*/
		log_printf("[adj_run]type=%d,begin=(%d,%d,)end=(%d,%d,),n_sta=%d,%d,deg=%d,%d\r\n",type,
					g_obst->bx,g_obst->by,g_obst->ex,g_obst->ey,navigat->adj_nsta,n_sta_back,begin_deg,next_deg);
		turn_to_deg_v(begin_deg);
		if(state != sys->sState)			///2018-07-17 jzz
			return 0;		
		toy = navigat->ty ;  //�����ĵ�
	}	
	//navigat->adj_nsta = 0;	
	if(state != sys->sState)		///2018-07-16 jzz
		return 0;
	if(navigat->adj_nsta ==0)
	{
		log_printf("[adj_run]nsta error\r\n");
		return 0;
	}
	delay_ms_sensers(300);
	
	//ת�������ж�һ�²����Ƿ��к��⣬û�к�����˳���
	if(navigat->adj_nsta ==RIGHT_SIDE_NEAR)
	{
		if(RIGHT_ADC() < IRD_RIGHT_ADJ_MIN)
		{
			log_printf("[adj_run]r_adc error(%d)\r\n",RIGHT_ADC() );
			return 0;
		}
		///n_adc = RIGHT_ADC();
	}
	if(navigat->adj_nsta ==LEFT_SIDE_NEAR)
	{
		if(LEFT_ADC() < IRD_LEFT_ADJ_MIN)
		{
			log_printf("[adj_run]left adc error(%d)\r\n",LEFT_ADC() );
			return 0;
		}
		///n_adc = LEFT_ADC();
	}
	
#if ADJ_NAER_TURN_V
	//ת����ֱ��λ�ã�δ��ϸ���ԣ���ȥ���������Ļ���������Ǵ�ֱ��ײ��ǽ���ӱ߽����ǶȾ�ʧ���ˡ�
	near_turn_check(navigat->adj_nsta);
	if(state != sys->sState)		///2018-07-16 jzz
		return 0;	
#endif	
//	STORT_XY();					//�洢��ǰ�ĵ㣬һ�����ʧ�ܵ�ʱ��Ҫ�ӱ߻�������Ҫ�õ�
	navigat->adj_walk_dis = navigat->adj_c_lost=0;
	//tobst = &navigat->adj_obst[navigat->j_obst++];
	log_printf("[adj_run]near go,c=%d\r\n",navigat->j_obst);
	
	//sta = navigat->sta;
//	navigat->sta = NAV_STA_ADJ;					
	navigat->adj_run = TRUE;			//��������߷���
	//if(navigat->angle ==90 || navigat->angle == 270)
	navigat->adj_x_org = navigat->x_org_f;	//��¼һ�³�ʼ��Xԭʼ���꣬���ڸ�ʽ��X���ԭʼ����
	navigat->adj_run_ret = 0;

#if LASER_IR_ONLY_ONE
//ֻ���м�һ������,������¼����ת����ʱ��߻��ұߵķǲ��������ֵ
	log_printf("n_adc:%d\r\n",n_adc);
	if(n_adc > 500 && n_adc < 4000)
	{
		uint16_t min,max,lock;
		min = n_adc - 1000;
		if(min>1000) min = 1000;
		else if(min <580) min=580;			////��ʽ��500 ����700
		max = n_adc + 1500;
		if(max>3500) max = 3500;			//500 ��ĳ� 1500 
		lock = n_adc -800;
		if(lock>1200)	lock = 1200;
		log_printf("reset side ir1:%d,%d,%d,%d,%d,[%d,%d,%d,%d]\r\n",cfg->min_left_adc,cfg->max_left_adc,cfg->lock_left_adc,cfg->lost_left_adc,cfg->side_left_adc,n_adc,min,max,lock);
	
		cfg->min_left_adc = cfg->min_right_adc = min;
		cfg->max_left_adc = cfg->max_right_adc = max;
		cfg->lock_left_adc = cfg->lock_right_adc = lock;
		cfg->lost_left_adc = min;
		cfg->lost_right_adc = min;
		cfg->side_left_adc = min;
		cfg->side_right_adc = min;
		log_printf("reset side ir2,min=%d,lock=%d,max=%d\r\n",min,lock,max);
	}
#endif

#if EDGE_DRAW_MAP		//micony20190305 �ر߻���ͼ 

	go_edgeways_for_map(navigat->adj_nsta ,20,begin_deg);//�ر���10��	
	
	log_printf("[adj_run]go back,now=(%d,%d,),to=(%d,%d,)\r\n",X_NOW,Y_NOW,bx,toy);
	navigat->c_go = navigat->c_turn = 0;
	//��������ײ
	navigat->adj_x_org = 0	;		//����ǿ��X����
	navigat->near.pid->c_lost_flag = 0;      //yanbian  2019 03 09 add
  	if(toy != Y_NOW)
	{
		float angle = sys->angle;
		/*
		if(toy > Y_NOW)
			turn_to_deg(90);					//��TOY������ת��90
		else 
			turn_to_deg(270);					//��toy������ת��270			���������أ����Թ�����Բ���һ��
		if(state != sys->sState)		///2018-07-16 jzz
			return 0;	
		if(disfloat(sys->angle,angle) <=40)		//���򲻱䣬�ӱ߾Ͳ���
		{
			log_printf("angle not chagne\r\n");
			n_sta_back = navigat->adj_nsta;
		}else
			n_sta_back = navigat->adj_nsta == LEFT_SIDE_NEAR?RIGHT_SIDE_NEAR:LEFT_SIDE_NEAR;
		*/
		if(navigat->adj_nsta == LEFT_SIDE_NEAR)
		{
			n_sta_back = RIGHT_SIDE_NEAR;
			log_printf("now near side=%d,turn dir=%d,back=%d\r\n",navigat->adj_nsta,GO_LEFT,RIGHT_SIDE_NEAR);
			
			robot_turn_deg(GO_LEFT,DEG_TURN_PWM,160);
		}else
		{
			n_sta_back = LEFT_SIDE_NEAR;
			log_printf("now near side=%d,turn dir=%d,back=%d\r\n",navigat->adj_nsta,GO_RIGTH,LEFT_SIDE_NEAR);
			robot_turn_deg(GO_RIGTH,DEG_TURN_PWM,160);
			
			
		}
		log_printf("motor_go_edgeways,nsta=%d\r\n",n_sta_back);
		//motor_go_edgeways(n_sta_back /*| 0x80*/ ,0,toy,GO_NEAR_TYPE_ADJ,0);
		adj_back_edgeways(n_sta_back,bx,toy);
		if(state != sys->sState)		///2018-07-16 jzz
			return 0;		
		log_printf("motor_go_edgeways_OK\r\n");
	}
	if(navigat->lst_angle ==0)
		turn_to_deg(180);
	else
		turn_to_deg(0);						//��ת��0��
	save_map(1, 1,0);	
	if(state != sys->sState)		///2018-07-16 jzz
		return 0;    
#else
	
	//===================================�˶�����������=============================
	//��һ����
	motor_go_forwark(0,(NO_SIDE_NEAR /*| 0x80*/), run_ajust_check); 	//�ȼ�1�ӱ�
	if(state != sys->sState)		///2018-07-16 jzz
		return 0;
	delay_ms(80);
	log_printf("after go fw,gsta=%d,ret=%d\r\n",sys->gsta,navigat->adj_run_ret);
	//=============================У�����ļ���==================================
	//if(navigat->adj_run_ret == 0)		//û�н��
	//	navigat->adj_run_ret = ADJ_BUM;
	
	dist1 = navigat->adj_walk_dis = navigat->walk_dis;			//��¼���ߵ�·��
	if(dist1 < HW_GRID)	//ǰ������һ�񣬾Ͳ����ˡ�
	{
		log_printf("[adj_run]short dis=%d,not go adj\r\n",navigat->walk_dis);
		navigat->adj_run = FALSE;
		return 0;
	}
	
	log_printf("[adj_run]walkdis=%d,now turn to %d,and go back\r\n",navigat->adj_walk_dis,next_deg);	

	tobst.ex = X_NOW;
	tobst.ey = Y_NOW;
	tobst.eret = navigat->adj_run_ret;
	tobst.bx = X_NOW;
	tobst.by = Y_NOW;
	tobst.bret = navigat->adj_run_ret;
	log_printf("begin(%d,%d,%d)\r\n",tobst.bx,tobst.by,tobst.bret);	
	
	//�����ͣ���һ���������������벻Զ�Ļ����Ͳ���ȥ�ˡ�
	if(type == TYPE_ADJ_FIRST_RUN && navigat->adj_walk_dis < (HW_GRID+HW_HF_GRID))
	{
		log_printf("short dist,not adj\r\n");
		navigat->adj_run = FALSE;
		goto l_robot_ajust_run_end;
	}	
	//ת��ȥ

	turn_to_deg(next_deg);				//��ת,�����ֻ��һ��������,��Ϊ�����������һ������
	if(state != sys->sState)		///2018-07-16 jzz
		return 0;

	//���߻�ȥ
	log_printf("adj go back again\r\n");	
	navigat->adj_x_org = navigat->x_org_f;
	navigat->adj_run_ret = 0;
	navigat->adj_nsta = navigat->adj_nsta == LEFT_SIDE_NEAR?RIGHT_SIDE_NEAR:LEFT_SIDE_NEAR;
	//===================================�˶�����������=============================
	motor_go_forwark(0,(NO_SIDE_NEAR /*| 0x80*/), run_ajust_check); 	//�ȼ�1�ӱ�
	if(state != sys->sState)		///2018-07-16 jzz
		return 0;
	delay_ms(80);
	log_printf("after go fw,gsta=%d,ret=%d\r\n",sys->gsta,navigat->adj_run_ret);	
	navigat->adj_run = FALSE;
	//���߻�ȥ��⣬����ͺ��˳��ˣ����������У�飬��������У��������
	if(type == TYPE_ADJ_CHECK_AGAIN)
	{
		int16_t ax,ay;		//������ֵ
		uint8_t ajusxy = 0;
		if(Y_NOW > tobst.by)	//������ʼ�ĵ㣬��Ϊ�����ĵ�
		{
			tobst.ex = X_NOW;
			tobst.ey = Y_NOW;
			tobst.eret = navigat->adj_run_ret;
			log_printf("end(%d,%d,%d)ret = %d\r\n",tobst.ex,tobst.ey,tobst.eret,navigat->adj_run_ret);
			//���첻�������,��ǰ��Ľ����һͷһ��
			log_printf("targe=(%d,%d,%d)-(%d,%d,%d),my=(%d,%d,%d)-(%d,%d,%d)\r\n",
				g_obst->bx,g_obst->by,g_obst->bret,g_obst->ex,g_obst->ey,g_obst->eret,
				tobst.bx,tobst.by,tobst.bret,tobst.ex,tobst.ey,tobst.eret);
			if(dis_xy((tobst.ey - tobst.by),(g_obst->ey - g_obst->by)) <=1  &&	//micony2017-08-26�������̵㣬2�ĳ�1
				(tobst.bret == g_obst->bret || tobst.eret == g_obst->eret) &&
				(navigat->adj_run_ret == ADJ_SIDE_LOST || navigat->adj_run_ret == ADJ_MID_BUM))
			{
				ajusxy = AJUST_X | AJUST_Y;
				ax  = g_obst->ex;
				ay	= g_obst->ey;

			}else if((tobst.ey - tobst.by) >=(g_obst->ey - g_obst->by) / 2 &&
					(tobst.ey - tobst.by) > 3)	//���ȹ������ٿ���У��X��
			{
				ajusxy = AJUST_X ;
				ax  = g_obst->ex;
				ay	= Y_NOW;
			}			
		}
		else
		{
			tobst.bx = X_NOW;
			tobst.by = Y_NOW;
			tobst.bret = navigat->adj_run_ret;
			log_printf("begin(%d,%d,%d)ret=%d\r\n",tobst.bx,tobst.by,tobst.bret,navigat->adj_run_ret);
			log_printf("targe=(%d,%d,%d)-(%d,%d,%d),my=(%d,%d,%d)-(%d,%d,%d)\r\n",
				g_obst->bx,g_obst->by,g_obst->bret,g_obst->ex,g_obst->ey,g_obst->eret,
				tobst.bx,tobst.by,tobst.bret,tobst.ex,tobst.ey,tobst.eret);
			if(dis_xy((tobst.ey - tobst.by),(g_obst->ey - g_obst->by)) <=1  &&  	//micony2017-08-26�������̵㣬2�ĳ�1
				(tobst.bret == g_obst->bret || tobst.eret == g_obst->eret) &&
				(navigat->adj_run_ret == ADJ_SIDE_LOST || navigat->adj_run_ret == ADJ_MID_BUM))
			{
				ajusxy = AJUST_X | AJUST_Y;
				ax  = g_obst->bx;
				ay	= g_obst->by;

			}
			else if((tobst.ey - tobst.by) >=(g_obst->ey - g_obst->by) / 2 &&
					(tobst.ey - tobst.by) > 4)	//���ȹ������ٿ���У��X��
			{
				ajusxy = AJUST_X ;
				ax  = g_obst->bx;
				ay	= Y_NOW;
			}
		}	
		if(ajusxy)
		{
			log_printf("ajusxy=%d\r\n",ajusxy);
			if(check_map_reason(X_NOW,Y_NOW,ax,ay))
			{
				save_now_point(navigat);						//�洢����
				ajust_xy_map(ax,ay,ajusxy)	;	//У׼����
				calc_point_offset(navigat); 				//����ƫ����
			}
			/*
			if(verify_on_map(navigat,10)==0)				//��ͼУ��ʧ�ܣ���ع�
			{
				log_printf("verify map error2\r\n");
				restor_point(navigat);
				return 0;
			}*/
		}
		navigat->c_turn = navigat->c_go = 0;
		//return 1;
		//�õ��Ѿ���ɨ�����˳��ˡ�
		/*
		�����򵥵��˳��������⣬��ʱ�����δ��ɨ����Ҳ���˳���
		if(SCANE_XY(X_NOW,Y_NOW))
		{
			return 1;
		}
		*/		
	}
	else			//��һ���ҵ�����ֱ��
	{
		TOBST *mobst;
		uint8_t other_adj=0;
//		int16_t yy,xx;
		
		if(Y_NOW > tobst.by)	//������ʼ�ĵ㣬��Ϊ�����ĵ�
		{
			tobst.ex = X_NOW;
			tobst.ey = Y_NOW;
			tobst.eret = navigat->adj_run_ret;
			log_printf("end(%d,%d,%d)\r\n",tobst.ex,tobst.ey,tobst.eret);
		}
		else
		{
			tobst.bx = X_NOW;
			tobst.by = Y_NOW;
			tobst.bret = navigat->adj_run_ret;
			log_printf("begin(%d,%d,%d)\r\n",tobst.bx,tobst.by,tobst.bret);
		}
		navigat->adj_walk_dis = navigat->walk_dis;	
		log_printf("adj walk ok,dist1=%d,nowdist=%d\r\n",dist1,navigat->adj_walk_dis);
		//�ж��ǲ��Ǵ��ظ���ADJ
		other_adj = 0;
		/*
		yy = (tobst.by + tobst.ey) / 2;
		for(xx=tobst.bx - 5; xx <= tobst.bx + 5;xx++)
		{
			mobst = get_adj_obst(xx, navigat->ty);
			if(mobst !=NULL)
			{
				log_printf("found an other adj,xdir=%d,%d\r\n",mobst->x_dir ,tobst.x_dir);
				if(mobst->x_dir == tobst.x_dir)		//����һ��,����һ�µ�adj
				{
					log_printf("the same side..\r\n");
					other_adj = 1;
					break;
				}
			}
		}
		*/
		if(other_adj == 1)
		{
			int16_t ax,ay;		//������ֵ
			uint8_t ajusxy = 0;
			if(dis_xy((tobst.ey - tobst.by),(mobst->ey - mobst->by)) <=2	&&
				(tobst.bret == mobst->bret || tobst.eret == mobst->eret) &&
				(navigat->adj_run_ret == ADJ_SIDE_LOST || navigat->adj_run_ret == ADJ_MID_BUM))
			{
				ajusxy = AJUST_X | AJUST_Y;
				ax	= g_obst->bx;
				ay	= g_obst->by;
			}
			else if((tobst.ey - tobst.by) >=(mobst->ey - mobst->by) / 2 &&
					(tobst.ey - tobst.by) > 4)	//���ȹ������ٿ���У��X��
			{
				ajusxy = AJUST_X ;
				ax	= g_obst->bx;
				ay	= Y_NOW;
			}
			if(ajusxy)
			{
				log_printf("ajusxy=%d\r\n",ajusxy);
				ajust_xy_map(ax,ay,ajusxy)	;	//У׼����
			}
		}
		else
		{		
			//�����������б�
			if(navigat->j_obst < MAX_TARGET)
			{
				log_printf("[adj_run]insert(%d,%d,%d)-(%d,%d,%d)idx=%d\r\n",
							tobst.bx,tobst.by,tobst.bret,tobst.ex,tobst.ey,tobst.eret,navigat->j_obst);
				mobst = &navigat->adj_obst[navigat->j_obst++];
				mobst->bx = tobst.bx;
				mobst->by = tobst.by;
				mobst->bret = tobst.bret;
				mobst->ex = tobst.ex;
				mobst->ey = tobst.ey;
				mobst->eret = tobst.eret;
				mobst->x_org = navigat->x_org;
				mobst->x_dir = tobst.x_dir;
			}else
				log_printf("[adj_run]target overflow,%d\r\n",navigat->j_obst);
		}
		
	}
	
	
	

	
	log_printf("[adj_run]go back,now=(%d,%d,),to=(%d,%d,)\r\n",X_NOW,Y_NOW,bx,toy);
	navigat->c_go = navigat->c_turn = 0;
	//��������ײ
	navigat->adj_x_org = 0	;		//����ǿ��X����
	if(toy != Y_NOW)
	{
		float angle = sys->angle;
		if(toy > Y_NOW)
			turn_to_deg(90);					//��TOY������ת��90
		else 
			turn_to_deg(270);					//��toy������ת��270			���������أ����Թ�����Բ���һ��
		if(state != sys->sState)		///2018-07-16 jzz
			return 0;	
		if(disfloat(sys->angle,angle) <=40)		//���򲻱䣬�ӱ߾Ͳ���
		{
			log_printf("angle not chagne\r\n");
			n_sta_back = navigat->adj_nsta;
		}else
			n_sta_back = navigat->adj_nsta == LEFT_SIDE_NEAR?RIGHT_SIDE_NEAR:LEFT_SIDE_NEAR;
		log_printf("motor_go_edgeways,nsta=%d\r\n",n_sta_back);
		//motor_go_edgeways(n_sta_back /*| 0x80*/ ,0,toy,GO_NEAR_TYPE_ADJ,0);
		adj_back_edgeways(n_sta_back,bx,toy);
		if(state != sys->sState)		///2018-07-16 jzz
			return 0;		
		log_printf("motor_go_edgeways_OK1\r\n");
	}
	/*
	if(motor_go_forwark(next_deg,NO_SIDE_NEAR, NULL)==0)
	{
		log_printf("[adj_run]go back bum\r\n");
		//�ߵò���ˣ��Ͳ����ˣ�ֱ����
		if(disxy(navigat->walk_dis,navigat->adj_walk_dis) < HW_HF_GRID)
		{

		}else
		{
			log_printf("go back error,near wall,n_near=%d\r\n",n_sta_back);
			motor_go_edgeways(n_sta_back ,bx,by,GO_NEAR_TYPE_ADJ,0);
			log_printf("near ok,now=(%d,%d,%3.1f)\r\n",X_NOW,Y_NOW,A_NOW);
			//near_cross_xy(int tox,int toy,uint8_t ctype,int dir)
		}
	}
	*/
l_robot_ajust_run_end:
	if(navigat->lst_angle ==0)
		turn_to_deg(180);
	else
		turn_to_deg(0);						//��ת��0��
	save_map(1, 1,0);	
	if(state != sys->sState)		///2018-07-16 jzz
		return 0;		
#endif
	return 1;

}

char adj_back_edgeways(uint8_t n_sta ,short tox,short toy)
{
	int calue=0;
	uint8_t sta;
	short llx=0,lly=0;
//	short lx=X_NOW,ly=Y_NOW;
	short by = Y_NOW;
//	short bx = X_NOW;
//	short nx1,ny1;
	float lagle;		//��¼�����ӱ߽Ƕȣ����ڼ����Ƿ�ת�����
//	uint16_t	gSta;
	uint32_t	t_begin;//��ʼ��ʱ��
	int ret_calc=1;
//	uint16_t c_dock_data=0;
	int16_t c_lost=0;
	uint16_t c_round=0;		//תȦ�Ĵ���
	int c_near_wall = 0;


	short x_begin_line=0,y_begin_line = 0;		//һ���߿�ʼ��X��Y������
	
//	u8 irData_bak[6];
	//int c_dock = 0;
	int16_t dist_y = dis_xy(Y_NOW,toy)+2;
	
	NEAR_WALL *near = &navigat->near;

	float m_angle[MAX_C_M_ANGLE];			//���������20����ײ�ĽǶ�
	int16_t c_m_angle=0;
	
#if LAGER_ROUND_MODE	
	int16_t cyc_x[MAX_C_M_ANGLE],cyc_y[MAX_C_M_ANGLE];
	uint8_t tmp=0;
#else
	int16_t c_near_wall1 = 0;
#endif	

	for(c_m_angle = 0;c_m_angle<MAX_C_M_ANGLE;c_m_angle++)
		m_angle[c_m_angle] = 0;
	c_m_angle = 0;

		
//	motor_run(GO_ROUND,600,0,0);
	CHECK_NAVI_STA_RT(0);
	pd_gyro_int(GO_NEAR_PWM_FAST);
	navigat->out =navigat->angle;
	navigat->distance = 0;
	motor.c_left_hw = motor.c_right_hw = 0;
	cord_calc_store(0);
	gyro_whlmap();
#if JUDGE_PID_CTRL_WHEEL_STOP
	m_speed_pid_ctrl(0,0,0,0,0,0,0);		//��¼���ֻ�����
#else	
	m_speed_pid_ctrl(0,0);
#endif	
	
	motor_run(GO_FORWARD, GO_NEAR_PWM_FAST, 0, 0);
	if(n_sta & 0x80)
		init_near_wall_navi(n_sta & 0x7F);
	else
		init_near_wall_navi(NO_SIDE_NEAR);
	n_sta &=0x7F;
	//init_near_wall_navi(NO_SIDE_NEAR);	//�տ�ʼ����Ҫһֱ�ر�
	navigat->is_walk =1;
	log_printf("[adj_back_edgeways]sta=%d(%d),now=(%d,%d,)to=(%d,%d,)dis=%d\r\n",navigat->near.n_sta,n_sta,X_NOW,Y_NOW,tox,toy,dist_y);
	//navigat->wheel_dir = 1;
	sta = sys->sState;
	lagle = sys->angle;
	mstimeout(&t_begin,0);
	while(1)
	{

		proc_uart_task();
		get_sensers(&sys->gSta);
		if(sys->sState !=sta)
			return RET_NEAR_ERROR;
		CHECK_NAVI_STA_RT(0);
		//��ײ
		if(sys->gSta & (MASK_BUM_MIDL) || ret_calc==0 ||  c_lost >=MAX_NEAR_LOST)
		{
//			gSta = sys->gSta;	//��¼��ײ��״̬
			motor_run(GO_STOP,0,0,0);
			//log_printf("BUM\r\n");
			coordinate_calcu(0);														//�����ԭʼ������ϵ
			coord_org2map(navigat->x_org,navigat->y_org,&navigat->tx,&navigat->ty); //����ϵת��
			motor_run(GO_STOP,0,0,0);
			log_printf("\r\n-----adj_back_edgeways bum(%d,%d,%d,%f,%f),gsta=%d,irda=(%d,%d,%d,%d)angle=%3.1f,ret=%d,lost=%d\r\n",navigat->tx,navigat->ty,motor.c_left_hw,navigat->x_org_f,navigat->y_org_f,
							sys->gSta,sys->g_sta[0],sys->g_sta[1],sys->g_sta[5],sys->g_sta[6],sys->angle,ret_calc,c_lost);
			
			
			if(!(ret_calc==0 || c_lost >=MAX_NEAR_LOST))
				c_round = 0;
			if(ret_calc==0)
					gyro_mapwhl();
//			gSta = sys->gSta;
			//motor_run(GO_STOP,0,0,0);
			if(sys->work_mod & MWO_MOP)
				delay_ms(200);
			//www_idleintel_com();
			lagle = sys->angle;
		
			navigat->distance = 0;
			motor.c_left_hw = 0;
			
#if !LAGER_ROUND_MODE	
			if(c_m_angle >=MAX_C_M_ANGLE)
				c_m_angle = 0;
			m_angle[c_m_angle++] = sys->angle;		//���ֽǶ�

			if(near_round_360(m_angle,c_m_angle))
			{
				log_printf("found round...go line...\r\n");

				init_near_wall_navi(NO_SIDE_NEAR);
				for(c_m_angle = 0;c_m_angle<MAX_C_M_ANGLE;c_m_angle++)
					m_angle[c_m_angle] = 0;
				c_m_angle = 0;
				//goto l_go_edeways;
				return RET_NEAR_OK;	
			}
#else////2018-07-10  δ����6
			if(near_large_round_360(cyc_x,cyc_y,m_angle,c_m_angle)) 					///�ƴ�׮�жϺ���
			{
				log_printf("found large round...go line...\r\n");
				if(near->n_sta == RIGHT_SIDE_NEAR)
				{
					robot_turn_deg(GO_LEFT,DEG_TURN_PWM,135);
					//goto l_mgo_edeways;
				}else if(near->n_sta == LEFT_SIDE_NEAR)
				{
					robot_turn_deg(GO_RIGTH,DEG_TURN_PWM,135);
					//goto l_mgo_edeways;
				}						
				for(tmp=0;tmp<c_m_angle;tmp++)
				{
					log_printf("%02d(%03d,%03d,%03f)\r\n",tmp,cyc_x[tmp],cyc_y[tmp],m_angle[tmp]);
				}
				init_near_wall_navi(NO_SIDE_NEAR);
				for(c_m_angle = 0;c_m_angle<MAX_C_M_ANGLE;c_m_angle++)
					m_angle[c_m_angle] = 0;
				c_m_angle = 0;
				return RET_NEAR_OK;	
			}	
#endif
			/**************************************************************************
				���tox��toy���������ã��򵽵��ˣ���ͣ������
			*****************************************************************************/
#if EDGE_DRAW_MAP
			if(   (Y_NOW == toy ) ||	//������
				( (by > toy && Y_NOW < toy) || (by < toy && Y_NOW > toy))/* ||
				 dis_xy(Y_NOW,toy) >= dist_y*/) //adj��ȥ��Y�ᳬ��?
#else
			if(   (Y_NOW == toy ) || 	//������
				( (by > toy && Y_NOW < toy) || (by < toy && Y_NOW > toy)) ||
				 dis_xy(Y_NOW,toy) >= dist_y) //adj��ȥ��Y�ᳬ����
#endif
			{
				log_printf("xy ok1(%d,%d,%3.1f),(by=%d,toy=%d),dist_ty=%d\r\n",X_NOW,Y_NOW,sys->angle,by,toy,dist_y);
				motor_run(GO_STOP,0,0,0);
				return RET_NEAR_OK;				
			}	
			
	
			if(near->n_sta ==NO_SIDE_NEAR)
			{
				init_near_wall_navi(n_sta);
			}

			if( (ret_calc==0 || c_lost >= MAX_NEAR_LOST)&& *(navigat->near.pid->adc) < navigat->near.pid->min_adc )
			{
				c_round++;
				log_printf("lost or calc error(%d,%d),cround=%d\r\n",ret_calc,c_lost,c_round);
				/*
				if(c_round >=3)
				{
					log_printf("big round not near\r\n");
					init_near_wall_navi(NO_SIDE_NEAR);
					goto l_go_edeways;
				}
				*/
				if(ret_calc==0)
					gyro_mapwhl();
				
				if(near->n_sta == RIGHT_SIDE_NEAR)
				{
					robot_turn_deg(GO_RIGTH,TURN_DEG_PWM,30);
					goto l_go_edeways;
				}else if(near->n_sta == LEFT_SIDE_NEAR)
				{
					robot_turn_deg(GO_LEFT,TURN_DEG_PWM,30);
					goto l_go_edeways;
				}
					
			}	



			
			if(near->n_sta == RIGHT_SIDE_NEAR)
			{
				//modify201710	�������bug�����·�����ײ��֮ǰ��֪��Ϊ�����
				/*
				if((gSta & MASK_BUM_LEFT))
				{
					log_printf("midbum,10deg\r\n");
					robot_turn_deg(GO_RIGTH,TURN_DEG_PWM,10);
				}else 
				*/
				{
					robot_turn_deg(GO_LEFT,TURN_DEG_PWM,12);
				}
			}else if(near->n_sta == LEFT_SIDE_NEAR)
			{
				//log_printf("gsta=%d,left=%d,%d\r\n",sys->gSta,sys->g_sta[0],sys->g_sta[1]);
				//modify201710	�������bug�����·�����ײ��֮ǰ��֪��Ϊ�����
				/*
				if((gSta & MASK_BUM_RIGHT)  )
				{
					log_printf("midbuml,10deg\r\n");
					robot_turn_deg(GO_LEFT,TURN_DEG_PWM,10);
				}else 
				*/
				{
						//log_printf("midbuml,12deg\r\n");
					robot_turn_deg(GO_RIGTH,TURN_DEG_PWM,12);
				}

			}else
			{
				log_printf("RET_NEAR_ERROR,nsta=%d\r\n",n_sta);
				return RET_NEAR_ERROR;	
			}
l_go_edeways:		
			c_lost=0;
			//coordinate_calcu(); 	
			motor_run(GO_STOP,0,0,0);
		//	log_printf("after bk(%d,%d,%d,%3.3f,%3.3f,%3.3f)\r\n==============\r\n",navigat->tx,navigat->ty,motor.c_left_hw,navigat->x_org_f,navigat->y_org_f,sys->angle);
			//if(ccc++ >=5)
			//	while(1);
			navigat->distance = 0;
			navigat->is_walk = 1;
			pd_gyro_int(GO_NEAR_PWM_FAST);
			navigat->out =sys->angle;
			cord_calc_store(0);
			//gyro_whlmap();
			
			motor_run(GO_FORWARD, GO_NEAR_PWM_FAST, 0, 0);
			motor.c_left_hw = motor.c_right_hw = 0;
#if JUDGE_PID_CTRL_WHEEL_STOP
	m_speed_pid_ctrl(0,0,0,0,0,0,0);		//��¼���ֻ�����
#else			
			m_speed_pid_ctrl(0,0);
#endif			
			c_near_wall = 0;
			navigat->near.pid->c_lost = 0;
			navigat->near.pid->c_lost_flag = 0;
			ret_calc = 1;
			//��¼����λ��
			x_begin_line = X_NOW;
			y_begin_line = Y_NOW;
#if !LAGER_ROUND_MODE			
			c_near_wall1 = 0;
#endif
		}

		if(TIM5->CNT >=5000)
		{
			TIM5->CNT = 0;
			www_idleintel_com();

			navigat->out = format_agle(navigat->out,ANGLE_360);
			proc_line_pid(navigat->out);
			navigat_near_wall_pid(&navigat->out,5);
			if(near->n_sta ==NO_SIDE_NEAR)
			{
				if(RIGHT_ADC() >= cfg->lock_right_adc)
					init_near_wall_navi(RIGHT_SIDE_NEAR);
				else if(LEFT_ADC() >= cfg->lock_left_adc)
					init_near_wall_navi(LEFT_SIDE_NEAR);
			}
			
			if( *(navigat->near.pid->adc) > navigat->near.pid->min_adc) //�ӱ�
			  lagle = sys->angle;
			else		//�ӱ���ʧ��ת�ĽǶȳ���180�ȣ���ʧ���˳�
			{
				if(disfloat( lagle , sys->angle) > 180)
				{
					log_printf("lost over(%d,%d,%3.1f,%3.1f)\r\n",X_NOW,Y_NOW,sys->angle,lagle);
					motor_run(GO_STOP,0,0,0);
					//delay_ms_sensers(200);
					coordinate_calcu(0);														//�����ԭʼ������ϵ
					coord_org2map(navigat->x_org,navigat->y_org,&navigat->tx,&navigat->ty);	//����ϵת��
					return RET_NEAR_ERROR;	
				}
			}
#if CALE_BY_FRON				
				ret_calc = coordinate_calcu(1);														//�����ԭʼ������ϵ
#endif	
			if(calue++ >=40)
			{
				calue = 0;
#if !CALE_BY_FRON				
				ret_calc = coordinate_calcu(0);														//�����ԭʼ������ϵ
#endif	
				//ret_calc = coordinate_calcu();														//�����ԭʼ������ϵ
				coord_org2map(navigat->x_org,navigat->y_org,&navigat->tx,&navigat->ty);	//����ϵת��
				//����0�ȣ�90�ȣ�80�ȵķ���,ˢ������
				/**/
				if(x_begin_line != X_NOW && y_begin_line != Y_NOW)
				{
					ajust_xy_by_near_line(x_begin_line,y_begin_line,X_NOW,Y_NOW,sys->angle,LINE_TYPE_LOST,n_sta);
					x_begin_line = X_NOW;
					y_begin_line = Y_NOW;	
				}
#if !LAGER_ROUND_MODE				
				if(c_near_wall1 ++ >=30)
				{
	
					for(c_m_angle = 0;c_m_angle<MAX_C_M_ANGLE;c_m_angle++)
						m_angle[c_m_angle] = 0;
					c_m_angle = 0;
					c_near_wall1 = 0;
					log_printf("log near\r\n");
				}
#endif				
				//�ӱ�����
				if(LEFT_ADC() > cfg->lock_left_adc || RIGHT_ADC() > cfg->lock_right_adc)
					c_near_wall ++ ;
#if !JUDGE_PID_CTRL_WHEEL_STOP						
				if(c_near_wall >=20 && dis_xy(L_FORWRK_PWM,R_FORWRK_PWM) < 100)
				{	
					m_speed_pid_ctrl(0,1);			///				
				}	
#endif					
				c_near_wall = 0;
				
				if( *(navigat->near.pid->adc) > navigat->near.pid->min_adc) //�ӱ�
				{
					//c_near++;
					c_round = 0;					
				}
				else if(near->n_sta !=NO_SIDE_NEAR)	//ǿ���ӱߣ��������ʧ�ĸ���
					c_lost ++;	
					
				if(llx!=navigat->tx || lly!=navigat->ty)
				{
#if WALK_PRINTF				
					log_printf("*(%d,%d,%3.1f,0)-[%3.1f,%3.1f,0]*\r\n",navigat->tx,navigat->ty,sys->angle,navigat->x_org_f,navigat->y_org_f);
#endif
#if WIFICONFIG					
		updata_stream_cache_data(navigat->tx,navigat->ty,0,POINT_SCAN);						
#endif	

#if LAGER_ROUND_MODE
					if(c_m_angle >=MAX_C_M_ANGLE)
					{
						for(tmp=0;tmp<MAX_C_M_ANGLE-1;tmp++)
						{
							cyc_x[tmp] = cyc_x[tmp+1];
							cyc_y[tmp] = cyc_y[tmp+1];
							m_angle[tmp] = m_angle[tmp+1] ; 
						}	
						cyc_x[MAX_C_M_ANGLE-1] = X_NOW;
						cyc_y[MAX_C_M_ANGLE-1] = Y_NOW;
						m_angle[MAX_C_M_ANGLE-1] = sys->angle;				//��֤���µ�40����
					}
					else			
					{
						cyc_x[c_m_angle] = X_NOW;
						cyc_y[c_m_angle] = Y_NOW;
						m_angle[c_m_angle++] = sys->angle;		//����Ƕ�				
					}	
#endif

					llx = navigat->tx;
					lly = navigat->ty;
				}
				/**************************************************************************
					���tox��toy���������ã��򵽵��ˣ���ͣ������
				*****************************************************************************/
#if EDGE_DRAW_MAP
				if(   (Y_NOW == toy ) ||	//������
				( (by > toy && Y_NOW < toy) || (by < toy && Y_NOW > toy))/* ||
				 dis_xy(Y_NOW,toy) >= dist_y*/) //adj��ȥ��Y�ᳬ����

#else
				if(   (Y_NOW == toy ) || 	//������
				( (by > toy && Y_NOW < toy) || (by < toy && Y_NOW > toy)) ||
				 dis_xy(Y_NOW,toy) >= dist_y) //adj��ȥ��Y�ᳬ����
#endif				 
				{    
				    log_printf("xy ok2(%d,%d,%3.1f),(by=%d,toy=%d),dist_ty=%d\r\n",X_NOW,Y_NOW,sys->angle,by,toy,dist_y);
					
					motor_run(GO_STOP,0,0,0);
					return RET_NEAR_OK;				
				}				
			}
		}		
	}
//	return 0;
}

//����һ�Σ�������
#if 0
int run_ajust_again(void)
{
//	int idx;
	//�ر���ʧ�������˳�
	if(navigat->adj_nsta == LEFT_SIDE_NEAR )		//���ر�
	{
		if(LEFT_ADC() < IRD_LEFT_ADJ_MIN)		
		{
			log_printf("adj:%d\r\n",LEFT_ADC());
			if(navigat->adj_c_lost++ > LOST_NUBUER)
			{
				motor_run(GO_STOP,0,0,0);
				if(LEFT_ADC() < SIDE_LOST_LEFT)	
					navigat->adj_run_ret = ADJ_SIDE_LOST;	//�������߷���ȷ
				else
					navigat->adj_run_ret = ADJ_FAIL;
				log_printf("[run_ajust_check]lost,ret=%d\r\n",navigat->adj_run_ret);
				return 0;
			}
		}else
			navigat->adj_c_lost = 0;
	}
	if(navigat->adj_nsta == RIGHT_SIDE_NEAR )		//���ر�
	{
		if(RIGHT_ADC() < IRD_RIGHT_ADJ_MIN)		
		{
			log_printf("adjr:%d\r\n",RIGHT_ADC());
			if(navigat->adj_c_lost++ > LOST_NUBUER)
			{
				motor_run(GO_STOP,0,0,0);
				//������ʧȥ��ǽ�ߣ��������ҵ�������
				if(RIGHT_ADC() < SIDE_LOST_RIGHT)	
					navigat->adj_run_ret = ADJ_SIDE_LOST;	//�������߷���ȷ
				else
					navigat->adj_run_ret = ADJ_FAIL;
				log_printf("[run_ajust_check]lost2,ret=%d\r\n",navigat->adj_run_ret);
				return 0;
			}
		}else
		  navigat->adj_c_lost = 0;
	}
	

	return 1;
}
#endif
//������������
/*
 type - 0 ��ǿ�Ƶ�����1ǿ�Ƶ���
 ��ǿ�Ƶ���:����ת��Ȧ�������ߵĴ�����
����ֵ

#define ADJ_NAVI_NO_RUN		0x00	//�����ϵ������������������û����û������ȥ
#define ADJ_NAVI_RUN_OK		0x01	//���ϵ�������������������ҵ�����ȥ�������ɹ�
#define ADJ_NAVI_RUN_ERR	0x02	//���ϵ�������������������ҵ�����ȥ�������Ƿ񣬵����Ӷ���

*/
int THERE_OBST(short X,short Y)    
{
	return (OBST_XY(X-1,Y) || OBST_XY(X,Y) || OBST_XY(X+1,Y));
}
#define ADJ_XY		0x00
#define ADJ_X		0x01
#define ADJ_Y		0x02

char navigat_to_target(uint8_t type)
{
	short nx,ny;
	//int x,y;
	int idx;//,j,i;
	float angle;//,a_back=0;
	TOBST *obst;//,*eobt;
//	uint8_t sta;
	uint8_t c_target=0;
	uint8_t router = 0;
	uint8_t	adj_type = 0;
	short x,y;
	uint16_t state;
//	int y,x;
	//��ǿ�Ƶ�����Ҫ�ȼ���һ�£��Ƿ���ϵ���������
#if !NAVI_TO_TARGET
	
	return ADJ_NAVI_NO_RUN;
#endif
	state = sys->sState;
	//�ոս����꣬һ�ɲ�����
	if(navigat->c_target_go < 8)
		return ADJ_NAVI_NO_RUN;
	//�������У׼����ȥ�ж�
	if(type == NAV_ADJ_ANGLE)
	{
		if(navigat->c_turn < 20 )
			return ADJ_NAVI_NO_RUN;
		log_printf("[navitag]NAV_ADJ_ANGLE...,%d,%d\r\n",navigat->c_turn ,navigat->c_go );
		goto l_nav_adj_check;

	}
	if( type == NAVA_ADJ_NEAR)
	{
		if(navigat->c_turn < 10 && navigat->c_go < 10)
			return ADJ_NAVI_NO_RUN;
		log_printf("[navitag]near...,%d,%d\r\n",navigat->c_turn ,navigat->c_go );
		goto l_nav_adj_check;
	}
	if(type ==0)
	{
		if(navigat->c_turn < 30 || navigat->c_go < 30)
		{
			log_printf("[navitag]c error,%d,%d\r\n",navigat->c_turn ,navigat->c_go);
			//goto l_navi_to_targ_check_again
			return ADJ_NAVI_NO_RUN;
		}
	}
		//����ǿ�Ƶ������͸��ݴ�����
	if(type != NAV_ADJ_FORCE_ALL)
	{
		if(navigat->c_go_force < 10)
		{
			log_printf("[navitag]c_go_force error,%d\r\n",navigat->c_go_force);
			//goto l_navi_to_targ_check_again
			return ADJ_NAVI_NO_RUN;
		}
	}
	log_printf("[navitag]c=%d\r\n",navigat->j_obst);
l_nav_adj_check:	
	//���������б���ܲ��ܼ򵥵ĵ�����ȥ���ܵ�����ȥ���͵��õ�������navigat_robot��������ȥ
	for(idx=0;idx<navigat->j_obst;idx++)
	{
		obst = &navigat->adj_obst[idx];
		//��������ĵ������
		y = (obst->by + obst->ey) / 2;
		x = obst->ex;
		//�ӳ��׮����,�ڸ����������Ҫ�Ƚϳ�����ֻ�ܵ�����������Ƚ�Զ�ĵط�
		if(navigat->from_charge && dis_xy(x,navigat->dock_x) < 5)
		{
			log_printf("targe near dock(%d,%d,%d)i=%d\r\n",x,y,obst->x_dir,idx);
			if(dis_xy(y,navigat->dock_y) < 5)
			{
				log_printf("near dock,not get\r\n");
				continue;
			}
		}else
			log_printf("targe(%d,%d,%d)i=%d\r\n",x,y,obst->x_dir,idx);
		//if( OBST_XY(x+1,y) && OBST_XY(x-1,y) ==0)
		if(dis_xy(obst->by,obst->ey) < 6)
		{
			log_printf("short dist(%d,%d,)\r\n",x,y);
			continue;
		}
		if(obst->x_dir == 1)
		{
			angle = 0;
			//��������
			x-=1;
			
		}//else if( OBST_XY(x+1,y)==0 && OBST_XY(x-1,y) ==1)
		else if(obst->x_dir == -1)
		{
			angle = 180;
			x +=1;
		}
		else
		{
			log_printf("no obst_12(%d,%d,)\r\n",x,y);
			continue;
		}
		
		

		//�ܰ��ռ򵥵�·��������ȥ�ģ����Կ��ǵ���
		router = 0;
		if(router_p2p(navigat->tx,navigat->ty, x,y,&nx,&ny) )
		{
			router = 1;
		}
		//�����ǿ�Ƶ��������Ǹ����ӵ�·��
		if(router == 0 && type == NAV_ADJ_FORCE_ALL)
		{
			if(search_route(navigat->tx,navigat->ty,x,y,&nx,&ny))
				router = 1;
		}
		if(router == 1)		//�ܵ�����ȥ
		{
			//�������ǿ��У�������㣬����������ĸ����Ž���
			if(type ==0 && navigat->c_turn < 30 && navigat->c_go < 30)
			{
				int max_dis;
				if(type == NAV_ADJ_FORCE_H)
					max_dis = 10;
				else
					max_dis = 5;
				if(disXY(navigat->tx, x) >= max_dis || disXY(navigat->ty, y) >= max_dis )
				{
					log_printf("[navitag]long dis out,(%d,%d,)\r\n",x,y);
					continue;
				}
			}
			//̫Զ�ģ�Ҳ����ȥУ׼�ˡ�
			if(disXY(navigat->tx, x) >= 55 || disXY(navigat->ty, y) >= 55 )
			{
				log_printf("[navitag]long dis out2,(%d,%d,)\r\n",x,y);
				continue;
			}
			if( type == NAVA_ADJ_NEAR)
			{
				if(disXY(navigat->tx, x) >= 5 || disXY(navigat->ty, y) >= 5 
					|| obst == navigat->l_adj_obst)
				{
					log_printf("[navitag]near long dis out2,(%d,%d,)\r\n",x,y);
					continue;
				}
				
			}
			if( type == NAV_ADJ_ANGLE)
			{
				if(disXY(navigat->tx, x) >= 5 || disXY(navigat->ty, y) >= 5 )
				{
					log_printf("[navitag]near long dis out2,(%d,%d,)\r\n",x,y);
					continue;
				}
				
			}
			goto l_navi_to_targ;
			
		}
	}
//l_navi_to_targ_check_again:
	adj_type++;
	log_printf("[navitag]end of check adj_type=%d\r\n",adj_type);
	return ADJ_NAVI_NO_RUN;
	/*
	if(adj_type>=2)
	{
		log_printf("[navitag]no run\r\n");
		return ADJ_NAVI_NO_RUN;
	}else 
		goto l_nav_adj_check;
		*/
l_navi_to_targ:
	log_printf("[navitag]found target=(%d,%d,%3.1f)\r\n",x,y,angle);
	
	if(type == NAV_ADJ_FORCE_H || c_target <=2)
		navigat->c_go_force = 0;		//ǿ�������������λ
	//������ȥ
	navigat->sta = NAV_STA_NAVIGATE;
	navigat->l_adj_obst = obst;		//��¼��������������
	navigat->c_target_go = 0;
	//if(navigat_robot(NAVI_TYPE_NJST,DONT_SEARCH_UNCLR,obst->x,obst->y,angle) == NAVIGAT_OK)
	//=========================================������adj��=====================================
	if(navigat_to_point(x,y,angle) == NAVIGAT_OK)
	{
		//������λ�ˣ��ȴ�ֱ�ĳ��ȥ��Ȼ���ٹ����ߣ�У׼
		log_printf("[navitag]turn to %3.1f\r\n",angle);
		
		//ת����ֱ�ĽǶȣ�Ȼ���߹�ȥ
		turn_to_deg(angle);
		if(sys->sState != state)			///2018-07-16 jzz
			return ADJ_NAVI_NO_RUN;
		//========================================��ֱǽ�ڳ��ȥ=====================================
		if(motor_go_forwark(6*HW_GRID,NO_SIDE_NEAR, NULL) ==1)	//���û����ײ��������ˡ�
		{
			log_printf("[navitag]navi targe error!!");
			return ADJ_NAVI_RUN_ERR;
			 //����������Y�����꣬�ٳ�һ��
			// if(navi_to_new_adj(obst->x,obst->y,eobt->x,eobt->y,navigat->walk_dis)==0)
			//	return ADJ_NAVI_RUN_ERR;
		}
		if(sys->sState != state)			///2018-07-16 jzz
			return ADJ_NAVI_NO_RUN;
		if(robot_ajust_run(TYPE_ADJ_CHECK_AGAIN,0,obst))
		{
			
			return ADJ_NAVI_RUN_OK;
		}
		else
			return ADJ_NAVI_RUN_ERR;
	
		
		
	}else
	{
		log_printf("[navitag]navi ERROR\r\n");
		return ADJ_NAVI_RUN_ERR;
	}
	
	
}

//��X���� �������ĸ�����ɨ����û���ϰ���
int four_cln_sta(short x ,short y)
{
	if(SCANE_XY(x,y) && SCANE_XY(x-1,y) && SCANE_XY(x-2,y) && SCANE_XY(x+1,y) && SCANE_XY(x+2,y) &&
		OBST_XY(x,y)==0 && OBST_XY(x-1,y)==0 && OBST_XY(x-2,y)==0 && OBST_XY(x+1,y)==0 && OBST_XY(x+2,y)==0 )
		return 1;
	else
		return 0;
		
}
/*
Ѱ�����������ʺϵĵ�
bx,by ������ȥ��������
ex,ey :�������յ�
dist:�ղŴ�ֱ���ȥ���߹��ľ��룬���ں��˻�ȥ

*/
int navi_to_new_adj(short bx,short by,short ex,short ey,short dist)
{
	int i;
	short ox,oy;
	int deg;
	float angle;
	//���˻�ȥ
	deg = navigat->angle ==0?180:0;
	log_printf("[navi_to_new_adj]turn back,to %d\r\n",deg);
	turn_to_deg(deg);
	if(motor_go_forwark(dist+10,NO_SIDE_NEAR,NULL)==0)
	{
		log_printf("[navi_to_new_adj]walk error\r\n");
		return 0;
	}
	
	//��Y�������ң��ҵ�һƬ����ȥ������,�������Y���ͷ�����ǹ�β�ˡ�
	for(i=1;i<6;i++)
	{
		if(four_cln_sta(X_NOW,Y_NOW+i))		//��������������ϰ����ˣ���ô�������ߵ���������
		{
			//��ͷ�ˡ��������
			ox = bx;
			oy = by -5;	//�������
			log_printf("[navi_to_new_adj]found four cln,may be here(%d,%d,) navi to(%d,%d,)\r\n",X_NOW,Y_NOW+i,ox,oy);
			goto l_navi_to_new_adj;
		}
	}
	for(i=1;i<6;i++)
	{
		//��β��ȥ�ˣ�����ǰ5��
		if(four_cln_sta(X_NOW,Y_NOW - i))		//��������������ϰ����ˣ���ô�������ߵ���������
		{
			ox = bx;
			oy = by + 5;	//����ǰ5��
			log_printf("[navi_to_new_adj]found four cln,may be here(%d,%d,) navi to(%d,%d,)\r\n",X_NOW,Y_NOW - i,ox,oy);
			goto l_navi_to_new_adj;
		}
	}
	return 0;
l_navi_to_new_adj:
	
	angle = navigat->angle ;		//������ĽǶ�
	

	if(walk_map_cord_only(X_NOW,oy,angle,0)==0)
	{
		log_printf("[navi_to_new_adj]walk_map error\r\n");
		return 0;
	}
	log_printf("[navi_to_new_adj]walk to obst...\r\n");
	if(motor_go_forwark(3*HW_GRID,NO_SIDE_NEAR, NULL) ==1)	//���û����ײ��������ˡ�
	{
		log_printf("[navi_to_new_adj]navi targe error!!");
		return 0;
	}
	return 1;
		
}
//90�Ȼ�270����ײ��ʱ��У׼Y��
#if ADJ_Y_BUM	

void ajust_y_on_move(void)
{
	int i,c=0;
	TOBST *t_obst;
	log_printf("[ajust_y_on_move]searchting...\r\n");
	for(i=0;i<navigat->j_obst;i++)
	{
		t_obst = &navigat->adj_obst[i];
		//ֻ���м���ײ�����ܽ���
		if(t_obst->angle == NO_ANGLE_V )
			c++;
		//X����ȡ�Y�����������������յ�,������ʱ������ײ
		if(navigat->tx == t_obst->x && disXY(navigat->ty, t_obst->y) <=2 && t_obst->angle == NO_ANGLE_V && t_obst->sta ==ADJ_MID_BUM)
		{
			if(i>0)
			{
				log_printf("[ajust_y_on_move]found i=%d,(%d,%d,%3.1f,%3.1f)c=%d\r\n",i,t_obst->x,t_obst->y,navigat->adj_obst[i-1].angle,sys->angle,c);
				//�Ƕ�һ�²���У׼
				if(disfloat(navigat->adj_obst[i-1].angle, sys->angle) <=20)
				{
					if(c <=2)
						ajust_xy_org(t_obst->x_org, t_obst->y_org, 80, AJUST_X | AJUST_Y);
					else
						ajust_xy_org(t_obst->x_org, t_obst->y_org, 50, AJUST_X | AJUST_Y);
					navigat->force_obst = 1;		//��ʱ����ǿ�����ϰ����ΪУ׼����Y���������ˣ����ܻ�����ԭ�ȵ�ɨ��������ϰ��
				}
			}
		}
		
	}
}
#endif
#else
char navigat_to_target(uint8_t type)
{

	
	return ADJ_NAVI_NO_RUN;
}	
#endif

void near_turn_check(uint8_t nsta)
{
#if ADJ_NAER_TURN_V


	uint16_t i;
	if(nsta == LEFT_SIDE_NEAR)		//���ر�
	{
		if(LEFT_ADC()  < cfg->lock_left_adc && LEFT_ADC() > cfg->min_left_adc)
		{
			for(i=0;i<4;i++)
			{
				log_printf("[near_turn_check]side=%d,adc=%d,%d\r\n",nsta,LEFT_ADC(),LEFT_ADC1());
				if(LEFT_ADC()  < cfg->lock_left_adc && LEFT_ADC() > cfg->min_left_adc)
				{
					log_printf("in lost side,turn to left\r\n");
					robot_turn_deg(GO_LEFT,650,8);
				}else
					break;
			}
		}
		if(LEFT_ADC() > cfg->max_left_adc || LEFT_ADC1() >= cfg->max_left2_adc)
		{
			for(i=0;i<4;i++)
			{
				log_printf("[near_turn_check]side=%d,adc=%d,%d\r\n",nsta,LEFT_ADC(),LEFT_ADC1());
				if(LEFT_ADC() > cfg->max_left_adc || LEFT_ADC1() >= cfg->max_left2_adc)
				{
					log_printf("in near side,turn to right\r\n");
					robot_turn_deg(GO_RIGTH,650,8);
				}else
					break;
			}

		}
		
	}else if(nsta == RIGHT_SIDE_NEAR)
	{
		if(RIGHT_ADC()  < cfg->lock_right_adc && RIGHT_ADC() > cfg->min_right_adc)
		{
			for(i=0;i<4;i++)
			{
				log_printf("[near_turn_check]side=%d,adc=%d,%d\r\n",nsta,RIGHT_ADC(),RIGHT_ADC1());
				if(RIGHT_ADC()  < cfg->lock_right_adc && RIGHT_ADC() > cfg->min_right_adc)
				{
					log_printf("in lost side,turn to rigjt\r\n");
					robot_turn_deg(GO_RIGTH,650,8);
				}else
					break;
			}
		}
		if(RIGHT_ADC() > cfg->max_right_adc || RIGHT_ADC1() >= cfg->max_right2_adc)
		{
			for(i=0;i<4;i++)
			{
				log_printf("[near_turn_check]side=%d,adc=%d,%d\r\n",nsta,RIGHT_ADC(),RIGHT_ADC1());
				if(RIGHT_ADC() > cfg->max_right_adc || RIGHT_ADC1() >= cfg->max_right2_adc)
				{
					log_printf("in near side,turn to right\r\n");
					robot_turn_deg(GO_LEFT,650,8);
				}else
					break;
			}

		}


	}
#endif	
}


/*
������ɨ��ʱ��
�ӱ���һ�Σ������ӱߵ�����
*/
int16_t target_run_z_go(void)
{
#if TARGE_RUN
	uint8_t n_sta;
	int deg;
	int16_t by=Y_NOW,idx;
	
	
	TARGET_T *target;
	CHECK_NAVI_STA_RT(0);

	if(!MIDLE_HAVE_IRDA2())
	{
		log_printf("[target_run_z_go]not mid\r\n");
		return 0;
	}
	deg = (int)navigat->lst_angle;
	target = get_target(deg,0,by,&idx);
	if(target != NULL)
	{
		int16_t x,y;
		x = xy[idx].x;
		y = xy[idx].y;
		log_printf("[target_run_z_go]found target=(%d,%d,%d),no run...\r\n",x,y,idx);
		//���������
		return 0;
	}
		
	if(navigat->side == LEFT_SIDE)
	{
		n_sta = navigat->lst_angle == 0?RIGHT_SIDE_NEAR:LEFT_SIDE_NEAR;
		deg = 90;
	}else
	{
		n_sta = navigat->lst_angle == 0?LEFT_SIDE_NEAR:RIGHT_SIDE_NEAR;
		deg = 270;
	}
	log_printf("[target_run_z_go](%d,%d,%3.1f)nsta=%d,turn to=%d\r\n",navigat->tx,navigat->ty,navigat->angle,n_sta,deg);
	turn_to_deg_v(deg);

	
	target_run_check(0,deg);
	motor_go_near_wall(0, n_sta,GO_TYPE_SAVE,OUT_TARGET_RUN,0,0);	
	n_sta = n_sta == LEFT_SIDE_NEAR?RIGHT_SIDE_NEAR:LEFT_SIDE_NEAR;
	if(n_sta == LEFT_SIDE_NEAR)
		robot_turn_deg(GO_LEFT,DEG_TURN_PWM,WALL_LOST_DEG);
	else
		robot_turn_deg(GO_RIGTH,DEG_TURN_PWM,WALL_LOST_DEG);
	
		
	motor_go_near_wall(by, n_sta,GO_TYPE_SAVE,OUT_TARGET_BACK,0,0);	
	if(navigat->lst_angle == 0)
		turn_to_deg(180);
	else
		turn_to_deg(0);
#endif		
	return 1;
}

/*
����X��Y �ҵ���Ӧ��������
*/
TARGET_T *get_target(int deg,int16_t xx,int16_t yy,int16_t *idx)
{
#if TARGE_RUN

	TARGET_T *target=NULL;
	for(int16_t i=0;i<navigat->i_target;i++)
	{
		target = &navigat->target[i];
		if(deg==0 || deg ==180)
		{
			if(target->deg != deg)
				continue;
		}
		for(int16_t j=target->bidx;j<target->eidx;j++)
		{
			if(xx >0 )
			{
				if(xx ==xy[j].x && yy == xy[j].y)
				{
					*idx = j;
					return target;
				}
			}else if( yy == xy[j].y)
			{
				*idx = j;
				return target;
			}
			
		}
	}
#endif	
	return NULL;
}

int16_t target_run_check(uint8_t type,int deg)
{
#if TARGE_RUN

	static int16_t by,bx;
	static int 	bdeg;
	int16_t ret;
	switch(type)
	{
		//	��ʼ����
		case 0:
			by = Y_NOW;
			bx = X_NOW;
			bdeg = deg;
			return 0;
		case 1:	
			//��ײ
			if(disxy(Y_NOW,by) >=10 || disxy(X_NOW,bx) >=10)
			{
				log_printf("[target_run]YOK\r\n");
				return 	1;
			}
		case 2: //�ӱ�
			//�Ƕ��ж� �����ۻ���
			if(bdeg == 90)
			{
				if(sys->angle > 220 && sys->angle < 330)
				{
					log_printf("[target_run]angle err,%d->%3.1f\r\n",deg,sys->angle);
					return 	1;
				}
			}else if(bdeg ==270)
			{
				if(sys->angle > 30 && sys->angle < 150)
				{
					log_printf("[target_run]angle errs,%d->%3.1f\r\n",deg,sys->angle);
					return	1;
				}

			}
	}	
#endif	
	return 0;
	
}
#define MAX_LINEX_OBST		5
//�������ϰ���,�ǲ���һ���ߣ���¼��������Ϊ������������
void side_obst_line(void)
{
	log_printf("[side_obst_line]wlen=%d,olen=%d\r\n",navigat->walk_len,navigat->c_side_obst);
	if(navigat->c_side_obst < MAX_LINEX_OBST)			//�������ϰ���С��5 ֱ�ӷ���
		return ;
	int16_t y_dir = navigat->side == LEFT_SIDE?1:-1;
	int16_t bx=0,by=0,ex=0;
	uint8_t sta;
	int16_t x,y;
	int16_t i;
	for(i=0;i<navigat->walk_len;i++)
	{
		x = navigat->x_walk[i];
		y = navigat->y_walk[i];
		sta = point_sta(x,y+y_dir);
		//ɨ�����һ��
		if(sta == OBST_STA)
		{
			if(bx ==0)
			{
				ex = bx = x;
				by = y;				
			}
		}
		//ɨ�赽ͷ��
		if(ex !=0 )	//ɨ����ϰ�����
		{
			if(sta != OBST_STA || by != y)
			{
				ex = x;
				break;
			}
		}
	}
	log_printf("check sideobst(%d,%d,)->(%d,%d,)\r\n",bx,by,ex,by);
	if(disxy(bx,ex) >=MAX_LINEX_OBST)
	{
		if(navigat->c_linex < MAX_LINEX)
		{
			LINEX  *linex = &navigat->linex[navigat->c_linex++];
			linex->tx = (bx+ex) >> 1;
			linex->ty = by;
			linex->dir = y_dir;			//�ϰ���ķ���
			log_printf("[side_obst_line]insert,len=%d,piont=(%d,%d,)\r\n",linex->tx,linex->ty,linex->dir);
		}
	}
	
}




//micony20190306	�ر߻���ͼ����
/************************************************************************************
 * ��������: go_edgeways_for_map
 * ��    ��: ������ɨ��ʱ�����ر߻�����ͼ
 * ��    ��: n_sta - �ر߷���
 *			 maxdist, X,Y���ƶ������Ĳ�ֵ��������������ֹͣ�ˡ�
 * ��    ��:
 * ˵    ��: ������ɨ�ߵ��ף�����Ǻ�����ײ����û���ع��ߣ�����øú����ر߻���ͼ��
 *************************************************************************************/
uint8_t go_edgeways_for_map(uint8_t n_sta ,int16_t maxdist,int begin_deg )
{
#if EDGE_DRAW_MAP
	int calue=0;
	uint8_t sta;
	int16_t llx=0,lly=0;
	int16_t bx = X_NOW,by = Y_NOW;					//��¼��ʼ�����꣬���ڼ����رߵ�λ�ˡ�

	float lagle;						//��¼�����ӱ߽Ƕȣ����ڼ����Ƿ�ת�����
	uint32_t	t_begin;				//��ʼ��ʱ��
	int ret_calc=1;
	int16_t c_lost=0;
	uint16_t c_round=0;		//תȦ�Ĵ���
	int c_near_wall = 0;
    int16_t tox;
    int16_t toy;

//short x_begin_line=0,y_begin_line = 0;		//һ���߿�ʼ��X��Y������
	
	int16_t dist_y = dis_xy(Y_NOW,toy)+2;
	
	NEAR_WALL *near = &navigat->near;

	float m_angle[MAX_C_M_ANGLE];			//���������20����ײ�ĽǶ�
	int16_t c_m_angle=0;
	
#if LAGER_ROUND_MODE	
	int16_t cyc_x[MAX_C_M_ANGLE],cyc_y[MAX_C_M_ANGLE];
	uint8_t tmp=0;
#else
	int16_t c_near_wall1 = 0;
#endif	

	for(c_m_angle = 0;c_m_angle<MAX_C_M_ANGLE;c_m_angle++)
		m_angle[c_m_angle] = 0;
	c_m_angle = 0;

		
//	motor_run(GO_ROUND,600,0,0);
	CHECK_NAVI_STA_RT(0);
	
	navigat->out =navigat->angle;
	navigat->distance = 0;
	motor.c_left_hw = motor.c_right_hw = 0;
	cord_calc_store(0);
	gyro_whlmap();
#if JUDGE_PID_CTRL_WHEEL_STOP
	m_speed_pid_ctrl(0,0,0,0,0,0,0);		//��¼���ֻ�����
#else	
	m_speed_pid_ctrl(0,0);
#endif	

	pd_gyro_int(GO_NEAR_PWM_FAST);			//��ʼ���ǶȻ�PID
	init_near_wall_navi(n_sta);				//��ʼ���ر�PID
	
	motor_run(GO_FORWARD, GO_NEAR_PWM_FAST, 0, 0);	//��·
	navigat->c_go = navigat->c_turn = 0;       // add  2019 03 09
	
	navigat->is_walk =1;
	log_printf("[go_edgeways_for_map]sta=%d(%d),now=(%d,%d,)to=(%d,%d,)dis=%d\r\n",navigat->near.n_sta,n_sta,X_NOW,Y_NOW,tox,toy,dist_y);
	//navigat->wheel_dir = 1;
	sta = sys->sState;
	lagle = sys->angle;
	mstimeout(&t_begin,0);
	while(1)
	{

		proc_uart_task();
		get_sensers(&sys->gSta);
		if(sys->sState !=sta)
			return RET_NEAR_ERROR;
		CHECK_NAVI_STA_RT(0);
		//��ײ
		if(sys->gSta & (MASK_BUM_MIDL) || ret_calc==0 ||  c_lost >=MAX_NEAR_LOST)
		{
//			gSta = sys->gSta;	//��¼��ײ��״̬
			motor_run(GO_STOP,0,0,0);
			//log_printf("BUM\r\n");
			coordinate_calcu(0);														//�����ԭʼ������ϵ
			coord_org2map(navigat->x_org,navigat->y_org,&navigat->tx,&navigat->ty); //����ϵת��
			motor_run(GO_STOP,0,0,0);
			
			log_printf("\r\n-----go_edgeways_for_map bum(%d,%d,%d,%f,%f),gsta=%d,irda=(%d,%d,%d,%d)angle=%3.1f,ret=%d,lost=%d\r\n",navigat->tx,navigat->ty,motor.c_left_hw,navigat->x_org_f,navigat->y_org_f,
							sys->gSta,sys->g_sta[0],sys->g_sta[1],sys->g_sta[5],sys->g_sta[6],sys->angle,ret_calc,c_lost);
			
			side_obst_by_bum(sys->gSta);	//���ϰ���
			if(!(ret_calc==0 || c_lost >=MAX_NEAR_LOST))
				c_round = 0;
			if(ret_calc==0)
					gyro_mapwhl();

			lagle = sys->angle;
		
			navigat->distance = 0;
			motor.c_left_hw = 0;
			
#if !LAGER_ROUND_MODE	
			if(c_m_angle >=MAX_C_M_ANGLE)
				c_m_angle = 0;
			m_angle[c_m_angle++] = sys->angle;		//���ֽǶ�

			if(near_round_360(m_angle,c_m_angle))
			{
			/*
				log_printf("found round...go line...\r\n");

				init_near_wall_navi(NO_SIDE_NEAR);
				for(c_m_angle = 0;c_m_angle<MAX_C_M_ANGLE;c_m_angle++)
					m_angle[c_m_angle] = 0;
				c_m_angle = 0;
				//goto l_go_edeways;
				return RET_NEAR_OK;	
				*/
			}
#else////2018-07-10  δ����6
			if(near_large_round_360(cyc_x,cyc_y,m_angle,c_m_angle)) 					///�ƴ�׮�жϺ���
			{
			/*
				log_printf("found large round...go line...\r\n");
				if(near->n_sta == RIGHT_SIDE_NEAR)
				{
					robot_turn_deg(GO_LEFT,DEG_TURN_PWM,135);
					//goto l_mgo_edeways;
				}else if(near->n_sta == LEFT_SIDE_NEAR)
				{
					robot_turn_deg(GO_RIGTH,DEG_TURN_PWM,135);
					//goto l_mgo_edeways;
				}						
				for(tmp=0;tmp<c_m_angle;tmp++)
				{
					log_printf("%02d(%03d,%03d,%03f)\r\n",tmp,cyc_x[tmp],cyc_y[tmp],m_angle[tmp]);
				}
				init_near_wall_navi(NO_SIDE_NEAR);
				for(c_m_angle = 0;c_m_angle<MAX_C_M_ANGLE;c_m_angle++)
					m_angle[c_m_angle] = 0;
				c_m_angle = 0;
				return RET_NEAR_OK;	
				*/
			}	
#endif
			/**************************************************************************
				������ߵ�·�̳������޶���·�̣���ͣ����
			*****************************************************************************/
			if(  disxy(bx,X_NOW) >=maxdist || disxy(by,Y_NOW) >= maxdist) 
			{
				log_printf("xy ok3(%d,%d,)->(%d,%d,%3.1f),\r\n",bx,by,X_NOW,Y_NOW,sys->angle);
				motor_run(GO_STOP,0,0,0);
				return RET_NEAR_OK;				
			}	

			if((begin_deg == 90 &&  disfloat(sys->angle,270)< 20) || 
				(begin_deg == 270 &&  disfloat(sys->angle,90)< 20))
            {
				log_printf("angle ok(%d,%3.1f)\r\n",begin_deg,sys->angle);
				motor_run(GO_STOP,0,0,0);
				return RET_NEAR_OK;				
			}	
			
 

	        
			if(near->n_sta ==NO_SIDE_NEAR)
			{
				init_near_wall_navi(n_sta);
			}


			
			if(near->n_sta == RIGHT_SIDE_NEAR)
			{
				//modify201710	�������bug�����·�����ײ��֮ǰ��֪��Ϊ�����
				/*
				if((gSta & MASK_BUM_LEFT))
				{
					log_printf("midbum,10deg\r\n");
					robot_turn_deg(GO_RIGTH,TURN_DEG_PWM,10);
				}else 
				*/
				{
					robot_turn_deg(GO_LEFT,TURN_DEG_PWM,12);
				}
			}else if(near->n_sta == LEFT_SIDE_NEAR)
			{
				//log_printf("gsta=%d,left=%d,%d\r\n",sys->gSta,sys->g_sta[0],sys->g_sta[1]);
				//modify201710	�������bug�����·�����ײ��֮ǰ��֪��Ϊ�����
				/*
				if((gSta & MASK_BUM_RIGHT)  )
				{
					log_printf("midbuml,10deg\r\n");
					robot_turn_deg(GO_LEFT,TURN_DEG_PWM,10);
				}else 
				*/
				{
						//log_printf("midbuml,12deg\r\n");
					robot_turn_deg(GO_RIGTH,TURN_DEG_PWM,12);
				}

			}else
			{
				log_printf("RET_NEAR_ERROR,nsta=%d\r\n",n_sta);
				return RET_NEAR_ERROR;	
			}
//�ll_go_edeways1:		
			c_lost=0;
			//coordinate_calcu(); 	
			motor_run(GO_STOP,0,0,0);
		//	log_printf("after bk(%d,%d,%d,%3.3f,%3.3f,%3.3f)\r\n==============\r\n",navigat->tx,navigat->ty,motor.c_left_hw,navigat->x_org_f,navigat->y_org_f,sys->angle);
			//if(ccc++ >=5)
			//	while(1);
			navigat->distance = 0;
			navigat->is_walk = 1;
			pd_gyro_int(GO_NEAR_PWM_FAST);
			navigat->out =sys->angle;
			cord_calc_store(0);
			//gyro_whlmap();
			
			motor_run(GO_FORWARD, GO_NEAR_PWM_FAST, 0, 0);
			motor.c_left_hw = motor.c_right_hw = 0;
#if JUDGE_PID_CTRL_WHEEL_STOP
	m_speed_pid_ctrl(0,0,0,0,0,0,0);		//��¼���ֻ�����
#else			
			m_speed_pid_ctrl(0,0);
#endif			
			c_near_wall = 0;
			navigat->near.pid->c_lost = 0;
			//navigat->near.pid->c_lost = 0;
			navigat->near.pid->c_lost_flag = 0;
			
			ret_calc = 1;

#if !LAGER_ROUND_MODE			
			c_near_wall1 = 0;
#endif
		}

		if(TIM5->CNT >=5000)
		{
			TIM5->CNT = 0;
			www_idleintel_com();

			navigat->out = format_agle(navigat->out,ANGLE_360);
			proc_line_pid(navigat->out);
			navigat_near_wall_pid(&navigat->out,5);
			if(near->n_sta ==NO_SIDE_NEAR)
			{
				if(RIGHT_ADC() >= cfg->lock_right_adc)
					init_near_wall_navi(RIGHT_SIDE_NEAR);
				else if(LEFT_ADC() >= cfg->lock_left_adc)
					init_near_wall_navi(LEFT_SIDE_NEAR);
			}
			
			if( *(navigat->near.pid->adc) > navigat->near.pid->min_adc) //�ӱ�
			  lagle = sys->angle;
			else		//�ӱ���ʧ��ת�ĽǶȳ���180�ȣ���ʧ���˳�
			{
				if(disfloat( lagle , sys->angle) > 180)
				{
					log_printf("lost over(%d,%d,%3.1f,%3.1f)\r\n",X_NOW,Y_NOW,sys->angle,lagle);
					motor_run(GO_STOP,0,0,0);
					//delay_ms_sensers(200);
					coordinate_calcu(0);														//�����ԭʼ������ϵ
					coord_org2map(navigat->x_org,navigat->y_org,&navigat->tx,&navigat->ty);	//����ϵת��
					return RET_NEAR_ERROR;	
				}
			}
#if CALE_BY_FRON				
				ret_calc = coordinate_calcu(1);														//�����ԭʼ������ϵ
#endif	
			if(calue++ >=40)
			{
				calue = 0;
#if !CALE_BY_FRON				
				ret_calc = coordinate_calcu(0);														//�����ԭʼ������ϵ
#endif	
				//ret_calc = coordinate_calcu();														//�����ԭʼ������ϵ
				coord_org2map(navigat->x_org,navigat->y_org,&navigat->tx,&navigat->ty);	//����ϵת��

#if !LAGER_ROUND_MODE				
				if(c_near_wall1 ++ >=30)
				{
	
					for(c_m_angle = 0;c_m_angle<MAX_C_M_ANGLE;c_m_angle++)
						m_angle[c_m_angle] = 0;
					c_m_angle = 0;
					c_near_wall1 = 0;
					log_printf("log near\r\n");
				}
#endif				
				//�ӱ�����
				if(LEFT_ADC() > cfg->lock_left_adc || RIGHT_ADC() > cfg->lock_right_adc)
					c_near_wall ++ ;
#if !JUDGE_PID_CTRL_WHEEL_STOP						
				if(c_near_wall >=20 && dis_xy(L_FORWRK_PWM,R_FORWRK_PWM) < 100)
				{	
					m_speed_pid_ctrl(0,1);			///				
				}	
#endif					
				c_near_wall = 0;
				
				if( *(navigat->near.pid->adc) > navigat->near.pid->min_adc) //�ӱ�
				{
					//c_near++;
					c_round = 0;					
				}
				else if(near->n_sta !=NO_SIDE_NEAR)	//ǿ���ӱߣ��������ʧ�ĸ���
					c_lost ++;	
					
				if(llx!=navigat->tx || lly!=navigat->ty)
				{
#if WALK_PRINTF				
					log_printf("*(%d,%d,%3.1f,0)-[%3.1f,%3.1f,0]*\r\n",navigat->tx,navigat->ty,sys->angle,navigat->x_org_f,navigat->y_org_f);
#endif
#if WIFICONFIG					
		updata_stream_cache_data(navigat->tx,navigat->ty,0,POINT_SCAN);						
#endif	

#if LAGER_ROUND_MODE
					if(c_m_angle >=MAX_C_M_ANGLE)
					{
						for(tmp=0;tmp<MAX_C_M_ANGLE-1;tmp++)
						{
							cyc_x[tmp] = cyc_x[tmp+1];
							cyc_y[tmp] = cyc_y[tmp+1];
							m_angle[tmp] = m_angle[tmp+1] ; 
						}	
						cyc_x[MAX_C_M_ANGLE-1] = X_NOW;
						cyc_y[MAX_C_M_ANGLE-1] = Y_NOW;
						m_angle[MAX_C_M_ANGLE-1] = sys->angle;				//��֤���µ�40����
					}
					else			
					{
						cyc_x[c_m_angle] = X_NOW;
						cyc_y[c_m_angle] = Y_NOW;
						m_angle[c_m_angle++] = sys->angle;		//����Ƕ�				
					}	
#endif

					llx = navigat->tx;
					lly = navigat->ty;

					
				}
					/**************************************************************************
						������ߵ�·�̳������޶���·�̣���ͣ����
					*****************************************************************************/
					if(  disxy(bx,X_NOW) >=maxdist || disxy(by,Y_NOW) >= maxdist) 
					{
						log_printf("xy ok4(%d,%d,)->(%d,%d,%3.1f),\r\n",bx,by,X_NOW,Y_NOW,sys->angle);
						motor_run(GO_STOP,0,0,0);
						return RET_NEAR_OK;				
					}	
					/*
					 if ((disxy(begin_deg,navigat->lst_angle))<=20)

			        {
						log_printf("angle ok(%d,%d)\r\n",begin_deg,navigat->lst_angle);
						motor_run(GO_STOP,0,0,0);
						return RET_NEAR_OK;				
					}	
					*/
					if((begin_deg == 90 &&  disfloat(sys->angle,270)< 20) || 
						(begin_deg == 270 &&  disfloat(sys->angle,90)< 20))
		            {
						log_printf("angle ok(%d,%3.1f)\r\n",begin_deg,sys->angle);
						motor_run(GO_STOP,0,0,0);
						return RET_NEAR_OK;				
					}	
					
 
					side_obst_by_ir();		//�����ϰ���
					
			}
		}		
	}
#else	
	return 0;
#endif
}

