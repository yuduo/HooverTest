
#ifndef _H_RANDCLEAN_
#define _H_RANDCLEAN_

#define RMCLN_RM	0x00		//���
#define RMCLN_NEAR	0x01		//��ǽ

typedef struct rm_cln_t
{
	uint8_t gSta;		//��ɨģʽ
	uint8_t slip;
	uint8_t	is_near;
	uint8_t	c_turn_near_right;		//ת�˶�κ󣬾Ͳ�����ת�ˡ�
	uint8_t	short_distance;			//�Ƿ�ʱ��Ķ̾������С�
	uint16_t	c_near_wall;		//��ǽ�˶��ʱ��
	int		check_lost;
}RmCln_t;

void proc_random_task(void);
void init_random_task(void);
char get_rand_idx(void);
void turn_vertical_right(void)	;
void turn_near_right(void);
//void near_wall_pid(void);
void proc_test_task(void);
//void near_wall_pid(float *agle,int c_lost);


#endif
