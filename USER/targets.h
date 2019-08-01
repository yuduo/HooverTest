#ifndef _TARGET_H_
#define _TARGET_H_

char robot_ajust_run(uint8_t type,int x_dir,TOBST *g_obst);
char adj_back_edgeways(uint8_t n_sta ,short tox,short toy);
void near_turn_check(uint8_t nsta);
int16_t run_ajust_check(void);
int16_t target_run_z_go(void);
TARGET_T *get_target(int deg,int16_t xx,int16_t yy,int16_t *idx);
int16_t target_run_check(uint8_t type,int deg);
void side_obst_line(void);
uint8_t go_edgeways_for_map(uint8_t n_sta ,int16_t maxdist,int begin_deg);
#endif

