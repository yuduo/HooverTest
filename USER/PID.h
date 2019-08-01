#ifndef _PID_H
#define _PID_H

typedef struct near_pid_t
{
	unsigned char level;
}NEAR_PID;

extern 	NEAR_PID  r_near_pid,l_near_pid;

typedef struct _pid_t{
	float SetSpeed;		
}pid_t;

#endif

