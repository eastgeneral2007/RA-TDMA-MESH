#ifdef __KERNEL__
#include <linux/time.h>
#else
#include <sys/time.h>
#endif

//#include <linux/time.h>
//#include <linux/in.h>

#ifndef __DRK_SCHED_H__
#define __DRK_SCHED_H__

#define DAEMON_PRIORITY 4;
// calls
#define CREATE_RESERVE 1
#define ATTACH_RESERVE 2
#define DETACH_RESERVE 3
#define DELETE_RESERVE 4
#define WAIT_NEXT_PERIOD 5
#define BUDGET_OVERRUN 6
#define GET_JIFFIES_MS 7
#define GET_RESERVE_ID 8
#define SCHED_PAUSE 9
#define SCHED_LAUNCH_ALL 10
#define RESERVE_WAIT 11
#define RESERVE_WAIT_UNTIL 12

#define TIMER_PERIOD 1

#define MAX_RESERVES 64

#define REMOTE_SERVER_PORT 1500

#define DRK_TIMER_SIGNAL (SIGRTMIN+1)

#define MAX_DEGRADED_MODES 4

#define GET_EFFECTIVE_UTILITY(i) (reserve_table[i].current_degraded_mode == -1 ? reserve_table[i].params.overloaded_marginal_utility: reserve_table[i].params.degraded_marginal_utility[reserve_table[i].current_degraded_mode][1])

/*struct drk_timer{
	timer_t tid;
	int timer_type;
	int reserve_desc;
	struct timespec expiration;
};*/

struct drk_sched_task_struct{
	int pid;
	void (*task)(void);
	struct timespec period;
	struct timespec execution_time;
};

#ifdef __KERNEL__
struct drk_sched_reserve {
	int priority;
	int sleeping;
	int paused;
	int degraded;
	int runnable;
	struct hrtimer period_timer;
	struct hrtimer enforcement_timer;
	struct hrtimer wakeup_timer;
	struct timespec start_of_period;
	struct drk_sched_task_struct params;
};
#endif

struct attach_api{
	int reserveid;
	int pid;
	int time;
};

struct api_call{
	int api_id;
	union {
		int reserveid;
		struct attach_api attach_params;
		struct drk_sched_task_struct reserve_parameters;
	}args;
};

// library calls signatures
int drk_sched_open(void);
int drk_sched_pause(int fd);
int drk_sched_launch_all(int fd);
int drk_sched_reserve_getid(void);
int drk_sched_reserve_create(int fd, struct drk_sched_task_struct *p);
int drk_sched_reserve_delete(int rid);
int drk_sched_get_jiffies_ms(int fd);
int drk_sched_close(void);
int drk_sched_wait_next_period(int rid);
int drk_sched_wait(int rid, int time_ms);
int drk_sched_wait_until(int rid, int time_ms);
#endif
