#ifdef __KERNEL__
#include <linux/time.h>
#else
#include <sys/time.h>
#endif

#ifndef __ZSRM_H__
#define __ZSRM_H__

#define DAEMON_PRIORITY 50;
// calls
#define CREATE_RESERVE 1
#define ATTACH_RESERVE 2
#define DETACH_RESERVE 3
#define DELETE_RESERVE 4
#define WAIT_NEXT_PERIOD 5
#define BUDGET_OVERRUN 6
#define GET_JIFFIES_MS 7

#define TIMER_ZS  1
#define TIMER_ENF 2
#define TIMER_ZS_ENF 3
#define TIMER_PERIOD 4

#define MAX_RESERVES 64

#define REMOTE_SERVER_PORT 1500

#define ZS_TIMER_SIGNAL (SIGRTMIN+1)
#define ZS_BUDGET_TIMER_SIGNAL (SIGRTMIN+2)

#define ZS_RESPONSE_TIME_ENFORCEMENT_MASK 0x1
#define ZS_PERIOD_DEGRADATION_MAKS 0x2
#define ZS_CRITICALITY_ENFORCEMENT_MASK 0x4

#define MAX_DEGRADED_MODES 4

#define GET_EFFECTIVE_UTILITY(i) (reserve_table[i].current_degraded_mode == -1 ? reserve_table[i].params.overloaded_marginal_utility: reserve_table[i].params.degraded_marginal_utility[reserve_table[i].current_degraded_mode][1])

struct zs_reserve_params{
	struct timespec period;
	struct timespec execution_time;
	struct timespec zs_instant;
	struct timespec response_time_instant;
	long normal_marginal_utility;
	long overloaded_marginal_utility;
	int num_degraded_modes;
	int critical_util_degraded_mode;
	int enforcement_mask;
	int criticality;
	int priority;
	long degraded_marginal_utility[MAX_DEGRADED_MODES][2];
	struct timespec degraded_period[MAX_DEGRADED_MODES];
	int degraded_priority[MAX_DEGRADED_MODES];
};

#ifdef __KERNEL__
struct zs_reserve {
	int pid;
	int effective_priority;
	int request_stop;
	struct hrtimer period_timer;
	struct hrtimer zs_timer;
	struct hrtimer response_time_timer;
	int in_critical_mode;
	int critical_utility_mode_enforced;
	int current_degraded_mode;
	int just_returned_from_degradation;
	struct zs_reserve_params params;
	struct timespec start_of_period;
};
#endif

struct attach_api{
	int reserveid;
	int pid;
};

struct api_call{
	int api_id;
	union {
		int reserveid;
		struct attach_api attach_params;
		struct zs_reserve_params reserve_parameters;
	}args;
};

// library calls signatures
int zs_open_sched(void);
int zs_close_sched(int fd);
int zs_create_reserve(int fd, struct zs_reserve_params *p);
int zs_attach_reserve(int fd, int rid, int pid);
int zs_detach_reserve(int fd, int rid);
int zs_delete_reserve(int fd, int rid);
int zs_wait_next_period(int fd, int rid);
int zs_get_jiffies_ms(int fd);
#endif
