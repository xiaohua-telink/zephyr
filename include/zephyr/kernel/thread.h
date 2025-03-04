/*
 * Copyright (c) 2016, Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_KERNEL_THREAD_H_
#define ZEPHYR_INCLUDE_KERNEL_THREAD_H_

#ifdef CONFIG_DEMAND_PAGING_THREAD_STATS
#include <zephyr/sys/mem_manage.h>
#endif

#include <zephyr/kernel/stats.h>

/**
 * @typedef k_thread_entry_t
 * @brief Thread entry point function type.
 *
 * A thread's entry point function is invoked when the thread starts executing.
 * Up to 3 argument values can be passed to the function.
 *
 * The thread terminates execution permanently if the entry point function
 * returns. The thread is responsible for releasing any shared resources
 * it may own (such as mutexes and dynamically allocated memory), prior to
 * returning.
 *
 * @param p1 First argument.
 * @param p2 Second argument.
 * @param p3 Third argument.
 */

#ifdef CONFIG_THREAD_MONITOR
struct __thread_entry {
	k_thread_entry_t pEntry;
	void *parameter1;
	void *parameter2;
	void *parameter3;
};
#endif

/* can be used for creating 'dummy' threads, e.g. for pending on objects */
struct _thread_base {

	/* this thread's entry in a ready/wait queue */
	union {
		sys_dnode_t qnode_dlist;
		struct rbnode qnode_rb;
	};

	/* wait queue on which the thread is pended (needed only for
	 * trees, not dumb lists)
	 */
	_wait_q_t *pended_on;

	/* user facing 'thread options'; values defined in include/kernel.h */
	uint8_t user_options;

	/* thread state */
	uint8_t thread_state;

	/*
	 * scheduler lock count and thread priority
	 *
	 * These two fields control the preemptibility of a thread.
	 *
	 * When the scheduler is locked, sched_locked is decremented, which
	 * means that the scheduler is locked for values from 0xff to 0x01. A
	 * thread is coop if its prio is negative, thus 0x80 to 0xff when
	 * looked at the value as unsigned.
	 *
	 * By putting them end-to-end, this means that a thread is
	 * non-preemptible if the bundled value is greater than or equal to
	 * 0x0080.
	 */
	union {
		struct {
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
			uint8_t sched_locked;
			int8_t prio;
#else /* LITTLE and PDP */
			int8_t prio;
			uint8_t sched_locked;
#endif
		};
		uint16_t preempt;
	};

#ifdef CONFIG_SCHED_DEADLINE
	int prio_deadline;
#endif

	uint32_t order_key;

#ifdef CONFIG_SMP
	/* True for the per-CPU idle threads */
	uint8_t is_idle;

	/* CPU index on which thread was last run */
	uint8_t cpu;

	/* Recursive count of irq_lock() calls */
	uint8_t global_lock_count;

#endif

#ifdef CONFIG_SCHED_CPU_MASK
	/* "May run on" bits for each CPU */
	uint8_t cpu_mask;
#endif

	/* data returned by APIs */
	void *swap_data;

#ifdef CONFIG_SYS_CLOCK_EXISTS
	/* this thread's entry in a timeout queue */
	struct _timeout timeout;
#endif

#ifdef CONFIG_TIMESLICE_PER_THREAD
	int32_t slice_ticks;
	k_thread_timeslice_fn_t slice_expired;
	void *slice_data;
#endif

#ifdef CONFIG_SCHED_THREAD_USAGE
	struct k_cycle_stats  usage;   /* Track thread usage statistics */
#endif
};

typedef struct _thread_base _thread_base_t;

#if defined(CONFIG_THREAD_STACK_INFO)
/* Contains the stack information of a thread */
struct _thread_stack_info {
	/* Stack start - Represents the start address of the thread-writable
	 * stack area.
	 */
	uintptr_t start;

	/* Thread writable stack buffer size. Represents the size of the actual
	 * buffer, starting from the 'start' member, that should be writable by
	 * the thread. This comprises of the thread stack area, any area reserved
	 * for local thread data storage, as well as any area left-out due to
	 * random adjustments applied to the initial thread stack pointer during
	 * thread initialization.
	 */
	size_t size;

	/* Adjustment value to the size member, removing any storage
	 * used for TLS or random stack base offsets. (start + size - delta)
	 * is the initial stack pointer for a thread. May be 0.
	 */
	size_t delta;
};

typedef struct _thread_stack_info _thread_stack_info_t;
#endif /* CONFIG_THREAD_STACK_INFO */

#if defined(CONFIG_USERSPACE)
struct _mem_domain_info {
	/** memory domain queue node */
	sys_dnode_t mem_domain_q_node;
	/** memory domain of the thread */
	struct k_mem_domain *mem_domain;
};

#endif /* CONFIG_USERSPACE */

#ifdef CONFIG_THREAD_USERSPACE_LOCAL_DATA
struct _thread_userspace_local_data {
#if defined(CONFIG_ERRNO) && !defined(CONFIG_ERRNO_IN_TLS) && !defined(CONFIG_LIBC_ERRNO)
	int errno_var;
#endif
};
#endif

typedef struct k_thread_runtime_stats {
#ifdef CONFIG_SCHED_THREAD_USAGE
	uint64_t execution_cycles;
	uint64_t total_cycles;        /* total # of non-idle cycles */
	/*
	 * In the context of thread statistics, [execution_cycles] is the same
	 * as the total # of non-idle cycles. In the context of CPU statistics,
	 * it refers to the sum of non-idle + idle cycles.
	 */
#endif

#ifdef CONFIG_SCHED_THREAD_USAGE_ANALYSIS
	/*
	 * For threads, the following fields refer to the time spent executing
	 * as bounded by when the thread was scheduled in and scheduled out.
	 * For CPUs, the same fields refer to the time spent executing
	 * non-idle threads as bounded by the idle thread(s).
	 */

	uint64_t current_cycles;      /* current # of non-idle cycles */
	uint64_t peak_cycles;         /* peak # of non-idle cycles */
	uint64_t average_cycles;      /* average # of non-idle cycles */
#endif

#ifdef CONFIG_SCHED_THREAD_USAGE_ALL
	/*
	 * This field is always zero for individual threads. It only comes
	 * into play when gathering statistics for the CPU. In that case it
	 * represents the total number of cycles spent idling.
	 */

	uint64_t idle_cycles;
#endif

#if defined(__cplusplus) && !defined(CONFIG_SCHED_THREAD_USAGE) &&                                 \
	!defined(CONFIG_SCHED_THREAD_USAGE_ANALYSIS) && !defined(CONFIG_SCHED_THREAD_USAGE_ALL)
	/* If none of the above Kconfig values are defined, this struct will have a size 0 in C
	 * which is not allowed in C++ (it'll have a size 1). To prevent this, we add a 1 byte dummy
	 * variable when the struct would otherwise be empty.
	 */
	uint8_t dummy;
#endif
}  k_thread_runtime_stats_t;

struct z_poller {
	bool is_polling;
	uint8_t mode;
};

/**
 * @ingroup thread_apis
 * Thread Structure
 */
struct k_thread {

	struct _thread_base base;

	/** defined by the architecture, but all archs need these */
	struct _callee_saved callee_saved;

	/** static thread init data */
	void *init_data;

	/** threads waiting in k_thread_join() */
	_wait_q_t join_queue;

#if defined(CONFIG_POLL)
	struct z_poller poller;
#endif

#if defined(CONFIG_EVENTS)
	struct k_thread *next_event_link;

	uint32_t   events;
	uint32_t   event_options;
#endif

#if defined(CONFIG_THREAD_MONITOR)
	/** thread entry and parameters description */
	struct __thread_entry entry;

	/** next item in list of all threads */
	struct k_thread *next_thread;
#endif

#if defined(CONFIG_THREAD_NAME)
	/** Thread name */
	char name[CONFIG_THREAD_MAX_NAME_LEN];
#endif

#ifdef CONFIG_THREAD_CUSTOM_DATA
	/** crude thread-local storage */
	void *custom_data;
#endif

#ifdef CONFIG_THREAD_USERSPACE_LOCAL_DATA
	struct _thread_userspace_local_data *userspace_local_data;
#endif

#if defined(CONFIG_ERRNO) && !defined(CONFIG_ERRNO_IN_TLS) && !defined(CONFIG_LIBC_ERRNO)
#ifndef CONFIG_USERSPACE
	/** per-thread errno variable */
	int errno_var;
#endif
#endif

#if defined(CONFIG_THREAD_STACK_INFO)
	/** Stack Info */
	struct _thread_stack_info stack_info;
#endif /* CONFIG_THREAD_STACK_INFO */

#if defined(CONFIG_USERSPACE)
	/** memory domain info of the thread */
	struct _mem_domain_info mem_domain_info;
	/** Base address of thread stack */
	k_thread_stack_t *stack_obj;
	/** current syscall frame pointer */
	void *syscall_frame;
#endif /* CONFIG_USERSPACE */


#if defined(CONFIG_USE_SWITCH)
	/* When using __switch() a few previously arch-specific items
	 * become part of the core OS
	 */

	/** z_swap() return value */
	int swap_retval;

	/** Context handle returned via arch_switch() */
	void *switch_handle;
#endif
	/** resource pool */
	struct k_heap *resource_pool;

#if defined(CONFIG_THREAD_LOCAL_STORAGE)
	/* Pointer to arch-specific TLS area */
	uintptr_t tls;
#endif /* CONFIG_THREAD_LOCAL_STORAGE */

#ifdef CONFIG_DEMAND_PAGING_THREAD_STATS
	/** Paging statistics */
	struct k_mem_paging_stats_t paging_stats;
#endif

	/** arch-specifics: must always be at the end */
	struct _thread_arch arch;
};

typedef struct k_thread _thread_t;
typedef struct k_thread *k_tid_t;

void z_init_cpu(int id);
void z_sched_ipi(void);
void z_smp_start_cpu(int id);

#endif
