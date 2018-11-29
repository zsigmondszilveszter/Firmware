/****************************************************************************
 * libc/wqueue/work_thread.c
 *
 *   Copyright (C) 2009-2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "work_queue.hpp"
#include "dq/dq.hpp"
#include "work_lock.h"

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_posix.h>
#include <px4_time.h>
#include <px4_tasks.h>
#include <drivers/drv_hrt.h>

namespace px4
{

/* The state of each work queue. */
struct wqueue_s h_work[NWORKERS];

px4_sem_t _px4_work_lock[NWORKERS];

/****************************************************************************
 * Name: work_process
 *
 * Description:
 *   This is the logic that performs actions placed on any work list.
 *
 * Input parameters:
 *   wqueue - Describes the work queue to be processed
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void work_process(struct wqueue_s *wqueue, int lock_id)
{
	/* Then process queued work.  We need to keep interrupts disabled while
	 * we process items in the work list.
	 */
	uint32_t next = 10000; // CONFIG_SCHED_WORKPERIOD;

	work_lock(lock_id);

	volatile struct work_s *work = (struct work_s *)wqueue->q.head;

	while (work) {
		/* Is this work ready?  It is ready if there is no delay or if
		 * the delay has elapsed. qtime is the time that the work was added
		 * to the work queue.  It will always be greater than or equal to
		 * zero.  Therefore a delay of zero will always execute immediately.
		 */

		uint64_t elapsed = USEC2TICK(px4_clock_systimer() - work->qtime);

		//printf("work_process: in ticks elapsed=%lu delay=%u\n", elapsed, work->delay);
		if (elapsed >= work->delay) {
			/* Remove the ready-to-execute work from the list */

			deque_rem((struct deque_entry_s *)work, &wqueue->q);

			/* Extract the work description from the entry (in case the work
			 * instance by the re-used after it has been de-queued).
			 */

			worker_t worker = work->worker;
			void *arg = work->arg;

			/* Mark the work as no longer being queued */

			work->worker = NULL;

			/* Do the work.  Re-enable interrupts while the work is being
			 * performed... we don't have any idea how long that will take!
			 */

			work_unlock(lock_id);

			if (!worker) {
				PX4_WARN("MESSED UP: worker = 0\n");

			} else {
				worker(arg);
			}

			/* Now, unfortunately, since we re-enabled interrupts we don't
			 * know the state of the work list and we will have to start
			 * back at the head of the list.
			 */

			work_lock(lock_id);
			work = (struct work_s *)wqueue->q.head;

		} else {
			/* This one is not ready.. will it be ready before the next
			 * scheduled wakeup interval?
			 */

			/* Here: elapsed < work->delay */
			uint32_t remaining = USEC_PER_TICK * (work->delay - elapsed);

			if (remaining < next) {
				/* Yes.. Then schedule to wake up when the work is ready */

				next = remaining;
			}

			/* Then try the next in the list. */

			work = (struct work_s *)work->dq.flink;
		}
	}

	/* Wait awhile to check the work list.  We will wait here until either
	 * the time elapses or until we are awakened by a signal.
	 */
	work_unlock(lock_id);

	usleep(next);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
void work_queues_init()
{
	px4_sem_init(&_px4_work_lock[HPWORK], 0, 1);

	// Create high priority worker thread
	h_work[HPWORK].pid = px4_task_spawn_cmd("px4hpwork",
						SCHED_DEFAULT,
						SCHED_PRIORITY_MAX - 1,
						2000,
						work_hpthread,
						(char *const *)NULL);
}

/****************************************************************************
 * Name: work_hpthread
 *
 * Description:
 *   These are the worker threads that performs actions placed on the work
 *   lists.
 *
 *   work_hpthread:  These are the kernel mode work queues
 *     (also build in the flat build).  One of these threads also performs
 *     periodic garbage collection (that is otherwise performed by the idle
 *     thread if CONFIG_SCHED_WORKQUEUE is not defined).
 *
 *     These worker threads are started by the OS during normal bringup.
 *
 *   work_usrthread:  This is a user mode work queue.  It must be built into
 *     the applicatino blob during the user phase of a kernel build.  The
 *     user work thread will then automatically be started when the system
 *     boots by calling through the pointer found in the header on the user
 *     space blob.
 *
 *   All of these entrypoints are referenced by OS internally and should not
 *   not be accessed by application logic.
 *
 * Input parameters:
 *   argc, argv (not used)
 *
 * Returned Value:
 *   Does not return
 *
 ****************************************************************************/

int work_hpthread(int argc, char *argv[])
{
	/* Loop forever */

	for (;;) {
		/* First, perform garbage collection.  This cleans-up memory de-allocations
		 * that were queued because they could not be freed in that execution
		 * context (for example, if the memory was freed from an interrupt handler).
		 * NOTE: If the work thread is disabled, this clean-up is performed by
		 * the IDLE thread (at a very, very low priority).
		 */

#ifndef CONFIG_SCHED_LPWORK
		sched_garbagecollection();
#endif

		/* Then process queued work.  We need to keep interrupts disabled while
		 * we process items in the work list.
		 */
		work_process(&h_work[HPWORK], HPWORK);
	}

	return PX4_OK; /* To keep some compilers happy */
}

uint32_t px4_clock_systimer()
{
	//printf("clock_systimer: %0lx\n", hrt_absolute_time());
	return (0x00000000ffffffff & hrt_absolute_time());
}

} // namespace px4
