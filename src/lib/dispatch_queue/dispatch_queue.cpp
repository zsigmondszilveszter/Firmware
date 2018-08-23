/****************************************************************************
 *
 *   Copyright (C) 2018 PX4 Development Team. All rights reserved.
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
 * 3. Neither the name PX4 nor the names of its contributors may be
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

#include "dispatch_queue.hpp"

#include <px4_defines.h>
#include <drivers/drv_hrt.h>

dispatch_queue::dispatch_queue(const char *name, size_t thread_cnt) :
	_name(name),
	_threads(thread_cnt)
{
	PX4_DEBUG("Creating dispatch queue: %s", name);
	PX4_DEBUG("Dispatch threads: %zu", thread_cnt);

	for (size_t i = 0; i < _threads.size(); i++) {
		_threads[i] = std::thread(std::bind(&dispatch_queue::dispatch_thread_handler, this));
	}
}

dispatch_queue::~dispatch_queue()
{
	PX4_DEBUG("Destructor");

	// signal threads to finish
	_should_exit = true;
	_cv.notify_all();

	// Wait for threads to finish before we exit
	for (size_t i = 0; i < _threads.size(); i++) {
		if (_threads[i].joinable()) {
			PX4_DEBUG("Destructor: Joining thread %zu until completion", i);
			_threads[i].join();
		}
	}
}

void dispatch_queue::dispatch(const fp_t &op)
{
	std::unique_lock<std::mutex> lock(_lock);
	_q.push(op);

	// Manual unlocking is done before notifying, to avoid waking up
	// the waiting thread only to block again (see notify_one for details)
	lock.unlock();
	_cv.notify_all();
}

void dispatch_queue::dispatch(fp_t &&op)
{
	std::unique_lock<std::mutex> lock(_lock);
	_q.push(std::move(op));

	// Manual unlocking is done before notifying, to avoid waking up
	// the waiting thread only to block again (see notify_one for details)
	lock.unlock();
	_cv.notify_all();
}

void dispatch_queue::dispatch_thread_handler()
{
	std::unique_lock<std::mutex> lock(_lock);

	do {
		//Wait until we have data or a quit signal
		_cv.wait(lock, [this] {
			return (_q.size() || _should_exit);
		});

		//after wait, we own the lock
		if (_q.size() && !_should_exit) {
			auto op = std::move(_q.front());
			_q.pop();

			//unlock now that we're done messing with the queue
			lock.unlock();

			op();

			lock.lock();
		}
	} while (!_should_exit);
}
