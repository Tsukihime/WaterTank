#include "Scheduler.h"

#include <util/atomic.h>

Scheduler::Timer Scheduler::timers[MAX_TIMERS_COUNT];
uint8_t Scheduler::timersCount = 0;
TaskPointer Scheduler::taskQueue[MAX_TASK_QUEUE_SIZE];
uint8_t Scheduler::taskQueueSize = 0;

/**
 * @brief Processes a task from the task queue.
 *
 * This function removes one task from the task queue, executes it,
 * and returns true. If the task queue is empty, it returns false.
 *
 * @return true if a task was processed, false if the task queue was empty.
 */
bool Scheduler::processTask() {
    TaskPointer currentTask;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if(taskQueueSize == 0) {
            return false; // Idle();
        }

        currentTask = taskQueue[0];
        taskQueueSize--;
        for(uint8_t i = 0; i < taskQueueSize; i++) {
            taskQueue[i] = taskQueue[i + 1];
        }
    }
    currentTask();
    return true;
}

/**
 * @brief Runs an infinite loop to process tasks.
 *
 * This function enters an infinite loop that continuously processes
 * tasks in the task queue.
 */

/**
 * @brief Enters an infinite loop to process tasks.
 *
 * This function enters an infinite loop that continuously processes
 * tasks in the task queue. If `idle_sleep` is enabled, the function
 * puts the device into sleep mode specified by `sleep_mode` when the
 * task queue is emptied.
 *
 * @param idle_sleep A boolean indicating whether to enter sleep mode
 *                   when idle.
 * @param sleep_mode The mode to set for sleeping when the task queue
 *                   is empty.
 */
void Scheduler::run(bool idle_sleep, uint8_t sleep_mode) {
    while(true) {
        while(processTask());
       	if(idle_sleep) {
			set_sleep_mode(sleep_mode);
			sleep_mode();
		}
    }
}

/**
 * @brief Creates a timer with the specified period.
 *
 * This function creates a timer and returns true on success, or
 * false if the maximum number of timers has been reached.
 *
 * @param task Pointer to the task to be executed when the timer expires.
 * @param period_ticks The period of the timer in ticks.
 * @param periodic Indicates whether the timer is periodic.
 * @return true if the timer was successfully created, false if it failed.
 */
bool Scheduler::setTimer(TaskPointer task, uint16_t period_ticks, bool periodic) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (timersCount == MAX_TIMERS_COUNT) {
            return false;
        }

        timers[timersCount].task = task;
        timers[timersCount].counter = period_ticks;
        timers[timersCount].period = periodic ? period_ticks : 0;
        timersCount++;
    }
    return true;
}

/**
 * @brief Resets the timer for a specified task.
 *
 * This function resets the timer counter for the given task and returns 
 * true on success, or false if the task was not found.
 *
 * @param task Pointer to the task for which the timer should be reset.
 * @return true if the timer was successfully reset, false if it failed.
 */
bool Scheduler::resetTimer(TaskPointer task) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        for(uint8_t i = 0; i < timersCount; i++) {
            if(timers[i].task != task) {
                continue;
            }
            timers[i].counter = timers[i].period;
            return true;
        }
    }
    return false;
}

/**
 * @brief Adds a task to the task queue.
 *
 * This function adds a new task to the task queue and returns true on 
 * success, or false if the queue is full.
 *
 * @param task Pointer to the task to be added.
 * @return true if the task was successfully added, false if it failed.
 */
bool Scheduler::setTask(TaskPointer task) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if(taskQueueSize == MAX_TASK_QUEUE_SIZE) {
            return false;
        }

        taskQueue[taskQueueSize] = task;
        taskQueueSize++;
    }
    return true;
}
