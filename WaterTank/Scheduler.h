#ifndef SCHEDULER_H_
#define SCHEDULER_H_

#include <stdint.h>
#include <avr/sleep.h>

const uint8_t MAX_TIMERS_COUNT = 8; /**< Maximum number of timers allowed */
const uint8_t MAX_TASK_QUEUE_SIZE = 8; /**< Maximum size of the task queue */

typedef void(*TaskPointer)();

class Scheduler {
    private:
        typedef struct {
            uint16_t counter;
            uint16_t period;
            TaskPointer task;
        } Timer;

        static Timer timers[MAX_TIMERS_COUNT];
        static uint8_t timersCount;
        static TaskPointer taskQueue[MAX_TASK_QUEUE_SIZE];
        static uint8_t taskQueueSize;

    public:
        static bool processTask();
        static void run(bool idle_sleep = false, uint8_t sleep_mode = SLEEP_MODE_IDLE);
        static bool setTimer(TaskPointer task, uint16_t period_ticks, bool periodic = false);
        static bool resetTimer(TaskPointer task);
        static bool setTask(TaskPointer task);

        /** 
         * @brief Timer interrupt service routine (ISR) to handle timer countdown.
         * 
         * This function should be called in the interrupt context of a hardware timer.
         * The duration between calls determines the length of one tick used by the 
         * setTimer(...) function with period_ticks.
         */
        static inline void TimerISR() {
            for(uint8_t i = 0; i < timersCount; i++) {
                timers[i].counter--;
                if(timers[i].counter != 0) {
                    continue;
                }

                setTask(timers[i].task);
                
                if(timers[i].period != 0) { // reset timer
                    timers[i].counter = timers[i].period;
                } else {                    // delete timer
                    timersCount--;
                    timers[i] = timers[timersCount];
                    i--;
                }
            }
        }
};

#endif /* SCHEDULER_H_ */
