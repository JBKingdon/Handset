
#include "HwTimer.h"

#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/timer.h"


// TODO this doesn't quite work, you can't call setInterval before init, and mustn't call init multiple times
// It feels like a non-static impl with a constructor would be better



#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

void inline HwTimer::nullCallback(void) {}

void (*HwTimer::callbackTick)() = &nullCallback;
void (*HwTimer::callbackTock)() = &nullCallback;

uint64_t HwTimer::interval = TimerIntervalUSDefault;
bool HwTimer::isTick = true;
int32_t HwTimer::phaseShift = 0;
int32_t HwTimer::freqOffset = 0;
bool HwTimer::running = false;
QueueHandle_t HwTimer::s_timer_queue = 0;

#define HWTIMER_TICKS_PER_US 5

#define TIMER_DIVIDER         (16)                              //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds


void HwTimer::setCallbackTick(void (*fn)()) 
{
    callbackTick = fn;
}
void HwTimer::setCallbackTock(void (*fn)()) 
{
    callbackTock = fn;
}

extern unsigned long micros();

void HwTimer::timer_task(void* arg)
{
    unsigned long tLast = 0;
    uint32_t dummyData;
    for(;;) {
        // check if this is getting called too often
        // unsigned long now = micros();
        // unsigned long delta = now - tLast;
        // if (delta < 1000) { // fastest expected call rate is 3000/2
        //     printf("timer_task %lu\n", delta);
        // }
        // tLast = now;

        if(xQueueReceive(s_timer_queue, &dummyData, portMAX_DELAY))
        {
            if (running)
            {
                if (HwTimer::isTick)
                {
                    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, (HwTimer::interval >> 1) + HwTimer::freqOffset);
                    HwTimer::callbackTick();
                }
                else
                {
                    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0,(HwTimer::interval >> 1) + HwTimer::phaseShift + HwTimer::freqOffset);
                    HwTimer::phaseShift = 0;
                    HwTimer::callbackTock();
                }
                HwTimer::isTick = !HwTimer::isTick;
            } else {
                printf("in timer_task when not running\n");
            } // if (running)
        } else {
            printf("time_task weird side\n");
        }
    }
}


static bool IRAM_ATTR timer_group_isr_callback(void *args)
{
    BaseType_t high_task_awoken = pdFALSE;

    QueueHandle_t *timerQueue = (QueueHandle_t *) args;

    xQueueSendFromISR(*timerQueue, NULL, &high_task_awoken);

    return high_task_awoken == pdTRUE; // return whether we need to yield at the end of ISR
}


void HwTimer::init()
{
    if (!running)
    {
        s_timer_queue = xQueueCreate(2, 0);
        xTaskCreate(timer_task, "timer_task", 2048, NULL, 15, NULL); // Tpriority 1=min, max is ??? Default main task is pri 1

        // timer_init(interval >> 1); // Run at twice the commanded frequency for tick/tock interleave

        /* Select and initialize basic parameters of the timer */
        timer_config_t config;
        memset(&config, 0, sizeof(timer_config_t));
        config.divider = TIMER_DIVIDER;
        config.counter_dir = TIMER_COUNT_UP;
        config.counter_en = TIMER_PAUSE;
        config.alarm_en = TIMER_ALARM_EN;
        config.auto_reload = TIMER_AUTORELOAD_EN;

        ESP_ERROR_CHECK(timer_init(TIMER_GROUP_0, TIMER_0, &config));

        /* Timer's counter will initially start from value below.
        Also, if auto_reload is set, this value will be automatically reload on alarm */
        ESP_ERROR_CHECK(timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0));

        /* Configure the alarm value and the interrupt on alarm. */
        ESP_ERROR_CHECK(timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, interval / 2));
        ESP_ERROR_CHECK(timer_enable_intr(TIMER_GROUP_0, TIMER_0));

        ESP_ERROR_CHECK(timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, timer_group_isr_callback, &s_timer_queue, 0));

        ESP_ERROR_CHECK(timer_start(TIMER_GROUP_0, TIMER_0));


        isTick = true;
        running = true;
    }
}

void HwTimer::stop()
{
    if (running)
    {
        timer_pause(TIMER_GROUP_0, TIMER_0);
        // timer1_detachInterrupt();
        running = false;
    }
}

void HwTimer::resume()
{
    if (!running)
    {
        // init();
        timer_start(TIMER_GROUP_0, TIMER_0);
        running = true;
    }
}

void HwTimer::setInterval(uint64_t newTimerInterval)
{
    interval = newTimerInterval * HWTIMER_TICKS_PER_US;
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, interval >> 1);
}

void HwTimer::resetFreqOffset()
{
    freqOffset = 0;
}

void HwTimer::incFreqOffset()
{
    freqOffset++;
}

void HwTimer::decFreqOffset()
{
    freqOffset--;
}

void HwTimer::setPhaseShift(int32_t newPhaseShift)
{
    int32_t maxVal = (interval >> 2) / HWTIMER_TICKS_PER_US; // XXX it would be nice if ticks/us was a power of 2
    int32_t minVal = -maxVal;

    phaseShift = constrain(newPhaseShift, minVal, maxVal) * HWTIMER_TICKS_PER_US;
}


