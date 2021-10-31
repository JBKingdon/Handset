#pragma once

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
// #include "freertos/task.h"

#define TimerIntervalUSDefault 20000000

class HwTimer
{
private:
	static uint64_t interval;
	static bool isTick;
	static int32_t phaseShift;
	static int32_t freqOffset;
	static bool running;
	static void (*callbackTick)();
	static void (*callbackTock)();
	static void timer_task(void* arg);
	static xQueueHandle s_timer_queue;

public:
	static void init();
	static void stop();
	static void resume();
	static void resetFreqOffset();
	static void incFreqOffset();
	static void decFreqOffset();
	static int32_t getFreqOffset() { return freqOffset; };
	static void setInterval(uint64_t newTimerInterval);
	static void setPhaseShift(int32_t newPhaseShift);
	static void setCallbackTick(void (*fn)());
	static void setCallbackTock(void (*fn)());

	static void inline nullCallback(void);
};