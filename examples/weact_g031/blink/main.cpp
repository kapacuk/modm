/*
 * Copyright (c) 2016, Sascha Schade
 * Copyright (c) 2017, Niklas Hauser
 * Copyright (c) 2019, Raphael Lehmann
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#include <modm/board.hpp>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

using namespace Board;

TaskHandle_t h_mainTask;

/*
 * Blinks the user LED with 1 Hz
 * or 5 Hz is the user button is pressed.
 */

void
taskMain(void *)
{
    Led::set();

    MODM_LOG_DEBUG   << "debug"   << modm::endl;
    MODM_LOG_INFO    << "info"    << modm::endl;
    MODM_LOG_WARNING << "warning" << modm::endl;
    MODM_LOG_ERROR   << "error"   << modm::endl;

    while (true)
    {
        Led::toggle();
        modm::delay(200ms);
    }
}

constexpr int stack_size = 200;
StackType_t stack[stack_size];
StaticTask_t taskBuffer;
int
main()
{
    Board::initialize();
    h_mainTask = xTaskCreateStatic(taskMain, "Main",  stack_size, NULL, 2, stack, &taskBuffer);
    configASSERT( h_mainTask != NULL );
    vTaskStartScheduler();
}
