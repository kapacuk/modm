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

/*
 * A single FreeRTOS task which reads symbols from Usart1
 * and sends them back, toggling the LED for every symbol
 */

using namespace Board;

using Uart = Usart1;
TaskHandle_t h_mainTask;

void
taskMain(void *)
{
    Uart::connect<GpioOutputB6::Tx, GpioInputB7::Rx>();
	Uart::initialize<SystemClock, 115200_Bd>();
    Led::set();

    uint8_t chr;
    while (true)
    {
        Uart::read( chr, portMAX_DELAY );
        Uart::write( chr );
        Led::toggle();
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
	return 0;
}
