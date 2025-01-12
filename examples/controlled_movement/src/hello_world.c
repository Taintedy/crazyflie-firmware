/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * hello_world.c - App layer application of a simple hello world debug print every
 *   2 seconds.
 */


#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"
#include "param.h"

#define DEBUG_MODULE "CONTROL_MOVE"
#include "debug.h"
#include "estimator.h"
#include "commander.h"
#include "crtp_commander_high_level.h"



static state_t state;
stabilizerStep_t stabilizerStep = 1;
setpoint_t setpoint;
int16_t startTask;

void appMain() {
  DEBUG_PRINT("Waiting for activation (set the start_task parameter)...\n");


  while(1) {
    vTaskDelay(M2T(2000));
    stateEstimator(&state, stabilizerStep);
    DEBUG_PRINT("pose: %f; %f; %f\n", (double)state.position.x, (double)state.position.y, (double)state.position.z);
    DEBUG_PRINT("setpoint: %f; %f; %f\n", (double)setpoint.position.x, (double)setpoint.position.y, (double)setpoint.position.z);

  }

}


void taskCallback(void)
{
    setpoint.mode.z = modeAbs;
    crtpCommanderHighLevelTakeoff(0.5f, 2.5f);
    vTaskDelay(M2T(5000));
    crtpCommanderHighLevelGoTo(0.0f, 0.0f, 0.5f, 0.0f, 1.0f, true);
    vTaskDelay(M2T(5000));
    crtpCommanderHighLevelGoTo(-0.5f, 0.0f, 0.0f, 0.0f, 1.0f, true);
    vTaskDelay(M2T(5000));
    crtpCommanderHighLevelGoTo(0.0f, -0.5f, 0.0f, 0.0f, 1.0f, true);
    vTaskDelay(M2T(5000));
    crtpCommanderHighLevelGoTo(0.5f, 0.0f, 0.0f, 0.0f, 1.0f, true);
    vTaskDelay(M2T(5000));
    crtpCommanderHighLevelGoTo(0.0f, 0.5f, 0.0f, 0.0f, 1.0f, true);
    vTaskDelay(M2T(5000));
    crtpCommanderHighLevelLand(0.0f, 5.0f);
}



PARAM_GROUP_START(task)
PARAM_ADD_WITH_CALLBACK(PARAM_UINT8, start_task, &startTask, &taskCallback)
PARAM_GROUP_STOP(task)
