/*
;    Project:       Open Vehicle Monitor System
;    Date:          11th Sep 2019
;
;    Changes:
;    1.0  Initial release
;
;    (C) 2011       Michael Stegen / Stegen Electronics
;    (C) 2011-2017  Mark Webb-Johnson
;    (C) 2011       Sonny Chen @ EPRO/DX
;    (C) 2018       Marcos Mezo
;    (C) 2019       Thomas Heuer @Dimitrie78
;
; Permission is hereby granted, free of charge, to any person obtaining a copy
; of this software and associated documentation files (the "Software"), to deal
; in the Software without restriction, including without limitation the rights
; to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
; copies of the Software, and to permit persons to whom the Software is
; furnished to do so, subject to the following conditions:
;
; The above copyright notice and this permission notice shall be included in
; all copies or substantial portions of the Software.
;
; THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
; IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
; FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
; AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
; LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
; OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
; THE SOFTWARE.
*/

#include "ovms_log.h"
static const char *TAG = "v-zoe";

#include <stdio.h>
#include <string>
#include <iomanip>
#include "pcp.h"
#include "ovms_metrics.h"
#include "ovms_events.h"
#include "ovms_config.h"
#include "ovms_command.h"
#include "metrics_standard.h"
#include "ovms_notify.h"
#include "ovms_peripherals.h"
#include "ovms_netmanager.h"

#include "vehicle_renaultzoe.h"

static const OvmsVehicle::poll_pid_t renault_kangoo_polls[] = {
  // { 0x7e4, 0x7ec, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x2006, { 0, 60, 0, 0 }, 0, ISOTP_STD },  // Odometer
  { 0x7e4, 0x7ec, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x3203, { 0, 10, 1, 2 }, 0, ISOTP_STD },  // Battery Voltage
  { 0x7e4, 0x7ec, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x3204, { 0, 10, 1, 2 }, 0, ISOTP_STD },  // Battery Current
  { 0x7e4, 0x7ec, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x33F6, { 0, 60, 600, 60 }, 0, ISOTP_STD }, // Inverter Temp
  { 0x79b, 0x7bb, VEHICLE_POLL_TYPE_OBDIIGROUP, 0x04, { 0, 60, 600, 60 }, 0, ISOTP_STD },  // Temp Bat Module 1
  { 0x79b, 0x7bb, VEHICLE_POLL_TYPE_OBDIIGROUP, 0x41, { 0, 60, 600, 60 }, 0, ISOTP_STD },  // Cell Bat Module 1-62
  { 0x79b, 0x7bb, VEHICLE_POLL_TYPE_OBDIIGROUP, 0x42, { 0, 60, 600, 60 }, 0, ISOTP_STD },  // Cell Bat Module 63-96
  // { 0x792, 0x793, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x504A, { 0, 0, 0, 10 }, 0, ISOTP_STD },  // Mains active Power consumed
  POLL_LIST_END
};

void OvmsVehicleRenaultZoe::KangooInit() {
  ESP_LOGI(TAG, "Starting: Kangoo");
  
  // Init polling:
  PollSetThrottling(0);
  PollSetResponseSeparationTime(1);
  PollSetPidList(m_can1, renault_kangoo_polls);
  POLLSTATE_OFF;
  
  // BMS configuration:
  BmsSetCellArrangementVoltage(96, 8);
  BmsSetCellArrangementTemperature(4, 1);
  
  BmsSetCellLimitsVoltage(2.0, 5.0);
  BmsSetCellLimitsTemperature(-39, 200);
  BmsSetCellDefaultThresholdsVoltage(0.030, 0.050);
  BmsSetCellDefaultThresholdsTemperature(4.0, 5.0);
}