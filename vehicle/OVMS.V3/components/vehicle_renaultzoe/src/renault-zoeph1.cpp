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

static const OvmsVehicle::poll_pid_t renault_zoe_polls[] = {
  //{ 0x7e4, 0x7ec, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x2002, { 0, 10, 10, 10 } },  // SOC
  //{ 0x7e4, 0x7ec, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x2006, { 0, 10, 10, 10 } },  // Odometer
  //{ 0x7e4, 0x7ec, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x3203, { 0, 10, 10, 10 } },  // Battery Voltage
  { 0x7e4, 0x7ec, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x3204, { 0, 30, 1, 2 }, 0, ISOTP_STD },  // Battery Current
  //{ 0x7e4, 0x7ec, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x3028, { 0, 10, 10, 10 } },  // 12Battery Current
  //7ec,24,39,.005,0,0,kwh,22320C,62320C,ff,Available discharge Energy
  //{ 0x7e4, 0x7ec, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x320C, { 0, 10, 10, 10 } },  // Available discharge Energy
  //7ec,30,31,1,0,0,,223332,623332,ff,Electrical Machine functionning Authorization,0:Not used;1:Inverter Off;2:Inverter On\n" //
  //{ 0x7e4, 0x7ec, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x3332, { 0, 10, 10, 10 } },  //  Inverter Off: 1; Inverter On: 2
  //{ 0x7e4, 0x7ec, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x2005, { 0, 10, 10, 10 } },  // 12Battery Voltage
  //{ 0x7e4, 0x7ec, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x3206, { 0, 10, 10, 10 } },  // Battery SOH
  //{ 0x7e4, 0x7ec, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x33dc, { 0, 10, 10, 10 } },  // Consumed Domnestic Engergy
  //{ 0x75a, 0x77e, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x3018, { 0, 10, 10, 10 } },  // DCDC Temp
  //{ 0x792, 0x793, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x504A, { 0, 10, 10, 10 } },  // Mains active Power consumed
  //{ 0x792, 0x793, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x5063, { 0, 10, 10, 10 } },  // Charging State
  //{ 0x792, 0x793, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x5062, { 0, 10, 10, 10 } },  // Ground Resistance
  { 0x79b, 0x7bb, VEHICLE_POLL_TYPE_OBDIIGROUP, 0x04, { 0, 60, 600, 60 }, 0, ISOTP_STD },  // Temp Bat Module 1
  { 0x79b, 0x7bb, VEHICLE_POLL_TYPE_OBDIIGROUP, 0x41, { 0, 60, 600, 60 }, 0, ISOTP_STD },  // Cell Bat Module 1-62
  { 0x79b, 0x7bb, VEHICLE_POLL_TYPE_OBDIIGROUP, 0x42, { 0, 60, 600, 60 }, 0, ISOTP_STD },  // Cell Bat Module 63-96
  //+"7bc,28,39,1,4094,0,N·m,224B7C,624B7C,ff,Electric brake wheels torque request\n" //
  //+ "Uncoupled Braking Pedal,2197,V,7bc,79c,UBP,-,5902ff\n"
  //{ 0x79c, 0x7bc, VEHICLE_POLL_TYPE_OBDIIGROUP, 0x04, { 0, 120, 1, 120 } }, // Braking Pedal
  //{ 0x742, 0x762, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x012D, { 0, 0, 10, 10 } }, // Motor temperature (nope)
  // -END-
  POLL_LIST_END
};

void OvmsVehicleRenaultZoe::ZoePH1Init() {
  ESP_LOGI(TAG, "Starting: ZoePH1");
  
  // Init polling:
  PollSetThrottling(0);
  PollSetResponseSeparationTime(1);
  PollSetPidList(m_can1, renault_zoe_polls);
  POLLSTATE_OFF;
  
  // BMS configuration:
  BmsSetCellArrangementVoltage(96, 8);
  BmsSetCellArrangementTemperature(12, 1);
  
  BmsSetCellLimitsVoltage(2.0, 5.0);
  BmsSetCellLimitsTemperature(-39, 200);
  BmsSetCellDefaultThresholdsVoltage(0.030, 0.050);
  BmsSetCellDefaultThresholdsTemperature(4.0, 5.0);
}