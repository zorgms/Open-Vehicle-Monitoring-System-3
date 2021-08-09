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

// obd can1 req dev 18dadbf1 18daf1db 229002 -E
static const OvmsVehicle::poll_pid_t renault_zoe_ph2_polls[] = {
  // BCB
  { 0x18dadef1, 0x18daf1de, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x5017, { 5, 1, 1, 1 }, 0, ISOTP_EXTFRAME },  // 18DAF1DE,29,31,1,0,0,,225017,625017,ff,($5017) Mains current type,0:Nok;1:AC mono;2:AC tri;3:DC;4:AC bi
  // EVC
  { 0x18dadaf1, 0x18daf1da, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x2006, { 0, 10, 10, 10 }, 0, ISOTP_EXTFRAME },  // 18DAF1DA,24,47,1,0,0,km,222006,622006,ff,Total vehicle distance
  { 0x18dadaf1, 0x18daf1da, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x2003, { 0, 1, 1, 0 }, 0, ISOTP_EXTFRAME },  // 18DAF1DA,24,39,.01,0,2,km/h,222003,622003,ff,Vehicle speed
  { 0x18dadaf1, 0x18daf1da, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x2005, { 0, 1, 1, 1 }, 0, ISOTP_EXTFRAME },  // 18DAF1DA,24,39,.01,0,2,V,222005,622005,ff,Battery voltage
//  { 0x18dadaf1, 0x18daf1da, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x29FD, { 0, 1, 1, 1 }, 0, ISOTP_EXTFRAME },  // 18DAF1DA,24,39,.01,32768,2,A,2229FD,6229FD,ff,Battery current
  { 0x18dadaf1, 0x18daf1da, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x20DE, { 0, 10, 10, 10 }, 0, ISOTP_EXTFRAME },  // 18DAF1DA,24,39,.1,2730,1,,2220DE,6220DE,ff,Ambient temperature
  // LBC
  { 0x18dadbf1, 0x18daf1db, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x9001, { 0, 10, 10, 10 }, 0, ISOTP_EXTFRAME },  // 18DAF1DB,24,39,.01,300,2,%,229001,629001,ff,Battery SOC(Zxx_bsoc_avg)rSOC
  { 0x18dadbf1, 0x18daf1db, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x9002, { 0, 10, 10, 10 }, 0, ISOTP_EXTFRAME },  // 18DAF1DB,24,39,.01,0,2,%,229002,629002,ff,Battery USOC(Wxx_usoc_avg)
  { 0x18dadbf1, 0x18daf1db, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x9003, { 0, 10, 10, 10 }, 0, ISOTP_EXTFRAME },  // 18DAF1DB,24,39,.01,0,2,%,229003,629003,ff,Battery SOH(Zxx_sohe_avg)
  { 0x18dadbf1, 0x18daf1db, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x9006, { 0, 1, 1, 1 }, 0, ISOTP_EXTFRAME },  // 18DAF1DB,24,55,.0009765625,0,3,V,229006,629006,1ff,Sum of all Cell Voltage(Wxx_pack_v)
  { 0x18dadbf1, 0x18daf1db, VEHICLE_POLL_TYPE_OBDIIEXTENDED, 0x9257, { 0, 1, 1, 1 }, 0, ISOTP_EXTFRAME },  // 18DAF1DB,24,39,.03125,32640,2,A,229257,629257,ff,10ms current value (offset correction), internal value (Wxx_bat_i_10ms)
  POLL_LIST_END
};

void OvmsVehicleRenaultZoe::ZoePH2Init() {
  ESP_LOGI(TAG, "Starting: ZoePH2");
  
  // Init polling:
  PollSetThrottling(0);
  PollSetResponseSeparationTime(1);
  PollSetPidList(m_can1, renault_zoe_ph2_polls);
  POLLSTATE_OFF;
  
  // BMS configuration:
  BmsSetCellArrangementVoltage(256, 16);
  BmsSetCellArrangementTemperature(17, 1);
  
  BmsSetCellLimitsVoltage(2.0, 5.0);
  BmsSetCellLimitsTemperature(-39, 200);
  BmsSetCellDefaultThresholdsVoltage(0.030, 0.050);
  BmsSetCellDefaultThresholdsTemperature(4.0, 5.0);
}