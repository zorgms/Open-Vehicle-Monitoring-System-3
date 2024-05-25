/*
 ;    Project:       Open Vehicle Monitor System
 ;    Date:          1th October 2018
 ;
 ;    Changes:
 ;    1.0  Initial release
 ;
 ;    (C) 2018       Martin Graml
 ;    (C) 2019       Thomas Heuer
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
 ;
 ; Most of the CAN Messages are based on https://github.com/MyLab-odyssey/ED_BMSdiag
 ; http://ed.no-limit.de/wiki/index.php/Hauptseite
 */

#include "ovms_log.h"
// static const char *TAG = "v-smarted";

#include <stdio.h>
#include <string>
#include <iomanip>
#include "pcp.h"
#include "ovms_events.h"
#include "metrics_standard.h"
#include "ovms_notify.h"
#include "ovms_peripherals.h"
#include "string_writer.h"

#include "vehicle_smarted.h"

using namespace std;

typedef struct
{
  uint16_t txid;
  uint16_t rxid;
  string read;
  string clear;
  string name;
  uint8_t session;
} smarted_dtc_t;

static const smarted_dtc_t DTC_data[] =
{
  { 0x7A0, 0x7A1, "1802ff00", "14ff00", "SAM_451", 1 },  // SAM_451
  { 0x7A2, 0x7A3, "1802ff00", "14ff00", "AC_451", 1 },   // AC_451
  { 0x796, 0x797, "1802ff00", "14ff00", "EPS_451", 1 },  // EPS_451
  { 0x784, 0x785, "1802ff00", "14ff00", "ESP8_451", 1 }, // ESP8_451
  { 0x788, 0x789, "1802ff00", "14ff00", "EWM_451", 1 },  // EWM_451
  { 0x782, 0x783, "1802ff00", "14ff00", "KI_451", 1 },   // KI_451
  { 0x7E5, 0x7ED, "19022f", "14ffffff", "CEPC_SEV3", 0 },  // CEPC_SEV3
  { 0x7E7, 0x7EF, "19022f", "14ffffff", "BMS", 0 },        // BMS
  { 0x7E6, 0x7EE, "19022f", "14ffffff", "LE_TUBE", 0 },    // LE_TUBE
  { 0x62D, 0x62F, "19022f", "14ffffff", "CSHV451EV", 0 },  // CSHV451EV
  { 0x61A, 0x483, "19022f", "14ffffff", "OBL_451", 0 },    // OBL_451 / NLG6
  { 0x769, 0x601, "19022f", "14ffffff", "PLGW_451EV", 0 }, // PLGW_451EV
};

void OvmsVehicleSmartED::DTCPollInit()
{
  // init commands:
  OvmsCommand* cmd;
  cmd = cmd_xse->RegisterCommand("dtc", "Show DTC report / clear DTC", shell_obd_showdtc);
  cmd->RegisterCommand("show", "Show DTC report", shell_obd_showdtc);
  cmd->RegisterCommand("clear", "Clear stored DTC in car", shell_obd_cleardtc,
    "Att: this clears the car DTC store (car must be turned on).");
}

void OvmsVehicleSmartED::shell_obd_showdtc(int verbosity, OvmsWriter* writer, OvmsCommand* cmd, int argc, const char* const* argv)
{
  OvmsVehicleSmartED* smart = GetInstance(writer);
  if (!smart)
    return;

  if (!smart->m_enable_write) {
    writer->puts("ERROR: CAN bus write access disabled.");
    return;
  }
  else if (!StandardMetrics.ms_v_env_on->AsBool()) {
    writer->puts("ERROR: Ignition is off.");
    return;
  }

  uint8_t protocol = ISOTP_STD;
  int timeout_ms = 3000;
  std::string response;
  int c;

  if (smart->mt_nlg6_present->AsBool()) {
    c=11;
  } else {
    c=12;
  }
  
  canbus* bus = (canbus*) MyPcpApp.FindDeviceByName("can1");

  for (int i = 0; i < c; i++) {
    // execute request:
    //int err = smart->ObdRequest(DTC_data[i].txid, DTC_data[i].rxid, hexdecode(DTC_data[i].read), response);
    int err = smart->PollSingleRequest(bus, DTC_data[i].txid, DTC_data[i].rxid, hexdecode(DTC_data[i].read), response, timeout_ms, protocol);
    if (err == POLLSINGLE_TXFAILURE) {
      writer->puts("ERROR: transmission failure (CAN bus error)");
      return;
    } else if (err < 0) {
      writer->puts("ERROR: timeout waiting for poller/response");
      return;
    } else if (err) {
      const char* errname = smart->PollResultCodeName(err);
      writer->printf("ERROR: request failed with response error code %02X%s%s\n", err,
        errname ? " " : "", errname ? errname : "");
      return;
    }
    if (!response[1]) {
      writer->printf("%s: 0 present\n", DTC_data[i].name.c_str());
    } else {
      if (DTC_data[i].session == 1) {
        writer->printf("%s: %d present\n", DTC_data[i].name.c_str(), response[0]);
      } else {
        int cnt = (response.size() - 1) / 4;
        writer->printf("%s: %d present\n", DTC_data[i].name.c_str(), cnt);
      }
      smart->DTCdecode(writer, response, DTC_data[i].session);
    }
    writer->puts("");
  }
}

void OvmsVehicleSmartED::shell_obd_cleardtc(int verbosity, OvmsWriter* writer, OvmsCommand* cmd, int argc, const char* const* argv)
{
  OvmsVehicleSmartED* smart = GetInstance(writer);
  if (!smart)
    return;

  if (!smart->m_enable_write) {
    writer->puts("ERROR: CAN bus write access disabled.");
    return;
  }
  else if (!StandardMetrics.ms_v_env_on->AsBool()) {
    writer->puts("ERROR: Ignition is off.");
    return;
  }
  CAN_frame_t txframe = {};
  txframe.FIR.B.FF = CAN_frame_std;
  txframe.FIR.B.DLC = 8;

  // StartDiagnosticSession.ExtendedDiagnostic:
  //  - Request 1092
  //  - Response 5092 (unchecked)
  txframe.data.u8[0] = 0x02;
  txframe.data.u8[1] = 0x10;
  txframe.data.u8[2] = 0x92;

  uint8_t protocol = ISOTP_STD;
  int timeout_ms = 3000;
  std::string response;
  int c;
  
  if (smart->mt_nlg6_present->AsBool()) {
    c = 11;
  } else {
    c = 12;
  }

  canbus* bus = (canbus*) MyPcpApp.FindDeviceByName("can1");

  for (int i = 0; i < c; i++) {
    if (DTC_data[i].session == 1) {
      txframe.MsgID = DTC_data[i].txid;
      txframe.Write(smart->m_can1);
      vTaskDelay(50 / portTICK_PERIOD_MS);
    }
    // execute request:
    //int err = smart->ObdRequest(DTC_data[i].txid, DTC_data[i].rxid, hexdecode(DTC_data[i].read), response);
    int err = smart->PollSingleRequest(bus, DTC_data[i].txid, DTC_data[i].rxid, hexdecode(DTC_data[i].clear), response, timeout_ms, protocol);
    if (err == POLLSINGLE_TXFAILURE) {
      writer->puts("ERROR: transmission failure (CAN bus error)");
      return;
    } else if (err < 0) {
      writer->puts("ERROR: timeout waiting for poller/response");
      return;
    } else if (err) {
      const char* errname = smart->PollResultCodeName(err);
      writer->printf("ERROR: request failed with response error code %02X%s%s\n", err,
        errname ? " " : "", errname ? errname : "");
      return;
    }
    writer->printf("%s: DTC store cleared\n", DTC_data[i].name.c_str());
/*    writer->puts("Response:");
    char *buf = NULL;
    size_t rlen = response.size(), offset = 0;
    do {
      rlen = FormatHexDump(&buf, response.data() + offset, rlen, 16);
      offset += 16;
      writer->puts(buf ? buf : "-");
    } while (rlen);
    if (buf)
      free(buf);
    writer->puts("");
*/
  }

  writer->puts("DTC store has been cleared.");

  smart->ResumePolling();
}

void OvmsVehicleSmartED::DTCdecode(OvmsWriter* writer, std::string response, int typ)
{
  int x;
  if (typ == 1) {
    x = 3;
  } else {
    x = 4;
  }
  int cnt = (response.size() - 1) / x;
  for (int i = 0; i < cnt; i++) {
    switch ((response[i*x+1] & 0b11000000) >> 6) {
      case 0b00:
        writer->printf("P"); // 00 = P = Powertrain
        break;
      case 0b01:
        writer->printf("C"); // 01 = C = Chassis
        break;
      case 0b10:
        writer->printf("B"); // 10 = B = Body
        break;
      case 0b11:
        writer->printf("U"); // 11 = U = serial data
        break;
    }
    switch ((response[i*x+1] & 0b00110000) >> 4) {
      case 0b0000:
        writer->printf("0");
        break;
      case 0b0001:
        writer->printf("1");
        break;
      case 0b0010:
        writer->printf("2");
        break;
      case 0b0011:
        writer->printf("3");
        break;
    }
    if (x==3) {
      writer->printf("%01x%02x", response[i*x+1] & 0x0f, response[i*x+2]);
      
      if ((response[i*x+3] & 0x20) > 0) {
        writer->printf(" saved");
      }
      if ((response[i*x+3] & 0x40) > 0) {
        writer->printf(" and present");
      }
      writer->puts("");
    }
    if (x==4) {
      writer->printf("%01x%02x%02x", response[i*x+1] & 0x0f, response[i*x+2], response[i*x+3]);
      
      if ((response[i*x+4] & 0x20) > 0) {
        writer->printf(" saved");
      }
      if ((response[i*x+4] & 0x40) > 0) {
        writer->printf(" and present");
      }
      writer->puts("");
    }
  }
}