/*
 AnycubicTouchscreen.h  --- Support for Anycubic i3 Mega TFT
 Created by Christian Hopp on 2017-12-09
 Modified by Oliver Köster on 2020-06-02

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.

 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef anycubic_touchscreen_h
#define anycubic_touchscreen_h

#include <stdio.h>
#include "../inc/MarlinConfig.h"

char *itostr2(const uint8_t &x);

#ifndef ULTRA_LCD
char *itostr3(const int);
char *ftostr32(const float &);
#endif

#define TFTBUFSIZE 4
#define TFT_MAX_CMD_SIZE 96
#define MSG_MY_VERSION "Knutwurst-1.0.5"

#define ANYCUBIC_TFT_STATE_IDLE 0
#define ANYCUBIC_TFT_STATE_SDPRINT 1
#define ANYCUBIC_TFT_STATE_SDPAUSE 2
#define ANYCUBIC_TFT_STATE_SDPAUSE_REQ 3
#define ANYCUBIC_TFT_STATE_SDPAUSE_OOF 4
#define ANYCUBIC_TFT_STATE_SDSTOP_REQ 5
#define ANYCUBIC_TFT_STATE_SDOUTAGE 99

class AnycubicTouchscreenClass
{
public:
  AnycubicTouchscreenClass();
  void Setup();
  void CommandScan();
  void BedHeatingStart();
  void BedHeatingDone();
  void HeatingDone();
  void HeatingStart();
  void FilamentRunout();
  void KillTFT();
  char TFTstate = ANYCUBIC_TFT_STATE_IDLE;

  /**
  * Anycubic TFT pause states:
  *
  * 0 - printing / stopped
  * 1 - regular pause
  * 2 - M600 pause
  * 3 - filament runout pause
  * 4 - nozzle timeout on M600
  * 5 - nozzle timeout on filament runout
  */
  uint8_t ai3m_pause_state = 0;

private:
  char TFTcmdbuffer[TFTBUFSIZE][TFT_MAX_CMD_SIZE];
  int TFTbuflen = 0;
  int TFTbufindr = 0;
  int TFTbufindw = 0;
  char serial3_char;
  int serial3_count = 0;
  char *TFTstrchr_pointer;
  char FlagResumFromOutage = 0;
  uint16_t filenumber = 0;
  unsigned long starttime = 0;
  unsigned long stoptime = 0;
  uint8_t tmp_extruder = 0;
  char LastSDstatus = 0;
  uint16_t HeaterCheckCount = 0;
  bool IsParked = false;

#if defined(POWER_OUTAGE_TEST)
  struct OutageDataStruct
  {
    char OutageDataVersion;
    char OutageFlag;
    float last_position[XYZE];
    float last_bed_temp;
    float last_hotend_temp;
    long lastSDposition;
  } OutageData;
#endif


  float CodeValue();
  bool CodeSeen(char);
  void AnycubicTouchscreen();
  void StartPrint();
  void PausePrint();
  void StopPrint();
  void StateHandler();
  void GetCommandFromTFT();
  void CheckSDCardChange();
  void CheckHeaterError();
  void HandleSpecialMenu();
  void FilamentChangePause();
  void FilamentChangeResume();
  void ReheatNozzle();
  void ParkAfterStop();

  char currentTouchscreenSelection[30];
  uint8_t SpecialMenu = false;
  uint8_t FilamentSensorEnabled = true;

#if ENABLED(ANYCUBIC_FILAMENT_RUNOUT_SENSOR)
  char FilamentTestStatus = false;
  char FilamentTestLastStatus = false;
  bool FilamentSetMillis = true;
#endif
};

extern AnycubicTouchscreenClass AnycubicTouchscreen;

#endif
