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
#define MSG_MY_VERSION "Knutwurst-1.0.7"
#define MAX_PRINTABLE_FILENAME_LEN 96

#define ANYCUBIC_TFT_STATE_IDLE 0
#define ANYCUBIC_TFT_STATE_SDPRINT 1
#define ANYCUBIC_TFT_STATE_SDPAUSE 2
#define ANYCUBIC_TFT_STATE_SDPAUSE_REQ 3
#define ANYCUBIC_TFT_STATE_SDPAUSE_OOF 4
#define ANYCUBIC_TFT_STATE_SDSTOP_REQ 5
#define ANYCUBIC_TFT_STATE_SDOUTAGE 99


#define SM_DIR_UP_L           "DIR_UP.gcode"
#define SM_DIR_UP_S           "DIR_UP~1.GCO"
#define SM_SPECIAL_MENU_L     "[Special Menu].gcode"
#define SM_SPECIAL_MENU_S     "_SPECI~1.GCO"
#define SM_PID_HOTEND_L       "[PID Tune Hotend].gcode"
#define SM_PID_HOTEND_S       "_PIDTU~1.GCO"
#define SM_PID_BED_L          "[PID Tune Ultrabase].gcode"
#define SM_PID_BED_S          "_PIDTU~2.GCO"
#define SM_SAVE_EEPROM_L      "[Save EEPROM].gcode"
#define SM_SAVE_EEPROM_S      "_SAVEE~1.GCO"
#define SM_LOAD_DEFAULTS_L    "[Load FW Defaults].gcode"
#define SM_LOAD_DEFAULTS_S    "_LOADF~1.GCO"
#define SM_PREHEAT_BED_L      "[Preheat Ultrabase].gcode"
#define SM_PREHEAT_BED_S      "_PREHE~1.GCO"
#define SM_MESH_START_L       "[Start Mesh Leveling].gcode"
#define SM_MESH_START_S       "_START~1.GCO"
#define SM_MESH_NEXT_L        "[Next Mesh Point].gcode"
#define SM_MESH_NEXT_S        "_NEXTM~1.GCO"
#define SM_Z_UP_01_L          "[Z Up 0.1].gcode"
#define SM_Z_UP_01_S          "_ZUP01~1.GCO"
#define SM_Z_DN_01_L          "[Z Down 0.1].gcode"
#define SM_Z_DN_01_S          "_ZDOWN~1.GCO"
#define SM_Z_UP_002_L         "[Z Up 0.02].gcode"
#define SM_Z_UP_002_S         "_ZUP00~1.GCO"
#define SM_Z_DN_002_L         "[Z Down 0.02].gcode"
#define SM_Z_DN_002_S         "_ZDOWN~2.GCO"
#define SM_Z_UP_001_L         "[Z Up 0.01].gcode"
#define SM_Z_UP_001_S         "_ZUP00~2.GCO"
#define SM_Z_DN_001_L         "[Z Down 0.01].gcode"
#define SM_Z_DN_001_S         "_ZDOWN~3.GCO"
#define SM_BLTOUCH_L          "[BLTouch Leveling].gcode"
#define SM_BLTOUCH_S          "_BLTOU~1.GCO"
#define SM_PAUSE_L            "[Fil. Change Pause].gcode"
#define SM_PAUSE_S            "_FILCH~2.GCO"
#define SM_RESUME_L           "[Fil. Change Resume].gcode"
#define SM_RESUME_S           "_FILCH~1.GCO"
#define SM_DIS_FILSENS_L      "[Disable Fil. Sensor].gcode"
#define SM_DIS_FILSENS_S      "_DISAB~1"
#define SM_EN_FILSENS_L       "[Enable Fil. Sensor].gcode"
#define SM_EN_FILSENS_S       "_ENABL~1.GCO"
#define SM_EXIT_L             "[Exit].gcode"
#define SM_EXIT_S             "_EXIT_~1.GCO"

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
  void PrintList();
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

  char currentTouchscreenSelection[64];
  char currentFileOrDirectory[64];
  uint16_t MyFileNrCnt = 0;
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
