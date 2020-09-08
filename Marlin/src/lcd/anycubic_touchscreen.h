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
#define MSG_MY_VERSION CUSTOM_BUILD_VERSION
#define MAX_PRINTABLE_FILENAME_LEN 30

#define FILAMENT_RUNOUT_PIN 19

#define ANYCUBIC_TFT_STATE_IDLE 0
#define ANYCUBIC_TFT_STATE_SDPRINT 1
#define ANYCUBIC_TFT_STATE_SDPAUSE 2
#define ANYCUBIC_TFT_STATE_SDPAUSE_REQ 3
#define ANYCUBIC_TFT_STATE_SDPAUSE_OOF 4
#define ANYCUBIC_TFT_STATE_SDSTOP_REQ 5
#define ANYCUBIC_TFT_STATE_SDOUTAGE 99

#if DISABLED(KNUTWURST_DGUS2_TFT)
#define SM_DIR_UP_L           "/.."
#define SM_DIR_UP_S           ".."
#define SM_SPECIAL_MENU_L     "<Special Menu>"
#define SM_SPECIAL_MENU_S     "<SPECM>"
#define SM_PID_HOTEND_L       "<PID Tune Hotend>"
#define SM_PID_HOTEND_S       "<PIDHE>"
#define SM_PID_BED_L          "<PID Tune Ultrabase>"
#define SM_PID_BED_S          "<PIDUB>"
#define SM_SAVE_EEPROM_L      "<Save EEPROM>"
#define SM_SAVE_EEPROM_S      "<SAVEE>"
#define SM_LOAD_DEFAULTS_L    "<Load FW Defaults>"
#define SM_LOAD_DEFAULTS_S    "<LDDEF>"
#define SM_PREHEAT_BED_L      "<Preheat Ultrabase>"
#define SM_PREHEAT_BED_S      "<PREHE>"
#define SM_MESH_MENU_L        "<Mesh Leveling>"
#define SM_MESH_MENU_S        "<MESHL>"
#define SM_MESH_START_L       "<Start Mesh Leveling>"
#define SM_MESH_START_S       "<SMESH>"
#define SM_MESH_NEXT_L        "<Next Mesh Point>"
#define SM_MESH_NEXT_S        "<NEXTM>"
#define SM_Z_UP_01_L          "<Z Up 0.1>"
#define SM_Z_UP_01_S          "<ZUP01>"
#define SM_Z_DN_01_L          "<Z Down 0.1>"
#define SM_Z_DN_01_S          "<ZDN01>"
#define SM_Z_UP_002_L         "<Z Up 0.02>"
#define SM_Z_UP_002_S         "<ZUP002>"
#define SM_Z_DN_002_L         "<Z Down 0.02>"
#define SM_Z_DN_002_S         "<ZDN002>"
#define SM_Z_UP_001_L         "<Z Up 0.01>"
#define SM_Z_UP_001_S         "<ZUP001>"
#define SM_Z_DN_001_L         "<Z Down 0.01>"
#define SM_Z_DN_001_S         "<ZDN001>"
#define SM_BLTOUCH_L          "<BLTouch Leveling>"
#define SM_BLTOUCH_S          "<BLTCH>"
#define SM_PAUSE_L            "<Fil. Change Pause>"
#define SM_PAUSE_S            "<PAUSE>"
#define SM_RESUME_L           "<Fil. Change Resume>"
#define SM_RESUME_S           "<RESUM>"
#define SM_DIS_FILSENS_L      "<Disable Fil. Sensor>"
#define SM_DIS_FILSENS_S      "<DISSEN>"
#define SM_EN_FILSENS_L       "<Enable Fil. Sensor>"
#define SM_EN_FILSENS_S       "<ENSEN>"
#define SM_EXIT_L             "<Exit>"
#define SM_EXIT_S             "<EXIT>"

#define SM_BACK_L             "<End Mesh Leveling>"
#define SM_BACK_S             "<BACK>"

#define SM_FLOWMENU_L         "<Set Flowrate>"
#define SM_FLOWMENU_S         "<SETFLO>"
#define SM_FLOW_DISP_L        "<Flow is XXX>"
#define SM_FLOW_DISP_S        "<FLDISP>"
#define SM_FLOW_UP_L          "<Up>"
#define SM_FLOW_UP_S          "<UP>"
#define SM_FLOW_DN_L          "<Down>"
#define SM_FLOW_DN_S          "<DOWN>"
#define SM_FLOW_EXIT_L        "<End Flow Settings>"
#define SM_FLOW_EXIT_S        "<EXTFLW>"
#endif

#if ENABLED(KNUTWURST_DGUS2_TFT)
#define SM_DIR_UP_L           "<<< BACK <<<        .gcode"
#define SM_DIR_UP_S           "DIR_UP~1.GCO"
#define SM_SPECIAL_MENU_L     "<Special Menu>      .gcode"
#define SM_SPECIAL_MENU_S     "<SPECI~1.GCO"
#define SM_PID_HOTEND_L       "<PID Tune Hotend>   .gcode"
#define SM_PID_HOTEND_S       "<PIDTU~1.GCO"
#define SM_PID_BED_L          "<PID Tune Ultrabase>.gcode"
#define SM_PID_BED_S          "<PIDTU~2.GCO"
#define SM_SAVE_EEPROM_L      "<Save EEPROM>       .gcode"
#define SM_SAVE_EEPROM_S      "<SAVEE~1.GCO"
#define SM_LOAD_DEFAULTS_L    "<Load FW Defaults>  .gcode"
#define SM_LOAD_DEFAULTS_S    "<LOADF~1.GCO"
#define SM_PREHEAT_BED_L      "<Preheat Ultrabase> .gcode"
#define SM_PREHEAT_BED_S      "<PREHE~1.GCO"
#define SM_MESH_MENU_L        "<Mesh Leveling>     .gcode"
#define SM_MESH_MENU_S        "<MESHL~1.GCO"
#define SM_MESH_START_L       "<Start MeshLeveling>.gcode"
#define SM_MESH_START_S       "<START~1.GCO"
#define SM_MESH_NEXT_L        "<Next Mesh Point>   .gcode"
#define SM_MESH_NEXT_S        "<NEXTM~1.GCO"
#define SM_Z_UP_01_L          "<Z Up 0.1>          .gcode"
#define SM_Z_UP_01_S          "<ZUP01~1.GCO"
#define SM_Z_DN_01_L          "<Z Down 0.1>        .gcode"
#define SM_Z_DN_01_S          "<ZDOWN~1.GCO"
#define SM_Z_UP_002_L         "<Z Up 0.02>         .gcode"
#define SM_Z_UP_002_S         "<ZUP00~1.GCO"
#define SM_Z_DN_002_L         "<Z Down 0.02>       .gcode"
#define SM_Z_DN_002_S         "<ZDOWN~2.GCO"
#define SM_Z_UP_001_L         "<Z Up 0.01>         .gcode"
#define SM_Z_UP_001_S         "<ZUP00~2.GCO"
#define SM_Z_DN_001_L         "<Z Down 0.01>       .gcode"
#define SM_Z_DN_001_S         "<ZDOWN~3.GCO"
#define SM_BLTOUCH_L          "<BLTouch Leveling>  .gcode"
#define SM_BLTOUCH_S          "<BLTOU~1.GCO"
#define SM_PAUSE_L            "<Fil. Change Pause> .gcode"
#define SM_PAUSE_S            "<FILCH~2.GCO"
#define SM_RESUME_L           "<Fil. Change Resume>.gcode"
#define SM_RESUME_S           "<FILCH~1.GCO"
#define SM_DIS_FILSENS_L      "<Disable Fil.Sensor>.gcode"
#define SM_DIS_FILSENS_S      "<DISAB~1.GCO"
#define SM_EN_FILSENS_L       "<Enable Fil. Sensor>.gcode"
#define SM_EN_FILSENS_S       "<ENABL~1.GCO"
#define SM_EXIT_L             "<Exit>              .gcode"
#define SM_EXIT_S             "<EXIT_~1.GCO"

#define SM_BACK_L             "<End Mesh Leveling> .gcode"
#define SM_BACK_S             "<BACK_~1.GCO"

#define SM_FLOWMENU_L         "<Set Flowrate>      .gcode"
#define SM_FLOWMENU_S         "<SETFLO1.GCO"
#define SM_FLOW_DISP_L        "<Flow is XXX%>      .gcode"
#define SM_FLOW_DISP_S        "<FLDISPL.GCO"
#define SM_FLOW_UP_L          "<Up>                .gcode"
#define SM_FLOW_UP_S          "<UPFLOW1.GCO"
#define SM_FLOW_DN_L          "<Down>              .gcode"
#define SM_FLOW_DN_S          "<DWNFLOW.GCO"
#define SM_FLOW_EXIT_L        "<End Flow Settings> .gcode"
#define SM_FLOW_EXIT_S        "<EXTFLW1.GCO"
#endif


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
  int currentFlowRate = 0;

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
  uint8_t FilamentSensorEnabled = true;

  uint8_t SpecialMenu = false;
  uint8_t MMLMenu = false;
  uint8_t FlowMenu = false;  

#if ENABLED(ANYCUBIC_FILAMENT_RUNOUT_SENSOR)
  char FilamentTestStatus = false;
  char FilamentTestLastStatus = false;
  bool FilamentSetMillis = true;
#endif
};

extern AnycubicTouchscreenClass AnycubicTouchscreen;

#endif
