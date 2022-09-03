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

// #include <stdio.h>
// #include "../inc/MarlinConfig.h"
// #include "../module/configuration_store.h"

#include "./src/inc/MarlinConfigPre.h"
#include "./src/feature/bedlevel/bedlevel.h"
#include "src/module/probe.h"


enum axis_t     : uint8_t { X, Y, Z, X2, Y2, Z2, Z3, Z4 };
enum extruder_t : uint8_t { E0, E1, E2, E3, E4, E5, E6, E7 };
void setAxisPosition_mm(const float, const axis_t, const feedRate_t=0);
void initializeGrid();

char *itostr2(const uint8_t &x);

#ifndef ULTRA_LCD
  char *itostr3(const int);
  char *ftostr32(const float &);
#endif

#define TFTBUFSIZE 4
#define TFT_MAX_CMD_SIZE 96
#define MSG_MY_VERSION CUSTOM_BUILD_VERSION
#define MAX_PRINTABLE_FILENAME_LEN 26

#if ENABLED(KNUTWURST_CHIRON)
  #define FILAMENT_RUNOUT_PIN 33
#else
  #define FILAMENT_RUNOUT_PIN 19
#endif

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
  #define SM_BLTOUCH_L          "<Start Auto Leveling>"
  #define SM_BLTOUCH_S          "<BLTCH>"
  #define SM_RESETLV_L          "<Reset Level Grid>"
  #define SM_RESETLV_S          "<RSTLV>"
  #define SM_PAUSE_L            "<Fil. Change Pause>"
  #define SM_PAUSE_S            "<PAUSE>"
  #define SM_RESUME_L           "<Fil. Change Resume>"
  #define SM_RESUME_S           "<RESUM>"
  #define SM_DIS_FILSENS_L      "<Disable Fil. Sensor>"
  #define SM_DIS_FILSENS_S      "<DISSEN>"
  #define SM_EN_FILSENS_L       "<Enable Fil. Sensor>"
  #define SM_EN_FILSENS_S       "<ENSEN>"
  #define SM_EXIT_L             "<Exit>"
  #define SM_EXIT_S             "<SMEXIT>"

  #define SM_BACK_L             "<End Mesh Leveling>"
  #define SM_BACK_S             "<BACK>"

  #define SM_FLOWMENU_L         "<Set Flowrate>"
  #define SM_FLOWMENU_S         "<SETFLO>"
  #define SM_FLOW_DISP_L        "<Flow is XXX>"
  #define SM_FLOW_DISP_S        "<FLDISP>"
  #define SM_FLOW_UP_L          "<Up>"
  #define SM_FLOW_UP_S          "<FLUP>"
  #define SM_FLOW_DN_L          "<Down>"
  #define SM_FLOW_DN_S          "<FLDN>"
  #define SM_FLOW_EXIT_L        "<Exit Flow Settings>"
  #define SM_FLOW_EXIT_S        "<EXTFLW>"

  #define SM_EZLVL_MENU_L       "<Easy 4 Point Level>"
  #define SM_EZLVL_MENU_S       "<EZLVLM>"
  #define SM_EZLVL_P1_L         "<Point A>"
  #define SM_EZLVL_P1_S         "<EZLPA>"
  #define SM_EZLVL_P2_L         "<Point B>"
  #define SM_EZLVL_P2_S         "<EZLPB>"
  #define SM_EZLVL_P3_L         "<Point C>"
  #define SM_EZLVL_P3_S         "<EZLPC>"
  #define SM_EZLVL_P4_L         "<Point D>"
  #define SM_EZLVL_P4_S         "<EZLPD>"
  #define SM_EZLVL_EXIT_L       "<Exit Easy Level>"
  #define SM_EZLVL_EXIT_S       "<EZLEXT>"

  #define SM_BLTZMENU_L         "<Auto Leveling>"
  #define SM_BLTZMENU_S         "<SETOFZ>"
  #define SM_BLTZ_DISP_L        "<Z Offset: XXXXX>"
  #define SM_BLTZ_DISP_S        "<OFZDSP>"
  #define SM_BLTZ_UP_L          "<Up>"
  #define SM_BLTZ_UP_S          "<UPOFFZ0>"
  #define SM_BLTZ_DN_L          "<Down>"
  #define SM_BLTZ_DN_S          "<DNOFFZ0>"
  #define SM_BLTZ_EXIT_L        "<SAVE and EXIT>"
  #define SM_BLTZ_EXIT_S        "<EXTABLM>"
#endif // !KNUTWURST_DGUS2_TFT

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
  #define SM_BLTOUCH_L          "<Start AutoLeveling>.gcode"
  #define SM_BLTOUCH_S          "<BLTOU~1.GCO"
  #define SM_RESETLV_L          "<Reset Level Grid>  .gcode"
  #define SM_RESETLV_S          "<RSTLV~1.GCO>"
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
  #define SM_FLOW_EXIT_L        "<Exit Flow Settings>.gcode"
  #define SM_FLOW_EXIT_S        "<EXTFLW1.GCO"

  #define SM_EZLVL_MENU_L       "<Easy 4 Point Level>.gcode"
  #define SM_EZLVL_MENU_S       "<EZLVLM1.GCO"
  #define SM_EZLVL_P1_L         "<Point A>           .gcode"
  #define SM_EZLVL_P1_S         "<EZLPA01.GCO"
  #define SM_EZLVL_P2_L         "<Point B>           .gcode"
  #define SM_EZLVL_P2_S         "<EZLPB01.GCO"
  #define SM_EZLVL_P3_L         "<Point C>           .gcode"
  #define SM_EZLVL_P3_S         "<EZLPC01.GCO"
  #define SM_EZLVL_P4_L         "<Point D>           .gcode"
  #define SM_EZLVL_P4_S         "<EZLPD01.GCO"
  #define SM_EZLVL_EXIT_L       "<Exit Easy Leveling>.gcode"
  #define SM_EZLVL_EXIT_S       "<EZLEXT1.GCO"

  #define SM_BLTZMENU_L         "<Auto Leveling>     .gcode"
  #define SM_BLTZMENU_S         "<SETOFZ0.GCO"
  #define SM_BLTZ_DISP_L        "<Z Offset: XXXXX>   .gcode"
  #define SM_BLTZ_DISP_S        "<OFZDSP0.GCO"
  #define SM_BLTZ_UP_L          "<Up>                .gcode"
  #define SM_BLTZ_UP_S          "<UPOFFZ0.GCO"
  #define SM_BLTZ_DN_L          "<Down>              .gcode"
  #define SM_BLTZ_DN_S          "<DOWNOFZ.GCO"
  #define SM_BLTZ_EXIT_L        "<SAVE and EXIT>     .gcode"
  #define SM_BLTZ_EXIT_S        "<EXTABLM.GCO"
#endif // KNUTWURST_DGUS2_TFT

#if ENABLED(KNUTWURST_TFT_LEVELING)
  // eeprom_index
  extern int z_values_index;
  extern int z_values_size;
  // temp value which needs to be saved
  extern float SAVE_zprobe_zoffset;
#endif

class AnycubicTouchscreenClass {
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
#if BOTH(ANYCUBIC_TFT_DEBUG, KNUTWURST_DGUS2_TFT)
  void Command(const char * const command);
#endif
char TFTstate = ANYCUBIC_TFT_STATE_IDLE;

/**
     * Anycubic TFT pause states:
     *
     * 0 - printing / stopped
     * 1 - regular pause
     * 2 - M600 pause
     * 3 - filament runout pause
     * 4 - nozzle timeout on regular pause   // OBSOLETE
     * 5 - nozzle timeout on M600            // OBSOLETE
     * 6 - nozzle timeout on filament runout // OBSOLETE
     */
uint8_t ai3m_pause_state = 0;

private:
char TFTcmdbuffer[TFTBUFSIZE][TFT_MAX_CMD_SIZE];
int TFTbuflen  = 0;
int TFTbufindr = 0;
int TFTbufindw = 0;
char serial3_char;
int serial3_count = 0;
char *TFTstrchr_pointer;
char FlagResumFromOutage  = 0;
int filenumber            = 0;
unsigned long starttime   = 0;
unsigned long stoptime    = 0;
uint8_t tmp_extruder      = 0;
char LastSDstatus         = 0;
uint16_t HeaterCheckCount = 0;
bool IsParked             = false;
int currentFlowRate       = 0;
bool PrintdoneAndPowerOFF = true;
bool powerOFFflag         = 0;

#if defined(POWER_OUTAGE_TEST)
  struct OutageDataStruct {
    char OutageDataVersion;
    char OutageFlag;
    float last_position[XYZE];
    float last_bed_temp;
    float last_hotend_temp;
    long lastSDposition;
  }
  OutageData;
#endif

int CodeValueInt();
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
String flowRateBuffer;
String zOffsetBuffer;
uint16_t MyFileNrCnt          = 0;
uint8_t FilamentSensorEnabled = true;

uint8_t SpecialMenu = false;
uint8_t MMLMenu     = false;
uint8_t FlowMenu    = false;
uint8_t BLTouchMenu = false;
uint8_t LevelMenu   = false;
uint8_t CaseLight   = true;

#if ENABLED(ANYCUBIC_FILAMENT_RUNOUT_SENSOR)
  char FilamentTestStatus     = false;
  char FilamentTestLastStatus = false;
  bool FilamentSetMillis      = true;
  int FilamentRunoutCounter   = 0;
#endif

#if ENABLED(KNUTWURST_MEGA_P_LASER)
  typedef struct {
    unsigned char bfType[2];
    unsigned char bfSize[4];
    unsigned char bfReserved1[2];
    unsigned char bfReserved2[2];
    unsigned char bfOffBits[4];
    unsigned char biSize[4];
    unsigned char biWidth[4];
    unsigned char biHeight[4];
    unsigned char biPlanes[2];
    unsigned char biBitCount[2];
    unsigned char biCompression[4];
    unsigned char biSizeImage[4];
    unsigned char biXPelsPerMeter[4];
    unsigned char biYPelsPerMeter[4];
    unsigned char biClrUsed[4];
    unsigned char biClrImportant[4];
  } BMP_HEAD;

  typedef struct {
    unsigned long pic_file_size;
    unsigned long pic_ptr;
    unsigned long pic_start;

    float pic_pixel_distance;
    float laser_height;
    float x_offset;
    float y_offset;

    unsigned int pic_realy_widht;
    unsigned int pic_widht;
    unsigned int pic_hight;
    unsigned char pic_bit;
    unsigned char pic_widht_odd;
    unsigned char pic_hight_odd;

    unsigned char pic_print_status;
    unsigned char pic_dir;

    unsigned char pic_vector;
    unsigned char pic_x_mirror;
    unsigned char pic_y_mirror;
    unsigned char pic_laser_time;
  } PRINTER_STRUCT;

  #define PIC_FIXED 0.1f     //  //  POINT/MM
  #define PIC_OPEN  50     //  //  ms
  #define PIC_SPEDD 20000
  #define MIN_GRAY_VLAUE  20
  #define LASER_PRINT_SPEED 30      // 50*60
#endif // if ENABLED(KNUTWURST_MEGA_P_LASER)
};

extern AnycubicTouchscreenClass AnycubicTouchscreen;

#endif // ifndef anycubic_touchscreen_h
