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

#include "../../../inc/MarlinConfigPre.h"
#include "../../../module/probe.h"

#define TFTBUFSIZE                 4
#define TFT_MAX_CMD_SIZE           96
#define MSG_MY_VERSION             CUSTOM_BUILD_VERSION
#define MAX_PRINTABLE_FILENAME_LEN 26
#define WAIT_MS_UNTIL_ACYCLIC_SEND 500

enum AnycubicMediaPrintState {
  AMPRINTSTATE_NOT_PRINTING,
  AMPRINTSTATE_PRINTING,
  AMPRINTSTATE_PAUSE_REQUESTED,
  AMPRINTSTATE_PAUSED,
  AMPRINTSTATE_STOP_REQUESTED,
  AMPRINTSTATE_PROBING
};

enum AnycubicMediaPauseState {
  AMPAUSESTATE_NOT_PAUSED,
  AMPAUSESTATE_PARKING,
  AMPAUSESTATE_PARKED,
  AMPAUSESTATE_FILAMENT_OUT,
  AMPAUSESTATE_FILAMENT_PURGING,
  AMPAUSESTATE_HEATER_TIMEOUT,
  AMPAUSESTATE_REHEATING,
  AMPAUSESTATE_REHEAT_FINISHED,
  AMPAUSESTATE_PAUSED
};


#define SM_DIR_UP_S        "DIR_UP~1.GCO"
#define SM_SPECIAL_MENU_S  "<SPECI~1.GCO"
#define SM_PID_HOTEND_S    "<PIDTU~1.GCO"
#define SM_PID_BED_S       "<PIDTU~2.GCO"
#define SM_SAVE_EEPROM_S   "<SAVEE~1.GCO"
#define SM_LOAD_DEFAULTS_S "<LOADF~1.GCO"
#define SM_LOAD_LAST_S     "<LOADLAS.GCO" //dsl
#define SM_PREHEAT_BED_S   "<PREHEBD.GCO" //dsl
#define SM_HEAT_BED_S      "<PREHEHT.GCO" //dsl
#define SM_MMLVL_MENU_S    "<MESHL~1.GCO"
#define SM_MMLVL_START_S   "<START~1.GCO"
#define SM_MMLVL_NEXT_S    "<NEXTM~1.GCO"
#define SM_Z_UP_01_S       "<ZUP01~1.GCO"
#define SM_Z_DN_01_S       "<ZDOWN~1.GCO"
#define SM_Z_UP_002_S      "<ZUP00~1.GCO"
#define SM_Z_DN_002_S      "<ZDOWN~2.GCO"
#define SM_Z_UP_001_S      "<ZUP00~2.GCO"
#define SM_Z_DN_001_S      "<ZDOWN~3.GCO"
//dsl depricated #define SM_BLTOUCH_S       "<BLTOU~1.GCO"
#define SM_RESETLV_S       "<RSTLV~1.GCO"
#define SM_FIL_PAUSE_S     "<FILCHPS.GCO" //dsl
#define SM_FIL_RESUME_S    "<FILCHRS.GCO" //dsl
#define SM_DIS_FILSENS_S   "<DISAB~1.GCO"
#define SM_EN_FILSENS_S    "<ENABL~1.GCO"
#define SM_SPECMENU_EXIT_S "<EXITSPE.GCO"
#define SM_MMLVL_EXIT_S  "<EXITMMM.GCO"
#define SM_FLOWMENU_S      "<SETFLO1.GCO"
#define SM_FLOW_DISP_S     "<FLDISPL.GCO"
#define SM_FLOW_UP_S       "<UPFLOW1.GCO"
#define SM_FLOW_DN_S       "<DWNFLOW.GCO"
#define SM_FLOW_EXIT_S     "<EXTFLW1.GCO"
#define SM_4PLVL_MENU_S    "<EZLVLM1.GCO" //dsl
#define SM_4PLVL_P1_S      "<EZLPA01.GCO" //dsl
#define SM_4PLVL_P2_S      "<EZLPB01.GCO" //dsl
#define SM_4PLVL_P3_S      "<EZLPC01.GCO" //dsl
#define SM_4PLVL_P4_S      "<EZLPD01.GCO" //dsl
#define SM_4PLVL_EXIT_S    "<EZLEXT1.GCO" //dsl
#define SM_PROBE_MENU_S    "<PRBOMNU.GCO" //dsl
#define SM_ZOFF_DISP_S     "<OFZDSP0.GCO"
#define SM_PRB_DN_S        "<PBOFFZ0.GCO" //dsl
#define SM_NOZ_UP_S        "<UPOFFZ0.GCO" //dsl
#define SM_NOZ_DN_S        "<DOWNOFZ.GCO" //dsl
#define SM_PROBE_EXIT_S    "<EXTABLM.GCO" //dsl
#define SM_PROBE_ENA_HS_S  "<HSENABL.GCO" //dsl
#define SM_BABYSTEP_UP_B_S "<BSTPUPB.GCO" //dsl
#define SM_BABYSTEP_UP_S_S "<BSTPUPS.GCO" //dsl
#define SM_BABYSTEP_DN_B_S "<BSTPDNB.GCO" //dsl
#define SM_BABYSTEP_DN_S_S "<BSTPDNS.GCO" //dsl
#define SM_AUTOMESH_MENU_S "<MSHLVLM.GCO" //dsl
#define SM_START_MESH_S    "<BLTLMNU.GCO" //dsl
#define SM_ENA_FADE_MESH_S "<ENAFADE.GCO" //dsl
#define SM_MESH_LVL_EXIT_S "<MSHEXIT.GCO" //dsl
#define SM_TEST_MENU_S     "<TESTMNU.GCO" //dsl
#define SM_TEST_ZOFFDISP_S "<TSTZOFF.GCO" //dsl
#define SM_TEST_NOZZLE_S   "<TESTNOZ.GCO" //dsl
#define SM_TEST_NOZ_UP_S   "<TSTBSUP.GCO" //dsl
#define SM_TEST_NOZ_DN_S   "<TSTBSDN.GCO" //dsl
#define SM_TEST_PRT_PLA_S  "<TESTPLA.GCO" //dsl
#define SM_TEST_PRT_ABS_S  "<TESTABS.GCO" //dsl
#define SM_TEST_PRT_TPU_S  "<TESTTPU.GCO" //dsl
#define SM_TESTMENU_EXIT_S "<EXTSMNU.GCO" //dsl
#define SM_NOOP_S          "<NOOPXXX.GCO" //dsl

#if DISABLED(KNUTWURST_DGUS2_TFT)
  #define SM_DIR_UP_L        "/.."
  #define SM_SPECIAL_MENU_L  "<SPECIAL MENU>"        //dsl
  #define SM_PID_HOTEND_L    "<Tune Hotend 215C>"    //dsl
  #define SM_PID_BED_L       "<Tune Bed 60C>"        //dsl
  #define SM_SAVE_EEPROM_L   "<SAVE to EEPROM>"      //dsl
  #define SM_LOAD_DEFAULTS_L "<Restore to FACTORYs>" //dsl
  #define SM_LOAD_LAST_L     "<Restore to LAST>"     //dsl
  #define SM_PREHEAT_BED_L   "<Preheat Bed 60C>"     //dsl
  #define SM_HEAT_BED_L      "<60C/OC Heat Bed>"     //dsl
  #define SM_MMLVL_MENU_L    "<MANL MESH LVL MENU>"  //dsl
  #define SM_MMLVL_START_L   "<Start Mesh Leveling>"
  #define SM_MMLVL_NEXT_L    "<Next Mesh Point>"
  #define SM_Z_UP_01_L       "<Z Up   0.1>"  //dsl
  #define SM_Z_DN_01_L       "<Z Down 0.1>"
  #define SM_Z_UP_002_L      "<Z Up   0.02>" //dsl
  #define SM_Z_DN_002_L      "<Z Down 0.02>"
  #define SM_Z_UP_001_L      "<Z Up   0.01>" //dsl
  #define SM_Z_DN_001_L      "<Z Down 0.01>"
  #define SM_RESETLV_L       "<Reset Level Grid>"
  #define SM_FIL_PAUSE_L     "<Fil. Change Pause>"
  #define SM_FIL_RESUME_L    "<Fil. Change Resume>"
  #define SM_DIS_FILSENS_L   "<Disable Fil. Sensor>"
  #define SM_EN_FILSENS_L    "<Enable Fil. Sensor>"
  #define SM_SPECMENU_EXIT_L "<Exit Special Menu>" //dsl
  #define SM_MMLVL_EXIT_L    "<Exit ManlMesh Lvlg>"//dsl
  #define SM_FLOWMENU_L      "<SET FLOWRATE MENU>" //dsl
  #define SM_FLOW_DISP_L     "<Flow is XXX>"
  #define SM_FLOW_UP_L       "<Up>"
  #define SM_FLOW_DN_L       "<Down>"
  #define SM_FLOW_EXIT_L     "<Exit Flow Menu>"
  #define SM_4PLVL_MENU_L    "<4 POINT LEVEL MENU>"//dsl
  #define SM_4PLVL_P1_L      "<Point A to paper>"  //dsl
  #define SM_4PLVL_P2_L      "<Point B to paper>"  //dsl
  #define SM_4PLVL_P3_L      "<Point C to paper>"  //dsl
  #define SM_4PLVL_P4_L      "<Point D to paper>"  //dsl
  #define SM_4PLVL_EXIT_L    "<Exit 4 Point Menu>" //dsl
  #define SM_PROBE_MENU_L    "<PROBE Z-OFF MENU>"  //dsl
  #define SM_ZOFF_DISP_L     "<Z Offset: XXXXX>"   //dsl
  #define SM_PRB_DN_L        "<Probe  Dn to paper>"//dsl
  #define SM_NOZ_UP_L        "<Up>"                //dsl
  #define SM_NOZ_DN_L        "<Down>"              //dsl
  #define SM_PROBE_EXIT_L    "<Exit Probe Menu>"   //dsl
  #define SM_PROBE_ENA_HS_L  "<Ena/Dis HiSpd Mesh>"//dsl
  #define SM_BABYSTEP_UP_L_L "<BabyStep Up .1>"    //dsl
  #define SM_BABYSTEP_UP_S_L "<BabyStep Up .02>"   //dsl
  #define SM_BABYSTEP_DN_L_L "<BabyStep Dn .1>"    //dsl
  #define SM_BABYSTEP_DN_S_L "<BabyStep Dn .02>"   //dsl
  #define SM_AUTOMESH_MENU_L "<MESH LEVEL MENU>"   //dsl
  #define SM_START_MESH_L    "<Start Mesh Level>"  //dsl
  #define SM_ENA_FADE_MESH_L "<Ena/Dis MeshFade 5>"//dsl
  #define SM_MESH_LVL_EXIT_L "<Mesh Level Exit>"   //dsl
  #define SM_TEST_MENU_L     "<TEST MENU>"         //dsl
  #define SM_TEST_ZOFFDISP_L "<Z Offset: XXXXX>"   //dsl
  #define SM_TEST_NOZZLE_L   "<Test Nozzle paper>" //dsl
  #define SM_TEST_NOZ_UP_L   "<Baby Step Up .02>"  //dsl
  #define SM_TEST_NOZ_DN_L   "<Baby Step DN .02>"  //dsl
  #define SM_TEST_PRT_PLA_L  "<Test Print PLA>"    //dsl
  #define SM_TEST_PRT_ABS_L  "<Test Print ABS>"    //dsl
  #define SM_TEST_PRT_TPU_L  "<Test Print TPU>"    //dsl
  #define SM_TESTMENU_EXIT_L "<Exit Test Menu>"    //dsl
  #define SM_NOOP_L          "<noop>"              //dsl
#endif // !KNUTWURST_DGUS2_TFT

#if ENABLED(KNUTWURST_DGUS2_TFT)
  #define SM_DIR_UP_L        "/..                 .gcode"
  #define SM_SPECIAL_MENU_L  "<SPECIAL MENU>      .gcode" //dsl
  #define SM_PID_HOTEND_L    "<Tune HotEnd 215C>  .gcode" //dsl
  #define SM_PID_BED_L       "<Tune Bed 60C>      .gcode" //dsl
  #define SM_SAVE_EEPROM_L   "<SAVE to EEPROM>    .gcode" //dsl
  #define SM_LOAD_DEFAULTS_L "<Restore to FACTORY>.gcode" //dsl
  #define SM_LOAD_LAST_L     "<Restore to LAST>   .gcode" //dsl
  #define SM_PREHEAT_BED_L   "<Preheat Bed 60C>   .gcode" //dsl
  #define SM_HEAT_BED_L      "<60C/OC Heat Bed>   .gcode" //dsl
  #define SM_MMLVL_MENU_L    "<MANL MESH LVL MENU>.gcode" //dsl
  #define SM_MMLVL_START_L   "<Start MeshLeveling>.gcode"
  #define SM_MMLVL_NEXT_L    "<Next Mesh Point>   .gcode"
  #define SM_Z_UP_01_L       "<Z Up 0.1>          .gcode"
  #define SM_Z_DN_01_L       "<Z Dn 0.1>          .gcode" //dsl
  #define SM_Z_UP_002_L      "<Z Up 0.02>         .gcode"
  #define SM_Z_DN_002_L      "<Z Dn 0.02>         .gcode" //dsl
  #define SM_Z_UP_001_L      "<Z Up 0.01>         .gcode"
  #define SM_Z_DN_001_L      "<Z Dn 0.01>         .gcode" //dsl
  #define SM_RESETLV_L       "<Reset Level Grid>  .gcode"
  #define SM_FIL_PAUSE_L     "<Fil. Change Pause> .gcode"
  #define SM_FIL_RESUME_L    "<Fil. Change Resume>.gcode"
  #define SM_DIS_FILSENS_L   "<Disable Fil.Sensor>.gcode"
  #define SM_EN_FILSENS_L    "<Enable Fil. Sensor>.gcode"
  #define SM_SPECMENU_EXIT_L "<Exit Special Menu> .gcode" //dsl
  #define SM_MMLVL_EXIT_L    "<Exit ManlMesh Lvl> .gcode" //dsl
  #define SM_FLOWMENU_L      "<SET FLOWRATE MENU> .gcode" //dsl
  #define SM_FLOW_DISP_L     "<Flow is XXX%>      .gcode"
  #define SM_FLOW_UP_L       "<Up>                .gcode"
  #define SM_FLOW_DN_L       "<Down>              .gcode"
  #define SM_FLOW_EXIT_L     "<Exit Flow Menu>    .gcode"
  #define SM_4PLVL_MENU_L    "<4 POINT LEVEL MENU>.gcode" //dsl
  #define SM_4PLVL_P1_L      "<Point A to paper>  .gcode" //dsl
  #define SM_4PLVL_P2_L      "<Point B to paper>  .gcode" //dsl
  #define SM_4PLVL_P3_L      "<Point C to paper>  .gcode" //dsl
  #define SM_4PLVL_P4_L      "<Point D to paper>  .gcode" //dsl
  #define SM_4PLVL_EXIT_L    "<Exit 4 Point Menu> .gcode" //dsl
  #define SM_PROBE_MENU_L    "<PROBE Z-OFF MENU>  .gcode" //dsl
  #define SM_ZOFF_DISP_L     "<Z Offset: XXXXX>   .gcode" //dsl
  #define SM_PRB_DN_L        "<Probe  Dn to paper>.gcode" //dsl
  #define SM_NOZ_UP_L        "<Nozzle Up to paper>.gcode" //dsl
  #define SM_NOZ_DN_L        "<Nozzle Dn to paper>.gcode" //dsl
  #define SM_PROBE_EXIT_L    "<Exit Probe Menu>   .gcode" //dsl
  #define SM_PROBE_ENA_HS_L  "<Ena/Dis HiSpd Mesh>.gcode" //dsl
  #define SM_BABYSTEP_UP_L_L "<BabyStep Up .1>    .gcode" //dsl
  #define SM_BABYSTEP_UP_S_L "<BabyStep Up .02>   .gcode" //dsl
  #define SM_BABYSTEP_DN_L_L "<BabyStep Dn .1>    .gcode" //dsl
  #define SM_BABYSTEP_DN_S_L "<BabyStep Dn .02>   .gcode" //dsl
  #define SM_AUTOMESH_MENU_L "<AUTO MESH LVL MENU>.gcode" //dsl
  #define SM_START_MESH_L    "<Start Mesh Level>  .gcode" //dsl
  #define SM_ENA_FADE_MESH_L "<Ena/Dis MeshFade 5>.gcode" //dsl
  #define SM_MESH_LVL_EXIT_L "<Mesh Level Exit>   .gcode" //dsl
  #define SM_TEST_MENU_L     "<TEST MENU>         .gcode" //dsl
  #define SM_TEST_ZOFFDISP_L "<Z Offset: XXXXX>   .gcode" //dsl
  #define SM_TEST_NOZZLE_L   "<Test Nozzle paper> .gcode" //dsl
  #define SM_TEST_NOZ_UP_L   "<Noz Up .02 paper>  .gcode" //dsl
  #define SM_TEST_NOZ_DN_L   "<Noz DN .02 paper>  .gcode" //dsl
  #define SM_TEST_PRT_PLA_L  "<Test Print PLA>    .gcode" //dsl
  #define SM_TEST_PRT_ABS_L  "<Test Print ABS>    .gcode" //dsl
  #define SM_TEST_PRT_TPU_L  "<Test Print TPU>    .gcode" //dsl
  #define SM_TESTMENU_EXIT_L "<Exit Test Menu>    .gcode" //dsl
  #define SM_NOOP_L          "<noop>              .gcode" //dsl
  
#endif // KNUTWURST_DGUS2_TFT

#define ENTER_MENU_SOUND    BUZZ(200, 1108); BUZZ(200, 1661); BUZZ(200, 1108); BUZZ(600, 1661); //dsl
#define EXIT_MENU_SOUND     BUZZ(200, 1661); BUZZ(200, 1108); BUZZ(600, 1661); BUZZ(200, 1108); //dsl 

class AnycubicTouchscreenClass {

  public:
    AnycubicTouchscreenClass();

    void Setup();
    void CommandScan();
    void FilamentRunout();
    void DoFilamentRunoutCheck();
    void UserConfirmRequired(const char*);
    void SDCardStateChange(bool);
    void SDCardError();
    void KillTFT();
    void OnPrintTimerStarted();
    void OnPrintTimerPaused();
    void OnPrintTimerStopped();

#if BOTH(ANYCUBIC_TFT_DEBUG, KNUTWURST_DGUS2_TFT)
    void Command(const char* const command);
#endif

#if ENABLED(KNUTWURST_CHIRON)
    void LevelingDone();
#endif

  private:
    char       TFTcmdbuffer[TFTBUFSIZE][TFT_MAX_CMD_SIZE];
    int        TFTbuflen  = 0;
    int        TFTbufindr = 0;
    int        TFTbufindw = 0;
    char       serial3_char;
    int        serial3_count = 0;
    char*      TFTstrchr_pointer;
    char       FlagResumFromOutage  = 0;
    uint16_t   HeaterCheckCount     = 0;
    int        currentFlowRate      = 0;
    bool       PrintdoneAndPowerOFF = true;
    bool       powerOFFflag         = false;
    xy_uint8_t selectedmeshpoint;
    float      live_Zoffset;

    static AnycubicMediaPrintState mediaPrintingState;
    static AnycubicMediaPauseState mediaPauseState;
    static uint32_t time_last_cyclic_tft_command;
    static uint8_t delayed_tft_command;

#if defined(POWER_OUTAGE_TEST)
    struct OutageDataStruct {
        char  OutageDataVersion;
        char  OutageFlag;
        float last_position[XYZE];
        float last_bed_temp;
        float last_hotend_temp;
        long  lastSDposition;
    } OutageData;
#endif

    int   CodeValueInt();
    float CodeValue();
    bool  CodeSeen(char);
    void  StartPrint();
    void  PausePrint();
    void  ResumePrint();
    void  StopPrint();
    void  GetCommandFromTFT();
    void  CheckHeaterError();
    void  HandleSpecialMenu();
    void  RenderCurrentFileList();
    void  RenderSpecialMenu(uint16_t);
    void  RenderCurrentFolder(uint16_t);

    char   currentTouchscreenSelection[64];
    char   currentFileOrDirectory[64];
    String flowRateBuffer;
    String zOffsetBuffer;

    //dsl Menu booleans
    uint8_t SpecialMenu          = false;
    uint8_t ManualMeshLevelMenu  = false; //dsl
    uint8_t FlowMenu             = false;
    uint8_t ProbeMenu            = false; //dsl
    uint8_t Manual4PntLevelMenu  = false; //dsl
    uint8_t CaseLight            = true;
    uint8_t TestMenu             = false; //dsl
    uint8_t AutoMeshLevelMenu    = false; //dsl

    //dsl Menu item toggles
    uint8_t HeatBed              = false; //dsl
    uint8_t EnableHighSpeedProbe = false; //dsl
    uint8_t EnableMeshFade       = false; //dsl



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

        unsigned int  pic_real_width;
        unsigned int  pic_width;
        unsigned int  pic_height;
        unsigned char pic_bit;
        unsigned char pic_width_odd;
        unsigned char pic_height_odd;

        unsigned char pic_print_status;
        unsigned char pic_dir;

        unsigned char pic_vector;
        unsigned char pic_x_mirror;
        unsigned char pic_y_mirror;
        unsigned char pic_laser_time;
    } PRINTER_STRUCT;

  #define PIC_FIXED         0.1f //  //  POINT/MM
  #define PIC_OPEN          50   //  //  ms
  #define PIC_SPEDD         20000
  #define MIN_GRAY_VLAUE    20
  #define LASER_PRINT_SPEED 30 // 50*60
#endif                         // if ENABLED(KNUTWURST_MEGA_P_LASER)
};

extern AnycubicTouchscreenClass AnycubicTouchscreen;

#endif // ifndef anycubic_touchscreen_h
