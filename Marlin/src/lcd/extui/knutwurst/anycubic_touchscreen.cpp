/*
   AnycubicTouchscreen.cpp  --- Support for Anycubic i3 Mega TFT
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

#include "Arduino.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../ui_api.h"
#include "../../../gcode/queue.h"
#include "../../../feature/bedlevel/bedlevel.h"
#include "../../../libs/buzzer.h"
#include "../../../libs/numtostr.h"
#include "../../../module/temperature.h"
#include "../../../module/motion.h"
#include "../../../module/settings.h"
#include "../../../module/stepper.h"

//#define ANYCUBIC_TFT_DEBUG

#ifdef ANYCUBIC_TOUCHSCREEN
  #include "./anycubic_touchscreen.h"

  // command sending macro's with debugging capability
  #define SEND_PGM(x)       send_P(PSTR(x))
  #define SENDLINE_PGM(x)   sendLine_P(PSTR(x))
  #define SEND_PGM_VAL(x,y) (send_P(PSTR(x)), sendLine(i16tostr3rj(y)))
  #define SEND(x)           send(x)
  #define SENDLINE(x)       sendLine(x)
  #if ENABLED(ANYCUBIC_TFT_DEBUG)
    #define SENDLINE_DBG_PGM(x,y)       do{ sendLine_P(PSTR(x)); SERIAL_ECHOLNPGM(y); }while(0)
    #define SENDLINE_DBG_PGM_VAL(x,y,z) do{ sendLine_P(PSTR(x)); SERIAL_ECHOLNPGM(y, z); }while(0)
  #else
    #define SENDLINE_DBG_PGM(x,y)       sendLine_P(PSTR(x))
    #define SENDLINE_DBG_PGM_VAL(x,y,z) sendLine_P(PSTR(x))
  #endif

  // Serial helpers
  static void sendNewLine() { LCD_SERIAL.write('\r'); LCD_SERIAL.write('\n'); }
  static void send(const char *str) { LCD_SERIAL.print(str); }
  static void send_P(PGM_P str) {
    while (const char c = pgm_read_byte(str++))
      LCD_SERIAL.write(c);
  }
  static void sendLine(const char *str) { send(str); sendNewLine(); }
  static void sendLine_P(PGM_P str) { send_P(str); sendNewLine(); }

  char _conv[8];

  AnycubicMediaPrintState AnycubicTouchscreenClass::mediaPrintingState = AMPRINTSTATE_NOT_PRINTING;
  AnycubicMediaPauseState AnycubicTouchscreenClass::mediaPauseState = AMPAUSESTATE_NOT_PAUSED;

  #if ENABLED(KNUTWURST_TFT_LEVELING)
    int z_values_index;
    int z_values_size;
    float SAVE_zprobe_zoffset;
    uint8_t x;
    uint8_t y;

    void restore_z_values() {
      uint16_t size  = z_values_size;
      int pos        = z_values_index;
      uint8_t* value = (uint8_t*)&bedlevel.z_values;
      do {
        uint8_t c = eeprom_read_byte((unsigned char*)pos);
        *value = c;
        pos++;
        value++;
        if(pos > 32766)
          break;
      } while (--size);
    }

    void setupMyZoffset() {
      #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
        SERIAL_ECHOPGM("MEANL_L:", 0x55);
        SAVE_zprobe_zoffset = probe.offset.z;
      #else
        SERIAL_ECHOPGM("MEANL_L:", 0xaa);
        constexpr float dpo[] = NOZZLE_TO_PROBE_OFFSET;
        probe.offset.z = dpo[Z_AXIS];
      #endif
    }

    void initializeGrid() {
      #if ENABLED(PROBE_MANUALLY)
        #define ABL_VAR static
      #else
        #define ABL_VAR
      #endif

      ABL_VAR xy_pos_t probe_position_lf, probe_position_rb;
      // ABL_VAR xy_float_t gridSpacing = { 0, 0 };

      const float x_min = probe.min_x(), x_max = probe.max_x(),
                  y_min = probe.min_y(), y_max = probe.max_y();

      constexpr xy_uint8_t abl_grid_points = { GRID_MAX_POINTS_X, GRID_MAX_POINTS_Y };
      GCodeParser parser;

      // Reset grid to 0.0 or "not probed". (Also disables ABL)
      reset_bed_level();

      // Initialize a grid with the given dimensions
      probe_position_lf.set(
        parser.seenval('L') ? RAW_X_POSITION(parser.value_linear_units()) : x_min,
        parser.seenval('F') ? RAW_Y_POSITION(parser.value_linear_units()) : y_min
        );
      probe_position_rb.set(
        parser.seenval('R') ? RAW_X_POSITION(parser.value_linear_units()) : x_max,
        parser.seenval('B') ? RAW_Y_POSITION(parser.value_linear_units()) : y_max
        );
      LevelingBilinear::grid_spacing.set((probe_position_rb.x - probe_position_lf.x) / (abl_grid_points.x - 1),
        (probe_position_rb.y - probe_position_lf.y) / (abl_grid_points.y - 1));

      LevelingBilinear::grid_start = probe_position_lf;
      // Can't re-enable (on error) until the new grid is written
      set_bed_leveling_enabled(false);

      constexpr float dpo[] = NOZZLE_TO_PROBE_OFFSET;
      probe.offset.z = dpo[Z_AXIS];

      for (uint8_t x = 0; x < GRID_MAX_POINTS_X; x++)
        for (uint8_t y = 0; y < GRID_MAX_POINTS_Y; y++) bedlevel.z_values[x][y] = float(-1.0);
      bedlevel.refresh_bed_level();
      set_bed_leveling_enabled(true);
    }
  #endif // if ENABLED(KNUTWURST_TFT_LEVELING)

  #if ENABLED(POWER_OUTAGE_TEST)
    int PowerInt                     = 6;
    unsigned char PowerTestFlag      = false;
    int Temp_Buf_Extuder_Temperature = 0;
    int Temp_Buf_Bed_Temperature     = 0;
    unsigned char ResumingFlag       = 0;
  #endif

  void setup_PowerOffPin() {
    #if ENABLED(KNUTWURST_4MAXP2)
      SET_OUTPUT(POWER_OFF_PIN);
      WRITE(POWER_OFF_PIN, HIGH);
    #endif
  }

  void setup_OutageTestPin() {
    #ifdef POWER_OUTAGE_TEST
      pinMode(OUTAGETEST_PIN, INPUT);
      pinMode(OUTAGECON_PIN, OUTPUT);
      WRITE(OUTAGECON_PIN, LOW);
    #endif
  }

  using namespace ExtUI;

  AnycubicTouchscreenClass::AnycubicTouchscreenClass() {
  }

  void AnycubicTouchscreenClass::Setup() {
    #ifndef LCD_BAUDRATE
      #define LCD_BAUDRATE 115200
    #endif
    LCD_SERIAL.begin(LCD_BAUDRATE);

    #if DISABLED(KNUTWURST_4MAXP2)
      SENDLINE_DBG_PGM("J17", "TFT Serial Debug: Main board reset... J17"); // J17 Main board reset
      delay_ms(10);
    #endif

    mediaPrintingState = AMPRINTSTATE_NOT_PRINTING;
    mediaPauseState = AMPAUSESTATE_NOT_PAUSED;

    currentTouchscreenSelection[0] = 0;
    currentFileOrDirectory[0]      = '\0';
    SpecialMenu                    = false;
    MMLMenu                        = false;
    FlowMenu                       = false;
    BLTouchMenu                    = false;
    LevelMenu                      = false;
    CaseLight                      = false;
    currentFlowRate                = 100;
    flowRateBuffer                 = SM_FLOW_DISP_L;

    #if BOTH(SDSUPPORT, HAS_SD_DETECT)
      SET_INPUT_PULLUP(SD_DETECT_PIN);
    #endif
    
    #if ENABLED(FILAMENT_RUNOUT_SENSOR)
      SET_INPUT_PULLUP(FIL_RUNOUT1_PIN);
    #endif

    #if ENABLED(KNUTWURST_TFT_LEVELING)
      setupMyZoffset();
      delay(10);
    #endif

    setup_OutageTestPin();
    setup_PowerOffPin();

    SENDLINE_DBG_PGM("J12", "TFT Serial Debug: Ready... J12");

    CheckHeaterError();
    DoFilamentRunoutCheck();
    

    #ifdef STARTUP_CHIME
      BUZZ(100, 554);
      BUZZ(100, 740);
      BUZZ(100, 831);
    #endif
  }

  #if ENABLED(KNUTWURST_MEGA_P_LASER)
    PRINTER_STRUCT Laser_printer_st = {0};
    BMP_HEAD st_bmp                 = {0};

    void laser_init() {
      Laser_printer_st.pic_pixel_distance = PIC_FIXED;
      Laser_printer_st.laser_height       = 50;
      Laser_printer_st.x_offset           = 0;
      Laser_printer_st.x_offset           = 0;

      Laser_printer_st.pic_vector     = 0;
      Laser_printer_st.pic_x_mirror   = 1;
      Laser_printer_st.pic_y_mirror   = 0;
      Laser_printer_st.pic_laser_time = 15;

      send_laser_param();
    }

    void send_pic_param() {
      SEND_PGM("A45V W"); LCD_SERIAL.print(Laser_printer_st.pic_widht);
      SEND_PGM(    " H"); LCD_SERIAL(Laser_printer_st.pic_hight);
      SENDLINE_PGM(" ");
    }

    void send_laser_param() {
      SEND_PGM("A44V A"); LCD_SERIAL.print(Laser_printer_st.pic_vector);
      SEND_PGM(    " B"); LCD_SERIAL.print(Laser_printer_st.pic_laser_time);
      SEND_PGM(    " C"); LCD_SERIAL.print(Laser_printer_st.laser_height);
      SEND_PGM(    " D"); LCD_SERIAL.print(Laser_printer_st.pic_pixel_distance);
      SEND_PGM(    " E"); LCD_SERIAL.print(Laser_printer_st.x_offset);
      SEND_PGM(    " F"); LCD_SERIAL.print(Laser_printer_st.y_offset);
      SEND_PGM(    " G"); LCD_SERIAL.print(Laser_printer_st.pic_x_mirror);
      SEND_PGM(    " H"); LCD_SERIAL.print(Laser_printer_st.pic_y_mirror);
      SENDLINE_PGM(" ");
    }
  #endif // if ENABLED(KNUTWURST_MEGA_P_LASER)

  void AnycubicTouchscreenClass::KillTFT() {
    SENDLINE_DBG_PGM("J11", "TFT Serial Debug: Kill command... J11");
  }

  void AnycubicTouchscreenClass::StartPrint() {
    SENDLINE_DBG_PGM("J04", "TFT Serial Debug: Starting SD Print... J04"); // J04 Starting Print

    if (!isPrinting() && strlen(currentFileOrDirectory) > 0) {
      #if ENABLED(ANYCUBIC_TFT_DEBUG)
        SERIAL_ECHOPGM("TFT Serial Debug: About to print file ... ");
        SERIAL_ECHO(isPrinting());
        SERIAL_ECHOPGM(" ");
        SERIAL_ECHOLN(currentFileOrDirectory);
      #endif
      mediaPrintingState = AMPRINTSTATE_PRINTING;
      mediaPauseState    = AMPAUSESTATE_NOT_PAUSED;
      printFile(currentFileOrDirectory);
    }
  }

  void AnycubicTouchscreenClass::PausePrint() {
    #ifdef SDSUPPORT
      if (isPrintingFromMedia() && mediaPrintingState != AMPRINTSTATE_STOP_REQUESTED && mediaPauseState == AMPAUSESTATE_NOT_PAUSED) {
      mediaPrintingState = AMPRINTSTATE_PAUSE_REQUESTED;
      mediaPauseState    = AMPAUSESTATE_NOT_PAUSED; // need the userconfirm method to update pause state
      SENDLINE_DBG_PGM("J05", "TFT Serial Debug: SD print pause started... J05"); // J05 printing pause

      // for some reason pausing the print doesn't retract the extruder so force a manual one here
      injectCommands(F("G91\nG1 E-2 F1800\nG90"));
      pausePrint();
    }
    #endif
  }

  inline void AnycubicTouchscreenClass::StopPrint() {
    #if ENABLED(SDSUPPORT)
      mediaPrintingState = AMPRINTSTATE_STOP_REQUESTED;
      mediaPauseState    = AMPAUSESTATE_NOT_PAUSED;
      SENDLINE_DBG_PGM("J16", "TFT Serial Debug: SD print stop called... J16");

      // for some reason stopping the print doesn't retract the extruder so force a manual one here
      injectCommands(F("G91\nG1 E-2 F1800\nG90"));
      stopPrint();
    #endif
  }

  void AnycubicTouchscreenClass::ResumePrint() {
    #if ENABLED(SDSUPPORT)
      #if ENABLED(FILAMENT_RUNOUT_SENSOR)
        if (READ(FIL_RUNOUT_PIN)) {
          #if ENABLED(ANYCUBIC_TFT_DEBUG)
            SERIAL_ECHOLNPGM("TFT Serial Debug: Resume Print with filament sensor still tripped... ");
          #endif

          // trigger the user message box
          DoFilamentRunoutCheck();

          // re-enable the continue button
          SENDLINE_DBG_PGM("J18", "TFT Serial Debug: Resume Print with filament sensor still tripped... J18");
          return;
        }
      #endif

      if (mediaPauseState == AMPAUSESTATE_HEATER_TIMEOUT) {
        mediaPauseState = AMPAUSESTATE_REHEATING;
        // reheat the nozzle
        setUserConfirmed();
      }
      else if (mediaPauseState == AMPAUSESTATE_FILAMENT_PURGING) {
        setUserConfirmed();
      }
      else {
        mediaPrintingState = AMPRINTSTATE_PRINTING;
        mediaPauseState    = AMPAUSESTATE_NOT_PAUSED;

        SENDLINE_DBG_PGM("J04", "TFT Serial Debug: SD print resumed... J04"); // J04 printing form sd card now
        resumePrint();
      }
    #endif
  }

  int AnycubicTouchscreenClass::CodeValueInt() {
    return (atoi(&TFTcmdbuffer[TFTbufindr][TFTstrchr_pointer - TFTcmdbuffer[TFTbufindr] + 1]));
  }

  float AnycubicTouchscreenClass::CodeValue() {
    return (strtod(&TFTcmdbuffer[TFTbufindr][TFTstrchr_pointer - TFTcmdbuffer[TFTbufindr] + 1], NULL));
  }

  bool AnycubicTouchscreenClass::CodeSeen(char code) {
    TFTstrchr_pointer = strchr(TFTcmdbuffer[TFTbufindr], code);
    return (TFTstrchr_pointer != NULL); // Return True if a character was found
  }

  void AnycubicTouchscreenClass::HandleSpecialMenu() {
    #if ENABLED(KNUTWURST_SPECIAL_MENU)
      #ifdef ANYCUBIC_TFT_DEBUG
        SERIAL_ECHOPGM("TFT Serial Debug: Special Menu Selection: ", currentTouchscreenSelection);
        SERIAL_EOL();
      #endif
      if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_SPECIAL_MENU_L)) != NULL)
          || (strcasestr_P(currentTouchscreenSelection, PSTR(SM_SPECIAL_MENU_S)) != NULL)
          ) {
        SpecialMenu = true;
      }
      else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_PID_HOTEND_L)) != NULL)
               || (strcasestr_P(currentTouchscreenSelection, PSTR(SM_PID_HOTEND_S)) != NULL)
               ) {
        SERIAL_ECHOLNPGM("Special Menu: PID Tune Hotend");

        #if ANY(KNUTWURST_MEGA, KNUTWURST_MEGA_S, KNUTWURST_MEGA_P)
          injectCommands(F("G28\nG90\nG1 Z20\nG1 X110 Y110 F4000\nG1 Z5\nM106 S172\nG4 P500\nM303 E0 S215 C15 U1\nG4 P500\nM107\nG28\nG1 Z10\nM84\nM500\nM300 S440 P200\nM300 S660 P250\nM300 S880 P300"));
        #endif

        #if ENABLED(KNUTWURST_MEGA_X)
          injectCommands(F("G28\nG90\nG1 Z20\nG1 X155 Y155 F4000\nG1 Z5\nM106 S172\nG4 P500\nM303 E0 S215 C15 U1\nG4 P500\nM107\nG28\nG1 Z10\nM84\nM500\nM300 S440 P200\nM300 S660 P250\nM300 S880 P300"));
        #endif

        #if ENABLED(KNUTWURST_CHIRON)
          injectCommands(F("G28\nG90\nG1 Z20\nG1 X205 Y205 F4000\nG1 Z5\nM106 S172\nG4 P500\nM303 E0 S215 C15 U1\nG4 P500\nM107\nG28\nG1 Z10\nM84\nM500\nM300 S440 P200\nM300 S660 P250\nM300 S880 P300"));
        #endif

        #if ENABLED(KNUTWURST_4MAXP2)
          injectCommands(F("G28\nG90\nG1 Z20\nG1 X105 Y135 F4000\nG1 Z5\nM106 S172\nG4 P500\nM303 E0 S215 C15 U1\nG4 P500\nM107\nG28\nG1 Z10\nM84\nM500\nM300 S440 P200\nM300 S660 P250\nM300 S880 P300"));
        #endif

        BUZZ(200, 1108);
        BUZZ(200, 1661);
        BUZZ(200, 1108);
        BUZZ(600, 1661);
      }
      else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_PID_BED_L)) != NULL)
               || (strcasestr_P(currentTouchscreenSelection, PSTR(SM_PID_BED_S)) != NULL)
               ) {
        SERIAL_ECHOLNPGM("Special Menu: PID Tune Ultrabase");
        BUZZ(200, 1108);
        BUZZ(200, 1661);
        BUZZ(200, 1108);
        BUZZ(600, 1661);
        injectCommands(F("M303 E-1 S60 C6 U1\nM500\nM300 S440 P200\nM300 S660 P250\nM300 S880 P300"));
        BUZZ(200, 1108);
        BUZZ(200, 1661);
        BUZZ(200, 1108);
        BUZZ(600, 1661);
      }
      else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_SAVE_EEPROM_L)) != NULL)
               || (strcasestr_P(currentTouchscreenSelection, PSTR(SM_SAVE_EEPROM_S)) != NULL)
               ) {
        SERIAL_ECHOLNPGM("Special Menu: Save EEPROM");
        settings.save(); // M500
        BUZZ(105, 1108);
        BUZZ(210, 1661);
      }
      else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_LOAD_DEFAULTS_L)) != NULL)
               || (strcasestr_P(currentTouchscreenSelection, PSTR(SM_LOAD_DEFAULTS_S)) != NULL)
               ) {
        SERIAL_ECHOLNPGM("Special Menu: Load FW Defaults");
        settings.reset(); // M502
        #if ENABLED(KNUTWURST_TFT_LEVELING)
          initializeGrid();
        #endif
        BUZZ(105, 1661);
        BUZZ(210, 1108);
      }
      else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_PREHEAT_BED_L)) != NULL)
               || (strcasestr_P(currentTouchscreenSelection, PSTR(SM_PREHEAT_BED_S)) != NULL)
               ) {
        SERIAL_ECHOLNPGM("Special Menu: Preheat Ultrabase");
        injectCommands(F("M140 S60"));
      }

      #if NONE(KNUTWURST_BLTOUCH, KNUTWURST_TFT_LEVELING)
        else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_MESH_MENU_L)) != NULL)
                 || (strcasestr_P(currentTouchscreenSelection, PSTR(SM_MESH_MENU_S)) != NULL)
                 ) {
          MMLMenu = true;
        }
        else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_MESH_START_L)) != NULL)
                 || (strcasestr_P(currentTouchscreenSelection, PSTR(SM_MESH_START_S)) != NULL)
                 ) {
          SERIAL_ECHOLNPGM("Special Menu: Start Mesh Leveling");
          injectCommands(F("G28\nG29 S1"));
        }
        else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_MESH_NEXT_L)) != NULL)
                 || (strcasestr_P(currentTouchscreenSelection, PSTR(SM_MESH_NEXT_S)) != NULL)
                 ) {
          SERIAL_ECHOLNPGM("Special Menu: Next Mesh Point");
          injectCommands(F("G29 S2"));
        }
        else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_Z_UP_01_L)) != NULL)
                 || (strcasestr_P(currentTouchscreenSelection, PSTR(SM_Z_UP_01_S)) != NULL)
                 ) {
          SERIAL_ECHOLNPGM("Special Menu: Z Up 0.1");
          injectCommands(F("G91\nG1 Z+0.1\nG90"));
        }
        else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_Z_DN_01_L)) != NULL)
                 || (strcasestr_P(currentTouchscreenSelection, PSTR(SM_Z_DN_01_S)) != NULL)
                 ) {
          SERIAL_ECHOLNPGM("Special Menu: Z Down 0.1");
          injectCommands(F("G91\nG1 Z-0.1\nG90"));
        }
        else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_Z_UP_002_L)) != NULL)
                 || (strcasestr_P(currentTouchscreenSelection, PSTR(SM_Z_UP_002_S)) != NULL)
                 ) {
          SERIAL_ECHOLNPGM("Special Menu: Z Up 0.02");
          injectCommands(F("G91\nG1 Z+0.02\nG90"));
        }
        else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_Z_DN_002_L)) != NULL)
                 || (strcasestr_P(currentTouchscreenSelection, PSTR(SM_Z_DN_002_S)) != NULL)
                 ) {
          SERIAL_ECHOLNPGM("Special Menu: Z Down 0.02");
          injectCommands(F("G91\nG1 Z-0.02\nG90"));
        }
        else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_Z_UP_001_L)) != NULL)
                 || (strcasestr_P(currentTouchscreenSelection, PSTR(SM_Z_UP_001_S)) != NULL)
                 ) {
          SERIAL_ECHOLNPGM("Special Menu: Z Up 0.01");
          injectCommands(F("G91\nG1 Z+0.03\nG4 P250\nG1 Z-0.02\nG90"));
        }
        else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_Z_DN_001_L)) != NULL)
                 || (strcasestr_P(currentTouchscreenSelection, PSTR(SM_Z_DN_001_S)) != NULL)
                 ) {
          SERIAL_ECHOLNPGM("Special Menu: Z Down 0.01");
          injectCommands(F("G91\nG1 Z+0.02\nG4 P250\nG1 Z-0.03\nG90"));
        }
      #endif // if NONE(KNUTWURST_BLTOUCH, KNUTWURST_TFT_LEVELING)

      #if ENABLED(KNUTWURST_BLTOUCH)
        else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_BLTOUCH_L)) != NULL)
                 || (strcasestr_P(currentTouchscreenSelection, PSTR(SM_BLTOUCH_S)) != NULL)
                 ) {
          SERIAL_ECHOLNPGM("Special Menu: BLTouch Leveling");
          injectCommands(F("G28\nG29\nM500\nG90\nM300 S440 P200\nM300 S660 P250\nM300 S880 P300\nG1 Z30 F4000\nG1 X0 F4000\nG91\nM84\nM420 S1"));
          BUZZ(105, 1108);
          BUZZ(210, 1661);
        }
      #endif

      #if ENABLED(KNUTWURST_TFT_LEVELING)
        else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_RESETLV_L)) != NULL)
                 || (strcasestr_P(currentTouchscreenSelection, PSTR(SM_RESETLV_S)) != NULL)
                 ) {
          SERIAL_ECHOLNPGM("Special Menu: initializeGrid()");
          initializeGrid();
          settings.save();
          BUZZ(105, 1108);
          BUZZ(210, 1661);
        }
      #endif

      else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_PAUSE_L)) != NULL)
               || (strcasestr_P(currentTouchscreenSelection, PSTR(SM_PAUSE_L)) != NULL)
               ) {
        SERIAL_ECHOLNPGM("Special Menu: Fil. Change Pause");
        injectCommands(F("M600"));
      }
      else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_RESUME_L)) != NULL)
               || (strcasestr_P(currentTouchscreenSelection, PSTR(SM_RESUME_S)) != NULL)
               ) {
        SERIAL_ECHOLNPGM("Special Menu: Fil. Change Resume");
        ResumePrint();
      }
      else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_DIS_FILSENS_L)) != NULL)
               || (strcasestr_P(currentTouchscreenSelection, PSTR(SM_DIS_FILSENS_S)) != NULL)
               ) {
        SERIAL_ECHOLNPGM("Special Menu: Disable Filament Sensor");
        injectCommands(F("M412 H0 S0\nM500"));
        BUZZ(105, 1108);
        BUZZ(105, 1108);
        BUZZ(105, 1108);
      }
      else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_EN_FILSENS_L)) != NULL)
               || (strcasestr_P(currentTouchscreenSelection, PSTR(SM_EN_FILSENS_S)) != NULL)
               ) {
        SERIAL_ECHOLNPGM("Special Menu: Enable Filament Sensor");
        injectCommands(F("M412 H0 S1\nM500"));
        BUZZ(105, 1108);
        BUZZ(105, 1108);
      }
      else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_EXIT_L)) != NULL)
               || (strcasestr_P(currentTouchscreenSelection, PSTR(SM_EXIT_S)) != NULL)
               ) {
        SpecialMenu = false;
      }
      else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_BACK_L)) != NULL)
               || (strcasestr_P(currentTouchscreenSelection, PSTR(SM_BACK_S)) != NULL)
               ) {
        MMLMenu = false;
      }
      else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_FLOWMENU_L)) != NULL)
               || (strcasestr_P(currentTouchscreenSelection, PSTR(SM_FLOWMENU_S)) != NULL)
               ) {
        SERIAL_ECHOLNPGM("Special Menu: Enter Flow Menu");
        FlowMenu = true;
      }
      else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_FLOW_UP_L)) != NULL)
               || (strcasestr_P(currentTouchscreenSelection, PSTR(SM_FLOW_UP_S)) != NULL)
               ) {
        SERIAL_ECHOLNPGM("Special Menu: Flow UP");
        currentFlowRate = currentFlowRate + 1;

        if (currentFlowRate > 800)
          currentFlowRate = 800;

        char value[30];
        sprintf_P(value, PSTR("M221 S%i"), currentFlowRate);
        queue.enqueue_one_now(value);

      }
      else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_FLOW_DN_L)) != NULL)
               || (strcasestr_P(currentTouchscreenSelection, PSTR(SM_FLOW_DN_S)) != NULL)
               ) {
        SERIAL_ECHOLNPGM("Special Menu: Flow Down");
        currentFlowRate = currentFlowRate - 1;

        if (currentFlowRate < 1)
          currentFlowRate = 1;

        char value[30];
        sprintf_P(value, PSTR("M221 S%i"), currentFlowRate);
        queue.enqueue_one_now(value);
      }
      else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_FLOW_EXIT_L)) != NULL)
               || (strcasestr_P(currentTouchscreenSelection, PSTR(SM_FLOW_EXIT_S)) != NULL)
               ) {
        SERIAL_ECHOLNPGM("Special Menu: Exit Flow Menu");
        FlowMenu = false;
      }

      #if ENABLED(KNUTWURST_BLTOUCH)
        else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_BLTZMENU_L)) != NULL)
                 || (strcasestr_P(currentTouchscreenSelection, PSTR(SM_BLTZMENU_S)) != NULL)
                 ) {
          SERIAL_ECHOLNPGM("Special Menu: Enter BLTouch Menu");
          BLTouchMenu = true;
        }
        else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_BLTZ_UP_L)) != NULL)
                 || (strcasestr_P(currentTouchscreenSelection, PSTR(SM_BLTZ_UP_S)) != NULL)
                 ) {
          SERIAL_ECHOLNPGM("Special Menu: Offset UP");
          probe.offset.z += 0.01F;
        }
        else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_BLTZ_DN_L)) != NULL)
                 || (strcasestr_P(currentTouchscreenSelection, PSTR(SM_BLTZ_DN_S)) != NULL)
                 ) {
          SERIAL_ECHOLNPGM("Special Menu: Offset Down");
          probe.offset.z -= 0.01F;
        }
        else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_BLTZ_EXIT_L)) != NULL)
                 || (strcasestr_P(currentTouchscreenSelection, PSTR(SM_BLTZ_EXIT_S)) != NULL)
                 ) {
          SERIAL_ECHOLNPGM("Special Menu: Exit BLTouch Menu & Save EEPROM");
          settings.save(); // M500
          BUZZ(105, 1108);
          BUZZ(210, 1661);
          BLTouchMenu = false;
        }
      #endif
      else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_EZLVL_MENU_L)) != NULL)
               || (strcasestr_P(currentTouchscreenSelection, PSTR(SM_EZLVL_MENU_S)) != NULL)
               ) {
        SERIAL_ECHOLNPGM("Special Menu: Enter Easy Level Menu");
        LevelMenu = true;
        injectCommands(F("G28\nM420 S0\nG90\nG1 Z5\nG1 X15 Y15 F4000\nG1 Z0"));
      }
      else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_EZLVL_P1_L)) != NULL)
               || (strcasestr_P(currentTouchscreenSelection, PSTR(SM_EZLVL_P1_S)) != NULL)
               ) {
        SERIAL_ECHOLNPGM("Special Menu: Easy Level POINT 1");
        injectCommands(F("G90\nG1 Z5\nG1 X15 Y15 F4000\nG1 Z0"));
      }
      else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_EZLVL_P2_L)) != NULL)
               || (strcasestr_P(currentTouchscreenSelection, PSTR(SM_EZLVL_P2_S)) != NULL)
               ) {
        SERIAL_ECHOLNPGM("Special Menu: Easy Level POINT 2");
        #if ANY(KNUTWURST_MEGA, KNUTWURST_MEGA_S, KNUTWURST_MEGA_P)
          injectCommands(F("G90\nG1 Z5\nG1 X205 Y15 F4000\nG1 Z0"));
        #endif

        #if ENABLED(KNUTWURST_MEGA_X)
          injectCommands(F("G90\nG1 Z5\nG1 X295 Y15 F4000\nG1 Z0"));
        #endif

        #if ENABLED(KNUTWURST_CHIRON)
          injectCommands(F("G90\nG1 Z5\nG1 X385 Y15 F4000\nG1 Z0"));
        #endif

        #if ENABLED(KNUTWURST_4MAXP2)
          injectCommands(F("G90\nG1 Z5\nG1 X255 Y15 F4000\nG1 Z0"));
        #endif
      }
      else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_EZLVL_P3_L)) != NULL)
               || (strcasestr_P(currentTouchscreenSelection, PSTR(SM_EZLVL_P3_S)) != NULL)
               ) {
        SERIAL_ECHOLNPGM("Special Menu: Easy Level POINT 3");
        #if ANY(KNUTWURST_MEGA, KNUTWURST_MEGA_S, KNUTWURST_MEGA_P)
          injectCommands(F("G90\nG1 Z5\nG1 X205 Y200 F4000\nG1 Z0"));
        #endif

        #if ENABLED(KNUTWURST_MEGA_X)
          injectCommands(F("G90\nG1 Z5\nG1 X295 Y295 F4000\nG1 Z0"));
        #endif

        #if ENABLED(KNUTWURST_CHIRON)
          injectCommands(F("G90\nG1 Z5\nG1 X395 Y395 F4000\nG1 Z0"));
        #endif

        #if ENABLED(KNUTWURST_4MAXP2)
          injectCommands(F("G90\nG1 Z5\nG1 X255 Y195 F4000\nG1 Z0"));
        #endif
      }
      else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_EZLVL_P4_L)) != NULL)
               || (strcasestr_P(currentTouchscreenSelection, PSTR(SM_EZLVL_P4_S)) != NULL)
               ) {
        SERIAL_ECHOLNPGM("Special Menu: Easy Level POINT 4");
        #if ANY(KNUTWURST_MEGA, KNUTWURST_MEGA_S, KNUTWURST_MEGA_P)
          injectCommands(F("G90\nG1 Z5\nG1 X15 Y200 F4000\nG1 Z0"));
        #endif

        #if ENABLED(KNUTWURST_MEGA_X)
          injectCommands(F("G90\nG1 Z5\nG1 X15 Y295 F4000\nG1 Z0"));
        #endif

        #if ENABLED(KNUTWURST_CHIRON)
          injectCommands(F("G90\nG1 Z5\nG1 X15 Y395 F4000\nG1 Z0"));
        #endif

        #if ENABLED(KNUTWURST_4MAXP2)
          injectCommands(F("G90\nG1 Z5\nG1 X15 Y195 F4000\nG1 Z0"));
        #endif
      }
      else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_EZLVL_EXIT_L)) != NULL)
               || (strcasestr_P(currentTouchscreenSelection, PSTR(SM_EZLVL_EXIT_S)) != NULL)
               ) {
        SERIAL_ECHOLNPGM("Special Menu: Exit Easy Level Menu");
        LevelMenu = false;
        injectCommands(F("G90\nG1 Z10\nG1 X15 Y15 F4000\nM420 S1"));
      }
    #endif // if ENABLED(KNUTWURST_SPECIAL_MENU)
  }


void AnycubicTouchscreenClass::RenderCurrentFileList() {
  uint16_t selectedNumber = 0;
  FileList currentFileList;
  currentFileOrDirectory[0] = 0;

  if (SpecialMenu == false) {
    currentTouchscreenSelection[0] = 0;
  }
    

  SENDLINE_PGM("FN "); // Filelist start

  if (!isMediaInserted() && !SpecialMenu) {
    SENDLINE_DBG_PGM("J02", "TFT Serial Debug: No SD Card mounted to render Current File List... J02");

    SENDLINE_PGM(SM_SPECIAL_MENU_S);
    SENDLINE_PGM(SM_SPECIAL_MENU_L);
  }
  else {
    if (CodeSeen('S')) {
      selectedNumber = CodeValue();
    }

    if (SpecialMenu) {
      RenderSpecialMenu(selectedNumber);
    } else if (selectedNumber <= currentFileList.count()) {
      RenderCurrentFolder(selectedNumber);
    }
  }
  SENDLINE_PGM("END"); // Filelist stop
}

void AnycubicTouchscreenClass::RenderSpecialMenu(uint16_t selectedNumber) {
  #if ENABLED(KNUTWURST_SPECIAL_MENU)
      if (MMLMenu) {
        switch (selectedNumber) {
          case 0: // Page 1
            SENDLINE_PGM(SM_MESH_START_S);
            SENDLINE_PGM(SM_MESH_START_L);
            SENDLINE_PGM(SM_Z_UP_01_S);
            SENDLINE_PGM(SM_Z_UP_01_L);
            SENDLINE_PGM(SM_Z_DN_01_S);
            SENDLINE_PGM(SM_Z_DN_01_L);
            SENDLINE_PGM(SM_Z_UP_002_S);
            SENDLINE_PGM(SM_Z_UP_002_L);
            break;

          case 4: // Page 2
            SENDLINE_PGM(SM_Z_DN_002_S);
            SENDLINE_PGM(SM_Z_DN_002_L);
            SENDLINE_PGM(SM_Z_UP_001_S);
            SENDLINE_PGM(SM_Z_UP_001_L);
            SENDLINE_PGM(SM_Z_DN_001_S);
            SENDLINE_PGM(SM_Z_DN_001_L);
            SENDLINE_PGM(SM_MESH_NEXT_S);
            SENDLINE_PGM(SM_MESH_NEXT_L);
            break;

          case 8: // Page 2
            SENDLINE_PGM(SM_SAVE_EEPROM_S);
            SENDLINE_PGM(SM_SAVE_EEPROM_L);
            SENDLINE_PGM(SM_BACK_S);
            SENDLINE_PGM(SM_BACK_L);
            break;

          default:
            break;
        }
      }
      else if (FlowMenu) {
        flowRateBuffer = SM_FLOW_DISP_L;
        flowRateBuffer.replace("XXX", String(currentFlowRate));

        switch (selectedNumber) {
          case 0: // Page 1
            SENDLINE_PGM(SM_FLOW_DISP_S);
            SENDLINE(flowRateBuffer.c_str());
            SENDLINE_PGM(SM_FLOW_UP_S);
            SENDLINE_PGM(SM_FLOW_UP_L);
            SENDLINE_PGM(SM_FLOW_DN_S);
            SENDLINE_PGM(SM_FLOW_DN_L);
            SENDLINE_PGM(SM_FLOW_EXIT_S);
            SENDLINE_PGM(SM_FLOW_EXIT_L);
            break;

          default:
            break;
        }
      }
      else if (BLTouchMenu) {
        zOffsetBuffer = SM_BLTZ_DISP_L;

        #ifdef ANYCUBIC_TFT_DEBUG
          SERIAL_ECHOPGM("TFT Serial Debug: Current probe.offset.z: ", float(probe.offset.z));
          SERIAL_EOL();
        #endif

        zOffsetBuffer.replace("XXXXX", String(float(probe.offset.z)));

        switch (selectedNumber) {
          case 0: // Page 1
            SENDLINE_PGM(SM_BLTZ_DISP_S);
            SENDLINE(zOffsetBuffer.c_str());
            SENDLINE_PGM(SM_BLTZ_UP_S);
            SENDLINE_PGM(SM_BLTZ_UP_L);
            SENDLINE_PGM(SM_BLTZ_DN_S);
            SENDLINE_PGM(SM_BLTZ_DN_L);
            SENDLINE_PGM(SM_BLTOUCH_S);
            SENDLINE_PGM(SM_BLTOUCH_L);
            break;

          case 4: // Page 2
            SENDLINE_PGM(SM_BLTZ_EXIT_S);
            SENDLINE_PGM(SM_BLTZ_EXIT_L);
            break;

          default:
            break;
        }
      }
      else if (LevelMenu) {
        switch (selectedNumber) {
          case 0: // Page 1
            SENDLINE_PGM(SM_EZLVL_P1_S);
            SENDLINE_PGM(SM_EZLVL_P1_L);
            SENDLINE_PGM(SM_EZLVL_P2_S);
            SENDLINE_PGM(SM_EZLVL_P2_L);
            SENDLINE_PGM(SM_EZLVL_P3_S);
            SENDLINE_PGM(SM_EZLVL_P3_L);
            SENDLINE_PGM(SM_EZLVL_P4_S);
            SENDLINE_PGM(SM_EZLVL_P4_L);
            break;

          case 4: // Page 2
            SENDLINE_PGM(SM_EZLVL_EXIT_S);
            SENDLINE_PGM(SM_EZLVL_EXIT_L);
            break;

          default:
            break;
        }
      }
      else if (SpecialMenu) {
        switch (selectedNumber) {
          case 0: // Page 1
            SENDLINE_PGM(SM_FLOWMENU_S);
            SENDLINE_PGM(SM_FLOWMENU_L);
            SENDLINE_PGM(SM_PREHEAT_BED_S);
            SENDLINE_PGM(SM_PREHEAT_BED_L);
            SENDLINE_PGM(SM_PAUSE_S);
            SENDLINE_PGM(SM_PAUSE_L);
            SENDLINE_PGM(SM_RESUME_S);
            SENDLINE_PGM(SM_RESUME_L);
            break;

            #if NONE(KNUTWURST_BLTOUCH, KNUTWURST_TFT_LEVELING)
                case 4: // Page 2 for Manual Mesh Bed Level
                  SENDLINE_PGM(SM_EZLVL_MENU_S);
                  SENDLINE_PGM(SM_EZLVL_MENU_L);
                  SENDLINE_PGM(SM_MESH_MENU_S);
                  SENDLINE_PGM(SM_MESH_MENU_L);
                  SENDLINE_PGM(SM_PID_HOTEND_S);
                  SENDLINE_PGM(SM_PID_HOTEND_L);
                  SENDLINE_PGM(SM_PID_BED_S);
                  SENDLINE_PGM(SM_PID_BED_L);
                  break;
            #endif

            #if ENABLED(KNUTWURST_BLTOUCH)
                case 4: // Page 2 for BLTouch
                  SENDLINE_PGM(SM_EZLVL_MENU_S);
                  SENDLINE_PGM(SM_EZLVL_MENU_L);
                  SENDLINE_PGM(SM_BLTZMENU_S);
                  SENDLINE_PGM(SM_BLTZMENU_L);
                  SENDLINE_PGM(SM_PID_HOTEND_S);
                  SENDLINE_PGM(SM_PID_HOTEND_L);
                  SENDLINE_PGM(SM_PID_BED_S);
                  SENDLINE_PGM(SM_PID_BED_L);
                  break;
            #endif

            #if ENABLED(KNUTWURST_TFT_LEVELING)
                case 4: // Page 2 for Chiron ABL
                  SENDLINE_PGM(SM_EZLVL_MENU_S);
                  SENDLINE_PGM(SM_EZLVL_MENU_L);
                  SENDLINE_PGM(SM_RESETLV_S);
                  SENDLINE_PGM(SM_RESETLV_L);
                  SENDLINE_PGM(SM_PID_HOTEND_S);
                  SENDLINE_PGM(SM_PID_HOTEND_L);
                  SENDLINE_PGM(SM_PID_BED_S);
                  SENDLINE_PGM(SM_PID_BED_L);
                  break;
            #endif

          case 8: // Page 3
            SENDLINE_PGM(SM_LOAD_DEFAULTS_S);
            SENDLINE_PGM(SM_LOAD_DEFAULTS_L);
            SENDLINE_PGM(SM_SAVE_EEPROM_S);
            SENDLINE_PGM(SM_SAVE_EEPROM_L);
            SENDLINE_PGM(SM_DIS_FILSENS_S);
            SENDLINE_PGM(SM_DIS_FILSENS_L);
            SENDLINE_PGM(SM_EN_FILSENS_S);
            SENDLINE_PGM(SM_EN_FILSENS_L);
            break;

          case 12: // Page 3
            SENDLINE_PGM(SM_EXIT_S);
            SENDLINE_PGM(SM_EXIT_L);
            break;

          default:
            break;
        }
      }
    #endif // if ENABLED(KNUTWURST_SPECIAL_MENU)
}

void AnycubicTouchscreenClass::RenderCurrentFolder(uint16_t selectedNumber) {
  FileList currentFileList;
  uint16_t count = selectedNumber;
  uint16_t max_files;
  uint16_t filesOnSDCard = currentFileList.count();

  if ((filesOnSDCard - selectedNumber) < 4) {
    max_files = filesOnSDCard;
    #ifdef ANYCUBIC_TFT_DEBUG
      SERIAL_ECHOLN("max_files = filesOnSDCard;");
    #endif
  } else {
    max_files = selectedNumber + 3;
    #ifdef ANYCUBIC_TFT_DEBUG
      SERIAL_ECHOLN("max_files = filenumber + 3;");
    #endif
  }

  if (filesOnSDCard == 3) filenumber = 0;

  for (count = selectedNumber; count <= max_files; count++) {
    if (count == 0) { // Special Entry
      if (currentFileList.isAtRootDir()) {
        SENDLINE_PGM(SM_SPECIAL_MENU_S);
        SENDLINE_PGM(SM_SPECIAL_MENU_L);
        #ifdef ANYCUBIC_TFT_DEBUG
          SERIAL_ECHO(count);
          SERIAL_ECHO(": ");
          SERIAL_ECHOLNPGM(SM_SPECIAL_MENU_L);
        #endif
      }
      else {
        SENDLINE_PGM(SM_DIR_UP_S);
        SENDLINE_PGM(SM_DIR_UP_L);
        #ifdef ANYCUBIC_TFT_DEBUG
          SERIAL_ECHO(count);
          SERIAL_ECHO(": ");
          SERIAL_ECHOLNPGM(SM_DIR_UP_L);
        #endif
      }
    } else {
      currentFileList.seek(count - 1, false);

      #if ENABLED(ANYCUBIC_LCD_DEBUG)
        SERIAL_ECHOLN(currentFileList.filename());
      #endif

      // The longname may not be filed, so we use the built-in fallback here.
      const char* fileName  = currentFileList.filename();
      int fileNameLen = strlen(fileName);
      bool fileNameWasCut = false;

      // Cut off too long filenames.
      // They don't fit on the screen anyway.
      #if ENABLED(KNUTWURST_DGUS2_TFT)
        if (fileNameLen >= MAX_PRINTABLE_FILENAME_LEN) {
          fileNameWasCut = true;
          fileNameLen    = MAX_PRINTABLE_FILENAME_LEN;
        }
        char outputString[MAX_PRINTABLE_FILENAME_LEN];
      #else
        char outputString[fileNameLen];
      #endif

      // Bugfix for non-printable special characters
      // which are now replaced by underscores.
      for (unsigned char i = 0; i <= fileNameLen; i++) {
        if (i >= fileNameLen) {
          outputString[i] = ' ';
        }
        else {
          outputString[i] = fileName[i];
          if (!isPrintable(outputString[i]))
            outputString[i] = '_';
        }
      }

      // I know, it's ugly, but it's faster than a string lib
      #if ENABLED(KNUTWURST_DGUS2_TFT)
        if (fileNameWasCut) {
          outputString[MAX_PRINTABLE_FILENAME_LEN - 7] = '~';
          outputString[MAX_PRINTABLE_FILENAME_LEN - 6] = '.';
          outputString[MAX_PRINTABLE_FILENAME_LEN - 5] = 'g';
          outputString[MAX_PRINTABLE_FILENAME_LEN - 4] = 'c';
          outputString[MAX_PRINTABLE_FILENAME_LEN - 3] = 'o';
          outputString[MAX_PRINTABLE_FILENAME_LEN - 2] = 'd';
          outputString[MAX_PRINTABLE_FILENAME_LEN - 1] = 'e';
          outputString[MAX_PRINTABLE_FILENAME_LEN] = '\0';
        } else if (currentFileList.isDir()) {
            for (unsigned char i = fileNameLen; i < MAX_PRINTABLE_FILENAME_LEN - 7; i++) {
              outputString[i] = ' ';
            }
            outputString[MAX_PRINTABLE_FILENAME_LEN - 7] = '\0';
        } else {
            for (unsigned char i = fileNameLen; i < MAX_PRINTABLE_FILENAME_LEN; i++) {
              outputString[i] = ' ';
            }
            outputString[MAX_PRINTABLE_FILENAME_LEN] = '\0';
        }
      #else
        outputString[fileNameLen] = '\0';
      #endif
    

      if (currentFileList.isDir()) {
        #if ENABLED(KNUTWURST_DGUS2_TFT)
          SEND_PGM("/");
          SEND(currentFileList.shortFilename());
          SENDLINE_PGM(".GCO");
          SEND_PGM("/");
          SEND(outputString);
          SENDLINE_PGM(".gcode");
        #else
          SEND_PGM("/");
          SENDLINE(currentFileList.shortFilename());
          SEND_PGM("/");
          SENDLINE(outputString);
        #endif
        SERIAL_ECHO(count);
        SERIAL_ECHOPGM(": /");
        SERIAL_ECHOLN(outputString);
      } else { // The current selection is a file and not a directory
        SENDLINE(currentFileList.shortFilename());
        SENDLINE(outputString);
        SERIAL_ECHO(count);
        SERIAL_ECHOPGM(": ");
        SERIAL_ECHOLN(outputString);
      }
    }
  }
}

  void AnycubicTouchscreenClass::CheckSDCardChange() {
  #if BOTH(SDSUPPORT, HAS_SD_DETECT)
    bool isInserted = isMediaInserted();
    if (isInserted)
      SENDLINE_DBG_PGM("J00", "TFT Serial Debug: SD card state changed... isInserted");
    else
      SENDLINE_DBG_PGM("J01", "TFT Serial Debug: SD card state changed... !isInserted");

  #endif
  }

  void AnycubicTouchscreenClass::SDCardStateChange(bool isInserted) {
    #if ENABLED(ANYCUBIC_TFT_DEBUG)
      SERIAL_ECHOLNPGM("TFT Serial Debug: SDCardStateChange event triggered...", isInserted);
    #endif
    CheckSDCardChange();
  }

  void AnycubicTouchscreenClass::SDCardError() {
    #if ENABLED(ANYCUBIC_TFT_DEBUG)
      SERIAL_ECHOLNPGM("TFT Serial Debug: SDCardError event triggered...");
    #endif
    SENDLINE_DBG_PGM("J21", "TFT Serial Debug: SD Card Error ... J21");
  }

  void AnycubicTouchscreenClass::CheckHeaterError() {
    if ( (getTargetTemp_celsius((extruder_t)E0) < 5)
      || (getTargetTemp_celsius((extruder_t)E0) > 300)) {
      if (HeaterCheckCount > 600000) {
        HeaterCheckCount = 0;
        SENDLINE_DBG_PGM("J10", "TFT Serial Debug: Hotend temperature abnormal... J10");
      }
      else {
        HeaterCheckCount++;
      }
    }
    else {
      HeaterCheckCount = 0;
    }
  }

  void AnycubicTouchscreenClass::FilamentRunout() {
    #if ENABLED(ANYCUBIC_TFT_DEBUG)
      SERIAL_ECHOLNPGM("TFT Serial Debug: FilamentRunout triggered...");
    #endif
    DoFilamentRunoutCheck();
  }


  void AnycubicTouchscreenClass::DoFilamentRunoutCheck() {
  #if ENABLED(FILAMENT_RUNOUT_SENSOR)
    // NOTE: getFilamentRunoutState() only returns the runout state if the job is printing
    // we want to actually check the status of the pin here, regardless of printstate
    if (READ(FIL_RUNOUT_PIN)) {
      if (mediaPrintingState == AMPRINTSTATE_PRINTING || mediaPrintingState == AMPRINTSTATE_PAUSED || mediaPrintingState == AMPRINTSTATE_PAUSE_REQUESTED) {
        // play tone to indicate filament is out
        injectCommands(F("\nM300 P200 S1567\nM300 P200 S1174\nM300 P200 S1567\nM300 P200 S1174\nM300 P2000 S1567"));

        // tell the user that the filament has run out and wait
        SENDLINE_DBG_PGM("J23", "TFT Serial Debug: Blocking filament prompt... J23");
      }
      else {
        SENDLINE_DBG_PGM("J15", "TFT Serial Debug: Non blocking filament runout... J15");
      }
    }
  #endif // FILAMENT_RUNOUT_SENSOR
}

  void AnycubicTouchscreenClass::UserConfirmRequired(const char * const msg) {
    #if ENABLED(ANYCUBIC_TFT_DEBUG)
      SERIAL_ECHOLNPGM("TFT Serial Debug: OnUserConfirmRequired triggered... ", msg);
    #endif

    #if ENABLED(SDSUPPORT)
      /**
       * Need to handle the process of following states
       * "Nozzle Parked"
       * "Load Filament"
       * "Filament Purging..."
       * "HeaterTimeout"
       * "Reheat finished."
       *
       * NOTE:  The only way to handle these states is strcmp_P with the msg unfortunately (very expensive)
       */
      if (strcmp_P(msg, PSTR("Nozzle Parked")) == 0) {
        mediaPrintingState = AMPRINTSTATE_PAUSED;
        mediaPauseState    = AMPAUSESTATE_PARKED;
        // enable continue button
        SENDLINE_DBG_PGM("J18", "TFT Serial Debug: UserConfirm SD print paused done... J18");
      }
      else if (strcmp_P(msg, PSTR("Load Filament")) == 0) {
        mediaPrintingState = AMPRINTSTATE_PAUSED;
        mediaPauseState    = AMPAUSESTATE_FILAMENT_OUT;
        // enable continue button
        SENDLINE_DBG_PGM("J18", "TFT Serial Debug: UserConfirm Filament is out... J18");
        SENDLINE_DBG_PGM("J23", "TFT Serial Debug: UserConfirm Blocking filament prompt... J23");
      }
      else if (strcmp_P(msg, PSTR("Filament Purging...")) == 0) {
        mediaPrintingState = AMPRINTSTATE_PAUSED;
        mediaPauseState    = AMPAUSESTATE_FILAMENT_PURGING;

        // enable continue button
        SENDLINE_DBG_PGM("J18", "TFT Serial Debug: UserConfirm Filament is purging... J18");
      }
      else if (strcmp_P(msg, PSTR("HeaterTimeout")) == 0) {
        mediaPrintingState = AMPRINTSTATE_PAUSED;
        mediaPauseState    = AMPAUSESTATE_HEATER_TIMEOUT;
        // enable continue button
        SENDLINE_DBG_PGM("J18", "TFT Serial Debug: UserConfirm SD Heater timeout... J18");
      }
      else if (strcmp_P(msg, PSTR("Reheat finished.")) == 0) {
        mediaPrintingState = AMPRINTSTATE_PAUSED;
        mediaPauseState    = AMPAUSESTATE_REHEAT_FINISHED;
        // enable continue button
        SENDLINE_DBG_PGM("J18", "TFT Serial Debug: UserConfirm SD Reheat done... J18");
      }
    #endif
  }

  static boolean TFTcomment_mode = false;

  void AnycubicTouchscreenClass::GetCommandFromTFT() {
    char *starpos = NULL;
    while (LCD_SERIAL.available() > 0  && TFTbuflen < TFTBUFSIZE) {
      serial3_char = LCD_SERIAL.read();
      if (serial3_char == '\n' || serial3_char == '\r' || (serial3_char == ':' && TFTcomment_mode == false) || serial3_count >= (TFT_MAX_CMD_SIZE - 1)) {
        if (!serial3_count) { // if empty line
          TFTcomment_mode = false; // for new command
          return;
        }

        TFTcmdbuffer[TFTbufindw][serial3_count] = 0; // terminate string

        if (!TFTcomment_mode) {
          /*
            // -------- START ERROR CORRECTION ----------
            TFTcomment_mode = false; //for new command
            if (strchr(TFTcmdbuffer[TFTbufindw], 'N') != NULL)
            {
              if (strchr(TFTcmdbuffer[TFTbufindw], '*') != NULL)
              {
                  byte checksum = 0;
                  byte count = 0;
                  while(TFTcmdbuffer[TFTbufindw][count] != '*') checksum = checksum^TFTcmdbuffer[TFTbufindw][count++];
                  TFTstrchr_pointer = strchr(TFTcmdbuffer[TFTbufindw], '*');

                  if ( (int)(strtod(&TFTcmdbuffer[TFTbufindw][TFTstrchr_pointer - TFTcmdbuffer[TFTbufindw] + 1], NULL)) != checksum)
                  {
                      SEND_PGM("ERR ");
                      LCD_SERIAL.flush();
                      SEND_PGM("ERR ");
                      LCD_SERIAL.flush();
                      serial3_count = 0;
                      return;
                  }
                  //if no errors, continue parsing
              } else {
                SEND_PGM("ERR ");
                LCD_SERIAL.flush();
                serial3_count = 0;
                return;
              }
              //if no errors, continue parsing
            } else { // if we don't receive 'N' but still see '*'
                if ((strchr(TFTcmdbuffer[TFTbufindw], '*') != NULL))
                {
                    SEND_PGM("ERR ");
                    serial3_count = 0;
                    return;
                }
            }
            // -------- FINISH ERROR CORRECTION ----------
            */

          if ((strchr(TFTcmdbuffer[TFTbufindw], 'A') != NULL)) {
            TFTstrchr_pointer = strchr(TFTcmdbuffer[TFTbufindw], 'A');
            switch ((int)((strtod(&TFTcmdbuffer[TFTbufindw][TFTstrchr_pointer - TFTcmdbuffer[TFTbufindw] + 1], NULL)))) {
              case 0: // A0 GET HOTEND TEMP
                SEND_PGM_VAL("A0V ", int(getActualTemp_celsius(E0) + 0.5));
                break;

              case 1: // A1  GET HOTEND TARGET TEMP
                SEND_PGM_VAL("A1V ", int(getTargetTemp_celsius(E0) + 0.5));
                break;

              case 2: // A2 GET HOTBED TEMP
                SEND_PGM_VAL("A2V ", int(getActualTemp_celsius(BED) + 0.5));
                break;

              case 3: // A3 GET HOTBED TARGET TEMP
                SEND_PGM_VAL("A3V ", int(getTargetTemp_celsius(BED) + 0.5));
                break;

              case 4: // A4 GET FAN SPEED
                SEND_PGM_VAL("A4V ", int(getActualFan_percent(FAN0)));
              break;
              case 5: // A5 GET CURRENT COORDINATE
                SEND_PGM("A5V X: "); LCD_SERIAL.print(current_position[X_AXIS]);
                SEND_PGM(   " Y: "); LCD_SERIAL.print(current_position[Y_AXIS]);
                SEND_PGM(   " Z: "); LCD_SERIAL.print(current_position[Z_AXIS]);
                SENDLINE_PGM("");
                break;

              case 6: // A6 GET SD CARD PRINTING STATUS
                #ifdef SDSUPPORT
                  if (isPrintingFromMedia())
                  {
                    SEND_PGM("A6V ");
                    if (isMediaInserted())
                      SENDLINE(ui8tostr3rj(getProgress_percent()));
                    else
                      SENDLINE_DBG_PGM("J02", "TFT Serial Debug: No SD Card mounted to return printing status... J02");
                  }
                  else
                    SENDLINE_PGM("A6V ---");
                #endif
                break;
              case 7: // A7 GET PRINTING TIME
              {
                const uint32_t elapsedSeconds = getProgress_seconds_elapsed();
                SEND_PGM("A7V ");
                if (elapsedSeconds != 0) {  // print time
                  const uint32_t elapsedMinutes = elapsedSeconds / 60;
                  SEND(ui8tostr2(elapsedMinutes / 60));
                  SEND_PGM(" H ");
                  SEND(ui8tostr2(elapsedMinutes % 60));
                  SENDLINE_PGM(" M");
                }
                else
                  SENDLINE_PGM(" 999:999");
              }
              break;

              case 8: // A8 GET SD LIST
                #ifdef SDSUPPORT
                  if (SpecialMenu == false)
                    currentTouchscreenSelection[0] = 0;

                  #if DISABLED(KNUTWURST_SPECIAL_MENU_WO_SD)
                    if (!IS_SD_INSERTED()) {
                      SENDLINE_DBG_PGM("J02", "TFT Serial Debug: No SD Card mounted to render Current File List... J02");
                    }
                    else
                  #endif
                  {
                    if (CodeSeen('S')) filenumber = CodeValue();
                    //PrintList();
                    RenderCurrentFileList();
                  }
                #endif
                break;

               case 9: // A9 pause sd print
                #if ENABLED(SDSUPPORT)
                  if (isPrintingFromMedia())
                    PausePrint();
                #endif
                break;

              case 10: // A10 resume sd print
                #if ENABLED(SDSUPPORT)
                  if (isPrintingFromMediaPaused())
                    ResumePrint();
                #endif
                break;

              case 11: // A11 stop sd print
                TERN_(SDSUPPORT, StopPrint());
                break;

              case 12: // A12 kill
                kill(F(STR_ERR_KILLED));
                break;

              case 13: // A13 SELECTION FILE
                #if ENABLED(SDSUPPORT)
                  {
                    starpos = (strchr(TFTstrchr_pointer + 4, '*'));
                    if (TFTstrchr_pointer[4] == '/') {
                      strcpy(currentTouchscreenSelection, TFTstrchr_pointer + 5);
                      currentFileOrDirectory[0] = 0;
                      SENDLINE_DBG_PGM("J21", "TFT Serial Debug: Clear file selection... J21 "); // J21 Not File Selected
                      SENDLINE_PGM("");
                    }
                    else if (TFTstrchr_pointer[4] == '<') {
                      strcpy(currentTouchscreenSelection, TFTstrchr_pointer + 4);
                      SpecialMenu = true;
                      currentFileOrDirectory[0] = 0;
                      SENDLINE_DBG_PGM("J21", "TFT Serial Debug: Clear file selection... J21 "); // J21 Not File Selected
                      SENDLINE_PGM("");
                    }
                    else {
                      currentTouchscreenSelection[0] = 0;

                      if (starpos) *(starpos - 1) = '\0';

                      strcpy(currentFileOrDirectory, TFTstrchr_pointer + 4);
                      SENDLINE_DBG_PGM_VAL("J20", "TFT Serial Debug: File Selected... J20 ", currentFileOrDirectory); // J20 File Selected
                    }
                  }
                #endif
                break;

              case 14: // A14 START PRINTING
                #if ENABLED(SDSUPPORT)
                  if (!isPrinting())
                    StartPrint();
                #endif

              case 15: // A15 RESUMING FROM OUTAGE
                #if defined(POWER_OUTAGE_TEST)
                  if ((!planner.movesplanned()) && (TFTstate != ANYCUBIC_TFT_STATE_SDPAUSE)) {
                    if (card.isFileOpen()) FlagResumFromOutage = true;

                    ResumingFlag = 1;
                    card.startOrResumeFilePrinting();
                    starttime = millis();
                    SENDLINE_PGM("OK");
                  }
                #endif
                break;

              case 16: // A16 set hotend temp
              {
                unsigned int tempvalue;
                if (CodeSeen('S')) {
                  tempvalue = constrain(CodeValue(), 0, 260);
                  if (getTargetTemp_celsius((extruder_t)E0) <= 260)
                    setTargetTemp_celsius(tempvalue, (extruder_t)E0);; // do not set Temp from TFT if it is set via gcode
                }
                else if ((CodeSeen('C')) && (!isPrinting())) {
                  if ((getAxisPosition_mm(Z) < 10))
                    injectCommands(F("G1 Z10")); // RASE Z AXIS
                  tempvalue = constrain(CodeValue(), 0, 260);
                  setTargetTemp_celsius(tempvalue, (extruder_t)E0);
                }
              }
              break;

              case 17: // A17 set heated bed temp
              {
                unsigned int tempbed;
                if (CodeSeen('S')) {
                  tempbed = constrain(CodeValue(), 0, 120);
                  if (getTargetTemp_celsius((heater_t)BED) <= 100)
                    setTargetTemp_celsius(tempbed, (heater_t)BED); // do not set Temp from TFT if it is set via gcode
                }
              }
              break;

              case 18: // A18 set fan speed
                float fanPercent;
                if (CodeSeen('S')) {
                  fanPercent = CodeValue();
                  fanPercent = constrain(fanPercent, 0, 100);
                  setTargetFan_percent(fanPercent, FAN0);
                }
                else {
                  fanPercent = 100;
                  setTargetFan_percent(fanPercent, FAN0);
                }
                SENDLINE_PGM("");
                break;

              case 19: // A19 stop stepper drivers
                if (!isPrinting()) {
                  quickstop_stepper();
                  stepper.disable_all_steppers();
                }

                SENDLINE_PGM("");
                break;

              case 20: // A20 read printing speed
              {
                if (CodeSeen('S'))
                  feedrate_percentage = constrain(CodeValue(), 40, 999);
                else
                  SEND_PGM_VAL("A20V ", feedrate_percentage);
              }
              break;

              case 21: // A21 all home
                if (!isPrinting() && !isPrintingFromMediaPaused()) {
                  if (CodeSeen('X') || CodeSeen('Y') || CodeSeen('Z')) {
                    if (CodeSeen('X'))
                      injectCommands(F("G28X"));
                    if (CodeSeen('Y'))
                      injectCommands(F("G28Y"));
                    if (CodeSeen('Z'))
                      injectCommands(F("G28Z"));
                  }
                  else if (CodeSeen('C')) {
                    injectCommands_P(G28_STR);
                  }
                }
                break;

              case 22: // A22 move X/Y/Z or extrude
                if (!isPrinting()) {
                  float coorvalue;
                  unsigned int movespeed = 0;
                  char value[30];
                  if (CodeSeen('F')) // Set feedrate
                    movespeed = CodeValue();

                  queue.enqueue_now_P(PSTR("G91")); // relative coordinates

                  if (CodeSeen('X')) { // Move in X direction
                    coorvalue = CodeValue();
                    if ((coorvalue <= 0.2) && coorvalue > 0)
                      sprintf_P(value, PSTR("G1 X0.1F%i"), movespeed);
                    else if ((coorvalue <= -0.1) && coorvalue > -1)
                      sprintf_P(value, PSTR("G1 X-0.1F%i"), movespeed);
                    else
                      sprintf_P(value, PSTR("G1 X%iF%i"), int(coorvalue), movespeed);
                    queue.enqueue_one_now(value);
                  }
                  else if (CodeSeen('Y')) { // Move in Y direction
                    coorvalue = CodeValue();
                    if ((coorvalue <= 0.2) && coorvalue > 0)
                      sprintf_P(value, PSTR("G1 Y0.1F%i"), movespeed);
                    else if ((coorvalue <= -0.1) && coorvalue > -1)
                      sprintf_P(value, PSTR("G1 Y-0.1F%i"), movespeed);
                    else
                      sprintf_P(value, PSTR("G1 Y%iF%i"), int(coorvalue), movespeed);
                    queue.enqueue_one_now(value);
                  }
                  else if (CodeSeen('Z')) { // Move in Z direction
                    coorvalue = CodeValue();
                    if ((coorvalue <= 0.2) && coorvalue > 0)
                      sprintf_P(value, PSTR("G1 Z0.1F%i"), movespeed);
                    else if ((coorvalue <= -0.1) && coorvalue > -1)
                      sprintf_P(value, PSTR("G1 Z-0.1F%i"), movespeed);
                    else
                      sprintf_P(value, PSTR("G1 Z%iF%i"), int(coorvalue), movespeed);
                    queue.enqueue_one_now(value);
                  }
                  else if (CodeSeen('E')) { // Extrude
                    coorvalue = CodeValue();
                    if ((coorvalue <= 0.2) && coorvalue > 0)
                      sprintf_P(value, PSTR("G1 E0.1F%i"), movespeed);
                    else if ((coorvalue <= -0.1) && coorvalue > -1)
                      sprintf_P(value, PSTR("G1 E-0.1F%i"), movespeed);
                    else
                      sprintf_P(value, PSTR("G1 E%iF500"), int(coorvalue));
                    queue.enqueue_one_now(value);
                  }
                  queue.enqueue_now_P(PSTR("G90")); // absolute coordinates
                }
                SENDLINE_PGM("");
                break;

              case 23: // A23 preheat pla
                if (!isPrinting()) {
                  if (getAxisPosition_mm(Z) < 10)
                    injectCommands(F("G1 Z10")); // RASE Z AXIS

                  setTargetTemp_celsius(PREHEAT_1_TEMP_BED, (heater_t)BED);
                  setTargetTemp_celsius(PREHEAT_1_TEMP_HOTEND, (extruder_t)E0);
                  SENDLINE_PGM("OK");
                }
                break;

              case 24: // A24 preheat abs
                if (!isPrinting()) {
                  if (getAxisPosition_mm(Z) < 10)
                    injectCommands(F("G1 Z10")); // RASE Z AXIS

                  setTargetTemp_celsius(PREHEAT_2_TEMP_BED, (heater_t)BED);
                  setTargetTemp_celsius(PREHEAT_2_TEMP_HOTEND, (extruder_t)E0);
                  SENDLINE_PGM("OK");
                }
                break;

              case 25: // A25 cool down
                if (!isPrinting()) {
                  setTargetTemp_celsius(0, (heater_t) BED);
                  setTargetTemp_celsius(0, (extruder_t) E0);

                  SENDLINE_DBG_PGM("J12", "TFT Serial Debug: Cooling down... J12"); // J12 cool down
                }
                break;

              case 26: // A26 refresh SD
              {
                #ifdef SDSUPPORT
                    #ifdef ANYCUBIC_TFT_DEBUG
                      SERIAL_ECHOPGM("TFT Serial Debug: RefreshSD(): currentTouchscreenSelection: ", currentTouchscreenSelection);
                      SERIAL_EOL();
                      SERIAL_ECHOPGM("TFT Serial Debug: RefreshSD(): currentFileOrDirectory: ", currentFileOrDirectory);
                      SERIAL_EOL();
                    #endif

                    FileList currentFileList;
                    if ((strcasestr_P(currentFileOrDirectory, PSTR(SM_DIR_UP_S)) != NULL)
                        ||  (strcasestr_P(currentFileOrDirectory, PSTR(SM_DIR_UP_L)) != NULL)
                        ) {
                      #ifdef ANYCUBIC_TFT_DEBUG
                        SERIAL_ECHOLNPGM("TFT Serial Debug: Directory UP (cd ..)");
                      #endif
                      currentFileList.upDir();
                    }
                    else {
                      if (currentTouchscreenSelection[0] == '<') {
                        #ifdef ANYCUBIC_TFT_DEBUG
                          SERIAL_ECHOLNPGM("TFT Serial Debug: Enter Special Menu");
                        #endif
                        HandleSpecialMenu();
                      }
                      else {
                        #ifdef ANYCUBIC_TFT_DEBUG
                          SERIAL_ECHOLNPGM("TFT Serial Debug: Not a menu. Must be a directory!");
                        #endif

                        #if ENABLED(KNUTWURST_DGUS2_TFT)
                          strcpy(currentFileOrDirectory, currentTouchscreenSelection);
                          int currentFileLen = strlen(currentFileOrDirectory);
                          currentFileOrDirectory[currentFileLen - 4] = '\0';
                          currentFileList.changeDir(currentFileOrDirectory);
                        #else
                          currentFileList.changeDir(currentTouchscreenSelection);
                        #endif
                      }
                    }
                  }
                  if (SpecialMenu == false) {
                    currentTouchscreenSelection[0] = 0;
                  }

                #endif // ifdef SDSUPPORT
                break;

              case 28: // A28 filament test
              {
                if (CodeSeen('O'))
                  ;
                else if (CodeSeen('C'))
                  ;
              }
                SENDLINE_PGM("");
                break;

                #if DISABLED(KNUTWURST_TFT_LEVELING)
                    case 33: // A33 get version info
                      SEND_PGM("J33 ");
                      SEND_PGM("KW-");
                      SENDLINE_PGM(MSG_MY_VERSION);
                      break;
                #endif
                #if ENABLED(KNUTWURST_TFT_LEVELING)
                    case 29: // A29 bed grid read
                    {
                      int mx, my;

                      if (CodeSeen('X')) mx = CodeValueInt();
                      if (CodeSeen('Y')) my = CodeValueInt();

                      float Zvalue = bedlevel.z_values[mx][my];
                      Zvalue = Zvalue * 100;

                      if (!isPrinting()) {
                        if (!all_axes_trusted()) {
                          injectCommands(F("G28\n"));
                          /*
                             set_axis_is_at_home(X_AXIS);
                             sync_plan_position();
                             set_axis_is_at_home(Y_AXIS);
                             sync_plan_position();
                             set_axis_is_at_home(Z_AXIS);
                             sync_plan_position();
                             report_current_position();
                          */
                        }
                        else {
                          // Go up before moving
                          // SERIAL_ECHOLNPGM("Z Up");
                          setAxisPosition_mm(5.0, Z);
                          // report_current_position();
                          setAxisPosition_mm(LevelingBilinear::get_mesh_x(mx), X);
                          // report_current_position();
                          setAxisPosition_mm(LevelingBilinear::get_mesh_y(my), Y);
                          // report_current_position();
                          setAxisPosition_mm(EXT_LEVEL_HIGH, Z);

                          report_current_position();
                        }
                      }
                      SEND_PGM("A29V ");
                      LCD_SERIAL.print(Zvalue, 2);
                      SENDLINE_PGM("");
                    }
                    break;

                    case 30: // A30 auto leveling (Old Anycubic TFT)
                      if (isPrinting())
                        SENDLINE_DBG_PGM("J24", "TFT Serial Debug: Forbid auto leveling... J24");
                      else
                        SENDLINE_DBG_PGM("J26", "TFT Serial Debug: Start auto leveling... J26");
                      if (CodeSeen('S'))
                        queue.enqueue_now_P(PSTR("G28\nG29\nM500\nG90\nM300 S440 P200\nM300 S660 P250\nM300 S880 P300\nG1 Z30 F4000\nG1 X0 F4000\nG91\nM84"));
                      break;

                    case 31: // A31 z-offset
                      if (CodeSeen('S')) { // set
                        // soft_endstops_enabled = false;  // disable endstops
                        float value = constrain(CodeValue(), -1.0, 1.0);
                        probe.offset.z += value;
                        for (x = 0; x < GRID_MAX_POINTS_X; x++)
                          for (y = 0; y < GRID_MAX_POINTS_Y; y++) bedlevel.z_values[x][y] += value;
                        set_bed_leveling_enabled(true);
                        bedlevel.refresh_bed_level();

                        SEND_PGM("A31V ");
                        LCD_SERIAL.print(float(probe.offset.z), 2);
                        SENDLINE_PGM("");
                      }

                      if (CodeSeen('G')) { // get
                        SAVE_zprobe_zoffset = probe.offset.z;
                        SEND_PGM("A31V ");
                        LCD_SERIAL.print(float(SAVE_zprobe_zoffset), 2);
                        SENDLINE_PGM("");
                      }

                      if (CodeSeen('D')) { // save
                        SAVE_zprobe_zoffset = probe.offset.z;
                        settings.save();
                        set_bed_leveling_enabled(true);
                        bedlevel.refresh_bed_level();
                      }
                      SENDLINE_PGM("");
                      break;

                    case 32: // a32 clean leveling beep flag
                      break;

                    case 33: // A33 get version info
                      SEND_PGM("J33 ");
                      SEND_PGM("KW-");
                      SENDLINE_PGM(MSG_MY_VERSION);
                      break;

                    case 34: // a34 bed grid write
                    {
                      if (CodeSeen('X')) x = constrain(CodeValueInt(), 0, GRID_MAX_POINTS_X);
                      if (CodeSeen('Y')) y = constrain(CodeValueInt(), 0, GRID_MAX_POINTS_Y);

                      if (CodeSeen('V')) {
                        float new_z_value = float(constrain(CodeValue() / 100, -10, 10));
                        bedlevel.z_values[x][y] = new_z_value;
                        set_bed_leveling_enabled(true);
                        bedlevel.refresh_bed_level();
                      }
                      if (CodeSeen('S')) {
                        bedlevel.refresh_bed_level();
                        set_bed_leveling_enabled(true);
                        settings.save();
                      }
                      if (CodeSeen('C')) {
                        restore_z_values();
                        probe.offset.z = SAVE_zprobe_zoffset;
                        set_bed_leveling_enabled(true);
                        bedlevel.refresh_bed_level();
                      }
                    }
                    break;

                    case 35: // RESET AUTOBED DATE //M1000
                      // initializeGrid();  //done via special menu
                      break;

                    case 36: // A36 auto leveling (New Anycubic TFT)
                      if (isPrinting())
                        SENDLINE_DBG_PGM("J24", "TFT Serial Debug: Forbid auto leveling... J24");
                      else
                        SENDLINE_DBG_PGM("J26", "TFT Serial Debug: Start auto leveling... J26");
                      if (CodeSeen('S'))
                        queue.enqueue_now_P(PSTR("G28\nG29\nM500\nG90\nM300 S440 P200\nM300 S660 P250\nM300 S880 P300\nG1 Z30 F4000\nG1 X0 F4000\nG91\nM84"));

                #endif // if ENABLED(KNUTWURST_TFT_LEVELING)

                #if ENABLED(KNUTWURST_4MAXP2)
                    case 40:
                      SENDLINE_DBG_PGM("J17", "TFT Serial Debug: Main board reset... J17"); // J17 Main board reset
                      delay(10);
                      break;

                    case 41:
                      if (CodeSeen('O')) {
                        PrintdoneAndPowerOFF = true;
                        break;
                      }
                      else if (CodeSeen('C')) {
                        PrintdoneAndPowerOFF = false;
                        break;
                      }
                      if (CodeSeen('S')) {
                        if (PrintdoneAndPowerOFF)
                          SENDLINE_PGM("J35 ");
                        else
                          SENDLINE_PGM("J34 ");
                      }
                      // break; <-- TODO: do we need it?
                    case 42:
                      if (CaseLight == true) {
                        SERIAL_ECHOLNPGM("Case Light OFF");
                        injectCommands(F("M355 S1 P0"));
                        CaseLight = false;
                      }
                      else {
                        SERIAL_ECHOLNPGM("Case Light ON");
                        injectCommands(F("M355 S1 P255"));
                        CaseLight = true;
                      }
                      // break; <-- TODO: do we need it?
                #endif
                #if ENABLED(KNUTWURST_DGUS2_TFT)
                    case 50:
                      SENDLINE_PGM("J38 ");
                      break;
                #endif

                #if ENABLED(KNUTWURST_MEGA_P_LASER)
                    case 34:// Continuous printing
                      en_continue = 1;
                      break;

                    case 35:// Continuous printing
                      en_continue = 0;
                      break;
                    
                    case 36:
                      if (CodeSeen('S')) {
                        int coorvalue;
                        coorvalue = CodeValueInt();
                        if (coorvalue != 0)
                          Laser_printer_st.pic_vector = 1;
                        else
                          Laser_printer_st.pic_vector = 0;
                        break;
                          case 37:
                            if (CodeSeen('S')) {
                              int coorvalue;
                              coorvalue = CodeValueInt();
                              if (coorvalue == 0)
                                Laser_printer_st.pic_x_mirror = 0;
                              else if (coorvalue == 1)
                                Laser_printer_st.pic_x_mirror = 1; // x
                            }
                            break;

                          case 38:
                            if (CodeSeen('S')) {
                              int coorvalue;
                              coorvalue                       = CodeValueInt();
                              Laser_printer_st.pic_laser_time = coorvalue;
                            }
                            break;

                          case 39:
                            if (CodeSeen('S')) { // A39
                              float coorvalue;
                              coorvalue                     = CodeValue();
                              Laser_printer_st.laser_height = coorvalue;
                              SEND_PGM("laser_height = ");
                              LCD_SERIAL.print(Laser_printer_st.laser_height);
                              SENDLINE_PGM("");
                            }
                            break;

                          case 40:
                            if (CodeSeen('S')) { // A40
                              float coorvalue;
                              coorvalue                           = CodeValue();
                              Laser_printer_st.pic_pixel_distance = coorvalue;
                            }
                            break;

                          case 41:
                            if (CodeSeen('S')) {
                              float coorvalue;
                              coorvalue                 = CodeValue();
                              Laser_printer_st.x_offset = coorvalue;
                            }
                            break;

                          case 42:
                            if (CodeSeen('S')) {
                              float coorvalue;
                              coorvalue                 = CodeValue();
                              Laser_printer_st.y_offset = coorvalue;
                            }
                            break;

                          case 43:
                            if (CodeSeen('S')) { // y
                              int coorvalue;
                              coorvalue = CodeValueInt();
                              if (coorvalue == 0)
                                Laser_printer_st.pic_y_mirror = 0;
                              else if (coorvalue == 1)
                                Laser_printer_st.pic_y_mirror = 1;
                            }
                            break;

                          case 44:
                            send_laser_param();
                            break;

                          case 49: // A49
                            laser_on_off = 0;
                            WRITE(HEATER_0_PIN, 0);
                            break;

                          case 50: // A50
                            if (laser_on_off == 0) {
                              laser_on_off = 1;
                            }
                            else {
                              laser_on_off = 0;
                              WRITE(HEATER_0_PIN, 0);
                            }
                            break;
                #endif // if ENABLED(KNUTWURST_MEGA_P_LASER)

                #if ENABLED(KNUTWURST_MEGA_P)
                    case 51:
                      if (CodeSeen('H')) {
                        queue.enqueue_now_P(PSTR("G1 Z5 F500"));
                        queue.enqueue_now_P(PSTR("G1 X30 Y30 F5000"));
                        queue.enqueue_now_P(PSTR("G1 Z0.15 F300"));
                      }
                      else if (CodeSeen('I')) {
                        queue.enqueue_now_P(PSTR("G1 Z5 F500"));
                        queue.enqueue_now_P(PSTR("G1 X190 Y30 F5000"));
                        queue.enqueue_now_P(PSTR("G1 Z0.15 F300"));
                      }
                      else if (CodeSeen('J')) {
                        queue.enqueue_now_P(PSTR("G1 Z5 F500"));
                        queue.enqueue_now_P(PSTR("G1 X190 Y190 F5000"));
                        queue.enqueue_now_P(PSTR("G1 Z0.15 F300"));
                      }
                      else if (CodeSeen('K')) {
                        queue.enqueue_now_P(PSTR("G1 Z5 F500"));
                        queue.enqueue_now_P(PSTR("G1 X30 Y190 F5000"));
                        queue.enqueue_now_P(PSTR("G1 Z0.15 F300"));
                      }
                      else if (CodeSeen('L')) {
                        queue.enqueue_now_P(PSTR("G1 X100 Y100  Z50 F5000"));
                      }
                      break;
                #endif

              default:
                break;
            }
          }
          TFTbufindw = (TFTbufindw + 1) % TFTBUFSIZE;
          TFTbuflen += 1;
        }
        serial3_count = 0; // clear buffer
      }
      else {
        if (serial3_char == ';') TFTcomment_mode = true;
        if (!TFTcomment_mode) TFTcmdbuffer[TFTbufindw][serial3_count++] = serial3_char;
      }
    }
  }

  #if ENABLED(KNUTWURST_MEGA_P_LASER)
    void prepare_laser_print() {
      static unsigned long times = 0;

      if (times > 100) {
        times--;
        return;
      }
      times = 10000;

      if (laser_print_steps == 0) {
        cvalue[0] = 0;
        while (planner.blocks_queued());
        queue.enqueue_now_P(PSTR("G28"));
        sprintf_P(cvalue, PSTR("G1 Z%i F500"), (int)Laser_printer_st.laser_height);

        SERIAL_PROTOCOLLN(cvalue);
        enqueue_and_echo_command_now(cvalue);
        laser_print_steps = 1;
        times             = 120000;
      }
      else if (laser_print_steps == 1) {
        if (planner.blocks_queued()) return;
        laser_print_steps = 2;
      }
      else if (laser_print_steps == 2) {
        Laset_print_picture( );
        laser_print_steps = 0;
        card.printingHasFinished();
        card.checkautostart(true);
        en_continue = 0;
      }
    }
  #endif

  #if ENABLED(KNUTWURST_4MAXP2)
    void PowerDown() {
      for (unsigned char i = 0; i < 3; i++) {
        WRITE(POWER_OFF_PIN, LOW);
        delay(10);
        WRITE(POWER_OFF_PIN, HIGH);
        delay(10);
      }
    }
  #endif
  

  void AnycubicTouchscreenClass::CommandScan() {
  static millis_t nextStopCheck = 0; // used to slow the stopped print check down to reasonable times
    const millis_t ms = millis();
    if (ELAPSED(ms, nextStopCheck)) {
      nextStopCheck = ms + 1000UL;
      if (mediaPrintingState == AMPRINTSTATE_STOP_REQUESTED) {
        #if ENABLED(ANYCUBIC_TFT_DEBUG)
          SERIAL_ECHOLNPGM("TFT Serial Debug: Finished stopping print, releasing motors ...");
        #endif
        mediaPrintingState = AMPRINTSTATE_NOT_PRINTING;
        mediaPauseState = AMPAUSESTATE_NOT_PAUSED;
        injectCommands(F("M84\nM27")); // disable stepper motors and force report of SD status
        delay_ms(200);
        // tell printer to release resources of print to indicate it is done
        SENDLINE_DBG_PGM("J14", "TFT Serial Debug: SD Print Stopped... J14");
      }
    }

    if (TFTbuflen < (TFTBUFSIZE - 1))
      GetCommandFromTFT();

    if (TFTbuflen) {
      TFTbuflen  = (TFTbuflen - 1);
      TFTbufindr = (TFTbufindr + 1) % TFTBUFSIZE;
    }
  }

  void AnycubicTouchscreenClass::OnPrintTimerStarted() {
    #if ENABLED(SDSUPPORT)
      if (mediaPrintingState == AMPRINTSTATE_PRINTING)
        SENDLINE_DBG_PGM("J04", "TFT Serial Debug: Starting SD Print... J04"); // J04 Starting Print

    #endif
  }

  void AnycubicTouchscreenClass::OnPrintTimerPaused() {
    #if ENABLED(SDSUPPORT)
      if (isPrintingFromMedia()) {
        mediaPrintingState = AMPRINTSTATE_PAUSED;
        mediaPauseState    = AMPAUSESTATE_PARKING;
      }
    #endif
  }

  void AnycubicTouchscreenClass::OnPrintTimerStopped() {
    #if ENABLED(SDSUPPORT)
      if (mediaPrintingState == AMPRINTSTATE_PRINTING) {
        mediaPrintingState = AMPRINTSTATE_NOT_PRINTING;
        mediaPauseState    = AMPAUSESTATE_NOT_PAUSED;
        SENDLINE_DBG_PGM("J14", "TFT Serial Debug: SD Print Completed... J14");
      }
      // otherwise it was stopped by the printer so don't send print completed signal to TFT
    #endif
  }

  #if BOTH(ANYCUBIC_TFT_DEBUG, KNUTWURST_DGUS2_TFT)
    void AnycubicTouchscreenClass::Command(const char * const command) {
      SENDLINE(command);
      #if ENABLED(ANYCUBIC_TFT_DEBUG)
        SERIAL_ECHOPGM("TFT Serial Debug: Sending ");
        SERIAL_ECHO(strlen(command));
        SERIAL_ECHOPGM(" ");
        SERIAL_ECHOLN(command);
      #endif
    }
  #endif

  void PowerKill() {
    #ifdef POWER_OUTAGE_TEST
      Temp_Buf_Extuder_Temperature = thermalManager.degTargetHotend(0);
      Temp_Buf_Bed_Temperature     = thermalManager.degTargetBed();
      if (PowerTestFlag == true) {
        thermalManager.disable_all_heaters();
        OutageSave();
        PowerTestFlag = false;
        thermalManager.setTargetHotend(Temp_Buf_Extuder_Temperature, 0);
        thermalManager.setTargetBed(Temp_Buf_Bed_Temperature);
      }
    #endif
  }

  #if ENABLED(KNUTWURST_TFT_LEVELING)
    void AnycubicTouchscreenClass::LevelingDone() {
      SENDLINE_DBG_PGM("J25", "TFT Serial Debug: Auto leveling done... J25");
    }
  #endif

  AnycubicTouchscreenClass AnycubicTouchscreen;
#endif // ifdef ANYCUBIC_TOUCHSCREEN
