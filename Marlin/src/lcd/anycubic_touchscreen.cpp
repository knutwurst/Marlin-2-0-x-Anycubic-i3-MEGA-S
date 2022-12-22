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

#include "../gcode/queue.h"
#include "../gcode/parser.h"
#include "../feature/e_parser.h"
#include "../feature/pause.h"
#include "../feature/bedlevel/bedlevel.h"
#include "../libs/buzzer.h"
#include "../libs/numtostr.h"
#include "../module/planner.h"
#include "../module/printcounter.h"
#include "../module/temperature.h"
#include "../module/motion.h"
#include "../module/probe.h"
#include "../module/settings.h"
#include "../module/stepper.h"
#include "../sd/cardreader.h"

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

    void setAxisPosition_mm(const float position, const axis_t axis, const feedRate_t feedrate /*=0*/) {
      // Get motion limit from software endstops, if any
      float min, max;
      // max = soft_endstop.max[axis];
      // min = soft_endstop.min[axis];
      soft_endstop.get_manual_axis_limits((AxisEnum)axis, min, max);

      current_position[axis] = constrain(position, min, max);
      line_to_current_position(feedrate ?: 60);
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

  char *itostr2(const uint8_t &x) {
    int xx = x;
    _conv[0] = (xx / 10) % 10 + '0';
    _conv[1] = (xx) % 10 + '0';
    _conv[2] = 0;
    return _conv;
  }

  #ifndef ULTRA_LCD
    #define DIGIT(n) ('0' + (n))
    #define DIGIMOD(n, f) DIGIT((n) / (f) % 10)
    #define RJDIGIT(n, f) ((n) >= (f) ? DIGIMOD(n, f) : ' ')
    #define MINUSOR(n, alt) (n >= 0 ? (alt) : (n = -n, '-'))

    char *itostr3(const int x) {
      int xx = x;
      _conv[4] = MINUSOR(xx, RJDIGIT(xx, 100));
      _conv[5] = RJDIGIT(xx, 10);
      _conv[6] = DIGIMOD(xx, 1);
      return &_conv[4];
    }

    // Convert signed float to fixed-length string with 023.45 / -23.45 format

    char *ftostr32(const float &x) {
      long xx = x * 100;
      _conv[1] = MINUSOR(xx, DIGIMOD(xx, 10000));
      _conv[2] = DIGIMOD(xx, 1000);
      _conv[3] = DIGIMOD(xx, 100);
      _conv[4] = '.';
      _conv[5] = DIGIMOD(xx, 10);
      _conv[6] = DIGIMOD(xx, 1);
      return &_conv[1];
    }
  #endif

  AnycubicTouchscreenClass::AnycubicTouchscreenClass() {
  }

  void AnycubicTouchscreenClass::Setup() {
    #ifndef LCD_BAUDRATE
      #define LCD_BAUDRATE 115200
    #endif
    LCD_SERIAL.begin(LCD_BAUDRATE);

    #if DISABLED(KNUTWURST_4MAXP2)
      SENDLINE_PGM("");
      SENDLINE_PGM("J17"); // J17 Main board reset
      delay(10);
    #endif

    SENDLINE_DBG_PGM("J12", "TFT Serial Debug: Ready... J12"); // J12 Ready

    TFTstate = ANYCUBIC_TFT_STATE_IDLE;

    currentTouchscreenSelection[0] = 0;
    currentFileOrDirectory[0]      = '\0';
    SpecialMenu                    = false;
    MMLMenu                        = false;
    FlowMenu                       = false;
    BLTouchMenu                    = false;
    LevelMenu                      = false;
    CaseLight                      = false;
    FilamentSensorEnabled          = true;
    MyFileNrCnt                    = 0;
    currentFlowRate                = 100;
    flowRateBuffer                 = SM_FLOW_DISP_L;

    #if ENABLED(SDSUPPORT) && PIN_EXISTS(SD_DETECT)
      pinMode(SD_DETECT_PIN, INPUT);
      WRITE(SD_DETECT_PIN, HIGH);
    #endif

    pinMode(FILAMENT_RUNOUT_PIN, INPUT);
    WRITE(FILAMENT_RUNOUT_PIN, HIGH);

    #if ENABLED(ANYCUBIC_FILAMENT_RUNOUT_SENSOR)
      if ((READ(FILAMENT_RUNOUT_PIN) == true) && FilamentSensorEnabled)
        SENDLINE_DBG_PGM("J15", "TFT Serial Debug: Non blocking filament runout... J15");
    #endif

    #if ENABLED(KNUTWURST_TFT_LEVELING)
      setupMyZoffset();
      delay(10);
    #endif

    setup_OutageTestPin();
    setup_PowerOffPin();

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

    // which kind of starting behaviour is needed?
    switch (ai3m_pause_state) {
      case 0:
        // no pause, just a regular start
        starttime = millis();
        card.startOrResumeFilePrinting();
        TFTstate = ANYCUBIC_TFT_STATE_SDPRINT;
        #ifdef ANYCUBIC_TFT_DEBUG
          SERIAL_ECHOPGM("DEBUG: AI3M Pause State: ", ai3m_pause_state);
          SERIAL_EOL();
          SERIAL_ECHOLNPGM("DEBUG: Regular Start");
        #endif
        break;
      case 1:
        // regular sd pause
        #ifdef ANYCUBIC_TFT_DEBUG
          SERIAL_ECHOPGM("DEBUG: AI3M Pause State: ", ai3m_pause_state);
          SERIAL_EOL();
          SERIAL_ECHOLNPGM("DEBUG: M108/M24 Resume from regular pause");
        #endif

        IsParked  = false; // remove parked flag
        starttime = millis();
        card.startOrResumeFilePrinting(); // resume regularly

        wait_for_heatup = false;
        wait_for_user   = false;

        TFTstate         = ANYCUBIC_TFT_STATE_SDPRINT;
        ai3m_pause_state = 0;
        #ifdef ANYCUBIC_TFT_DEBUG
          SERIAL_ECHOPGM("DEBUG: AI3M Pause State: ", ai3m_pause_state);
          SERIAL_EOL();
        #endif
        break;
      case 2:
        // paused by M600
        #ifdef ANYCUBIC_TFT_DEBUG
          SERIAL_ECHOPGM("DEBUG: AI3M Pause State: ", ai3m_pause_state);
          SERIAL_EOL();
          SERIAL_ECHOLNPGM("DEBUG: Start M108 routine");
        #endif
        FilamentChangeResume(); // enter display M108 routine
        ai3m_pause_state = 0; // clear flag
        #ifdef ANYCUBIC_TFT_DEBUG
          SERIAL_ECHOLNPGM("DEBUG: Filament Change Flag cleared");
        #endif
        break;
      case 3:
        // paused by filament runout
        #ifdef ANYCUBIC_TFT_DEBUG
          SERIAL_ECHOLNPGM("DEBUG: M108/M24 Resume from Filament Runout");
        #endif
        IsParked = false; // remove parked flag
        card.startOrResumeFilePrinting(); // resume regularly

        wait_for_heatup = false;
        wait_for_user   = false;

        TFTstate         = ANYCUBIC_TFT_STATE_SDPRINT;
        ai3m_pause_state = 0;
        #ifdef ANYCUBIC_TFT_DEBUG
          SERIAL_ECHOLNPGM("DEBUG: Filament Pause Flag cleared");
        #endif
        break;
      case 4:
        // nozzle was timed out before, do not enter printing state yet
        TFTstate = ANYCUBIC_TFT_STATE_SDPAUSE_REQ;
        #ifdef ANYCUBIC_TFT_DEBUG
          SERIAL_ECHOLNPGM("DEBUG: Set Pause again because of timeout");
        #endif

        // clear the timeout flag to ensure the print continues on the
        // next push of CONTINUE
        ai3m_pause_state = 1;
        #ifdef ANYCUBIC_TFT_DEBUG
          SERIAL_ECHOLNPGM("DEBUG: Nozzle timeout flag cleared");
        #endif
        break;
      case 5:
        // nozzle was timed out before (M600), do not enter printing state yet
        TFTstate = ANYCUBIC_TFT_STATE_SDPAUSE_REQ;
        #ifdef ANYCUBIC_TFT_DEBUG
          SERIAL_ECHOLNPGM("DEBUG: Set Pause again because of timeout");
        #endif

        // clear the timeout flag to ensure the print continues on the
        // next push of CONTINUE
        ai3m_pause_state = 2;
        #ifdef ANYCUBIC_TFT_DEBUG
          SERIAL_ECHOLNPGM("DEBUG: Nozzle timeout flag cleared");
        #endif
        break;
      case 6:
        // nozzle was timed out before (runout), do not enter printing state yet
        TFTstate = ANYCUBIC_TFT_STATE_SDPAUSE_REQ;
        #ifdef ANYCUBIC_TFT_DEBUG
          SERIAL_ECHOLNPGM("DEBUG: Set Pause again because of timeout");
        #endif

        // clear the timeout flag to ensure the print continues on the
        // next push of CONTINUE
        ai3m_pause_state = 3;
        #ifdef ANYCUBIC_TFT_DEBUG
          SERIAL_ECHOLNPGM("DEBUG: Nozzle timeout flag cleared");
        #endif
        break;
      default:
        break;
    }
  }

  void AnycubicTouchscreenClass::PausePrint() {
    SENDLINE_DBG_PGM("J05", "TFT Serial Debug: SD print pause started... J05"); // J05 printing pause
    #ifdef SDSUPPORT
      if (ai3m_pause_state < 2) { // is this a regular pause?
        card.pauseSDPrint(); // pause print regularly
        #ifdef ANYCUBIC_TFT_DEBUG
          SERIAL_ECHOPGM("DEBUG: AI3M Pause State: ", ai3m_pause_state);
          SERIAL_EOL();
          SERIAL_ECHOLNPGM("DEBUG: Regular Pause");
        #endif
      }
      else { // pause caused by filament runout
        #ifdef ANYCUBIC_TFT_DEBUG
          SERIAL_ECHOLNPGM("DEBUG: Filament Runout Pause");
        #endif
        // filament runout, retract and beep
        // queue.inject_P(PSTR("G91"));          // relative mode
        // queue.inject_P(PSTR("G1 E-3 F1800")); // retract 3mm
        // queue.inject_P(PSTR("G90"));          // absolute mode
        BUZZ(200, 1567);
        BUZZ(200, 1174);
        BUZZ(200, 1567);
        BUZZ(200, 1174);
        BUZZ(2000, 1567);
        #ifdef ANYCUBIC_TFT_DEBUG
          SERIAL_ECHOLNPGM("DEBUG: Filament runout - Retract, beep and park.");
        #endif
        card.pauseSDPrint(); // pause print and park nozzle
        ai3m_pause_state = 1;
        #ifdef ANYCUBIC_TFT_DEBUG
          SERIAL_ECHOLNPGM("DEBUG: M25 sent, parking nozzle");
        #endif
        // show filament runout prompt on screen
        SENDLINE_DBG_PGM("J23", "TFT Serial Debug: Blocking filament prompt... J23");
      }
    #endif
    TFTstate = ANYCUBIC_TFT_STATE_SDPAUSE_REQ;
  }

  inline void AnycubicTouchscreenClass::StopPrint() {
    card.abortFilePrintSoon();

    print_job_timer.stop();
    thermalManager.disable_all_heaters();
    thermalManager.zero_fan_speeds();

    #ifdef ANYCUBIC_TFT_DEBUG
      SERIAL_ECHOLNPGM("DEBUG: Stopped and cleared");
    #endif

    ai3m_pause_state = 0;
    #ifdef ANYCUBIC_TFT_DEBUG
      SERIAL_ECHOPGM("DEBUG: AI3M Pause State: ", ai3m_pause_state);
      SERIAL_EOL();
    #endif

    TFTstate = ANYCUBIC_TFT_STATE_SDSTOP_REQ;
  }

  void AnycubicTouchscreenClass::FilamentChangeResume() {
    #ifdef ANYCUBIC_TFT_DEBUG
      SERIAL_ECHOLNPGM("DEBUG: M108 Resume called");
    #endif

    // IsParked = false; // remove parked flag
    starttime = millis();
    card.startOrResumeFilePrinting(); // resume regularly

    wait_for_heatup = false;
    wait_for_user   = false;

    TFTstate         = ANYCUBIC_TFT_STATE_SDPRINT;
    ai3m_pause_state = 0;


    #ifdef ANYCUBIC_TFT_DEBUG
      SERIAL_ECHOLNPGM("DEBUG: M108 Resume done");
    #endif
  }

  void AnycubicTouchscreenClass::FilamentChangePause() {
    // set filament change flag to ensure the M108 routine
    // gets used when the user hits CONTINUE
    ai3m_pause_state = 2;
    #ifdef ANYCUBIC_TFT_DEBUG
      SERIAL_ECHOPGM("DEBUG: AI3M Pause State: ", ai3m_pause_state);
      SERIAL_EOL();
    #endif

    // call M600 and set display state to paused
    queue.inject_P(PSTR("M600"));
    TFTstate = ANYCUBIC_TFT_STATE_SDPAUSE_REQ;
    #ifdef ANYCUBIC_TFT_DEBUG
      SERIAL_ECHOLNPGM("DEBUG: M600 Pause called");
    #endif
  }

  void AnycubicTouchscreenClass::ReheatNozzle() {
    #ifdef ANYCUBIC_TFT_DEBUG
      SERIAL_ECHOLNPGM("DEBUG: Send reheat M108");
    #endif

    // M108
    wait_for_heatup = false;
    wait_for_user   = false;

    #ifdef ANYCUBIC_TFT_DEBUG
      SERIAL_ECHOLNPGM("DEBUG: Resume heating");
    #endif

    // enable heaters again
    HOTEND_LOOP()
    thermalManager.reset_hotend_idle_timer(e);
    #ifdef ANYCUBIC_TFT_DEBUG
      SERIAL_ECHOLNPGM("DEBUG: Clear flags");
    #endif

    if (ai3m_pause_state > 3) {
      ai3m_pause_state -= 3;
      #ifdef ANYCUBIC_TFT_DEBUG
        SERIAL_ECHOPGM("DEBUG: NTO done, AI3M Pause State: ", ai3m_pause_state);
        SERIAL_EOL();
      #endif
    }

    // set pause state to show CONTINUE button again
    TFTstate = ANYCUBIC_TFT_STATE_SDPAUSE_REQ;
  }

  void AnycubicTouchscreenClass::ParkAfterStop() {

    queue.enqueue_now_P(PSTR("M84")); // disable stepper motors
    queue.enqueue_now_P(PSTR("M27")); // force report of SD status

    ai3m_pause_state = 0;
    #ifdef ANYCUBIC_TFT_DEBUG
      SERIAL_ECHOPGM("DEBUG: AI3M Pause State: ", ai3m_pause_state);
      SERIAL_EOL();
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
        SERIAL_ECHOPGM("DEBUG: Special Menu Selection: ", currentTouchscreenSelection);
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
          queue.inject_P(PSTR("G28\nG90\nG1 Z20\nG1 X110 Y110 F4000\nG1 Z5\nM106 S172\nG4 P500\nM303 E0 S215 C15 U1\nG4 P500\nM107\nG28\nG1 Z10\nM84\nM500\nM300 S440 P200\nM300 S660 P250\nM300 S880 P300"));
        #endif

        #if ENABLED(KNUTWURST_MEGA_X)
          queue.inject_P(PSTR("G28\nG90\nG1 Z20\nG1 X155 Y155 F4000\nG1 Z5\nM106 S172\nG4 P500\nM303 E0 S215 C15 U1\nG4 P500\nM107\nG28\nG1 Z10\nM84\nM500\nM300 S440 P200\nM300 S660 P250\nM300 S880 P300"));
        #endif

        #if ENABLED(KNUTWURST_CHIRON)
          queue.inject_P(PSTR("G28\nG90\nG1 Z20\nG1 X205 Y205 F4000\nG1 Z5\nM106 S172\nG4 P500\nM303 E0 S215 C15 U1\nG4 P500\nM107\nG28\nG1 Z10\nM84\nM500\nM300 S440 P200\nM300 S660 P250\nM300 S880 P300"));
        #endif

        #if ENABLED(KNUTWURST_4MAXP2)
          queue.inject_P(PSTR("G28\nG90\nG1 Z20\nG1 X105 Y135 F4000\nG1 Z5\nM106 S172\nG4 P500\nM303 E0 S215 C15 U1\nG4 P500\nM107\nG28\nG1 Z10\nM84\nM500\nM300 S440 P200\nM300 S660 P250\nM300 S880 P300"));
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
        queue.inject_P(PSTR("M303 E-1 S60 C6 U1\nM500\nM300 S440 P200\nM300 S660 P250\nM300 S880 P300"));
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
        queue.inject_P(PSTR("M140 S60"));
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
          queue.inject_P(PSTR("G28\nG29 S1"));
        }
        else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_MESH_NEXT_L)) != NULL)
                 || (strcasestr_P(currentTouchscreenSelection, PSTR(SM_MESH_NEXT_S)) != NULL)
                 ) {
          SERIAL_ECHOLNPGM("Special Menu: Next Mesh Point");
          queue.inject_P(PSTR("G29 S2"));
        }
        else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_Z_UP_01_L)) != NULL)
                 || (strcasestr_P(currentTouchscreenSelection, PSTR(SM_Z_UP_01_S)) != NULL)
                 ) {
          SERIAL_ECHOLNPGM("Special Menu: Z Up 0.1");
          queue.inject_P(PSTR("G91\nG1 Z+0.1\nG90"));
        }
        else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_Z_DN_01_L)) != NULL)
                 || (strcasestr_P(currentTouchscreenSelection, PSTR(SM_Z_DN_01_S)) != NULL)
                 ) {
          SERIAL_ECHOLNPGM("Special Menu: Z Down 0.1");
          queue.inject_P(PSTR("G91\nG1 Z-0.1\nG90"));
        }
        else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_Z_UP_002_L)) != NULL)
                 || (strcasestr_P(currentTouchscreenSelection, PSTR(SM_Z_UP_002_S)) != NULL)
                 ) {
          SERIAL_ECHOLNPGM("Special Menu: Z Up 0.02");
          queue.inject_P(PSTR("G91\nG1 Z+0.02\nG90"));
        }
        else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_Z_DN_002_L)) != NULL)
                 || (strcasestr_P(currentTouchscreenSelection, PSTR(SM_Z_DN_002_S)) != NULL)
                 ) {
          SERIAL_ECHOLNPGM("Special Menu: Z Down 0.02");
          queue.inject_P(PSTR("G91\nG1 Z-0.02\nG90"));
        }
        else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_Z_UP_001_L)) != NULL)
                 || (strcasestr_P(currentTouchscreenSelection, PSTR(SM_Z_UP_001_S)) != NULL)
                 ) {
          SERIAL_ECHOLNPGM("Special Menu: Z Up 0.01");
          queue.inject_P(PSTR("G91\nG1 Z+0.03\nG4 P250\nG1 Z-0.02\nG90"));
        }
        else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_Z_DN_001_L)) != NULL)
                 || (strcasestr_P(currentTouchscreenSelection, PSTR(SM_Z_DN_001_S)) != NULL)
                 ) {
          SERIAL_ECHOLNPGM("Special Menu: Z Down 0.01");
          queue.inject_P(PSTR("G91\nG1 Z+0.02\nG4 P250\nG1 Z-0.03\nG90"));
        }
      #endif // if NONE(KNUTWURST_BLTOUCH, KNUTWURST_TFT_LEVELING)

      #if ENABLED(KNUTWURST_BLTOUCH)
        else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_BLTOUCH_L)) != NULL)
                 || (strcasestr_P(currentTouchscreenSelection, PSTR(SM_BLTOUCH_S)) != NULL)
                 ) {
          SERIAL_ECHOLNPGM("Special Menu: BLTouch Leveling");
          queue.inject_P(PSTR("G28\nG29\nM500\nG90\nM300 S440 P200\nM300 S660 P250\nM300 S880 P300\nG1 Z30 F4000\nG1 X0 F4000\nG91\nM84\nM420 S1"));
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
        FilamentChangePause();
      }
      else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_RESUME_L)) != NULL)
               || (strcasestr_P(currentTouchscreenSelection, PSTR(SM_RESUME_S)) != NULL)
               ) {
        SERIAL_ECHOLNPGM("Special Menu: Fil. Change Resume");
        FilamentChangeResume();
      }
      else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_DIS_FILSENS_L)) != NULL)
               || (strcasestr_P(currentTouchscreenSelection, PSTR(SM_DIS_FILSENS_S)) != NULL)
               ) {
        SERIAL_ECHOLNPGM("Special Menu: Disable Filament Sensor");
        FilamentSensorEnabled = false;
        BUZZ(105, 1108);
        BUZZ(105, 1108);
        BUZZ(105, 1108);
      }
      else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_EN_FILSENS_L)) != NULL)
               || (strcasestr_P(currentTouchscreenSelection, PSTR(SM_EN_FILSENS_S)) != NULL)
               ) {
        SERIAL_ECHOLNPGM("Special Menu: Enable Filament Sensor");
        FilamentSensorEnabled = true;
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
        queue.inject_P(PSTR("G28\nM420 S0\nG90\nG1 Z5\nG1 X15 Y15 F4000\nG1 Z0"));
      }
      else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_EZLVL_P1_L)) != NULL)
               || (strcasestr_P(currentTouchscreenSelection, PSTR(SM_EZLVL_P1_S)) != NULL)
               ) {
        SERIAL_ECHOLNPGM("Special Menu: Easy Level POINT 1");
        queue.inject_P(PSTR("G90\nG1 Z5\nG1 X15 Y15 F4000\nG1 Z0"));
      }
      else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_EZLVL_P2_L)) != NULL)
               || (strcasestr_P(currentTouchscreenSelection, PSTR(SM_EZLVL_P2_S)) != NULL)
               ) {
        SERIAL_ECHOLNPGM("Special Menu: Easy Level POINT 2");
        #if ANY(KNUTWURST_MEGA, KNUTWURST_MEGA_S, KNUTWURST_MEGA_P)
          queue.inject_P(PSTR("G90\nG1 Z5\nG1 X205 Y15 F4000\nG1 Z0"));
        #endif

        #if ENABLED(KNUTWURST_MEGA_X)
          queue.inject_P(PSTR("G90\nG1 Z5\nG1 X295 Y15 F4000\nG1 Z0"));
        #endif

        #if ENABLED(KNUTWURST_CHIRON)
          queue.inject_P(PSTR("G90\nG1 Z5\nG1 X385 Y15 F4000\nG1 Z0"));
        #endif

        #if ENABLED(KNUTWURST_4MAXP2)
          queue.inject_P(PSTR("G90\nG1 Z5\nG1 X255 Y15 F4000\nG1 Z0"));
        #endif
      }
      else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_EZLVL_P3_L)) != NULL)
               || (strcasestr_P(currentTouchscreenSelection, PSTR(SM_EZLVL_P3_S)) != NULL)
               ) {
        SERIAL_ECHOLNPGM("Special Menu: Easy Level POINT 3");
        #if ANY(KNUTWURST_MEGA, KNUTWURST_MEGA_S, KNUTWURST_MEGA_P)
          queue.inject_P(PSTR("G90\nG1 Z5\nG1 X205 Y200 F4000\nG1 Z0"));
        #endif

        #if ENABLED(KNUTWURST_MEGA_X)
          queue.inject_P(PSTR("G90\nG1 Z5\nG1 X295 Y295 F4000\nG1 Z0"));
        #endif

        #if ENABLED(KNUTWURST_CHIRON)
          queue.inject_P(PSTR("G90\nG1 Z5\nG1 X395 Y395 F4000\nG1 Z0"));
        #endif

        #if ENABLED(KNUTWURST_4MAXP2)
          queue.inject_P(PSTR("G90\nG1 Z5\nG1 X255 Y195 F4000\nG1 Z0"));
        #endif
      }
      else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_EZLVL_P4_L)) != NULL)
               || (strcasestr_P(currentTouchscreenSelection, PSTR(SM_EZLVL_P4_S)) != NULL)
               ) {
        SERIAL_ECHOLNPGM("Special Menu: Easy Level POINT 4");
        #if ANY(KNUTWURST_MEGA, KNUTWURST_MEGA_S, KNUTWURST_MEGA_P)
          queue.inject_P(PSTR("G90\nG1 Z5\nG1 X15 Y200 F4000\nG1 Z0"));
        #endif

        #if ENABLED(KNUTWURST_MEGA_X)
          queue.inject_P(PSTR("G90\nG1 Z5\nG1 X15 Y295 F4000\nG1 Z0"));
        #endif

        #if ENABLED(KNUTWURST_CHIRON)
          queue.inject_P(PSTR("G90\nG1 Z5\nG1 X15 Y395 F4000\nG1 Z0"));
        #endif

        #if ENABLED(KNUTWURST_4MAXP2)
          queue.inject_P(PSTR("G90\nG1 Z5\nG1 X15 Y195 F4000\nG1 Z0"));
        #endif
      }
      else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_EZLVL_EXIT_L)) != NULL)
               || (strcasestr_P(currentTouchscreenSelection, PSTR(SM_EZLVL_EXIT_S)) != NULL)
               ) {
        SERIAL_ECHOLNPGM("Special Menu: Exit Easy Level Menu");
        LevelMenu = false;
        queue.inject_P(PSTR("G90\nG1 Z10\nG1 X15 Y15 F4000\nM420 S1"));
      }
    #endif // if ENABLED(KNUTWURST_SPECIAL_MENU)
  }


  void AnycubicTouchscreenClass::PrintList() {
    #if ENABLED(KNUTWURST_SPECIAL_MENU)
      if (MMLMenu) {
        switch (filenumber) {
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

        switch (filenumber) {
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
          SERIAL_ECHOPGM("DEBUG: Current probe.offset.z: ", float(probe.offset.z));
          SERIAL_EOL();
        #endif

        zOffsetBuffer.replace("XXXXX", String(float(probe.offset.z)));

        switch (filenumber) {
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
        switch (filenumber) {
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
        switch (filenumber) {
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

    #ifdef SDSUPPORT
      #if ENABLED(KNUTWURST_SPECIAL_MENU)
        else if (card.isMounted())
      #else
        if (card.isMounted())
      #endif
      {
        int count = filenumber;
        int max_files;
        int filesOnSDCard = card.countFilesInWorkDir();

        if ((filesOnSDCard - filenumber) < 4) {
          max_files = filesOnSDCard;
          #ifdef ANYCUBIC_TFT_DEBUG
            SERIAL_ECHOLN("max_files = filesOnSDCard;");
          #endif
        }
        else {
          max_files = filenumber + 3;
          #ifdef ANYCUBIC_TFT_DEBUG
            SERIAL_ECHOLN("max_files = filenumber + 3;");
          #endif
        }

        if (filesOnSDCard == 3) filenumber = 0;

        #ifdef ANYCUBIC_TFT_DEBUG
          SERIAL_ECHOPGM("filesOnSDCard: ");
          SERIAL_ECHOLN(filesOnSDCard);
          SERIAL_ECHOPGM("filenumber: ");
          SERIAL_ECHOLN(filenumber);
          SERIAL_ECHOPGM("max_files: ");
          SERIAL_ECHOLN(max_files);
          SERIAL_ECHOPGM("count: ");
          SERIAL_ECHOLN(count);
        #endif

        for (count = filenumber; count <= max_files; count++) {
          if (count == 0) { // Special Entry
            if (strcmp(card.getWorkDirName(), "/") == 0) {
              SENDLINE_PGM(SM_SPECIAL_MENU_S);
              SENDLINE_PGM(SM_SPECIAL_MENU_L);
              SERIAL_ECHO(count);
              SERIAL_ECHO(": ");
              SERIAL_ECHOLNPGM(SM_SPECIAL_MENU_L);
            }
            else {
              SENDLINE_PGM(SM_DIR_UP_S);
              SENDLINE_PGM(SM_DIR_UP_L);
              SERIAL_ECHO(count);
              SERIAL_ECHO(": ");
              SERIAL_ECHOLNPGM(SM_DIR_UP_L);
            }
          }
          else {
            card.selectFileByIndex(count - 1);

            // The longname may not be filed, so we use the built-in fallback here.
            char* fileName      = card.longest_filename();
            int fileNameLen     = strlen(fileName);

            // Cut off too long filenames. They don't fit on the screen anyway.
            #if ENABLED(KNUTWURST_DGUS2_TFT)
              bool fileNameWasCut = false;
              if (fileNameLen > MAX_PRINTABLE_FILENAME_LEN) {
                fileNameWasCut = true;
                fileNameLen    = MAX_PRINTABLE_FILENAME_LEN;
              }
            #endif

            char outputString[fileNameLen];

            // Bugfix for non-printable special characters which are now replaced by underscores.
            for (unsigned char i = 0; i < fileNameLen; i++) {
              if (isPrintable(fileName[i]))
                outputString[i] = fileName[i];
              else
                outputString[i] = '_';
            }

            #if ENABLED(KNUTWURST_DGUS2_TFT)
              // Append extension, if filename was truncated. I know, it's ugly, but it's faster than a string lib.
              if (fileNameWasCut) {
                outputString[fileNameLen - 7] = '~';
                outputString[fileNameLen - 6] = '.';
                outputString[fileNameLen - 5] = 'g';
                outputString[fileNameLen - 4] = 'c';
                outputString[fileNameLen - 3] = 'o';
                outputString[fileNameLen - 2] = 'd';
                outputString[fileNameLen - 1] = 'e';
              } else {
                // Make sure to fill the output buffer with blanks.
                for (unsigned char i = fileNameLen; i < MAX_PRINTABLE_FILENAME_LEN; i++) {
                  outputString[i] = ' ';
                }
              }
              outputString[MAX_PRINTABLE_FILENAME_LEN] = '\0';
            #else
              // Just terminate the string.
              outputString[fileNameLen] = '\0';
            #endif

            if (card.flag.filenameIsDir) {
              #if ENABLED(KNUTWURST_DGUS2_TFT)
                SEND_PGM("/");
                SEND(card.filename);
                SENDLINE_PGM(".GCO");
                SEND_PGM("/");
                SEND(outputString);
                SENDLINE_PGM(".gcode");
              #else
                SEND_PGM("/");
                SEND(card.filename);
                SEND_PGM("/");
                SENDLINE(outputString);
              #endif
              SERIAL_ECHO(count);
              SERIAL_ECHOPGM(": /");
              SERIAL_ECHOLN(outputString);
            }
            else {
              SENDLINE(card.filename);
              SENDLINE(outputString);
              SERIAL_ECHO(count);
              SERIAL_ECHOPGM(": ");
              SERIAL_ECHOLN(outputString);
            }
          }
        }
      }
    #endif // ifdef SDSUPPORT
    else {
      #if ENABLED(KNUTWURST_SPECIAL_MENU_WO_SD)
        SENDLINE_PGM(SM_SPECIAL_MENU_S);
        SENDLINE_PGM(SM_SPECIAL_MENU_L);
      #endif
    }
  }

  void AnycubicTouchscreenClass::CheckSDCardChange() {
    #ifdef SDSUPPORT
      if (LastSDstatus != IS_SD_INSERTED()) {
        LastSDstatus = IS_SD_INSERTED();

        if (LastSDstatus) {
          card.mount();
          SENDLINE_DBG_PGM("J00", "TFT Serial Debug: SD card inserted... J00");
        }
        else {
          SENDLINE_DBG_PGM("J01", "TFT Serial Debug: SD card removed... J01");
        }
      }
    #endif
  }

  void AnycubicTouchscreenClass::CheckHeaterError() {
    if ((thermalManager.degHotend(0) < 5) || (thermalManager.degHotend(0) > 300)) {
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

  void AnycubicTouchscreenClass::StateHandler() {
    switch (TFTstate) {
      case ANYCUBIC_TFT_STATE_IDLE:
        #ifdef SDSUPPORT
          if (card.isPrinting()) {
            TFTstate  = ANYCUBIC_TFT_STATE_SDPRINT;
            starttime = millis();
          }
        #endif
        break;
      case ANYCUBIC_TFT_STATE_SDPRINT:
        #ifdef SDSUPPORT
          if (!card.isPrinting()) {
            if (card.isFileOpen()) {
              // File is still open --> paused
              TFTstate = ANYCUBIC_TFT_STATE_SDPAUSE;
            }
            else if ((!card.isFileOpen()) && (ai3m_pause_state == 0)) {
              // File is closed --> stopped
              TFTstate = ANYCUBIC_TFT_STATE_IDLE;
              SENDLINE_DBG_PGM("J14", "TFT Serial Debug: SD Print Completed... J14");
              powerOFFflag = 1;
              ai3m_pause_state = 0;
              #ifdef ANYCUBIC_TFT_DEBUG
                SERIAL_ECHOPGM("DEBUG: AI3M Pause State: ", ai3m_pause_state);
                SERIAL_EOL();
              #endif
            }
          }
        #endif
        break;
      case ANYCUBIC_TFT_STATE_SDPAUSE:
        break;
      case ANYCUBIC_TFT_STATE_SDPAUSE_OOF:
        #ifdef ANYCUBIC_FILAMENT_RUNOUT_SENSOR
          if (!FilamentTestStatus)
            // We got filament again
            TFTstate = ANYCUBIC_TFT_STATE_SDPAUSE;

        #endif
        break;
      case ANYCUBIC_TFT_STATE_SDPAUSE_REQ:
        #ifdef SDSUPPORT
          if ((!card.isPrinting()) && (!planner.movesplanned())) {
            SENDLINE_PGM("J18");
            if (ai3m_pause_state < 2) {
              // no flags, this is a regular pause.
              ai3m_pause_state = 1;
              #ifdef ANYCUBIC_TFT_DEBUG
                SERIAL_ECHOPGM("DEBUG: AI3M Pause State: ", ai3m_pause_state);
                SERIAL_EOL();
                SERIAL_ECHOLNPGM("DEBUG: Regular Pause requested");
              #endif
              if (!IsParked) {
                // park head and retract 2mm
                queue.inject_P(PSTR("M125 L2"));
                IsParked = true;
              }
            }
            #ifdef ANYCUBIC_FILAMENT_RUNOUT_SENSOR
              if (FilamentTestStatus)
                TFTstate = ANYCUBIC_TFT_STATE_SDPAUSE;
              else
                // Pause because of "out of filament"
                TFTstate = ANYCUBIC_TFT_STATE_SDPAUSE_OOF;

            #endif
            #ifdef ANYCUBIC_TFT_DEBUG
              SERIAL_ECHOLNPGM("TFT Serial Debug: SD print paused done... J18");
            #endif
          }
        #endif
        break;
      case ANYCUBIC_TFT_STATE_SDSTOP_REQ:
        #ifdef SDSUPPORT
          SENDLINE_DBG_PGM("J16", "TFT Serial Debug: SD print stop called... J16");
          if ((!card.isPrinting()) && (!planner.movesplanned())) {
            TFTstate         = ANYCUBIC_TFT_STATE_IDLE;
            ai3m_pause_state = 0;
            #ifdef ANYCUBIC_TFT_DEBUG
              SERIAL_ECHOPGM("DEBUG: AI3M Pause State: ", ai3m_pause_state);
              SERIAL_EOL();
            #endif
          }
          // did we park the hotend already?
          if ((!IsParked) && (!card.isPrinting()) && (!planner.movesplanned())) {
            queue.enqueue_now_P(PSTR("G91\nG1 E-1 F1800\nG90")); // retract
            ParkAfterStop();
            IsParked = true;
          }
        #endif
        break;
      default:
        break;
    }
  }


/*
 * TODO: Refactoring of the filamentsensor-Stuff.
 *
 * Every cycle a timer should be reset if the sensor reads "filament is present"
 * If the timer is not reset within a period of time, the filament runout state
 * should be triggered.
 */

  void AnycubicTouchscreenClass::FilamentRunout() {
    if (FilamentSensorEnabled == true) {
      #if ENABLED(ANYCUBIC_FILAMENT_RUNOUT_SENSOR)
        FilamentTestStatus = READ(FILAMENT_RUNOUT_PIN) & 0xff;

        if (FilamentTestStatus > FilamentTestLastStatus) {
          // filament sensor pin changed, save current timestamp.
          const millis_t fil_ms = millis();
          static millis_t fil_delay;

          // since this is inside a loop, only set delay time once
          if (FilamentSetMillis) {
            #ifdef ANYCUBIC_TFT_DEBUG
              SERIAL_ECHOLNPGM("DEBUG: Set filament trigger time");
            #endif
            // set the delayed timestamp to 5000ms later
            fil_delay = fil_ms + 5000UL;
            // this doesn't need to run until the filament is recovered again
            FilamentSetMillis = false;
          }

          // if five seconds passed and the sensor is still triggered,
          // we trigger the filament runout status
          if ((FilamentTestStatus > FilamentTestLastStatus) && (ELAPSED(fil_ms, fil_delay))) {
            if (!IsParked) {
              #ifdef ANYCUBIC_TFT_DEBUG
                SERIAL_ECHOLNPGM("DEBUG: 5000ms delay done");
              #endif
              if (card.isPrinting()) {
                ai3m_pause_state = 3; // set runout pause flag
                #ifdef ANYCUBIC_TFT_DEBUG
                  SERIAL_ECHOPGM("DEBUG: AI3M Pause State: ", ai3m_pause_state);
                  SERIAL_EOL();
                #endif
                PausePrint();
              }
              else if (!card.isPrinting()) {
                SENDLINE_DBG_PGM("J15", "TFT Serial Debug: Non blocking filament runout... J15");
                FilamentTestLastStatus = FilamentTestStatus;
              }
            }
            FilamentTestLastStatus = FilamentTestStatus;
          }
        }
        else if (FilamentTestStatus != FilamentTestLastStatus) {
          FilamentSetMillis      = true; // set the timestamps on the next loop again
          FilamentTestLastStatus = FilamentTestStatus;
          #ifdef ANYCUBIC_TFT_DEBUG
            SERIAL_ECHOLNPGM("TFT Serial Debug: Filament runout recovered");
          #endif
        }
      #endif // if ENABLED(ANYCUBIC_FILAMENT_RUNOUT_SENSOR)
    }
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
                SEND_PGM_VAL("A0V ", int(thermalManager.degHotend(0) + 0.5));
                break;

              case 1: // A1  GET HOTEND TARGET TEMP
                SEND_PGM_VAL("A1V ", int(thermalManager.degTargetHotend(0) + 0.5));
                break;

              case 2: // A2 GET HOTBED TEMP
                SEND_PGM_VAL("A2V ", int(thermalManager.degBed() + 0.5));
                break;

              case 3: // A3 GET HOTBED TARGET TEMP
                SEND_PGM_VAL("A3V ", int(thermalManager.degTargetBed() + 0.5));
                break;

              case 4: // A4 GET FAN SPEED
              {
                unsigned int temp;

                temp = ((thermalManager.fan_speed[0] * 100) / 255);
                temp = constrain(temp, 0, 100);

                SEND_PGM_VAL("A4V ", temp);
              }
              break;
              case 5: // A5 GET CURRENT COORDINATE
                SEND_PGM("A5V X: "); LCD_SERIAL.print(current_position[X_AXIS]);
                SEND_PGM(   " Y: "); LCD_SERIAL.print(current_position[Y_AXIS]);
                SEND_PGM(   " Z: "); LCD_SERIAL.print(current_position[Z_AXIS]);
                SENDLINE_PGM("");
                break;
              case 6: // A6 GET SD CARD PRINTING STATUS
                #ifdef SDSUPPORT
                  if (card.isPrinting()) {
                    SEND_PGM("A6V ");
                    if (card.isMounted())
                      SENDLINE(itostr3(card.percentDone()));
                    else
                      SENDLINE_DBG_PGM("J02", "TFT Serial Debug: SD Card initialized... J02");
                  }
                  else {
                    SENDLINE_PGM("A6V ---");
                  }
                #endif
                break;
              case 7: // A7 GET PRINTING TIME
              {
                SEND_PGM("A7V ");
                if (starttime != 0) { // print time
                  uint16_t time = millis() / 60000 - starttime / 60000;
                  SEND(itostr2(time / 60));
                  SEND_PGM(" H ");
                  SEND(itostr2(time % 60));
                  SEND_PGM(" M");
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
                    SENDLINE_PGM("FN "); // Filelist start
                    PrintList();
                    SENDLINE_PGM("END"); // Filelist stop
                  }
                #endif
                break;
              case 9: // A9 pause sd print
                #ifdef SDSUPPORT
                  if (card.isPrinting()) {
                    PausePrint();
                  }
                  else {
                    ai3m_pause_state = 0;
                    #ifdef ANYCUBIC_TFT_DEBUG
                      SERIAL_ECHOPGM("DEBUG: AI3M Pause State: ", ai3m_pause_state);
                      SERIAL_EOL();
                    #endif
                    StopPrint();
                  }
                #endif
                break;
              case 10: // A10 resume sd print
                #ifdef SDSUPPORT
                  #ifdef ANYCUBIC_TFT_DEBUG
                    SERIAL_ECHOPGM("DEBUG: AI3M Pause State: ", ai3m_pause_state);
                    SERIAL_EOL();
                  #endif
                  if ((TFTstate == ANYCUBIC_TFT_STATE_SDPAUSE) || (TFTstate == ANYCUBIC_TFT_STATE_SDOUTAGE))
                    StartPrint();
                  if (ai3m_pause_state > 3)
                    ReheatNozzle(); // obsolete!

                #endif
                break;
              case 11: // A11 STOP SD PRINT
                #ifdef SDSUPPORT
                  if ((card.isPrinting()) || (TFTstate == ANYCUBIC_TFT_STATE_SDOUTAGE)) {
                    StopPrint();
                  }
                  else {
                    SENDLINE_DBG_PGM("J16", "TFT Serial Debug: SD print stop called... J16");
                    TFTstate         = ANYCUBIC_TFT_STATE_IDLE;
                    ai3m_pause_state = 0;
                    #ifdef ANYCUBIC_TFT_DEBUG
                      SERIAL_ECHOPGM("DEBUG: AI3M Pause State: ", ai3m_pause_state);
                      SERIAL_EOL();
                    #endif
                  }
                #endif
                break;
              case 12: // A12 kill
                kill(F(STR_ERR_KILLED));
                break;
              case 13: // A13 SELECTION FILE
                #ifdef SDSUPPORT
                  if ((TFTstate != ANYCUBIC_TFT_STATE_SDOUTAGE)) {
                    starpos = (strchr(TFTstrchr_pointer + 4, '*'));
                    if (TFTstrchr_pointer[4] == '/') {
                      strcpy(currentTouchscreenSelection, TFTstrchr_pointer + 5);
                      #ifdef ANYCUBIC_TFT_DEBUG
                        SERIAL_ECHOPGM(" TFT Serial Debug: currentTouchscreenSelection: ", currentTouchscreenSelection);
                        SERIAL_EOL();
                      #endif
                    }
                    else if (TFTstrchr_pointer[4] == '<') {
                      strcpy(currentTouchscreenSelection, TFTstrchr_pointer + 4);
                    }
                    else {
                      if (SpecialMenu == false)
                        currentTouchscreenSelection[0] = 0;

                      if (starpos != NULL) *(starpos - 1) = '\0';
                      card.openFileRead(TFTstrchr_pointer + 4);
                      if (card.isFileOpen())
                        SENDLINE_DBG_PGM_VAL("J20", "TFT Serial Debug: File Selected... J20 ", currentTouchscreenSelection);
                      else
                        SENDLINE_DBG_PGM("J21", "TFT Serial Debug: On SD Card Error ... J21");
                    }
                    SENDLINE_PGM("");
                  }
                #endif
                break;
              case 14: // A14 START PRINTING
                #ifdef SDSUPPORT
                  if ((!planner.movesplanned()) && (TFTstate != ANYCUBIC_TFT_STATE_SDPAUSE) && (TFTstate != ANYCUBIC_TFT_STATE_SDOUTAGE) && (card.isFileOpen())) {
                    ai3m_pause_state = 0;
                    powerOFFflag     = 0;
                    #ifdef ANYCUBIC_TFT_DEBUG
                      SERIAL_ECHOPGM("DEBUG: AI3M Pause State: ", ai3m_pause_state);
                      SERIAL_EOL();
                    #endif
                    StartPrint();
                    IsParked = false;
                    SENDLINE_DBG_PGM("J04", "TFT Serial Debug: Starting SD Print... J04"); // J04 Starting Print
                  }
                #endif
                break;
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
                  if (thermalManager.degTargetHotend(0) <= 260)
                    thermalManager.setTargetHotend(tempvalue, 0); // do not set Temp from TFT if it is set via gcode
                }
                else if ((CodeSeen('C')) && (!planner.movesplanned())) {
                  if ((current_position[Z_AXIS] < 10))
                    queue.inject_P(PSTR("G1 Z10")); // RASE Z AXIS
                  tempvalue = constrain(CodeValue(), 0, 260);
                  thermalManager.setTargetHotend(tempvalue, 0);
                }
              }
              break;
              case 17: // A17 set heated bed temp
              {
                unsigned int tempbed;
                if (CodeSeen('S')) {
                  tempbed = constrain(CodeValue(), 0, 115);
                  thermalManager.setTargetBed(tempbed);
                  if (thermalManager.degTargetBed() <= 100)
                    thermalManager.setTargetBed(tempbed); // do not set Temp from TFT if it is set via gcode
                }
              }
              break;
              case 18: // A18 set fan speed
                unsigned int temp;
                if (CodeSeen('S')) {
                  temp = (CodeValue() * 255 / 100);
                  temp = constrain(temp, 0, 255);
                  thermalManager.set_fan_speed(0, temp);
                }
                else {
                  thermalManager.set_fan_speed(0, 255);
                }
                SENDLINE_PGM("");
                break;
              case 19: // A19 stop stepper drivers
                if ((!planner.movesplanned())
                    #ifdef SDSUPPORT
                      && (!card.isPrinting())
                    #endif
                    ) {
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
                if ((!planner.movesplanned()) && (TFTstate != ANYCUBIC_TFT_STATE_SDPAUSE) && (TFTstate != ANYCUBIC_TFT_STATE_SDOUTAGE)) {
                  if (CodeSeen('X') || CodeSeen('Y') || CodeSeen('Z')) {
                    if (CodeSeen('X')) queue.inject_P(PSTR("G28 X"));
                    if (CodeSeen('Y')) queue.inject_P(PSTR("G28 Y"));
                    if (CodeSeen('Z')) queue.inject_P(PSTR("G28 Z"));
                  }
                  else if (CodeSeen('C')) {
                    queue.inject_P(PSTR("G28"));
                  }
                }
                break;
              case 22: // A22 move X/Y/Z or extrude
                if ((!planner.movesplanned()) && (TFTstate != ANYCUBIC_TFT_STATE_SDPAUSE) && (TFTstate != ANYCUBIC_TFT_STATE_SDOUTAGE)) {
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
                if ((!planner.movesplanned()) && (TFTstate != ANYCUBIC_TFT_STATE_SDPAUSE) && (TFTstate != ANYCUBIC_TFT_STATE_SDOUTAGE)) {
                  if ((current_position[Z_AXIS] < 10))
                    queue.inject_P(PSTR("G1 Z10")); // RAISE Z AXIS

                  thermalManager.setTargetBed(KNUTWURST_PRHEAT_BED_PLA);
                  thermalManager.setTargetHotend(KNUTWURST_PRHEAT_NOZZLE_PLA, 0);
                  SENDLINE_PGM("OK");
                }
                break;
              case 24: // A24 preheat abs
                if ((!planner.movesplanned()) && (TFTstate != ANYCUBIC_TFT_STATE_SDPAUSE) && (TFTstate != ANYCUBIC_TFT_STATE_SDOUTAGE)) {
                  if ((current_position[Z_AXIS] < 10))
                    queue.inject_P(PSTR("G1 Z10")); // RAISE Z AXIS
                  thermalManager.setTargetBed(KNUTWURST_PRHEAT_BED_ABS);
                  thermalManager.setTargetHotend(KNUTWURST_PRHEAT_NOZZLE_ABS, 0);
                  SENDLINE_PGM("OK");
                }
                break;
              case 25: // A25 cool down
                if ((!planner.movesplanned()) && (TFTstate != ANYCUBIC_TFT_STATE_SDPAUSE) && (TFTstate != ANYCUBIC_TFT_STATE_SDOUTAGE)) {
                  thermalManager.setTargetHotend(0, 0);
                  thermalManager.setTargetBed(0);
                  SENDLINE_DBG_PGM("J12", "TFT Serial Debug: Cooling down... J12"); // J12 cool down
                }
                break;
              case 26: // A26 refresh SD
                #ifdef SDSUPPORT
                  #ifdef ANYCUBIC_TFT_DEBUG
                    SERIAL_ECHOPGM(" TFT Serial Debug: currentTouchscreenSelection: ", currentTouchscreenSelection);
                    SERIAL_EOL();
                  #endif
                  if (currentTouchscreenSelection[0] == 0) {
                    card.mount();
                  }
                  else {
                    if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_DIR_UP_S)) != NULL)
                        ||  (strcasestr_P(currentTouchscreenSelection, PSTR(SM_DIR_UP_L)) != NULL)
                        ) {
                      #ifdef ANYCUBIC_TFT_DEBUG
                        SERIAL_ECHOLNPGM("TFT Serial Debug: Directory UP (cd ..)");
                      #endif
                      card.cdup();
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
                          card.cd(currentFileOrDirectory);
                        #else
                          card.cd(currentTouchscreenSelection);
                        #endif
                      }
                    }
                  }
                  if (SpecialMenu == false)
                    currentTouchscreenSelection[0] = 0;

                #endif // ifdef SDSUPPORT
                break;
                #ifdef SERVO_ENDSTOPS
                    case 27: // A27 servos angles  adjust
                      break;
                #endif
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

                      if ((!planner.movesplanned()) && (TFTstate != ANYCUBIC_TFT_STATE_SDPAUSE) && (TFTstate != ANYCUBIC_TFT_STATE_SDOUTAGE)) {
                        if (!all_axes_trusted()) {
                          queue.inject_P(PSTR("G28\n"));
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
                      if ((planner.movesplanned()) || (card.isPrinting()))
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
                      if ((planner.movesplanned()) || (card.isPrinting()))
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
                    case 42:
                      if (CaseLight == true) {
                        SERIAL_ECHOLNPGM("Case Light OFF");
                        queue.inject_P(PSTR("M355 S1 P0"));
                        CaseLight = false;
                      }
                      else {
                        SERIAL_ECHOLNPGM("Case Light ON");
                        queue.inject_P(PSTR("M355 S1 P255"));
                        CaseLight = true;
                      }
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
    CheckHeaterError();
    CheckSDCardChange();
    StateHandler();

    #if ENABLED(KNUTWURST_4MAXP2)
      if (PrintdoneAndPowerOFF && powerOFFflag && (thermalManager.degHotend(0) < 50 )) {
        powerOFFflag = 0;
        PowerDown();
      }
    #endif

    if (TFTbuflen < (TFTBUFSIZE - 1))
      GetCommandFromTFT();
    if (TFTbuflen) {
      TFTbuflen  = (TFTbuflen - 1);
      TFTbufindr = (TFTbufindr + 1) % TFTBUFSIZE;
    }
  }

  void AnycubicTouchscreenClass::HeatingStart() {
    SENDLINE_DBG_PGM("J06", "TFT Serial Debug: Nozzle is heating... J06");
  }

  void AnycubicTouchscreenClass::HeatingDone() {
    SENDLINE_DBG_PGM("J07", "TFT Serial Debug: Nozzle is done... J07");

    if (TFTstate == ANYCUBIC_TFT_STATE_SDPRINT) {
      SENDLINE_DBG_PGM("J04", "TFT Serial Debug: Continuing SD print after heating... J04");
    }
  }

  void AnycubicTouchscreenClass::BedHeatingStart() {
    SENDLINE_DBG_PGM("J08", "TFT Serial Debug: Bed is heating... J08");
  }

  void AnycubicTouchscreenClass::BedHeatingDone() {
    SENDLINE_DBG_PGM("J09", "TFT Serial Debug: Bed heating is done... J09");

    if (TFTstate == ANYCUBIC_TFT_STATE_SDPRINT) {
      SENDLINE_DBG_PGM("J04", "TFT Serial Debug: Continuing SD print after heating... J04");
    }
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

  AnycubicTouchscreenClass AnycubicTouchscreen;
#endif // ifdef ANYCUBIC_TOUCHSCREEN
