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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../../../gcode/queue.h"
#include "../../../libs/buzzer.h"
#include "../../../libs/numtostr.h"
#include "../../../module/motion.h"
#include "../../../module/stepper.h"
#include "../../../module/temperature.h"
#include "../ui_api.h"

#ifdef ANYCUBIC_TOUCHSCREEN
  #include "./anycubic_touchscreen.h"

  // command sending macro's with debugging capability
  #define SEND_PGM(x)        send_P(PSTR(x))
  #define SENDLINE_PGM(x)    sendLine_P(PSTR(x))
  #define SEND_PGM_VAL(x, y) (send_P(PSTR(x)), sendLine(i16tostr3rj(y)))
  #define SEND(x)            send(x)
  #define SENDLINE(x)        sendLine(x)
  #if ENABLED(ANYCUBIC_TFT_DEBUG)
    #define SENDLINE_DBG_PGM(x, y)                                                                                     \
      do {                                                                                                             \
        sendLine_P(PSTR(x));                                                                                           \
        SERIAL_ECHOLNPGM(y);                                                                                           \
      } while (0)
    #define SENDLINE_DBG_PGM_VAL(x, y, z)                                                                              \
      do {                                                                                                             \
        sendLine_P(PSTR(x));                                                                                           \
        SERIAL_ECHOLNPGM(y, z);                                                                                        \
      } while (0)
  #else
    #define SENDLINE_DBG_PGM(x, y)        sendLine_P(PSTR(x))
    #define SENDLINE_DBG_PGM_VAL(x, y, z) sendLine_P(PSTR(x))
  #endif

// Serial helpers
static void sendNewLine() {
  LCD_SERIAL.write('\r');
  LCD_SERIAL.write('\n');
}
static void send(const char* str) { LCD_SERIAL.print(str); }
static void send_P(PGM_P str) {
  while (const char c = pgm_read_byte(str++)) {
    LCD_SERIAL.write(c);
  }
}
static void sendLine(const char* str) {
  send(str);
  sendNewLine();
}
static void sendLine_P(PGM_P str) {
  send_P(str);
  sendNewLine();
}

AnycubicMediaPrintState AnycubicTouchscreenClass::mediaPrintingState = AMPRINTSTATE_NOT_PRINTING;
AnycubicMediaPauseState AnycubicTouchscreenClass::mediaPauseState    = AMPAUSESTATE_NOT_PAUSED;
uint32_t AnycubicTouchscreenClass::time_last_cyclic_tft_command = 0;
uint8_t AnycubicTouchscreenClass::delayed_tft_command = 0;

  #if ENABLED(POWER_OUTAGE_TEST)
int           PowerInt                     = 6;
unsigned char PowerTestFlag                = false;
int           Temp_Buf_Extuder_Temperature = 0;
int           Temp_Buf_Bed_Temperature     = 0;
unsigned char ResumingFlag                 = 0;
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

AnycubicTouchscreenClass::AnycubicTouchscreenClass() {}

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
  mediaPauseState    = AMPAUSESTATE_NOT_PAUSED;

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
  live_Zoffset                   = 0.0;

  #if BOTH(SDSUPPORT, HAS_SD_DETECT)
  SET_INPUT_PULLUP(SD_DETECT_PIN);
  #endif

  #if ENABLED(FILAMENT_RUNOUT_SENSOR)
  SET_INPUT_PULLUP(FIL_RUNOUT1_PIN);
  #endif

  setup_OutageTestPin();
  setup_PowerOffPin();

  #if ENABLED(KNUTWURST_MEGA_P_LASER)
    laser_init();
  #endif

  SENDLINE_DBG_PGM("J12", "TFT Serial Debug: Ready... J12");

  CheckHeaterError();
  // DoFilamentRunoutCheck();

  #ifdef STARTUP_CHIME
  BUZZ(100, 554);
  BUZZ(100, 740);
  BUZZ(100, 831);
  #endif
}

#if ENABLED(KNUTWURST_MEGA_P_LASER)
  /**
   * ============================================================================
   * Laser Engraving Feature for Anycubic Mega Pro
   * ============================================================================
   *
   * This section implements the integrated laser engraving functionality for
   * the Anycubic Mega Pro printer. The laser module can engrave BMP images
   * by converting them to grayscale and controlling laser intensity via PWM.
   *
   * Key features:
   * - BMP file support (16-bit, 24-bit, 32-bit color depths)
   * - Grayscale conversion using ITU-R BT.601 coefficients
   * - Configurable laser parameters (height, pixel distance, offsets)
   * - Mirror modes for X and Y axes
   * - Vector/raster engraving modes
   */

  // Start and end coordinates for laser engraving area
  static float x_start, y_start;
  static float x_end, y_end;

  /**
   * Initialize laser engraving parameters with default values.
   * Called once during setup to configure the laser module.
   */
  void AnycubicTouchscreenClass::laser_init() {
    laser_printer_st.pic_pixel_distance = PIC_FIXED;      // Distance between pixels (0.1mm default)
    laser_printer_st.laser_height       = 50;             // Z-height for laser engraving (50mm default)
    laser_printer_st.x_offset           = 0;              // X offset between laser and extruder
    laser_printer_st.y_offset           = 0;              // Y offset between laser and extruder

    laser_printer_st.pic_vector     = 0;                  // Raster mode (0) or vector mode (1)
    laser_printer_st.pic_x_mirror   = 1;                  // Mirror image on X axis
    laser_printer_st.pic_y_mirror   = 0;                  // Mirror image on Y axis
    laser_printer_st.pic_laser_time = 15;                 // Laser pulse duration (ms)

    send_laser_param();                                   // Send initial parameters to TFT
  }

  /**
   * Send BMP image dimensions to the TFT display.
   * Format: A45V W<width> H<height>
   */
  void AnycubicTouchscreenClass::send_pic_param() {
    SEND_PGM("A45V W"); LCD_SERIAL.print(laser_printer_st.pic_width);
    SEND_PGM(    " H"); LCD_SERIAL.print(laser_printer_st.pic_height);
    SENDLINE_PGM(" ");
  }

  /**
   * Send all laser configuration parameters to the TFT display.
   * Format: A44V A<vector> B<time> C<height> D<distance> E<x_off> F<y_off> G<x_mir> H<y_mir>
   */
  void AnycubicTouchscreenClass::send_laser_param() {
    SEND_PGM("A44V A"); LCD_SERIAL.print(laser_printer_st.pic_vector);        // Vector mode
    SEND_PGM(    " B"); LCD_SERIAL.print(laser_printer_st.pic_laser_time);    // Laser pulse time
    SEND_PGM(    " C"); LCD_SERIAL.print(laser_printer_st.laser_height);      // Z height
    SEND_PGM(    " D"); LCD_SERIAL.print(laser_printer_st.pic_pixel_distance);// Pixel distance
    SEND_PGM(    " E"); LCD_SERIAL.print(laser_printer_st.x_offset);          // X offset
    SEND_PGM(    " F"); LCD_SERIAL.print(laser_printer_st.y_offset);          // Y offset
    SEND_PGM(    " G"); LCD_SERIAL.print(laser_printer_st.pic_x_mirror);      // X mirror
    SEND_PGM(    " H"); LCD_SERIAL.print(laser_printer_st.pic_y_mirror);      // Y mirror
    SENDLINE_PGM(" ");
  }

  /**
   * Read a pixel from the BMP file and convert it to grayscale.
   *
   * Supports 16-bit, 24-bit, and 32-bit BMP formats. Uses ITU-R BT.601
   * coefficients for RGB to grayscale conversion:
   * Y = 0.212671*R + 0.715160*G + 0.072169*B
   *
   * @param gray Pointer to store the resulting grayscale value (0-255)
   * @param y    Y coordinate in the image
   * @param x    X coordinate in the image
   */
  void AnycubicTouchscreenClass::read_bmp(unsigned char *gray, unsigned int y, unsigned int x) {
    unsigned char red_t, green_t, blue_t;
    unsigned int temp;
    float Y;
    unsigned char buffer[4];

    // 32-bit BMP (RGBA format)
    if (laser_printer_st.pic_bit == 32) {
      // Calculate file position: BMP is stored bottom-up, so invert Y
      laser_printer_st.pic_ptr = (laser_printer_st.pic_height - y - 1) * laser_printer_st.pic_real_width + x * 4 + laser_printer_st.pic_start;
      card.setIndex(laser_printer_st.pic_ptr);
      card.read(buffer, 4);

      // Convert RGB to grayscale using ITU-R BT.601 coefficients
      Y     = buffer[2] * 0.212671f + buffer[1] * 0.715160f + buffer[0] * 0.072169f;
      *gray = 0xff - (unsigned char)Y;  // Invert for laser (0=white, 255=black)
    }
    // 24-bit BMP (RGB format)
    else if (laser_printer_st.pic_bit == 24) {
      laser_printer_st.pic_ptr = (unsigned long)(laser_printer_st.pic_height - y - 1) * laser_printer_st.pic_real_width + x * 3 + laser_printer_st.pic_start;
      card.setIndex(laser_printer_st.pic_ptr);
      card.read(buffer, 3);

      Y     = buffer[2] * 0.212671f + buffer[1] * 0.715160f + buffer[0] * 0.072169f;
      *gray = 0xff - (unsigned char)Y;
    }
    // 16-bit BMP (RGB565 format)
    else if (laser_printer_st.pic_bit == 16) {
      laser_printer_st.pic_ptr = (laser_printer_st.pic_height - y - 1) * laser_printer_st.pic_real_width + x * 2 + laser_printer_st.pic_start;
      card.setIndex(laser_printer_st.pic_ptr);
      card.read(buffer, 2);

      // Extract RGB components from 16-bit RGB565 format
      temp    = (buffer[1] << 8) + buffer[0];
      red_t   = (temp & 0xf800) >> 8;   // 5 bits red
      green_t = (temp & 0x07e0) >> 3;   // 6 bits green
      blue_t  = (temp & 0x001f) << 3;   // 5 bits blue

      // Simplified grayscale conversion for 16-bit
      Y       = (red_t * 77 + green_t * 150 + blue_t * 29) / 256;
      *gray = 0xff - (unsigned char)Y;
    }
  }

  /**
   * Prepare the printer for laser engraving.
   *
   * This is a 3-step state machine:
   * Step 0: Home all axes and move to laser height
   * Step 1: Wait for homing to complete
   * Step 2: Start the actual engraving process
   */
  void AnycubicTouchscreenClass::prepare_laser_print() {
    if (laser_print_step == 0) {
      SERIAL_ECHOLNPGM("Laser: Homing and moving to engraving height");

      // Home all axes
      queue.enqueue_now_P(PSTR("G28"));

      // Move Z axis to configured laser height
      char value[30];
      sprintf_P(value, PSTR("G1 Z%s F500"), ftostr42_52(laser_printer_st.laser_height));
      queue.enqueue_one_now(value);

      laser_print_step = 1;
    }
    else if (laser_print_step == 1) {
      // Wait for all movement commands to complete
      if (commandsInQueue()) return;

      SERIAL_ECHOLNPGM("Laser: Preparation complete, ready to engrave");
      laser_print_step = 2;
    }
    else if (laser_print_step == 2) {
      SERIAL_ECHOLNPGM("Laser: Starting engraving process");

      // Execute the actual laser engraving
      laser_print_picture();

      // Clean up after engraving is complete
      laser_print_step = 0;
      card.fileHasFinished();
      card.autofile_check();
      en_continue = 0;
    }
  }

  /**
   * Main laser engraving function.
   *
   * Processes the BMP image pixel by pixel, moving the laser head and
   * controlling laser intensity based on grayscale values. Uses a
   * bidirectional scanning pattern (left-to-right on even rows,
   * right-to-left on odd rows) to minimize travel time.
   *
   * The laser is controlled via PWM on HEATER_0_PIN:
   * - Raster mode: Laser pulses proportional to pixel brightness
   * - Vector mode: Laser stays on while moving (2x speed)
   */
  void AnycubicTouchscreenClass::laser_print_picture() {
    SERIAL_ECHOLNPGM("Laser: Starting image engraving");

    unsigned char Y;                    // Grayscale value of current pixel
    unsigned long time;                 // Laser pulse duration
    static int i, j;                    // Loop counters for Y and X
    int ftemp;                          // Feed rate (speed)
    static char laser_counter = 0;      // Counter for TFT status updates
    static char cvalue[30];             // Buffer for G-code commands

    int x_max = laser_printer_st.pic_width;
    int y_max = laser_printer_st.pic_height;

    WRITE(HEATER_0_PIN, 0);             // Ensure laser is off at start

    laser_status  = 1;                  // Mark laser as active
    laser_counter = 0;

    // Set engraving speed based on mode
    if (laser_printer_st.pic_vector == 0)
      ftemp = LASER_PRINT_SPEED;        // Raster mode: normal speed
    else
      ftemp = 2 * LASER_PRINT_SPEED;    // Vector mode: double speed

    // Calculate X axis start/end positions based on mirror setting
    // Image is centered on the bed with laser offset applied
    if (laser_printer_st.pic_x_mirror == 1) {
      x_start = MAX_X_SIZE / 2  + LASER_X_OFFSET + (laser_printer_st.pic_width * laser_printer_st.pic_pixel_distance) / 2;
      x_end   = MAX_X_SIZE / 2  + LASER_X_OFFSET - (laser_printer_st.pic_width * laser_printer_st.pic_pixel_distance) / 2;
    }
    else {
      x_end   = MAX_X_SIZE / 2  + LASER_X_OFFSET + (laser_printer_st.pic_width * laser_printer_st.pic_pixel_distance) / 2;
      x_start = MAX_X_SIZE / 2  + LASER_X_OFFSET - (laser_printer_st.pic_width * laser_printer_st.pic_pixel_distance) / 2;
    }

    // Calculate Y axis start/end positions based on mirror setting
    if (laser_printer_st.pic_y_mirror == 1) {
      y_start = MAX_Y_SIZE / 2  + LASER_Y_OFFSET + (laser_printer_st.pic_height * laser_printer_st.pic_pixel_distance) / 2;
      y_end   = MAX_Y_SIZE / 2  + LASER_Y_OFFSET - (laser_printer_st.pic_height * laser_printer_st.pic_pixel_distance) / 2;
    }
    else {
      y_end   = MAX_Y_SIZE / 2  + LASER_Y_OFFSET + (laser_printer_st.pic_height * laser_printer_st.pic_pixel_distance) / 2;
      y_start = MAX_Y_SIZE / 2  + LASER_Y_OFFSET - (laser_printer_st.pic_height * laser_printer_st.pic_pixel_distance) / 2;
    }

    // Send status to TFT (J29 = laser engraving in progress)
    SENDLINE_PGM("J29");
    SENDLINE_PGM("J29");

    // Main engraving loop: process image row by row
    for (i = 0; i < y_max; i++) {
      // Calculate Y position for this row
      float y_pos;
      if (laser_printer_st.pic_y_mirror == 0)
        y_pos = y_start + i * laser_printer_st.pic_pixel_distance;
      else
        y_pos = y_start - i * laser_printer_st.pic_pixel_distance;

      // Even rows: scan left to right
      if (i % 2 == 0) {
        for (j = 0; j < x_max; j++) {
          read_bmp(&Y, i, j);           // Read pixel grayscale value

          // Only engrave if pixel is dark enough (above threshold)
          if (Y > MIN_GRAY_VLAUE) {
            // Calculate X position
            float x_pos;
            if (laser_printer_st.pic_x_mirror == 1)
              x_pos = x_start - j * laser_printer_st.pic_pixel_distance;
            else
              x_pos = x_start + j * laser_printer_st.pic_pixel_distance;

            // Move to pixel position (X and Y together)
            sprintf_P(cvalue, PSTR("G1 X%s Y%s F%i"), ftostr42_52(x_pos), ftostr42_52(y_pos), ftemp);
            queue.enqueue_one_now(cvalue);
            while (commandsInQueue()) idle();

            // Fire laser
            WRITE(HEATER_0_PIN, 1);

            // In raster mode, pulse duration is proportional to pixel darkness
            if (laser_printer_st.pic_vector == 0) {
              time = Y * laser_printer_st.pic_laser_time;
              while (time--) WRITE(HEATER_0_PIN, 1);
              WRITE(HEATER_0_PIN, 0);
            }
          }
          else {
            WRITE(HEATER_0_PIN, 0);     // Turn off laser for white pixels
          }

          // Check for pause/stop commands from TFT
          TFTCommandScan();
          while (laser_print_pause) {
            TFTCommandScan();
            if (laser_status == 0) return;
          }
          if (laser_status == 0) return;
        }
      }
      // Odd rows: scan right to left (bidirectional)
      else {
        for (j = 0; j < x_max; j++) {
          // Read pixels in reverse order: x_max-1, x_max-2, ..., 0
          int pixel_x = x_max - 1 - j;
          read_bmp(&Y, i, pixel_x);

          // Only engrave if pixel is dark enough (above threshold)
          if (Y > MIN_GRAY_VLAUE) {
            // Calculate X position (moving from right to left)
            float x_pos;
            if (laser_printer_st.pic_x_mirror == 1)
              x_pos = x_end + j * laser_printer_st.pic_pixel_distance;
            else
              x_pos = x_end - j * laser_printer_st.pic_pixel_distance;

            // Move to pixel position (X and Y together)
            sprintf_P(cvalue, PSTR("G1 X%s Y%s F%i"), ftostr42_52(x_pos), ftostr42_52(y_pos), ftemp);
            queue.enqueue_one_now(cvalue);
            while (commandsInQueue()) idle();

            // Fire laser
            WRITE(HEATER_0_PIN, 1);

            // In raster mode, pulse duration is proportional to pixel darkness
            if (laser_printer_st.pic_vector == 0) {
              time = Y * laser_printer_st.pic_laser_time;
              while (time--) WRITE(HEATER_0_PIN, 1);
              WRITE(HEATER_0_PIN, 0);
            }

            // Send progress update to TFT every 20 pixels
            if (laser_counter != 20) {
              laser_counter = 20;
              SENDLINE_PGM("J30");
              SENDLINE_PGM("J30");
            }
          }
          else {
            WRITE(HEATER_0_PIN, 0);     // Turn off laser for white pixels
          }

          // Check for pause/stop commands from TFT
          TFTCommandScan();
          while (laser_print_pause) {
            TFTCommandScan();
            if (laser_status == 0) return;
          }
          if (laser_status == 0) return;
        }
      }
    }
    laser_status = 0;
  }

  /**
   * Handle laser indication.
   */
  void AnycubicTouchscreenClass::laser_indicate() {
    static unsigned char laser_indicate_step = 0;
    static unsigned char laser_indicate_on   = 0;
    static char cvalue[30];

    if (laser_on_off == 1 && laser_indicate_on == 0 && laser_indicate_step == 3) {
      // Activate the laser and start indication routine.
      sprintf_P(cvalue, PSTR("M3 S%i"), LASER_INDICATE_LEVEL);
      queue.enqueue_one_now(cvalue);
      laser_indicate_on = 1;
    }
    else if (laser_on_off == 0 && (laser_indicate_on == 1 || laser_indicate_step > 0)) {
      // Deactivate the laser.
      queue.enqueue_now_P(PSTR("M5"));
      laser_indicate_step = 0;
      laser_indicate_on   = 0;
      return;
    }
    else if (laser_on_off == 0) {
      // Indication not active and laser is off, nothing to do.
      return;
    }

    if (file_type == 0) return;

    if (laser_indicate_step == 0) {
      // Home X and Y axis and initialize start/end coordinates for given picture.
      queue.enqueue_now_P(PSTR("G28 X0 Y0"));

      if (laser_printer_st.pic_x_mirror == 1) {
        x_start = MAX_X_SIZE / 2 + LASER_X_OFFSET + (laser_printer_st.pic_width * laser_printer_st.pic_pixel_distance) / 2;
        x_end   = MAX_X_SIZE / 2  + LASER_X_OFFSET - (laser_printer_st.pic_width * laser_printer_st.pic_pixel_distance) / 2;
      }
      else {
        x_end   = MAX_X_SIZE / 2  + LASER_X_OFFSET + (laser_printer_st.pic_width * laser_printer_st.pic_pixel_distance) / 2;
        x_start = MAX_X_SIZE / 2 + LASER_X_OFFSET - (laser_printer_st.pic_width * laser_printer_st.pic_pixel_distance) / 2;
      }

      if (laser_printer_st.pic_y_mirror == 1) {
        y_start = MAX_Y_SIZE / 2 + LASER_Y_OFFSET + (laser_printer_st.pic_height * laser_printer_st.pic_pixel_distance) / 2;
        y_end   = MAX_Y_SIZE / 2 + LASER_Y_OFFSET - (laser_printer_st.pic_height * laser_printer_st.pic_pixel_distance) / 2;
      }
      else {
        y_end   = MAX_Y_SIZE / 2 + LASER_Y_OFFSET + (laser_printer_st.pic_height * laser_printer_st.pic_pixel_distance) / 2;
        y_start = MAX_Y_SIZE / 2 + LASER_Y_OFFSET - (laser_printer_st.pic_height * laser_printer_st.pic_pixel_distance) / 2;
      }

      laser_indicate_step = 1;
    }
    else if (laser_indicate_step == 1) {
      // Wait for homing to finish.
      if (commandsInQueue()) return;
      // Move to first corner.
      SERIAL_ECHOLNPGM("1 x=", x_end);
      sprintf_P(cvalue, PSTR("G1 X%sF3600"), ftostr42_52(x_end));
      queue.enqueue_one_now(cvalue);
      laser_indicate_step = 2;
    }
    else if (laser_indicate_step == 2) {
      // Wait for previous move to finish.
      if (commandsInQueue()) return;
      // Move to second corner.
      SERIAL_ECHOLNPGM("2 y=", y_end);
      sprintf_P(cvalue, PSTR("G1 Y%sF3600"), ftostr42_52(y_end));
      queue.enqueue_one_now(cvalue);
      laser_indicate_step = 3;
    }
    else if (laser_indicate_step == 3) {
      // Wait for previous move to finish.
      if (commandsInQueue()) return;
      // Move to third corner.
      SERIAL_ECHOLNPGM("3 x=", x_start);
      sprintf_P(cvalue, PSTR("G1 X%sF3600"), ftostr42_52(x_start));
      queue.enqueue_one_now(cvalue);
      laser_indicate_step = 4;
    }
    else if (laser_indicate_step == 4) {
      // Wait for previous move to finish.
      if (commandsInQueue()) return;
      // Move to fourth corner.
      SERIAL_ECHOLNPGM("4 y=", y_start);
      sprintf_P(cvalue, PSTR("G1 Y%sF3600"), ftostr42_52(y_start));
      queue.enqueue_one_now(cvalue);
      laser_indicate_step = 1;
    }
  }
#endif // if ENABLED(KNUTWURST_MEGA_P_LASER)

void AnycubicTouchscreenClass::KillTFT() { SENDLINE_DBG_PGM("J11", "TFT Serial Debug: Kill command... J11"); }

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
  if (isPrintingFromMedia() && mediaPrintingState != AMPRINTSTATE_STOP_REQUESTED &&
      mediaPauseState == AMPAUSESTATE_NOT_PAUSED) {
    mediaPrintingState = AMPRINTSTATE_PAUSE_REQUESTED;
    mediaPauseState    = AMPAUSESTATE_NOT_PAUSED;                               // need the userconfirm method to
                                                                                // update pause state
    SENDLINE_DBG_PGM("J05", "TFT Serial Debug: SD print pause started... J05"); // J05 printing pause

    // for some reason pausing the print doesn't retract the extruder so force a
    // manual one here
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

  // for some reason stopping the print doesn't retract the extruder so force a
  // manual one here
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
    if ( millis() - time_last_cyclic_tft_command >= WAIT_MS_UNTIL_ACYCLIC_SEND ) {
      // re-enable the continue button
      SENDLINE_DBG_PGM("J18", "TFT Serial Debug: Resume Print with filament sensor still "
                              "tripped... J18");
    }
    else
      delayed_tft_command = 18;
    return;
  }
    #endif

  if (mediaPauseState == AMPAUSESTATE_HEATER_TIMEOUT) {
    mediaPauseState = AMPAUSESTATE_REHEATING;
    // reheat the nozzle
    setUserConfirmed();
  } else if (mediaPauseState == AMPAUSESTATE_FILAMENT_PURGING) {
    setUserConfirmed();
  } else {
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
  if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_SPECIAL_MENU_L)) != NULL) ||
      (strcasestr_P(currentTouchscreenSelection, PSTR(SM_SPECIAL_MENU_S)) != NULL)) {
    SpecialMenu = true;
  } else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_PID_HOTEND_L)) != NULL) ||
             (strcasestr_P(currentTouchscreenSelection, PSTR(SM_PID_HOTEND_S)) != NULL)) {
    SERIAL_ECHOLNPGM("Special Menu: PID Tune Hotend");

    #if ANY(KNUTWURST_MEGA, KNUTWURST_MEGA_S, KNUTWURST_MEGA_P)
    injectCommands(F("G28\nG90\nG1 Z20\nG1 X110 Y110 F4000\nG1 Z5\nM106 "
                     "S172\nG4 P500\nM303 E0 S215 C15 U1\nG4 "
                     "P500\nM107\nG28\nG1 Z10\nM84\nM500\nM300 S440 P200\nM300 "
                     "S660 P250\nM300 S880 P300"));
    #endif

    #if ENABLED(KNUTWURST_MEGA_X)
    injectCommands(F("G28\nG90\nG1 Z20\nG1 X155 Y155 F4000\nG1 Z5\nM106 "
                     "S172\nG4 P500\nM303 E0 S215 C15 U1\nG4 "
                     "P500\nM107\nG28\nG1 Z10\nM84\nM500\nM300 S440 P200\nM300 "
                     "S660 P250\nM300 S880 P300"));
    #endif

    #if ENABLED(KNUTWURST_CHIRON)
    injectCommands(F("G28\nG90\nG1 Z20\nG1 X205 Y205 F4000\nG1 Z5\nM106 "
                     "S172\nG4 P500\nM303 E0 S215 C15 U1\nG4 "
                     "P500\nM107\nG28\nG1 Z10\nM84\nM500\nM300 S440 P200\nM300 "
                     "S660 P250\nM300 S880 P300"));
    #endif

    #if ENABLED(KNUTWURST_4MAXP2)
    injectCommands(F("G28\nG90\nG1 Z20\nG1 X105 Y135 F4000\nG1 Z5\nM106 "
                     "S172\nG4 P500\nM303 E0 S215 C15 U1\nG4 "
                     "P500\nM107\nG28\nG1 Z10\nM84\nM500\nM300 S440 P200\nM300 "
                     "S660 P250\nM300 S880 P300"));
    #endif

    BUZZ(200, 1108);
    BUZZ(200, 1661);
    BUZZ(200, 1108);
    BUZZ(600, 1661);
  } else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_PID_BED_L)) != NULL) ||
             (strcasestr_P(currentTouchscreenSelection, PSTR(SM_PID_BED_S)) != NULL)) {
    SERIAL_ECHOLNPGM("Special Menu: PID Tune Ultrabase");
    BUZZ(200, 1108);
    BUZZ(200, 1661);
    BUZZ(200, 1108);
    BUZZ(600, 1661);
    injectCommands(F("M303 E-1 S60 C6 U1\nM500\nM300 S440 P200\nM300 S660 "
                     "P250\nM300 S880 P300"));
    BUZZ(200, 1108);
    BUZZ(200, 1661);
    BUZZ(200, 1108);
    BUZZ(600, 1661);
  } else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_SAVE_EEPROM_L)) != NULL) ||
             (strcasestr_P(currentTouchscreenSelection, PSTR(SM_SAVE_EEPROM_S)) != NULL)) {
    SERIAL_ECHOLNPGM("Special Menu: Save EEPROM");
    injectCommands(F("M500"));
    BUZZ(105, 1108);
    BUZZ(210, 1661);
  } else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_LOAD_DEFAULTS_L)) != NULL) ||
             (strcasestr_P(currentTouchscreenSelection, PSTR(SM_LOAD_DEFAULTS_S)) != NULL)) {
    SERIAL_ECHOLNPGM("Special Menu: Load FW Defaults");
    injectCommands(F("M502"));
    BUZZ(105, 1661);
    BUZZ(210, 1108);
  } else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_PREHEAT_BED_L)) != NULL) ||
             (strcasestr_P(currentTouchscreenSelection, PSTR(SM_PREHEAT_BED_S)) != NULL)) {
    SERIAL_ECHOLNPGM("Special Menu: Preheat Ultrabase");
    injectCommands(F("M140 S60"));
  }

    #if NONE(KNUTWURST_BLTOUCH, KNUTWURST_CHIRON)
  else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_MESH_MENU_L)) != NULL) ||
           (strcasestr_P(currentTouchscreenSelection, PSTR(SM_MESH_MENU_S)) != NULL)) {
    MMLMenu = true;
    SERIAL_ECHOLNPGM("Special Menu: Manual Med Leveling + disable soft endstops");
    setSoftEndstopState(false);
  } else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_MESH_START_L)) != NULL) ||
             (strcasestr_P(currentTouchscreenSelection, PSTR(SM_MESH_START_S)) != NULL)) {
    SERIAL_ECHOLNPGM("Special Menu: Start Mesh Leveling");
    injectCommands(F("G28\nG29 S1"));
  } else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_MESH_NEXT_L)) != NULL) ||
             (strcasestr_P(currentTouchscreenSelection, PSTR(SM_MESH_NEXT_S)) != NULL)) {
    SERIAL_ECHOLNPGM("Special Menu: Next Mesh Point");
    injectCommands(F("G29 S2"));
  } else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_Z_UP_01_L)) != NULL) ||
             (strcasestr_P(currentTouchscreenSelection, PSTR(SM_Z_UP_01_S)) != NULL)) {
    SERIAL_ECHOLNPGM("Special Menu: Z Up 0.1");
    injectCommands(F("G91\nG1 Z+0.1\nG90"));
  } else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_Z_DN_01_L)) != NULL) ||
             (strcasestr_P(currentTouchscreenSelection, PSTR(SM_Z_DN_01_S)) != NULL)) {
    SERIAL_ECHOLNPGM("Special Menu: Z Down 0.1");
    injectCommands(F("G91\nG1 Z-0.1\nG90"));
  } else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_Z_UP_002_L)) != NULL) ||
             (strcasestr_P(currentTouchscreenSelection, PSTR(SM_Z_UP_002_S)) != NULL)) {
    SERIAL_ECHOLNPGM("Special Menu: Z Up 0.02");
    injectCommands(F("G91\nG1 Z+0.02\nG90"));
  } else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_Z_DN_002_L)) != NULL) ||
             (strcasestr_P(currentTouchscreenSelection, PSTR(SM_Z_DN_002_S)) != NULL)) {
    SERIAL_ECHOLNPGM("Special Menu: Z Down 0.02");
    injectCommands(F("G91\nG1 Z-0.02\nG90"));
  } else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_Z_UP_001_L)) != NULL) ||
             (strcasestr_P(currentTouchscreenSelection, PSTR(SM_Z_UP_001_S)) != NULL)) {
    SERIAL_ECHOLNPGM("Special Menu: Z Up 0.01");
    injectCommands(F("G91\nG1 Z+0.03\nG4 P250\nG1 Z-0.02\nG90"));
  } else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_Z_DN_001_L)) != NULL) ||
             (strcasestr_P(currentTouchscreenSelection, PSTR(SM_Z_DN_001_S)) != NULL)) {
    SERIAL_ECHOLNPGM("Special Menu: Z Down 0.01");
    injectCommands(F("G91\nG1 Z+0.02\nG4 P250\nG1 Z-0.03\nG90"));
  }
    #endif // if NONE(KNUTWURST_BLTOUCH, KNUTWURST_CHIRON)

    #if ENABLED(KNUTWURST_BLTOUCH)
  else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_BLTOUCH_L)) != NULL) ||
           (strcasestr_P(currentTouchscreenSelection, PSTR(SM_BLTOUCH_S)) != NULL)) {
    SERIAL_ECHOLNPGM("Special Menu: BLTouch Leveling");
    injectCommands(F("G28\nG29\nG90\nM300 S440 P200\nM300 S660 P250\nM300 S880 P300\nG1 Z30 "
                     "F4000\nG1 X0 F4000\nG91\nM84\nM420 S1"));
    BUZZ(105, 1108);
    BUZZ(210, 1661);
  }
    #endif

    #if ENABLED(KNUTWURST_CHIRON)
  else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_RESETLV_L)) != NULL) ||
           (strcasestr_P(currentTouchscreenSelection, PSTR(SM_RESETLV_S)) != NULL)) {
    SERIAL_ECHOLNPGM("Special Menu: initializeGrid()");
    injectCommands(F("M501\nM420 S1"));
    BUZZ(105, 1108);
    BUZZ(210, 1661);
  }
    #endif

  else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_PAUSE_L)) != NULL) ||
           (strcasestr_P(currentTouchscreenSelection, PSTR(SM_PAUSE_L)) != NULL)) {
    SERIAL_ECHOLNPGM("Special Menu: Fil. Change Pause");
    injectCommands(F("M600"));
  } else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_RESUME_L)) != NULL) ||
             (strcasestr_P(currentTouchscreenSelection, PSTR(SM_RESUME_S)) != NULL)) {
    SERIAL_ECHOLNPGM("Special Menu: Fil. Change Resume");
    ResumePrint();
  } else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_DIS_FILSENS_L)) != NULL) ||
             (strcasestr_P(currentTouchscreenSelection, PSTR(SM_DIS_FILSENS_S)) != NULL)) {
    SERIAL_ECHOLNPGM("Special Menu: Disable Filament Sensor");
    injectCommands(F("M412 H0 S0\nM500"));
    BUZZ(210, 1661);
    BUZZ(105, 1108);
  } else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_EN_FILSENS_L)) != NULL) ||
             (strcasestr_P(currentTouchscreenSelection, PSTR(SM_EN_FILSENS_S)) != NULL)) {
    SERIAL_ECHOLNPGM("Special Menu: Enable Filament Sensor");
    injectCommands(F("M412 H0 S1\nM500"));
    BUZZ(105, 1108);
    BUZZ(210, 1661);
  } else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_EXIT_L)) != NULL) ||
             (strcasestr_P(currentTouchscreenSelection, PSTR(SM_EXIT_S)) != NULL)) {
    SpecialMenu = false;
  } else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_BACK_L)) != NULL) ||
             (strcasestr_P(currentTouchscreenSelection, PSTR(SM_BACK_S)) != NULL)) {
    SERIAL_ECHOLNPGM("Special Menu: Exit Manual Bed Leveling + enable soft endstops");
    setSoftEndstopState(true);
    MMLMenu = false;
  } else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_FLOWMENU_L)) != NULL) ||
             (strcasestr_P(currentTouchscreenSelection, PSTR(SM_FLOWMENU_S)) != NULL)) {
    SERIAL_ECHOLNPGM("Special Menu: Enter Flow Menu");
    FlowMenu = true;
  } else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_FLOW_UP_L)) != NULL) ||
             (strcasestr_P(currentTouchscreenSelection, PSTR(SM_FLOW_UP_S)) != NULL)) {
    SERIAL_ECHOLNPGM("Special Menu: Flow UP");
    currentFlowRate = currentFlowRate + 1;

    if (currentFlowRate > 800) {
      currentFlowRate = 800;
    }

    char value[30];
    sprintf_P(value, PSTR("M221 S%i"), currentFlowRate);
    queue.enqueue_one_now(value);

  } else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_FLOW_DN_L)) != NULL) ||
             (strcasestr_P(currentTouchscreenSelection, PSTR(SM_FLOW_DN_S)) != NULL)) {
    SERIAL_ECHOLNPGM("Special Menu: Flow Down");
    currentFlowRate = currentFlowRate - 1;

    if (currentFlowRate < 1) {
      currentFlowRate = 1;
    }

    char value[30];
    sprintf_P(value, PSTR("M221 S%i"), currentFlowRate);
    queue.enqueue_one_now(value);
  } else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_FLOW_EXIT_L)) != NULL) ||
             (strcasestr_P(currentTouchscreenSelection, PSTR(SM_FLOW_EXIT_S)) != NULL)) {
    SERIAL_ECHOLNPGM("Special Menu: Exit Flow Menu");
    FlowMenu = false;
  }

    #if ENABLED(KNUTWURST_BLTOUCH)
  else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_BLTZMENU_L)) != NULL) ||
           (strcasestr_P(currentTouchscreenSelection, PSTR(SM_BLTZMENU_S)) != NULL)) {
    SERIAL_ECHOLNPGM("Special Menu: Enter BLTouch Menu + disable SoftEndstops");
    BLTouchMenu = true;
    setSoftEndstopState(false);
  } else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_BLTZ_UP_L)) != NULL) ||
             (strcasestr_P(currentTouchscreenSelection, PSTR(SM_BLTZ_UP_S)) != NULL)) {
    SERIAL_ECHOLNPGM("Special Menu: Offset UP");
    setZOffset_mm(getZOffset_mm() + 0.01F);
  } else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_BLTZ_DN_L)) != NULL) ||
             (strcasestr_P(currentTouchscreenSelection, PSTR(SM_BLTZ_DN_S)) != NULL)) {
    SERIAL_ECHOLNPGM("Special Menu: Offset Down");
    setZOffset_mm(getZOffset_mm() - 0.01F);
  } else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_HS_ENABLE_L)) != NULL) ||
             (strcasestr_P(currentTouchscreenSelection, PSTR(SM_HS_ENABLE_S)) != NULL)) {
    SERIAL_ECHOLNPGM("Special Menu: HighSpeed Mode ENABLED");
    injectCommands(F("M401 S1\nM500"));
    BUZZ(105, 1108);
    BUZZ(210, 1661);
  } else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_HS_DISABLE_L)) != NULL) ||
             (strcasestr_P(currentTouchscreenSelection, PSTR(SM_HS_DISABLE_S)) != NULL)) {
    SERIAL_ECHOLNPGM("Special Menu: HighSpeed Mode DISABLED!");
    injectCommands(F("M401 S0\nM500"));
    BUZZ(210, 1661);
    BUZZ(105, 1108);
  } else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_BLTZ_EXIT_L)) != NULL) ||
             (strcasestr_P(currentTouchscreenSelection, PSTR(SM_BLTZ_EXIT_S)) != NULL)) {
    SERIAL_ECHOLNPGM("Special Menu: Exit BLTouch Menu & Save EEPROM");
    setSoftEndstopState(true);
    injectCommands(F("M500"));
    BUZZ(105, 1108);
    BUZZ(210, 1661);
    BLTouchMenu = false;
  }


    #endif
  else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_EZLVL_MENU_L)) != NULL) ||
           (strcasestr_P(currentTouchscreenSelection, PSTR(SM_EZLVL_MENU_S)) != NULL)) {
    SERIAL_ECHOLNPGM("Special Menu: Enter Easy Level Menu");
    LevelMenu = true;
    injectCommands(F("G28\nM420 S0\nG90\nG1 Z5\nG1 X15 Y15 F4000\nG1 Z0"));
  } else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_EZLVL_P1_L)) != NULL) ||
             (strcasestr_P(currentTouchscreenSelection, PSTR(SM_EZLVL_P1_S)) != NULL)) {
    SERIAL_ECHOLNPGM("Special Menu: Easy Level POINT 1");
    injectCommands(F("G90\nG1 Z5\nG1 X15 Y15 F4000\nG1 Z0"));
  } else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_EZLVL_P2_L)) != NULL) ||
             (strcasestr_P(currentTouchscreenSelection, PSTR(SM_EZLVL_P2_S)) != NULL)) {
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
  } else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_EZLVL_P3_L)) != NULL) ||
             (strcasestr_P(currentTouchscreenSelection, PSTR(SM_EZLVL_P3_S)) != NULL)) {
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
  } else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_EZLVL_P4_L)) != NULL) ||
             (strcasestr_P(currentTouchscreenSelection, PSTR(SM_EZLVL_P4_S)) != NULL)) {
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
  } else if ((strcasestr_P(currentTouchscreenSelection, PSTR(SM_EZLVL_EXIT_L)) != NULL) ||
             (strcasestr_P(currentTouchscreenSelection, PSTR(SM_EZLVL_EXIT_S)) != NULL)) {
    SERIAL_ECHOLNPGM("Special Menu: Exit Easy Level Menu");
    LevelMenu = false;
    injectCommands(F("G90\nG1 Z10\nG1 X15 Y15 F4000\nM420 S1"));
  }
  #endif // if ENABLED(KNUTWURST_SPECIAL_MENU)
}


void AnycubicTouchscreenClass::RenderCurrentFileList() {
  currentFileOrDirectory[0] = 0;
  uint16_t selectedNumber   = 0;
  FileList currentFileList;

  if (SpecialMenu == false) {
    currentTouchscreenSelection[0] = 0;
  }

  if (CodeSeen('S')) {
    selectedNumber = CodeValue();
  }

  // Filelist start
  SEND_PGM("FN ");
  SENDLINE_PGM("");

  if (!isMediaInserted() && !SpecialMenu) {
    SENDLINE_PGM(SM_SPECIAL_MENU_S);
    SENDLINE_PGM(SM_SPECIAL_MENU_L);
  } else {
    if (SpecialMenu) {
      RenderSpecialMenu(selectedNumber);
    } else if (selectedNumber <= currentFileList.count()) {
      RenderCurrentFolder(selectedNumber);
    }
  }
  // Filelist stop
  SEND_PGM("END");
  SENDLINE_PGM("");
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
  } else if (FlowMenu) {
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
  } else if (BLTouchMenu) {
    zOffsetBuffer = SM_BLTZ_DISP_L;

    #ifdef ANYCUBIC_TFT_DEBUG
    SERIAL_ECHOPGM("TFT Serial Debug: Current getZOffset_mm(): ", getZOffset_mm());
    SERIAL_EOL();
    #endif

    zOffsetBuffer.replace("XXXXX", String(getZOffset_mm()));

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
        SENDLINE_PGM(SM_HS_ENABLE_S);
        SENDLINE_PGM(SM_HS_ENABLE_L);
        SENDLINE_PGM(SM_HS_DISABLE_S);
        SENDLINE_PGM(SM_HS_DISABLE_L);
        SENDLINE_PGM(SM_BLTZ_EXIT_S);
        SENDLINE_PGM(SM_BLTZ_EXIT_L);
        break;

      default:
        break;
    }
  } else if (LevelMenu) {
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
  } else if (SpecialMenu) {
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

    #if NONE(KNUTWURST_BLTOUCH, KNUTWURST_CHIRON)
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

    #if ENABLED(KNUTWURST_CHIRON)
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
    SERIAL_ECHOLN("max_files = selectedNumber + 3;");
  #endif
  }

  if (filesOnSDCard == 3) {
    selectedNumber = 0;
  }

  for (uint16_t count = selectedNumber; count <= max_files; count++) {
    if (count == 0) { // Special Entry
      if (currentFileList.isAtRootDir()) {
        SENDLINE_PGM(SM_SPECIAL_MENU_S);
        SENDLINE_PGM(SM_SPECIAL_MENU_L);
  #ifdef ANYCUBIC_TFT_DEBUG
        SERIAL_ECHO(count);
        SERIAL_ECHO(": ");
        SERIAL_ECHOLNPGM(SM_SPECIAL_MENU_L);
  #endif
      } else {
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

      const char*  fileName    = currentFileList.filename();
      unsigned int fileNameLen = strlen(fileName);

  // Cut off too long filenames.
  // They don't fit on the screen anyway.
  #if ENABLED(KNUTWURST_DGUS2_TFT)
      bool fileNameWasCut = false;
      if (fileNameLen >= MAX_PRINTABLE_FILENAME_LEN) {
        fileNameWasCut = true;
        fileNameLen    = MAX_PRINTABLE_FILENAME_LEN;
      }
      static char outputString[MAX_PRINTABLE_FILENAME_LEN];
  #else
      char outputString[fileNameLen];
  #endif

      // Bugfix for non-printable special characters which are now replaced by underscores.
      for (unsigned int i = 0; i <= fileNameLen; i++) {
        if (isPrintable(fileName[i])) {
          outputString[i] = fileName[i];
        } else {
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
        outputString[MAX_PRINTABLE_FILENAME_LEN]     = '\0';
      } else if (currentFileList.isDir()) {
        for (unsigned char i = fileNameLen; i < MAX_PRINTABLE_FILENAME_LEN - 7; i++) {
          outputString[i] = ' ';
        }
        outputString[MAX_PRINTABLE_FILENAME_LEN - 7] = '\0';
      } else {
        for (unsigned char i = fileNameLen; i < MAX_PRINTABLE_FILENAME_LEN; i++) {
          outputString[i] = ' ';
        }
        // fix for .GCO files, which are not displayed correctly.
        if (outputString[fileNameLen - 4] == '.') {
          outputString[fileNameLen - 4] = '.';
          outputString[fileNameLen - 3] = 'g';
          outputString[fileNameLen - 2] = 'c';
          outputString[fileNameLen - 1] = 'o';
          outputString[fileNameLen]     = 'd';
          outputString[fileNameLen + 1] = 'e';
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

void AnycubicTouchscreenClass::SDCardStateChange(bool isInserted) {
  #if BOTH(SDSUPPORT, HAS_SD_DETECT)
  if (isInserted) {
    SENDLINE_DBG_PGM("J00", "TFT Serial Debug: SD card state changed... card inserted");
  } else {
    SENDLINE_DBG_PGM("J01", "TFT Serial Debug: SD card state changed... card removed");
  }
  #endif
}

void AnycubicTouchscreenClass::SDCardError() {
  #if ENABLED(ANYCUBIC_TFT_DEBUG)
  SERIAL_ECHOLNPGM("TFT Serial Debug: SDCardError event triggered...");
  #endif
  SENDLINE_DBG_PGM("J21", "TFT Serial Debug: SD Card Error ... J21");
}

void AnycubicTouchscreenClass::CheckHeaterError() {
  if ((getTargetTemp_celsius((extruder_t)E0) < 5) || (getTargetTemp_celsius((extruder_t)E0) > 300)) {
    if (HeaterCheckCount > 60000) {
      HeaterCheckCount = 0;
      SENDLINE_DBG_PGM("J10", "TFT Serial Debug: Hotend temperature abnormal... J10");
    } else {
      HeaterCheckCount++;
    }
  } else {
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
  // NOTE: getFilamentRunoutState() only returns the runout state if the job is
  // printing we want to actually check the status of the pin here, regardless
  // of printstate
  if (READ(FIL_RUNOUT_PIN)) {
    if (mediaPrintingState == AMPRINTSTATE_PRINTING || mediaPrintingState == AMPRINTSTATE_PAUSED ||
        mediaPrintingState == AMPRINTSTATE_PAUSE_REQUESTED) {
      // play tone to indicate filament is out
      injectCommands(F("\nM300 P200 S1567\nM300 P200 S1174\nM300 P200 "
                       "S1567\nM300 P200 S1174\nM300 P2000 S1567"));
      if ( millis() - time_last_cyclic_tft_command >= WAIT_MS_UNTIL_ACYCLIC_SEND ) {
        // tell the user that the filament has run out and wait
        SENDLINE_DBG_PGM("J23", "TFT Serial Debug: Blocking filament prompt... J23");
      }
      else
        delayed_tft_command = 23;
    } else {
      SENDLINE_DBG_PGM("J15", "TFT Serial Debug: Non blocking filament runout... J15");
    }
  }
  #endif // FILAMENT_RUNOUT_SENSOR
}

void AnycubicTouchscreenClass::UserConfirmRequired(const char* const msg) {
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
   * NOTE:  The only way to handle these states is strcmp_P with the msg
   * unfortunately (very expensive)
   */
  if (strcmp_P(msg, PSTR("Nozzle Parked")) == 0) {
    mediaPrintingState = AMPRINTSTATE_PAUSED;
    mediaPauseState    = AMPAUSESTATE_PARKED;
    if ( millis() - time_last_cyclic_tft_command >= WAIT_MS_UNTIL_ACYCLIC_SEND ) {
      // enable continue button
      SENDLINE_DBG_PGM("J18", "TFT Serial Debug: UserConfirm SD print paused done... J18");
    }
    else
      delayed_tft_command = 18;
  } else if (strcmp_P(msg, PSTR("Load Filament")) == 0) {
    mediaPrintingState = AMPRINTSTATE_PAUSED;
    mediaPauseState    = AMPAUSESTATE_FILAMENT_OUT;
    if ( millis() - time_last_cyclic_tft_command >= WAIT_MS_UNTIL_ACYCLIC_SEND ) {
      // enable continue button
      SENDLINE_DBG_PGM("J18", "TFT Serial Debug: UserConfirm Filament is out... J18");
      SENDLINE_DBG_PGM("J23", "TFT Serial Debug: UserConfirm Blocking filament prompt... J23");
    }
    else
      delayed_tft_command = 118;
  } else if (strcmp_P(msg, PSTR("Filament Purging...")) == 0) {
    mediaPrintingState = AMPRINTSTATE_PAUSED;
    mediaPauseState    = AMPAUSESTATE_FILAMENT_PURGING;
    if ( millis() - time_last_cyclic_tft_command >= WAIT_MS_UNTIL_ACYCLIC_SEND ) {
      // enable continue button
      SENDLINE_DBG_PGM("J18", "TFT Serial Debug: UserConfirm Filament is purging... J18");
    }
    else
      delayed_tft_command = 18;
  } else if (strcmp_P(msg, PSTR("HeaterTimeout")) == 0) {
    mediaPrintingState = AMPRINTSTATE_PAUSED;
    mediaPauseState    = AMPAUSESTATE_HEATER_TIMEOUT;
    if ( millis() - time_last_cyclic_tft_command >= WAIT_MS_UNTIL_ACYCLIC_SEND ) {
      // enable continue button
      SENDLINE_DBG_PGM("J18", "TFT Serial Debug: UserConfirm SD Heater timeout... J18");
    }
    else
      delayed_tft_command = 18;
  } else if (strcmp_P(msg, PSTR("Reheat finished.")) == 0) {
    mediaPrintingState = AMPRINTSTATE_PAUSED;
    mediaPauseState    = AMPAUSESTATE_REHEAT_FINISHED;
    if ( millis() - time_last_cyclic_tft_command >= WAIT_MS_UNTIL_ACYCLIC_SEND ) {
      // enable continue button
      SENDLINE_DBG_PGM("J18", "TFT Serial Debug: UserConfirm SD Reheat done... J18");
    }
    else
      delayed_tft_command = 18;
  }
  #endif
}

static boolean TFTcomment_mode = false;

void AnycubicTouchscreenClass::GetCommandFromTFT() {
  char* starpos = NULL;
  while (LCD_SERIAL.available() > 0 && TFTbuflen < TFTBUFSIZE) {
    serial3_char = LCD_SERIAL.read();
    if (serial3_char == '\n' || serial3_char == '\r' || (serial3_char == ':' && !TFTcomment_mode) ||
        serial3_count >= (TFT_MAX_CMD_SIZE - 1)) {
      if (!serial3_count) {      // if empty line
        TFTcomment_mode = false; // for new command
        return;
      }

      TFTcmdbuffer[TFTbufindw][serial3_count] = 0; // terminate string

      if (!TFTcomment_mode) {
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
              SEND_PGM("A5V X: ");
              LCD_SERIAL.print(current_position[X_AXIS]);
              SEND_PGM(" Y: ");
              LCD_SERIAL.print(current_position[Y_AXIS]);
              SEND_PGM(" Z: ");
              LCD_SERIAL.print(current_position[Z_AXIS]);
              SENDLINE_PGM("");
              break;

            case 6: // A6 GET SD CARD PRINTING STATUS
  #ifdef SDSUPPORT
              if (isPrintingFromMedia()) {
                SEND_PGM("A6V ");
                if (isMediaInserted()) {
                  SENDLINE(ui8tostr3rj(getProgress_percent()));
                } else {
                  SENDLINE_DBG_PGM("J02", "TFT Serial Debug: No SD Card mounted to return "
                                          "printing status... J02");
                }
              } else {
                SENDLINE_PGM("A6V ---");
              }
  #endif
              break;

            case 7: // A7 GET PRINTING TIME
              {
                const uint32_t elapsedSeconds = getProgress_seconds_elapsed();
                SEND_PGM("A7V ");
                if (elapsedSeconds != 0) { // print time
                  const uint32_t elapsedMinutes = elapsedSeconds / 60;
                  SEND(ui8tostr2(elapsedMinutes / 60));
                  SEND_PGM(" H ");
                  SEND(ui8tostr2(elapsedMinutes % 60));
                  SEND_PGM(" M");
                } else {
                  SEND_PGM(" 999:999");
                }
                SENDLINE_PGM("");
              }
              break;

            case 8: // A8 GET SD LIST
  #ifdef SDSUPPORT
              RenderCurrentFileList();
  #endif
              break;

            case 9: // A9 pause sd print
  #if ENABLED(SDSUPPORT)
              if (isPrintingFromMedia()) {
                PausePrint();
              }
  #endif
              break;

            case 10: // A10 resume sd print
  #if ENABLED(SDSUPPORT)
              if (isPrintingFromMediaPaused()) {
                ResumePrint();
              }
  #endif
              break;

            case 11: // A11 stop sd print
              TERN_(SDSUPPORT, StopPrint());
              #if ENABLED(KNUTWURST_MEGA_P_LASER)
                laser_print_step = 0;
                if (laser_status == 1) {
                  WRITE(HEATER_0_PIN, 0);
                  laser_status      = 0;
                  laser_print_step  = 0;
                  laser_print_pause = 0;
                }
              #endif
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
                } else if (TFTstrchr_pointer[4] == '<') {
                  strcpy(currentTouchscreenSelection, TFTstrchr_pointer + 4);
                  SpecialMenu               = true;
                  currentFileOrDirectory[0] = 0;
                  SENDLINE_DBG_PGM("J21", "TFT Serial Debug: Clear file selection... J21 "); // J21 Not File Selected
                  SENDLINE_PGM("");
                } else {
                  currentTouchscreenSelection[0] = 0;
                  if (starpos) {
                    *(starpos - 1) = '\0';
                  }
                  strcpy(currentFileOrDirectory, TFTstrchr_pointer + 4);
                  SENDLINE_DBG_PGM_VAL("J20", "TFT Serial Debug: File Selected... J20 ",
                                       currentFileOrDirectory); // J20 File Selected
                }
              }
  #endif
              break;

            case 14: // A14 START PRINTING
  #if ENABLED(SDSUPPORT)
              if (!isPrinting()) {
                #if ENABLED(KNUTWURST_MEGA_P_LASER)
                  file_type = 0;
                  if (strstr(TFTstrchr_pointer, ".bmp")) file_type = 1;

                  if (file_type == 1) {
                    // BMP file selected for laser engraving.
                    card.openFileRead(currentFileOrDirectory);

                    // Read BMP header.
                    card.setIndex(0);
                    card.read(st_bmp.bfType, 2);
                    card.read(st_bmp.bfSize, 4);
                    card.read(st_bmp.bfReserved1, 2);
                    card.read(st_bmp.bfReserved2, 2);
                    card.read(st_bmp.bfOffBits, 4);
                    card.read(st_bmp.biSize, 4);
                    card.read(st_bmp.biWidth, 4);
                    card.read(st_bmp.biHeight, 4);
                    card.read(st_bmp.biPlanes, 2);
                    card.read(st_bmp.biBitCount, 2);
                    card.read(st_bmp.biCompression, 4);
                    card.read(st_bmp.biSizeImage, 4);
                    card.read(st_bmp.biXPelsPerMeter, 4);
                    card.read(st_bmp.biYPelsPerMeter, 4);
                    card.read(st_bmp.biClrUsed, 4);
                    card.read(st_bmp.biClrImportant, 4);

                    laser_printer_st.pic_start = (unsigned long)st_bmp.bfOffBits[0] + ((unsigned long)st_bmp.bfOffBits[1] << 8) + ((unsigned long)st_bmp.bfOffBits[2] << 16) + ((unsigned long)st_bmp.bfOffBits[3] << 24);
                    laser_printer_st.pic_width = (unsigned int)st_bmp.biWidth[0] + ((unsigned int)st_bmp.biWidth[1] << 8);
                    laser_printer_st.pic_height = (unsigned int)st_bmp.biHeight[0] + ((unsigned int)st_bmp.biHeight[1] << 8);
                    laser_printer_st.pic_bit = (unsigned char)st_bmp.biBitCount[0];

                    if (laser_printer_st.pic_bit == 32) {
                      laser_printer_st.pic_real_width = laser_printer_st.pic_width * 4;
                    }
                    else if (laser_printer_st.pic_bit == 24) {
                      laser_printer_st.pic_real_width = laser_printer_st.pic_width * 3;
                      if (laser_printer_st.pic_real_width % 4 != 0) {
                        laser_printer_st.pic_real_width = (laser_printer_st.pic_real_width / 4 + 1) * 4;
                      }
                    }
                    else if (laser_printer_st.pic_bit == 16) {
                      laser_printer_st.pic_real_width = laser_printer_st.pic_width * 2;
                    }

                    send_pic_param();
                  }
                  else
                #endif
                  {
                    StartPrint();
                  }
              }
  #endif
              break;

            case 15: // A15 RESUMING FROM OUTAGE
  #if defined(POWER_OUTAGE_TEST)
              if (!isPrinting()) {
                if (card.isFileOpen()) {
                  FlagResumFromOutage = true;
                }

                ResumingFlag = 1;
                resumePrint();
                SENDLINE_PGM("OK");
              }
  #endif
              #if ENABLED(KNUTWURST_MEGA_P_LASER)
                laser_on_off = 0;
              #endif
              break;

            case 16: // A16 set hotend temp
              {
                unsigned int tempvalue;
                if (CodeSeen('S')) {
                  tempvalue = constrain(CodeValue(), 0, 260);
                  if (getTargetTemp_celsius((extruder_t)E0) <= 260) {
                    setTargetTemp_celsius(tempvalue, (extruder_t)E0);
                  } // do not set Temp from TFT if it is set via gcode
                } else if ((CodeSeen('C')) && (!isPrinting())) {
                  if ((getAxisPosition_mm(Z) < 10)) {
                    injectCommands(F("G1 Z10")); // RASE Z AXIS
                  }
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
                  if (getTargetTemp_celsius((heater_t)BED) <= 100) {
                    setTargetTemp_celsius(tempbed, (heater_t)BED); // do not set Temp from TFT if it is set via gcode
                  }
                }
              }
              break;

            case 18: // A18 set fan speed
              float fanPercent;
              if (CodeSeen('S')) {
                fanPercent = CodeValue();
                fanPercent = constrain(fanPercent, 0, 100);
                setTargetFan_percent(fanPercent, FAN0);
              } else {
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
                if (CodeSeen('S')) {
                  feedrate_percentage = constrain(CodeValue(), 40, 999);
                } else {
                  SEND_PGM_VAL("A20V ", feedrate_percentage);
                  SENDLINE_PGM("");
                  time_last_cyclic_tft_command = millis();
                }
              }
              break;

            case 21: // A21 all home
              if (!isPrinting() && !isPrintingFromMediaPaused()) {
                if (CodeSeen('X') || CodeSeen('Y') || CodeSeen('Z')) {
                  if (CodeSeen('X')) {
                    injectCommands(F("G28X"));
                  }
                  if (CodeSeen('Y')) {
                    injectCommands(F("G28Y"));
                  }
                  if (CodeSeen('Z')) {
                    injectCommands(F("G28Z"));
                  }
                } else if (CodeSeen('C')) {
                  injectCommands_P(G28_STR);
                }
              }
              break;

            case 22: // A22 move X/Y/Z or extrude
              if (!isPrinting()) {
                float    coorvalue;
                uint16_t movespeed = 0;
                char     commandStr[30];
                char     fullCommandStr[38];

                commandStr[0] = 0;   // empty string
                if (CodeSeen('F')) { // Set feedrate
                  movespeed = CodeValue();
                }

                if (CodeSeen('X')) { // Move in X direction
                  coorvalue = CodeValue();
                  if ((coorvalue <= 0.2) && coorvalue > 0) {
                    sprintf_P(commandStr, PSTR("G1 X0.1F%i"), movespeed);
                  } else if ((coorvalue <= -0.1) && coorvalue > -1) {
                    sprintf_P(commandStr, PSTR("G1 X-0.1F%i"), movespeed);
                  } else {
                    sprintf_P(commandStr, PSTR("G1 X%iF%i"), int(coorvalue), movespeed);
                  }
                } else if (CodeSeen('Y')) { // Move in Y direction
                  coorvalue = CodeValue();
                  if ((coorvalue <= 0.2) && coorvalue > 0) {
                    sprintf_P(commandStr, PSTR("G1 Y0.1F%i"), movespeed);
                  } else if ((coorvalue <= -0.1) && coorvalue > -1) {
                    sprintf_P(commandStr, PSTR("G1 Y-0.1F%i"), movespeed);
                  } else {
                    sprintf_P(commandStr, PSTR("G1 Y%iF%i"), int(coorvalue), movespeed);
                  }
                } else if (CodeSeen('Z')) { // Move in Z direction
                  coorvalue = CodeValue();
                  if ((coorvalue <= 0.2) && coorvalue > 0) {
                    sprintf_P(commandStr, PSTR("G1 Z0.1F%i"), movespeed);
                  } else if ((coorvalue <= -0.1) && coorvalue > -1) {
                    sprintf_P(commandStr, PSTR("G1 Z-0.1F%i"), movespeed);
                  } else {
                    sprintf_P(commandStr, PSTR("G1 Z%iF%i"), int(coorvalue), movespeed);
                  }
                } else if (CodeSeen('E')) { // Extrude
                  coorvalue = CodeValue();
                  if ((coorvalue <= 0.2) && coorvalue > 0) {
                    sprintf_P(commandStr, PSTR("G1 E0.1F%i"), movespeed);
                  } else if ((coorvalue <= -0.1) && coorvalue > -1) {
                    sprintf_P(commandStr, PSTR("G1 E-0.1F%i"), movespeed);
                  } else {
                    sprintf_P(commandStr, PSTR("G1 E%iF500"), int(coorvalue));
                  }
                }

                if (strlen(commandStr) > 0) {
                  sprintf_P(fullCommandStr, PSTR("G91\n%s\nG90"), commandStr);
  #if ENABLED(ANYCUBIC_LCD_DEBUG)
                  SERIAL_ECHOPGM("TFT Serial Debug: A22 Move final request with gcode... ");
                  SERIAL_ECHOLN(fullCommandStr);
  #endif
                  injectCommands(fullCommandStr);
                }
              }
              SENDLINE_PGM("");
              break;

            case 23: // A23 preheat pla
              if (!isPrinting()) {
                if (getAxisPosition_mm(Z) < 10) {
                  injectCommands(F("G1 Z10")); // RASE Z AXIS
                }

                setTargetTemp_celsius(PREHEAT_1_TEMP_BED, (heater_t)BED);
                setTargetTemp_celsius(PREHEAT_1_TEMP_HOTEND, (extruder_t)E0);
                SENDLINE_PGM("OK");
              }
              break;

            case 24: // A24 preheat abs
              if (!isPrinting()) {
                if (getAxisPosition_mm(Z) < 10) {
                  injectCommands(F("G1 Z10")); // RASE Z AXIS
                }

                setTargetTemp_celsius(PREHEAT_2_TEMP_BED, (heater_t)BED);
                setTargetTemp_celsius(PREHEAT_2_TEMP_HOTEND, (extruder_t)E0);
                SENDLINE_PGM("OK");
              }
              break;

            case 25: // A25 cool down
              if (!isPrinting()) {
                setTargetTemp_celsius(0, (heater_t)BED);
                setTargetTemp_celsius(0, (extruder_t)E0);

                SENDLINE_DBG_PGM("J12", "TFT Serial Debug: Cooling down... J12"); // J12 cool down
              }
              break;

            case 26: // A26 refresh SD
              {
  #ifdef SDSUPPORT
                FileList currentFileList;

                if (currentTouchscreenSelection[0] == '<') {
    #ifdef ANYCUBIC_TFT_DEBUG
                  SERIAL_ECHOLNPGM("TFT Serial Debug: Enter Special Menu");
    #endif
                  HandleSpecialMenu();
                } else if (((strcasestr_P(currentFileOrDirectory, PSTR(SM_DIR_UP_S)) != NULL) ||
                            (strcasestr_P(currentFileOrDirectory, PSTR(SM_DIR_UP_L)) != NULL)) &&
                           isMediaInserted()) {
    #ifdef ANYCUBIC_TFT_DEBUG
                  SERIAL_ECHOLNPGM("TFT Serial Debug: Directory UP (cd ..)");
    #endif
                  currentFileList.upDir();
                } else if (isMediaInserted() && (currentFileList.count() > 0)) {
    #ifdef ANYCUBIC_TFT_DEBUG
                  SERIAL_ECHOLNPGM("TFT Serial Debug: Enter Directoy");
    #endif
    #if ENABLED(KNUTWURST_DGUS2_TFT)
                  strcpy(currentFileOrDirectory, currentTouchscreenSelection);
                  int currentFileLen                         = strlen(currentFileOrDirectory);
                  currentFileOrDirectory[currentFileLen - 4] = '\0';
                  currentFileList.changeDir(currentFileOrDirectory);
    #else
                  currentFileList.changeDir(currentTouchscreenSelection);
    #endif
                } else if (currentTouchscreenSelection[0] == 0 && isMediaInserted()) {
                  card.mount();
                }
                if (SpecialMenu == false) {
                  currentTouchscreenSelection[0] = 0;
                }
  #endif // ifdef SDSUPPORT
              }
              break;

  #if ENABLED(KNUTWURST_4MAXP2)
    #ifdef SERVO_ENDSTOPS
            case 27: // a27 servos angles  adjust
              if (!isPrintingFromMedia()) {
                char value[30];
                planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], 20, current_position[E_AXIS],
                                    10, active_extruder);
                stepper.synchronize();
                SEND_PGM("A27V ");
                SEND_PGM("R ");
                SEND(RiseAngles);
                SEND(" ");
                SEND_PGM("F ");
                SEND(FallAngles);
                SEND(" ");
                if (CodeSeen('R')) {
                  RiseAngles = CodeValue();
                }
                if (CodeSeen('F')) {
                  FallAngles = CodeValue();
                }
                if (CodeSeen('O')) {
                  SaveMyServoAngles();
                  delay(200);
                  servos[0].detach();
                }
              }
              SENDLINE_PGM("");
              break;
    #endif
  #endif // KNUTWURST_4MAXP2

            case 28: // A28 filament test
              {
                if (CodeSeen('O'))
                  ;
                else if (CodeSeen('C'))
                  ;
              }
              SENDLINE_PGM("");
              break;
  #if ENABLED(KNUTWURST_4MAXP2)
    #ifdef SERVO_ENDSTOPS
            case 29: // A29 Z PROBE OFFESET SET
              {
                SEND_PGM("The past value:");
                SEND(MY_Z_PROBE);
                if (CodeSeen('S')) {
                  MY_Z_PROBE_OFFSET_FROM_EXTRUDER = CodeValue();
                  SaveMyServoAngles();
                }
              }
              SENDLINE_PGM("OK");
              break;
    #endif
  #endif // KNUTWURST_4MAXP2

  #if ENABLED(KNUTWURST_CHIRON)
            case 29: // A29 bed grid read
              {
                xy_uint8_t pos;
                float      pos_z;

                if (CodeSeen('X')) {
                  pos.x = CodeValueInt();
                }
                if (CodeSeen('Y')) {
                  pos.y = CodeValueInt();
                }

                pos_z = getMeshPoint(pos);

                SEND_PGM("A29V ");
                LCD_SERIAL.print(pos_z * 100, 2);
                SENDLINE_PGM("");

                if (!isPrinting()) {
                  setSoftEndstopState(true);
                  if ((selectedmeshpoint.x == pos.x) && (selectedmeshpoint.y == pos.y)) {
                    if (!isPositionKnown()) {
                      injectCommands_P(G28_STR);
                    }

                    if (isPositionKnown()) {
    #if ENABLED(ANYCUBIC_TFT_DEBUG)
                      SERIAL_ECHOLNPGM("Moving to mesh point at x: ", pos.x, " y: ", pos.y, " z: ", pos_z);
    #endif
                      setAxisPosition_mm(5.0, Z);
                      setAxisPosition_mm(17 + (93 * pos.x), X);
                      setAxisPosition_mm(20 + (93 * pos.y), Y);
                      setAxisPosition_mm(0.0, Z);
    #if ENABLED(ANYCUBIC_TFT_DEBUG)
                      SERIAL_ECHOLNPGM("Current Z: ", getAxisPosition_mm(Z));
    #endif
                    }
                  }
                  selectedmeshpoint.x = pos.x;
                  selectedmeshpoint.y = pos.y;
                }
              }
              break;

            case 30: // A30 auto leveling (Old Anycubic TFT)
              if (CodeSeen('S')) {
                if (isPrinting()) {
                  SENDLINE_DBG_PGM("J24", "TFT Serial Debug: Forbid auto leveling... J24");
                } else {
                  SENDLINE_DBG_PGM("J26", "TFT Serial Debug: Start auto leveling... J26");
                  // injectCommands(F("G28\nG29"));
                  injectCommands(F("G28\nG29\nM500\nG90\nM300 S440 P200\nM300 "
                                   "S660 P250\nM300 S880 P300\nG1 Z30 "
                                   "F4000\nG1 X5 F4000\nG91\nM84\nM420 S1"));
                  mediaPrintingState = AMPRINTSTATE_PROBING;
                }
              } else {
                SENDLINE_DBG_PGM("J26", "TFT Serial Debug: Enable level menu... J26");
              }
              break;

            case 31:               // A31 z-offset
              if (CodeSeen('C')) { // Restore and apply original offsets
                if (!isPrinting()) {
                  injectCommands(F("M501\nM420 S1"));
                  selectedmeshpoint.x = selectedmeshpoint.y = 99;
                  SERIAL_ECHOLNF(F("Mesh changes abandoned, previous mesh restored."));
                }
              }

              else if (CodeSeen('D')) { // Save Z Offset tables and restore leveling state
                if (!isPrinting()) {
                  setAxisPosition_mm(5.0, Z); // Lift nozzle before any further movements are made
                  injectCommands(F("M500"));
    #if ENABLED(ANYCUBIC_TFT_DEBUG)
                  SERIAL_ECHOLNF(F("Mesh changes saved."));
    #endif
                  selectedmeshpoint.x = selectedmeshpoint.y = 99;
                }
              }

              else if (CodeSeen('G')) { // Get current offset
                if (isPrinting()) {
                  SEND_PGM("A31V ");
                  LCD_SERIAL.print(live_Zoffset, 2);
                  SENDLINE_PGM("");
                } else {
                  SEND_PGM("A31V ");
                  LCD_SERIAL.print(getZOffset_mm(), 2);
                  SENDLINE_PGM("");
                  selectedmeshpoint.x = selectedmeshpoint.y = 99;
                }
              }

              else {
                if (CodeSeen('S')) { // Set offset (adjusts all points by value)
                  float Zshift = CodeValue();
                  setSoftEndstopState(false);
                  if (isPrinting()) {
    #if ENABLED(ANYCUBIC_TFT_DEBUG)
                    SERIAL_ECHOLNPGM("Change Zoffset from:", live_Zoffset, " to ", live_Zoffset + Zshift);
    #endif
                    if (isAxisPositionKnown(Z)) {
    #if ENABLED(ANYCUBIC_TFT_DEBUG)
                      const float currZpos = getAxisPosition_mm(Z);
                      SERIAL_ECHOLNPGM("Nudge Z pos from ", currZpos, " to ",
                                       currZpos + constrain(Zshift, -0.05, 0.05));
    #endif
                      int16_t steps = mmToWholeSteps(constrain(Zshift, -0.05, 0.05), Z);
    #if ENABLED(ANYCUBIC_TFT_DEBUG)
                      SERIAL_ECHOLNPGM("Steps to move Z: ", steps);
    #endif
                      babystepAxis_steps(steps, Z);
                      live_Zoffset += Zshift;
                    }

                    SEND_PGM("A31V ");
                    LCD_SERIAL.print(live_Zoffset, 2);
                    SENDLINE_PGM("");
                  } else {
                    GRID_LOOP(x, y) {
                      const xy_uint8_t pos{x, y};
                      const float      currval = getMeshPoint(pos);
                      setMeshPoint(pos, constrain(currval + Zshift, -10, 2));
    #if ENABLED(ANYCUBIC_TFT_DEBUG)
                      SERIAL_ECHOLNPGM("Change mesh point X", x, " Y", y, " from ", currval, " to ", getMeshPoint(pos));
    #endif
                    }
                    const float currZOffset = getZOffset_mm();
    #if ENABLED(ANYCUBIC_TFT_DEBUG)
                    SERIAL_ECHOLNPGM("Change probe offset from ", currZOffset, " to  ", currZOffset + Zshift);
    #endif

                    setZOffset_mm(currZOffset + Zshift);

                    SEND_PGM("A31V ");
                    LCD_SERIAL.print(getZOffset_mm(), 2);
                    SENDLINE_PGM("");

                    if (isAxisPositionKnown(Z)) {
                      const float currZpos = getAxisPosition_mm(Z);
    #if ENABLED(ANYCUBIC_TFT_DEBUG)
                      SERIAL_ECHOLNPGM("Move Z pos from ", currZpos, " to ", currZpos + constrain(Zshift, -0.05, 0.05));
    #endif
                      setAxisPosition_mm(currZpos + constrain(Zshift, -0.05, 0.05), Z);
                    }
                  }
                }
              }
              break;

  #endif // if ENABLED(KNUTWURST_CHIRON)

  #if ENABLED(KNUTWURST_4MAXP2)

            case 30: // a30 assist leveling
              SEND_PGM("J22"); // level watching finish
              SENDLINE_PGM("");

              if (!isPrintingFromMedia()) {

                if (CodeSeen('S')) {
                  injectCommands(F("G28\nM420 S0\nG90\nG1 Z5\nG1 X15 Y15 F4000\nG1 Z0"));
                } else if (CodeSeen('O')) {
                  injectCommands(F("G90\nG1 Z5\nG1 X15 Y15 F4000\nG1 Z0"));
                } else if (CodeSeen('T')) {
                  injectCommands(F("G90\nG1 Z5\nG1 X255 Y15 F4000\nG1 Z0"));
                } else if (CodeSeen('C')) {
                  injectCommands(F("G90\nG1 Z5\nG1 X255 Y195 F4000\nG1 Z0"));
                } else if (CodeSeen('Q')) {
                  injectCommands(F("G90\nG1 Z5\nG1 X15 Y195 F4000\nG1 Z0"));
                } else if (CodeSeen('H') || CodeSeen('L')) {
                  if (CodeSeen('L')) {
                    injectCommands(F("G90\nG1 Z10\nG1 X15 Y15 F4000\nM420 S1"));
                  }
                }
              }
              SENDLINE_PGM("");

              break;

            case 31: // a31 zoffset
              if (!isPrintingFromMedia()) {

                if (CodeSeen('S')) {
                  SENDLINE_PGM("A9V ");
                  LCD_SERIAL.print(getZOffset_mm() * 100, 2);
                  SENDLINE_PGM("");
                }
                if (CodeSeen('D')) {
                  const_float_t newOffset = CodeValue() / 100;
                  setZOffset_mm(newOffset);
                  injectCommands(F("M500"));
                }
              }
              SENDLINE_PGM("");
              break;
  #endif // #if ENABLED(KNUTWURST_4MAXP2)

            case 32: // a32 clean leveling beep flag
              break;

            case 33: // A33 get version info
              SEND_PGM("J33 ");
              SEND_PGM("KW-");
              SEND_PGM(MSG_MY_VERSION);
              SENDLINE_PGM("");
              break;

  #if ENABLED(KNUTWURST_MEGA_P_LASER)
            case 34: // A34 set laser vector
              if (CodeSeen('V')) {
                laser_printer_st.pic_vector = CodeValueInt();
                send_laser_param();
              }
              break;

            case 35: // A35 set laser x mirror
              if (CodeSeen('V')) {
                laser_printer_st.pic_x_mirror = CodeValueInt();
                send_laser_param();
              }
              break;

            case 36: // A36 set laser y mirror
              if (CodeSeen('V')) {
                laser_printer_st.pic_y_mirror = CodeValueInt();
                send_laser_param();
              }
              break;

            case 37: // A37 set laser time
              if (CodeSeen('V')) {
                laser_printer_st.pic_laser_time = CodeValueInt();
                send_laser_param();
              }
              break;

            case 38: // A38 set laser height
              if (CodeSeen('V')) {
                laser_printer_st.laser_height = CodeValue();
                send_laser_param();
              }
              break;

            case 39: // A39 set laser pixel distance
              if (CodeSeen('V')) {
                laser_printer_st.pic_pixel_distance = CodeValue();
                send_laser_param();
              }
              break;

            case 40: // A40 set laser x offset
              if (CodeSeen('V')) {
                laser_printer_st.x_offset = CodeValue();
                send_laser_param();
              }
              break;

            case 41: // A41 set laser y offset
              if (CodeSeen('V')) {
                laser_printer_st.y_offset = CodeValue();
                send_laser_param();
              }
              break;

            case 42: // A42 laser on/off
              if (CodeSeen('V')) {
                laser_on_off = CodeValueInt();
              }
              break;

            case 43: // A43 start laser print
              en_continue = 1;
              break;

            case 46: // A46 pause laser print
              if (CodeSeen('V')) {
                laser_print_pause = CodeValueInt();
              }
              break;
  #endif // KNUTWURST_MEGA_P_LASER

  #if ENABLED(KNUTWURST_CHIRON)
            case 34: // a34 bed grid write
              {
                xy_uint8_t pos;
                if (CodeSeen('X')) {
                  pos.x = constrain(CodeValueInt(), 0, GRID_MAX_POINTS_X);
                }
                if (CodeSeen('Y')) {
                  pos.y = constrain(CodeValueInt(), 0, GRID_MAX_POINTS_Y);
                }

                float currmesh = getMeshPoint(pos);

                if (CodeSeen('V')) {
                  float newval = float(constrain(CodeValue() / 100, -10, 10));
    #if ENABLED(ANYCUBIC_TFT_DEBUG)
                  SERIAL_ECHOLNPGM("Change mesh point x:", pos.x, " y:", pos.y);
                  SERIAL_ECHOLNPGM("from ", currmesh, " to ", newval);
    #endif
                  setMeshPoint(pos, newval);
                  if (mediaPrintingState == AMPRINTSTATE_NOT_PRINTING || mediaPrintingState == AMPRINTSTATE_PROBING) {
                    if (selectedmeshpoint.x == pos.x && selectedmeshpoint.y == pos.y) {
                      setSoftEndstopState(false);
                      float currZpos = getAxisPosition_mm(Z);
    #if ENABLED(ANYCUBIC_TFT_DEBUG)
                      SERIAL_ECHOLNPGM("Move Z pos from ", currZpos, " to ",
                                       currZpos + constrain(newval - currmesh, -0.05, 0.05));
    #endif
                      setAxisPosition_mm(currZpos + constrain(newval - currmesh, -0.05, 0.05), Z);
                    }
                  }
                }
                if (CodeSeen('S')) {
                  injectCommands(F("M500"));
                }
                if (CodeSeen('C')) {
                  injectCommands(F("M501\nM420 S1"));
                  selectedmeshpoint.x = selectedmeshpoint.y = 99;
                }
              }
              break;

            case 35: // RESET AUTOBED DATE //M1000
              // initializeGrid();  //done via special menu
              break;

            case 36: // A36 auto leveling (New Anycubic TFT)
              SENDLINE_DBG_PGM("J26", "TFT Serial Debug: Start auto leveling... J26");
              break;
  #endif // if ENABLED(KNUTWURST_CHIRON)

  #if ENABLED(KNUTWURST_4MAXP2)
            case 40:
              SENDLINE_DBG_PGM("J17", "TFT Serial Debug: Main board reset... J17"); // J17 Main board reset
              delay(10);
              break;

            case 41:
              if (CodeSeen('O')) {
                PrintdoneAndPowerOFF = true;
    #if ENABLED(ANYCUBIC_TFT_DEBUG)
                SERIAL_ECHOLNPGM("TFT Serial Debug: PrintdoneAndPowerOFF is set!");
    #endif
                break;
              } else if (CodeSeen('C')) {
                PrintdoneAndPowerOFF = false;
    #if ENABLED(ANYCUBIC_TFT_DEBUG)
                SERIAL_ECHOLNPGM("TFT Serial Debug: PrintdoneAndPowerOFF is disabled!");
    #endif
                break;
              }
              if (CodeSeen('S')) {
                if (PrintdoneAndPowerOFF) {
    #if ENABLED(ANYCUBIC_TFT_DEBUG)
                  SERIAL_ECHOLNPGM("TFT Serial Debug: Sending J35 because "
                                   "PrintdoneAndPowerOFF = true");
    #endif
                  SEND_PGM("J35 ");
                  SENDLINE_PGM("");
                } else {
    #if ENABLED(ANYCUBIC_TFT_DEBUG)
                  SERIAL_ECHOLNPGM("TFT Serial Debug: Sending J34 because "
                                   "PrintdoneAndPowerOFF = false");
    #endif
                  SEND_PGM("J34 ");
                  SENDLINE_PGM("");
                }
              }
              break;

            case 42:
              if (CaseLight == true) {
                SERIAL_ECHOLNPGM("Case Light OFF");
                injectCommands(F("M355 S1 P0"));
                CaseLight = false;
              } else {
                SERIAL_ECHOLNPGM("Case Light ON");
                injectCommands(F("M355 S1 P255"));
                CaseLight = true;
              }
              break;

  #endif
  #if ENABLED(KNUTWURST_DGUS2_TFT)
            case 50:
              SENDLINE_PGM("J38 ");
              break;
  #endif

  #if ENABLED(KNUTWURST_MEGA_P)
            case 51:
              if (CodeSeen('H')) {
                injectCommands(F(
                  "G1 Z5 F500\n"
                  "G1 X30 Y30 F5000\n"
                  "G1 Z0.15 F300"
                ));
              } else if (CodeSeen('I')) {
                injectCommands(F(
                  "G1 Z5 F500\n"
                  "G1 X190 Y30 F5000\n"
                  "G1 Z0.15 F300"
                ));
              } else if (CodeSeen('J')) {
                injectCommands(F(
                  "G1 Z5 F500\n"
                  "G1 X190 Y190 F5000\n"
                  "G1 Z0.15 F300"
                ));
              } else if (CodeSeen('K')) {
                injectCommands(F(
                  "G1 Z5 F500\n"
                  "G1 X30 Y190 F5000\n"
                  "G1 Z0.15 F300"
                ));
              } else if (CodeSeen('L')) {
                injectCommands(F("G1 X100 Y100 Z50 F5000"));
              }
              break;
  #endif

            default:
              break;
          } // switch
        }
        TFTbufindw  = (TFTbufindw + 1) % TFTBUFSIZE;
        TFTbuflen  += 1;
      } // if (!TFTcomment_mode)
      serial3_count = 0; // clear buffer
    } else {
      if (serial3_char == ';') {
        TFTcomment_mode = true;
      }
      if (!TFTcomment_mode) {
        TFTcmdbuffer[TFTbufindw][serial3_count++] = serial3_char;
      }
    }
  } // while
}

  #if ENABLED(KNUTWURST_4MAXP2)
  void PowerDown() {
    #if ENABLED(ANYCUBIC_TFT_DEBUG)
    SERIAL_ECHOLNPGM("TFT Serial Debug: PowerDown is called!");
    #endif
    for (unsigned char i = 0; i < 3; i++) {
      WRITE(POWER_OFF_PIN, LOW);
      delay(10);
      WRITE(POWER_OFF_PIN, HIGH);
      delay(10);
    }
  }
  #endif

#if ENABLED(KNUTWURST_MEGA_P_LASER)
  /**
   * Scan for TFT commands during laser printing.
   * This is a simplified version of CommandScan() that only processes incoming commands
   * without the full state machine logic.
   */
  void AnycubicTouchscreenClass::TFTCommandScan() {
    if (TFTbuflen < (TFTBUFSIZE - 1)) {
      GetCommandFromTFT();
    }

    if (TFTbuflen) {
      TFTbuflen  = (TFTbuflen - 1);
      TFTbufindr = (TFTbufindr + 1) % TFTBUFSIZE;
    }
  }
#endif

  void AnycubicTouchscreenClass::CommandScan() {
    static millis_t nextStopCheck = 0; // used to slow the stopped print check down to reasonable times
    const millis_t  ms            = millis();
    if (ELAPSED(ms, nextStopCheck)) {
      nextStopCheck = ms + 1000UL;
      if (mediaPrintingState == AMPRINTSTATE_STOP_REQUESTED) {
  #if ENABLED(ANYCUBIC_TFT_DEBUG)
        SERIAL_ECHOLNPGM("TFT Serial Debug: Finished stopping print, releasing motors ...");
  #endif
        mediaPrintingState = AMPRINTSTATE_NOT_PRINTING;
        mediaPauseState    = AMPAUSESTATE_NOT_PAUSED;
        injectCommands(F("M84\nM27")); // disable stepper motors and force report of SD status
        delay_ms(200);
        // tell printer to release resources of print to indicate it is done
        SENDLINE_DBG_PGM("J14", "TFT Serial Debug: SD Print Stopped... J14");
      }
    }

  #if ENABLED(KNUTWURST_4MAXP2)
    if (PrintdoneAndPowerOFF && powerOFFflag && (thermalManager.degHotend(0) < 50)) {
      powerOFFflag = 0;
      PowerDown();
    }
  #endif

    if (TFTbuflen < (TFTBUFSIZE - 1)) {
      GetCommandFromTFT();
    }

    if (TFTbuflen) {
      TFTbuflen  = (TFTbuflen - 1);
      TFTbufindr = (TFTbufindr + 1) % TFTBUFSIZE;
    }

  #if ENABLED(KNUTWURST_MEGA_P_LASER)
    laser_indicate();
    if (en_continue == 1) {
      prepare_laser_print();
    }
  #endif

    // In case of too short time after last cyclic tft command it has to be
    // wait to avoid missing action after acyclic command by the tft.
    if ( (delayed_tft_command > 0) && ( millis() - time_last_cyclic_tft_command >= WAIT_MS_UNTIL_ACYCLIC_SEND ) ) {
      switch (delayed_tft_command) {
        case 23: {
          SENDLINE_DBG_PGM("J23", "TFT Serial Debug: delayed J23");
          delayed_tft_command = 0;
          break;
        }
        case 18: {
          SENDLINE_DBG_PGM("J18", "TFT Serial Debug: delayed J18");
          delayed_tft_command = 0;
          break;
        }
        case 118: {
          SENDLINE_DBG_PGM("J23", "TFT Serial Debug: delayed J23");
          SENDLINE_DBG_PGM("J18", "TFT Serial Debug: delayed J18");
          delayed_tft_command = 0;
          break;
        }
      }
    }
  }

  void AnycubicTouchscreenClass::OnPrintTimerStarted() {
  #if ENABLED(SDSUPPORT)
    if (mediaPrintingState == AMPRINTSTATE_PRINTING) {
      SENDLINE_DBG_PGM("J04",
                       "TFT Serial Debug: Starting SD Print... soft endstops disabled J04"); // J04 Starting Print
      setSoftEndstopState(false);
      live_Zoffset = 0.0;
      powerOFFflag = false;
    }
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
      setSoftEndstopState(true);
      powerOFFflag = true;
      SENDLINE_DBG_PGM("J14", "TFT Serial Debug: SD Print Completed... soft endstops enabled J14");
    }
      // otherwise it was stopped by the printer so don't send print completed
      // signal to TFT
  #endif
  }

  #if BOTH(ANYCUBIC_TFT_DEBUG, KNUTWURST_DGUS2_TFT)
  void AnycubicTouchscreenClass::Command(const char* const command) {
    SENDLINE(command);
    #if ENABLED(ANYCUBIC_TFT_DEBUG)
    SERIAL_ECHOPGM("TFT Serial Debug: Sending ");
    SERIAL_ECHO(strlen(command));
    SERIAL_ECHOPGM(" ");
    SERIAL_ECHOLN(command);
    #endif
  }
  #endif

  #if ENABLED(KNUTWURST_CHIRON)
  void AnycubicTouchscreenClass::LevelingDone() {
    SENDLINE_DBG_PGM("J25", "TFT Serial Debug: Auto leveling done... J25");
  }
  #endif

  AnycubicTouchscreenClass AnycubicTouchscreen;
#endif // ifdef ANYCUBIC_TOUCHSCREEN
