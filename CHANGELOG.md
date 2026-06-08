# Changelog

All notable changes to this firmware are documented here.
Dates are the release dates of the binaries. Pre-releases (betas/RCs) are omitted.

## [1.6.0] - 2026-06-08

### New Features
- Integrated BMP laser engraving from SD card on the Anycubic Mega Pro, driven by the stock touchscreen. The previous experimental port is now wired up to the factory engraving flow.
- Laser positioning pointer (low power) to trace the image outline before engraving.

### Bugfixes
- Restore the stock Mega Pro laser command protocol: parameters via the `S` code and the factory command numbers (A34/A36–A43/A49/A50). A previous rewrite had switched these to `V` and renumbered them, so the touchscreen and firmware no longer matched.
- Read the BMP header on file selection (A13) and report the image size, so the display shows the dimensions before engraving starts.
- Robust BMP parsing: 4-byte row stride for 16-, 24- and 32-bit images, and correct handling of top-down (negative height) bitmaps.
- Pause and resume engraving through the standard A9/A10 handlers.
- Refuse to engrave on an invalid or unreadable header.
- Resolve the `case 50` command collision that had kept the laser build from compiling.

### Known Issues
- The SD-card bitmap laser engraving is new and not yet tested on real hardware. Lasering via USB with LightBurn (`M3`/`M4`/`M5`) is the proven path. The feature expects the stock Mega Pro touchscreen protocol.
- During engraving the progress bar may not advance, and the screen gets no "finished" signal when the job ends.
- Keep the hotend thermistor connected while lasering, or MINTEMP stops the job. Do not set a hotend temperature in laser mode.
- The built-in leveling of the 4MAX Pro does not work yet. Use the 4-point-easy leveling and mesh leveling via the special menu instead.
- The nozzle temperature can only be set above 260°C via gcode, not the touchscreen.
- The gcode file transfer via WiFi is slow due to hardware limitations.

## [1.5.5] - 2026-01-24

### New Features
- Update codebase to Marlin 2.1.2.7 (thanks @stklcode).

## [1.5.4] - 2024-06-05

### New Features
- Update codebase to Marlin 2.1.2.2 (thanks @stklcode).

### Bugfixes
- Fix ignored Z endstop when only one endstop is used (thanks @hostops).
- Fix continue button behavior after pause or filament runout (thanks @uwetaz).
- Fix "lack of filament" message not displayed when filament runs out (thanks @uwetaz).
- Fix homing issue when using two endstops (thanks @stklcode).

## [1.5.3] - 2024-03-22

### Bugfixes
- Fix build issues when compiling with newer Python versions.
- Fix thermal runaway that was too aggressive in some cases.
- Fix filament runout false positives.
- Fix manual bed leveling on Mega Pro.

## [1.5.2] - 2023-07-06

### New Features
- Add Marlin M73 support for a more accurate print time display.
- Improve overall stability (thanks @stklcode).

### Bugfixes
- Heater error check on boot did not trigger at all.

## [1.5.1] - 2023-06-13

Hotfixed on 2023-06-13.

### New Features
- Update codebase to Marlin 2.1.2.1 (thanks @stklcode).
- Faster file list load and refresh.
- Support for SD card extenders.

### Bugfixes
- Fix Anycubic 1.0 displays that had issues reading the SD card or crashed at startup.
- Fix scrambled output on DGUS clone screens when scrolling through long file lists.
- Fix elapsed time showing `--.--` after long prints.
- Fix boot loop on Anycubic 1.0 displays when no filament sensor is connected.

## [1.5.0] - 2023-06-07

### New Features
- Update codebase to Marlin 2.1.2 (thanks @stklcode).
- Low-noise SoftPWM for fans and heaters.
- Touchscreen code as a fully functional ExtUI module.
- Print/Pause/Resume/Stop work as intended.
- M600 filament change with or without USB connection.
- Host action commands for most touchscreen functions.
- Filament sensor works internally and externally via USB, and can be enabled/disabled permanently via the special menu.
- On manual stop, the bed moves forward and the printhead parks.
- Live Z-offset on Anycubic Chiron, with automatic mesh adjustment when altering the Z-offset.
- Support for `.gco` files and alphabetic file/folder sorting on all displays.
- With ABL or MBL, the nozzle can go below endstop zero.
- High-speed mode for BLTouch probing (special menu).

### Bugfixes
- Fix compile error when using NeoPixels.
- Fix rectangular bed shapes.
- Fix Z home position not touching the build plate.
- Fix default baud rate and port number for the ESP WiFi module.
- Fix some files not displayed on older touchscreens and corrupted file parsing.
- Fix BLTouch probing/errors with clone probes.
- Fix filament sensor on Chiron and 4MAX Pro 2.0.
- Fix navigation so you can go back inside a directory.
- Fix directory access on Anycubic 1.0 displays.
- Fix 4MAX Pro 2.0 M600 taking too long on filament change.
- Fix missing Z2 pin definitions on Chiron and 4MAX.
- Fix Chiron ABL, special menu without an SD card, load-defaults routine, live offset rendering and 4MAX Pro auto power-off.

## [1.4.4] - 2022-05-12

### New Features
- Combine M851 and babystepping so babystepping can alter the current Z-offset while printing.
- Enable M808 gcode for repeat markers (thanks @stklcode).
- Show the correct Marlin version instead of just "2.0.x".

### Bugfixes
- 4MAX Pro now resets correctly after boot.
- Fix build error after the TMC26XX library was removed.
- Fix USB pause that was not automatically resumed.
- Fix missing software endstop on Z_MIN.

## [1.4.3] - 2022-02-20

### Bugfixes
- Fix 4-point-easy-leveling applying the mesh wrongly and falsifying the result (#282).

## [1.4.2] - 2022-02-13

### New Features
- Integrated assisted leveling for the Mega Pro.
- Stop motors and heaters right after aborting a print.

### Bugfixes
- Fix a case where a print could not be aborted while heating.

## [1.4.1] - 2022-02-09

### New Features
- Same features as 1.4.0, but with hardware PWM and a higher PWM frequency.

### Bugfixes
- Switch back to the previous PWM controller (as in 1.3.1 and earlier) to stop clicking/ticking noise from the PSU when the bed reaches temperature.

## [1.4.0] - 2022-02-07

Some users reported clicking/ticking noise from the power supply and worse bed heating. Use 1.4.1 or newer.

### New Features
- Update codebase to Marlin 2.0.9.2.
- Mega Pro laser support with M3 (Spindle CW / Laser On); works with tools like LightBurn.
- Support for Anycubic 4MAX Pro v1 and v2, including the 4MAX Pro 2.0 with the original DWIN II display, plus 4MAX Pro auto power-off.
- More reliable acceleration/jerk values and higher possible extruder speeds on all printers.
- Enable host action commands to control the firmware via USB.
- Add M9999 Anycubic TFT debug command (thanks @etet100).
- Switch thermistor type from 5 to 1 or 11 depending on the printer.
- Allow longer filenames on the DGUS clone / Anycubic 0.0.2 display.
- Lower fan noise at low fan speeds.

### Bugfixes
- 4MAX Pro 4-point-easy-leveling and PID-tune hotend now work.
- Fix Chiron TFT crash after auto leveling and Z-offset not stored in EEPROM.
- Add missing touchscreen handling for M104 and M109.
- Increase probe margin to avoid probing outside bed boundaries; default level fade height 0.
- Fix resume from pause with M108, advanced pause/resume, and nozzle re-heat after timeout.
- Fix humming noise on Mega X Z-axis and stock Mega Pro X/Y drivers (A4988 timings).
- Fix missing home/G28 when starting manual mesh bed leveling.
- Fix lockup when pressing STOP on the Anycubic 0.0.2 display.

## [1.3.1] - 2021-09-23

### New Features
- Same features as 1.3.0 but with WiFi disabled by default. Re-enable via `#define SERIAL_PORT_2` in Configuration.h.

### Bugfixes
- Fix USB communication issues with Octoprint and a USB webcam by disabling the WiFi functionality.

## [1.3.0] - 2021-09-18

### New Features
- Adjust the BLTouch Z-probe offset via the special menu.
- MeatPack support to compress gcode on the fly.
- Full ARC Welder support via the Octoprint or Cura plugin.
- ESP3D WiFi support with an additional UART interface.

### Bugfixes
- Fix SD card file list when only three gcode files are present.
- Keep the cursor in place when selecting a special menu entry.
- Faster and more reliable manual mesh bed leveling.
- Fix G2/G3 segment calculation (thanks @ashleysommer, #191).
- Fix the T0 sensor display message and Chiron Z-offset editing.

## [1.2.0] - 2021-06-27

### New Features
- Support for Chiron and Mega X with the new Anycubic 0.0.2 display.
- Support for both Chiron models (Anycubic OFW 1.3.0 and 1.3.5).
- Working manual (MBL) and auto (ABL) bed leveling for the Chiron, with no custom gcode files needed and a "reset leveling" menu entry.
- Manual leveling while printing on the Chiron and automatic mesh rebuild on "load defaults".
- Ultra-fast BLTouch/3DTouch probing, automatic mesh save to EEPROM, and automatic Z-offset save/restore.
- Flow rate adjustable in 1% steps.
- New pin assignment for all Trigorilla boards.
- Auto-home X after G29 on Chiron; nozzle moves 2 mm off the endstops after homing on Mega S/P/X.
- Automatic stepper driver release after auto leveling and A4988/TMC22XX differentiation.
- Max E0 nozzle temperature raised to 300°C.

### Bugfixes
- Fix crash when pressing "back" in the advanced leveling menu.
- Fix whining noise on the stock Mega Pro and filament in/out nozzle heating.
- Fix support for 3DTouch and other BLTouch clones and BLTouch mesh not loading correctly.
- Fix Chiron extruder fan, mesh out of bed boundaries and easy 4-point leveling beyond point 1.
- Fix print progress and print time on the display.
- Fix crashes when using USB and SD at the same time (DGUS version).
- Fix slow touchscreen on some Mega S/P models and stuttering on small ARC curves.

## [1.1.9] - 2021-01-13

### Bugfixes
- Fix extruder or an axis suddenly stopping or running backwards in some environments.
- Fix the printer stopping in a cold environment (< 10°C).
- Prepare the codebase for Mega Pro laser and Chiron support.

## [1.1.8] - 2021-01-01

### New Features
- Babystepping is now usable without BLTouch.

### Bugfixes
- Show the special menu without an SD card.
- Make ARC settings default for best laser-engraving compatibility.
- Increase stepper direction change delay to eliminate extruder bugs with TMC drivers.

## [1.1.7] - 2020-11-29

### New Features
- Easy 4-point leveling assistant.
- Jerk control instead of a fixed junction deviation factor to save CPU cycles.
- G2/G3 arc/circle move support for the laser implementation.
- Buzzer sound when PID tuning finishes.
- Support for cheap TMC2208 steppers and the BMG extruder on the Mega X.
- Raise maximum nozzle temperature to 300°C and bed temperature to 150°C.

### Bugfixes
- Fix stuttering on large curved objects and via USB/Octoprint.
- Fix random resets on Mega S (DGUS) and Mega P and unexpected axis movement.
- Fix default jerk settings and less aggressive thermal runaway protection.
- Fix SlowDown and add a wait cycle on direction changes to reduce layer shifts.
- Fix inverted Mega Pro extruder direction and slow axis movement when engraving small circles.

## [1.1.6] - 2020-09-11

### New Features
- Mega Pro support (prerelease, use with caution).
- First-generation i3 Mega support (single Z endstop).
- Cleaner mesh-leveling submenu with "start" and "save".
- Flow rate control in % before or during a print.
- Support for the BondTech BMG extruder (`KNUTWURST_BMG`).
- New naming scheme for `.hex` files.

### Bugfixes
- Fix build error on Windows regarding max path length.
- Make BLTouch more reliable with a 10 mm max deviation.

## [1.1.5] - 2020-08-27

### New Features
- Submenu for manual mesh bed leveling.
- Increased Z speed for the Mega X.
- Increased filament runout watch from 3 to 5 seconds.
- Better menu structure on the DGUS clone TFT.
- PID tuning auto-save (no M500 needed).

### Bugfixes
- Fix the "no SD card" message when entering the special menu without an SD card.
- Fix BLTouch nozzle not retracting between probes in some configurations.
- Fix too-long version number on the Mega X TFT.

## [1.1.4] - 2020-08-24

### New Features
- Junction deviation for the Mega X.
- Version info for DGUS and DGUS2 TFTs.
- Remove `.gcode` from the special menu on the Mega X.
- Support for longer filenames on Mega S/P.
- Special menu works without an SD card inserted.

### Bugfixes
- Lower Z feedrate from 20 to 8 mm/s on the Mega X with TMC drivers.
- Center the nozzle on the bed during automatic PID tuning.

## [1.1.3] - 2020-08-16

### New Features
- Fully automated PID tuning (movement, fans, temperature, EEPROM).
- Increased overall movement speed.
- Deployment script creates subfolders per version.

### Bugfixes
- Z up/down 0.01 mm stepping now works.
- Fix Mega X default acceleration/junction deviation and too-high homing speed.

## [1.1.2] - 2020-08-13

### New Features
- Faster BLTouch probing.
- M115 reports the real firmware version and distribution date.

### Bugfixes
- Fix hotend fan not starting during PID tuning.
- Fix DGUS II TFT not filling the file list at times.

## [1.1.1] - 2020-08-10

### New Features
- Mega X support.
- Official BLTouch support with an auto-save feature.
- Better feature toggles in Configuration.h and via PlatformIO, with multi-environment handling.
- New filename naming scheme.

### Bugfixes
- Fix 0.01 mm mesh movement in the wrong direction.
- Fix mixed-up endstops in the Mega S configuration.

## [1.1.0] - 2020-07-26

### New Features
- Update codebase to Marlin 2.0.5.4.
- BLTouch support (`#define KNUTWURST_BLTOUCH`).
- Support for the new Anycubic touchscreen (DGUS II clone).
- Maxed-out host receive buffer for better USB prints.

### Bugfixes
- Fix BLTouch special menu.

### Known Issues
- On the new touchscreen, every menu item shows the `.gcode` extension.
- Pressing "print" on a special menu item crashes the print menu.

## [1.0.6] - 2020-07-11

### New Features
- Bed size pushed to 220 x 225 x 215 mm.
- Endstop noise termination and no endstop beeps (reactivatable in code).
- Faster responding touchscreen.
- Change default E steps for the S extruder to 392 (Titan default).

### Bugfixes
- Fix unwanted beeping while printing.
- Fix endless print loop when pressing Stop while heating.
- Fix nozzle not reheating in pause state.
- Fix special menu shown when going back more than 15 times.

## [1.0.5] - 2020-06-21

### New Features
- Bed leveling restored after power off.
- Advanced pause and nozzle parking.
- Filament change without a host PC or Pronterface.
- Short startup chime to indicate a working serial connection.
- Endstop beeps are back.
- Semi-preconfigured BLTouch support (not active in binary releases).

### Bugfixes
- Fix babystepping.
- Fix Trigorilla selection for BLTouch in Configuration.h.
- Fix graphical glitches from too-long filenames.
- Fix M600 by implementing the advanced pause feature.

## [1.0.4] - 2020-06-08

### New Features
- Add 0.02 mm and 0.01 mm Z-stepping for mesh calibration.

### Bugfixes
- Minor fixes in the current menu handling.

## [1.0.3] - 2020-06-06

### New Features
- Filament runout sensor can be disabled via the special menu.

### Bugfixes
- Fix relative positioning when using the touchscreen.
- Optimize Mega S extruder feedrate and acceleration values.
- Initialize EEPROM automatically at startup in case of errors.

## [1.0.2] - 2020-06-06

### Bugfixes
- Major touchscreen reliability fixes.
- Working pause and filament runout functions.
- Stability improvements when printing from SD card.

## [1.0.1] - 2020-06-05

### Bugfixes
- Bugfix release.

## [1.0.0] - 2020-06-05

- Initial public release.
