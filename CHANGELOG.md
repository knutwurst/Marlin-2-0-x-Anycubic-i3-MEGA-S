# Changelog

All notable changes to the Knutwurst Marlin firmware for the Anycubic i3 Mega, Mega S, Mega Pro, Mega X, Chiron and 4MAX Pro.

The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/). Each entry notes the upstream Marlin release it was built on where it changed. Pre-releases (betas and RCs) are folded into their final version.

## [1.6.0] - 2026-06-08

Built on Marlin 2.1.2.7.

### Added
- Integrated BMP laser engraving from SD card on the Mega Pro, driven by the stock touchscreen. The earlier experimental port is now wired into the factory engraving flow.
- Low-power positioning pointer that traces the image outline before a job starts.
- Plausibility guard that refuses to engrave on an invalid or unreadable BMP header.
- `CHANGELOG.md` covering every released version.

### Changed
- Read the BMP header on file selection (A13) and report the image size, so the display shows the dimensions before engraving begins.
- Start engraving from A14 when a bitmap is selected, matching the proven factory trigger.
- Parse bitmaps with the standard 4-byte row stride for 16-, 24- and 32-bit images, and handle top-down (negative height) files.

### Fixed
- Restore the stock Mega Pro laser protocol: image parameters via the `S` code and the factory command numbers (A34, A36–A43, A49, A50). A previous rewrite had switched these to `V` and renumbered them, so the touchscreen and firmware no longer agreed.
- Pause and resume engraving through the existing A9/A10 handlers, and turn the laser off the moment a job is paused or stopped.
- Resolve the `case 50` command collision that had stopped the laser build from compiling.

## [1.5.5] - 2026-01-24

### Changed
- Update the codebase to Marlin 2.1.2.7 (thanks @stklcode).

## [1.5.4] - 2024-06-05

### Changed
- Merge upstream Marlin 2.1.2.2 and 2.1.2.3 (thanks @stklcode).

### Fixed
- Restore homing with dual Z endstops in the `STEPTEST` macro.
- Fix the DGUS build by setting `WAIT_MS_UNTIL_ACYCLIC_SEND` globally.
- Guard against sending acyclic display commands too quickly (thanks @uwetaz).
- Fix the ignored Z endstop when only a single Z endstop is used (thanks @hostops).
- Fix the continue button after pause or filament runout, and show the "out of filament" message reliably (thanks @uwetaz).
- Revert unintended config changes pulled in with the 2.1.2.2 merge.

## [1.5.3] - 2024-03-22

### Changed
- Relax thermal-runaway parameters, especially for the heated bed, to allow higher temperatures.
- Raise the filament-runout threshold and clean up the runout pin definitions.

### Fixed
- Fix manual bed leveling from the Mega Pro touchscreen.
- Fix the build with newer Python versions and the GitHub artifact upload.
- Fix filament-runout false positives.

## [1.5.2] - 2023-07-06

### Added
- Implement `M73` to set print-job progress, overriding the built-in time estimate for a more accurate display.
- Assisted leveling for the 4MAX Pro 2.0, including servo endstop-angle adjustment.

### Changed
- Replace the `KNUTWURST_TFT_LEVELING` flag with `KNUTWURST_CHIRON`.
- Clean up the touchscreen code with clang-format and drop unused declarations.

### Fixed
- Fix the heater error check on boot, which never triggered (#475).
- Fix the `RenderCurrentFolder` output string.

## [1.5.1] - 2023-06-13

Hotfixed on 2023-06-13. Built on Marlin 2.1.2.1.

### Added
- Support for SD card extenders.

### Changed
- Load and sort the file list far faster while using less RAM.
- Restructure the A6 print-status and A26 SD-refresh routines for speed.
- Disable steppers after one minute of idle time.
- Lower the maximum hotend temperature to stay within the thermistor range.

### Fixed
- Fix SD card reads and startup crashes on Anycubic 1.0 displays.
- Fix scrambled output on DGUS clone screens after long file lists.
- Fix elapsed time showing `--.--` after long prints.
- Fix the boot loop on Anycubic 1.0 displays when no filament sensor is connected.

## [1.5.0] - 2023-06-07

Built on Marlin 2.1.2. This release rebuilt the touchscreen layer from the ground up.

### Added
- Rewrite the Anycubic touchscreen as a fully functional ExtUI module.
- Host action commands for most touchscreen functions, controllable over USB.
- Live Z-offset on the Chiron, with automatic mesh adjustment when the offset changes.
- Manual probing on the Chiron and a dedicated probing print state.
- Support for `.gco` files and alphabetical file/folder sorting on all displays.
- High-speed BLTouch probing, selectable from the special menu.
- M600 filament change with or without a USB connection.

### Changed
- Switch the touchscreen to `LCD_SERIAL` and use `longest_filename()` for SD entries.
- Use soft PWM for the fans for lower noise.
- Disable software endstops while printing and leveling so mesh points below zero are reachable.
- Park the toolhead when aborting an SD print instead of homing.

### Fixed
- Fix the Z home position not touching the build plate (#396).
- Fix the default baud rate and port number for the ESP WiFi module (#406).
- Fix print/pause/resume and M600 filament change end to end.
- Fix the filament sensor clashing with host action commands.
- Fix rectangular bed shapes, NeoPixel builds, and several Chiron and 4MAX Pro build errors.

## [1.4.4] - 2022-05-12

Built on Marlin 2.0.9.2.

### Added
- Enable `M808` repeat markers (#325).
- Combine `M851` with Z babystepping so you can shift the live Z-offset while printing.

### Changed
- Show the real Marlin version instead of "2.0.x".
- Re-enable the software endstop on `Z_MIN`, previously disabled for BLTouch.

### Fixed
- Reset the 4MAX Pro correctly after boot.
- Fix the build error after the TMC26XX library was removed.
- Restore the USB pause response so M600 works with Octoprint.

## [1.4.3] - 2022-02-20

### Fixed
- Fix 4-point-easy-leveling applying the mesh incorrectly and falsifying the result (#282).

## [1.4.2] - 2022-02-13

### Added
- Integrated 4-point assisted leveling for the Mega Pro.

### Changed
- Stop motors and heaters immediately after a print is aborted.

### Fixed
- Fix aborting a print before it actually starts.

## [1.4.1] - 2022-02-09

### Fixed
- Switch the bed back to hardware PWM to stop the clicking/ticking noise from the power supply during heating.

### Changed
- Move the images out of the main repository into the wiki.

## [1.4.0] - 2022-02-07

Built on Marlin 2.0.9.2. Some units showed PSU ticking and worse bed heating; use 1.4.1 or newer.

### Added
- Mega Pro laser support via `M3` (Spindle CW / Laser On), compatible with tools such as LightBurn.
- Support for the Anycubic 4MAX Pro v1 and v2, including the original DWIN II display, plus 4MAX Pro auto power-off.
- Host action commands and host prompt support over USB.
- `M9999` Anycubic TFT debug command (thanks @etet100).

### Changed
- Switch the thermistor type from 5 to 1 or 11 depending on the printer, with new PID values to match.
- Use soft PWM with dithering for fans and heaters to lower noise.
- Re-enable Linear Advance on all models, disabled since the 2.0.9.2 update.
- Tune acceleration, jerk and feedrate across all printers and raise extruder speeds.
- Rename the Mega Pro environment to `MEGA_P_DGUS` for naming consistency.

### Fixed
- Fix the lockup when pressing STOP during an SD print on DGUS variants.
- Fix advanced pause/resume, nozzle re-heat after timeout, and resume with `M108`.
- Add the missing touchscreen handling for `M104` and `M109`.
- Fix the humming noise on the Mega X Z-axis and the stock Mega Pro X/Y drivers (A4988 timings).
- Add `Z_SAFE_HOMING` for the 4MAX with BLTouch and increase the probing margin.

## [1.3.1] - 2021-09-23

### Fixed
- Disable WiFi by default to fix USB communication drops with Octoprint and a USB webcam (#204). Re-enable it with `#define SERIAL_PORT_2`.

## [1.3.0] - 2021-09-18

### Added
- Adjust the BLTouch Z-probe offset from the special menu, with the offset saved to EEPROM.
- Backport MeatPack to compress gcode on the fly.
- Full ARC support by backporting the upstream G2/G3 changes, useful for laser engraving.
- ESP3D WiFi support over a second serial interface on the EXP1 header (#194).

### Fixed
- Fix the SD file list when exactly three files or folders are present (#197).
- Keep the cursor in place when selecting a special-menu entry.
- Make manual mesh bed leveling faster and more reliable.
- Disable steppers after three minutes of idle time.

## [1.2.0] - 2021-06-27

### Added
- Full Anycubic Chiron support: working manual (MBL) and auto (ABL) bed leveling with no custom gcode files, a "reset leveling" menu entry, manual leveling while printing, and automatic mesh rebuild on "load defaults".
- Support for the Chiron and Mega X with the new Anycubic 0.0.2 display (DGUS clone).
- Ultra-fast BLTouch/3DTouch probing with automatic mesh save and Z-offset save/restore.
- Groundwork for the Mega Pro laser and BMP feature (structs and initialization).

### Changed
- Raise the maximum hotend temperature to 300°C.
- Adjust the flow rate in 1% steps instead of 5%.
- New pin assignment for all Trigorilla boards and an A4988/TMC22XX split for best performance.
- Home X after G29 on the Chiron, release steppers after auto leveling, and move the nozzle 2 mm off the endstops after homing.

### Fixed
- Fix the lockup when pressing "back" in the advanced leveling menu.
- Fix the whining noise on the stock Mega Pro and the non-working extruder fan on the Mega P and Chiron.
- Fix print progress and print time on the display and crashes when using USB and SD at once.
- Fix easy 4-point leveling beyond point 1 on the Chiron and ARC stuttering on small curves.

## [1.1.9] - 2021-01-13

### Changed
- Set `MINTEMP` to 1°C and the minimum stepper pulse to 1, and add `CodeValueInt()` to prepare for laser support.

### Fixed
- Fix an axis or the extruder suddenly stopping or running backwards in some environments.
- Fix the printer stalling in a cold environment (below 10°C).

## [1.1.8] - 2021-01-01

### Added
- Babystepping without a BLTouch installed.

### Changed
- Make ARC settings default for the best laser-engraving compatibility.
- Increase the stepper direction-change delay to remove extruder glitches with TMC drivers.

### Fixed
- Show the special menu when no SD card is inserted.

## [1.1.7] - 2020-11-29

### Added
- Easy 4-point leveling assistant.
- G2/G3 arc support for the laser implementation.
- Jerk control instead of a fixed junction-deviation factor to save CPU cycles.
- BMG extruder support on the Mega X and support for cheaper TMC2208 drivers.
- Buzzer tone when PID tuning finishes.

### Changed
- Raise the maximum nozzle temperature to 300°C and bed temperature to 150°C.

### Fixed
- Fix stuttering on large curved objects and over USB/Octoprint.
- Fix random resets on the Mega S (DGUS) and Mega P and unexpected axis movement.
- Fix the inverted Mega Pro extruder direction and slow movement when engraving small circles.

## [1.1.6] - 2020-09-11

### Added
- First Mega Pro support (prerelease).
- Support for the first-generation i3 Mega with a single Z endstop.
- BondTech BMG extruder support (`KNUTWURST_BMG`).
- Flow-rate control in percent before or during a print.
- GitHub CI that builds every environment, plus a new `.hex` naming scheme.

### Changed
- Move "start mesh leveling" into a submenu.

### Fixed
- Make BLTouch leveling more reliable (10 mm max deviation).
- Fix the Windows linker error by shortening environment names.

## [1.1.5] - 2020-08-27

### Added
- Submenu for manual mesh bed leveling.
- PID tuning auto-save, so `M500` is no longer needed.

### Changed
- Better menu structure on the DGUS clone display.
- Raise the filament-runout watch from 3 to 5 seconds and the max Z feedrate to 18 mm/s.

### Fixed
- Fix the "no SD card" message when entering the special menu without a card.
- Fix the BLTouch nozzle not retracting between probes.

## [1.1.4] - 2020-08-24

### Added
- Junction deviation for the Mega X.
- Detailed version info for DGUS and DGUS II displays.
- Longer filenames on the Mega S/P, and a special menu that works without an SD card.

### Fixed
- Lower the Z feedrate from 20 to 8 mm/s on the Mega X with TMC drivers.
- Center the nozzle on the bed during automatic PID tuning.

## [1.1.3] - 2020-08-16

### Added
- Fully automated PID tuning, including movement, fans, temperature and EEPROM save.
- Deployment script that creates a subfolder per version.

### Changed
- Increase overall movement speed.

### Fixed
- Fix the Z height after hotend PID autotune and 0.01 mm Z stepping.
- Fix Mega X acceleration, junction deviation and homing speed.

## [1.1.2] - 2020-08-13

### Added
- Much faster BLTouch probing.

### Changed
- `M115` now reports the real firmware version and build date.

### Fixed
- Fix the hotend fan not starting during PID tuning.
- Fix the DGUS II file list not filling at times.

## [1.1.1] - 2020-08-10

### Added
- Full Anycubic Mega X support.
- Official BLTouch support with auto-save and homing.
- Multi-environment build handling and deployment.
- User-defined touchscreen preheat settings.

### Fixed
- Fix mixed-up endstops in the Mega S configuration.
- Fix 0.01 mm mesh movement going the wrong way.

## [1.1.0] - 2020-07-26

Built on Marlin 2.0.5.4.

### Added
- BLTouch support (`#define KNUTWURST_BLTOUCH`).
- Support for the new Anycubic touchscreen (DGUS II clone).

### Changed
- Rework the serial protocol handling and append `.gcode` only when needed.
- Build the special menu from a fake filesystem and store TFT strings in PROGMEM.
- Max out the host receive buffer for better USB prints.

### Fixed
- Fix a hardware-serial bug that crashed the TFT firmware.
- Fix long filenames and the special menu with long filenames.

## [1.0.6] - 2020-07-11

### Added
- Push the bed size to 220 × 225 × 215 mm.
- Endstop noise termination and silent endstops (reactivatable in code).

### Changed
- Faster touchscreen response and a new default of 392 E-steps for the S extruder (Titan).

### Fixed
- Fix unwanted beeping while printing and the endless print loop when pressing Stop while heating.
- Fix the nozzle not reheating from a paused state.

## [1.0.5] - 2020-06-21

### Added
- Advanced pause and nozzle parking, so filament change works without a host PC (#1).
- Startup chime to signal a working serial connection, plus endstop and thermal-runaway tones.
- Semi-preconfigured BLTouch support (off in binary releases).

### Fixed
- Fix babystepping and resuming from a parked nozzle.
- Fix graphical glitches from long filenames.

## [1.0.4] - 2020-06-08

### Added
- 0.02 mm and 0.01 mm Z-stepping for mesh calibration.

### Fixed
- Minor fixes in the menu handling.

## [1.0.3] - 2020-06-06

### Added
- Disable the filament-runout sensor from the special menu.

### Changed
- Optimize Mega S extruder feedrate and acceleration.

### Fixed
- Fix relative positioning from the touchscreen.
- Initialize the EEPROM automatically at startup after errors.

## [1.0.2] - 2020-06-06

### Fixed
- Major touchscreen reliability fixes.
- Working pause and filament-runout functions and steadier SD printing.

## [1.0.1] - 2020-06-05

### Fixed
- Bugfix release.

## [1.0.0] - 2020-06-05

- Initial public release.

[1.6.0]: https://github.com/knutwurst/Marlin-2-0-x-Anycubic-i3-MEGA-S/compare/1.5.5...1.6.0
[1.5.5]: https://github.com/knutwurst/Marlin-2-0-x-Anycubic-i3-MEGA-S/compare/1.5.4...1.5.5
[1.5.4]: https://github.com/knutwurst/Marlin-2-0-x-Anycubic-i3-MEGA-S/compare/1.5.3...1.5.4
[1.5.3]: https://github.com/knutwurst/Marlin-2-0-x-Anycubic-i3-MEGA-S/compare/1.5.2...1.5.3
[1.5.2]: https://github.com/knutwurst/Marlin-2-0-x-Anycubic-i3-MEGA-S/compare/1.5.1...1.5.2
[1.5.1]: https://github.com/knutwurst/Marlin-2-0-x-Anycubic-i3-MEGA-S/compare/1.5.0...1.5.1
[1.5.0]: https://github.com/knutwurst/Marlin-2-0-x-Anycubic-i3-MEGA-S/compare/1.4.4...1.5.0
[1.4.4]: https://github.com/knutwurst/Marlin-2-0-x-Anycubic-i3-MEGA-S/compare/1.4.3...1.4.4
[1.4.3]: https://github.com/knutwurst/Marlin-2-0-x-Anycubic-i3-MEGA-S/compare/1.4.2...1.4.3
[1.4.2]: https://github.com/knutwurst/Marlin-2-0-x-Anycubic-i3-MEGA-S/compare/1.4.1...1.4.2
[1.4.1]: https://github.com/knutwurst/Marlin-2-0-x-Anycubic-i3-MEGA-S/compare/1.4.0...1.4.1
[1.4.0]: https://github.com/knutwurst/Marlin-2-0-x-Anycubic-i3-MEGA-S/compare/1.3.1...1.4.0
[1.3.1]: https://github.com/knutwurst/Marlin-2-0-x-Anycubic-i3-MEGA-S/compare/1.3.0...1.3.1
[1.3.0]: https://github.com/knutwurst/Marlin-2-0-x-Anycubic-i3-MEGA-S/compare/1.2.0...1.3.0
[1.2.0]: https://github.com/knutwurst/Marlin-2-0-x-Anycubic-i3-MEGA-S/compare/1.1.9...1.2.0
[1.1.9]: https://github.com/knutwurst/Marlin-2-0-x-Anycubic-i3-MEGA-S/compare/1.1.8...1.1.9
[1.1.8]: https://github.com/knutwurst/Marlin-2-0-x-Anycubic-i3-MEGA-S/compare/1.1.7...1.1.8
[1.1.7]: https://github.com/knutwurst/Marlin-2-0-x-Anycubic-i3-MEGA-S/compare/1.1.6...1.1.7
[1.1.6]: https://github.com/knutwurst/Marlin-2-0-x-Anycubic-i3-MEGA-S/compare/1.1.5...1.1.6
[1.1.5]: https://github.com/knutwurst/Marlin-2-0-x-Anycubic-i3-MEGA-S/compare/1.1.4...1.1.5
[1.1.4]: https://github.com/knutwurst/Marlin-2-0-x-Anycubic-i3-MEGA-S/compare/1.1.3...1.1.4
[1.1.3]: https://github.com/knutwurst/Marlin-2-0-x-Anycubic-i3-MEGA-S/compare/1.1.2...1.1.3
[1.1.2]: https://github.com/knutwurst/Marlin-2-0-x-Anycubic-i3-MEGA-S/compare/1.1.1...1.1.2
[1.1.1]: https://github.com/knutwurst/Marlin-2-0-x-Anycubic-i3-MEGA-S/compare/1.1.0...1.1.1
[1.1.0]: https://github.com/knutwurst/Marlin-2-0-x-Anycubic-i3-MEGA-S/compare/1.0.6...1.1.0
[1.0.6]: https://github.com/knutwurst/Marlin-2-0-x-Anycubic-i3-MEGA-S/compare/1.0.5...1.0.6
[1.0.5]: https://github.com/knutwurst/Marlin-2-0-x-Anycubic-i3-MEGA-S/compare/1.0.4...1.0.5
[1.0.4]: https://github.com/knutwurst/Marlin-2-0-x-Anycubic-i3-MEGA-S/compare/1.0.3-Bugfix...1.0.4
[1.0.3]: https://github.com/knutwurst/Marlin-2-0-x-Anycubic-i3-MEGA-S/compare/1.0.2...1.0.3-Bugfix
[1.0.2]: https://github.com/knutwurst/Marlin-2-0-x-Anycubic-i3-MEGA-S/compare/1.0.1...1.0.2
[1.0.1]: https://github.com/knutwurst/Marlin-2-0-x-Anycubic-i3-MEGA-S/compare/1.0.0...1.0.1
[1.0.0]: https://github.com/knutwurst/Marlin-2-0-x-Anycubic-i3-MEGA-S/releases/tag/1.0.0
