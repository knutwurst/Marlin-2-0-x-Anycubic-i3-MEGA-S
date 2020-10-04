# Knutwurst's i3 MEGA (M/S/P/X) Firmware <br>(based on Marlin 2.0.x)

<span style="color: red;">(BITTE GENAU DURCHLESEN! / PLEASE READ CAREFULLY!)</span>

### Wenn dir gefällt, was ich mache, kannst du mir hier einen Kaffee spendieren*: [![Donate](https://img.shields.io/badge/Donate-PayPal-green.svg)](https://paypal.me/oliverkoester)
<sub>*Es muss jetzt keine großzügige Spende sein. Ein paar Cent reichen um mir zu zeigen, wer überhaupt Interesse daran hat und wem die Weiterentwicklung wichtig ist. So bleibt die Motivation da und ich weiß einfach, dass ich nicht für die Tonne programmiere ;)<sub>


### Wenn du Fragen hast, schaue gern in der offiziellen [Facebook-Gruppe](https://www.facebook.com/groups/3094090037303577/) vorbei.

### WARNUNG: Der Mega Pro / Mega P Support befindet sich noch in der Entwicklung. Aktuell wird der Laser noch nicht unterstützt, ebensowenig wie das Piezo-Leveling! 

# Inhaltsverzeichnis (Deutsch)
- [Funktionen](#funktionen)
   - [Was ist besser?](#besser-im-vergleich-zu-anderen-firmwares-bugfixes)
- [Häufig gestellte Fragen (FAQ)](https://github.com/knutwurst/Marlin-2-0-x-Anycubic-i3-MEGA-S/wiki/FAQ-(deutsch))
- [Fotos / Bilder](https://github.com/knutwurst/Marlin-2-0-x-Anycubic-i3-MEGA-S/wiki/Pictures)
- [Downloads](#downloads)


### WARNING: The Mega Pro / Mega P support is still under development. The laser is currently not supported, nor is piezo leveling!

# Table of Contets (english)
- [Features](#features)
   - [What's better?](#whats-better-in-coparison-to-other-firmwares-bug-fixes)
- [Frequently asked questions (FAQ)](https://github.com/knutwurst/Marlin-2-0-x-Anycubic-i3-MEGA-S/wiki/FAQ-(english))
- [Photos / Pictures](https://github.com/knutwurst/Marlin-2-0-x-Anycubic-i3-MEGA-S/wiki/Pictures)
- [Download](#download)


# Readme - German

## Funktionen

 * 4-Punkt Leveling-Hilfe "Easy Leveling"
 * Mesh-Bed Kalibrierung / Autokalibrierung mit [BLTouch (Installations-Anleitung)](https://github.com/knutwurst/Marlin-2-0-x-Anycubic-i3-MEGA-S/wiki/BLTouch-Installation-(deutsch))
 * S-Kurven Beschleunigung
 * "Linear Pressure Control v1.5" aktiviert (kann mit M900 konfiguriert werden)
 * Babystepping während des Druckvorgangs
 * Bilinear Bed Leveling (BBL)
 * Manuelles Editieren der Messpunkte
 * Volle Unterstützung aller [Anycubic Touchscreens](https://github.com/knutwurst/Marlin-2-0-x-Anycubic-i3-MEGA-S/wiki/Types-of-Anycubic-Touchscreens)
 * Bauteilkühler läuft nun auf 100% statt maximal 70%
 * Pause & Filamentwechselfunktion
 * Automatische EEPROM Initialisierung
 * Filament Runout Sensor kann im Menü temporär deaktiviert werden
 * Wiederaufnahme des Drucks nach Stromausfall (WiP)
 * [Druckbettgröße erweitert auf 225 x 220 x 210 mm](https://github.com/knutwurst/Marlin-2-0-x-Anycubic-i3-MEGA-S/wiki/Set-new-print-bed-size)
 * Automatischer 'Slowdown', falls Daten nicht schnell genug fließen
 * Vollautomatisches Hotend und Ultrabase PID Tuning
 * Optimierungen für [Trinamic TMC Schrittmotortreiber](https://github.com/knutwurst/Marlin-2-0-x-Anycubic-i3-MEGA-S/wiki/Schrittmotortreiber-gegen-TMC2xxx-tauschen)


## Besser im Vergleich zu anderen Firmwares (Bugfixes):
 * Aufgeräumtes Special-Menü
 * Drucker hängt sich nicht auf, wenn man Dateien mit Sonderzeichen (Umlaute, Chinesisch etc.) auf der SD Karte hat
 * Kein dummes "wackeln" der Düse nach Stoppen eines Druckvorgangs
 * Kein Abstürzen bei zu vielen Dateien auf der SD Karte
 * Kein Abstürzen bei SD-Karten über 16 GB
 * Fehler "Melodie" bei Thermal Runaway Protection
 * Kein Aufhängen, wenn SD Karte + USB gleichzeitig genutzt wird
 * Kein Aufhängen wenn man Pause drückt
 * Kein Aufhängen, wenn das Filament leer ist
 * Keine "spezielle" BLTouch Firmware nötig. Hier ist alles drin
 * Konfiguration ganz einfach über Feature-Toggles
 * Man muss keine Grundkonfiguration per GCODE machen.
 * Man muss keinen Werksreset durchführen

---

# Downloads

Du kannst die fertigen Binärdateien hier herunterladen: https://github.com/knutwurst/Marlin-2-0-x-Anycubic-i3-MEGA-S/releases. Natürlich kannst du die Firmware mit PlatformIO oder der Arduino IDE auch [selbst kompilieren](https://github.com/knutwurst/Marlin-2-0-x-Anycubic-i3-MEGA-S/wiki/Howto:-Compile-Firmware-with-PlatformIO).

Um es übersichtlich zu gestalten, beinhalten die Dateinamen die einzelnen Features.

`_1G` steht für die erste Generation des i3 Mega mit nur einem Z Endstop. Diese Firmware ist nicht für den normalen i3 Mega geeignet!

`_S` steht für den Mega S mit dem Titan Extruder.

`_P` steht für den Mega P mit dem BMG Extruder.

`_X` steht für den Mega X.

`_TMC` steht für Trinamic TMC Motortreiber. Hierbei wird auch die Drehrichtung der Motoren invertiert.

`_DGUS` steht für das "neue" blau/gelbe DGUS II Display, welches sonst kein Special-Menü anzeigen kann.

`_BLT` steht für die BL-Touch Version mit Autoleveling-Sensor. Das manuelle Mesh-Leveling ist hier deaktiviert.

`_10` steht für das Trigorilla_14 v1.0 Mainboard, welches normalerweise der Standard sein sollte.

`_11` steht für das Trigorilla_14 v1.1 Mainboard, bei welchem sich die Pinbelegung für den Server-Port geändert hat.


Für (fast) jede Kombination gibt es eine passende Firmware im Download-Bereich. ;)

---

### If you like what I do you can buy me a coffee: [![Donate](https://img.shields.io/badge/Donate-PayPal-green.svg)](https://paypal.me/oliverkoester)
<sub>*It doesn't have to be a generous donation. A few cents are enough to show me who is interested in further development. So the motivation stays and I just know that I am not programming for the bin ;)<sub>

# Readme - English

## Features:

 * 4-Point Leveling-Assistant "Easy Leveling"
 * Mesh bed calibration / Auto calibration with [BLTouch (Installation Manual)](https://github.com/knutwurst/Marlin-2-0-x-Anycubic-i3-MEGA-S/wiki/BLTouch-Installation-(english))
 * S-curve acceleration
 * "Linear Pressure Control v1.5" activated (can be configured with M900)
 * Baby stepping during the printing process
 * Bilinear Bed Leveling (BBL)
 * Manual editing of the measuring points
 * Full Support for all [Anycubic Touchscreens](https://github.com/knutwurst/Marlin-2-0-x-Anycubic-i3-MEGA-S/wiki/Types-of-Anycubic-Touchscreens)
 * Part cooling fan now runs at 100% instead of 70%
 * Pause & filament change function
 * Automatic EEPROM initialization
 * Filament runout sensor can be temporarily deactivated in the menu
 * Resumption of printing after a power failure (WiP)
 * [Print bed size enlarged to 225 x 220 x 210 mm](https://github.com/knutwurst/Marlin-2-0-x-Anycubic-i3-MEGA-S/wiki/Set-new-print-bed-size)
 * Automatic 'slowdown' if data does not flow fast enough
 * Fully automatic hotend and ultrabase PID tuning
 * Optimizations for [Trinamic TMC Stepper drivers](https://github.com/knutwurst/Marlin-2-0-x-Anycubic-i3-MEGA-S/wiki/Swap-stepper-motor-driver-for-TMC2xxx)

### What's better in coparison to other firmwares (bug fixes):
 * Tidy special menu
 * Printer does not freeze if you have files with special characters (umlauts, Chinese..) on your SD card
 * No stupid "wobble" of the nozzle after stopping printing
 * No firmware crash when there are too many files on the SD card
 * No crash with SD cards over 16 GB
 * Acoustic alarm in case of a thermal runaway
 * No freezing when SD card + USB is used at the same time
 * No freezing when you press pause and try to resume
 * No freezing when the filament is empty
 * No need for a "special" BLTouch firmware. In this firmware is everything included
 * Configuration made easy via feature toggles


---

# Download

You can download the precompiled binary files from here: https://github.com/knutwurst/Marlin-2-0-x-Anycubic-i3-MEGA-S/releases. Of course you can also [compile the firmware yourself](https://github.com/knutwurst/Marlin-2-0-x-Anycubic-i3-MEGA-S/wiki/Howto:-Compile-Firmware-with-PlatformIO) with PlatformIO or the Arduino IDE.

In order to make it clear, the file names contain the individual features.

`_1G` stands for the firt generation i3 Mega with only one Z endstop. Do NOT use this Firmware on the regular i3 Mega!

`_S` stands for the Mega S with the titan extruder.

`_P` stands for the Mega P with the BMG extruder.

`_X` stands for the Mega X.

`_TMC` stands for Trinamic TMC motor driver. The direction of rotation of the motors is also inverted.

`_DGUS` stands for the "new" blue / yellow DGUS II display, which otherwise cannot show a special menu.

`_BLT` stands for the BL-Touch version with auto-leveling sensor. Manual mesh leveling is deactivated here.

`_10` stands for the Trigorilla_14 v1.0 mainboard, which should normally be the standard.

`_11` stands for the Trigorilla_14 v1.1 mainboard, on which the pin assignment for the server port has changed.


For (almost) every combination there is a suitable firmware in the download area. ;)


### If you like what I do you can buy me a coffee: [![Donate](https://img.shields.io/badge/Donate-PayPal-green.svg)](https://paypal.me/oliverkoester)
<sub>*It doesn't have to be a generous donation. A few cents are enough to show me who is interested in further development. So the motivation stays and I just know that I am not programming for the bin ;)<sub>
