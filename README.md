# Knutwurst's i3 MEGA S Firmware (based on Marlin 2.0.5.3)
# This is still WIP and not intended for regular use!

# Readme - German (english below)

Diese Firmwarekonfiguration aktiviert viele neue erweitere Funktionen der Marlin Firmware:

 * Mesh-Bed Kalibrierung
 * S-Kurven Beschleunigung
 * Babystepping während des Druckvorgangs (nur über USB)
 * Biliniar Bed Leveling (BBL)
 * Manuelles Editieren der Messpunkte
 * Volle Anycubic Touchscreen Unterstützung
 * Pause & Filamentwechselfunktion

Bevor du irgendwas machst, nachdem du die Firmware geupdated hast, gehe zu `Special Menu > Load FW Defaults` um alte Einstellungen zu löschen!

# Download

Du kannst die fertigen Binärdateien hier herunterladen: https://github.com/knutwurst/Marlin-2-0-x-Anycubic-i3-MEGA-S/releases. Natürlich kannst du die Firmware auch selbst mit PlatformIO oder der Arduino IDE selbst kompilieren.

-> `i3_Mega.hex` ist für den unmodifizierten originalen Anycubic i3 MEGA (ohne S).

-> `i3_Mega_TMC.hex` hat Optimierungen und invertierte Ausgänge für TMC2100 and TMC2208 Motortreiber.

-> `i3_Mega_S.hex` wird verwendet, wenn du den neuere "S" Version mit Titan-Extruder verwendest.

-> `i3_Mega_S_TMC.hex` benutzt du, wenn du sowohl die "S" Version mit Titan-Extruder und TMC Motortreiber verwendest.




# Readme - English

- coming soon -
