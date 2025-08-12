## Knutwurst's i3 MEGA M/S/P/X/Chiron/4MP2 Hybrid Firmware <br>(based on Marlin 2.1.x)

## Fork purpose: Avoid nozzle heat loss during bed heating at the start of a print (new parameter C for G-code M104)

## For a detailed description, license etc. please see the [original repository](https://github.com/knutwurst/Marlin-2-0-x-Anycubic-i3-MEGA-S).

This fork attempts to fix or at least alleviate the problem of nozzle heat loss while the printer is waiting for the bed to heat up.
Cura, for example, by default emits G-code that waits for 60 seconds after bed heating starts until nozzle heating begins.
When experimenting with new filament types it is often the case that settings need to be tweaked; so, printing is stopped relatively early
and the model is sliced again. If it takes only a few seconds until the print can be restarted the bed and nozzle do not fully cool down.
As the bed typically requires more time to heat up than the nozzle it is being given a "head start" of 60 seconds.

However, if the nozzle is not being heated during this time, it cools down by a lot and needs to be re-heated once the 60 seconds are up.
This results in an annoying loss of time, especially if a print run needs to be re-started often.

Typical Cura G-code looks like this (from _Machine Settings_, _Start G-code_):

	...
	M107          ; start with the fan off
	M140 S80      ; Start heating the bed 
	G4 S60        ; wait 1 minute 
	M104 S235     ; start heating the hot end 
	M190 S80      ; wait for bed 
	M109 S235     ; wait for hotend 
	...

This fork attempts to alleviate this problem by adding a new parameter to the [M104 G-code](https://marlinfw.org/docs/gcode/M104.html): **`C<min_temp_in_celsius>`**

During the start of a print, right before bed heating begins, the slicer should emit a new command like e. g.:

    M104 C100

This command sets the current extruder's target nozzle temperature to its **c**urrent value, but at least to 100 °C if the current value is less than that.
It overrides any following `S<value>` specifications. If `I` has been specified it has no effect. It should not be used with the `F` autotemp flag;
at least this behavior has not been tested. It _should_ work on machines with multiple extruders (i. e. it respects the `T<index>` parameter) but this case still needs to be tested as well.

The effect of this line is that the nozzle temperature does not drop as much during bed heating, or is set to a sensible value when starting with a cold nozzle.
Subsequent nozzle heating takes less valuable time and conserves the user's nerves. 

Example for new Cura _Start G-code_:

	...
	M107          ; start with the fan off
	M104 C100     ; avoid nozzle heat loss
	M140 S80      ; Start heating the bed
	...

This patch does not modify existing lines of code so it should (theoretically) not break anything if the new C parameter is not being used.
Further documentation than what's in the source code is not yet provided.

Ideally this patch could be merged upstream into the Marlin repository. However, I have no proper way to test the effects as I do with Knutwurst's firmware.

There may still be some minor issues (behavior with autotemp, conflicting parameters, ...) but the basic functionality works (at least on my machine ;-)
