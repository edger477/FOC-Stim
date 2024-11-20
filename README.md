FOC-Stim is an implementation of the [Restim](https://github.com/diglet48/restim) threephase signal generation algorithm
for the [B-G431B-ESC1](https://www.st.com/en/evaluation-tools/b-g431b-esc1.html) electronic speed controller.

It utilizes MRAC (model reference adaptive control) to generate consistent current-controlled waveforms
as environment conditions change.

Powered by SimpleFOC.

# Hardware setup

BOM:

* B-G431B-ESC1
* USB cable and 12v
* 3x transformer (XICON 42TU200-RC suggested)
* 3x inductor (100-470µH 1A, exact specs TBD)
* 3x ceramic caps (1-10µF 50v, exact specs TBD)

How to wire:

TODO

# Software setup

Install Visual Studio Code with plugins `platformio` and `teleplot`.  
Firmware should compile and upload without problems.

Inspect `FOC-Stim/src/config.h` to configure the current limits.

# Control

Control over serial with Restim.

View live stats with teleplot.