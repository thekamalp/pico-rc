This project uses a raspberry pico w to run a lego rc car
(although an rc car could be used), connecting to a game
controller via bluetooth.

A pico2 W board is also supported.  Add -DPICO2=1 argument
to the cmake command.

The LED on the pico flashes slowly when scanning for a device.
The LED flashes medium when trying to connect to the last known
controller.  The LED flashes rapidly when connected.

Controls:
Left x axis - steering
D-pad up - forward gear
D-pad down - reverse gear
Right trigger - accelerate
Left trigger - brake
Left button (button 1) - toggle lights (off, low beam, or high beam)

Servo feedback:
Lego power functions servos have 15 set positions that can be
controlled through pulse width modulation.  However many third-
party servos only have 3 positions (center, fully deflected left,
or fully deflected right).  In order to have finer steering control,
a small linear potentiometer can be connected to the servo output,
and this feedback signal can be sent to the pico.  The program will
deflect the steering left and right on boot up to determine if the
feedback is connected.  If this feedback is not needed (if a true
lego servo is used, or finer steer control is not needed), then the
feedback input on the pico can be connected to ground.

Wiring:
GP18 and GP19 of the pico should drive a pair of inputs on an H-bridge
motor driver (such as the TI DRV8833), supplied with 9V.  The corresponding
outputs of the H-bridge should connect to the servo control lines (the inner 
2 wires of a standard 4-wire lego connector).  GP20 and GND should connect 
to another 9V H-bridge input, which will then drive the power of the servo 
(the outer 2 wires of a standard 4-wire lego connector).

GP12 and GP13 should connect to another H-bridge (9-12V) whose output should
connect to power lines of the motor (outer 2 wires).

GP14 and GND should be connected to an H-bridge which drives the 9V lego LED
for the front headlight.  GP15 and GND should be connected to an H-bridge
to drive te rear lights.

GP26 should be connected to the output of the potentiometer connected to the
servo, if servo feedback is needed.  Otherwise, it should be connected to GND.
