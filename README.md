# elevator-control-unit
**TLDR - Using C and a PIC18F4520 microcontroller for a hypothetical elevator control unit.**
A hypothetical company has been tasked with designing and implementing an Elevator Control Unit (ECU) for a 3-story building. The sequence of operations the ECU will perform to operate the elevator are yet to be defined by the client therefore, stage 1 will only correctly operate each component of the elevator hardware and test all interface routines. The overall functions for this ECU will include:
  •	4 normally open switch contact buttons which will call the elevator to the user.
  •	3 normally open momentary buttons which will select the user’s destination.
  •	1 normally closed set of switch contacts which will detect if the door is ajar.
  •	Opening and closing the elevator doors through a 24V relay
  •	Determining what floor, the elevator is on within +/-20mm using a position sensor pointed at the ground.
  •	Display what floor the elevator is on and the recording of the position sensor to mm.
  •	Send to input signals to the Winch unit through optical isolation which control the winches direction and speed using PWM.

The chosen microcontroller for this task is the PIC18F4520 due to its multiple built-in features such as analog to digital conversion (ADC), number of ports and its capture, compare, pwm module. Furthermore, it was decided that to display which floor the user was on the hd44780 16x2 LCD was to be used.
The first stage of this ECU design was to interface the PIC18F with the elevator equipment. This included finding the right components for an external crystal oscillator, optical isolation and a debounce circuit. Then determining which ports would be used for each piece of elevator equipment 
