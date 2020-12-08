# Intersection-Controller
Implementing intersection traffic control in FreeRTOS using STM32 Microcontroller.

![me](https://github.com/ammaralvi21/Intersection-Controller/blob/main/Docs/Operation.gif)

# Project Description
The purpose of this project was to implement a four way traffic intersection controller which was prototyped by interfacing the control logic with LEDs.
The project had real time requirements for traffic lights and walk lights, and they were to transition in a timely manner.
See the timing diagram below for which our state machine was derived:
![me](https://github.com/ammaralvi21/Intersection-Controller/blob/main/Docs/TimingDiagram.PNG)

 See below for state diagram of the Traffic Lights derived from timing diagram:
 ![me](https://github.com/ammaralvi21/Intersection-Controller/blob/main/Docs/Static%20Cyclic%20Mode%20State%20Machine.png)

The intersection has 2 intersecting roads and supports 1 *or more*
lanes of vehicular traffic, as well as pedestrian traffic, in all 4
directions. 

The intersection timing detailed requirements can be found at: https://github.com/ammaralvi21/Intersection-Controller/blob/main/Docs/intersection-timing.pdf 

The intersection controller has multiple operating modes to
accomodate different traffic situations, including
   1. **FSM: failsafe mode** (4-way flashing red, at 0.5Hz with a 75%
      duty cycle)
   2. **SCM: static cycle mode** (fixed cycle times, does not respond to
      pedestrians, ERV, or BRT)
   3. The intersection controller shall have a mode **ATM: Accelerated
      **Test Mode**.  ATM mode allows an observer to visually verify
      correct operation.  It allows the system to be sped up by a given
      factor, for instance 10x, or 100x.  The point is to make
      the sequencing faster so the observer can watch it for seconds
      instead of minutes.

The Walk light (blue LED) has three states, with the following indications:
   1. When in Walk mode, the Walk light is on
   1. When in Walk Warning mode, the Walk light blinks at 1 Hz, with a
      50% duty cycle.
   2. When in Don't Walk mode, the Walk light is off.
   
When the system starts up it enters FSM.

## Command Line Interface
 The command line interface supports the following command:
   1. **help**: display a list of valid commands
   1. **mode fsm**: switch to Failsafe mode
   1. **mode scm**: switch to Static Cycle mode
   1. **atm x**: enter accelerated test mode with multiplication
      factor *x*. 
There is a status window at the top of the CLI that displays the current mode and status of 
the traffic lights.See below.

![me](https://github.com/ammaralvi21/Intersection-Controller/blob/main/Docs/StatusWindow.PNG)

As seen the from the status window, the traffic lights on the left are the primary and the ones on
top and bottom are secondary. The color of the respective traffic light becomes brighter indicating that its
active.
For blue walking light it becomes active when it is brighter and has the letter "W" written in it indicating
a walking state. When it transitions to Walk Warning state, it stays bright but it has the letter "S" written inside
which to warn the person about to cross the road to stop.
 
See below for a log of the CLI
<pre>
ItC-CLI > help

                                 HELP MENU:
|===================|=====================================================|
|{Commands}         |    {Description}                                    |
|===================|=====================================================|
|'mode fsm'         |    Switch to Failsafe mode                          |
|-------------------|-----------------------------------------------------|
|'mode scm'         |    Switch to Static Cycle mode                      |
|-------------------|-----------------------------------------------------|
|'atm [x]'          |    Enter Accelerated test mode with multiplication  |
|                   |    factor x where x is between 1 and 100            |
|-------------------|-----------------------------------------------------|
|'help'             |    Show HELP menu                                   |
|-------------------|-----------------------------------------------------|

ItC-CLI > mode fsm

!! Switched to Failsafe mode !!

ItC-CLI > mode scm

!! Switched to Static Cycle mode !!

ItC-CLI > atm 0

ERROR! ATM multiplier should be between 1 and 100
Please try again! Or Type 'help' for more info

ItC-CLI > atm 101

ERROR! ATM multiplier should be between 1 and 100
Please try again! Or Type 'help' for more info

ItC-CLI > atm 5

!! Accelerated test mode with multiplication factor: 5 !!

ItC-CLI >
</pre?

Explanation Video
https://urcourses-video.uregina.ca/p/104/sp/10400/serveFlavor/entryId/0_ifqxabsv/v/2/ev/7/flavorId/0_1meou7a4/forceproxy/true/name/a.mp4
