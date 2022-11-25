This is document is meant to detail how to use
this software should I be unable to attend the final event.

The Mission File (Mission.JSON)
The mission is split in to sections labelled as:
  Section_1
  Section_2
  etc...

The Flight Director:

This is the companion computer, the companion computer should be kept dumb and all heavy computation should be done before run time,
the companion computer should be simply following orders whenever possible. the clear exception is the HALO operation

each section has a Type, the type informs the flight director of how it should
behave in order to achieve the desired actions

Types:
  WP:
    Follows the waypoints listed
  PCR:
    A set of 3 waypoints that describes a line along which the cargo drop will occur
    during the traversal the flight director monitors the position and hence calculates when it should drop the cargo
  HALO:
    A set of 2 waypoints that describe the heading alignment point and final touchdown location

The Flight director simply parses over the json section by section

HOW TO:
Unless you have technical problems this is all you should need to do to run the computer

upload

Mission.JSON
Geo_Fence.JSON
Rally.JSON

the usb stick
move the drone to the box. DO NOT POWER ON THE DRONE AND THEN MOVE IT, The drone takes its power on location as home.
power on the drone
wait for the confirmation sounds
switch to auto.
The drone should takeoff within a few moments
leave the controller throttle at 0
if anything should go wrong switch the drone to FBWA flight mode for a controlled Landing or manual for a manual abort
