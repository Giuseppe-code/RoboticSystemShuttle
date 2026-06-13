# Robotic System Shuttle

The project focuses on the automatic placement of objects picked from a shelf (loading zone) into unloading zones.

## Project Specifications

Three zones are present: 2 for unloading and 1 for loading (Zone B).
The robot picks objects from the shelf and sorts them by color: blue objects are delivered to Zone A, red objects to Zone C.

A robotic arm handles the picking operation. The object is held in an optimal position until the robot reaches the destination zone, where it is deposited.

Zones are color-coded: Zone A = Blue, Zone C = Red. Zone B contains an elevated shelf.

Objects have variable weight and fixed size. At the start of the simulation, packages are generated within predefined weight ranges.

> **Current scope:** one loading zone and one unloading zone are implemented.


# features to be implemented
1. complete arm logic combined with the cart.
2. shelf creation, pick up area within the shelf, generating objects inside the pick up area on top of the shelf.
3. code refactoring
