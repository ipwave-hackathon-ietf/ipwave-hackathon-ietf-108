
CNP (Context-Aware Navigation Protocol)

Introduction on CNP
____________________

In this project,  we implement the CNP simulation. Is is a protocol that we developed to enhance the safety of vehicles driving in urban roads.
CNP senses vehicle driving environment, study the collision risks and plans the safe trajectory of a driving vehicle by using lateral maneuvers.
CNP is simulated using OMNeT++ and SUMO on top of Veins.




Simulation Environnement
_________________________

To run CNP simulation requires:
- Install OMNeT++ 5.4.1 in lunix environment
- Install SUMO Version 0.32.0



Necessary Modifications
_______________________
Importing your cnp-veins project in your OMNeT++ workspace or extract it to run it in command line environment.


Running CNP
___________________

Using OMNeT++ workspace environment

- Make sure you run SUMO (./sumo-launchd.py)
- Now in OMNeT++ environment, compile and run.

Using command Line:

- in your VREP directory, run SUMO (./sumo-launchd.py)
- in your terminal,  open cnp-veins location and type: make, to compile
- run the simulation by: ./run



Hope you enjoy our CNP Simulation in OMNeT++ and SUMO
