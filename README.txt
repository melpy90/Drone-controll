 Perform a simulation of a quadcopter
 
 
 open loop:
 1. with the fixed angular velocities   -  you need to comment 69-70 lines,
 remains only input function
  2. with the deviations of angular velocities  the  lines 69-70 have to be uncommented
 
 
 closed loop:
 simulate(controller('pid'), 0, 3, 0.01) - start, time, step
 the resulting structure with all the parameters