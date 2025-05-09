# Reality Gap Tests

The purpose of this directory is to hold different tests that aim to test the reality gap between the simulation and the physical robot.

Bear in mind  that all these tests have been set up for the gecko V2 body and will not work out of the box for other morphologies (except the hinge test which automatically adapts to the number of hinges in the robot), but they serve as guide to do so. You will only need to change the body and hinge ids declarations in the code, and for the basic and movement tests, you will also have to design your own movement patterns to test out.

## Basic Test

The first step is to run `parallel_basic_test.py`. This test acts as an overall test that should allow you to check if the simulation and phisical robot traverse the same or similar distance. A big discrepancy in distance would be the first indicator of a problem causing a reality gap. If you are unsure you have correctly set up the hinge pin ids to match the ones in the simulation, you can run the next step to figure out the correct mapping.

## Hinge Movement and Mapping Test

The second step is to run the `parallel_hinge_test.py` file. This will run the simulation and the physical robot in parallel and check the correct functionality of the individual hinges of the robot, while simultaneusly allowing you to correctly map the hinges to the correct pins ont he robot.

## Movement Test
Finally, you can run the `parallel_movement_test.py` file. This will run the simulation and the physical robot in parallel and check different movement patterns which can help you identify a bit better the reality gap.


