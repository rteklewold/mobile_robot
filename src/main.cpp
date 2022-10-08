#include <iostream>
#include <math.h>
#include <cmath>

#include <robot.h>
#include <envir.h>
#include <sensor.h>
#include <string>
#include <rangesensor.h>
#include <bearingsensor.h>

using namespace std;
using namespace arpro;

int main(int argc, char **argv)
{

  // default environment with moving target
  Environment envir;
  // sensors gets measurements from this environment
  Sensor::setEnvironment(envir);
  // init robot at (0,0,0)
  Robot robot("R2D2", 0,0,0);
  envir.addRobot(robot);
  robot.initWheel(0.07, 0.3);
  //range sensor variable
  //Q 3.1.2
  RangeSensor ingsoc(robot, 0.1,0,0);


  //Q 3.2.1
  Robot robot2("R3D3", 0,0,0);
  envir.addRobot(robot2);
  robot2.initWheel(0.05,0.3);
  // bearing sensor variable
  BearingSensor lucid(robot2, 0.1,0,0);

  // simulate 100 sec
  while(envir.time() < 100)
  {
    cout << "---------------------" << endl;

    // update target position
    envir.updateTarget();

    // try to follow target
    //robot.rotateWheels(12,12);
    //robot.moveVW(2, 2);
    robot.goTo(envir.target());

    //robot2.goTo(envir.target());


    robot2.moveWithSensor(Twist(0.4,0,0));

  }

  // plot trajectory
  envir.plot();

}
