#ifndef BEARINGSENSOR_H
#define BEARINGSENSOR_H

#include <iostream>
#include <vector>
#include <math.h>
#include <string>
#include <envir.h>
#include <robot.h>
#include <sensor.h>

using std::endl;using std::cout;

namespace arpro
{

class BearingSensor : public Sensor
{
public:
    BearingSensor(Robot &_robot , double _x , double _y , double _theta ): Sensor(_robot , _x , _y , _theta ) {} // the BearingSensor constructor does nothing more

    void update(const Pose &_p){
        // look for first other robot
        //cout << "This is bearing sensor update method" <<endl;
        for (auto other: envir_->robots_){
            if (other!=robot_)
            {
        // compute angle between sensor and detected robot
                angle=atan2(other->pose().y-_p.y,other->pose().x-_p.x)-_p.theta;
                break;
            }

        }


        // set angle back to [ - pi , pi ]
        angle=fmod(angle+M_PI, 2*M_PI);
        if(angle<0)
            angle=angle+2*M_PI;
        angle=angle-M_PI;


    }
    void correctTwist(Twist &_v){
        //cout << "This is bearing sensor correct twist method" <<endl;
        double g=0.5;
        _v.w=_v.w-g*angle;
        //return(_v.w);*/
    }
};
}

#endif // BEARINGSENSOR_H
