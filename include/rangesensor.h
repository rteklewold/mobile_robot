#ifndef RANGESENSOR_H
#define RANGESENSOR_H

#include <iostream>
#include <vector>
#include <math.h>
#include <string>
#include <envir.h>
#include <robot.h>
#include <sensor.h>


//using namespace arpro;
using std::endl;using std::cout;

namespace arpro
{
class RangeSensor : public Sensor
{
public:
    RangeSensor(Robot &_robot , double _x , double _y , double _theta ): Sensor(_robot , _x , _y , _theta ) {}


    void update(const Pose &_p){
        //cout << "war is peace" <<endl;
        //Q 3.1.4
        s_=20;
        Pose p1 , p2 ;
        std::vector <double> d(envir_->walls.size(),0);
        for (unsigned int i =0; i<envir_->walls.size();++i )
        {
            p1 = envir_->walls[i];
            p2 = envir_->walls[(i+1)%envir_->walls.size ()];
            // do whatever you want to do with points p1 and p2
            //if(_p.theta!=PI/2)
            d[i]=(p1.x*p2.y-p1.x*_p.y-p2.x*p1.y+p2.x*_p.y+_p.x*p1.y-_p.x*p2.y)/(p1.x*sin(_p.theta)-p2.x*sin(_p.theta)-p1.y*cos(_p.theta)+p2.y*cos(_p.theta));
            cout << d[i] << endl;
            if(d[i]>0 && d[i]<s_)
            {
                s_=d[i];
            }
        }


    }
    void correctTwist(Twist &_v) {
        //cout << "Freedom is slavery" <<endl;
        //Q.3.1.5
        double Sm=0.1;
        double g=0.1;
        if(s_<3){
            if ((_v.vx)>(g*(s_-Sm))){
                _v.vx=g*(s_-Sm);
            }
        }
        //return (_v);
    }
}; // the RangeSensor constructor does nothing more
}

#endif // RANGESENSOR_H
