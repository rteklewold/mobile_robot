

#include <iostream>
#include <math.h>
#include <robot.h>
#include <sensor.h>
#include <rangesensor.h>
using namespace arpro;
using namespace std;

Environment* Sensor::envir_ = nullptr;

Robot::Robot(string _name, double _x, double _y, double _theta)
{    
    pose_.x = _x;
    pose_.y = _y;
    pose_.theta = _theta;

    name_ = _name;

    // init position history
    x_history_.push_back(_x);
    y_history_.push_back(_y);
}


void Robot::moveXYT(double _vx, double _vy, double _omega)
{
    // update position
    pose_.x += _vx*dt_;
    pose_.y += _vy*dt_;
    pose_.theta += _omega*dt_;

    // store position history
    x_history_.push_back(pose_.x);
    y_history_.push_back(pose_.y);
}

void Robot::initWheel(double _r, double _b){
    wheels_init_=true;
    r=_r;
    b=_b;
    wmax=10;
    }
void Robot::rotateWheels(double _left, double _right)
{
    /*// to fill up after defining an initWheel method
    //Q 2.2.6
    double _r=0;
    double _b=0;
    double* _wmax;
    initWheel(_r, _b, _wmax=0);

   if (wheels_init_){
       double vx_=r*((_left+_right)/2)*cos(pose_.theta);
       double vy_=r*((_left+_right)/2)*sin(pose_.theta);
       double omega_=r*((_left+_right)/2*b);
       moveXYT(vx_,vy_,omega_);
   }*/
   //Q 2.3.2
   if(wheels_init_)
   {
       double a=max(abs(_left)/wmax,abs(_right)/wmax);
       if (a<1){
           a=1;
       }
       else
       {
           cout << "angular velocities are high!" <<endl;
           _left=_left/a;
           _right=_right/a;
       }

   /*double _r=0;
   double _b=0;
   double* _wmax;
   initWheel(_r, _b, _wmax=NULL);*/

      double vx_=r*((_left+_right)/2)*cos(pose_.theta);
      double vy_=r*((_left+_right)/2)*sin(pose_.theta);
      double omega_=r*((_left-_right)/(2*b));
      moveXYT(vx_,vy_,omega_);
       //cout << r<< b <<endl;
  }
   else {
       cout<< "Wheels are not initialized"<<endl;
   }

   //double _v=r*(wl+wr)/2;
   //double _w=r*(wl-wr)/2*b;
}



// move robot with linear and angular velocities
void Robot::moveVW(double _v, double _omega)
{
    // to fill up
    // Q 2.2.3
    /*double vx_=_v*cos(pose_.theta);
    double vy_=_v*sin(pose_.theta);
    double omega_=_omega;
    Robot::moveXYT(vx_,vy_,omega_);*/

    //Q 2.3.3

    /*double _r=0;
    double _b=0;
    double* _wmax;
    initWheel(_r, _b, _wmax=NULL);*/
    if(wheels_init_){
    double _left=(_v+(b*_omega))/r;
    double _right=(_v-(b*_omega))/r;
    rotateWheels(_left,_right);
    }
 }




// try to go to a given x-y position
void Robot::goTo(const Pose &_p)
{
    // error in robot frame
    Pose error = _p.transformInverse(pose_);

    // try to do a straight line with sensor constraints
    moveWithSensor(Twist(error.x, error.y, 0));
}


void Robot::moveWithSensor(Twist _twist)
{
    // to fill up, sensor measurement and twist checking



    // uses X-Y motion (perfect but impossible in practice)
    //moveXYT(_twist.vx, _twist.vy,_twist.w);

    // to fill up, use V-W motion when defined
    //Q 2.2.5
    for (auto & sensor: sensors_){  // Q3.1.3
        sensor->updateFromRobotPose(pose_);
        sensor->correctRobotTwist(_twist);
        double alpha=20;
        double v_=_twist.vx;
        double omega_=(alpha*_twist.vy)+_twist.w;
        moveVW(v_,omega_);
    }

}


void Robot::printPosition() {
    cout << "Current position: " << pose_.x << ", " << pose_.y << endl;
}

