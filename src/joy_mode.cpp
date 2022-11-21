//////////////////////////////////
// Novint Falcon Ros Joy Mode
// Based on lobnifalcon
// See: https://github.com/libnifalcon/libnifalcon
// Chen-Lung Eric Lu 11/17/2022

#include <iostream>
#include <string>
#include <cmath>
#include "falcon/gmtl/gmtl.h"
#include <chrono>

// ros 
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Wrench.h"
#include "sensor_msgs/Joy.h"

// using namespace libnifalcon;
using namespace std;

typedef std::chrono::high_resolution_clock Time;
typedef std::chrono::duration<double> dsec;

class joyFalcon{
private:

    const gmtl::Vec3d origin=gmtl::Vec3d(0,0,0);
    ros::Publisher force_pub;
    ros::Publisher joy_pub;
    ros::Subscriber pos_sub;
    double last_error;
    gmtl::Vec3d last_pos;
    ros::Time last_t;
    gmtl::Vec3d cum_error=gmtl::Vec3d(0,0,0);
    bool first_loop=true;
    ros::NodeHandle*nhptr;

public:
    joyFalcon(ros::NodeHandle*);
    ~joyFalcon();
    void falcon_pos_cb(const sensor_msgs::JointState&);
};

joyFalcon::joyFalcon(ros::NodeHandle*nh){

    // set pid parameter
    nh->setParam("joy_Kp",100);
    nh->setParam("joy_Ki",10);
    nh->setParam("joy_Kd",0);
    nh->setParam("joy_max_distance",40);
    nhptr=nh;

    // publisher init
    force_pub=nh->advertise<const geometry_msgs::Wrench>("falcon_wrench_input",1);
    joy_pub=nh->advertise<const sensor_msgs::Joy>("falcon_joy",1);
    // subscriber init
    pos_sub=nh->subscribe("falcon_cart_state",1,&joyFalcon::falcon_pos_cb,this);
}
joyFalcon::~joyFalcon(){
}
void joyFalcon::falcon_pos_cb(const sensor_msgs::JointState &msg_state){

    ros::Time time_now=ros::Time::now();

    gmtl::Vec3d pos_now=gmtl::Vec3d(msg_state.position[0],msg_state.position[1],msg_state.position[2]);

    // cout<<pos_now<<endl;

    if(first_loop){
        last_t=time_now;
        last_pos=pos_now;
        first_loop=false;
        return;
    }

    double t_elas=time_now.toSec()-last_t.toSec();
    // cout<<t_elas<<endl;

    // Joy force feedback
    gmtl::Vec3d error=pos_now-origin;
    cum_error = cum_error+error*t_elas;
    gmtl::Vec3d rateerror = (pos_now-last_pos)/t_elas;

    double Kp,Kd,Ki;

    nhptr->getParam("joy_Kp",Kp);
    nhptr->getParam("joy_Ki",Ki);
    nhptr->getParam("joy_Kd",Kd);

    double force_x = -1*(Kp*(error[0])+Kd*rateerror[0]+Ki*cum_error[0]);
    double force_y = -1*(Kp*(error[1])+Kd*rateerror[1]+Ki*cum_error[1]);
    double force_z = -1*(Kp*(error[2])+Kd*rateerror[2]+Ki*cum_error[2]);

    geometry_msgs::Wrench wrench;
    wrench.force.x=force_x/1000.; // because we use mm, convert to newtonw
    wrench.force.y=force_y/1000.;
    wrench.force.z=force_z/1000.;
    // cout<<wrench<<endl;
    force_pub.publish(wrench);

    // Joy msg
    double max_distance,joy_x,joy_y,joy_z;
    nhptr->getParam("joy_max_distance",max_distance);

    cout<<error<<endl;
    joy_x=abs(error[0])>7.5?error[0]:0;
    joy_x=abs(joy_x)>max_distance?max_distance:joy_x;
    joy_x=joy_x/max_distance;

    joy_y=abs(error[1])>7.5?error[1]:0;
    joy_y=abs(joy_y)>max_distance?max_distance:joy_y;
    joy_y=joy_y/max_distance;

    joy_z=abs(error[2])>7.5?error[2]:0;
    joy_z=abs(joy_z)>max_distance?max_distance:joy_z;
    joy_z=joy_z/max_distance;

    sensor_msgs::Joy msg_joy;
    msg_joy.axes.push_back(joy_x);
    msg_joy.axes.push_back(joy_y);
    msg_joy.axes.push_back(joy_z);
    joy_pub.publish(msg_joy);

    last_t = time_now;
}



int main(int argc, char* argv[]){
    
    // initialize ros
    ros::init(argc,argv,"joy_falcon");
    ros::NodeHandle nh;

    // joy initialize
    joyFalcon jf = joyFalcon(&nh);

    ros::spin();
}
