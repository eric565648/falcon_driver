//////////////////////////////////
// Novint Falcon Ros Space Mode
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

const double G=9.8;

typedef std::chrono::high_resolution_clock Time;
typedef std::chrono::duration<double> dsec;

class spaceFalcon{
private:

    const gmtl::Vec3d origin=gmtl::Vec3d(0,0,0);
    ros::Publisher force_pub;
    ros::Publisher joy_pub;
    ros::Subscriber pos_sub;
    ros::Subscriber feedback_sub;
    geometry_msgs::Wrench feedback_wrench= geometry_msgs::Wrench();
    double last_error,last_z_center,last_z_range;
    gmtl::Vec3d last_pos;
    ros::Time last_t;
    gmtl::Vec3d cum_error=gmtl::Vec3d(0,0,0);
    bool first_loop=true;
    bool activate_wall=false;
    ros::NodeHandle*nhptr;

public:
    spaceFalcon(ros::NodeHandle*);
    ~spaceFalcon();
    void falcon_pos_cb(const sensor_msgs::JointState&);
    void falcon_joy_feedback_cb(const geometry_msgs::Wrench&);
};

spaceFalcon::spaceFalcon(ros::NodeHandle*nh){

    // set pid parameter
    nh->setParam("joy_Kp",30);
    nh->setParam("joy_Ki",10);
    nh->setParam("joy_Kd",0);
    nh->setParam("joy_mass",0.1);
    nh->setParam("joy_center_z",27.5);
    nh->setParam("joy_range_z",5);
    nh->setParam("joy_wall_stiffness",1);
    nh->setParam("joy_max_distance",40);
    nhptr=nh;

    // publisher init
    force_pub=nh->advertise<const geometry_msgs::Wrench>("falcon_wrench_input",1);
    joy_pub=nh->advertise<const sensor_msgs::Joy>("falcon_joy",1);
    // subscriber init
    pos_sub=nh->subscribe("falcon_cart_state",1,&spaceFalcon::falcon_pos_cb,this);
    feedback_sub=nh->subscribe("falcon_joy_feedback",1,&spaceFalcon::falcon_joy_feedback_cb,this);
}
spaceFalcon::~spaceFalcon(){
}
void spaceFalcon::falcon_pos_cb(const sensor_msgs::JointState &msg_state){

    ros::Time time_now=ros::Time::now();

    gmtl::Vec3d pos_now=gmtl::Vec3d(msg_state.position[0],msg_state.position[1],msg_state.position[2]);

    // cout<<pos_now<<endl;

    if(first_loop){
        last_t=time_now;
        last_pos=pos_now;
        first_loop=false;
        nhptr->getParam("joy_center_z",last_z_center);
        nhptr->getParam("joy_range_z",last_z_range);
        return;
    }

    double t_elas=time_now.toSec()-last_t.toSec();
    // cout<<t_elas<<endl;

    // Joy pose error
    gmtl::Vec3d error=pos_now-origin;
    cum_error = cum_error+error*t_elas;
    gmtl::Vec3d rateerror = (pos_now-last_pos)/t_elas;

    // Joy force feedback
    geometry_msgs::Wrench wrench;
    wrench=feedback_wrench;
    // gravity compensation
    double joy_mass;
    nhptr->getParam("joy_mass",joy_mass);
    double force_y = joy_mass;
    wrench.force.y+=force_y*G;

    // constrain to a plain
    double z_center,z_range,z_min,z_max,wall_stiffness;
    nhptr->getParam("joy_center_z",z_center);
    nhptr->getParam("joy_range_z",z_range);
    z_min=z_center-z_range/2;
    z_max=z_center+z_range/2;

    if((last_z_center!=z_center)||(last_z_range!=z_range)) activate_wall=false;

    if(activate_wall){
        nhptr->getParam("joy_wall_stiffness",wall_stiffness);
        if (pos_now[2]<z_min) wrench.force.z+=(z_min-pos_now[2])*wall_stiffness;
        if (pos_now[2]>z_max) wrench.force.z+=(z_max-pos_now[2])*wall_stiffness;
    }
    else{
        if ((pos_now[2]>=z_min)&&(pos_now[2]<=z_max)){
            activate_wall=true;
            cout<<"Activate Wall"<<endl;
        }
    }
    last_z_center=z_center;
    last_z_range=z_range;
    
    // publish force feedback
    force_pub.publish(wrench);

    // Joy stick value
    double max_distance,joy_x,joy_y,joy_z;
    // nhptr->getParam("joy_max_distance",max_distance);
    // // cout<<error<<endl;
    // joy_x=abs(error[0])>7.5?error[0]:0;
    // joy_x=abs(joy_x)>max_distance?max_distance:joy_x;
    // joy_x=joy_x/max_distance;
    // joy_y=abs(error[1])>7.5?error[1]:0;
    // joy_y=abs(joy_y)>max_distance?max_distance:joy_y;
    // joy_y=joy_y/max_distance;
    // joy_z=abs(error[2])>7.5?error[2]:0;
    // joy_z=abs(joy_z)>max_distance?max_distance:joy_z;
    // joy_z=joy_z/max_distance;
    // sensor_msgs::Joy msg_joy;
    // msg_joy.axes.push_back(joy_x);
    // msg_joy.axes.push_back(joy_y);
    // msg_joy.axes.push_back(joy_z);

    sensor_msgs::Joy msg_joy;
    joy_x=pos_now[0];
    joy_y=pos_now[1];
    joy_z=pos_now[2];
    msg_joy.axes.push_back(joy_x);
    msg_joy.axes.push_back(joy_y);
    msg_joy.axes.push_back(joy_z);

    msg_joy.buttons.push_back(msg_state.position[3]);
    msg_joy.buttons.push_back(msg_state.position[4]);
    msg_joy.buttons.push_back(msg_state.position[5]);
    msg_joy.buttons.push_back(msg_state.position[6]);
    msg_joy.header.stamp=ros::Time::now();
    
    for(int i;i<6;i++)
        msg_joy.axes.push_back((double)0);
        msg_joy.buttons.push_back((double)0);
    
    joy_pub.publish(msg_joy);

    last_t = time_now;
}
void spaceFalcon::falcon_joy_feedback_cb(const geometry_msgs::Wrench &msg_wrench){
    feedback_wrench=msg_wrench;
}


int main(int argc, char* argv[]){
    
    // initialize ros
    ros::init(argc,argv,"space_falcon");
    ros::NodeHandle nh;

    // joy initialize
    spaceFalcon jf = spaceFalcon(&nh);

    ros::spin();
}
