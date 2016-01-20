#ifndef LAPAROSCOPETASK_H
#define LAPAROSCOPETASK_H

#include "task.h"
#include "masterslave/kinematic/laparoscope.h"
#include "ros/ros.h"
#include "masterslave/Button.h"
#include <geometry_msgs/TwistStamped.h>



#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_listener.h>

class LaparoscopeTask : public Task
{
    public:
        LaparoscopeTask(ros::NodeHandle& nh, double rosRate);

    private:
        double rosRate_;
        ros::Subscriber lbrPositionSub;
        ros::Subscriber velocitySub;
        ros::Subscriber buttonSub;
        geometry_msgs::TwistStamped velocity_;
        Eigen::Affine3d moveEEFrame(Eigen::Affine3d);
        void flangeCallback(const geometry_msgs::PoseStampedConstPtr&);
        void loop();
        ros::NodeHandle nh_;
        void Q4StateCallback(const sensor_msgs::JointStateConstPtr &state);
        void Q5StateCallback(const sensor_msgs::JointStateConstPtr &state);
        void Q6nStateCallback(const sensor_msgs::JointStateConstPtr &state);
        void Q6pStateCallback(const sensor_msgs::JointStateConstPtr &state);
        void velocityCallback(const geometry_msgs::TwistStampedConstPtr&);
        void buttonCallback(const masterslave::ButtonConstPtr&);
        void buttonCheck();
        std::vector<std::string> buttons;
        Laparoscope* kinematic;



};


#endif // LAPAROSCOPETASK_H
