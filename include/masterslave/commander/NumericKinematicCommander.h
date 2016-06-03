#ifndef URSULATASK_H
#define URSULATASK_H

#include <array>

#include "ICommander.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"

#include "masterslave/NumericKinematicRCM.h"
#include "masterslave/NumericKinematicDirectKinematics.h"
#include "masterslave/NumericKinematicInverseKinematics.h"


#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_listener.h>

#include "staticFunctions.h"

/**
 * @file NumericKinematicCommander.h
 *
 * @class NumericKinematicCommander
 * @brief Klasse, die sich um die ROS-Kommunikation für die numerisch bestimmte Kinematik kümmert
 *
 * @author Fabian Baier
 * @date 20.03.2016
 *
 * @see ICommander
 */

class NumericKinematicCommander: public ICommander
{
    public:
        NumericKinematicCommander(ros::NodeHandle& nh, ros::NodeHandle& drNH);
    private:

        /**
         * @fn statemachineThread
         * @brief Thread der alle 20ms aufgerufen wird und den Status an ROSOpenIGTLBridge schickt
         * @see ICommander
         */
        void statemachineThread(const ros::TimerEvent&);

        /**
         * @fn configurationCallback
         * @brief Dynamic Reconfigure Callbackmethode
         * @param config Dynamic Reconfigure Struktur
         * @param level Bitmaske
         * @see GeometricKinematicCommander, MasterSlaveConfig.cfg
         */
        void configurationCallback(masterslave::MasterSlaveConfig &config, uint32_t level);
        ros::NodeHandle nh_;

        /**
         * @fn buttonCheck
         * @brief Die Methode holt die Knopfkonfiguration vom Parameterserver
         * @see ControlDevice
         */
        void buttonCheck();

        void pliersDistanceCallback(const std_msgs::Float64ConstPtr &value);

        /**
         * @fn getControlDevice
         * @brief Die Methode findet den Typ des Eingabegerätes auf dem Parameterserver
         * @see ControlDevice
         */
        void getControlDevice();

        /**
         * @fn calcQ6
         * @brief Berechnet aus dem Gelenkwinkel Q6 die Gelenkwinkel Q6n und Q6p des Zangengreifers
         * @see ICommander
         */
        //void calcQ6();

        /**
         * @fn commandVelocities
         * @brief Schickt die Gelenkwinkelgeschwindigkeiten an die Nodes der Antriebe des Werkzeuges
         * @see ICommander
         */
        //void commandVelocities();

        /**
         * @var lbrJointAnglePub
         * @brief Array mit den Sendern für die LBR-Gelenkwinkel
         */
        std::array<ros::Publisher,7> lbrJointAnglePub;

        /**
         * @var lbrJointAngleSub
         * @brief Array mit den Empfängern für die LBR-Gelenkwinkeln
         */
        std::array<ros::Subscriber,7> lbrJointAngleSub;
        /**
         * @fn lbrJointAngleCallback
         * @brief Multicallback für die Gelenkwinkel
         * @param state
         * @param number
         */
        void lbrJointAngleCallback(const sensor_msgs::JointStateConstPtr& state, int number);

        void Q4StateCallback(const sensor_msgs::JointStateConstPtr &state);
        void Q5StateCallback(const sensor_msgs::JointStateConstPtr &state);
        void Q6nStateCallback(const sensor_msgs::JointStateConstPtr &state);
        void Q6pStateCallback(const sensor_msgs::JointStateConstPtr &state);
        void buttonCallback(const masterslave::ButtonConstPtr&);

        /**
         * @var startPositionLBR
         * @brief startPositionLBR des LBRs
         */
        Eigen::Affine3d startPositionLBR;

        /**
         * @fn loop
         * @see ICommander
         */
        void loop();

        /**
         * @var lbrJointAngles
         * @todo check ob noch benötigt
         */
        Eigen::VectorXd lbrJointAngles;



};

#endif // URSULATASK_H
