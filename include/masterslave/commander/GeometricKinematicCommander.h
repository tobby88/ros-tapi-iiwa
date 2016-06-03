#ifndef GEOMETRICKINEMATICCOMMANDER_H
#define GEOMETRICKINEMATICCOMMANDER_H

#include "ICommander.h"
#include "masterslave/kinematic/GeometricKinematic.h"
#include "ros/ros.h"
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_listener.h>

#include "staticFunctions.h"

/**
 * @file GeometricKinematicCommander.h
 *
 * @class GeometricKinematicCommander
 * @brief Klasse, die sich um die ROS-Kommunikation für die geometrisch bestimmte Kinematik kümmert
 *
 * @author Fabian Baier
 * @date 19.03.2016
 * @see ICommander
 */


class GeometricKinematicCommander : public ICommander
{
    public:
        GeometricKinematicCommander(ros::NodeHandle& nh, ros::NodeHandle& drNH);


    private:

        /**
         * @fn statemachineThread
         * @brief Thread der alle 20ms aufgerufen wird und den Status an ROSOpenIGTLBridge schickt
         * @see ICommander
         */
        void statemachineThread(const ros::TimerEvent&);

        /**
         * @fn calcQ6
         * @brief Berechnet aus dem Gelenkwinkel Q6 die Gelenkwinkel Q6n und Q6p des Zangengreifers
         * @see ICommander
         *
         */
        //void calcQ6();

        /**
         * @fn commandVelocities
         * @brief Schickt die Gelenkwinkelgeschwindigkeiten an die Nodes der Antriebe des Werkzeuges
         * @see ICommander
         */
        //void commandVelocities();

        void pliersDistanceCallback(const std_msgs::Float64ConstPtr &value);

        /**
         * @fn buttonCheck
         * @brief Die Methode holt die Knopfkonfiguration vom Parameterserver
         * @see ControlDevice
         */
        void buttonCheck();

        /**
         * @fn getControlDevice
         * @brief Die Methode findet den Typ des Eingabegerätes auf dem Parameterserver
         * @see ControlDevice
         */
        void getControlDevice();

        /**
         * @var lbrTargetPositionPub
         * @brief Sender der LBR-Flanschlage in ROS
         */
        ros::Publisher lbrTargetPositionPub;

        /**
         * @var velocity_
         * @todo check ob noch benötigt
         */
        geometry_msgs::TwistStamped velocity_;

        /**
         * @var flangeCallback
         * @brief Methode zum Empfang der aktuellen Roboterflanschlage
         */
        void flangeCallback(const geometry_msgs::PoseStampedConstPtr&);
        /**
         * @var loop
         * @see ICommander
         */
        void loop();
        ros::NodeHandle nh_;

        // Ein Haufen Callbacks ;)
        void Q4StateCallback(const sensor_msgs::JointStateConstPtr &state);
        void Q5StateCallback(const sensor_msgs::JointStateConstPtr &state);
        void Q6nStateCallback(const sensor_msgs::JointStateConstPtr &state);
        void Q6pStateCallback(const sensor_msgs::JointStateConstPtr &state);
        void velocityCallback(const geometry_msgs::TwistStampedConstPtr&);
        void buttonCallback(const masterslave::ButtonConstPtr&);
        /**
         * @var T_0_FL
         * @brief Roboterflanschlage des LBR iiwa
         */
        Eigen::Affine3d T_0_FL;



};


#endif // LAPAROSCOPETASK_H
