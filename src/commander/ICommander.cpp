#include "masterslave/commander/ICommander.h"



void ICommander::configurationCallback(masterslave::MasterSlaveConfig &config, uint32_t level)
{
    state = static_cast<OPENIGTL_STATE>(config.cur_state);
    rosRate = config.rosRate;
    setTrocar = config.set_Trocar;
    ROS_INFO("STATE CHANGED");
}

Eigen::Quaternion<double> ICommander::QuaternionFromEuler(const Eigen::Vector3d &eulerXYZ, bool ZYX=true)
{
    Eigen::Quaternion<double> quat;
    quat.Identity();
    Eigen::AngleAxisd zAngle(eulerXYZ[2], Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yAngle(eulerXYZ[1], Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd xAngle(eulerXYZ[0], Eigen::Vector3d::UnitX());
    if(ZYX)
        quat = zAngle * yAngle * xAngle;
    else
        quat = xAngle * yAngle * zAngle;

    return quat;
}

void ICommander::commandVelocities()
{
    double gripperVelocity;
    std_msgs::Float64 Q4Vel, Q5Vel, Q6nVel, Q6pVel;
    Q4Vel.data = (jointAnglesTar.bottomRows(3)(0) - jointAnglesAct.bottomRows(3)(0));
    Q5Vel.data = (jointAnglesTar.bottomRows(3)(1) - jointAnglesAct.bottomRows(3)(1));
    if(gripper_close && !gripper_open && !gripper_stop)
    {
        gripperVelocity = gripperVelocityValue;
    }
    else if(gripper_open && !gripper_close && !gripper_stop)
    {
        gripperVelocity = -gripperVelocityValue;
    }

    if(gripper_stop)
    {
        gripperVelocity = 0;
    }
    if(motorAngles(1)<=CRITICAL_PLIERS_ANGLE && motorAngles(1)>=-CRITICAL_PLIERS_ANGLE && motorAngles(0)>=-CRITICAL_PLIERS_ANGLE && motorAngles(0)<=CRITICAL_PLIERS_ANGLE)
    {
    Q6nVel.data = (jointAnglesTar.tail(3)(2)-jointAnglesAct.tail(3)(2));
    Q6pVel.data = (jointAnglesTar.tail(3)(2)-jointAnglesAct.tail(3)(2));
    }

    ROS_INFO_STREAM(Q6nVel.data << " " <<  Q6pVel.data);

    // Stoppen der Greiferbacken, wenn eine der beiden am Anschlag ist, um Greiferöffnungswinkel nicht zu ändern
    if(motorAngles(1)<=-CRITICAL_PLIERS_ANGLE && (jointAnglesTar.bottomRows(3)(2)-jointAnglesAct.bottomRows(3)(2))<0)
    {
        Q6pVel.data = 0;
    }
    if(motorAngles(0)>=CRITICAL_PLIERS_ANGLE && (jointAnglesTar.bottomRows(3)(2)-jointAnglesAct.bottomRows(3)(2))>0)
    {
        Q6nVel.data = 0;
    }
    Q6nVel.data -= gripperVelocity;
    Q6pVel.data += gripperVelocity;

    //ROS_INFO_STREAM(Q6pVel.data);

    Q4Pub.publish(Q4Vel);
    Q5Pub.publish(Q5Vel);
    Q6nPub.publish(Q6nVel);
    Q6pPub.publish(Q6pVel);

}

void ICommander::calcQ6()
{
    if(motorAngles(0) - motorAngles(1)<0.0 && gripper_close)
    {
        gripper_stop = true;
    }
    else
    {
        gripper_stop = false;
        // reset the callback counter
    }
    Q6CallbacksCalled = 0;
    jointAnglesAct.tail(3)(2) = (motorAngles(0) + motorAngles(1)) / 2;

}


