#include "masterslave/task/task.h"

void Task::calcQ6()
{
    toolAnglesAct.Q6 = (motorAngles.Q6P + motorAngles.Q6N) / 2;

    if(fabs(motorAngles.Q6P-motorAngles.Q6N)<0)
        gripper_stop = true;
    else
        gripper_stop = false;
}

void Task::commandVelocities()
{
    double gripperVelocity;
    std_msgs::Float64 Q4Vel, Q5Vel, Q6nVel, Q6pVel;
    Q4Vel.data = (toolAnglesTar.Q4 - toolAnglesAct.Q4)/cycleTime;
    Q5Vel.data = (toolAnglesTar.Q5 - toolAnglesAct.Q5)/cycleTime;
    if(gripper_close && !gripper_open && !gripper_stop)
    {
        gripperVelocity = gripperVelocityValue;
    }
    else if(gripper_open && !gripper_close && !gripper_stop)
    {
        gripperVelocity = -gripperVelocityValue;
    }
    else
    {
        gripperVelocity = 0;
    }

    Q6nVel.data = (toolAnglesTar.Q6-toolAnglesAct.Q6)/cycleTime;
    Q6pVel.data = (toolAnglesTar.Q6-toolAnglesAct.Q6)/cycleTime;
    // Stoppen der Greiferbacken, wenn eine der beiden am Anschlag ist, um Greiferöffnungswinkel nicht zu ändern
    if(motorAngles.Q6P>=0.95*M_PI && (toolAnglesTar.Q6-toolAnglesAct.Q6)>0)
    {
        Q6nVel.data = 0;
    }
    if(motorAngles.Q6N<=-0.95*M_PI && (toolAnglesTar.Q6-toolAnglesAct.Q6)<0)
    {
        Q6pVel.data = 0;
    }

    Q6nVel.data += gripperVelocity/cycleTime;
    Q6pVel.data -= gripperVelocity/cycleTime;

    Q4Pub.publish(Q4Vel);
    Q5Pub.publish(Q5Vel);
    Q6nPub.publish(Q6nVel);
    Q6pPub.publish(Q6pVel);

}

void Task::configurationCallback(masterslave::masterslaveConfig &config, uint32_t level)
{
    apertureLimit = config.apertureLimit;
    gripperVelocityValue = config.gripperVelocity;
    heightSafety = config.safetyHeight;
}

Eigen::Quaternion<double> Task::QuaternionFromEuler(const Eigen::Vector3d &eulerXYZ, bool ZYX=true)
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
