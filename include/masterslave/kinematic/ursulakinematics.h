#ifndef URSULAKINEMATICS_H
#define URSULAKINEMATICS_H

#include "kinematics.h"

class UrsulaKinematics: public Kinematics
{
public:
    UrsulaKinematics(const Eigen::Affine3d);
private:
    void calcDirKin();
    //void calcInvKin();
    //geometroc description parameters of the LBR iiwa 14 R820
    static const lbrDescriptionParameters LBR_PARAMETERS;

    //joint angles of the LBR iiwa 14 R820
    lbrJointAngles lbrJointAnglesAct;

    Eigen::Affine3d T_0_FL;
};

#endif // URSULAKINEMATICS_H
