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
    static const lbrDescriptionParameters LBR_PARAMETERS;
    lbrJointAngles lbrJointAnglesAct;
    Eigen::Affine3d T_0_FL;
};

#endif // URSULAKINEMATICS_H
