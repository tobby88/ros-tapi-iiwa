#ifndef LAPAROSCOPE_H
#define LAPAROSCOPE_H

#include "kinematics.h"
#include "Eigen/Dense"
#include "masterslave/descriptionparameters.h"

class Laparoscope: public Kinematics {
    public:
        Laparoscope(Eigen::Affine3d);
        Eigen::Affine3d getT_0_FL(){return T_0_FL;}
        void setAngles(const toolAngles value);
        Eigen::Affine3d getT_0_Q4(){return T_0_Q4;}

    private:
        //Implementation of the inhereted methods of Kinematics class
        void calcDirKin();
        void calcInvKin();

        Eigen::Affine3d T_0_FL;



};

#endif // LAPAROSCOPE_H
