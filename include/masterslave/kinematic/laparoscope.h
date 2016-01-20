#ifndef LAPAROSCOPE_H
#define LAPAROSCOPE_H

#include "kinematics.h"
#include "Eigen/Dense"
#include "masterslave/descriptionparameters.h"

class Laparoscope: public Kinematics {
    public:
        Laparoscope(Eigen::Affine3d);
    private:
        void setAngles(const toolAngles value);
        void calcDirKin();
        void calcInvKin();


};

#endif // LAPAROSCOPE_H
