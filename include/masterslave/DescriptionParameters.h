#ifndef DESCRIPTIONPARAMETERS_H
#define DESCRIPTIONPARAMETERS_H

// geometric tool description
struct toolDescriptionParameters{
    const double X_0_Q4;
    const double Y_0_Q4;
    const double Z_0_Q4;
    const double A_0_Q4;
    const double B_0_Q4;
    const double C_0_Q4;
    const double L_Q5_Q6;
    const double L_Q6_EE;
    const double X_RCM;
};

struct toolAngles{
    double Q4;
    double Q5;
    double Q6;
};

struct toolMotorAngles{
    double Q4;
    double Q5;
    double Q6N;
    double Q6P;
};

struct lbrDescriptionParameters
{
    const double L_0_Q1;
    const double L_Q1_Q2;
    const double L_Q2_Q3;
    const double L_Q3_Q4;
    const double L_Q4_Q5;
    const double L_Q5_Q6;
    const double L_Q6_Q7;
    const double L_Q7_FL;
};

struct lbrJointAngles
{
    double Q1;
    double Q2;
    double Q3;
    double Q4;
    double Q5;
    double Q6;
    double Q7;
};


#endif // DESCRIPTIONPARAMETERS_H
