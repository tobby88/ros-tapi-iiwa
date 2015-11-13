#ifndef LAPRASCOPICTOOL_H
#define LAPRASCOPICTOOL_H

#include <Eigen/Geometry>
#include <vector>

class LaprascopicTool {
    public:
        LaprascopicTool(const Eigen::Affine3d startPoseLBR);
        void setAngles(std::vector<double>);
        Eigen::Vector3d getRCM(){return RemoteCenterOfMotion;}
        std::vector<double> getAngles();
	void setQ4(double);
	double getQ4(){ return Q4;}
        void setQ5(double);
        double getQ5(){ return Q5; }
        void setQ6(double);
        double getQ6() { return Q6; }
        void setT_0_EE(Eigen::Affine3d);
        Eigen::Affine3d getT_0_FL(){ return T_0_FL; }

    private:
	double Q4;
        double Q5;
        double Q6;
	//dirKin
        Eigen::Affine3d T_FL_Q4;
        Eigen::Affine3d T_Q4_Q5;
        Eigen::Affine3d T_Q5_EE;
	Eigen::Affine3d T_FL_EE;
	//iKin
	Eigen::Affine3d T_0_EE;
	Eigen::Affine3d T_0_FL;
        Eigen::Vector3d RemoteCenterOfMotion;


        struct toolDescriptionParameters{
            double X_0_Q4;
            double Y_0_Q4;
            double Z_0_Q4;
            double A_0_Q4;
            double B_0_Q4;
            double C_0_Q4;
            double L_Q5_Q6;
            double L_Q6_EE;
        } toolParameters;
        void calcDirKin();
        void calcInvKin();
        Eigen::Quaternion<double> QuaternionFromEuler(const Eigen::Vector3d &eulerXYZ, bool ZYX);


};

#endif // LAPRASCOPICTOOL_H
