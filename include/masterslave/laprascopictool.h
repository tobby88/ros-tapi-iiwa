#ifndef LAPRASCOPICTOOL_H
#define LAPRASCOPICTOOL_H

#include <Eigen/Geometry>
#include <vector>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>

class LaprascopicTool {
    public:
        LaprascopicTool(const Eigen::Affine3d startPoseLBR);
        void buildDebugFrameFromTM(const Eigen::Affine3d &T_0_XX, const std::string &name);
        Eigen::Affine3d buildAffine3d(const Eigen::Vector3d &translXYZ, const Eigen::Vector3d &axisZYX, bool zyx);
        void setAngles(std::vector<double>);
        Eigen::Vector3d getRCM(){return RemoteCenterOfMotion;}
        std::vector<double> getAngles();
	void setQ4(double);
	double getQ4(){ return Q4tar;}
        void setQ5(double);
        double getQ5(){ return Q5tar; }
        void setQ6(double);
        double getQ6() { return Q6tar; }
        void setT_0_EE(Eigen::Affine3d);
        Eigen::Affine3d getT_FL_EE(){ return T_FL_EE; }
	Eigen::Affine3d getT_0_FL(){ return T_0_FL; }

    private:
        double Q4act;
        double Q5act;
        double Q6act;

	double Q4tar;
	double Q5tar;
	double Q6tar;
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
