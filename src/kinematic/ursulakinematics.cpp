#include "masterslave/kinematic/ursulakinematics.h"

#define DEG_TO_RAD M_PI/180

UrsulaKinematics::UrsulaKinematics(const Eigen::Affine3d startPositionLBR)
{
    setRCM(startPositionLBR);
    ROS_WARN_STREAM_NAMED("Remote Center of Motion","Position: " << RCM.translation());
    isDirKinCalced = false;
    //in according to LBR Specifications (in Degree)
    const double MAX_ANGLES[] = {170/180*M_PI, 120/180*M_PI, 170/180*M_PI, 120/180*M_PI, 170/180*M_PI, 120/180*M_PI, 175/180*M_PI, 85/180*M_PI, 90/180*M_PI, 90/180*M_PI};
    const double MAX_ANGLES_SPEED[] = { 85/180*M_PI, 85/180*M_PI, 100/180*M_PI, 75/180*M_PI, 130/180*M_PI, 135/180*M_PI, 135/180*M_PI, 45/180*M_PI, 45/180*M_PI, 45/180*M_PI};
    URSULA_MAX_ANGLES = Eigen::Matrix<double,10,1>(MAX_ANGLES);

    URSULA_MAX_ANGLES_SPEED = Eigen::Matrix<double,10,1>(MAX_ANGLES_SPEED);
    //in according to LBR Specifications (in Degree per Second)
}

void UrsulaKinematics::setT_0_EE(Eigen::Affine3d value)
{
    T_0_EE = value;
    desEEPosition(0) = T_0_EE.translation().x();
    desEEPosition(1) = T_0_EE.translation().y();
    desEEPosition(2) = T_0_EE.translation().z();
    desEEPosition(3) =  atan2(T_0_EE.matrix()(2,1),T_0_EE.matrix()(2,2)); //alpha
    desEEPosition(5) = atan2(T_0_EE.matrix()(1,0),T_0_EE.matrix()(0,0)); //gamma
    desEEPosition(4) = atan2(-T_0_EE.matrix()(2,1),T_0_EE.matrix()(0,0)*cos(curEEPosition(5))+T_0_EE.matrix()(1,0)*sin(curEEPosition(5))); //beta
    calcInvKin();
}

//direkte Kinematik vielleicht als Funktion anstatt als Methode
Eigen::VectorXd UrsulaKinematics::calcDirKin(Eigen::VectorXd jointAngles)
{
    //TODO: In DH-Form bringen,

    // LBR Kinematik
    T_0_Q1 = buildAffine3d(Eigen::Vector3d(0,0,LBR_PARAMETERS.L_0_Q1),Eigen::Vector3d(0,0,jointAngles(0)),true);
    Eigen::Affine3d T_Q1_Q2 = buildAffine3d(Eigen::Vector3d(0,0,LBR_PARAMETERS.L_Q1_Q2),Eigen::Vector3d(0,jointAngles(1),0),true);
    Eigen::Affine3d T_Q2_Q3 = buildAffine3d(Eigen::Vector3d(0,0,LBR_PARAMETERS.L_Q2_Q3),Eigen::Vector3d(0,0,jointAngles(2)),true);
    Eigen::Affine3d T_Q3_Q4 = buildAffine3d(Eigen::Vector3d(0,0,LBR_PARAMETERS.L_Q3_Q4),Eigen::Vector3d(0,jointAngles(3),0),true);
    Eigen::Affine3d T_Q4_Q5 = buildAffine3d(Eigen::Vector3d(0,0,LBR_PARAMETERS.L_Q4_Q5),Eigen::Vector3d(0,0,jointAngles(4)),true);
    Eigen::Affine3d T_Q5_Q6 = buildAffine3d(Eigen::Vector3d(0,0,LBR_PARAMETERS.L_Q5_Q6),Eigen::Vector3d(0,jointAngles(5),0),true);
    Eigen::Affine3d T_Q6_Q7 = buildAffine3d(Eigen::Vector3d(0,0,LBR_PARAMETERS.L_Q6_Q7),Eigen::Vector3d(0,0,jointAngles(6)),true);
    Eigen::Affine3d T_Q7_FL = buildAffine3d(Eigen::Vector3d(0,0,LBR_PARAMETERS.L_Q7_FL),Eigen::Vector3d(0,0,0),true);
    Eigen::Affine3d T_FL_Q8 = buildAffine3d(Eigen::Vector3d(TOOL_PARAMETERS.X_0_Q4,TOOL_PARAMETERS.Y_0_Q4,TOOL_PARAMETERS.Z_0_Q4),Eigen::Vector3d(TOOL_PARAMETERS.A_0_Q4*DEG_TO_RAD,TOOL_PARAMETERS.B_0_Q4*DEG_TO_RAD,TOOL_PARAMETERS.C_0_Q4*DEG_TO_RAD),true);
    T_FL_Q8.rotate(Eigen::AngleAxis<double>(jointAngles(7),Eigen::Vector3d::UnitZ()));
    Eigen::Affine3d T_Q8_Q9 = buildAffine3d(Eigen::Vector3d::Zero(),Eigen::Vector3d(90*DEG_TO_RAD,0,90*DEG_TO_RAD+jointAngles(8)),false);
    /* Fehler: Zuerst um Q5 drehen, dann verschieben */
    Eigen::Affine3d T_Q9_Q10 = buildAffine3d(Eigen::Vector3d(TOOL_PARAMETERS.L_Q5_Q6,0,0),Eigen::Vector3d(-90*DEG_TO_RAD,0,0),true);
    Eigen::Affine3d T_10_EE = buildAffine3d(Eigen::Vector3d::Zero(),Eigen::Vector3d(0,0,jointAngles(9)),true);
    T_10_EE.translate(Eigen::Vector3d(TOOL_PARAMETERS.L_Q6_EE,0,0));

    // important for jacobi matrix
    T_0_Q2 = T_0_Q1*T_Q1_Q2;
    T_0_Q3 = T_0_Q2*T_Q2_Q3;
    T_0_Q4 = T_0_Q3*T_Q3_Q4;
    T_0_Q5 = T_0_Q4*T_Q4_Q5;
    T_0_Q6 = T_0_Q5*T_Q5_Q6;
    T_0_Q7 = T_0_Q6*T_Q6_Q7;
    T_0_FL = T_0_Q7*T_Q7_FL;
    T_0_Q8 = T_0_FL*T_FL_Q8;
    T_0_Q9 = T_FL_Q8*T_Q8_Q9;
    T_0_Q10 = T_0_Q9*T_Q9_Q10;



    T_0_EE = T_0_FL*T_FL_EE;

    curEEPosition(0) = T_0_EE.translation().x();
    curEEPosition(1) = T_0_EE.translation().y();
    curEEPosition(2) = T_0_EE.translation().z();
    curEEPosition(3) =  atan2(T_0_EE.matrix()(2,1),T_0_EE.matrix()(2,2)); //alpha
    curEEPosition(5) = atan2(T_0_EE.matrix()(1,0),T_0_EE.matrix()(0,0)); //gamma
    curEEPosition(4) = atan2(-T_0_EE.matrix()(2,1),T_0_EE.matrix()(0,0)*cos(curEEPosition(5))+T_0_EE.matrix()(1,0)*sin(curEEPosition(5))); // beta

    return curEEPosition;
}

void UrsulaKinematics::calcInvKin()
{
    int iterations = 0;
    double residual = std::numeric_limits<double>::infinity();
    double residualOld = std::numeric_limits<double>::infinity();
    Eigen::VectorXd jointAnglesIterationPrevious(10);

    Eigen::VectorXd dQIteration(10);
    double delta = 0.01;

    //inequality constraints; q > MIN_ANGLES
    Eigen::MatrixXd Aieq = Eigen::MatrixXd::Identity(10,10);
    Eigen::VectorXd bieq = Eigen::VectorXd(URSULA_MAX_ANGLES);
    jointAnglesIterationPrevious = jointAngles;
    do
    {
        iterations++;
        if(iterations>=maxIterations) break;
        //equality constraints
        Eigen::MatrixXd Aeq = calcAnalyticalJacobian(jointAnglesIterationPrevious);
        Eigen::VectorXd beq = -calcDirKin(jointAnglesIterationPrevious) - calcAnalyticalJacobian(jointAnglesIterationPrevious)*(jointAngles-jointAnglesIterationPrevious) + desEEPosition;

        // potential for max angles H(0,:) = H; H(1,:)= dH/dq
        Eigen::MatrixXd H_ang = angleMonitoring(jointAnglesIterationPrevious,10000);
        Eigen::MatrixXd C_ang = H_ang.row(1).asDiagonal();
        Eigen::VectorXd d_ang = -delta*(H_ang.row(0) * C_ang*(jointAngles-jointAnglesIterationPrevious));

        // https://forum.kde.org/viewtopic.php?f=74&t=102468
        Eigen::MatrixXd cTc = C_ang.transpose()*C_ang;
        Eigen::VectorXd cTd = -C_ang.transpose()*d_ang;
        residual = Eigen::solve_quadprog(cTc,cTd,Aeq,beq,Aieq,bieq,dQIteration,1);
        jointAnglesIterationPrevious+= dQIteration;
        if(residual == std::numeric_limits<double>::infinity()) break;
    }
    while(residualOld<=residual);
    jointAngles = jointAnglesIterationPrevious;

}

Eigen::MatrixXd UrsulaKinematics::calcAnalyticalJacobian(Eigen::VectorXd jointAngles)
{
    calcDirKin(jointAngles);
    //z-Axis of each joint
    Eigen::Vector3d z_Q1 = T_0_Q1.matrix().col(3).head(3);
    Eigen::Vector3d z_Q2 = T_0_Q2.matrix().col(3).head(3);
    Eigen::Vector3d z_Q3 = T_0_Q3.matrix().col(3).head(3);
    Eigen::Vector3d z_Q4 = T_0_Q4.matrix().col(3).head(3);
    Eigen::Vector3d z_Q5 = T_0_Q5.matrix().col(3).head(3);
    Eigen::Vector3d z_Q6 = T_0_Q6.matrix().col(3).head(3);
    Eigen::Vector3d z_Q7 = T_0_Q7.matrix().col(3).head(3);
    Eigen::Vector3d z_Q8 = T_0_Q8.matrix().col(3).head(3);
    Eigen::Vector3d z_Q9 = T_0_Q9.matrix().col(3).head(3);
    Eigen::Vector3d z_Q10 = T_0_Q10.matrix().col(3).head(3);

    Eigen::Vector3d x_1_EE = T_0_EE.translation() - T_0_Q1.translation();
    Eigen::Vector3d x_2_EE = T_0_EE.translation() - T_0_Q2.translation();
    Eigen::Vector3d x_3_EE = T_0_EE.translation() - T_0_Q3.translation();
    Eigen::Vector3d x_4_EE = T_0_EE.translation() - T_0_Q4.translation();
    Eigen::Vector3d x_5_EE = T_0_EE.translation() - T_0_Q5.translation();
    Eigen::Vector3d x_6_EE = T_0_EE.translation() - T_0_Q6.translation();
    Eigen::Vector3d x_7_EE = T_0_EE.translation() - T_0_Q7.translation();
    Eigen::Vector3d x_8_EE = T_0_EE.translation() - T_0_Q8.translation();
    Eigen::Vector3d x_9_EE = T_0_EE.translation() - T_0_Q9.translation();
    Eigen::Vector3d x_10_EE = T_0_EE.translation() - T_0_Q10.translation();

    Eigen::Vector3d zr1 = z_Q1.cross(x_1_EE);
    Eigen::Vector3d zr2 = z_Q2.cross(x_2_EE);
    Eigen::Vector3d zr3 = z_Q3.cross(x_3_EE);
    Eigen::Vector3d zr4 = z_Q4.cross(x_4_EE);
    Eigen::Vector3d zr5 = z_Q5.cross(x_5_EE);
    Eigen::Vector3d zr6 = z_Q6.cross(x_6_EE);
    Eigen::Vector3d zr7 = z_Q7.cross(x_7_EE);
    Eigen::Vector3d zr8 = z_Q8.cross(x_8_EE);
    Eigen::Vector3d zr9 = z_Q9.cross(x_9_EE);
    Eigen::Vector3d zr10 = z_Q10.cross(x_10_EE);

    geomJacobian(6,10);
    geomJacobian.row(0) << zr1, z_Q1;
    geomJacobian.row(1) << zr2, z_Q2;
    geomJacobian.row(2) << zr3, z_Q3;
    geomJacobian.row(3) << zr4, z_Q4;
    geomJacobian.row(4) << zr5, z_Q5;
    geomJacobian.row(5) << zr6, z_Q6;
    geomJacobian.row(6) << zr7, z_Q7;
    geomJacobian.row(7) << zr8, z_Q8;
    geomJacobian.row(8) << zr9, z_Q9;
    geomJacobian.row(9) << zr10, z_Q10;

    Eigen::MatrixXd omega(6,6);

    omega.topLeftCorner(3,3) = Eigen::MatrixXd::Identity(3,3);
    omega.topRightCorner(3,3) = Eigen::MatrixXd::Zero(3,3);
    omega.bottomLeftCorner(3,3) = Eigen::MatrixXd::Zero(3,3);

    //Transformation from geometrical Jacobian to analytical Jacobian
    double alpha = atan2(T_0_EE.matrix()(2,1),T_0_EE.matrix()(2,2));
    double gamma = atan2(T_0_EE.matrix()(1,0),T_0_EE.matrix()(0,0));
    double beta = atan2(-T_0_EE.matrix()(2,0),T_0_EE.matrix()(0,0)*cos(gamma)+T_0_EE.matrix()(1,0)*sin(gamma));

    Eigen::Matrix3d omegaKard;
    omegaKard.row(0) << cos(beta)*cos(gamma), -sin(gamma), 0;
    omegaKard.row(1) << sin(gamma)*cos(beta), cos(gamma), 0;
    omegaKard.row(2) << -sin(beta), 0, 1;

    omega.bottomRightCorner(3,3) = omegaKard;

    analyticalJacobian(6,10);
    return analyticalJacobian = omega.inverse()*geomJacobian;
}

//Potential in according to Schreiber (p. 63f)
Eigen::MatrixXd UrsulaKinematics::angleMonitoring(Eigen::VectorXd q, double Hmax)
{
    // percentage when the angle gets critical
    const double critical = 0.85;
    //combined matrix of H and dH (dH/dq)
    Eigen::MatrixXd H(q.rows(),2);
    for(int i=0;i<q.rows();i++)
    {
        if(-URSULA_MAX_ANGLES(i,0)>q(i))
        {
            //Outside the permissible zone
            H(i,0) = Hmax;
            H(i,1) = 0; // Differentiation: dH
        }

        else if(-URSULA_MAX_ANGLES(i)<=q(i) && critical*-URSULA_MAX_ANGLES(i) >= q(i))
        {
            //Critical Zone
            H(i,0) = Hmax/2*(1-cos(M_PI*(q(i)+critical*URSULA_MAX_ANGLES(i))/(1-critical)*URSULA_MAX_ANGLES(i)));
            H(i,1) = M_PI*Hmax/(2*((1-critical)*URSULA_MAX_ANGLES(i))*sin(M_PI*(q(i)+critical*URSULA_MAX_ANGLES(i))/(1-critical)*URSULA_MAX_ANGLES(i)));
        }

        else if(critical*-URSULA_MAX_ANGLES(i) < q(i) && q(i) < critical*URSULA_MAX_ANGLES(i))
        {
            //ordinary Zone
            H(i,0) = 0;
            H(i,1) = 0;
        }

        else if(critical*URSULA_MAX_ANGLES(i) <= q(i) && URSULA_MAX_ANGLES(i) >= q(i))
        {
            H(i,0) = Hmax/2*(1-cos(M_PI*(q(i)+critical*URSULA_MAX_ANGLES(i))/(1-critical)*URSULA_MAX_ANGLES(i)));
            H(i,1) = M_PI*Hmax/(2*((1-critical)*URSULA_MAX_ANGLES(i))*sin(M_PI*(q(i)+critical*URSULA_MAX_ANGLES(i))/(1-critical)*URSULA_MAX_ANGLES(i)));
        }
        else if(URSULA_MAX_ANGLES(i)< q(i))
        {
            H(i,0) = Hmax;
            H(i,1) = 0;
        }
    }
    return H;
}

Eigen::MatrixXd UrsulaKinematics::trocarMonitoring(Eigen::VectorXd q)
{

}




const lbrDescriptionParameters UrsulaKinematics::LBR_PARAMETERS = { 160, 200, 200, 220, 180, 220, 76, 50};

