#include "masterslave/kinematic/ursulakinematics.h"

UrsulaKinematics::UrsulaKinematics(Eigen::Affine3d startPosLBR)
{
    desEEPosition = Eigen::VectorXd::Zero(6);
    curEEPosition = Eigen::VectorXd::Zero(6);
    jointAnglesAct = Eigen::VectorXd::Zero(10);
    jointAnglesTar = Eigen::VectorXd::Zero(10);
    setRCM(startPosLBR);
    ROS_WARN_STREAM_NAMED("Remote Center of Motion","Position: " << RCM.translation());
    isDirKinCalced = false;
    //in according to LBR Specifications (in Degree)
    const double MAX_ANGLES[] = {170*DEG_TO_RAD, 120*DEG_TO_RAD, 170*DEG_TO_RAD, 120*DEG_TO_RAD, 170*DEG_TO_RAD, 120*DEG_TO_RAD, 175*DEG_TO_RAD, 85*DEG_TO_RAD, 90*DEG_TO_RAD, 90*DEG_TO_RAD};
    const double MAX_ANGLES_SPEED[] = { 85*DEG_TO_RAD, 85*DEG_TO_RAD, 100*DEG_TO_RAD, 75*DEG_TO_RAD, 130*DEG_TO_RAD, 135*DEG_TO_RAD, 135*DEG_TO_RAD, 45*DEG_TO_RAD, 45*DEG_TO_RAD, 45*DEG_TO_RAD};
    URSULA_MAX_ANGLES = Eigen::Matrix<double,10,1>(MAX_ANGLES);
    URSULA_MAX_ANGLES_SPEED = Eigen::Matrix<double,10,1>(MAX_ANGLES_SPEED);

    //in according to LBR Specifications (in Degree per Second)
}

void UrsulaKinematics::setAngles(const Eigen::VectorXd value)
{
    jointAnglesAct = value;
}

void UrsulaKinematics::setToolAngles(const Eigen::VectorXd value)
{
    jointAnglesAct.tail(3) = value;
}


void UrsulaKinematics::setT_0_EE(Eigen::Affine3d value)
{
    T_0_EE = value;

    desEEPosition = rotation2RPY(T_0_EE);
    calcInvKin();
}

Eigen::Affine3d UrsulaKinematics::calcStartPos(Eigen::Affine3d T_0_FL, Eigen::VectorXd toolAngles)
{
    Eigen::Affine3d T_FL_Q4 = buildAffine3d(Eigen::Vector3d(TOOL_PARAMETERS.X_0_Q4,TOOL_PARAMETERS.Y_0_Q4,TOOL_PARAMETERS.Z_0_Q4),Eigen::Vector3d(TOOL_PARAMETERS.A_0_Q4*DEG_TO_RAD,TOOL_PARAMETERS.B_0_Q4*DEG_TO_RAD,TOOL_PARAMETERS.C_0_Q4*DEG_TO_RAD),true);
    T_FL_Q4.rotate(Eigen::AngleAxis<double>(toolAngles(0),Eigen::Vector3d::UnitZ()));
    Eigen::Affine3d T_Q4_Q5 = buildAffine3d(Eigen::Vector3d::Zero(),Eigen::Vector3d(90*DEG_TO_RAD,0,90*DEG_TO_RAD+toolAngles(1)),false);
    /* Fehler: Zuerst um Q5 drehen, dann verschieben */
    Eigen::Affine3d T_Q5_Q6 = buildAffine3d(Eigen::Vector3d(TOOL_PARAMETERS.L_Q5_Q6,0,0),Eigen::Vector3d(-90*DEG_TO_RAD,0,0),true);
    Eigen::Affine3d T_Q6_EE = buildAffine3d(Eigen::Vector3d::Zero(),Eigen::Vector3d(0,0,toolAngles(2)),true);
    T_Q6_EE.translate(Eigen::Vector3d(TOOL_PARAMETERS.L_Q6_EE,0,0));
    T_0_EE = T_0_FL*T_FL_Q4*T_Q4_Q5*T_Q5_Q6*T_Q6_EE;
    return T_0_EE;
}

//direkte Kinematik vielleicht als Funktion anstatt als Methode
Eigen::VectorXd UrsulaKinematics::calcDirKin(Eigen::VectorXd jointAngles)
{
    //TODO: In DH-Form bringen,
    // LBR Kinematik
    T_0_Q1 = buildAffine3d(Eigen::Vector3d(0,0,LBR_PARAMETERS.L_0_Q1+LBR_PARAMETERS.L_Q1_Q2),Eigen::Vector3d(0,0,M_PI),true);
    Eigen::Affine3d T_Q1_Q2 = buildAffine3d(Eigen::Vector3d(0,0,0),Eigen::Vector3d(M_PI/2,0,jointAngles(0)),true);
    Eigen::Affine3d T_Q2_Q3 = buildAffine3d(Eigen::Vector3d(0,0,0),Eigen::Vector3d(-M_PI/2,0,jointAngles(1)),true);
    Eigen::Affine3d T_Q3_Q4 = buildAffine3d(Eigen::Vector3d(0,0,LBR_PARAMETERS.L_Q2_Q3+LBR_PARAMETERS.L_Q3_Q4),Eigen::Vector3d(-M_PI/2,0,jointAngles(2)),true);
    Eigen::Affine3d T_Q4_Q5 = buildAffine3d(Eigen::Vector3d(0,0,0),Eigen::Vector3d(M_PI/2,0,jointAngles(3)),true);
    Eigen::Affine3d T_Q5_Q6 = buildAffine3d(Eigen::Vector3d(0,0,LBR_PARAMETERS.L_Q4_Q5+LBR_PARAMETERS.L_Q5_Q6),Eigen::Vector3d(M_PI/2,0,jointAngles(4)),true);
    Eigen::Affine3d T_Q6_Q7 = buildAffine3d(Eigen::Vector3d(0,0,0),Eigen::Vector3d(-M_PI/2,0,jointAngles(5)),true);
    Eigen::Affine3d T_Q7_FL = buildAffine3d(Eigen::Vector3d(0,0,LBR_PARAMETERS.L_Q6_Q7+LBR_PARAMETERS.L_Q7_FL),Eigen::Vector3d(0,0,M_PI-22.5*DEG_TO_RAD+jointAngles(6)),true); //Hotfix: 22.5°

    Eigen::Affine3d T_FL_Q8 = buildAffine3d(Eigen::Vector3d(TOOL_PARAMETERS.X_0_Q4,TOOL_PARAMETERS.Y_0_Q4,TOOL_PARAMETERS.Z_0_Q4),Eigen::Vector3d(TOOL_PARAMETERS.A_0_Q4*DEG_TO_RAD,TOOL_PARAMETERS.B_0_Q4*DEG_TO_RAD,TOOL_PARAMETERS.C_0_Q4*DEG_TO_RAD),true);
    //ROS_INFO_STREAM("T_FL_Q8: \n" << T_FL_Q8.matrix());
    T_FL_Q8.rotate(Eigen::AngleAxis<double>(jointAngles(7),Eigen::Vector3d::UnitZ()));
    Eigen::Affine3d T_Q8_Q9 = buildAffine3d(Eigen::Vector3d::Zero(),Eigen::Vector3d(90*DEG_TO_RAD,0,90*DEG_TO_RAD+jointAngles(8)),false);
    // Fehler: Zuerst um Q5 drehen, dann verschieben
    Eigen::Affine3d T_Q9_Q10 = buildAffine3d(Eigen::Vector3d(TOOL_PARAMETERS.L_Q5_Q6,0,0),Eigen::Vector3d(-90*DEG_TO_RAD,0,0),true);
    Eigen::Affine3d T_Q10_EE = buildAffine3d(Eigen::Vector3d::Zero(),Eigen::Vector3d(0,0,jointAngles(9)),true);
    T_Q10_EE.translate(Eigen::Vector3d(TOOL_PARAMETERS.L_Q6_EE,0,0));

    // important for jacobi matrix
    T_0_Q2 = T_0_Q1*T_Q1_Q2;
    T_0_Q3 = T_0_Q2*T_Q2_Q3;
    T_0_Q4 = T_0_Q3*T_Q3_Q4;
    T_0_Q5 = T_0_Q4*T_Q4_Q5;
    T_0_Q6 = T_0_Q5*T_Q5_Q6;
    T_0_Q7 = T_0_Q6*T_Q6_Q7;
    T_0_FL = T_0_Q7*T_Q7_FL;
    //ROS_INFO_STREAM("T_0_FL: " << T_0_FL.matrix());
    T_0_Q8 = T_0_FL*T_FL_Q8;
    T_0_Q9 = T_0_Q8*T_Q8_Q9;
    T_0_Q10 = T_0_Q9*T_Q9_Q10;


    T_FL_EE = T_FL_Q8*T_Q8_Q9*T_Q9_Q10*T_Q10_EE;
    //ROS_INFO_STREAM("T_FL_EE: \n" << T_FL_EE.matrix());
    T_0_EE = T_0_Q10*T_Q10_EE;

   // ROS_INFO_STREAM("T_0_EE" << T_0_EE.matrix());


    curEEPosition = rotation2RPY(T_0_EE);
    ROS_INFO_STREAM("current Position: \n" << curEEPosition);
    return curEEPosition;
}

void UrsulaKinematics::calcInvKin()
{
    int iterations = 0;
    double residual = std::numeric_limits<double>::infinity();
    double residualOld = std::numeric_limits<double>::infinity();
    Eigen::VectorXd jointAnglesIterationPrevious = Eigen::VectorXd::Zero(10);

    Eigen::VectorXd dQIteration = Eigen::VectorXd::Zero(10);
    double delta = 1;

    //inequality constraints; q > MIN_ANGLES
    Eigen::MatrixXd Aieq = Eigen::MatrixXd::Identity(10,20);

    Aieq.rightCols(10) = Eigen::MatrixXd::Identity(10,10);


    Eigen::VectorXd bieq = Eigen::VectorXd(20);
    bieq.head(10) = URSULA_MAX_ANGLES_SPEED*0.01;
    bieq.tail(10) = URSULA_MAX_ANGLES_SPEED*0.01;
    jointAnglesIterationPrevious = jointAnglesAct;

    while(true)
    {
        iterations++;
        residualOld = residual;
        if(iterations>=maxIterations) break;
        //equality constraints
        Eigen::MatrixXd Aeq = calcAnalyticalJacobian(jointAnglesIterationPrevious);
        Eigen::MatrixXd AeqT = Aeq.transpose();
        Eigen::VectorXd beq = -calcDirKin(jointAnglesIterationPrevious) - calcAnalyticalJacobian(jointAnglesIterationPrevious)*(jointAnglesAct-jointAnglesIterationPrevious) + desEEPosition;
        //ROS_INFO_STREAM("desiredPosition" << calcDirKin(jointAnglesIterationPrevious) << "\n actAngle: " << jointAnglesIterationPrevious);
        ROS_INFO_STREAM("Aeq: \n" << Aeq << " beq: " << beq);
        // potential for max angles H(0,:) = H; H(1,:)= dH/dq
        Eigen::MatrixXd H_ang = angleMonitoring(jointAnglesIterationPrevious,5000);
        Eigen::MatrixXd C_ang = H_ang.col(1).asDiagonal();

        Eigen::VectorXd d_ang = -delta*(H_ang.col(0)+C_ang*(jointAnglesAct-jointAnglesIterationPrevious));

        // https://forum.kde.org/viewtopic.php?f=74&t=102468
        Eigen::MatrixXd cTc = C_ang.transpose()*C_ang;
        Eigen::VectorXd cTd = -C_ang.transpose()*d_ang;
        //ROS_INFO_STREAM("cTc: " << cTc << " cTd: " << cTd << " Aieq: " << Aieq << " bieq: " << bieq << " x: " << dQIteration);
        residual = std::abs(Eigen::solve_quadprog(cTc,cTd,AeqT,beq,Aieq,bieq,dQIteration,1));
        if(residual>residualOld || residual == std::numeric_limits<double>::infinity()) break;
        jointAnglesIterationPrevious += dQIteration;
    }

    ROS_INFO_STREAM("residual: \n" << residual << "\n ResidualOld: " << residualOld);
    ROS_INFO_STREAM("iterations: " << iterations);
    jointAnglesTar = jointAnglesIterationPrevious;
    ROS_INFO_STREAM("jointAnglesTar: \n" << jointAnglesTar);
}

Eigen::MatrixXd UrsulaKinematics::calcAnalyticalJacobian(Eigen::VectorXd jointAngles)
{
    calcDirKin(jointAngles);
    //z-Axis of each joint
    Eigen::Vector3d z_Q1 = T_0_Q1.matrix().col(2).head(3);
    Eigen::Vector3d z_Q2 = T_0_Q2.matrix().col(2).head(3);
    Eigen::Vector3d z_Q3 = T_0_Q3.matrix().col(2).head(3);
    Eigen::Vector3d z_Q4 = T_0_Q4.matrix().col(2).head(3);
    Eigen::Vector3d z_Q5 = T_0_Q5.matrix().col(2).head(3);
    Eigen::Vector3d z_Q6 = T_0_Q6.matrix().col(2).head(3);
    Eigen::Vector3d z_Q7 = T_0_Q7.matrix().col(2).head(3);
    Eigen::Vector3d z_Q8 = T_0_Q8.matrix().col(2).head(3);
    Eigen::Vector3d z_Q9 = T_0_Q9.matrix().col(2).head(3);
    Eigen::Vector3d z_Q10 = T_0_Q10.matrix().col(2).head(3);

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

    geomJacobian = Eigen::MatrixXd::Zero(6,10);
    geomJacobian.col(0) << zr1, z_Q1;
    geomJacobian.col(1) << zr2, z_Q2;
    geomJacobian.col(2) << zr3, z_Q3;
    geomJacobian.col(3) << zr4, z_Q4;
    geomJacobian.col(4) << zr5, z_Q5;
    geomJacobian.col(5) << zr6, z_Q6;
    geomJacobian.col(6) << zr7, z_Q7;
    geomJacobian.col(7) << zr8, z_Q8;
    geomJacobian.col(8) << zr9, z_Q9;
    geomJacobian.col(9) << zr10, z_Q10;

    ROS_DEBUG_STREAM("geomJacobi: " << geomJacobian);

    Eigen::MatrixXd omega = Eigen::MatrixXd::Zero(6,6);

    omega.topLeftCorner(3,3) = Eigen::MatrixXd::Identity(3,3);
    omega.topRightCorner(3,3) = Eigen::MatrixXd::Zero(3,3);
    omega.bottomLeftCorner(3,3) = Eigen::MatrixXd::Zero(3,3);

    //Transformation from geometrical Jacobian to analytical Jacobian
    Eigen::VectorXd r_EE = rotation2RPY(T_0_EE);
    double alpha = r_EE.tail(3)(0);
    double gamma = r_EE.tail(3)(2);
    double beta = r_EE.tail(3)(1);

    Eigen::Matrix3d omegaKard;
    omegaKard.row(0) << cos(beta)*cos(gamma), -sin(gamma), 0;
    omegaKard.row(1) << sin(gamma)*cos(beta), cos(gamma), 0;
    omegaKard.row(2) << -sin(beta), 0, 1;

    omega.bottomRightCorner(3,3) = omegaKard;

    analyticalJacobian = Eigen::MatrixXd::Zero(6,10);
    analyticalJacobian = omega.inverse()*geomJacobian;
    return analyticalJacobian;
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
            H(i,0) = -Hmax;
            H(i,1) = -0.00001; // Differentiation: dH
        }

        else if(-URSULA_MAX_ANGLES(i)<=q(i) && critical*-URSULA_MAX_ANGLES(i) >= q(i))
        {
            //Critical Zone
            H(i,0) = -Hmax/2*(1-cos(M_PI*(q(i)+critical*URSULA_MAX_ANGLES(i))/(1-critical)*URSULA_MAX_ANGLES(i)));
            H(i,1) = -M_PI*Hmax/(2*((1-critical)*URSULA_MAX_ANGLES(i)))*sin(M_PI*(q(i)+critical*URSULA_MAX_ANGLES(i))/(1-critical)*URSULA_MAX_ANGLES(i));
        }

        else if(critical*-URSULA_MAX_ANGLES(i) < q(i) && q(i) < critical*URSULA_MAX_ANGLES(i))
        {
            //ordinary Zone
            H(i,0) = 0.00001;
            H(i,1) = 0.00001;
        }

        else if(critical*URSULA_MAX_ANGLES(i) <= q(i) && URSULA_MAX_ANGLES(i) >= q(i))
        {
            H(i,0) = Hmax/2*(1-cos(M_PI*(q(i)-critical*URSULA_MAX_ANGLES(i))/(1-critical)*URSULA_MAX_ANGLES(i)));
            H(i,1) = M_PI*Hmax/(2*((1-critical)*URSULA_MAX_ANGLES(i)))*sin(M_PI*(q(i)-critical*URSULA_MAX_ANGLES(i))/(1-critical)*URSULA_MAX_ANGLES(i));
        }
        else if(URSULA_MAX_ANGLES(i)< q(i))
        {
            H(i,0) = Hmax;
            H(i,1) = 0.00001;
        }
    }
    ROS_WARN_STREAM("H: " << H);
    return H;
}

Eigen::MatrixXd UrsulaKinematics::trocarMonitoring(Eigen::VectorXd q)
{

}

Eigen::VectorXd UrsulaKinematics::rotation2RPY(Eigen::Affine3d transformation)
{
    Eigen::VectorXd retVal = Eigen::VectorXd::Zero(6);

    retVal.head(3) = transformation.translation();
    if(transformation.matrix()(0,0) > 0.03 || transformation.matrix()(1,0) > 0.03)
    {
        retVal(5) = atan2(transformation.matrix()(1,0),transformation.matrix()(0,0));
    }
    retVal(4) = atan2(-transformation.matrix()(2,0),cos(retVal(5))*transformation.matrix()(0,0)+sin(retVal(5))*transformation.matrix()(1,0));
    retVal(3) = atan2(sin(retVal(5))*transformation.matrix()(0,2)-cos(retVal(5))*transformation.matrix()(1,2),cos(retVal(5))*transformation.matrix()(1,1)-sin(retVal(5))*transformation.matrix()(0,1));
    return retVal;
}



const lbrDescriptionParameters UrsulaKinematics::LBR_PARAMETERS = { 0.160, 0.200, 0.200, 0.220, 0.180, 0.220, 0.076, 0.050};

