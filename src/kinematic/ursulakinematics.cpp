#include "masterslave/kinematic/ursulakinematics.h"

UrsulaKinematics::UrsulaKinematics(ros::NodeHandle& nh): nh_(nh)
{
    desEEPosition = Eigen::VectorXd::Zero(6);
    curEEPosition = Eigen::VectorXd::Zero(6);
    jointAnglesAct = Eigen::VectorXd::Zero(10);
    jointAnglesTar = Eigen::VectorXd::Zero(10);

    //in according to LBR Specifications (in Degree)
    const double MAX_ANGLES[] = {170*DEG_TO_RAD, 120*DEG_TO_RAD, 170*DEG_TO_RAD, 120*DEG_TO_RAD, 170*DEG_TO_RAD, 120*DEG_TO_RAD, 175*DEG_TO_RAD, 85*DEG_TO_RAD, 90*DEG_TO_RAD, 90*DEG_TO_RAD};
    const double MAX_ANGLES_SPEED[] = { 85*DEG_TO_RAD, 85*DEG_TO_RAD, 100*DEG_TO_RAD, 75*DEG_TO_RAD, 130*DEG_TO_RAD, 135*DEG_TO_RAD, 135*DEG_TO_RAD, 45*DEG_TO_RAD, 45*DEG_TO_RAD, 45*DEG_TO_RAD};
    URSULA_MAX_ANGLES = Eigen::Matrix<double,10,1>(MAX_ANGLES);
    URSULA_MAX_ANGLES_SPEED = Eigen::Matrix<double,10,1>(MAX_ANGLES_SPEED);

    rcmServiceServer = nh_.advertiseService("/RCM",&UrsulaKinematics::rcmCallback,this);
    directKinematicsServer = nh_.advertiseService("/directKinematics",&UrsulaKinematics::directKinematicsCallback,this);
    inverseKinematicsServer = nh_.advertiseService("/inverseKinematics",&UrsulaKinematics::inverseKinematicsCallback,this);
    cycleTimeSub = nh_.subscribe("/cycleTime",1,&UrsulaKinematics::cycleTimeCallback,this);

}

void UrsulaKinematics::cycleTimeCallback(const std_msgs::Float64ConstPtr &value)
{
    cycleTime = value->data;
}

bool UrsulaKinematics::rcmCallback(masterslave::rcmTest::Request &req, masterslave::rcmTest::Response &resp)
{
    Eigen::VectorXd jointAngles = Eigen::VectorXd::Map(req.trocarAngles.data(),req.trocarAngles.size());
    calcDirKin(jointAngles);
    RCM = T_0_SCH;
}

bool UrsulaKinematics::directKinematicsCallback(masterslave::directKinematics::Request &req, masterslave::directKinematics::Response &resp)
{
    Eigen::VectorXd jointAngles = Eigen::VectorXd::Map(req.jointAngles.data(),req.jointAngles.size());
    calcDirKin(jointAngles);
    jointAnglesAct = jointAngles;
    geometry_msgs::Pose pose;
    tf::poseEigenToMsg(T_0_EE,pose);
    resp.T_0_EE  = pose;
}

bool UrsulaKinematics::inverseKinematicsCallback(masterslave::inverseKinematics::Request &req, masterslave::inverseKinematics::Response &resp)
{
    Eigen::Affine3d desEEPose;
    tf::poseMsgToEigen(req.T_0_EE,desEEPose);
    ROS_DEBUG_STREAM("desEEPose: \n" << desEEPose.matrix());
    calcInvKin(desEEPose);
    std::vector<double> jointAnglesTarget(jointAnglesTar.data(),jointAnglesTar.data()+jointAnglesTar.rows()*jointAnglesTar.cols());

    resp.jointAnglesTarget = jointAnglesTarget;

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

    Eigen::Affine3d T_FL_SCH = buildAffine3d(Eigen::Vector3d(TOOL_PARAMETERS.X_RCM,TOOL_PARAMETERS.Y_0_Q4,TOOL_PARAMETERS.Z_0_Q4),Eigen::Vector3d(TOOL_PARAMETERS.A_0_Q4*DEG_TO_RAD,TOOL_PARAMETERS.B_0_Q4*DEG_TO_RAD,TOOL_PARAMETERS.C_0_Q4*DEG_TO_RAD),true);

    Eigen::Affine3d T_FL_Q8 = buildAffine3d(Eigen::Vector3d(TOOL_PARAMETERS.X_0_Q4,TOOL_PARAMETERS.Y_0_Q4,TOOL_PARAMETERS.Z_0_Q4),Eigen::Vector3d(TOOL_PARAMETERS.A_0_Q4*DEG_TO_RAD,TOOL_PARAMETERS.B_0_Q4*DEG_TO_RAD,TOOL_PARAMETERS.C_0_Q4*DEG_TO_RAD),true);
    //ROS_INFO_STREAM("T_FL_Q8: \n" << T_FL_Q8.matrix());
    T_FL_Q8.rotate(Eigen::AngleAxis<double>(jointAngles(7),Eigen::Vector3d::UnitZ()));
    Eigen::Affine3d T_Q8_Q9 = buildAffine3d(Eigen::Vector3d::Zero(),Eigen::Vector3d(90*DEG_TO_RAD,0,90*DEG_TO_RAD+jointAngles(8)),false);
    // Fehler: Zuerst um Q5 drehen, dann verschieben
    Eigen::Affine3d T_Q9_Q10 = buildAffine3d(Eigen::Vector3d(TOOL_PARAMETERS.L_Q5_Q6,0,0),Eigen::Vector3d(-90*DEG_TO_RAD,0,0),true);
    Eigen::Affine3d T_Q10_EE = buildAffine3d(Eigen::Vector3d::Zero(),Eigen::Vector3d(0,-M_PI/2,jointAngles(9)),true);
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
    T_0_SCH = T_0_FL*T_FL_SCH;
    T_0_Q8 = T_0_FL*T_FL_Q8;
    T_0_Q9 = T_0_Q8*T_Q8_Q9;
    T_0_Q10 = T_0_Q9*T_Q9_Q10;


    T_FL_EE = T_FL_Q8*T_Q8_Q9*T_Q9_Q10*T_Q10_EE;
    ROS_DEBUG_STREAM("T_FL_EE: \n" << T_FL_EE.matrix());
    T_0_EE = T_0_Q10*T_Q10_EE;

    ROS_DEBUG_STREAM("DIR KIN: Joint Angles \n" << jointAngles);


    curEEPosition = rotation2RPY(T_0_EE);
    return curEEPosition;
}

void UrsulaKinematics::calcInvKin(Eigen::Affine3d T_0_EE)
{
    Eigen::VectorXd desiredEEPosition = rotation2RPY(T_0_EE);
    int iterations = 0;
    double residual = std::numeric_limits<double>::infinity();
    double residualNorm = std::numeric_limits<double>::infinity();
    double residualOld = std::numeric_limits<double>::infinity();
    Eigen::VectorXd jointAnglesIterationPrevious = Eigen::VectorXd::Zero(10);

    Eigen::VectorXd weightVector(10);
    weightVector << 1,1,1,1,1,10,1,0.1,1,1;
    Eigen::MatrixXd weightMatrix(10,10);
    weightMatrix = weightVector.asDiagonal();
    Eigen::VectorXd dQIteration = Eigen::VectorXd::Zero(10);
    //weight parameters
    double delta = 1;
    double epsi = 1;
    double zet = 1;

    //inequality constraints; q > MIN_ANGLES_VELOCITY && q < MAX_ANGLES_VELOCITY
    Eigen::MatrixXd Aieq = Eigen::MatrixXd::Identity(10,20);
    Aieq.rightCols(10) = -Eigen::MatrixXd::Identity(10,10);
    Eigen::VectorXd bieq = Eigen::VectorXd(20);
    bieq.head(10) = URSULA_MAX_ANGLES_SPEED*cycleTime*2;
    bieq.tail(10) = URSULA_MAX_ANGLES_SPEED*cycleTime*2;
    jointAnglesIterationPrevious = jointAnglesAct;


    while(residualNorm<=residualOld )
    {
        iterations++;
        residualOld = residualNorm;
        ROS_DEBUG_STREAM("jointAnglesItPrev: \n" << jointAnglesIterationPrevious);
        Eigen::VectorXd deltaQ = jointAnglesAct-jointAnglesIterationPrevious;
        //equality constraints
        Eigen::MatrixXd Akin = calcAnalyticalJacobian(jointAnglesIterationPrevious);
        Eigen::VectorXd bkin = -calcDirKin(jointAnglesIterationPrevious) - calcAnalyticalJacobian(jointAnglesIterationPrevious)*deltaQ + desiredEEPosition;

        // potential for max angles H(0,:) = H; H(1,:)= dH/dq

        Eigen::MatrixXd C_trokar = Eigen::MatrixXd::Zero(2,10);
        Eigen::VectorXd d_trokar = trocarMonitoring(jointAnglesIterationPrevious,deltaQ,C_trokar);

        Eigen::MatrixXd Aeq(C_trokar.cols(),Akin.rows()+C_trokar.rows());
        Aeq <<C_trokar.transpose(), Akin.transpose();
        Eigen::VectorXd beq(bkin.rows()+d_trokar.rows());
        beq << d_trokar, bkin;

        //Aeq = 1/cycleTime*Aeq;
        //beq = 1/cycleTime*beq;
        ROS_INFO_STREAM("Aeq: \n" << Aeq);
        ROS_INFO_STREAM(" beq: " << beq);

        double d_ang = 0;
        Eigen::MatrixXd C_ang = angleMonitoring(deltaQ, jointAnglesIterationPrevious,5000,d_ang);


        Eigen::MatrixXd C_vel = minimizeVelocities(cycleTime, weightMatrix);
        Eigen::VectorXd d_vel = Eigen::VectorXd::Zero(10);

        Eigen::MatrixXd d_acc = Eigen::VectorXd::Zero(10,1);

        Eigen::MatrixXd C_acc = minimizeAcceleration(cycleTime, weightMatrix,deltaQ,d_acc);
        d_acc = epsi*d_acc;
        ROS_DEBUG_STREAM("d_acc" << d_acc);

        Eigen::MatrixXd C(C_acc.rows()+C_vel.rows(),Akin.cols());
        Eigen::VectorXd d(d_acc.rows()+d_vel.rows());



        C << 0.000000001*C_acc, 0.00000000001*C_vel;
        d <<-0.000000001*d_acc,-0.00000000001*d_vel;
        ROS_DEBUG_STREAM("C: " << C << "\n d: " << d);

        // https://forum.kde.org/viewtopic.php?f=74&t=102468
        Eigen::MatrixXd cTc = C.transpose()*C;
        Eigen::VectorXd cTd = -C.transpose()*d;
        ROS_DEBUG_STREAM("cTc: " << cTc << " cTd: " << cTd);
        residual = Eigen::solve_quadprog(cTc,cTd,Aeq,beq,Aieq,bieq,dQIteration,1);
        ROS_WARN_STREAM("deltaQ: \n" << dQIteration);
        residualNorm = std::abs(residual);
        for(int i=0;i<dQIteration.rows();i++)
        {
            if(std::abs(dQIteration(i))>bieq.head(10)(i))
            {
                residualNorm = std::numeric_limits<double>::infinity();
                break;
            }
        }
        if(residualNorm == std::numeric_limits<double>::infinity() || iterations >= maxIterations || std::abs(residualNorm - residualOld) < 1e-10) break;
        jointAnglesIterationPrevious -= dQIteration;


    }
    //ROS_INFO_STREAM("residual: \n" << residual);
    //ROS_INFO_STREAM("iterations: " << iterations);
    jointAnglesTar = jointAnglesIterationPrevious;
    jointAnglesAct = jointAnglesTar;
   // ROS_INFO_STREAM("jointAnglesTar: \n" << jointAnglesTar);
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

    ROS_DEBUG_STREAM("geomJacobi: \n\n" << geomJacobian);

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

    ROS_DEBUG_STREAM("omegaKard: \n" << omegaKard << "T_0_EE: \n" << T_0_EE.matrix());

    omega.bottomRightCorner(3,3) = omegaKard;

    analyticalJacobian = Eigen::MatrixXd::Zero(6,10);
    analyticalJacobian = omega.inverse()*geomJacobian;
    ROS_DEBUG_STREAM("analytical: \n\n" << analyticalJacobian);
    return analyticalJacobian;
}

//Potential in according to Schreiber (p. 63f)
Eigen::VectorXd UrsulaKinematics::angleMonitoring(Eigen::VectorXd deltaQ, Eigen::VectorXd q, double Hmax, double &a)
{
    // percentage when the angle gets critical
    const double critical = 0.85;
    //combined matrix of H and dH (dH/dq)
    Eigen::VectorXd dH(q.rows());
    double H=0;
    for(int i=0;i<q.rows();i++)
    {
        if(-URSULA_MAX_ANGLES(i,0)>q(i))
        {
            //Outside the permissible zone
            H += Hmax;
            dH(i) = 0; // Differentiation: dH
        }

        else if(-URSULA_MAX_ANGLES(i)<=q(i) && critical*-URSULA_MAX_ANGLES(i) >= q(i))
        {
            //Critical Zone
            H += Hmax/2*(1-cos(M_PI*(q(i)+critical*URSULA_MAX_ANGLES(i))/(1-critical)*URSULA_MAX_ANGLES(i)));
            dH(i) = -M_PI*Hmax/(2*((1-critical)*URSULA_MAX_ANGLES(i)))*sin(M_PI*(q(i)+critical*URSULA_MAX_ANGLES(i))/(1-critical)*URSULA_MAX_ANGLES(i));
        }

        else if(critical*-URSULA_MAX_ANGLES(i) < q(i) && q(i) < critical*URSULA_MAX_ANGLES(i))
        {
            //ordinary Zone
            H += 0.0000;
            dH(i) = 0.0000;
        }

        else if(critical*URSULA_MAX_ANGLES(i) <= q(i) && URSULA_MAX_ANGLES(i) >= q(i))
        {
            H += Hmax/2*(1-cos(M_PI*(q(i)-critical*URSULA_MAX_ANGLES(i))/(1-critical)*URSULA_MAX_ANGLES(i)));
            dH(i) = M_PI*Hmax/(2*((1-critical)*URSULA_MAX_ANGLES(i)))*sin(M_PI*(q(i)-critical*URSULA_MAX_ANGLES(i))/(1-critical)*URSULA_MAX_ANGLES(i));
        }
        else if(URSULA_MAX_ANGLES(i)< q(i))
        {
            H += Hmax;
            dH(i) = 0.0000;
        }
    }
    a = H+dH.dot(deltaQ);
    return dH;
}

Eigen::MatrixXd UrsulaKinematics::minimizeVelocities(double cycleTime, Eigen::MatrixXd weightMatrix)
{
    // p. 65 Dissertation Schreiber (5.28, 5.29)
    Eigen::MatrixXd A = 1/cycleTime*weightMatrix;
    return A;

    // a = 0
}

Eigen::MatrixXd UrsulaKinematics::minimizeAcceleration(double cycleTime, Eigen::MatrixXd weightMatrix, Eigen::VectorXd deltaQ, Eigen::MatrixXd& a)
{

    //Attention "a" is a return for the optimization!!!!

    //(p. 65 Schreiber Dissertation 5.31, 5.32)
    Eigen::MatrixXd A = 1/std::pow(cycleTime,2) * weightMatrix;
    // is only valid if cycleTime is constant
    a = 1/std::pow(cycleTime,2)*weightMatrix * deltaQ;
    return A;
}

Eigen::VectorXd UrsulaKinematics::trocarMonitoring(Eigen::VectorXd qAct, Eigen::VectorXd deltaQ, Eigen::MatrixXd& A)
{
    calcDirKin(qAct);

    // Wahl des Koordinatensystems so, dass keine Rotationsfehler auftauchen können
    Eigen::Affine3d trocar = Eigen::Affine3d::Identity();
    // Vermeidung Rotationsfehler, da sich das Trokar-KS mit dem Schaft mitdreht
    trocar.rotate(T_0_SCH.rotation());
    ROS_INFO_STREAM(RCM.translation());
    trocar.translation() = RCM.translation();

    Eigen::Affine3d shaft(T_0_SCH);
    // Vermeidung Translationsfehler in z-Koordinate, da der Schaft virtuell auf der z-Koordinate des Trokars fixiert wird

    Eigen::Vector3d shaftVector = Eigen::Vector3d::Zero(3);
    double beta = (RCM.translation().z()-shaft.translation().z())/T_0_SCH.matrix()(2,2);
    ROS_INFO_STREAM("Distance: " << Eigen::Vector3d(beta*T_0_SCH.matrix().col(2).head(3)));
    shaft.translation() = shaft.translation() + Eigen::Vector3d(beta*T_0_SCH.matrix().col(2).head(3));
    ROS_WARN_STREAM("trocar: \n" << trocar.matrix() << "\n shaft: \n" << shaft.matrix() << "\n shaftVector: \n" << shaftVector );

    // dH/dQ
    Eigen::Vector3d z_Q1 = T_0_Q1.matrix().col(2).head(3);
    Eigen::Vector3d z_Q2 = T_0_Q2.matrix().col(2).head(3);
    Eigen::Vector3d z_Q3 = T_0_Q3.matrix().col(2).head(3);
    Eigen::Vector3d z_Q4 = T_0_Q4.matrix().col(2).head(3);
    Eigen::Vector3d z_Q5 = T_0_Q5.matrix().col(2).head(3);
    Eigen::Vector3d z_Q6 = T_0_Q6.matrix().col(2).head(3);
    Eigen::Vector3d z_Q7 = T_0_Q7.matrix().col(2).head(3);

    Eigen::Vector3d x_1_trocar = (shaft).translation() - (T_0_Q1).translation();
    Eigen::Vector3d x_2_trocar = (shaft).translation() - (T_0_Q2).translation();
    Eigen::Vector3d x_3_trocar = (shaft).translation() - (T_0_Q3).translation();
    Eigen::Vector3d x_4_trocar = (shaft).translation() - (T_0_Q4).translation();
    Eigen::Vector3d x_5_trocar = (shaft).translation() - (T_0_Q5).translation();
    Eigen::Vector3d x_6_trocar = (shaft).translation() - (T_0_Q6).translation();
    Eigen::Vector3d x_7_trocar = (shaft).translation() - (T_0_Q7).translation();

    Eigen::VectorXd shaftPositionRPY = rotation2RPY(shaft);

    Eigen::VectorXd trocarPositionRPY = rotation2RPY(trocar);


    // A = dH/dq
    Eigen::MatrixXd Ages = Eigen::MatrixXd::Zero(6,10);

    Ages.col(0) << z_Q1.cross(x_1_trocar), z_Q1;
    Ages.col(1) << z_Q2.cross(x_2_trocar), z_Q2;
    Ages.col(2) << z_Q3.cross(x_3_trocar), z_Q3;
    Ages.col(3) << z_Q4.cross(x_4_trocar), z_Q4;
    Ages.col(4) << z_Q5.cross(x_5_trocar), z_Q5;
    Ages.col(5) << z_Q6.cross(x_6_trocar), z_Q6;
    Ages.col(6) << z_Q7.cross(x_7_trocar), z_Q7;

    Eigen::VectorXd ages = -(shaftPositionRPY + Ages*deltaQ-trocarPositionRPY); // - 0-Vektor im Trokarsystem da der gewünschte Punkt [0 0 0] im Trokar-KS ist.
    ROS_INFO_STREAM_NAMED("TrocarMonitoring","a:\n" << ages << "\n A: \n" << Ages << "\n shaftPos: \n" << shaftPositionRPY);

    A = Ages.topRows(2);
    Eigen::VectorXd a = ages.topRows(2);
    return a;

}

Eigen::VectorXd UrsulaKinematics::rotation2RPY(Eigen::Affine3d transformation)
{
    Eigen::VectorXd retVal = Eigen::VectorXd::Zero(6);

    retVal.head(3) = transformation.translation();
    retVal(3) = atan2(transformation.matrix()(2,1),transformation.matrix()(2,2));
    retVal(5) = atan2(transformation.matrix()(1,0),transformation.matrix()(0,0));
    retVal(4) = atan2(-transformation.matrix()(2,0),sqrt(pow(transformation.matrix()(0,0),2)+pow(transformation.matrix()(1,0),2)));
    return retVal;
}


Eigen::Affine3d UrsulaKinematics::buildAffine3d(const Eigen::Vector3d &translXYZ, const Eigen::Vector3d &axisZYX, bool zyx=true)
{
    Eigen::Affine3d transl;
    transl.setIdentity();
    transl.translate(translXYZ);
    if(zyx)
    {
        transl.rotate(Eigen::AngleAxis<double>(axisZYX(2), Eigen::Vector3d::UnitZ()));
        transl.rotate(Eigen::AngleAxis<double>(axisZYX(1), Eigen::Vector3d::UnitY()));
        transl.rotate(Eigen::AngleAxis<double>(axisZYX(0), Eigen::Vector3d::UnitX()));
    }
    else
    {
        transl.rotate(Eigen::AngleAxis<double>(axisZYX(0), Eigen::Vector3d::UnitX()));
        transl.rotate(Eigen::AngleAxis<double>(axisZYX(1), Eigen::Vector3d::UnitY()));
        transl.rotate(Eigen::AngleAxis<double>(axisZYX(2), Eigen::Vector3d::UnitZ()));
    }
    return transl;
}



const lbrDescriptionParameters UrsulaKinematics::LBR_PARAMETERS = { 0.160, 0.200, 0.200, 0.220, 0.180, 0.220, 0.076, 0.050};

const toolDescriptionParameters UrsulaKinematics::TOOL_PARAMETERS = {0.438, 0.0, 0.062, 0.0, 90.0, 0.0, 0.0088, 0.017, 0.305};

int main (int argc, char** argv)
{
    ros::init(argc,argv, "UrsulaKinematics");
    ros::NodeHandle UrsulaKinematicsHandle;
    UrsulaKinematics* UrsulaKinematicsNode = new UrsulaKinematics(UrsulaKinematicsHandle);
    ros::spin();
}
