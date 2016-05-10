#ifndef I_COMMANDER_H
#define I_COMMANDER_H

#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include "Eigen/Dense"
#include "Eigen/Core"

#include "masterslave/DescriptionParameters.h"
#include "masterslave/kinematic/IKinematic.h"
#include "masterslave/commander/BoundingBox.h"

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/JointState.h"
#include "masterslave/Button.h"

#include "masterslave/Manipulation.h"

#include <dynamic_reconfigure/server.h>

#include <masterslave/MasterSlaveConfig.h>
#include <masterslave/BoundingBoxConfig.h>

// State Service MEssage
#include "masterslave/OpenIGTLStateService.h"

#include "masterslave/OpenIGTLStateDescription.h"

/**
 * @file ICommander.h
 *
 * @class ICommander
 * @brief Abstrakte Basisklasse zur Ansteuerung der beiden Kinematikmodelle über ROS
 *
 * @author Fabian Baier
 * @date 20.03.2016
 */


class ICommander
{
    public:

    /**
         * @fn setGripperStatus
         * @brief
         * @param open Flag, ob der Greifer geöffnet werden soll. Diese wird vom Knopfdruck getriggert.
         * @param close Flag, ob der Greifer geschlossen werden soll. Diese vom Knopfdruck getriggert.
         */
        void setGripperStatus(bool open, bool close){ gripper_open = open; gripper_close = close;}

        /**
         * @fn configurationCallback
         * @brief dynamic reconfigure Callbackmethode
         * @param config Konfiguration von dynamic reconfigure als Struktur
         * @param level Bitmaske zur Maskierung einzelner Einstellungen
         * @see MasterSlaveConfig.h
         */
        void configurationCallback(masterslave::MasterSlaveConfig &config, uint32_t level);


    protected:
        /**
         * @fn setZero
         * @brief Stellt die Gelenkwinkelstellung des Werkzeuges auf 0
         */

        void setZero();

        /**
         * @fn statemachineThread
         * @brief abstrakte Methode um den Zustand des OpenIGTLinkInterfaces wechseln
         */
        virtual void statemachineThread(const ros::TimerEvent&)=0;

        /**
         * @var startPositionLBR
         * @brief kartesische Startposition des Endeffektors des LBR
         */
        Eigen::Affine3d startPositionLBR;

        /**
         * @var TCP
         * @brief kartesische Startposition des TCP des Werkzeuges
         */
        Eigen::Affine3d TCP;

        /**
         * @var RCM
         * @brief Trokarpunkt
         */
        Eigen::Affine3d RCM;

        /**
         * @fn calcQ6
         * @brief Die abstrakte Methode berechnet aus dem Gelenkwinkel Q6 die Gelenkwinkel Q6n und Q6p des Zangengreifers
         *
         */
        void calcQ6();

        /**
         * @fn commandVelocities
         * @brief Die abstrakte Methode übernimmt die Kommandierung der Gelenkwinkel
         */
        void commandVelocities();

        /**
         * @fn loop
         * @brief Die abstrakte Methode übernimmt je nach Implementierung die Kommunikation mit dem Roboter bzw. der Simulation und der Kinematikklasse über ROS-Servies
         */
        virtual void loop()=0;

        /**
         * @var buttons
         * @brief Vektor, der die Knopfkonfiguration enthält
         * @todo check ob noch benötigt
         * @see buttonCheck
         */
        std::vector<std::string> buttons;

        ros::Subscriber pliersDistanceSub;

        /**
         * @var velocitySub
         * @todo check ob noch benötigt
         */
        ros::Subscriber velocitySub;

        /**
         * @var Q4StateSub
         * @brief Empfänger in ROS für den Gelenkwinkel Q4
         */
        ros::Subscriber Q4StateSub;

        /**
         * @var Q5StateSub
         * @brief Empfänger in ROS für den Gelenkwinkel Q5
         */
        ros::Subscriber Q5StateSub;

        /**
         * @var Q6pStateSub
         * @brief Empfänger in ROS für den Gelenkwinkel Q6p
         */
        ros::Subscriber Q6pStateSub;

        /**
         * @var Q6nStateSub
         * @brief Empfänger in ROS für den Gelenkwinkel Q6n
         */
        ros::Subscriber Q6nStateSub;

        /**
         * @var lbrPositionSub
         * @brief Empfänger in ROS für die kartesische Flanschpose
         */
        ros::Subscriber lbrPositionSub;

        /**
         * @var buttonSub
         * @brief Empfänger in ROS für die Knopfeingaben für das Eingabegerät
         */
        ros::Subscriber buttonSub;

        /**
         * @var tcpClient
         * @brief ServiceClient zur Ansteuerung der Manipulationsschnittstelle in ROS
         */
        ros::ServiceClient tcpClient;

        /**
         * @var rcmClient
         * @brief ServiceClient zur Bestimmung des Trokarpunktes
         */
        ros::ServiceClient rcmClient;

        /**
         * @var directKinematicsClient
         * @brief ServiceClient zur Durchführung der direkten Kinematik in ROS mit der jeweiligen Kinematik
         */
        ros::ServiceClient directKinematicsClient;

        /**
         * @var inverseKinematicsClient
         * @brief ServiceClient zur Durchführung der inversen Kinematik in ROS mit der jeweiligen Kinematik
         */
        ros::ServiceClient inverseKinematicsClient;

        /**
         * @var stateService
         * @brief ServiceClient zum Zustandswechsel des OpenIGTLink-Interfaces in der ROSOpenIGTLinkBridge
         * @see RosOpenIGTLBridge
         */
        ros::ServiceClient  stateService;

        /**
         * @var Q4Pub
         * @brief Sender in ROS für den Gelenkwinkel Q4
         */
        ros::Publisher  Q4Pub;

        /**
         * @var Q5Pub
         * @brief Sender in ROS für den Gelenkwinkel Q5
         */
        ros::Publisher  Q5Pub;

        /**
         * @var Q6nPub
         * @brief Sender in ROS für den Gelenkwinkel Q6n
         */
        ros::Publisher  Q6pPub;

        /**
         * @var Q6pPub
         * @brief Sender in ROS für den Gelenkwinkel Q6p
         */
        ros::Publisher  Q6nPub;

        /**
         * @var motorAngles
         * @brief Zwischenspeicher für die eingehenden Motorwinkel der Gelenke Q6n und Q6p
         * @see calcQ6
         */
        Eigen::VectorXd motorAngles;

        /**
         * @var jointAnglesTar
         * @brief Vektor der gewünschten Gelenkwinkel des Werkzeuges bzw. des Gesamtsystems
         */
        Eigen::VectorXd jointAnglesTar;

        /**
         * @var jointAnglesAct
         * @brief Vektor der aktuellen Gelenkwinkel des Werkzeuges bzw. des Gesamtsystems
         */
        Eigen::VectorXd jointAnglesAct;

        /**
         * @var gripper_stop
         * @brief Flag, ob der Greifer gestoppt werden soll
         * @see setGripperStatus
         */
        bool gripper_stop{false};

        /**
         * @var gripper_open
         * @brief Flag, ob der Greifer geöffnet werden soll
         * @see setGripperStatus
         */
        bool gripper_open{false};

        /**
         * @var gripper_close
         * @brief Flag, ob der Greifer geschlossen werden soll
         * @see setGripperStatus
         */
        bool gripper_close{false};

        /**
         * @var cycleTime
         * @brief Zykluszeit
         */
        double cycleTime{1};

        /**
         * @var cycleTimeScaleFactor
         * @brief Faktor für die Gelenkwinkelansteuerung des Tools. Wenn es in der Realität verwendet wird, muss cycleTimeScaleFactor = cycleTime gelten. Ansonsten gilt cycleTimeScaleFactor = 1
         * @see configurationCallback
         */
        double cycleTimeScaleFactor{1};

        /**
         * @var gripperVelocityValue
         * @brief Greifergeschwindigkeit in der Einheit [rad/s]
         */
        double gripperVelocityValue{0.1};

        /**
         * @fn QuaternionFromEuler
         * @brief Hilfsmethode um ein Quaternion aus Eulerwinkel zu erstellen
         * @param eulerXYZ Eulerwinkel (Reihenfolge im Vektor X-Y-Z)
         * @param ZYX Flag, ob die Rotationsreihenfolge X-Y'-Z'' oder Z-Y'-X'' ist
         * @return Das berechnete Quaternion
         */
        Eigen::Quaternion<double> QuaternionFromEuler(const Eigen::Vector3d &eulerXYZ, bool ZYX);

        const double DEG_TO_RAD{M_PI/180};

        /**
         * @var boundingBox
         * @brief Zeiger auf ein BoundingBox-Objekt
         * @todo Weitere Begrenzungsgeometrien einfügen und abstrakte Basisklasse einfügen
         */
        std::unique_ptr<BoundingBox> boundingBox;

        Eigen::Vector3d boundingBoxSize{0.33,0.33,0.2};
        double rcmDistance{0.05};

        int Q6CallbacksCalled{0};

        /**
         * @var callBacksCalled
         * @brief Test, ob alle Gelenkwinkelcallbacks aufgerufen wurden
         */
        int callBacksCalled{0};

        int rosRate{1000};

        bool start_{false};

        /**
         * @var statemachineIsRunning
         * @brief Flag, ob das OpenIGTLink-Interface noch läuft
         */
        bool statemachineIsRunning;

        /**
         * @var newState
         * @var state
         * @brief OpenIGTLink-Zustand
         * @see OpenIGTLState.h
         */
        OPENIGTL_STATE newState{NO_STATE};
        OPENIGTL_STATE state{NO_STATE};

        geometry_msgs::TwistStamped velocity_;

        /**
         * @var cycleTimePub
         * @brief Sender zur Bereitstellung der Zykluszeit für verschiedene ROS-Nodes
         */
        ros::Publisher cycleTimePub;

        bool setTrocar{false};

        double CRITICAL_PLIERS_ANGLE{0.95*M_PI};

        double PLIERS_DISTANCE_TOLERANCE{0.04};

        double PLIERS_LENGTH{0.017};

        double pliersOpeningAngle{0};

        double pliersOpeningAngleOld{0};

};

#endif // TASK_H
