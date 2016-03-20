#ifndef ROSOPENIGTLBRIDGE_H
#define ROSOPENIGTLBRIDGE_H

#include <ros/ros.h>
#include <stdlib.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/JointState.h"
#include <dynamic_reconfigure/server.h>
#include <masterslave/RosOpenIGTLBridgeConfig.h>
#include "masterslave/OpenIGTLStateDescription.h"
#include "masterslave/OpenIGTLStateService.h"

// for own thread
#include "ros/callback_queue.h"
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include "boost/thread/mutex.hpp"

// for own OpenIGTLink
#include "igtlOSUtil.h"
#include "igtlMessageHeader.h"
#include "igtlTransformMessage.h"
#include "igtlClientSocket.h"
#include "igtlStatusMessage.h"
#include "igtlStringMessage.h"
#include "igtl_transform.h"

#include <sstream>
#include "Eigen/Eigen"
#include "Eigen/Dense"
#include <eigen_conversions/eigen_msg.h>


/**
 * \file RosOpenIGTLBridge.h
 *
 * \class RosOpenIgtlBridge
 * \brief Eine ROS-Node zur Kommunikation zwischen ROS und dem LBR iiwa, welche über das OpenIGTLink-Protokoll durchgeführt.
 *
 *
 * \author Fabian Baier
 * \date 19.03.2016
 *
 * \version 0.3
 *
 */
class RosOpenIgtlBridge
{
public:

    RosOpenIgtlBridge(ros::NodeHandle);
    /**
     * \fn bool stateService(masterslave::OpenIGTLStateService::Request&, masterslave::OpenIGTLStateService::Response&)
     * \brief Dieses ist eine Callback-Methode, die aufgerufen wird, wenn der Zustand des Roboters geändert werden soll
     * Mögliche Zustände: IDLE, FREE, MOVE_TO_POSE
     * \return War der Service-Call erfolgreich?
     */
    bool stateService(masterslave::OpenIGTLStateService::Request&, masterslave::OpenIGTLStateService::Response&);
private:

    /**
     * \fn void transformCallback(geometry_msgs::PoseStampedConstPtr)
     * \brief In der Methode werden die Sollpositionen aus der geometrischen Kinematik entgegen genommen
     *
     * \param Die Solllage des Roboterendeffektors
     * \see GeometricKinematic.h
     */
    void transformCallback(geometry_msgs::PoseStampedConstPtr);

    /**
     * \fn void lbrJointAngleCallback(const std_msgs::Float64ConstPtr&, int number)
     * \brief In dieser Methode werden die
     * \param Gelenkwinkel des Gelenks
     * \param number Nummer des LBR-Gelenks
     */
    void lbrJointAngleCallback(const std_msgs::Float64ConstPtr&, int number);

    /**
     * \fn void openIGTLinkThread()
     * \brief zyklische Versendung der durch ROS empfangenen Daten über einen OpenIGTLink-Clientsocket
     */
    void openIGTLinkThread();

    /**
     * \fn void openIGTLinkTransformThread()
     * \brief zyklischer Empfang der Gelenktransformationen bzw. der Endeffektorlage des LBR
     */
    void openIGTLinkTransformThread();

    /**
     * \fn int sendCommand(igtl::ClientSocket::Pointer&, std::string)
     * \brief Methode zum Versenden von std::strings über den OpenIGTLink-Socket
     * \param Zeiger auf den Socket, über den gesendet werden soll
     * \param Zuversendende String-Nachricht
     * \return Status nach Versendung des Pakets
     */
    int sendCommand(igtl::ClientSocket::Pointer&, std::string);

    /**
     * \fn int positionReached(igtl::ClientSocket::Pointer&, igtl::MessageBase::Pointer&)
     * @brief Überwachung, ob die angegebene Zielpose bereits erreicht wurde
     * \param Zeiger auf den Socket, über den gesendet wurde
     * \param Zeiger auf den Nachrichtenkopf, der als Acknowledge zurück kam
     * @return Empfangsstatus des Pakets
     */
    int positionReached(igtl::ClientSocket::Pointer&, igtl::MessageBase::Pointer&);

    /**
     * \fn int receiveTransform(igtl::ClientSocket::Pointer&, igtl::MessageBase::Pointer&)
     * @brief Empfang der Transformationsnachrichten
     * \param Zeiger auf den Socket, mithilfe dessen die Nachrichten empfangen werden
     * \param Zeiger auf den Nachrichtenkopf, and die Transformationen angehängt sind
     * @return Empfangsstatus des Pakets
     */
    int receiveTransform(igtl::ClientSocket::Pointer&, igtl::MessageBase::Pointer&);

    /**
     * \fn std::string rosPoseToIGTL(geometry_msgs::Pose)
     * @brief Hilfsfunktion um eine geometry_msgs::Pose in einen String zu verwandeln, um diesen anschließend über OpenIGTLink zu versenden
     * \param Die gewünschen Solllage im Variablentyp geometry_msgs::Pose
     * @return die Transformationsmatrix im Variablentyp std::string
     */
    std::string rosPoseToIGTL(geometry_msgs::Pose);

    /**
     * \fn geometry_msgs::Pose igtlMatrixToRosPose(igtl::Matrix4x4&)
     * @brief Hilfsfunktion um eine igtl::Matrix4x4 in eine geometry_msgs::Pose zu wandeln und diese in ROS zu übertragen
     * @return
     */
    geometry_msgs::Pose igtlMatrixToRosPose(igtl::Matrix4x4&);

    /**
     * \fn configurationIGTLCallback(masterslave::RosOpenIGTLBridgeConfig &config, uint32_t level)
     * @brief Callbackmethode für dynamic reconfigure
     * @param config Konfiguration von dynamic reconfigure
     * @param level Bitmaske zur Maskierung einzelner Einstellungen
     * @see RosOpenIGTLBridge.cfg
     */
    void configurationIGTLCallback(masterslave::RosOpenIGTLBridgeConfig &config, uint32_t level);

    /**
     * \var flangeTargetSub
     * \brief Empfänger für Solllagen des Endeffektors in ROS
     */
    ros::Subscriber flangeTargetSub;

    /**
     * \var lbrJointAngleSub
     * @brief Empfänger für die  gewünschte Gelenkwinkelposition des LBR iiwa in ROS
     */
    std::array<ros::Subscriber,7> lbrJointAngleSub;

    /**
     * \var flangePub
     * \brief Sender für die eingenommene Endeffektorlage in ROS
     */
    ros::Publisher flangePub;

    /**
     * \var lbrJointAnglePub
     * @brief Sender für die aktuelle Gelenkwinkelkonfiguration des LBR iiwa
     */
    std::array<ros::Publisher,7> lbrJointAnglePub;

    /**
     * \var stateServiceServer
     * @brief Objekt zur Bereitstellung des stateServices
     * @see bool stateService(masterslave::OpenIGTLStateService::Request&, masterslave::OpenIGTLStateService::Response&)
     */
    ros::ServiceServer stateServiceServer;


    ros::NodeHandle nh_;
    boost::mutex update_mutex_;
    boost::thread openIGTLThread;

    /**
     * \var positionReached_
     * @brief wurde die Pose durch den LBR erfolgreich angefahren?
     */
    int positionReached_;

    /**
     *  \var sendTransformFlag
     *  \brief wurde die aktuell vorliegende Solllage des LBR iiwa schon über OpenIGTLink übertragen?
     */
    bool sendTransformFlag{false};

    boost::mutex transformUpdateMutex_;

    /**
     * \var T_FL
     * \brief Solllage des Endeffektors des LBR
     */
    igtl::Matrix4x4 T_FL;

    /**
     * \var T_FL_new
     * @brief aktuell empfangene Lage des Endeffektors des LBR
     */
    igtl::Matrix4x4 T_FL_new;

    /**
     * \var poseFL
     * \brief Solllage des Endeffektors im ROS-konformen Variablentyp
     * \see T_FL
     */
    geometry_msgs::Pose poseFL;

    /**
     * \var poseFL_new
     * \brief aktuell empfangene Lage des Endeffektors des LBR im ROS-konformen Variablentyp
     * \see T_FL_new
     */
    geometry_msgs::Pose poseFL_new;

    /**
     * \var jointAngles
     * \brief gewünschte Gelenkwinkel des LBR iiwa
     */
    Eigen::VectorXd jointAngles;

    /**
     * \var jointAngles_new
     * \brief aktuell empfangene Gelenkwinkel des LBR iiwa
     */
    Eigen::VectorXd jointAngles_new;

    /**
     * \var commandSocket_
     * \brief Socket über den Gelenkwinkel oder Endeffektorsolllagen kommandiert werden mit IP und Port als Konstanten
     * \see COMMAND_PORT, COMMAND_IP
     */
    igtl::ClientSocket::Pointer commandSocket_;
    const int COMMAND_PORT{49001};
    const char* COMMAND_IP{"172.31.1.147"};

    /**
     * @var rCommand
     * \brief Status des Sockets
     */
    int rCommand;

    /**
     * \var CMD_UID
     * \brief Nummer des übermittelten Datenpakets
     */
    unsigned long long CMD_UID{0};

    /**
     * @var transformSocket_
     * \brief Socket über den die Gelenktransformationen und die Endeffektorlage empfangen werden mit IP und Port als Konstanten
     * \see TRANSFORM_PORT, COMMAND_IP
     */
    igtl::ClientSocket::Pointer transformSocket_;
    const int TRANSFORM_PORT{49002};
    const char* TRANSFORM_IP{"172.31.1.147"};

    /**
     * @var rTransform
     * \brief Status des Empfangssockets
     */
    int rTransform;

    std::string openIGTLCommandString;
    std::string stateString;

    double sampleTime_;

    bool stop_;
    bool start_;
    bool transformReceived_{false};
    bool rosTransformReceived_{false};

    const unsigned int CONNECTION_TIMEOUT{30};

};

#endif // ROSOPENIGTLBRIDGE_H
