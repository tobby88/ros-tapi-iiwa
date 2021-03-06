/******************************************************************************
 *  Copyright (C) 2016 by Fabian Baier and Tobias Holst                       *
 *                                                                            *
 *  This file is part of tapi_iiwa.                                           *
 *                                                                            *
 *  tapi_iiwa is free software: you can redistribute it and/or modify         *
 *  it under the terms of the GNU General Public License as published by      *
 *  the Free Software Foundation, either version 3 of the License, or         *
 *  (at your option) any later version.                                       *
 *                                                                            *
 *  tapi_iiwa is distributed in the hope that it will be useful,              *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of            *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             *
 *  GNU General Public License for more details.                              *
 *                                                                            *
 *  You should have received a copy of the GNU General Public License         *
 *  along with tapi_iiwa.  If not, see <http://www.gnu.org/licenses/>.        *
 *                                                                            *
 *  Diese Datei ist Teil von tapi_iiwa.                                       *
 *                                                                            *
 *  tapi_iiwa ist Freie Software: Sie können es unter den Bedingungen         *
 *  der GNU General Public License, wie von der Free Software Foundation,     *
 *  Version 3 der Lizenz oder (nach Ihrer Wahl) jeder späteren                *
 *  veröffentlichten Version, weiterverbreiten und/oder modifizieren.         *
 *                                                                            *
 *  tapi_iiwa wird in der Hoffnung, dass es nützlich sein wird, aber          *
 *  OHNE JEDE GEWÄHRLEISTUNG, bereitgestellt; sogar ohne die implizite        *
 *  Gewährleistung der MARKTFÄHIGKEIT oder EIGNUNG FÜR EINEN BESTIMMTEN ZWECK *
 *  Siehe die GNU General Public License für weitere Details.                 *
 *                                                                            *
 *  Sie sollten eine Kopie der GNU General Public License zusammen mit diesem *
 *  Programm erhalten haben. Wenn nicht, siehe <http://www.gnu.org/licenses/>.*
 ******************************************************************************/

#include "tapi_iiwa.hpp"

#define M_TO_MM 1.0 / 1000.0f

#define MM_TO_M 1000

Tapi_iiwa::Tapi_iiwa(ros::NodeHandle nh) : nh_(nh)
{
  tpub = new Tapi::Publisher(&nh, "Kuka iiwa");
  tsub = new Tapi::Subscriber(&nh, "Kuka iiwa");
  tservice = new Tapi::ServiceServer(&nh, "Kuka iiwa Service");

  jointAngles = Eigen::VectorXd::Zero(7);
  jointAngles_new = Eigen::VectorXd::Zero(7);
  // flangeTargetSub = nh_.subscribe("/flangeTarget", 1, &Tapi_iiwa::transformCallback, this);
  ros::SubscribeOptions opt;
  opt = SubscribeOptionsForTapi(geometry_msgs::PoseStamped, 1, &Tapi_iiwa::transformCallback);
  tsub->AddFeature(opt, "Target Position");

  // flangePub = nh_.advertise<geometry_msgs::PoseStamped>("/flangeLBR", 1);
  flangePub = tpub->AddFeature<geometry_msgs::PoseStamped>("Current Position", 1);

  /*for (int i = 0; i < lbrJointAngleSub.size(); i++)
  {
    std::stringstream sstream;
    sstream << "/LBR/des/joint" << i + 1;
    lbrJointAngleSub[i] = nh_.subscribe<std_msgs::Float64>(sstream.str().c_str(), 1,
                                                           boost::bind(&Tapi_iiwa::lbrJointAngleCallback, this, _1, i));
    sstream.str(std::string());

    sstream << "/LBR/act/joint" << i + 1;
    lbrJointAnglePub[i] = nh_.advertise<sensor_msgs::JointState>(sstream.str().c_str(), 1);
    sstream.str(std::string());
  }*/
  // stateServiceServer = nh_.advertiseService("/openIGTLState", &Tapi_iiwa::stateService, this);
  ros::AdvertiseServiceOptions opt2;
  opt2 = ServiceServerOptionsForTapi(tapi_iiwa::OpenIGTLStateService, &Tapi_iiwa::stateService);
  stateServiceServer = tservice->AddFeature(opt2, "Mode");

  boost::thread(boost::bind(&Tapi_iiwa::openIGTLinkTransformThread, this));
  boost::thread(boost::bind(&Tapi_iiwa::openIGTLinkThread, this));

  ros::Timer timer = nh_.createTimer(ros::Duration(0.001), &Tapi_iiwa::loop, this);
  ros::spin();
}

Tapi_iiwa::~Tapi_iiwa()
{
  delete tpub;
  delete tsub;
  delete tservice;
}

void Tapi_iiwa::loop(const ros::TimerEvent &event)
{
  // Get pose of flange
  geometry_msgs::PoseStamped poseFLmsg;
  poseFLmsg.header.stamp = ros::Time::now();
  poseFLmsg.pose = poseFL;
  this->flangePub->publish(poseFLmsg);

  /*for (int i = 0; i < 7; i++)
  {
    sensor_msgs::JointState temp;
    temp.position.push_back(jointAngles[i]);
    lbrJointAnglePub[i].publish(temp);
  }*/
}

void Tapi_iiwa::transformCallback(geometry_msgs::PoseStampedConstPtr transform)
{
  poseFL_new = transform->pose;
  rosTransformReceived_ = true;
}

// second thread to get transform messages from the OpenIGTL-Interface
void Tapi_iiwa::openIGTLinkTransformThread()
{
  ROS_INFO("Entering Transformation Thread");
  transformSocket_ = igtl::ClientSocket::New();
  transformSocket_->DebugOff();

  double startTime = ros::Time::now().toSec();
  double endTime = ros::Time::now().toSec();
  do
  {
    rTransform = transformSocket_->ConnectToServer(TRANSFORM_IP, TRANSFORM_PORT);
    endTime = ros::Time::now().toSec();
    ROS_WARN_NAMED("ROSOpenIGTL", "Trying to connect to OpenIGTL-Server");
  } while (rTransform == -1 && ros::ok() && endTime - startTime <= CONNECTION_TIMEOUT);
  if (rTransform == -1)
  {
    ROS_ERROR("No OpenIGTLink-Server available");
    ros::shutdown();
  }

  while (rTransform != -1 && ros::ok())
  {
    igtl::MessageHeader::Pointer messageHeaderTransform;
    messageHeaderTransform = igtl::MessageHeader::New();
    this->receiveTransform(transformSocket_, messageHeaderTransform);
    rTransform = transformSocket_->GetConnected();
  }
  transformSocket_->CloseSocket();
  ROS_ERROR("No OpenIGTLink-Server available");
  ros::shutdown();
}

void Tapi_iiwa::openIGTLinkThread()
{
  ROS_INFO("Entering Command Thread");
  commandSocket_ = igtl::ClientSocket::New();
  commandSocket_->DebugOff();
  rCommand = -10;
  double startTime = ros::Time::now().toSec();
  double endTime = ros::Time::now().toSec();
  ROS_INFO_STREAM("rCommand: " << rCommand);
  do
  {
    rCommand = commandSocket_->ConnectToServer(COMMAND_IP, COMMAND_PORT);
    endTime = ros::Time::now().toSec();
    ROS_WARN_NAMED("ROSOpenIGTL", "Trying to connect to OpenIGTL-Server");
  } while (rCommand == -1 && ros::ok() && endTime - startTime <= CONNECTION_TIMEOUT);
  if (rCommand == -1)
  {
    ROS_ERROR("No OpenIGTLink-Server available");
    ros::shutdown();
  }

  ROS_DEBUG_STREAM("rCommand: " << rCommand << " rTransform: " << rTransform);

  this->sendCommand(commandSocket_, "Idle;");
  ros::Rate rate(30);
  while (rCommand != -1 && ros::ok())
  {
    if (sendTransformFlag)
    {
      sendTransformFlag = false;
      igtl::MessageHeader::Pointer messageHeaderCommand;
      messageHeaderCommand = igtl::MessageHeader::New();
      positionReached_ = this->positionReached(commandSocket_, messageHeaderCommand);
    }

    if (commandReceivedFromROS)
    {
      commandStringMutex_.lock();
      this->sendCommand(commandSocket_, openIGTLCommandString);
      commandStringMutex_.unlock();
      transformReceived_ = false;
    }
    rCommand = commandSocket_->GetConnected();
    rate.sleep();
  }
  commandSocket_->CloseSocket();
  ROS_ERROR("No OpenIGTLink-Server available");
  ros::shutdown();
}

int Tapi_iiwa::positionReached(igtl::ClientSocket::Pointer &socket, igtl::MessageBase::Pointer &msgHeader)
{
  msgHeader->InitPack();
  rCommand = socket->Receive(msgHeader->GetPackPointer(), msgHeader->GetPackSize());
  msgHeader->Unpack();
  igtl::StringMessage::Pointer stringMsg;
  stringMsg->SetMessageHeader(msgHeader);
  stringMsg->AllocatePack();
  socket->Receive(stringMsg->GetPackBodyPointer(), stringMsg->GetPackBodySize());
  stringMsg->Unpack();
  std::string msg = stringMsg->GetString();
  if (msg == "true")
  {
    return 1;
  }
  return 0;
}

int Tapi_iiwa::receiveTransform(igtl::ClientSocket::Pointer &socket, igtl::MessageBase::Pointer &msgHeader)
{
  igtl::Matrix4x4 T_FL;
  igtl::Matrix4x4 jointAnglesIGTL;
  igtl::TransformMessage::Pointer transformMsg;
  msgHeader->InitPack();
  stateUpdateMutex_.try_lock();
  rTransform = socket->Receive(msgHeader->GetPackPointer(), msgHeader->GetPackSize());
  stateUpdateMutex_.unlock();
  msgHeader->Unpack();
  transformMsg = igtl::TransformMessage::New();
  transformMsg->SetMessageHeader(msgHeader);
  transformMsg->AllocatePack();

  socket->Receive(transformMsg->GetPackBodyPointer(), transformMsg->GetPackBodySize());

  transformMsg->Unpack();
  if (strcmp(transformMsg->GetDeviceName(), "JointData") == 0)
  {
    int cnt = 0;

    // Empfang der Gelenkwinkel vom Visualisierungsinterface bzw. dem Socket
    transformMsg->GetMatrix(jointAnglesIGTL);
    jointAngleUpdateMutex_.lock();
    for (int i = 0; i < 4; i++)
    {
      for (int j = 0; j < 3; j++)
      {
        jointAngles[cnt] = jointAnglesIGTL[j][i];
        cnt++;
        if (cnt == 7)
          break;
      }
      if (cnt == 7)
        break;
    }
    jointAngleUpdateMutex_.unlock();
  }
  if (strcmp(transformMsg->GetDeviceName(), "T_EE") == 0)
  {
    // ROS_DEBUG("T_EE empfangen");

    transformMsg->GetMatrix(T_FL);
    transformUpdateMutex_.lock();
    poseFL = this->igtlMatrixToRosPose(T_FL);
    transformUpdateMutex_.unlock();
    return 1;
  }
  return 0;
}

int Tapi_iiwa::sendCommand(igtl::ClientSocket::Pointer &socket, std::string command)
{
  // if(!commandReceivedFromROS) return -1;
  igtl::StringMessage::Pointer transformStringMsg;
  transformStringMsg = igtl::StringMessage::New();
  std::stringstream transformStream;
  CMD_UID++;
  if (this->CMD_UID > 999999999999)
    this->CMD_UID = 0;
  transformStream << "CMD_" << CMD_UID;
  // Set UID
  transformStringMsg->SetDeviceName(transformStream.str().c_str());
  transformStream.str(std::string());
  transformStringMsg->SetString(command);
  transformStringMsg->Pack();
  ROS_INFO_STREAM("Send: \n" << command);

  return socket->Send(transformStringMsg->GetPackPointer(), transformStringMsg->GetPackSize());
}

std::string Tapi_iiwa::rosPoseToIGTL(geometry_msgs::Pose pose)
{
  std::stringstream sstream;
  pose.position.x *= MM_TO_M;
  pose.position.y *= MM_TO_M;
  pose.position.z *= MM_TO_M;
  Eigen::Affine3d poseToEigen;
  tf::poseMsgToEigen(pose, poseToEigen);

  sstream << poseToEigen.translation().x() << ";";
  sstream << poseToEigen.translation().y() << ";";
  sstream << poseToEigen.translation().z() << ";";
  for (int row = 0; row < 3; row++)
  {
    for (int col = 0; col < 3; col++)
    {
      sstream << poseToEigen.matrix()(row, col) << ";";
    }
  }
  return sstream.str();
}

geometry_msgs::Pose Tapi_iiwa::igtlMatrixToRosPose(igtl::Matrix4x4 &igtlMatrix)
{
  geometry_msgs::Pose returnValue;
  Eigen::Affine3d eigenMatrix;
  for (int row = 0; row < 4; row++)
  {
    for (int col = 0; col < 4; col++)
    {
      eigenMatrix.matrix()(row, col) = igtlMatrix[row][col];
    }
  }
  tf::poseEigenToMsg(eigenMatrix, returnValue);
  returnValue.position.x *= M_TO_MM;
  returnValue.position.y *= M_TO_MM;
  returnValue.position.z *= M_TO_MM;
  return returnValue;
}

bool Tapi_iiwa::stateService(tapi_iiwa::OpenIGTLStateService::Request &req,
                             tapi_iiwa::OpenIGTLStateService::Response &res)
{
  std::stringstream sstream;
  stateUpdateMutex_.try_lock();
  if (rTransform == -1)
  {
    res.alive = false;
  }
  else
  {
    res.alive = true;
  }
  stateUpdateMutex_.unlock();
  // ROS_WARN_STREAM(req.state);
  stateString = req.state;
  sstream << stateString;

  // Checkt ob eine Transformation von ROS angekommen ist, die weitergeleitet werden kann
  // ROS_INFO_STREAM(stateString);
  // ROS_INFO_STREAM(rosTransformReceived_);
  if (rosTransformReceived_ && stateString == "MoveToPose;rob;")
  {
    transformUpdateMutex_.lock();
    sstream << rosPoseToIGTL(poseFL_new);
    transformUpdateMutex_.unlock();
    rosTransformReceived_ = false;
    commandReceivedFromROS = true;
  }
  // Checkt den Statestring und ob alle Gelenkwinkel empfangen wurden
  else if (stateString == "MoveToPose;rob;" && (jointAnglesCalled + 1 >> 7) == 1)
  {
    jointAngleUpdateMutex_.lock();
    for (int i = 0; i < jointAngles_new.rows(); i++)
    {
      sstream << jointAngles_new(i) << ";";
    }
    jointAngleUpdateMutex_.unlock();
    jointAnglesCalled = 0;
    commandReceivedFromROS = true;
  }
  else if (stateString == "IDLE;" || stateString == "GravComp;")
  {
    commandReceivedFromROS = true;
  }
  commandStringMutex_.lock();
  openIGTLCommandString = sstream.str();
  commandStringMutex_.unlock();
  // ROS_WARN_STREAM("Commandstring: " << openIGTLCommandString);
  transformReceived_ = true;
  sstream.str(std::string());
  stateServiceCalled_ = true;
  return true;
}

void Tapi_iiwa::lbrJointAngleCallback(const std_msgs::Float64ConstPtr &jointAngle, int number)
{
  jointAngles_new(number) = jointAngle->data;

  // Es wird geguckt, ob alle Gelenke schon empfangen wurden
  if (jointAnglesCalled + 1 < 1 << 7)
  {
    // Binäre Operation (Bits der Gelenknummern zu 1 gesetzt)
    jointAnglesCalled += 1 << number;
  }
}
