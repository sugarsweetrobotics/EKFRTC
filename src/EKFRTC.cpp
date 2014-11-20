// -*- C++ -*-
/*!
 * @file  EKFRTC.cpp
 * @brief Extended Kalman Fileter RTC
 * @date $Date$
 *
 * $Id$
 */

#include "EKFRTC.h"

// Module specification
// <rtc-template block="module_spec">
static const char* ekfrtc_spec[] =
  {
    "implementation_id", "EKFRTC",
    "type_name",         "EKFRTC",
    "description",       "Extended Kalman Fileter RTC",
    "version",           "1.0.0",
    "vendor",            "Sugar Sweet Robotics",
    "category",          "Experimental",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.debug", "0",
    "conf.default.odom_cov_matrix", "1,0,0, 0,1,0, 0,0,1", 
    "conf.default.imu_cov_matrix", "1,0,0,0,0,0, 0,1,0,0,0,0, 0,0,1,0,0,0, 0,0,0,1,0,0, 0,0,0,0,1,0, 0,0,0,0,0,1",
    // Widget
    "conf.__widget__.debug", "text",
    "conf.__widget__.odom_cov_matrix", "text",
    "conf.__widget__.imu_cov_matrix", "text",
    // Constraints
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
EKFRTC::EKFRTC(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_accelIn("accel", m_accel),
    m_angularVelIn("angularVel", m_angularVel),
    m_odomIn("odom", m_odom),
    m_estimatedPoseOut("estimatedPose", m_estimatedPose)


    // </rtc-template>
  ,m_odomVelocity(0, 0, 0), m_imuVelocity(0, 0, 0)
{
}

/*!
 * @brief destructor
 */
EKFRTC::~EKFRTC()
{
}



RTC::ReturnCode_t EKFRTC::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("accel", m_accelIn);
  addInPort("angularVel", m_angularVelIn);
  addInPort("odom", m_odomIn);
  
  // Set OutPort buffer
  addOutPort("estimatedPose", m_estimatedPoseOut);
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("debug", m_debug, "0");
  bindParameter("odom_cov_matrix", m_odom_cov_matrix, "1,0,0,0,1,0,0,0,1");
  bindParameter("imu_cov_matrix", m_imu_cov_matrix, "1,0,0,0,0,0, 0,1,0,0,0,0, 0,0,1,0,0,0, 0,0,0,1,0,0, 0,0,0,0,1,0, 0,0,0,0,0,1");
  // </rtc-template>
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t EKFRTC::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t EKFRTC::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t EKFRTC::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t EKFRTC::onActivated(RTC::UniqueId ec_id)
{
  std::cout << "[RTC::EKFRTC] onActivated." << std::endl;
  double odom_cov_matrix[9];
  double imu_cov_matrix[36];
  std::stringstream nss(m_odom_cov_matrix);
  std::string token;
  int i = 0;
  while(std::getline(nss, token, ',')) {
    std::stringstream trimmer;
    trimmer << token;
    token.clear();
    trimmer >> token;
    //std::cout << " key = " << token << std::endl;
    //keys.push_back(token);
    if (i >= 9) { break; }
    odom_cov_matrix[i] = atof(token.c_str());
    i++;
  }

  std::stringstream iss(m_imu_cov_matrix);
  std::string token2;
  i = 0;
  while(std::getline(iss, token, ',')) {
    std::stringstream trimmer;
    trimmer << token2;
    token2.clear();
    trimmer >> token2;
    //std::cout << " key = " << token << std::endl;
    //keys.push_back(token);
    if (i >= 36) { break; }
    imu_cov_matrix[i] = atof(token2.c_str());
    i++;
  }

  ssr::Matrix33 odomCovMat(odom_cov_matrix[0], odom_cov_matrix[1], odom_cov_matrix[2],
			   odom_cov_matrix[3], odom_cov_matrix[4], odom_cov_matrix[5],
			   odom_cov_matrix[6], odom_cov_matrix[7], odom_cov_matrix[8]);

  std::cout << "[RTC::EKFRTC] Odom Cov Matrix = \n"
	    << odomCovMat.str() << std::endl;
  ssr::Matrix33 imuCovMat(imu_cov_matrix[0], imu_cov_matrix[1], imu_cov_matrix[5],
			  imu_cov_matrix[6], imu_cov_matrix[7], imu_cov_matrix[11],
			  imu_cov_matrix[30], imu_cov_matrix[31], imu_cov_matrix[35]);
	
  std::cout << "[RTC::EKFRTC] IMU Cov Matrix = \n"
	    << imuCovMat.str() << std::endl;
  

  m_pEKF = new ssr::EKF4MobileRobot(odomCovMat, imuCovMat);
  m_odomUpdated = m_accelUpdated = m_angularVelUpdated = false;
  m_initEkf = false;
  return RTC::RTC_OK;
}


RTC::ReturnCode_t EKFRTC::onDeactivated(RTC::UniqueId ec_id)
{
  std::cout << "[RTC::EKFRTC] onDeactivated." << std::endl;
  delete m_pEKF;
  return RTC::RTC_OK;
}


RTC::ReturnCode_t EKFRTC::onExecute(RTC::UniqueId ec_id)
{
  if(m_odomIn.isNew()) {
    m_odomIn.read();

    double dx = m_odom.data.position.x - m_odom_old.data.position.x;
    double dy = m_odom.data.position.y - m_odom_old.data.position.y;
    double dh = m_odom.data.heading - m_odom_old.data.heading;
    double dt = m_odom.tm.sec - m_odom_old.tm.sec + (m_odom.tm.nsec-m_odom_old.tm.nsec)/1000000000;
    m_odomVelocity.vx = dx / dt;
    m_odomVelocity.vy = dy / dt;
    m_odomVelocity.va = dh / dt;
    m_timestamp = m_odom.tm;
    m_odom_old = m_odom;
    m_odomUpdated = true;
  }

  if(m_accelIn.isNew()) {
    m_accelIn.read();
    double dt = m_accel.tm.sec - m_accel_old.tm.sec + (m_accel.tm.nsec - m_accel_old.tm.nsec)/1000000000;
    double vx = (m_accel.data.ax + m_accel_old.data.ax)/2 * dt;
    double vy = (m_accel.data.ay + m_accel_old.data.ay)/2 * dt;
    //double vz = (m_accel.data.az + m_accel_old.data.az)/2 * dt;
    m_imuVelocity.vx = vx;
    m_imuVelocity.vy = vy;
    
    m_timestamp = m_accel.tm;
    m_accel_old = m_accel;
    m_accelUpdated = true;
  }

  if(m_angularVelIn.isNew()) {
    m_angularVelIn.read();
    double vz = m_angularVel.data.avz;
    m_imuVelocity.va = vz;
    m_timestamp = m_angularVel.tm;
    m_angularVel_old = m_angularVel;
    m_angularVelUpdated = true;
  }

  if(m_odomUpdated && m_accelUpdated && m_angularVelUpdated) {
    if(m_initEkf) {
      double dt = m_timestamp.sec - m_timestamp_old.sec + (m_timestamp.nsec - m_timestamp_old.nsec)/1000000000;
	  ssr::Pose2D estimatedPose = (*m_pEKF)(m_odomVelocity, m_imuVelocity, dt);
	  m_estimatedPose.data.position.x = estimatedPose.x;
      m_estimatedPose.data.position.y = estimatedPose.y;
      m_estimatedPose.data.heading = estimatedPose.th;
      m_estimatedPose.tm = m_timestamp;
      m_estimatedPoseOut.write();

      if(m_debug) {
	std::cout << "[RTC::EKFRTC] [DEBUG] Estimated Pose = ("
		  << estimatedPose.x << ", " << estimatedPose.y << ", " << estimatedPose.th << ")" << std::endl;
	  }
    } else {
      m_initEkf = true;
      std::cout << "[RTC::EKFRTC] Kalman Filter Successfully Initialized" << std::endl;
    }      

    m_odomUpdated = m_accelUpdated = m_angularVelUpdated = false;
    m_timestamp_old = m_timestamp;

  }

  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t EKFRTC::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t EKFRTC::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t EKFRTC::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t EKFRTC::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t EKFRTC::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void EKFRTCInit(RTC::Manager* manager)
  {
    coil::Properties profile(ekfrtc_spec);
    manager->registerFactory(profile,
                             RTC::Create<EKFRTC>,
                             RTC::Delete<EKFRTC>);
  }
  
};


