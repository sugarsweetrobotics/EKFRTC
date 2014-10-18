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
    "conf.default.odom_cov_matrix", "[1,0,0,0,0,0, 0,1,0,0,0,0, 0,0,1,0,0,0, 0,0,0,1,0,0, 0,0,0,0,1,0, 0,0,0,0,0,1]",
    "conf.default.imu_cov_matrix", "[1,0,0,0,0,0, 0,1,0,0,0,0, 0,0,1,0,0,0, 0,0,0,1,0,0, 0,0,0,0,1,0, 0,0,0,0,0,1]",
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
  bindParameter("odom_cov_matrix", m_odom_cov_matrix, "[1,0,0,0,0,0, 0,1,0,0,0,0, 0,0,1,0,0,0, 0,0,0,1,0,0, 0,0,0,0,1,0, 0,0,0,0,0,1]");
  bindParameter("imu_cov_matrix", m_imu_cov_matrix, "[1,0,0,0,0,0, 0,1,0,0,0,0, 0,0,1,0,0,0, 0,0,0,1,0,0, 0,0,0,0,1,0, 0,0,0,0,0,1]");
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
  return RTC::RTC_OK;
}


RTC::ReturnCode_t EKFRTC::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}


RTC::ReturnCode_t EKFRTC::onExecute(RTC::UniqueId ec_id)
{
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


