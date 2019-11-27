/*
 * Ubitrack - Library for Ubiquitous Tracking
 * Copyright 2006, Technische Universitaet Muenchen, and individual
 3* contributors as indicated by the @authors tag. See the
 * copyright.txt in the distribution for a full listing of individual
 * contributors.
 *
 * This is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this software; if not, write to the Free
 * Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA, or see the FSF site: http://www.fsf.org.
 */

/**
 * @ingroup vision_components
 * @file
 * Reads IMU data
 *
 * @author Adnane Jadid <jadid@in.tum.de>
 */

// Include this header file to get access to VectorNav sensors.
#include "vn/sensors.h"
#include "vn/ezasyncdata.h"
// We need this file for our sleep function.

#include "vn/thread.h"

#include <string>
#include <list>
#include <iostream>
#include <iomanip>
#include <strstream>
#include <log4cpp/Category.hh>

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/scoped_array.hpp>

#include <utDataflow/PushSupplier.h>
#include <utDataflow/PullSupplier.h>
#include <utDataflow/Component.h>
#include <utDataflow/ComponentFactory.h>
#include <utMeasurement/Measurement.h>
#include <utMeasurement/TimestampSync.h>
#include <utUtil/OS.h>
#include <utUtil/TracingProvider.h>


// get a logger
static log4cpp::Category& logger( log4cpp::Category::getInstance( "Ubitrack.Components.VectorNAVSensor" ) );

using namespace Ubitrack;
using namespace vn::sensors;

namespace Ubitrack { namespace Drivers {

/**
 * @ingroup components
 *
 * @par Input Ports
 * None.
 *
 * @par Output Ports
 * \c Output push port of type Ubitrack::Measurement::IMUMeasurement.
 *
 * @par Configuration
 * The configuration tag contains a \c <dsvl_input> configuration.
 * For details, see the InertialSense Docs
 *
 */
	class VectorNAVSensor
	: public Dataflow::Component
{
public:

	/** constructor */
	VectorNAVSensor(const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph >);

	/** destructor, waits until thread stops */
	~VectorNAVSensor();

	/** starts the IMU */
	void start();

	/** starts the capturing */
	void startCapturing();

	/** stops the IMU */
	void stop();

protected:

	
	void handleImuMessage(QuaternionMagneticAccelerationAndAngularRatesRegister reg);


	// shift timestamps (ms)
	int m_timeOffset;

	//  create a VnSensor object to use it to connect to VNsensor.
	VnSensor vs;
	//EzAsyncData* ez;

	// Get some orientation and IMU data.
	QuaternionMagneticAccelerationAndAngularRatesRegister reg;

	//params
	int freq;
	std::string portName;
	int baudRate;

	/** thread is running?*/
	bool m_bStop;

	// pointer to the thread
	boost::shared_ptr< boost::thread > m_pThread;

	int count;

	uint8_t inByte;

	int messageSize;
	uint8_t buffer[2048];

	/** timestamp of last frame */
	double m_lastTime;


	// the ports
	Dataflow::PushSupplier< Measurement::Vector3D > m_acc_OutPort;
	Dataflow::PushSupplier< Measurement::RotationVelocity > m_gyro_OutPort;
	Dataflow::PushSupplier< Measurement::Vector3D > m_mag_OutPort;

	Dataflow::PushSupplier< Measurement::Rotation > m_gyro_outPortRAW;
  
};



	VectorNAVSensor::VectorNAVSensor(const std::string& sName, boost::shared_ptr< Graph::UTQLSubgraph > subgraph)
	: Dataflow::Component(sName)
	, m_timeOffset(0)
	, m_lastTime(-1e10)
	, m_bStop(true)
	, m_acc_OutPort("acc_OutPort", *this)
	, m_gyro_OutPort("gyro_OutPort", *this)
	, m_mag_OutPort("mag_OutPort", *this)
	, m_gyro_outPortRAW("gyro_outPortRAW", *this)
{
	subgraph->m_DataflowAttributes.getAttributeData("timeOffset", m_timeOffset);
	subgraph->m_DataflowAttributes.getAttributeData("frequency", freq);
	subgraph->m_DataflowAttributes.getAttributeData("baudrate", baudRate);
	portName = subgraph->m_DataflowAttributes.getAttributeString("port");
	
	
				
}



VectorNAVSensor::~VectorNAVSensor()
{
	
}


void VectorNAVSensor::start()
{
	if ( !m_running ) {
		vs.connect(portName, baudRate);	
		vs.writeAsyncDataOutputFrequency(freq);
		
		LOG4CPP_INFO(logger,  vs.readAsyncDataOutputFrequency());
		//ez = EzAsyncData::connect(portName, baudRate);
		m_running = true;
		m_bStop = false;
		m_pThread.reset(new boost::thread(boost::bind(&VectorNAVSensor::startCapturing, this)));
	}
	Component::start();
}

void VectorNAVSensor::startCapturing()
{	
	while (m_running)
	{

		reg = vs.readQuaternionMagneticAccelerationAndAngularRates();
		handleImuMessage(reg);
		//	 cd = ez->currentData();
		
		//LOG4CPP_INFO(logger, cd.acceleration().x);
		//boost::this_thread::sleep_for(boost::chrono::milliseconds(20));
	}
}

void VectorNAVSensor::stop()
{
	if (m_running)
	{
		m_running = false;
		m_bStop = true;
		//vs.unregisterAsyncPacketReceivedHandler();
		vs.disconnect();
		
		if (m_pThread)
		{
			m_pThread->join();
		}
	}
	Component::stop();
}

void VectorNAVSensor::handleImuMessage(QuaternionMagneticAccelerationAndAngularRatesRegister reg){

	Measurement::Timestamp ts = Measurement::now();// ins->timeOfWeek;
	m_acc_OutPort.send(Measurement::Vector3D(ts, Math::Vector3d(reg.accel.x, reg.accel.y, reg.accel.z)));
	m_gyro_OutPort.send(Measurement::RotationVelocity(ts, Math::RotationVelocity(reg.gyro.x, reg.gyro.y, reg.gyro.z)));
	m_mag_OutPort.send(Measurement::Vector3D(ts, Math::Vector3d(reg.mag.x, reg.mag.y, reg.mag.z)));
}

} } // namespace Ubitrack::Components

UBITRACK_REGISTER_COMPONENT( Dataflow::ComponentFactory* const cf ) {
	cf->registerComponent< Ubitrack::Drivers::VectorNAVSensor >("VectorNAVSensor");

}