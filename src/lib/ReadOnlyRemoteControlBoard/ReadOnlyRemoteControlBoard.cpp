/*
* Copyright (C) 2017 iCub Facility - Istituto Italiano di Tecnologia
* Author: Francesco Romano
* CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
*/

#include "ReadOnlyRemoteControlBoard.h"

#include <yarp/os/Time.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/LogStream.h>

#include <yarp/sig/Vector.h>


#include <algorithm>
#include <cassert>

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;

#ifndef DOXYGEN_SHOULD_SKIP_THIS

const double TIMEOUT=0.5;

#endif /*DOXYGEN_SHOULD_SKIP_THIS*/


#if defined(_MSC_VER) && !defined(YARP_NO_DEPRECATED) // since YARP 2.3.65
// A class implementing setXxxxxMode() causes a warning on MSVC
YARP_WARNING_PUSH
YARP_DISABLE_DEPRECATED_WARNING
#endif


namespace yarp {
namespace dev {

    ReadOnlyRemoteControlBoard::ReadOnlyRemoteControlBoard()
    : m_numberOfJoints(0) {}

    /**
    * Destructor.
    */
    ReadOnlyRemoteControlBoard::~ReadOnlyRemoteControlBoard()
    {
    }

    bool ReadOnlyRemoteControlBoard::getSingleData(int field, int j, double* val)
    {
        double localArrivalTime = 0.0;

        m_extendedPortMutex.wait();
        bool ret = m_extendedIntputStatePort.getLastSingle(j, field, val, m_lastStamp, localArrivalTime);
        m_extendedPortMutex.post();

        if ((Time::now() - localArrivalTime) > TIMEOUT) {
            return false;
        }

        return ret;
    }

    bool ReadOnlyRemoteControlBoard::getVectorData(int field, double* val)
    {
        double localArrivalTime = 0.0;

        m_extendedPortMutex.wait();
        bool ret = m_extendedIntputStatePort.getLastVector(field, val, m_lastStamp, localArrivalTime);
        m_extendedPortMutex.post();

        if ((Time::now() - localArrivalTime) > TIMEOUT) {
            return false;
        }

        return ret;
    }

    bool ReadOnlyRemoteControlBoard::getTimedSingleData(int field, int j, double* val, double* time)
    {
        double localArrivalTime = 0.0;

        m_extendedPortMutex.wait();
        bool ret = m_extendedIntputStatePort.getLastSingle(j, field, val, m_lastStamp, localArrivalTime);
        *time = m_lastStamp.getTime();
        m_extendedPortMutex.post();

        if ((Time::now() - localArrivalTime) > TIMEOUT) {
            return false;
        }

        return ret;
    }

    bool ReadOnlyRemoteControlBoard::getTimedVectorData(int field, double* val, double* times)
    {
        double localArrivalTime = 0.0;

        m_extendedPortMutex.wait();
        bool ret = m_extendedIntputStatePort.getLastVector(field, val, m_lastStamp, localArrivalTime);
        std::fill_n(times, m_numberOfJoints, m_lastStamp.getTime());
        m_extendedPortMutex.post();

        if ((Time::now() - localArrivalTime) > TIMEOUT) {
            return false;
        }

        return ret;
    }

    /**
    * Default open.
    * @return always true.
    */
    bool ReadOnlyRemoteControlBoard::open() {
        return true;
    }

    bool ReadOnlyRemoteControlBoard::open(Searchable& config) {
        std::string remote = config.find("remote").asString();
        std::string local = config.find("local").asString();

        if (local.empty()) {
            yError("Problem connecting to remote controlboard, 'local' port prefix not given");
            return false;
        }

        if (remote.empty()) {
            yError("Problem connecting to remote controlboard, 'remote' port name not given");
            return false;
        }

        std::string carrier = config.check("carrier", Value("udp"), "default carrier for streaming robot state").asString();

        bool portProblem = false;
        if (!m_extendedIntputStatePort.open(local + "/stateExt:i")) {
            portProblem = true;
        }

        if (!portProblem) {
            m_extendedIntputStatePort.useCallback();
        }


        bool connectionProblem = false;
        if (!portProblem) {
            bool ok = false;
            ok = Network::connect(remote + "/stateExt:o", m_extendedIntputStatePort.getName(), carrier);
            if (!ok) {
                connectionProblem = true;
                yError("*** Extended port %s was not found on the controlBoardWrapper I'm connecting to.", (remote + "/stateExt:o").c_str());
            }
        }

        if (connectionProblem || portProblem) {
            m_extendedIntputStatePort.close();
            return false;
        }

        Value &axesDescription = config.find("axesDescription");
        if (axesDescription.isNull() || !axesDescription.isList()) {
            yError("*** Option 'axesDescription' not found or malformed.");
            m_extendedIntputStatePort.close();
            return false;
        }

        Bottle *axesDescriptionList = axesDescription.asList();
        m_numberOfJoints = axesDescriptionList->size();
        m_extendedIntputStatePort.init(m_numberOfJoints);
        m_axes.reserve(m_numberOfJoints);

        for (int index = 0; index < m_numberOfJoints; ++index) {
            const Value& axis = axesDescriptionList->get(index);
            if (axis.isNull() || !axis.isList() || axis.asList()->size() != 2) {
                yError("*** Option 'axesDescription' malformed at index %d.", index);
                m_extendedIntputStatePort.close();
                return false;
            }
            std::string axisName = axis.asList()->get(0).asString();
            JointTypeEnum axisVocab = static_cast<JointTypeEnum>(axis.asList()->get(1).asVocab());
            m_axes.push_back(std::pair<std::string, yarp::dev::JointTypeEnum>(axisName, axisVocab));
        }

        return true;
    }

    /**
    * Close the device driver and stop the port connections.
    * @return true/false on success/failure.
    */
    bool ReadOnlyRemoteControlBoard::close() {
        m_extendedIntputStatePort.close();
        return true;
    }

    /* IEncoder */
    bool ReadOnlyRemoteControlBoard::getAxes(int *ax)
    {
        if (!ax) return false;
        m_extendedPortMutex.wait();
        bool ret = m_extendedIntputStatePort.getJointPositionSize(*ax); //It is possible that the joint size provided in the configuration do not match what it is read from stateExt
        m_extendedPortMutex.post();
        return ret;
    }

    bool ReadOnlyRemoteControlBoard::getEncoder(int j, double *v)
    {
        if (j < 0 || j >= m_numberOfJoints || !v) return false;

        return getSingleData(VOCAB_ENCODER, j, v);
    }

    bool ReadOnlyRemoteControlBoard::getEncoderTimed(int j, double *v, double *t)
    {
        if (j < 0 || j >= m_numberOfJoints || !v || !t) return false;

        return getTimedSingleData(VOCAB_ENCODER, j, v, t);
    }


    bool ReadOnlyRemoteControlBoard::getEncoders(double *encs) {
        if (!encs) return false;

        return getVectorData(VOCAB_ENCODERS, encs);
    }

    bool ReadOnlyRemoteControlBoard::getEncodersTimed(double *encs, double *ts) {
        if (!encs || !ts) return false;

        return getTimedVectorData(VOCAB_ENCODERS, encs, ts);
    }

    bool ReadOnlyRemoteControlBoard::getEncoderSpeed(int j, double *sp)
    {
        if (j < 0 || j >= m_numberOfJoints || !sp) return false;

        return getSingleData(VOCAB_ENCODER_SPEED, j, sp);
    }


    bool ReadOnlyRemoteControlBoard::getEncoderSpeeds(double *spds)
    {
        if (!spds) return false;

        return getVectorData(VOCAB_ENCODER_SPEEDS, spds);
    }

    bool ReadOnlyRemoteControlBoard::getEncoderAcceleration(int j, double *acc)
    {
        if (j < 0 || j >= m_numberOfJoints || !acc) return false;

        return getSingleData(VOCAB_ENCODER_ACCELERATION, j, acc);
    }


    bool ReadOnlyRemoteControlBoard::getEncoderAccelerations(double *accs)
    {
        if (!accs) return false;

        return getVectorData(VOCAB_ENCODER_ACCELERATIONS, accs);
    }

    /* IAxisInfo */
    bool ReadOnlyRemoteControlBoard::getAxisName(int j, std::string& name) {
        if (j < 0 || j >= m_numberOfJoints) return false;
        name = m_axes[j].first;
        return true;
    }

    bool ReadOnlyRemoteControlBoard::getJointType(int j, yarp::dev::JointTypeEnum& type) {
        if (j < 0 || j >= m_numberOfJoints) return false;
        type = m_axes[j].second;
        return true;
    }

    /* IMotorEncoders */
    bool ReadOnlyRemoteControlBoard::getNumberOfMotorEncoders(int *num)
    {
        if (!num) return false;
        *num = m_numberOfJoints;
        return true;
    }

    bool ReadOnlyRemoteControlBoard::getMotorEncoder(int m, double *v)
    {
        if (m < 0 || m >= m_numberOfJoints || !v) return false;

        return getSingleData(VOCAB_MOTOR_ENCODER, m, v);
    }

    bool ReadOnlyRemoteControlBoard::getMotorEncoderTimed(int m, double *encs, double *time)
    {
        if (m < 0 || m >= m_numberOfJoints || !encs || !time) return false;

        return getTimedSingleData(VOCAB_MOTOR_ENCODER, m, encs, time);
    }


    bool ReadOnlyRemoteControlBoard::getMotorEncoders(double *encs) {
        if (!encs) return false;

        return getVectorData(VOCAB_MOTOR_ENCODERS, encs);
    }

    bool ReadOnlyRemoteControlBoard::getMotorEncodersTimed(double *encs, double *time) {
        if (!encs || !time) return false;

        return getTimedVectorData(VOCAB_MOTOR_ENCODERS, encs, time);
    }

    bool ReadOnlyRemoteControlBoard::getMotorEncoderSpeed(int m, double *sp)
    {
        if (m < 0 || m >= m_numberOfJoints || !sp) return false;

        return getSingleData(VOCAB_MOTOR_ENCODER_SPEED, m, sp);
    }


    bool ReadOnlyRemoteControlBoard::getMotorEncoderSpeeds(double *spds)
    {
        if (!spds) return false;

        return getVectorData(VOCAB_MOTOR_ENCODER_SPEEDS, spds);
    }

    bool ReadOnlyRemoteControlBoard::getMotorEncoderAcceleration(int m, double *acc)
    {
        if (m < 0 || m >= m_numberOfJoints || !acc) return false;

        return getSingleData(VOCAB_MOTOR_ENCODER_ACCELERATION, m, acc);
    }


    bool ReadOnlyRemoteControlBoard::getMotorEncoderAccelerations(double *accs)
    {
        if (!accs) return false;

        return getVectorData(VOCAB_MOTOR_ENCODER_ACCELERATIONS, accs);
    }


    /* ITorqueControl */
    bool ReadOnlyRemoteControlBoard::getTorque(int j, double *t)
    {
        if (j < 0 || j >= m_numberOfJoints || !t) return false;

        return getSingleData(VOCAB_TRQ, j, t);
    }

    bool ReadOnlyRemoteControlBoard::getTorques(double *t)
    {
        if (!t) return false;

        return getVectorData(VOCAB_TRQS, t);
    }

    /* IPWMControl */
    bool ReadOnlyRemoteControlBoard::getNumberOfMotors(int *number)
    {
        if (!number) return false;
        *number = m_numberOfJoints;
        return true;
    }

    bool ReadOnlyRemoteControlBoard::getDutyCycle(int m, double *val)
    {
        if (m < 0 || m >= m_numberOfJoints || !val) return false;

        return getSingleData(VOCAB_PWMCONTROL_PWM_OUTPUT, m, val);
    }

    bool ReadOnlyRemoteControlBoard::getDutyCycles(double *vals)
    {
        if (!vals) return false;

        return getVectorData(VOCAB_PWMCONTROL_PWM_OUTPUTS, vals);
    }

    /* IAmplifierControl */
    bool ReadOnlyRemoteControlBoard::getCurrent(int j, double* val)
    {
        if (j < 0 || j >= m_numberOfJoints || !val) return false;

        return getSingleData(VOCAB_AMP_CURRENT, j, val);
    }

    bool ReadOnlyRemoteControlBoard::getCurrents(double *vals)
    {
        if (!vals) return false;

        return getVectorData(VOCAB_AMP_CURRENTS, vals);
    }

}
}
