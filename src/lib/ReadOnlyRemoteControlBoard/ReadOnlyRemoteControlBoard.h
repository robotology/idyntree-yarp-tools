/*
* Copyright (C) 2017 iCub Facility - Istituto Italiano di Tecnologia
* Author: Francesco Romano
* CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
*/

#ifndef YARP_DEV_READONLYREMOTECONTROLBOARD_READONLYREMOTECONTROLBOARD_H
#define YARP_DEV_READONLYREMOTECONTROLBOARD_READONLYREMOTECONTROLBOARD_H

#include <yarp/dev/IEncodersTimed.h>
#include <yarp/dev/IMotorEncoders.h>
#include <yarp/dev/ITorqueControl.h>
#include <yarp/dev/IPWMControl.h>
#include <yarp/dev/IAmplifierControl.h>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>

#include <yarp/os/Semaphore.h>

#include "stateExtendedReader.h"

#include <vector>
#include <iostream>
namespace yarp {
        namespace dev {
            class ReadOnlyRemoteControlBoard;

        }
}

/*!
 * @brief Readonly version of the remote control board.
 *
 * It opens only interfaces though for providing data, not for commanding the robot.
 * Information usually obtained though calls to the underlining device are expected to be
 * provided by the configuration parameters
 *
 *  Parameters required by this device are:
 * | Parameter name | SubParameter   | Type    | Units          | Default Value | Required                    | Description                                                       | Notes |
 * |:--------------:|:--------------:|:-------:|:--------------:|:-------------:|:--------------------------: |:-----------------------------------------------------------------:|:-----:|
 * | remote     |      -         | string  | -      |   -           | Yes     | Prefix of the remote port which this device will connect. The port opened will be "{remote}/stateExt:o" |  |
 * | local     |      -         | string  | -      |   -           | Yes     | Prefix of the ports opened by this device |  |
 * | carrier     |      -         | string  | -      |   udp           | No     | Protocol to be used during the connection. Default to udp |  |
 * | axesDescription     |      -         | vector of pairs of string and vocab  | -      |   -           | Yes     | Description of the axes managed by this control board. Each element is a pair: a string denoting the axis name and a vocab denoting the axis type (rotational, prismatic, etc). |  |
 *
 */
class yarp::dev::ReadOnlyRemoteControlBoard
: public yarp::dev::IEncodersTimed
, public yarp::dev::DeviceDriver
, public yarp::dev::IAxisInfo
, public yarp::dev::IMotorEncoders
, public yarp::dev::ITorqueControl
, public yarp::dev::IPWMControl
, public yarp::dev::IAmplifierControl
, public yarp::dev::IPositionControl
, public yarp::dev::IVelocityControl
, public yarp::dev::IControlMode
{

#ifndef DOXYGEN_SHOULD_SKIP_THIS

    // Buffer associated to the extendedOutputStatePort port; in this case we will use the type generated
    // from the YARP .thrift file
    StateExtendedInputPort m_extendedIntputStatePort;  // Buffered port storing new data
    yarp::os::Semaphore m_extendedPortMutex;

    std::vector<std::pair<std::string, yarp::dev::JointTypeEnum> > m_axes;

    mutable Stamp m_lastStamp;  //this is shared among all calls that read encoders
    // Semaphore mutex;
    int m_numberOfJoints;

#endif /*DOXYGEN_SHOULD_SKIP_THIS*/

public:
    /**
     * Constructor.
     */
    ReadOnlyRemoteControlBoard();

    /**
     * Destructor.
     */
    ~ReadOnlyRemoteControlBoard();

    virtual bool open();

    virtual bool open(yarp::os::Searchable& config);

    virtual bool close();

    bool getSingleData(int field, int j, double* val);
    bool getVectorData(int field, double* val);
    bool getTimedSingleData(int field, int j, double* val, double* time);
    bool getTimedVectorData(int field, double* val, double* time);

    /* IEncodersTimed */
    virtual bool getAxes(int *ax);
    virtual bool resetEncoder(int j) {return false;}
    virtual bool resetEncoders() {return false;}
    virtual bool setEncoder(int j, double val) {return false;}
    virtual bool setEncoders(const double *vals) {return false;}
    virtual bool getEncoder(int j, double *v);
    virtual bool getEncoderTimed(int j, double *v, double *t);
    virtual bool getEncoders(double *encs);
    virtual bool getEncodersTimed(double *encs, double *ts);
    virtual bool getEncoderSpeed(int j, double *sp);
    virtual bool getEncoderSpeeds(double *spds);
    virtual bool getEncoderAcceleration(int j, double *acc);
    virtual bool getEncoderAccelerations(double *accs);

    /* IAxisInfo */
    virtual bool getAxisName(int j, std::string &name);
    virtual bool getJointType(int j, yarp::dev::JointTypeEnum &type);

    /* IMotorEncoders */
    virtual bool getNumberOfMotorEncoders(int *num);
    virtual bool resetMotorEncoder(int m) {return false;}
    virtual bool resetMotorEncoders() {return false;}
    virtual bool setMotorEncoderCountsPerRevolution(int m, const double cpr) {return false;}
    virtual bool getMotorEncoderCountsPerRevolution(int m, double *cpr) {return false;}
    virtual bool setMotorEncoder(int m, const double val) {return false;}
    virtual bool setMotorEncoders(const double *vals) {return false;}
    virtual bool getMotorEncoder(int m, double *v);
    virtual bool getMotorEncoders(double *encs);
    virtual bool getMotorEncodersTimed(double *encs, double *time);
    virtual bool getMotorEncoderTimed(int m, double *encs, double *time);
    virtual bool getMotorEncoderSpeed(int m, double *sp);
    virtual bool getMotorEncoderSpeeds(double *spds);
    virtual bool getMotorEncoderAcceleration(int m, double *acc);
    virtual bool getMotorEncoderAccelerations(double *accs);

    /* ITorqueControl */
    virtual bool getRefTorques(double *t) {return false;}
    virtual bool getRefTorque(int j, double *t) {return false;}
    virtual bool setRefTorques(const double *t) {return false;}
    virtual bool setRefTorque(int j, double t) {return false;}
    virtual bool setRefTorques(const int n_joint, const int *joints, const double *t) {return false;}
    virtual bool getMotorTorqueParams(int j,  yarp::dev::MotorTorqueParameters *params) {return false;}
    virtual bool setMotorTorqueParams(int j,  const yarp::dev::MotorTorqueParameters params) {return false;}
    virtual bool getTorque(int j, double *t);
    virtual bool getTorques(double *t);
    virtual bool getTorqueRange(int j, double *min, double *max) {return false;}
    virtual bool getTorqueRanges(double *min, double *max) {return false;}


    /* IPWMControl */
    virtual bool getNumberOfMotors(int *number);
    virtual bool setRefDutyCycle(int m, double ref) {return false;}
    virtual bool setRefDutyCycles(const double *refs) {return false;}
    virtual bool getRefDutyCycle(int m, double *ref) {return false;}
    virtual bool getRefDutyCycles(double *refs) {return false;}
    virtual bool getDutyCycle(int m, double *val);
    virtual bool getDutyCycles(double *vals);

    /* IAmplifierControl */
    virtual bool enableAmp(int j) {return false;}
    virtual bool disableAmp(int j) {return false;}
    virtual bool getAmpStatus(int *st) {return false;}
    virtual bool getAmpStatus(int j, int *v) {return false;}
    virtual bool getCurrents(double *vals);
    virtual bool getCurrent(int j, double *val);
    virtual bool getMaxCurrent(int j, double *v) {return false;}
    virtual bool setMaxCurrent(int j, double v) {return false;}
    virtual bool getNominalCurrent(int m, double *val) {return false;}
    virtual bool setNominalCurrent(int m, const double val) { return false; }
    virtual bool getPeakCurrent(int m, double *val) {return false;}
    virtual bool setPeakCurrent(int m, const double val) {return false;}
    virtual bool getPWM(int j, double* val) {return false;}
    virtual bool getPWMLimit(int j, double* val) {return false;}
    virtual bool setPWMLimit(int j, const double val) {return false;}
    virtual bool getPowerSupplyVoltage(int j, double* val) {return false;}

    /* IPositionControl */
    bool positionMove(int j, double ref) {return false;}
    bool positionMove(const int n_joint, const int *joints, const double *refs) {return false;}
    bool positionMove(const double *refs) {return false;}
    bool getTargetPosition(const int joint, double *ref) {return false;}
    bool getTargetPositions(double *refs) {return false;}
    bool getTargetPositions(const int n_joint, const int *joints, double *refs) {return false;}
    bool relativeMove(int j, double delta) {return false;}
    bool relativeMove(const int n_joint, const int *joints, const double *refs) {return false;}
    bool relativeMove(const double *deltas) {return false;}
    bool checkMotionDone(int j, bool *flag) {return false;}
    bool checkMotionDone(const int n_joint, const int *joints, bool *flag) {return false;}
    bool checkMotionDone(bool *flag) {return false;}
    bool setRefSpeed(int j, double sp) {return false;}
    bool setRefSpeeds(const int n_joint, const int *joints, const double *spds) {return false;}
    bool setRefSpeeds(const double *spds) {return false;}
    bool setRefAcceleration(int j, double acc) {return false;}
    bool setRefAccelerations(const int n_joint, const int *joints, const double *accs) {return false;}
    bool setRefAccelerations(const double *accs) {return false;}
    bool getRefSpeed(int j, double *ref) {return false;}
    bool getRefSpeeds(const int n_joint, const int *joints, double *spds) {return false;}
    bool getRefSpeeds(double *spds) {return false;}
    bool getRefAcceleration(int j, double *acc) {return false;}
    bool getRefAccelerations(const int n_joint, const int *joints, double *accs) {return false;}
    bool getRefAccelerations(double *accs) {return false;}
    bool stop(int j) {return false;};
    bool stop(const int len, const int *val1) {return false;}
    bool stop() {return false;}

    /* IVelocityControl */
    bool velocityMove(int j, double v) {return false;}
    bool velocityMove(const double *v) {return false;}
    bool velocityMove(const int n_joint, const int *joints, const double *spds) {return false;}
    bool getRefVelocity(const int joint, double* vel) {return false;}
    bool getRefVelocities(double* vels) {return false;}
    bool getRefVelocities(const int n_joint, const int* joints, double* vels) {return false;}

    /* IControlMode */
    bool getControlMode(int j, int *mode) {return false;}
    bool getControlModes(int *modes) {return false;}
    bool getControlModes(const int n_joint, const int *joints, int *modes) {return false;}
    bool setControlMode(const int j, const int mode) {return false;}
    bool setControlModes(const int n_joint, const int *joints, int *modes) {return false;}
    bool setControlModes(int *modes) {return false;}


};

#if defined(_MSC_VER) && !defined(YARP_NO_DEPRECATED) // since YARP 2.3.65
YARP_WARNING_POP
#endif


#endif // YARP_DEV_READONLYREMOTECONTROLBOARD_READONLYREMOTECONTROLBOARD_H
