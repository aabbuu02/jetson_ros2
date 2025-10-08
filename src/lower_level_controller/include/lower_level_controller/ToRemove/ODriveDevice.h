#ifndef ODRIVE_DEVICE_H
#define ODRIVE_DEVICE_H

#include <ros/ros.h>
#include <ros/time.h>
#include <iostream>
#include "CommunicationSerial.h"



/* Definitions */
#define START_BYTE 1
#define NL_LEN 0
#define PR_WRITE true
#define PR_READ false
#define AXIS_COMMAND true
#define SYS_COMMAND false
#define WAIT_TO_NEXT_COMMAND 500          // in micro-seconds
#define STATE_RQST_TIMEOUT (10 * 1000000) // 10 seconds, in micro-seconds
#define STATE_RQST_COUNTS (STATE_RQST_TIMEOUT / WAIT_TO_NEXT_COMMAND)
#define STATE_RQST_WAIT true
#define STATE_RQST_NO_WAIT false

/* Properties that can be Read / Written to O-Drive */
#define ODRV_SERIAL_NUMBER "serial_number"
#define ODRV_CURRENT_LIM_RW "motor.config.current_lim"
#define ODRV_ENCODER_IS_READY_RW "encoder.is_ready"
#define ODRV_MEAS_CURRENT_R "motor.current_control.Iq_measured"
#define ODRV_POSITION_RW "controller.pos_setpoint"
#define ODRV_VELOCITY "controller.input_vel"
#define ODRV_RAMP_VELOCITY "controller.vel_ramp_target"
#define ODRV_RAMP_ENABLE "controller.vel_ramp_enable"
#define ODRV_POS_ERROR_R "controller.error"
#define ODRV_AXIS_ERROR_W "error 0"
#define ODRV_MOTOR_ERROR_w "motor.error"
#define ODRV_ENCODER_ERROR_W "encoder.error"
#define ODRV_WATCH_DOG_TIMER_W "u"

#define ODRV_VELOCITY_RW "controller.config.vel_limit"
#define ODRV_CURRENT_VELOCITY_R "encoder.vel_estimate"
#define ODRV_ENCODER_CPR_RW "encoder.config.cpr"
#define ODRV_SPIN_MOTOR
#define ODRV_CURRENT_POSITION_R "encoder.pos_estimate_counts"
#define ODRV_POS_GAIN_RW "controller.config.pos_gain"
#define ODRV_VEL_GAIN_RW "controller.config.vel_gain"
#define ODRV_VEL_INTEGRATOR_GAIN_RW "controller.config.vel_integrator_gain"
#define ODRV_DRIVE_TEMP_R "motor.get_inverter_temp()"
#define ODRV_MOTOR_TEMP_R ""
#define ODRV_DRIVE_FAULT_R "motor.gate_driver.drv_fault"
#define ODRV_INDEX_FOUND_R "encoder.index_found"
#define ODRV_AXIS_ERROR_R "error"
#define ODRV_SAVE_CONFIG "ss"
#define ODRV_ERASE_CONFIG "se"
#define ODRV_SYSTEM_REBOOT "sr"
#define ODRV_REQUEST_STATE "requested_state"
#define ODRV_CURRENT_STATE "current_state"
#define ODRV_VOLTAGE "vbus_voltage"

#define AXIS_STATE_UNDEFINED 0                  //<! will fall through to idle
#define AXIS_STATE_IDLE 1                       //<! disable PWM and do nothing
#define AXIS_STATE_STARTUP_SEQUENCE 2           //<! the actual sequence is defined by the config.startup_... flags
#define AXIS_STATE_FULL_CALIBRATION_SEQUENCE 3  //<! run all calibration procedures, then idle
#define AXIS_STATE_MOTOR_CALIBRATION 4          //<! run motor calibration
#define AXIS_STATE_SENSORLESS_CONTROL 5         //<! run sensorless control
#define AXIS_STATE_ENCODER_INDEX_SEARCH 6       //<! run encoder index search
#define AXIS_STATE_ENCODER_OFFSET_CALIBRATION 7 //<! run encoder offset calibration
#define AXIS_STATE_CLOSED_LOOP_CONTROL 8        //<! run closed loop control

class ODriveDevice
{

public:
    ODriveDevice(CommunicationSerial &serial, int motor_number);

    /*Member Functions*/

    void writeCommand(std::string property, int motor_id);
    void writeProperty(std::string property);
    void writeProperty(std::string property, double value);
    void writeProperty(std::string property, long value);
    void readProperty(std::string property, double &value);
    void readProperty(std::string property, long &value,std::string useCase);
    void sendCommand(std::string property, bool read_write, bool axis);
    bool waitToIdle();
    bool runState(long requested_state, bool wait_for_idle);


    long getError();





private:
    /*Member Functions*/

    /*Member variables*/

    int motorNumber;

    CommunicationSerial serial;
};

#endif
