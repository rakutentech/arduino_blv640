/*
BLV640_Library
Copyright (C) 2019 Rakuten
julian.desvignes@rakuten.com
wilson.a.colin@rakuten.com

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 3 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program; if not, write to the Free Software Foundation,
Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

#ifndef BLV_COMM_H
#define BLV_COMM_H

#ifndef SYMBOL_TIMEOUT
#define SYMBOL_TIMEOUT 100 //In ms
#endif

#define MIN(a,b) (((a)<(b))?(a):(b))

#include <string.h>
#include "modbus.h"

namespace blv_commands{

enum class Acceleration: uint8_t {Acc = 0x00, Dec = 0x80};

enum class Alerts: uint16_t {Alarm = 0x0080, Warning = 0x0096, Comm_Error = 0x00AC};

enum class ClearAlerts: uint8_t {Alarm = 0x84, Warning = 0x86, Comm_Error = 0x88};

enum class Command: uint8_t {Input = 0xC8, Feedback = 0xCE};

enum class Direction: uint8_t {CCW = 0x00, CW = 0x01};

enum class EBrake: uint8_t {Lock = 0b00000000, Unlock = 0b10000000};

enum class Mode: uint8_t {Mode_0 = 0b00000000, Mode_1 = 0b00000001,
                          Mode_2 = 0b00000010, Mode_3 = 0b00000011,
                          Mode_4 = 0b00000100, Mode_5 = 0b00000101,
                          Mode_6 = 0b00000110, Mode_7 = 0b00000111,};

enum class Rotation: uint8_t {FWD = 0b00000000, REV = 0b00100000};

enum class StopMode: uint8_t {Instant = 0b00001000, Soft = 0b00010000};

enum class WireMode: uint8_t {Mode_2_Wire = 0x00, Mode_3_Wire = 0x01};


class blv_comm
{
    //------------------------------------------------------//
    // 31 ids are available. However the id 0 is reserved   //
    // to broadcast mode. Any messages sent to this id will //
    // be received by every single slave. No response will  //
    // be sent back.                                        //
    //------------------------------------------------------//

private:
    uint8_t m_message[22];
    uint8_t m_response[40];
    uint8_t m_operation = 0;        // Driver Input Command p.19

    static void defaultPrint(const char* message_) {
        Serial.print(message_);
    }

    void (*m_printFunction)(const char*) = &defaultPrint;

    // 0 : Everything ; 1 : Errors only
    uint8_t m_logLevel = 0;

    // For sprintf
    char m_printOut[256];

    //------------------------------------------------------//
    // This function handles the communication process.     //
    // It send the message m_message and wait for the       //
    // response (except if the message id is 0). The CRC16  //
    // is added to the message in this function.            //
    //                                                      //
    // msg_ : Number of bytes to send in m_message. It must //
    // not include the CRC16.                               //
    // resp_: Expected size of the response with the CRC16  //
    // included.                                            //
    // returns true if the message was correctly send and   //
    // the response not corrupted. Else false.              //
    //------------------------------------------------------//
    bool m_communicate(size_t msg_, size_t resp_);

    //------------------------------------------------------//
    // Verify if the response of the slave match the        //
    // message sent.                                        //
    //                                                      //
    // size_: Expected size of the message received in      //
    //        bytes                                         // 
    // Return true if the messages match sent, else false   //
    //------------------------------------------------------//
    bool m_check_response(size_t size_);
    
    //------------------------------------------------------//
    // Source: Oriental Motor, User Manual - RS485. p.26    //
    //  Query: Motor Wire Input Mode Selection              //
    //  Registers: 0x1041 (lower)                           //
    //  Function code: 0x06                                 //
    //  Data: 0x00 (2-wire) - 0x01 (3-wire)                 //
    //                                                      //
    // id_ : id of the slave concerned                      //
    // val_: desired mode                                   //
    // Return true if message successfully sent, else false //
    //------------------------------------------------------//
    bool m_config_wire_input_mode(uint8_t id_, WireMode val_ = WireMode::Mode_3_Wire);

    //------------------------------------------------------//
    // Send m_operation attribute to the Slave. This bytes  //
    // set the parameters of the remote NET-IN(0 to 15).    //
    //                                                      //
    // Source: Oriental Motor, User Manual - RS485. p.19    //
    //  Query: Create Operation Command Message             //
    //  Registers: 0x007D (lower)                           //
    //  Function code: 0x06                                 //
    //  Data: cf p.19                                       //
    //                                                      //
    // id_ : slave id                                       //
    // Return true if message successfully sent, else false //
    //------------------------------------------------------//
    bool m_send_operation_message(uint8_t id_);


public:
    //------------------------------------------------------//
    // Source: Oriental Motor, User Manual - Basic Function //
    //                                                      //
    // 1. Set the slaves into 3_Wire Mode - p.39            //
    // 2. Execute the recalculation for each slave          //
    //                                                      //
    // nbr_slave_: number of slaves                         //
    // slaves_: pointer on an array of slave ids            // 
    // printFunction_: print function for feedback          // 
    //------------------------------------------------------//
    blv_comm(size_t nbr_slave_, uint8_t* slaves_, void (*printFunction_)(const char*) = NULL);
    
    blv_comm(void) = default;

    void setLogLevel(uint8_t logLevel_) {
        m_logLevel = logLevel_;
    }


    /********************************************************
     *                  CONFIG FUNCTIONS                    *
     ********************************************************/

    //------------------------------------------------------//
    // Source: Oriental Motor, User Manual - RS485. p.26    //
    //  Query: Motor Direction Selection                    //
    //  Registers: 0x0385 (lower)                           //
    //  Function code: 0x06                                 //
    //  Data : 0x00 (CCW) - 0x01 (CW)                       //
    //                                                      //
    // id_ : slave id                                       //
    // val_: desired direction                              //
    // Return true if message successfully sent, else false //
    //------------------------------------------------------//
    bool config_Fwd_Direction(uint8_t id_, Direction val_ = Direction::CCW);

    //------------------------------------------------------//
    // Source: Oriental Motor, User Manual - RS485. p.19    //
    // Query: Action of Electromag Brake when motor stop    //
    // Register: 0x007D (lower)                             //
    // Function code: 0x06                                  //
    // Data: cf p.19                                        //
    //                                                      //
    // id_ : slave id                                       //
    // mode_ : 0b00000000 (Lock) - 0b100000000 (Unlock)     //
    // Return true if message successfully sent, else false //
    //------------------------------------------------------//
    bool electromagnetic_brake_action(uint8_t id_, EBrake mode_ = EBrake::Lock);
    
    //------------------------------------------------------//
    // Source: Oriental Motor, User Manual - RS485. p.19    //
    //         User Manual - Basic Function p.39            //
    // Query: Select Operating Mode                         //
    // Register: 0x007D (lower)                             //
    // Function code: 0x06                                  //
    // Data: cf p.19                                        //
    //                                                      //
    // id_ : slave id                                       //
    // mode_: desired mode. For digital control, the mode   //
    //        must be in the range [2; 7] (cf p.39)         //
    // Return true if message successfully sent, else false //
    //------------------------------------------------------//
    bool select_mode(uint8_t id_, Mode mode_ = Mode::Mode_2);

    //------------------------------------------------------//
    // Function to set the torque of the motor in a specific//
    // mode. The Torque limitation goes from 0% to 200% of  //
    // the rated torque.                                    //
    //                                                      //
    // Source: Oriental Motor, User Manual - RS485. p.25    //
    // Query: Set the torque limitation of the Motor.       //
    // Register: 0x0700 ~ 0x070F                            //
    // Function code: 0x06                                  //
    // Data: Torque in range [0; 200]                       //
    //                                                      //
    // id_ : slave id                                       //
    // torque_ : range from 0 to 200 - resolution = 1       //
    // mode_: desired mode. For digital control, the mode   //
    //        must be in the range [2; 7] (cf p.39)         //
    // Return true if message successfully sent, else false //
    //------------------------------------------------------//
    bool set_torque_limit(uint8_t id_, uint16_t torque_, Mode mode_ = Mode::Mode_2);

    //------------------------------------------------------//
    // Executes the new config set using the methods above  //
    //                                                      //
    // Source: Oriental Motor, User Manual - RS485. p.??    //
    // Query: Execute the config                            //
    // Register: 0x018D                                     //
    // Function code: 0x06                                  //
    // Data: 0x01                                           //
    //                                                      //
    // id_ : slave id                                       //
    //------------------------------------------------------//
    bool exec_config(uint8_t id_);


    /********************************************************
     *                   WRITE FUNCTIONS                    *
     ********************************************************/

    //------------------------------------------------------//
    // Function to set the speed of the motor in a specific //
    // mode. The motor start spining from 80rpm, under it   //
    // will be considered as 0rpm.                          //
    //                                                      //
    // Source: Oriental Motor, User Manual - RS485. p.25    //
    // Query: Set speed of the Motor in rpm.                //
    // Register: 0x0480 ~ 0x048F                            //
    // Function code: 0x06                                  //
    // Data: speed in range {0,[80 4000]}                   //
    //                                                      //
    // id_ : slave id                                       //
    // speed_: range from 0 to 4000 rpm - resolution = 1 rpm//
    // mode_: desired mode. For digital control, the mode   //
    //        must be in the range [2; 7] (cf p.39)         //
    // Return true if message successfully sent, else false //
    //------------------------------------------------------//
    bool set_speed(uint8_t id_, uint16_t speed_, Mode mode_ = Mode::Mode_2);
    
    //------------------------------------------------------//
    // Function to set the accelerations of the motor in a  //
    // specific mode. The acceleration time goes from 0.2s  //
    // to 15s with  0.1 resolution.                         //
    //                                                      //
    // Source: Oriental Motor, User Manual - RS485. p.25    //
    // Query: Set acceleration/deceleration time of the     //
    //        Motor                                         //
    // Register: 0x0600 ~ 0x060F (Acceleration)             //
    // Function code: 0x06                                  //
    // Data: acc in range [2, 150]                          //
    //                                                      //
    // id_ : slave id                                       //
    // acc_: range from 2 to 150 - resolution = 1           //
    // type_: select between set acc or deceleration        //
    // mode_: desired mode. For digital control, the mode   //
    //        must be in the range [2; 7] (cf p.39)         //
    // Return true if message successfully sent, else false //
    //------------------------------------------------------//
    bool set_acc(uint8_t id_, uint16_t acc_, Acceleration type_, Mode mode_ = Mode::Mode_2);
    
    bool set_acc(uint8_t id_, uint16_t acc_, Mode mode_ = Mode::Mode_2);
    
    //------------------------------------------------------//
    // Set the direction in which the motor should turn.    //
    // Forward or reverse (backward)                        //
    //                                                      //
    // Source: Oriental Motor, User Manual - RS485. p.19    //
    // Query: Set Motor Forward Direction                   //
    // Register: 0x007D (lower)                             //
    // Function code: 0x06                                  //
    // Data: False (Fwd) - True (Rev)                       //
    //                                                      //
    // id_ : slave id                                       //
    // val_: select the direction                           //
    // Return true if message successfully sent, else false //
    //------------------------------------------------------//
    bool set_Rotation_Direction(uint8_t id_, Rotation val_ = Rotation::FWD);

    //------------------------------------------------------//
    // Make the motor spin                                  //
    //                                                      //
    // Source: Oriental Motor, User Manual - RS485. p.19    //
    // Query: Start Motor Rotation                          //
    // Register: 0x007D (lower)                             //
    // Function code: 0x06                                  //
    // Data: 0b00011000                                     //
    // Resets the speed to 0                                //
    //                                                      //
    // id_ : slave id                                       //
    // Return true if message successfully sent, else false //
    //------------------------------------------------------//
    bool start_rotation(uint8_t id_);

    //------------------------------------------------------//
    // Make the motor stop. Can be an instant stop or a     //
    // decelerating stop                                    //
    //                                                      //
    // Source: Oriental Motor, User Manual - RS485. p.19    //
    // Query: Smooth/Instant Brake                          //
    // Register: 0x007D (lower)                             //
    // Function code: 0x06                                  //
    // Data: 0b00001000(Soft) - 0b00010000 (Instant)        //
    //                                                      //
    // id_ : slave id                                       //
    // mode_: Soft - Instant                                //
    // Return true if message successfully sent, else false //
    //------------------------------------------------------//
    bool stop_rotation(uint8_t id_, StopMode mode_ = StopMode::Soft);




    /********************************************************
     *                MAINTENANCE FUNCTIONS                 *
     ********************************************************/

    //------------------------------------------------------//
    // Source: Oriental Motor, User Manual - RS485, p.17    //
    // Query: Diagnosis                                     //
    // Function code: 0x08h                                 //
    //                                                      //
    // id_ : slave id                                       // 
    // Return true if message successfully sent, else false //
    //------------------------------------------------------//
    bool diagnose(uint8_t id_);

    //------------------------------------------------------//
    // Source: Oriental Motor, User Manual - RS485, p.17    //
    // Query: Reset Alarms                                  //
    // Function code: 0x06h                                 //
    // Register: 0x0180 (upper) - 0x0181 (lower)            //
    //                                                      //
    // id_ : slave id                                       // 
    // Return true if message successfully sent, else false //
    //------------------------------------------------------//
    bool reset_alarm(uint8_t id_);

    //------------------------------------------------------//
    // Source: Oriental Motor, User Manual - RS485, p.17    //
    // Query: Clear Alarms - Warnings - Comm Errors         //
    // Function code: 0x06h                                 //
    // Register: 0x0184 (upper) - 0x0189 (lower)            //
    //                                                      //
    // id_ : slave id                                       // 
    // type_: Alarm - Warning - Comm_Error                  // 
    // Return true if message successfully sent, else false //
    //------------------------------------------------------//
    bool clear_alert(uint8_t id_, ClearAlerts type_);

    //------------------------------------------------------//
    // Clear all the Alarms, Warnings and Coomunication     //
    // Errors at once.                                      //
    //                                                      //
    // Source: Oriental Motor, User Manual - RS485, p.17    //
    // Query: Clear Alarms - Warnings - Comm Errors         //
    // Function code: 0x10h                                 //
    // Register: 0x0184 (upper)                             //
    //                                                      //
    // id_ : slave id                                       //
    // Return true if message successfully sent, else false //
    //------------------------------------------------------//
    bool clear_all_alerts(uint8_t id_);

    //------------------------------------------------------//
    // Sends the given message to the motors                //
    //                                                      //
    //                                                      //
    // message_ : the message to send                       //
    // message_size_ : size of the message w/o the CRC16    //
    // response_size_ : size of the response w/o the CRC16  //
    // Return true if message successfully sent, else false //
    //------------------------------------------------------//
    bool send_debug(const uint8_t* message_, size_t message_size_, size_t response_size_);




    /********************************************************
     *                    READ FUNCTIONS                    *
     ********************************************************/

    //------------------------------------------------------//
    // Function to read multiple registers at once.         //
    //                                                      //
    // Source: Oriental Motor, User Manual - RS485. p.16    //
    // Query: Create a Reading Registers Message            //
    //                                                      //
    // id_ : slave id                                       //
    // register_: Address of the first register             //
    // nbr_reg_ : Number of registers to read               //
    // Return true if message successfully sent, else false //
    //------------------------------------------------------//
    bool reading_registers(uint8_t id_, uint16_t register_, uint8_t nbr_reg_);

    //------------------------------------------------------//
    // Function to read wether the input speed or the       //
    // feedback speed from the registers.                   //
    //                                                      //
    // Source: Oriental Motor, User Manual - RS485. p.25    //
    // Query: Get the Command Speed of the current mode     //
    // Register: 0x00C8 ~ 0x00CF                            //
    // Function code: 0x03                                  //
    //                                                      //
    // id_ : slave id                                       //
    // type_: Input - Feedback)                             //
    // Return: Speed (uint16_t)                             //
    //         If the response from the slave failed,       //
    //         return a speed > 4000 rpm                    //
    //------------------------------------------------------//
    int32_t get_speed(uint8_t id_, Command type_ = Command::Feedback);

    //------------------------------------------------------//
    // Function to read all the Errors trigerred on the     //
    // driver.                                              //
    //                                                      //
    // Source: Oriental Motor, User Manual - RS485. p.25    //
    // Query: Get the Alarms from 0 to i (with i ranging    //
    //        from 1 to 10)                                 //
    // Register: 0x0080 ~ 0x0095                            //
    // Function code: 0x03                                  //
    //                                                      //
    // id_ : slave id                                       //
    // nbr_ : range from 1 to 10                            //
    // type_ : Alarm - Warning - Comm_Error                 //
    // Return true if message successfully sent, else false //
    //------------------------------------------------------//
    bool get_alerts(uint8_t id_, uint8_t nbr_, Alerts type_ = Alerts::Alarm);
    

}; // Class blv_comm
}  // Namespace blv_commands

#include "blv_comm.hxx"

#endif // BLV_COMM_H
