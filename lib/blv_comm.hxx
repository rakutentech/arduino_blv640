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

bool blv_commands::blv_comm::m_communicate(size_t msg_, size_t resp_){
    /**************************************/
    /*      Add CRC16 to the message      */
    /**************************************/
    uint16_t check_sum = modbus::CRC16(m_message, msg_);
    m_message[msg_] = static_cast<uint8_t>((check_sum & 0x00FF));          // Lower
    m_message[msg_+1] = static_cast<uint8_t>(((check_sum & 0xFF00) >> 8)); // Upper

    /**************************************/
    /*        Communicate With BLV        */
    /**************************************/
    // Send Message to Slave
    modbus::send(m_message, msg_+2);

    if(m_logLevel > 0) {
        memset(m_printOut, 0, 256);
        sprintf(m_printOut, "[Blv_comm][Id: %d] Message sent : ", m_message[0]);
        for (size_t i = 0 ; i < msg_+2 ; i++) {
            sprintf(&m_printOut[strlen(m_printOut)], "%02x ", m_message[i]);
        }
        sprintf(&m_printOut[strlen(m_printOut)], "\n");
        m_printFunction(m_printOut);
    }

    // Get Slave Answer
    if(m_message[0] != 0x00) {
        int length = modbus::receive(m_response, resp_);
        if(length == -1){
            m_printFunction("[Blv_comm] Slave Response Corrupted\n");
            return false;
        }

        /**************************************/
        /*            Verify CRC16            */
        /**************************************/
        uint16_t slave_crc16 = (m_response[length-1] << 8) | m_response[length-2];
        uint16_t expected_crc16 = modbus::CRC16(m_response, length-2);
        if((expected_crc16 ^ slave_crc16) != 0) return false;

        /**************************************/
        /*  Broadcast Slave Response   */
        /**************************************/
        if(m_logLevel > 0) {
            memset(m_printOut, 0, 256);
            sprintf(m_printOut, "[Blv_comm][Id: %d] Response : ", m_message[0]);
            for (int i = 0 ; i < length ; i++) {
                sprintf(&m_printOut[strlen(m_printOut)], "%02x ", m_response[i]);
            }
            sprintf(&m_printOut[strlen(m_printOut)], "\n");
            m_printFunction(m_printOut);
        }
        
    }

    return true;
}

bool blv_commands::blv_comm::m_check_response(size_t size_){
    if(m_message[0] == 0x00) return true;
    // Check if it is an Exception Response
    if(m_response[1] >= 0x80) return false;

    // Check if the Response matches the message sent
    for(size_t i = 0; i < size_; i++){
        if(m_response[i] != m_message[i]) return false;
    }

    return true;
}

bool blv_commands::blv_comm::m_config_wire_input_mode(uint8_t id_, blv_commands::WireMode val_){
    m_message[0] = id_;
    m_message[1] = 0x06;
    m_message[2] = 0x10;
    m_message[3] = 0x41;
    m_message[4] = 0x00;
    m_message[5] = static_cast<uint8_t>(val_);

    return(m_communicate(6, 8) && ((id_ == 0x00) || m_check_response(8)));
}

bool blv_commands::blv_comm::m_config_transmission_waiting_time(uint8_t id_, uint16_t val_){
    m_message[0] = id_;
    m_message[1] = 0x06;
    m_message[2] = 0x14;
    m_message[3] = 0x0B;
    m_message[4] = static_cast<uint8_t>((val_ & 0xF0) >> 8);
    m_message[5] = static_cast<uint8_t>((val_ & 0x0F));

    return(m_communicate(6, 8) && ((id_ == 0x00) || m_check_response(8)));
}

bool blv_commands::blv_comm::m_send_operation_message(uint8_t id_){
    m_message[0] = id_;
    m_message[1] = 0x06;
    m_message[2] = 0x00;
    m_message[3] = 0x7D;
    m_message[4] = 0x00;
    m_message[5] = m_operation;

    return(m_communicate(6, 8) && ((id_ == 0x00) || m_check_response(8)));
}

blv_commands::blv_comm::blv_comm(size_t nbr_slave_, uint8_t* slaves_, void (*printFunction_)(const char*)){
    if(printFunction_ != NULL) m_printFunction = printFunction_;

    // Initialize class attributes
    memset(m_message, 0, sizeof(m_message));
    memset(m_response, 0, sizeof(m_response));

    // Initialize BLV motors
    for(size_t i = 0; i < nbr_slave_; i++){
        slaves_[i] = slaves_[i];

        // Set 3-wire input mode (p.19)
        if(!m_config_wire_input_mode(slaves_[i])){
            // Error Message
            memset(m_printOut, 0, 256);
            sprintf(m_printOut, "[Blv_comm][Id: %d] Wire Input Mode Failed\n", slaves_[i]);
            m_printFunction(m_printOut);
        } else {

            if(!m_config_transmission_waiting_time(slaves_[i])){
                // Error Message
                memset(m_printOut, 0, 256);
                sprintf(m_printOut, "[Blv_comm][Id: %d] Transmission Waiting Time Set Failed\n", slaves_[i]);
                m_printFunction(m_printOut);
            } else {
                exec_config(slaves_[i]);

                // According to the BLV documentation, the default mode is Mode 0. And 
                // within this mode, we must use mode 2 to 7 for digital control.
                // Mode 2 is by default in select_mode function
                select_mode(slaves_[i]);
            }
        }
    }
}

bool blv_commands::blv_comm::config_Fwd_Direction(uint8_t id_,  blv_commands::Direction val_){
    m_message[0] = id_;
    m_message[1] = 0x06;
    m_message[2] = 0x03;
    m_message[3] = 0x85;
    m_message[4] = 0x00;
    m_message[5] = static_cast<uint8_t>(val_);

    return(m_communicate(6, 8) && m_check_response(8));
}

bool blv_commands::blv_comm::electromagnetic_brake_action(uint8_t id_, blv_commands::EBrake mode_){
    m_operation = (m_operation & 127) | static_cast<uint8_t>(mode_);
    if(!m_send_operation_message(id_)){
        // Error Message
        memset(m_printOut, 0, 256);
        sprintf(m_printOut, "[Blv_comm][Id: %d] Setting Electromagnetic Brake failed\n", id_);
        m_printFunction(m_printOut);
        return false;
    }

    return true;
}

bool blv_commands::blv_comm::select_mode(uint8_t id_, blv_commands::Mode mode_){
    m_operation = (m_operation & 248) | static_cast<uint8_t>(mode_);
    if(!m_send_operation_message(id_)){
        // Error Message
        memset(m_printOut, 0, 256);
        sprintf(m_printOut, "[Blv_comm][Id: %d] Setting Mode 0 - M%d failed\n", id_, static_cast<uint8_t>(mode_));
        m_printFunction(m_printOut);
        return false;
    }

    return true;
}

bool blv_commands::blv_comm::set_torque_limit(uint8_t id_, uint16_t torque_, blv_commands::Mode mode_){
    if(torque_ > 200) torque_ = static_cast<uint16_t>(200);
    
    m_message[0] = id_;
    m_message[1] = 0x06;
    m_message[2] = 0x07;
    m_message[3] = 0x00 + (static_cast<uint8_t>(mode_)<<1);
    m_message[4] = static_cast<uint8_t>(((torque_ & 0xFF00) >> 8));
    m_message[5] = static_cast<uint8_t>((torque_ & 0x00FF));

    return(m_communicate(6, 8) && m_check_response(8));
}

bool blv_commands::blv_comm::set_speed(uint8_t id_, uint16_t speed_, blv_commands::Mode mode_){
    if(speed_ < 80) speed_ = static_cast<uint16_t>(0);
    if(speed_ > 4000) speed_ = static_cast<uint16_t>(4000);

    m_message[0] = id_;
    m_message[1] = 0x06;
    m_message[2] = 0x04;
    m_message[3] = 0x81 + (static_cast<uint8_t>(mode_)<<1);
    m_message[4] = static_cast<uint8_t>(((speed_ & 0xFF00) >> 8));
    m_message[5] = static_cast<uint8_t>((speed_ & 0x00FF));

    return(m_communicate(6, 8) && m_check_response(8));
}

bool blv_commands::blv_comm::set_acc(uint8_t id_, uint16_t acc_, blv_commands::Acceleration type_, blv_commands::Mode mode_){
    if(acc_ < 2) acc_ = static_cast<uint16_t>(2);
    if(acc_ > 150) acc_ = static_cast<uint16_t>(150);
    
    m_message[0] = id_;
    m_message[1] = 0x06;
    m_message[2] = 0x06;
    m_message[3] = static_cast<uint8_t>(type_) + (static_cast<uint8_t>(mode_)<<1);
    m_message[4] = static_cast<uint8_t>(((acc_ & 0xFF00) >> 8));
    m_message[5] = static_cast<uint8_t>((acc_ & 0x00FF));

    return(m_communicate(6, 8) && m_check_response(8));
}

bool blv_commands::blv_comm::set_acc(uint8_t id_, uint16_t acc_, blv_commands::Mode mode_){
    bool acc_check = set_acc(id_, acc_, blv_commands::Acceleration::Acc, mode_);
    bool dec_check = set_acc(id_, acc_, blv_commands::Acceleration::Dec, mode_);

    return (acc_check && dec_check);
}

bool blv_commands::blv_comm::set_Rotation_Direction(uint8_t id_, blv_commands::Rotation val_){
    m_operation = (m_operation & 223) | static_cast<uint8_t>(val_);

    m_operation |= static_cast<uint8_t>(val_);
    if(!m_send_operation_message(id_)){
        // Error Message
        memset(m_printOut, 0, 256);
        sprintf(m_printOut, "[Blv_comm][Id: %d] Setting Direction failed\n", id_);
        m_printFunction(m_printOut);
        return false;
    }

    return true;
}

bool blv_commands::blv_comm::start_rotation(uint8_t id_){
    m_operation = (m_operation & 231) | 0b00011000;

    if(!m_send_operation_message(id_)){
        // Error Message
        memset(m_printOut, 0, 256);
        sprintf(m_printOut, "[Blv_comm][Id: %d] Starting Motor failed\n", id_);
        m_printFunction(m_printOut);
        return false;
    }

    return true;
}

bool blv_commands::blv_comm::stop_rotation(uint8_t id_, blv_commands::StopMode mode_){
    m_operation = (m_operation & 231) | static_cast<uint8_t>(mode_);

    if(!m_send_operation_message(id_)){
        // Error Message
        memset(m_printOut, 0, 256);
        sprintf(m_printOut, "[Blv_comm][Id: %d] Stoping Motor failed\n", id_);
        m_printFunction(m_printOut);
        return false;
    }

    return true;
}

bool blv_commands::blv_comm::diagnose(uint8_t id_){
    if(id_ == 0x00) return false;
    m_message[0] = id_;
    m_message[1] = 0x08;
    m_message[2] = 0x00;
    m_message[3] = 0x00;
    m_message[4] = 0x12;
    m_message[5] = 0x34;

    return(m_communicate(6, 8) && m_check_response(8));
}

bool blv_commands::blv_comm::reset_alarm(uint8_t id_){
    m_message[0] = id_;
    m_message[1] = 0x06;
    m_message[2] = 0x18;
    m_message[3] = 0x81;
    m_message[4] = 0x00;
    m_message[5] = 0x01;

    return(m_communicate(6, 8) && m_check_response(8));;
}

bool blv_commands::blv_comm::clear_alert(uint8_t id_, blv_commands::ClearAlerts type_){
    m_message[0] = id_;
    m_message[1] = 0x06;
    m_message[2] = 0x01;
    m_message[3] = static_cast<uint8_t>(type_);
    m_message[4] = 0x00;
    m_message[5] = 0x01;

    return(m_communicate(6, 8) && m_check_response(8));

}

bool blv_commands::blv_comm::clear_all_alerts(uint8_t id_){
    m_message[0] = id_;     // Slave adrs
    m_message[1] = 0x10;    // Function Code
    m_message[2] = 0x01;    // Register (upper)
    m_message[3] = 0x84;    // Register (lower)
    m_message[4] = 0x00;
    m_message[5] = 0x06;    // Nbr of registers (6)
    m_message[6] = 0x0C;    
    m_message[7] = 0x00;    // Nbr of data (12 bytes)
    m_message[8] = 0x00;    
    m_message[9] = 0x00;    // Registers Alarm Upper
    m_message[10] = 0x00;    
    m_message[11] = 0x01;   // Registers Alarm Lower
    m_message[12] = 0x00;    
    m_message[13] = 0x00;   // Registers Warnings Upper
    m_message[14] = 0x00;    
    m_message[15] = 0x01;   // Registers Warnings Lower
    m_message[16] = 0x00;    
    m_message[17] = 0x00;   // Registers Comm Errors Upper
    m_message[18] = 0x00;   
    m_message[19] = 0x01;   // Registers Comm Errors Lower

    return (m_communicate(20, 8) && m_check_response(6));
}

bool blv_commands::blv_comm::send_debug(const uint8_t* message_, size_t message_size_, size_t response_size_) {
    memcpy(m_message, message_, MIN(sizeof(m_message), message_size_));
    return (m_communicate(message_size_, response_size_+2) && m_check_response(response_size_));
}

bool blv_commands::blv_comm::reading_registers(uint8_t id_, uint16_t register_, uint8_t nbr_reg_){
    if(id_ == 0x00){
        // Error Message
        m_printFunction("[Blv_comm] Cannot use Id 0 for Reading Registers Query\n");
        return false;
    }

    m_message[0] = id_;
    m_message[1] = 0x03;
    m_message[2] = static_cast<uint8_t>(((register_ & 0xFF00) >> 8));   // Upper Register Address
    m_message[3] = static_cast<uint8_t>((register_ & 0x00FF));          // Lower Register Address
    m_message[4] = 0x00;
    m_message[5] = nbr_reg_;

    return (m_communicate(6, 2*nbr_reg_ + 5) && m_check_response(6));    // [slave_id, Function_code, nbr_bytes, {data} , {CRC16_low, CRC16_up}]
}

int32_t blv_commands::blv_comm::get_speed(uint8_t id_, blv_commands::Command type_){
    m_message[0] = id_;
    m_message[1] = 0x03;
    m_message[2] = 0x00;
    m_message[3] = static_cast<uint8_t>(type_);
    m_message[4] = 0x00;
    m_message[5] = 0x02;

    if(m_communicate(6, 9) /* && m_check_response(6)*/){
        return ((m_response[3] << 24) | m_response[4] << 16 | m_response[5] << 8 | m_response[6]);
    }

    return 0x00000FA1;
}

bool blv_commands::blv_comm::get_alerts(uint8_t id_, uint8_t nbr_, blv_commands::Alerts type_){
    if(nbr_ < 1) nbr_ = static_cast<uint8_t>(1);
    if(nbr_ > 10) nbr_ = static_cast<uint8_t>(10);
    
    uint16_t reg_adrs = static_cast<uint16_t>(type_) + static_cast<uint16_t>(nbr_<<1);
    return reading_registers(id_, reg_adrs, nbr_);
}


bool blv_commands::blv_comm::exec_config(uint8_t id_) {
    m_message[0] = id_;
    m_message[1] = 0x06;
    m_message[2] = 0x01;
    m_message[3] = 0x8D;
    m_message[4] = 0x00;
    m_message[5] = 0x01;

    return(m_communicate(6, 8) && m_check_response(8));
}
