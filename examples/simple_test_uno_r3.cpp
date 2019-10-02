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

#include <Arduino.h>
#include <blv_comm.h>

using namespace blv_commands;

// Declare the serial port
HardwareSerial* modbus::blv_serial;

// The communication handle
blv_comm* motors;

// The IDs of the slaves (the motors)
uint8_t slaves[1] = {0x01};

// Since the Arduino UNO R3 has only one Serial peripheral, 
// we cannot use the log function as it uses the only Serial,
// so we replace the print statement in the library by a placeholder
void emptyPrint(const char* msg) {
    (void)msg; //Doesn't do anything, and makes the compiler happy
}

void setup() {
    
    // First, configure the serial port the TTL to RS485 module is connected to, this time it's the default Serial
    modbus::blv_serial = &Serial;
    modbus::blv_serial->begin(9600, SERIAL_8E1);

    // Define the communication handle, specifying the slaves and our replacement for the print statement
    motors = new blv_comm(sizeof(slaves), slaves, emptyPrint);
}

void loop() {

    // Set the rotation direction to 'forward'
    motors->set_Rotation_Direction(slaves[0], Rotation::FWD);
    // Unlock the magnetic brakes and start spinning at the speed set in the register (0 at this point in time)
    motors->start_rotation(slaves[0]);
    // Set the speed in RPM
    motors->set_speed(slaves[0], 1000);
    
    delay(5000);

    // Stop the motors in 'soft' mode (creates a smooth decceleration curve)
    motors->stop_rotation(slaves[0], StopMode::Soft);
    // Set the speed to 0 in the register
    motors->set_speed(slaves[0], 0);

    delay(1000);

    // Set the rotation direction to 'reverse'
    motors->set_Rotation_Direction(slaves[0], Rotation::REV);
    // Unlock the magnetic brakes and start spinning at the speed set in the register (0 at this point in time)
    motors->start_rotation(slaves[0]);
    // Set the speed in RPM
    motors->set_speed(slaves[0], 500); // Unit is in rpm

    delay(5000);

    // Stop the motors in 'instant' mode (breaks the motor immediately, be careful of the intertia)
    motors->stop_rotation(slaves[0], StopMode::Instant);
    // Set the speed to 0 in the register
    motors->set_speed(slaves[0], 0);

    delay(1000);
}