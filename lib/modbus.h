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

#ifndef MODBUS_H
#define MODBUS_H

#include <Arduino.h>

namespace modbus
{
    extern HardwareSerial* blv_serial;

    inline uint16_t CRC16(const uint8_t *nData, uint16_t wLength);

    //------------------------------------------------------//
    // Send a Modbus RTU message over RS485                 //
    // message_: Modbus message.                            //
    // Must comply with modbus protocol i.e,                //
    // [slave_id, function-code, Registers, Data, CRC16]    //
    // size_: number of bytes to send                       //
    //------------------------------------------------------//
    void send(uint8_t* message_, size_t size_);

    //------------------------------------------------------//
    // Read the Modbus RTU message sent by the Slave        //
    // response_: Modbus message.                           //
    // [slave_id, function-code, Registers, Data, CRC16]    //
    // size_: Specificied the expected length of the message//
    //                                                      //
    // Return -1 if the message is corrupted. Return the    //
    // number of bytes received oterwise.                   //
    //------------------------------------------------------//
    int receive(uint8_t* response_, size_t size_);
} // Namespace modbus

#include "modbus.hxx"

#endif //MODBUS_H