/*
BLV640_Library
Copyright (C) 2019 Rakuten
julian.desvignes@rakuten.com
wilson.colin@rakuten.com

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

#ifndef BLV_ALERT_CODE_H
#define BLV_ALERT_CODE_H

#include <Arduino.h>

namespace blv_alert_code {

const char* get_error(uint8_t code_){
    switch (code_)
    {
    case 0x20:
        return "Overcurrent";
    case 0x21:
        return "Overheat";
    case 0x22:
        return "Overvoltage";
    case 0x25:
        return "Undervoltage";
    case 0x28:
        return "Sensor error";
    case 0x2D:
        return "Circuit out error";
    case 0x30:
        return "Overload";
    case 0x31:
        return "Overspeed";
    case 0x41:
        return "EEPROM error";
    case 0x42:
        return "Init. sensor error";
    case 0x46:
        return "Init. op. error";
    case 0x6E:
        return "External stop";
    case 0x6C:
        return "Operation error";
    case 0x81:
        return "Network bus error";
    case 0x83:
        return "Comm. switch setting error";
    case 0x84:
        return "RS-485 error";
    case 0x85:
        return "RS-485 timeout";
    case 0x88:
        return "Cmd undefined";
    case 0x89:
        return "OPX-2A progressing";
    case 0x8A:
        return "NV mem processing";
    case 0x8C:
        return "Outside setting range";
    case 0x8D:
        return "Cmd not executed";
    case 0x8E:
        return "Network converter error";
    default:
        return "Err undefined";
    }
}

} // namespace blv_alert_code
#endif // BLV_ALERT_CODE_H