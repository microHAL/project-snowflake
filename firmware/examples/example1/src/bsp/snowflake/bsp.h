/**
 * @license    BSD 3-Clause
 * @copyright  microHAL
 * @version    $Id$
 * @brief
 *
 * @authors    Pawel Okas
 * created on: 16-11-2017
 * last modification: 16-11-2017
 *
 * @copyright Copyright (c) 2017, Pawel Okas
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 *
 *     1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 *     2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *     3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this
 *        software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _MICROHAL_BSP_H_
#define _MICROHAL_BSP_H_
/* **************************************************************************************************************************************************
 * INCLUDES
 */
#include "microhal.h"

constexpr microhal::GPIO::IOPin ledAR_pin(microhal::nrf51::GPIO::Port::P0, 19);
constexpr microhal::GPIO::IOPin ledAG_pin(microhal::nrf51::GPIO::Port::P0, 18);
constexpr microhal::GPIO::IOPin ledAB_pin(microhal::nrf51::GPIO::Port::P0, 17);

constexpr microhal::GPIO::IOPin ledBR_pin(microhal::nrf51::GPIO::Port::P0, 8);
constexpr microhal::GPIO::IOPin ledBG_pin(microhal::nrf51::GPIO::Port::P0, 9);
constexpr microhal::GPIO::IOPin ledBB_pin(microhal::nrf51::GPIO::Port::P0, 10);

constexpr microhal::GPIO::IOPin ledCR_pin(microhal::nrf51::GPIO::Port::P0, 30);
constexpr microhal::GPIO::IOPin ledCG_pin(microhal::nrf51::GPIO::Port::P0, 0);
constexpr microhal::GPIO::IOPin ledCB_pin(microhal::nrf51::GPIO::Port::P0, 1);

constexpr microhal::GPIO::IOPin ledDR_pin(microhal::nrf51::GPIO::Port::P0, 23);
constexpr microhal::GPIO::IOPin ledDG_pin(microhal::nrf51::GPIO::Port::P0, 22);
constexpr microhal::GPIO::IOPin ledDB_pin(microhal::nrf51::GPIO::Port::P0, 21);

constexpr microhal::GPIO::IOPin ledER_pin(microhal::nrf51::GPIO::Port::P0, 2);
constexpr microhal::GPIO::IOPin ledEG_pin(microhal::nrf51::GPIO::Port::P0, 3);
constexpr microhal::GPIO::IOPin ledEB_pin(microhal::nrf51::GPIO::Port::P0, 4);

constexpr microhal::GPIO::IOPin ledFR_pin(microhal::nrf51::GPIO::Port::P0, 11);
constexpr microhal::GPIO::IOPin ledFG_pin(microhal::nrf51::GPIO::Port::P0, 12);
constexpr microhal::GPIO::IOPin ledFB_pin(microhal::nrf51::GPIO::Port::P0, 13);

constexpr microhal::GPIO::IOPin button_pin(microhal::nrf51::GPIO::Port::P0, 27);

#endif  // _MICROHAL_BSP_H_
