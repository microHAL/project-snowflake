/**
 * @license    BSD 3-Clause
 * @copyright  Pawel Okas
 * @version    $Id$
 * @brief
 *
 * @authors    Pawel Okas
 * created on: 16-11-2017
 * last modification: 16-11-2017
 *
 * @copyright Copyright (c) 2016, Pawel Okas
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

#ifndef _MICROHAL_MICROHALPORTCONFIG_NRF51_H_
#define _MICROHAL_MICROHALPORTCONFIG_NRF51_H_

#include <cstdint>

namespace microhal {
namespace nrf51 {
//***********************************************************************************************//
//                                       configure interfaces                                    //
//***********************************************************************************************//
//***********************************************************************************************//
//                                    Serial Port configurations                                 //
//***********************************************************************************************//
#define MICROHAL_USE_SERIAL_PORT1_INTERRUPT  // available settings are MICROHAL_USE_SERIAL_PORTx_POLLING
//						 MICROHAL_USE_SERIAL_PORTx_INTERRUPT
//						 MICROHAL_USE_SERIAL_PORTx_DMA
constexpr uint32_t serialPort1_priority = 3;
#define MICROHAL_SERIAL_PORT1_TX_BUFFER_SIZE 1024
#define MICROHAL_SERIAL_PORT1_RX_BUFFER_SIZE 128
}  // namespace nrf51
}  // namespace microhal

#endif  // _MICROHAL_MICROHALPORTCONFIG_NRF51_H_
