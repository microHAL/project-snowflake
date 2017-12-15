/**
 * @license    BSD 3-Clause
 * @copyright  microHAL
 * @version    $Id$
 * @brief      Snowflake main file.
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

#include <algorithm>

#include "bsp.h"
#include "diagnostic/diagnostic.h"
#include "microhal.h"
#include "ports/nrf51/timer.h"

using namespace microhal;
using namespace microhal::diagnostic;
using namespace std::literals::chrono_literals;

nrf51::Timer timer(*NRF_TIMER1);
uint8_t ledPattern = 1;

class RGB {
 public:
    RGB(GPIO::IOPin ledr_pin, GPIO::IOPin ledg_pin, GPIO::IOPin ledb_pin)
        : r(ledr_pin, GPIO::Direction::Output), g(ledg_pin, GPIO::Direction::Output), b(ledb_pin, GPIO::Direction::Output) {}

    GPIO r;
    GPIO g;
    GPIO b;
};

RGB ledA(ledAR_pin, ledAG_pin, ledAB_pin);
RGB ledB(ledBR_pin, ledBG_pin, ledBB_pin);
RGB ledC(ledCR_pin, ledCG_pin, ledCB_pin);
RGB ledD(ledDR_pin, ledDG_pin, ledDB_pin);
RGB ledE(ledER_pin, ledEG_pin, ledEB_pin);
RGB ledF(ledFR_pin, ledFG_pin, ledFB_pin);
std::array<RGB *, 6> leds = {&ledA, &ledF, &ledB, &ledE, &ledC, &ledD};

std::array<uint8_t, 6 * 3> pwm;

void timerInterrupt() {
    static uint8_t counter;

    for (size_t i = 0; i < leds.size(); i++) {
        if (pwm[3 * i] > counter) {
            leds[i]->r.reset();
        } else {
            leds[i]->r.set();
        }

        if (pwm[3 * i + 1] > counter) {
            leds[i]->g.reset();
        } else {
            leds[i]->g.set();
        }

        if (pwm[3 * i + 2] > counter) {
            leds[i]->b.reset();
        } else {
            leds[i]->b.set();
        }
    }

    counter++;
}

void offAllLeds() {
    for (auto led : leds) {
        led->r.set();
        led->g.set();
        led->b.set();
    }
}

void pattern1() {
    static uint8_t state = 0;
    offAllLeds();
    for (auto led : leds) {
        if (state == 0) led->r.reset();
        if (state == 1) led->g.reset();
        if (state == 2) led->b.reset();
    }
    state++;
    if (state == 3) state = 0;
}

void pattern2() {
    static uint8_t state = 0;
    offAllLeds();

    for (size_t i = 0; i < leds.size(); i++) {
        if (state < 6 && i <= state) leds[i]->r.reset();
        if (state >= 6 && state < 13 && i <= state - 6) leds[i]->g.reset();
        if (state >= 13 && state < 19 && i <= state - 13) leds[i]->b.reset();
        if (state >= 19 && i <= state - 19) {
            leds[i]->r.reset();
            leds[i]->g.reset();
            leds[i]->b.reset();
        }
    }
    state++;
    if (state == 25) state = 0;
}

void pattern3_init() {
    timer.prescaler(4);
    timer.bitMode(nrf51::Timer::BitMode::Width_32Bits);
    timer.mode(nrf51::Timer::Mode::Timer);

    timer.enableInterrupt(3);

    timer.captureCompare[0].enableInterrupt();
    timer.captureCompare[0].enableTimerClearOnCompare();
    timer.captureCompare[0].value(50);
    timer.captureCompare[0].connectInterrupt(timerInterrupt);
}
void pattern3() {
    static uint8_t j = 0;
    uint8_t tmp = pwm[j];

    for (size_t i = 0; i < pwm.size() / 3; i++) {
        pwm[3 * i + j] += 10;
    }
    if (pwm[j] < tmp) j++;
    if (j == 3) j = 0;
}

void bleInit();
void power_manage();

int main(void) {
    bsp::init();

    // GPIO button(button_pin, GPIO::Direction::Input);
    offAllLeds();

    auto configureSerialPort = [](SerialPort &serial) {
        serial.open(SerialPort::ReadWrite);
        serial.setBaudRate(SerialPort::Baud38400);
        serial.setDataBits(SerialPort::Data8);
        serial.setStopBits(SerialPort::OneStop);
        serial.setParity(SerialPort::NoParity);
    };

    configureSerialPort(bsp::serialPort);
    diagChannel.setOutputDevice(bsp::serialPort);

    diagChannel << MICROHAL_DEBUG << "Snowflake demo started" << endl;
    bleInit();
    pattern3_init();

    // Enter main loop.
    for (;;) {
        //        power_manage();

        switch (ledPattern) {
            case 0:  // off
                offAllLeds();
                power_manage();
                break;
            case 1:
                pattern1();
                std::this_thread::sleep_for(20ms);
                break;
            case 2:
                pattern2();
                std::this_thread::sleep_for(20ms);
                break;
            case 3:
                pattern3();
                std::this_thread::sleep_for(1ms);
                break;
        }
    }

    return 0;
}

void bleUartHandler(uint8_t *p_data, uint16_t length) {
    static uint8_t data[256];
    std::copy_n(p_data, length, data);
    data[length + 1] = 0;

    diagChannel << MICROHAL_DEBUG << "From nus_data_handler: " << (const char *)data;

    if (p_data[0] == 0) {
        diagChannel << MICROHAL_DEBUG << "Entering sleep mode.";
        ledPattern = 0;
        timer.stop();
    }
    if (p_data[0] == 1) {
        diagChannel << MICROHAL_DEBUG << "Setting LED pattern to 1.";
        ledPattern = 1;
        timer.stop();
    }
    if (p_data[0] == 2) {
        diagChannel << MICROHAL_DEBUG << "Setting LED pattern to 2.";
        ledPattern = 2;
        timer.stop();
    }
    if (p_data[0] == 3) {
        diagChannel << MICROHAL_DEBUG << "Setting LED pattern to 3.";
        timer.start();
        ledPattern = 3;
    }
}
