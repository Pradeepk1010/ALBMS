/************************************
REVISION HISTORY
$Revision: 1200 $
$Date: 2020-6-22

Copyright (c) 2016, Linear Technology Corp.(LTC)
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of Linear Technology Corp.

The Linear Technology Linduino is not affiliated with the official Arduino team.
However, the Linduino is only possible because of the Arduino team's commitment
to the open-source community.  Please, visit http://www.arduino.cc and
http://store.arduino.cc , and consider a purchase that will help fund their
ongoing work.

Copyright 2016 Linear Technology Corp. (LTC)
***********************************************************/

/*! @file
    @ingroup LTC68061
    Header for LTC6806-1 Multicell Battery Monitor
*/

#ifndef LTC68061_H
#define LTC68061_H


#ifndef LTC6806_CS
#define LTC6806_CS QUIKEVAL_CS
#endif

#define MD_FAST 0
#define MD_NORMAL 1
#define MD_ALTERNATE 2
#define MD_FILTERED 3

#define CELL_CH_ALL 0
#define CELL_CH_1 1
#define CELL_CH_2 2
#define CELL_CH_3 3
#define CELL_CH_4 4
#define CELL_CH_5 5
#define CELL_CH_6 6
#define CELL_CH_7 7
#define CELL_CH_8 8
#define CELL_CH_9 9
#define CELL_CH_10 10
#define CELL_CH_11 11
#define CELL_CH_12 12
#define CELL_CH_13 13
#define CELL_CH_14 14
#define CELL_CH_15 15
#define CELL_CH_16 16
#define CELL_CH_17 17
#define CELL_CH_18 18
#define CELL_CH_19 19
#define CELL_CH_20 20
#define CELL_CH_21 21
#define CELL_CH_22 22
#define CELL_CH_23 23
#define CELL_CH_24 24
#define CELL_CH_25 25
#define CELL_CH_26 26
#define CELL_CH_27 27
#define CELL_CH_28 28
#define CELL_CH_29 29
#define CELL_CH_30 30
#define CELL_CH_31 31
#define CELL_CH_32 32
#define CELL_CH_33 33
#define CELL_CH_34 34
#define CELL_CH_35 35
#define CELL_CH_36 36

#define AUX_CH_ALL 0
#define AUX_CH_VREF2 1
#define AUX_CH_GPIO1 2
#define AUX_CH_GPIO2 3
#define AUX_CH_GPIO3 4
#define AUX_CH_GPIO4 5
#define AUX_CH_GPIO5 6
#define AUX_CH_GPIO6 7


#define STAT_CH_ALL 0
#define STAT_CH_SOC 1
#define STAT_CH_ITEMP 2
#define STAT_CH_VREGA 3


#define PULL_UP_CURRENT 0
#define PULL_DOWN_CURRENT 1

#define CELL_CHANNELS_IC 12
#define AUX_CHANNELS_IC 6

#define PRECHARGE_100NS 0x00
#define PRECHARGE_1MS 0x10
#define PRECHARGE_10MS 0x20
#define PRECHARGE_100MS 0x30

void LTC6806_adcv(uint8_t MD, //!< ADC Mode
                  uint8_t CH //!< Cell Channels to be measured
                 );

void LTC6806_adcvsc(
  uint8_t MD //!< ADC Mode
);

void LTC6806_cvst(
  uint8_t MD, //!< ADC Mode
  uint8_t ST //!< Self Test
);
void LTC6806_adow(
  uint8_t MD, //!< ADC Mode
  uint8_t PUP, //!< Discharge Permit
  uint8_t CH //!< Cell Channels to be measured
);
void LTC6806_adax(
  uint8_t MD, //!< ADC Mode
  uint8_t CHG //!< GPIO Channels to be measured
);

void LTC6806_adaxsc(
  uint8_t MD //!< ADC Mode
);

void LTC6806_axst(
  uint8_t MD, //!< ADC Mode
  uint8_t ST //!< Self Test
);

void LTC6806_adstat(
  uint8_t MD, //!< ADC Mode
  uint8_t CHST //!< GPIO Channels to be measured
);

void LTC6806_statst(
  uint8_t MD, //!< ADC Mode
  uint8_t ST //!< Self Test
);



int8_t LTC6806_rdstat(uint8_t reg,
                      uint8_t nIC,
                      uint16_t stat_codes[][6]);

void LTC6806_rdstat_reg(uint8_t reg,
                        uint8_t nIC,
                        uint8_t *data);



void LTC6806_diagn();

uint8_t LTC6811_pladc();

uint32_t LTC6806_pollAdc();

uint8_t LTC6806_rdcv(uint8_t reg,
                     uint8_t total_ic,
                     int16_t cell_codes[][36]);

void LTC6806_rdcv_reg(uint8_t reg,
                      uint8_t nIC,
                      uint8_t *data);

int8_t LTC6806_rdaux(uint8_t reg,
                     uint8_t nIC,
                     uint16_t aux_codes[][8]);

void LTC6806_rdaux_reg(uint8_t reg,
                       uint8_t nIC,
                       uint8_t *data);

int8_t LTC6806_rdstat(uint8_t reg, //Determines which GPIO voltage register is read back.
                      uint8_t total_ic,//the number of ICs in the system
                      uint16_t stat_codes[][3],//A two dimensional array of the gpio voltage codes.
                      uint8_t flags[][9],
                      uint8_t mux_fail[][1],
                      uint8_t temp_flags[][1]
                     );

void LTC6806_clrcell();

void LTC6806_clraux();

void LTC6806_clrstat();

void LTC6806_wrcfg(uint8_t nIC,
                   uint8_t config[][6]);

int8_t LTC6806_rdcfg(uint8_t nIC,
                     uint8_t r_config[][8]);

void wakeup_idle(uint8_t total_ic);

void wakeup_sleep(uint8_t total_ic);

uint16_t pec15_calc(uint8_t len,
                    uint8_t *data);

void spi_write_array( uint8_t length,
                      uint8_t *data);

void spi_write_read(uint8_t *TxData,
                    uint8_t TXlen,
                    uint8_t *rx_data,
                    uint8_t RXlen);

int16_t conver12to16(uint16_t data);

const uint16_t crc15Table[256] PROGMEM = {0x0,0xc599, 0xceab, 0xb32, 0xd8cf, 0x1d56, 0x1664, 0xd3fd, 0xf407, 0x319e, 0x3aac,  //!<precomputed CRC15 Table
                               0xff35, 0x2cc8, 0xe951, 0xe263, 0x27fa, 0xad97, 0x680e, 0x633c, 0xa6a5, 0x7558, 0xb0c1,
                               0xbbf3, 0x7e6a, 0x5990, 0x9c09, 0x973b, 0x52a2, 0x815f, 0x44c6, 0x4ff4, 0x8a6d, 0x5b2e,
                               0x9eb7, 0x9585, 0x501c, 0x83e1, 0x4678, 0x4d4a, 0x88d3, 0xaf29, 0x6ab0, 0x6182, 0xa41b,
                               0x77e6, 0xb27f, 0xb94d, 0x7cd4, 0xf6b9, 0x3320, 0x3812, 0xfd8b, 0x2e76, 0xebef, 0xe0dd,
                               0x2544, 0x2be, 0xc727, 0xcc15, 0x98c, 0xda71, 0x1fe8, 0x14da, 0xd143, 0xf3c5, 0x365c,
                               0x3d6e, 0xf8f7,0x2b0a, 0xee93, 0xe5a1, 0x2038, 0x7c2, 0xc25b, 0xc969, 0xcf0, 0xdf0d,
                               0x1a94, 0x11a6, 0xd43f, 0x5e52, 0x9bcb, 0x90f9, 0x5560, 0x869d, 0x4304, 0x4836, 0x8daf,
                               0xaa55, 0x6fcc, 0x64fe, 0xa167, 0x729a, 0xb703, 0xbc31, 0x79a8, 0xa8eb, 0x6d72, 0x6640,
                               0xa3d9, 0x7024, 0xb5bd, 0xbe8f, 0x7b16, 0x5cec, 0x9975, 0x9247, 0x57de, 0x8423, 0x41ba,
                               0x4a88, 0x8f11, 0x57c, 0xc0e5, 0xcbd7, 0xe4e, 0xddb3, 0x182a, 0x1318, 0xd681, 0xf17b,
                               0x34e2, 0x3fd0, 0xfa49, 0x29b4, 0xec2d, 0xe71f, 0x2286, 0xa213, 0x678a, 0x6cb8, 0xa921,
                               0x7adc, 0xbf45, 0xb477, 0x71ee, 0x5614, 0x938d, 0x98bf, 0x5d26, 0x8edb, 0x4b42, 0x4070,
                               0x85e9, 0xf84, 0xca1d, 0xc12f, 0x4b6, 0xd74b, 0x12d2, 0x19e0, 0xdc79, 0xfb83, 0x3e1a, 0x3528,
                               0xf0b1, 0x234c, 0xe6d5, 0xede7, 0x287e, 0xf93d, 0x3ca4, 0x3796, 0xf20f, 0x21f2, 0xe46b, 0xef59,
                               0x2ac0, 0xd3a, 0xc8a3, 0xc391, 0x608, 0xd5f5, 0x106c, 0x1b5e, 0xdec7, 0x54aa, 0x9133, 0x9a01,
                               0x5f98, 0x8c65, 0x49fc, 0x42ce, 0x8757, 0xa0ad, 0x6534, 0x6e06, 0xab9f, 0x7862, 0xbdfb, 0xb6c9,
                               0x7350, 0x51d6, 0x944f, 0x9f7d, 0x5ae4, 0x8919, 0x4c80, 0x47b2, 0x822b, 0xa5d1, 0x6048, 0x6b7a,
                               0xaee3, 0x7d1e, 0xb887, 0xb3b5, 0x762c, 0xfc41, 0x39d8, 0x32ea, 0xf773, 0x248e, 0xe117, 0xea25,
                               0x2fbc, 0x846, 0xcddf, 0xc6ed, 0x374, 0xd089, 0x1510, 0x1e22, 0xdbbb, 0xaf8, 0xcf61, 0xc453,
                               0x1ca, 0xd237, 0x17ae, 0x1c9c, 0xd905, 0xfeff, 0x3b66, 0x3054, 0xf5cd, 0x2630, 0xe3a9, 0xe89b,
                               0x2d02, 0xa76f, 0x62f6, 0x69c4, 0xac5d, 0x7fa0, 0xba39, 0xb10b, 0x7492, 0x5368, 0x96f1, 0x9dc3,
                               0x585a, 0x8ba7, 0x4e3e, 0x450c, 0x8095
                                         };
#endif
