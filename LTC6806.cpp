/*!
  LTC6806-1 Multicell Battery Monitor
@verbatim
  The LTC6806

  Using the LTC6806-1, multiple devices are connected in
  a daisy-chain with one host processor connection for all
  devices.
@endverbatim
REVISION HISTORY
$Revision: 1200 $
$Date: 2020-6-22


Copyright (c) 2013, Linear Technology Corp.(LTC)
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
//! @defgroup LTC68061 LTC6806-1: Multicell Battery Monitor

/*! @file
    @ingroup LTC68061
    Library for LTC6806-1 Multicell Battery Monitor
*/

#include <stdint.h>
#include <Arduino.h>
#include "Linduino.h"
#include "LT_SPI.h"
#include "LTC6806.h"
#include <SPI.h>






/*!*******************************************************************************************************************
 \brief Maps  global ADC control variables to the appropriate control bytes for each of the different ADC commands

@param[in] uint8_t MD The adc conversion mode
@param[in] uint8_t DCP Controls if Discharge is permitted during cell conversions
@param[in] uint8_t CH Determines which cells are measured during an ADC conversion command
@param[in] uint8_t CHG Determines which GPIO channels are measured during Auxiliary conversion command

Command Code:
-------------

|command  |  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|-----------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|ADCV:      |   0   |   0   |   0   |   0   |   0   |   0   |   1   | MD[1] | MD[2] |   1   |   1   |  DCP  |   0   | CH[2] | CH[1] | CH[0] |
|ADAX:      |   0   |   0   |   0   |   0   |   0   |   1   |   0   | MD[1] | MD[2] |   1   |   1   |  DCP  |   0   | CHG[2]| CHG[1]| CHG[0]|
 ******************************************************************************************************************/


/*!*********************************************************************************************
  \brief Starts cell voltage conversion

  Starts ADC conversions of the LTC6806 Cpin inputs.
  The type of ADC conversion executed can be changed by setting the associated global variables:
 |Variable|Function                                      |
 |--------|----------------------------------------------|
 | MD     | Determines the filter corner of the ADC      |
 | CH     | Determines which cell channels are converted |
 | DCP    | Determines if Discharge is Permitted       |

Command Code:
-------------

|CMD[0:1] |  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|-----------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|ADCV:      |   0   |   0   |   0   |   0   |   0   |   0   |   1   | MD[1] | MD[2] |   1   |   1   |  DCP  |   0   | CH[2] | CH[1] | CH[0] |
***********************************************************************************************/
void LTC6806_adcv(uint8_t MD, //ADC Mode
                  uint8_t CH //Cell Channels to be measured
                 )
{
  uint8_t md_bits;
  uint8_t cmd[4];
  uint16_t cmd_pec;
  md_bits = (MD & 0x02) >> 1;

  cmd[0] = 0x04;
  cmd[1] =  (MD<<6)  + CH;

  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  output_low(LTC6806_CS);
  spi_write_array(4,cmd);
  output_high(LTC6806_CS);

}

void LTC6806_adcvsc(
  uint8_t MD //ADC Mode
)
{
  uint8_t md_bits;
  uint8_t cmd[4];
  uint16_t cmd_pec;
  md_bits = (MD & 0x02) >> 1;

  cmd[0] = 0x04;
  cmd[1] =  (MD<<6)  | 0x30;

  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  output_low(LTC6806_CS);
  spi_write_array(4,cmd);
  output_high(LTC6806_CS);

}

void LTC6806_cvst(  uint8_t MD, //ADC Mode
                    uint8_t ST //Self Test
                 )
{
  uint8_t md_bits;
  uint8_t cmd[4];
  uint16_t cmd_pec;

  cmd[0] =   ST|0x04;
  cmd[1] =  (MD<<6)|0x3F ;

  //2
  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  //4
  output_low(LTC6806_CS);
  spi_write_array(4,cmd);
  output_high(LTC6806_CS);

}

/*!******************************************************************************************************
 \brief Start an GPIO Conversion

  Starts an ADC conversions of the LTC6806 GPIO inputs.
  The type of ADC conversion executed can be changed by setting the associated global variables:
 |Variable|Function                                      |
 |--------|----------------------------------------------|
 | MD     | Determines the filter corner of the ADC      |
 | CHG    | Determines which GPIO channels are converted |


Command Code:
-------------

|CMD[0:1] |  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|-----------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|ADAX:      |   0   |   0   |   0   |   0   |   0   |   1   |   0   | MD[1] | MD[2] |   1   |   1   |  DCP  |   0   | CHG[2]| CHG[1]| CHG[0]|
*********************************************************************************************************/
void LTC6806_adax(
  uint8_t MD, //ADC Mode
  uint8_t CHG //GPIO Channels to be measured
)
{
  uint8_t md_bits;
  uint8_t cmd[4];
  uint16_t cmd_pec;


  cmd[0] =  0x03;
  md_bits = (MD & 0x03) << 6;
  cmd[1] = md_bits + 0x20 + CHG ;
  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  output_low(LTC6806_CS);
  spi_write_array(4,cmd);
  output_high(LTC6806_CS);

}

void LTC6806_adaxsc(
  uint8_t MD //ADC Mode
)
{
  uint8_t md_bits;
  uint8_t cmd[4];
  uint16_t cmd_pec;

  md_bits = (MD & 0x02) >> 1;
  cmd[0] =  0x03;
  md_bits = (MD & 0x03) << 6;
  cmd[1] = md_bits | 0x30;
  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  output_low(LTC6806_CS);
  spi_write_array(4,cmd);
  output_high(LTC6806_CS);

}

void LTC6806_axst(
  uint8_t MD, //ADC Mode
  uint8_t ST //Self Test
)
{
  uint8_t md_bits;
  uint8_t cmd[4];
  uint16_t cmd_pec;

  cmd[0] =   ST;
  cmd[1] =  (MD<<6)|0x37 ;
  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  output_low(LTC6806_CS);
  spi_write_array(4,cmd);
  output_high(LTC6806_CS);

}

void LTC6806_adstat(
  uint8_t MD, //ADC Mode
  uint8_t CHST //GPIO Channels to be measured
)
{
  uint8_t md_bits;
  uint8_t cmd[4];
  uint16_t cmd_pec;


  cmd[0] =  0x03;
  md_bits = (MD & 0x03) << 6;
  cmd[1] = md_bits | 0x28 | CHST ;
  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  output_low(LTC6806_CS);
  spi_write_array(4,cmd);
  output_high(LTC6806_CS);

}

void LTC6806_statst(
  uint8_t MD, //ADC Mode
  uint8_t ST //Self Test
)
{
  uint8_t md_bits;
  uint8_t cmd[4];
  uint16_t cmd_pec;

  cmd[0] =   ST;
  cmd[1] =  (MD<<6)|0x3F ;
  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  output_low(LTC6806_CS);
  spi_write_array(4,cmd);
  output_high(LTC6806_CS);

}
/*!******************************************************************************************************
 \brief Start an open wire Conversion

  Starts an ADC conversions of the LTC6811 GPIO inputs.
  The type of ADC conversion executed can be changed by setting the associated global variables:
 |Variable|Function                                      |
 |--------|----------------------------------------------|
 | MD     | Determines the filter corner of the ADC      |
 | CHG    | Determines which GPIO channels are converted |


Command Code:
-------------

|CMD[0:1] |  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|-----------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|ADAX:      |   0   |   0   |   0   |   0   |   0   |   1   |   0   | MD[1] | MD[2] |   1   |   1   |  DCP  |   0   | CHG[2]| CHG[1]| CHG[0]|
*********************************************************************************************************/
void LTC6806_adow(
  uint8_t MD, //ADC Mode
  uint8_t PUP, //Discharge Permit
  uint8_t CH //Cell Channels to be measured
)
{
  uint8_t md_bits;
  uint8_t cmd[4];
  uint16_t cmd_pec;

  cmd[0] = 0x06 + PUP;
  cmd[1] =  (MD<<6)  + CH;
  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  output_low(LTC6806_CS);
  spi_write_array(4,cmd);
  output_high(LTC6806_CS);

}

void LTC6806_diagn()
{
  uint8_t cmd[4];
  uint16_t cmd_pec;

  cmd[0] = 0x00;
  cmd[1] = 0x1D;
  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec );


  output_low(LTC6806_CS);
  spi_write_read(cmd,4,0,0);
  output_high(LTC6806_CS);
}

/***********************************************//**
 \brief Reads and parses the LTC6806 cell voltage registers.

 The function is used to read the cell codes of the LTC6806.
 This function will send the requested read commands parse the data
 and store the cell voltages in cell_codes variable.

 @param[in] uint8_t reg; This controls which cell voltage register is read back.

          0: Read back all Cell registers

          1: Read back cell group A

          2: Read back cell group B

          3: Read back cell group C

          4: Read back cell group D

      5: Read back cell group E

      6: Read back cell group F

      7: Read back cell group G

      8: Read back cell group H

      9: Read back cell group I

 @param[in] uint8_t total_ic; This is the number of ICs in the daisy chain(-1 only)

 @param[out] uint16_t cell_codes[]; An array of the parsed cell codes from lowest to highest. The cell codes will
  be stored in the cell_codes[] array in the following format:
  |  cell_codes[0][0]| cell_codes[0][1] |  cell_codes[0][2]|    .....     |  cell_codes[0][11]|  cell_codes[1][0] | cell_codes[1][1]|  .....   |
  |------------------|------------------|------------------|--------------|-------------------|-------------------|-----------------|----------|
  |IC1 Cell 1        |IC1 Cell 2        |IC1 Cell 3        |    .....     |  IC1 Cell 12      |IC2 Cell 1         |IC2 Cell 2       | .....    |

  @return int8_t, PEC Status.

    0: No PEC error detected

    -1: PEC error detected, retry read


 *************************************************/
uint8_t LTC6806_rdcv(uint8_t reg, // Controls which cell voltage register is read back.
                     uint8_t total_ic, // the number of ICs in the system
                     int16_t cell_codes[][36] // Array of the parsed cell codes
                    )
{

  uint8_t NUM_RX_BYT = 8;
  uint8_t BYT_IN_REG = 6;
  uint8_t CELL_IN_REG = 4;

  uint8_t *cell_data;
  uint8_t pec_error = 0;
  uint16_t parsed_cell,parsed_cell2;
  uint16_t received_pec;
  uint16_t data_pec;
  uint8_t data_counter=0; //data counter
  cell_data = (uint8_t *) malloc((NUM_RX_BYT*total_ic)*sizeof(uint8_t));

  if (reg == 0)
  {

    for (uint8_t cell_reg = 1; cell_reg<10; cell_reg++)                   //executes once for each of the LTC6806 cell voltage registers
    {
      data_counter = 0;
      LTC6806_rdcv_reg(cell_reg, total_ic,cell_data );                //Reads a single Cell voltage register

      for (uint8_t current_ic = 0 ; current_ic < total_ic; current_ic++)      // executes for every LTC6806 in the daisy chain
      {
        // current_ic is used as the IC counter


        for (uint8_t current_cell = 0; current_cell<(CELL_IN_REG-2); current_cell++)  // This loop parses the read back data into cell voltages, it
        {
          // loops once for each of the 3 cell voltage codes in the register

          parsed_cell = (uint16_t)(cell_data[data_counter]<<4) + ((cell_data[data_counter + 1] & 0xF0)>>4);//Each cell code is received as two bytes and is combined to
          parsed_cell2 = ((uint16_t)((uint16_t)cell_data[data_counter + 1] & 0x0F)<<8) + (cell_data[data_counter+2]);                                      // create the parsed cell voltage code

          cell_codes[current_ic][current_cell*2  + ((cell_reg - 1) * CELL_IN_REG)] = conver12to16(parsed_cell);
          cell_codes[current_ic][current_cell*2 + 1 + ((cell_reg - 1) * CELL_IN_REG)] = conver12to16(parsed_cell2);
          data_counter = data_counter + 3;                       //Because cell voltage codes are two bytes the data counter
          //must increment by two for each parsed cell code
        }

        received_pec = (cell_data[data_counter] << 8) + cell_data[data_counter+1]; //The received PEC for the current_ic is transmitted as the 7th and 8th
        //after the 6 cell voltage data bytes
        data_pec = pec15_calc(BYT_IN_REG, &cell_data[current_ic * NUM_RX_BYT]);
        if (received_pec != data_pec)
        {
          pec_error = -1;                             //The pec_error variable is simply set negative if any PEC errors
          //are detected in the serial data
        }
        data_counter=data_counter+2;                        //Because the transmitted PEC code is 2 bytes long the data_counter
        //must be incremented by 2 bytes to point to the next ICs cell voltage data
      }
    }
  }

  else
  {

    LTC6806_rdcv_reg(reg, total_ic,cell_data);
    for (uint8_t current_ic = 0 ; current_ic < total_ic; current_ic++)        // executes for every LTC6806 in the daisy chain
    {
      // current_ic is used as the IC counter

      for (uint8_t current_cell = 0; current_cell < 2; current_cell++)  // This loop parses the read back data into cell voltages, it
      {
        // loops once for each of the 3 cell voltage codes in the register

        parsed_cell = (uint16_t)(cell_data[data_counter]<<4) + ((cell_data[data_counter + 1] & 0xF0)>>4);//Each cell code is received as two bytes and is combined to
        parsed_cell2 = ((uint16_t)((uint16_t)cell_data[data_counter + 1] & 0x0F)<<8) + (cell_data[data_counter+2]);
        // parsed_cell2 = (cell_data[data_counter + 1] & 0x0F) + (uint16_t)(cell_data[data_counter+2]<<4);                                      // create the parsed cell voltage code

        cell_codes[current_ic][current_cell*2  + ((reg - 1) * CELL_IN_REG)] = conver12to16(parsed_cell);

        cell_codes[current_ic][current_cell*2 + 1 + ((reg - 1) * CELL_IN_REG)] = conver12to16(parsed_cell2);
        data_counter = data_counter + 3;                       //Because cell voltage codes are two bytes the data counter

      }

      received_pec = (cell_data[data_counter] << 8 )+ cell_data[data_counter + 1]; //The received PEC for the current_ic is transmitted as the 7th and 8th
      //after the 6 cell voltage data bytes
      data_pec = pec15_calc(BYT_IN_REG, &cell_data[current_ic * NUM_RX_BYT]);
      if (received_pec != data_pec)
      {
        pec_error = -1;                             //The pec_error variable is simply set negative if any PEC errors
        //are detected in the serial data
      }
      data_counter= data_counter + 2;                       //Because the transmitted PEC code is 2 bytes long the data_counter
      //must be incremented by 2 bytes to point to the next ICs cell voltage data
    }
  }


  free(cell_data);
  return(pec_error);
}



/***********************************************//**
 \brief Read the raw data from the LTC6806 cell voltage register

 The function reads a single cell voltage register and stores the read data
 in the *data point as a byte array. This function is rarely used outside of
 the LTC6806_rdcv() command.

 @param[in] uint8_t reg; This controls which cell voltage register is read back.

          1: Read back cell group A

          2: Read back cell group B

          3: Read back cell group C

          4: Read back cell group D

 @param[in] uint8_t total_ic; This is the number of ICs in the daisy chain(-1 only)

 @param[out] uint8_t *data; An array of the unparsed cell codes

Command Code:
-------------

|CMD[0:1] |  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|-----------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|RDCVA:     |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   1   |   0   |   0   |
|RDCVB:     |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   1   |   1   |   0   |
|RDCVC:     |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   1   |   0   |   0   |   0   |
|RDCVD:     |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   1   |   0   |   1   |   0   |

 *************************************************/
void LTC6806_rdcv_reg(uint8_t reg, //Determines which cell voltage register is read back
                      uint8_t total_ic, //the number of ICs in the
                      uint8_t *data //An array of the unparsed cell codes
                     )
{
  const uint8_t REG_LEN = 8; //number of bytes in each ICs register + 2 bytes for the PEC
  uint8_t cmd[4];
  uint16_t cmd_pec;


  if (reg == 1)     //1: RDCVA
  {
    cmd[1] = 0x04;
    cmd[0] = 0x00;
  }
  else if (reg == 2) //2: RDCVB
  {
    cmd[1] = 0x05;
    cmd[0] = 0x00;
  }
  else if (reg == 3) //3: RDCVC
  {
    cmd[1] = 0x06;
    cmd[0] = 0x00;
  }
  else if (reg == 4) //4: RDCVD
  {
    cmd[1] = 0x07;
    cmd[0] = 0x00;
  }
  else if (reg == 5) //5: RDCVE
  {
    cmd[1] = 0x08;
    cmd[0] = 0x00;
  }
  else if (reg == 6) //6: RDCVF
  {
    cmd[1] = 0x09;
    cmd[0] = 0x00;
  }
  else if (reg == 7) //7: RDCVG
  {
    cmd[1] = 0x0A;
    cmd[0] = 0x00;
  }
  else if (reg == 8) //8: RDCVH
  {
    cmd[1] = 0x0B;
    cmd[0] = 0x00;
  }
  else if (reg == 9) //9: RDCVI
  {
    cmd[1] = 0x0C;
    cmd[0] = 0x00;
  }


  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  output_low(LTC6806_CS);
  spi_write_read(cmd,4,data,(REG_LEN*total_ic));
  output_high(LTC6806_CS);

}


/***********************************************************************************//**
 \brief Reads and parses the LTC6806 auxiliary registers.

 The function is used
 to read the  parsed GPIO codes of the LTC6806. This function will send the requested
 read commands parse the data and store the gpio voltages in aux_codes variable

@param[in] uint8_t reg; This controls which GPIO voltage register is read back.

          0: Read back all auxiliary registers

          1: Read back auxiliary group A

          2: Read back auxiliary group B


@param[in] uint8_t total_ic; This is the number of ICs in the daisy chain(-1 only)


 @param[out] uint16_t aux_codes[][6]; A two dimensional array of the gpio voltage codes. The GPIO codes will
 be stored in the aux_codes[][6] array in the following format:
 |  aux_codes[0][0]| aux_codes[0][1] |  aux_codes[0][2]|  aux_codes[0][3]|  aux_codes[0][4]|  aux_codes[0][5]| aux_codes[1][0] |aux_codes[1][1]|  .....    |
 |-----------------|-----------------|-----------------|-----------------|-----------------|-----------------|-----------------|---------------|-----------|
 |IC1 GPIO1        |IC1 GPIO2        |IC1 GPIO3        |IC1 GPIO4        |IC1 GPIO5        |IC1 Vref2        |IC2 GPIO1        |IC2 GPIO2      |  .....    |

@return  int8_t, PEC Status

  0: No PEC error detected

 -1: PEC error detected, retry read
 *************************************************/
int8_t LTC6806_rdaux(uint8_t reg, //Determines which GPIO voltage register is read back.
                     uint8_t total_ic,//the number of ICs in the system
                     uint16_t aux_codes[][8]//A two dimensional array of the gpio voltage codes.
                    )
{


  uint8_t NUM_RX_BYT = 8;
  uint8_t BYT_IN_REG = 6;
  uint8_t GPIO_IN_REG = 4;

  uint8_t *data;
  uint8_t data_counter = 0;
  int8_t pec_error = 0;
  uint16_t parsed_aux;
  uint16_t parsed_aux2;
  uint16_t received_pec;
  uint16_t data_pec;
  data = (uint8_t *) malloc((NUM_RX_BYT*total_ic)*sizeof(uint8_t));
  //1.a
  if (reg == 0)
  {
    //a.i
    for (uint8_t gpio_reg = 1; gpio_reg<3; gpio_reg++)                //executes once for each of the LTC6806 aux voltage registers
    {
      data_counter = 0;
      LTC6806_rdaux_reg(gpio_reg, total_ic,data);                 //Reads the raw auxiliary register data into the data[] array

      for (uint8_t current_ic = 0 ; current_ic < total_ic; current_ic++)      // executes for every LTC6806 in the daisy chain
      {
        // current_ic is used as the IC counter

        //a.ii
        for (uint8_t current_gpio = 0; current_gpio< 2; current_gpio++) // This loop parses the read back data into GPIO voltages, it
        {
          // loops once for each of the 3 gpio voltage codes in the register

          parsed_aux = (uint16_t)(data[data_counter]<<4) + ((data[data_counter + 1] & 0xF0)>>4);//Each cell code is received as two bytes and is combined to
          parsed_aux2 = ((uint16_t)((uint16_t)data[data_counter + 1] & 0x0F)<<8) + (uint16_t)(data[data_counter+2]);                                       // create the parsed cell voltage code

          aux_codes[current_ic][current_gpio*2  + ((gpio_reg - 1) * GPIO_IN_REG)] = parsed_aux;
          aux_codes[current_ic][current_gpio*2 + 1 + ((gpio_reg - 1) * GPIO_IN_REG)] = parsed_aux2;
          data_counter = data_counter + 3;                       //Because cell voltage codes are two bytes the data counter                                      //must increment by two for each parsed gpio voltage code

        }
        //a.iii
        received_pec = (data[data_counter]<<8)+ data[data_counter+1];          //The received PEC for the current_ic is transmitted as the 7th and 8th
        //after the 6 gpio voltage data bytes
        data_pec = pec15_calc(BYT_IN_REG, &data[current_ic*NUM_RX_BYT]);
        if (received_pec != data_pec)
        {
          pec_error = -1;                             //The pec_error variable is simply set negative if any PEC errors
          //are detected in the received serial data
        }

        data_counter=data_counter+2;                        //Because the transmitted PEC code is 2 bytes long the data_counter
        //must be incremented by 2 bytes to point to the next ICs gpio voltage data
      }


    }

  }
  else
  {
    //b.i
    LTC6806_rdaux_reg(reg, total_ic, data);
    for (int current_ic = 0 ; current_ic < total_ic; current_ic++)            // executes for every LTC6806 in the daisy chain
    {
      // current_ic is used as an IC counter

      //b.ii
      for (int current_gpio = 0; current_gpio<2; current_gpio++)    // This loop parses the read back data. Loops
      {
        // once for each aux voltage in the register

        parsed_aux = (uint16_t)(data[data_counter]<<4) + ((data[data_counter + 1] & 0xF0)>>4);//Each cell code is received as two bytes and is combined to
        parsed_aux2 = (data[data_counter + 1] & 0x0F) + (uint16_t)(data[data_counter+2]<<4);                                       // create the parsed cell voltage code

        aux_codes[current_ic][current_gpio*2  + ((reg - 1) * GPIO_IN_REG)] = parsed_aux;
        aux_codes[current_ic][current_gpio*2 + 1 + ((reg - 1) * GPIO_IN_REG)] = parsed_aux2;
        data_counter = data_counter + 3;
        //must increment by two for each parsed gpio voltage code
      }
      //b.iii
      received_pec = (data[data_counter]<<8) + data[data_counter+1];         //The received PEC for the current_ic is transmitted as the 7th and 8th
      //after the 6 gpio voltage data bytes
      data_pec = pec15_calc(BYT_IN_REG, &data[current_ic*NUM_RX_BYT]);
      if (received_pec != data_pec)
      {
        pec_error = -1;                               //The pec_error variable is simply set negative if any PEC errors
        //are detected in the received serial data
      }

      data_counter=data_counter+2;                        //Because the transmitted PEC code is 2 bytes long the data_counter
      //must be incremented by 2 bytes to point to the next ICs gpio voltage data
    }
  }
  free(data);
  return (pec_error);
}
/*
  LTC6806_rdaux Sequence

  1. Switch Statement:
    a. Reg = 0
      i. Read GPIO voltage registers A-D for every IC in the daisy chain
      ii. Parse raw GPIO voltage data in cell_codes array
      iii. Check the PEC of the data read back vs the calculated PEC for each read register command
    b. Reg != 0
      i.Read single GPIO voltage register for all ICs in daisy chain
      ii. Parse raw GPIO voltage data in cell_codes array
      iii. Check the PEC of the data read back vs the calculated PEC for each read register command
  2. Return pec_error flag
*/


/***********************************************//**
 \brief Read the raw data from the LTC6806 auxiliary register

 The function reads a single GPIO voltage register and stores thre read data
 in the *data point as a byte array. This function is rarely used outside of
 the LTC6806_rdaux() command.

 @param[in] uint8_t reg; This controls which GPIO voltage register is read back.

          1: Read back auxiliary group A

          2: Read back auxiliary group B


@param[in] uint8_t total_ic; This is the number of ICs in the daisy chain

@param[out] uint8_t *data; An array of the unparsed aux codes



Command Code:
-------------

|CMD[0:1]     |  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|---------------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|RDAUXA:      |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   1   |   1   |   0   |   0   |
|RDAUXB:      |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   1   |   1   |   1   |   0   |

 *************************************************/
void LTC6806_rdaux_reg(uint8_t reg, //Determines which GPIO voltage register is read back
                       uint8_t total_ic, //The number of ICs in the system
                       uint8_t *data //Array of the unparsed auxiliary codes
                      )
{
  uint8_t REG_LEN = 8; // number of bytes in the register + 2 bytes for the PEC
  uint8_t cmd[4];
  uint16_t cmd_pec;


  if (reg == 1)     //Read back auxiliary group A
  {
    cmd[1] = 0x10;
    cmd[0] = 0x00;
  }
  else if (reg == 2)  //Read back auxiliary group B
  {
    cmd[1] = 0x11;
    cmd[0] = 0x00;
  }
  else          //Read back auxiliary group A
  {
    cmd[1] = 0x10;
    cmd[0] = 0x00;
  }

  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  output_low(LTC6806_CS);
  spi_write_read(cmd,4,data,(REG_LEN*total_ic));
  output_high(LTC6806_CS);

}

/*
 Reads and parses the ltc6811 stat registers.

 The function is used
 to read the  parsed stat codes of the ltc6811. This function will send the requested
 read commands parse the data and store the stat voltages in stat_codes variable
*/
int8_t LTC6806_rdstat(uint8_t reg, //Determines which GPIO voltage register is read back.
                      uint8_t total_ic,//the number of ICs in the system
                      uint16_t stat_codes[][3],//A two dimensional array of the gpio voltage codes.
                      uint8_t flags[][9],
                      uint8_t mux_fail[][1],
                      uint8_t temp_flags[][1]
                     )
{


  const uint8_t NUM_RX_BYT = 8;
  const uint8_t BYT_IN_REG = 6;
  const uint8_t STAT_IN_REG = 3;

  uint8_t *data;
  uint8_t data_counter = 0;
  int8_t pec_error = 0;
  uint16_t received_pec;
  uint16_t data_pec;
  uint16_t  parsed_stat = 0;
  uint16_t  parsed_stat2 = 0;
  data = (uint8_t *) malloc((NUM_RX_BYT*total_ic)*sizeof(uint8_t));

  if (reg == 0)
  {

    for (uint8_t stat_reg = 1; stat_reg<4; stat_reg++)                //executes once for each of the ltc6811 stat voltage registers
    {
      data_counter = 0;
      LTC6806_rdstat_reg(stat_reg, total_ic,data);                  //Reads the raw statiliary register data into the data[] array

      for (uint8_t current_ic = 0 ; current_ic < total_ic; current_ic++)      // executes for every ltc6811 in the daisy chain
      {
        // current_ic is used as the IC counter
        if (stat_reg == 1)
        {
          for (int k = 0; k<6; k++)
          {
            flags[current_ic][k] = data[data_counter++];
          }
        }
        else if (stat_reg == 3)
        {
          parsed_stat = (uint16_t)(data[data_counter]<<4) + ((data[data_counter + 1] & 0xF0)>>4);//Each cell code is received as two bytes and is combined to
          parsed_stat2 = ((uint16_t)((uint16_t)data[data_counter + 1] & 0x0F)<<8) + (uint16_t)(data[data_counter+2]);                                      // create the parsed cell voltage code
          stat_codes[current_ic][0] = parsed_stat;
          stat_codes[current_ic][1] = parsed_stat2;
          data_counter = data_counter + 3;
          parsed_stat = (uint16_t)(data[data_counter]<<4) + ((data[data_counter + 1] & 0xF0)>>4);
          stat_codes[current_ic][2] = parsed_stat;
          data_counter = data_counter +3;
        }
        else if (stat_reg == 2)
        {
          flags[current_ic][6] = data[data_counter++];
          flags[current_ic][7] = data[data_counter++];
          flags[current_ic][8] = data[data_counter++];
          data_counter+=2;
          mux_fail[current_ic][0] = (data[data_counter] & 0x02)>>1;
          temp_flags[current_ic][0] = (data[data_counter] & 0xC0)>>6;
          data_counter++;
        }


        received_pec = (data[data_counter]<<8)+ data[data_counter+1];          //The received PEC for the current_ic is transmitted as the 7th and 8th
        //after the 6 gpio voltage data bytes
        data_pec = pec15_calc(BYT_IN_REG, &data[current_ic*NUM_RX_BYT]);

        if (received_pec != data_pec)
        {
          pec_error = -1;                             //The pec_error variable is simply set negative if any PEC errors
          //are detected in the received serial data
        }

        data_counter=data_counter+2;                        //Because the transmitted PEC code is 2 bytes long the data_counter
        //must be incremented by 2 bytes to point to the next ICs gpio voltage data
      }


    }

  }
  else
  {

    LTC6806_rdstat_reg(reg, total_ic, data);
    for (int current_ic = 0 ; current_ic < total_ic; current_ic++)            // executes for every ltc6811 in the daisy chain
    {
      // current_ic is used as an IC counter

      if (reg ==1)
      {
        for (int k = 0; k<6; k++)
        {
          flags[current_ic][k] = data[data_counter++];
        }
      }
      else if (reg == 2)
      {
        flags[current_ic][6] = data[data_counter++];
        flags[current_ic][7] = data[data_counter++];
        flags[current_ic][8] = data[data_counter++];
        data_counter+=2;
        mux_fail[current_ic][0] = (data[data_counter] & 0x02)>>1;
        temp_flags[current_ic][0] = (data[data_counter] & 0xC0)>>6;
        data_counter++;
      }
      else if (reg == 3)
      {
        for (uint8_t current_gpio = 0; current_gpio< STAT_IN_REG; current_gpio++) // This loop parses the read back data into GPIO voltages, it
        {
          // loops once for each of the 3 gpio voltage codes in the register

          parsed_stat = data[data_counter] + (data[data_counter+1]<<8);              //Each gpio codes is received as two bytes and is combined to
          stat_codes[current_ic][current_gpio] = parsed_stat;
          data_counter=data_counter+2;                        //Because gpio voltage codes are two bytes the data counter

        }
      }

      received_pec = (data[data_counter]<<8)+ data[data_counter+1];          //The received PEC for the current_ic is transmitted as the 7th and 8th
      //after the 6 gpio voltage data bytes
      data_pec = pec15_calc(BYT_IN_REG, &data[current_ic*NUM_RX_BYT]);
      if (received_pec != data_pec)
      {
        pec_error = -1;                             //The pec_error variable is simply set negative if any PEC errors
        //are detected in the received serial data
      }

      data_counter=data_counter+2;
    }
  }
  free(data);
  return (pec_error);
}



/*
 The function reads a single stat  register and stores the read data
 in the *data point as a byte array. This function is rarely used outside of
 the ltc6811_rdstat() command.
*/
void LTC6806_rdstat_reg(uint8_t reg, //Determines which stat register is read back
                        uint8_t total_ic, //The number of ICs in the system
                        uint8_t *data //Array of the unparsed stat codes
                       )
{
  const uint8_t REG_LEN = 8; // number of bytes in the register + 2 bytes for the PEC
  uint8_t cmd[4];
  uint16_t cmd_pec;


  if (reg == 1)     //Read back stat group A
  {
    cmd[1] = 0x14;
    cmd[0] = 0x00;
  }
  else if (reg == 2)  //Read back stat group B
  {
    cmd[1] = 0x15;
    cmd[0] = 0x00;
  }
  else if (reg == 3)  //Read back stat group B
  {
    cmd[1] = 0x16;
    cmd[0] = 0x00;
  }
  else          //Read back stat group A
  {
    cmd[1] = 0x14;
    cmd[0] = 0x00;
  }

  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  output_low(LTC6806_CS);
  spi_write_read(cmd,4,data,(REG_LEN*total_ic));
  output_high(LTC6806_CS);

}



/********************************************************//**
 \brief Clears the LTC6806 cell voltage registers

 The command clears the cell voltage registers and intiallizes
 all values to 1. The register will read back hexadecimal 0xFF
 after the command is sent.


Command Code:
-------------

|CMD[0:1]     |  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|---------------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|CLRCELL:     |   0   |   0   |   0   |   0   |   0   |   1   |   1   |   1   |   0   |   0   |   0   |   1   |   0   |   0   |   0   |   1   |
************************************************************/
void LTC6806_clrcell()
{
  uint8_t cmd[4];
  uint16_t cmd_pec;


  cmd[0] = 0x00;
  cmd[1] = 0x19;

  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec );

  output_low(LTC6806_CS);
  spi_write_read(cmd,4,0,0);
  output_high(LTC6806_CS);
}


/***********************************************************//**
 \brief Clears the LTC6806 Auxiliary registers

 The command clears the Auxiliary registers and intiallizes
 all values to 1. The register will read back hexadecimal 0xFF
 after the command is sent.


Command Code:
-------------

|CMD[0:1]     |  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|---------------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|CLRAUX:      |   0   |   0   |   0   |   0   |   0   |   1   |   1   |   1   |   0   |   0   |   0   |   1   |   0   |   0   |  1    |   0   |
***************************************************************/
void LTC6806_clraux()
{
  uint8_t cmd[4];
  uint16_t cmd_pec;

  cmd[0] = 0x00;
  cmd[1] = 0x1A;

  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  output_low(LTC6806_CS);
  spi_write_read(cmd,4,0,0);
  output_high(LTC6806_CS);
}

void LTC6806_clrstat()
{
  uint8_t cmd[4];
  uint16_t cmd_pec;

  cmd[0] = 0x00;
  cmd[1] = 0x1B;

  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  output_low(LTC6806_CS);
  spi_write_read(cmd,4,0,0);
  output_high(LTC6806_CS);
}

//Sends the poll adc command
uint8_t LTC6806_pladc()
{
  uint8_t cmd[4];
  uint16_t cmd_pec;
  uint8_t adc_state = 0xFF;

  cmd[0] = 0x00;
  cmd[1] = 0x1c;
  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  output_low(LTC6806_CS);
  spi_write_array(4,cmd);
  adc_state = spi_read(0xFF);

  output_high(LTC6806_CS);
  return(adc_state);
}

//This function will block operation until the ADC has finished it's conversion
uint32_t LTC6806_pollAdc()
{
  uint32_t counter = 0;
  uint8_t finished = 0;
  uint8_t current_time = 0;
  uint8_t cmd[4];
  uint16_t cmd_pec;


  cmd[0] = 0x00;
  cmd[1] = 0x1c;
  cmd_pec = pec15_calc(2, cmd);
  cmd[2] = (uint8_t)(cmd_pec >> 8);
  cmd[3] = (uint8_t)(cmd_pec);

  output_low(LTC6806_CS);
  spi_write_array(4,cmd);

  while ((counter<200000)&&(finished == 0))
  {
    current_time = spi_read(0xFF);
    if (current_time>0)
    {
      finished = 1;
    }
    else
    {
      counter = counter + 10;
    }
  }

  output_high(LTC6806_CS);


  return(counter);
}


/*****************************************************//**
 \brief Write the LTC6806 configuration register

 This command will write the configuration registers of the LTC6806-1s
 connected in a daisy chain stack. The configuration is written in descending
 order so the last device's configuration is written first.

 @param[in] uint8_t total_ic; The number of ICs being written to.

 @param[in] uint8_t config[][6] is a two dimensional array of the configuration data that will be written, the array should contain the 6 bytes for each
 IC in the daisy chain. The lowest IC in the daisy chain should be the first 6 byte block in the array. The array should
 have the following format:
 |  config[0][0]| config[0][1] |  config[0][2]|  config[0][3]|  config[0][4]|  config[0][5]| config[1][0] |  config[1][1]|  config[1][2]|  .....    |
 |--------------|--------------|--------------|--------------|--------------|--------------|--------------|--------------|--------------|-----------|
 |IC1 CFGR0     |IC1 CFGR1     |IC1 CFGR2     |IC1 CFGR3     |IC1 CFGR4     |IC1 CFGR5     |IC2 CFGR0     |IC2 CFGR1     | IC2 CFGR2    |  .....    |

 The function will calculate the needed PEC codes for the write data
 and then transmit data to the ICs on a daisy chain.


Command Code:
-------------
|               |             CMD[0]                              |                            CMD[1]                             |
|---------------|---------------------------------------------------------------|---------------------------------------------------------------|
|CMD[0:1]     |  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|---------------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|WRCFG:         |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   1   |
********************************************************/
void LTC6806_wrcfg(uint8_t total_ic, //The number of ICs being written to
                   uint8_t config[][6] //A two dimensional array of the configuration data that will be written
                  )
{
  uint8_t BYTES_IN_REG = 6;
  uint8_t CMD_LEN = 4+(8*total_ic);
  uint8_t *cmd;
  uint16_t cfg_pec;
  uint8_t cmd_index; //command counter

  cmd = (uint8_t *)malloc(CMD_LEN*sizeof(uint8_t));

  cmd[0] = 0x00;
  cmd[1] = 0x01;
  cmd[2] = 0x3d;
  cmd[3] = 0x6e;

  cmd_index = 4;
  for (uint8_t current_ic = total_ic; current_ic > 0; current_ic--)       // executes for each LTC6806 in daisy chain, this loops starts with
  {
    // the last IC on the stack. The first configuration written is
    // received by the last IC in the daisy chain

    for (uint8_t current_byte = 0; current_byte < BYTES_IN_REG; current_byte++) // executes for each of the 6 bytes in the CFGR register
    {
      // current_byte is the byte counter

      cmd[cmd_index] = config[current_ic-1][current_byte];            //adding the config data to the array to be sent
      cmd_index = cmd_index + 1;
    }
    cfg_pec = (uint16_t)pec15_calc(BYTES_IN_REG, &config[current_ic-1][0]);   // calculating the PEC for each ICs configuration register data
    cmd[cmd_index] = (uint8_t)(cfg_pec >> 8);
    cmd[cmd_index + 1] = (uint8_t)cfg_pec;
    cmd_index = cmd_index + 2;
  }

  output_low(LTC6806_CS);
  spi_write_array(CMD_LEN, cmd);
  output_high(LTC6806_CS);
  free(cmd);
}


/*!******************************************************
 \brief Reads configuration registers of a LTC6806 daisy chain

@param[in] uint8_t total_ic: number of ICs in the daisy chain

@param[out] uint8_t r_config[][8] is a two dimensional array that the function stores the read configuration data. The configuration data for each IC
is stored in blocks of 8 bytes with the configuration data of the lowest IC on the stack in the first 8 bytes
block of the array, the second IC in the second 8 byte etc. Below is an table illustrating the array organization:

|r_config[0][0]|r_config[0][1]|r_config[0][2]|r_config[0][3]|r_config[0][4]|r_config[0][5]|r_config[0][6]  |r_config[0][7] |r_config[1][0]|r_config[1][1]|  .....    |
|--------------|--------------|--------------|--------------|--------------|--------------|----------------|---------------|--------------|--------------|-----------|
|IC1 CFGR0     |IC1 CFGR1     |IC1 CFGR2     |IC1 CFGR3     |IC1 CFGR4     |IC1 CFGR5     |IC1 PEC High    |IC1 PEC Low    |IC2 CFGR0     |IC2 CFGR1     |  .....    |


@return int8_t, PEC Status.

  0: Data read back has matching PEC

  -1: Data read back has incorrect PEC


Command Code:
-------------

|CMD[0:1]   |  15   |  14   |  13   |  12   |  11   |  10   |   9   |   8   |   7   |   6   |   5   |   4   |   3   |   2   |   1   |   0   |
|---------------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|-------|
|RDCFG:         |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   0   |   1   |   0   |   0   |   1   |   0   |
********************************************************/
int8_t LTC6806_rdcfg(uint8_t total_ic, //Number of ICs in the system
                     uint8_t r_config[][8] //A two dimensional array that the function stores the read configuration data.
                    )
{
  uint8_t BYTES_IN_REG = 8;

  uint8_t cmd[4];
  uint8_t *rx_data;
  int8_t pec_error = 0;
  uint16_t data_pec;
  uint16_t received_pec;

  rx_data = (uint8_t *) malloc((8*total_ic)*sizeof(uint8_t));

  cmd[0] = 0x00;
  cmd[1] = 0x02;
  cmd[2] = 0x2b;
  cmd[3] = 0x0A;

  output_low(LTC6806_CS);
  spi_write_read(cmd, 4, rx_data, (BYTES_IN_REG*total_ic));         //Read the configuration data of all ICs on the daisy chain into
  output_high(LTC6806_CS);                          //rx_data[] array

  for (uint8_t current_ic = 0; current_ic < total_ic; current_ic++)       //executes for each LTC6806 in the daisy chain and packs the data
  {
    //into the r_config array as well as check the received Config data
    //for any bit errors

    for (uint8_t current_byte = 0; current_byte < BYTES_IN_REG; current_byte++)
    {
      r_config[current_ic][current_byte] = rx_data[current_byte + (current_ic*BYTES_IN_REG)];
    }

    received_pec = (r_config[current_ic][6]<<8) + r_config[current_ic][7];
    data_pec = pec15_calc(6, &r_config[current_ic][0]);
    if (received_pec != data_pec)
    {
      pec_error = -1;
    }
  }

  free(rx_data);

  return(pec_error);
}

/*!****************************************************
  \brief Wake isoSPI up from idle state
 Generic wakeup commannd to wake isoSPI up out of idle
 *****************************************************/
void wakeup_idle(uint8_t total_ic)
{
  for (int i =0; i<total_ic; i++)
  {
    output_low(LTC6806_CS);
    delayMicroseconds(2); //Guarantees the isoSPI will be in ready mode
    output_high(LTC6806_CS);
  }
}

/*!****************************************************
  \brief Wake the LTC6806 from the sleep state

 Generic wakeup commannd to wake the LTC6806 from sleep
 *****************************************************/
void wakeup_sleep(uint8_t total_ic)
{
  for (int i =0; i<total_ic; i++)
  {
    output_low(LTC6806_CS);
    delayMicroseconds(300); // Guarantees the ltc6813 will be in standby
    output_high(LTC6806_CS);
  }
}
/*!**********************************************************
 \brief calaculates  and returns the CRC15

  @param[in] uint8_t len: the length of the data array being passed to the function

  @param[in] uint8_t data[] : the array of data that the PEC will be generated from


  @returns The calculated pec15 as an unsigned int
***********************************************************/
uint16_t pec15_calc(uint8_t len, //Number of bytes that will be used to calculate a PEC
                    uint8_t *data //Array of data that will be used to calculate  a PEC
                   )
{
  uint16_t remainder,addr;

  remainder = 16;//initialize the PEC
  for (uint8_t i = 0; i<len; i++) // loops for each byte in data array
  {
    addr = ((remainder>>7)^data[i])&0xff;//calculate PEC table address
    remainder = (remainder<<8)^pgm_read_word_near(crc15Table+addr);
  }
  return(remainder*2);//The CRC15 has a 0 in the LSB so the remainder must be multiplied by 2
}


int16_t conver12to16(uint16_t data)
{
  if ((data>>11) == 1) return(data | 0xF000);
  else return(data);
}

/*!
 \brief Writes an array of bytes out of the SPI port

 @param[in] uint8_t len length of the data array being written on the SPI port
 @param[in] uint8_t data[] the data array to be written on the SPI port

*/
void spi_write_array(uint8_t len, // Option: Number of bytes to be written on the SPI port
                     uint8_t data[] //Array of bytes to be written on the SPI port
                    )
{
  for (uint8_t i = 0; i < len; i++)
  {
    spi_write((int8_t)data[i]);
  }
}

/*!
 \brief Writes and read a set number of bytes using the SPI port.

@param[in] uint8_t tx_data[] array of data to be written on the SPI port
@param[in] uint8_t tx_len length of the tx_data array
@param[out] uint8_t rx_data array that read data will be written too.
@param[in] uint8_t rx_len number of bytes to be read from the SPI port.

*/

void spi_write_read(uint8_t tx_Data[],//array of data to be written on SPI port
                    uint8_t tx_len, //length of the tx data arry
                    uint8_t *rx_data,//Input: array that will store the data read by the SPI port
                    uint8_t rx_len //Option: number of bytes to be read from the SPI port
                   )
{
  for (uint8_t i = 0; i < tx_len; i++)
  {
    spi_write(tx_Data[i]);

  }

  for (uint8_t i = 0; i < rx_len; i++)
  {
    rx_data[i] = (uint8_t)spi_read(0xFF);
  }

}

