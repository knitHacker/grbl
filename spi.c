/* 
  Not part of GRBL, KeyMe specific
*/

#include "spi.h"

#define SPI_TRANSFER_FLAG (SPSR & (1<<SPIF))


//Setup spi
void spi_init()
{

  DDRB = ((1 << DDB2) | (1 << DDB1) | (1 << DDB0)); //spi pins on port b MOSI SCK,SS outputs

  SPCR = ((1 << SPE) | (1 << MSTR) | (1 << SPR0) | (1 << CPOL) | (1 << CPHA));  // SPI enable, Master, f/16

  //Configure SCS Pins as outputs
  bit_true(SCS_DDR_PORT, SCS_DDR_MASK); 

  //Set SCS to low for all steppers
  bit_false(SCS_PORT, SCS_MASK);

}

void spi_transact_array(uint8_t * dataout, uint8_t * datain, uint8_t len)
{

  uint8_t idx;

  for(idx = 0; idx < len; idx++) {

    if (dataout != NULL) {
      SPDR = dataout[idx];  //SPDR - SPI Data register
    } else {
      SPDR = 0;    
    }

    uint16_t timeout = 0xFFFF;
    while(!(SPI_TRANSFER_FLAG)) {
      if (timeout-- == 0) {
        return;     
      }
    } //Wait for transmit to be completed

    if (datain != NULL) {
      datain[idx] = SPDR;
    } 
  
  }
}

void spi_read(uint8_t * datain, uint8_t len) 
{
  spi_transact_array(NULL, datain, len);
}

void spi_write(uint8_t * dataout, uint8_t len)
{
  spi_transact_array(dataout, NULL, len);
}

