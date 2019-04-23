/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "string.h"

#define MODDEF_XPT2046_WIDTH  240
#define MODDEF_XPT2046_HEIGHT 320

#define PIN_NUM_CS 16
#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23 
#define PIN_NUM_CLK 18

#define Z_THRESHOLD 0

#define XMIN 250
#define XMAX 3722
#define YMIN 250
#define YMAX 3760

// bit 7 - start bit
// bit 6-4 A2-A0 - Channel select bits
// bit 3 - mode (low for 12bit, high for 8bit)
// bit 2 - single ended / differential reference
// bit 1-0 - power down mode (00 - enabled, 01 ref off, adc on, 10 ref on, adc off, 11-always on)
#define CTRLZ1 0b10110011 		// 0xB3 = 179				// 10110011
#define CTRLZ2 0b11000011 		// 0xC3 = 195				// 11000011
#define CTRLY  0b10010011 		// 0x93 = 147				// 10010011
#define CTRLX  0b11010011 		// 0xD3 = 211				// 11010011
#define CTRL_RESET 0b11010100	// 0xD4 = 212		    // 11010100

esp_err_t ret;
uint16_t x, y, z;

uint16_t readValue1();
uint16_t readValue2();
uint16_t readValue3();

spi_device_handle_t dev;
spi_device_handle_t lcd_dev;

spi_bus_config_t buscfg={
  .miso_io_num=PIN_NUM_MISO,
  .mosi_io_num=PIN_NUM_MOSI,
  .sclk_io_num=PIN_NUM_CLK,
  .quadwp_io_num=-1,
  .quadhd_io_num=-1
};

void app_main()
{
  //Initialize the SPI bus

  gpio_set_direction(PIN_NUM_CS, GPIO_MODE_OUTPUT);
  gpio_set_level(PIN_NUM_CS, 1);
  ret = spi_bus_initialize(VSPI_HOST, &buscfg, 0);
  assert(ret == ESP_OK);

  while(1){
    uint16_t x;
    
    x = readValue1(); // get the position
    printf("Read1 x: %d", x);
    x = readValue2(); // get the position
    printf(", Read2 x: %d", x);
    x = readValue3(); // get the position
    printf(", Read3 x: %d\n", x);

      /* Block */
    const TickType_t xDelay = 500 / portTICK_PERIOD_MS;
    vTaskDelay( xDelay );    
  }
}

// N.B. callers assume this function is synchronous - modSPITxRx
// Send three bytes and receieve buffer is the transmit buffer
uint16_t readValue1()
{
    spi_device_interface_config_t devcfg = {
    .clock_speed_hz = 1000000,
    .mode = 0,
    .spics_io_num = PIN_NUM_CS,
    .queue_size = 7,
  };
  
  // Add the XPT2046
  ret = spi_bus_add_device(VSPI_HOST, &devcfg, &dev);
  assert(ret == ESP_OK);

  uint8_t data[] = {CTRLX, 0 , 0};
  
  esp_err_t ret;
	spi_transaction_t t;

	memset(&t, 0, sizeof(t));
	t.length = 8 * sizeof(data);
	t.tx_buffer = &data;

	t.rxlength = t.length;
	t.rx_buffer = &data;

	ret =  spi_device_transmit(dev, &t);
  printf("Byte 0: %d, Byte 1: %d, Byte 2: %d\n", data[0], data[1], data[2]);
	
	if (0 > ret)
		printf("problems sending spi message: ret: %d\n", ret);

  ret = spi_bus_remove_device(dev);
  assert(ret == ESP_OK);

  uint16_t val = (data[1] << 8 | data[2]) >> 3;
  return val;
}

// Send the command alone and read 2 bytes using half-duplex 

uint16_t readValue2(){

  spi_device_interface_config_t devcfg = {
    .clock_speed_hz = 1000000,
    .mode = 0,
    .spics_io_num = PIN_NUM_CS,
    .queue_size = 7,
    .flags = SPI_DEVICE_HALFDUPLEX,
  };

  // Add the XPT2046
  ret = spi_bus_add_device(VSPI_HOST, &devcfg, &dev);
  assert(ret == ESP_OK);

  uint8_t data[1] = {CTRLX};

  spi_transaction_t t = {
    .length = 1 * 8,
    .tx_buffer = &data,
    .flags = SPI_TRANS_USE_RXDATA,
    .rxlength = 16
  };
  
  ret = spi_device_transmit(dev, &t);
  assert(ret == ESP_OK);
  uint16_t val = (t.rx_data[0] << 8 | t.rx_data[1]) >> 3;

  //printf("Byte 0: %d, Byte 1: %d, Byte 2: %d", trans.rx_data[0], trans.rx_data[1], trans.rx_data[2]);
  //printf(" val: %d\n", val);   

  ret = spi_bus_remove_device(dev);
  assert(ret == ESP_OK);
  return val;
}

// Send the command alone and read 2 bytes using half-duplex 
// Use command syntax

uint16_t readValue3()
{
    /**
     * Half duplex mode is not compatible with DMA when both writing and reading phases exist.
     * try to use command and address field to replace the write phase.
    */

  spi_device_interface_config_t devcfg = {
    .clock_speed_hz = 1000000,
    .mode = 0,
    .spics_io_num = PIN_NUM_CS,
    .queue_size = 7,
    .flags = SPI_DEVICE_HALFDUPLEX,
    .command_bits = 8
  };

  // Add the XPT2046
  ret = spi_bus_add_device(VSPI_HOST, &devcfg, &dev);
  assert(ret == ESP_OK);

  const uint8_t command = CTRLX;

  spi_transaction_ext_t t = (spi_transaction_ext_t) {
      .base = {
          .flags = SPI_TRANS_VARIABLE_CMD | SPI_TRANS_USE_RXDATA,
          .cmd = command,
          .rxlength = 2 * 8, // Total data length received, in bits
      },
      .command_bits = 8,
      .address_bits=0
  };

  assert(spi_device_transmit(dev, (spi_transaction_t *)&t) == ESP_OK);
  ret = spi_bus_remove_device(dev);
  assert(ret == ESP_OK);
  //printf("Byte 0: %d, Byte 1: %d\n", t.base.rx_data[0], t.base.rx_data[1]);

  return (t.base.rx_data[0] << 8 | t.base.rx_data[1]) >> 3;
}
