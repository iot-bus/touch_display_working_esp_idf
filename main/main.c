#include "esp32-hal-spi.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define SS 16
#define MISO 19
#define MOSI 23 
#define SCK 18

#define WIDTH 240
#define HEIGHT 320

#define min_x 250
#define max_x 3600

#define min_y 250
#define max_y 3600

//#define SPI_SETTING     SPISettings(2000000, MSBFIRST, SPI_MODE0)

// bit 7 - start bit
// bit 6-4 A2-A0 - Channel select bits
// bit 3 - mode (low for 12bit, high for 8bit)
// bit 2 - single ended / differential reference
// bit 1-0 - power down mode (00 - enabled, 01 ref off, adc on, 10 ref on, adc off, 11-always on)
#define CTRLZ1 0b10110011 		// 0xB3 = 179				// 10110011
#define CTRLZ2 0b11000011 		// 0xC3 = 195				// 11000011
#define CTRLY  0b10010011 		// 0x93 = 147				// 10010011
#define CTRLX  0b11010011 		// 0xD3 = 211				// 11010011
#define CTRL_RESET 0b11010100	// 0xD4 = 212		        // 11010100

spi_t * _spi;
uint8_t _sck;
uint8_t _miso;
uint8_t _mosi;
uint8_t _ss;

int8_t _spi_num = VSPI;
uint32_t _div;
uint32_t _freq = 2000000;
bool _inTransaction;

const TickType_t xDelay = 500 / portTICK_PERIOD_MS;

uint16_t readValue(uint8_t cmd);

void begin(int8_t sck, int8_t miso, int8_t mosi, int8_t ss);
void transfer(uint8_t * data, uint32_t size);
void transferBytes(uint8_t * data, uint8_t * out, uint32_t size);
void mapValues(uint16_t* x, uint16_t* y);

void loop();

void app_main()
{
  gpio_set_direction(SS, GPIO_MODE_OUTPUT);
  gpio_set_level(SS, 1);
  begin(SCK, MISO, MOSI, SS);

  while(1){
      loop();
  }
}

void loop() {
 
  uint16_t z1 = readValue(CTRLZ1);
  if(z1){
    uint16_t x = readValue(CTRLX);
    printf("pressed raw x: %d", x);

    uint16_t y = readValue(CTRLY);
    printf(", raw y: %d", x);

    mapValues(&x, &y);

    printf(", x: %d", x);
    printf(", y: %d\n", y);

  }
  
  vTaskDelay( xDelay );  
}

uint16_t readValue(uint8_t cmd){

  uint8_t data[3];
  data[0] = cmd;
  data[1] = data[2] = 0;

  gpio_set_level(SS, 0);
  transfer((uint8_t*)&data, sizeof(data));
  gpio_set_level(SS, 1);

  uint16_t val = (data[1] << 8 | data[2]) >> 3;
//  printf("data[0]: %d, data[1]: %d, data[2]: %d\n", data[0], data[1], data[2]);
  return val;
}

void mapValues(uint16_t* x, uint16_t* y){
    *x = (*x - min_x) * ((float)WIDTH) / (max_x - min_x);
	*y = (*y - min_y) * ((float)HEIGHT) / (max_y - min_y);

	if (*x > (WIDTH - 1)) *x = WIDTH - 1;
	if (*y > (HEIGHT - 1)) *y = HEIGHT - 1;
}

void begin(int8_t sck, int8_t miso, int8_t mosi, int8_t ss){
    if(_spi) {
        return;
    }

    if(!_div) {
        _div = spiFrequencyToClockDiv(_freq);
    }

    _spi = spiStartBus(_spi_num, _div, SPI_MODE0, SPI_MSBFIRST);
    if(!_spi) {
        return;
    }

    if(sck == -1 && miso == -1 && mosi == -1 && ss == -1) {
        _sck = (_spi_num == VSPI) ? SCK : 14;
        _miso = (_spi_num == VSPI) ? MISO : 12;
        _mosi = (_spi_num == VSPI) ? MOSI : 13;
        _ss = (_spi_num == VSPI) ? SS : 15;
    } else {
        _sck = sck;
        _miso = miso;
        _mosi = mosi;
        _ss = ss;
    }

    spiAttachSCK(_spi, _sck);
    spiAttachMISO(_spi, _miso);
    spiAttachMOSI(_spi, _mosi);
}

void transfer(uint8_t * data, uint32_t size) 
{ 
	transferBytes(data, data, size); 
}

/**
 * @param data uint8_t * data buffer. can be NULL for Read Only operation
 * @param out  uint8_t * output buffer. can be NULL for Write Only operation
 * @param size uint32_t
 */
void transferBytes(uint8_t * data, uint8_t * out, uint32_t size)
{
    if(_inTransaction){
        return spiTransferBytesNL(_spi, data, out, size);
    }
    spiTransferBytes(_spi, data, out, size);
}

