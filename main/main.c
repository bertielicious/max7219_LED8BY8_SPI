
#include <stdio.h>                      // Standard I/O functions
#include "esp_log.h"                    // ESP-IDF logging macros
#include "freertos/FreeRTOS.h"          // FreeRTOS base definitions
#include "freertos/task.h"              // Task management
#include "driver/spi_master.h"          // SPI master driver
#include <stdbool.h>


//Defines the SPI pin mapping. These GPIOs connect to the MAX7219 module.
#define MOSI 23                         // GPIO for SPI MOSI
#define CLK 18                          // GPIO for SPI Clock
#define CS  5                           // GPIO for Chip Select (CS)

//These are register addresses for configuring the MAX7219. 
//You're using raw mode (no BCD decoding), full scan, and disabling test mode.
#define DECODE_MODE_REG     0x09       // MAX7219 decode mode register
#define INTENSITY_REG       0x0A       // Brightness control
#define SCAN_LIMIT_REG      0x0B       // Number of digits to scan (0–7)
#define SHUTDOWN_REG        0x0C       // Shutdown control
#define DISPLAY_TEST_REG    0x0F       // Display test mode

//This is the SPI device handle used for transactions with the MAX7219.
spi_device_handle_t phil;// Handle for SPI device

//Start SPI bus and device configuration
static void spi_init() {
    esp_err_t ret;

    spi_bus_config_t buscfg={
        .miso_io_num = -1,  //Configures the SPI bus. Only MOSI and CLK are needed for MAX7219
        .mosi_io_num = MOSI,
        .sclk_io_num = CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0,
    };

    //Initializes SPI2 bus with automatic DMA channel selection. 
    ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret); //Checks for errors

    //Device configuration for MAX7219
    spi_device_interface_config_t devcfg={
        .clock_speed_hz = 10000000,  // 1 MHz
        .mode = 0,                  //SPI mode 0
        .spics_io_num = CS,     
        .queue_size = 1,
        .flags = SPI_DEVICE_HALFDUPLEX,
        .pre_cb = NULL,
        .post_cb = NULL,
    };
    //Adds MAX7219 as a device on SPI2 and stores the handle in phil
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &phil));
};

//Prepares a 2-byte command: register address + value
static void write_reg(uint8_t reg, uint8_t value) {
    uint8_t tx_data[2] = { reg, value };

//Creates a blocking SPI transaction
    spi_transaction_t t = {
        .tx_buffer = tx_data,
        .length = 16,
    };
    //Sends the transaction and waits for completion. Ensures reliability
    ESP_ERROR_CHECK(spi_device_polling_transmit(phil, &t));
}

//Lights up a specific row (1–8) by writing 0xFF to that digit register
static void set_row(uint8_t row_index) {
  write_reg(1,row_index);
}

//Lights up a specific column by shifting a bit in each row
static void set_col(uint8_t col_index) {
 for (int i = 0; i < 8; i++) {
    write_reg(i+1, 1<<col_index);
  }
}

//Clears the display by writing 0x00 to all digit registers
static void clear(void) {
  for (int i = 0; i < 8; i++) {
    write_reg(i + 1, 0x00);
  }
}

static void max7219_init() {
    write_reg(DISPLAY_TEST_REG, 0); // Disable test mode
    write_reg(SCAN_LIMIT_REG, 7); // Scan enable all 8 digits
    write_reg(DECODE_MODE_REG, 0); // No BCD decoding
    write_reg(INTENSITY_REG, 1); // Set medium brightness
    write_reg(SHUTDOWN_REG, 1); // Exit shutdown mode
    clear();
}

//char splash [8] = {0x00, 0x00, 0x8c, 0x92, 0x92, 0x62, 0x00, 0x00};
char zero[8] = {0x00,0x00,0x7e,0x81,0x81,0x7a,0x00,0x00};
char one[8] = {0x00,0x00,0x84,0x82,0xFF,0x80,0x00,0x00};
char two[8] = {0x00,0x00,0xc2,0xa1,0x91,0x8e,0x00,0x00};
char three[8] = {0x00,0x00,0x42,0x81,0x89,0x76,0x00,0x00};
char four[8] = {0x00,0x00,0x38,0x26,0xf1,0x20,0x00,0x00};
char five[8] = {0x00,0x00,0x47,0x89,0x89,0x71,0x00,0x00};
char six[8] = {0x00,0x00,0x7e,0x89,0x89,0x70,0x00,0x00};
char seven[8] = {0x00,0x00,0x03,0xe1,0x19,0x07,0x00,0x00};
char eight[8] = {0x00,0x00,0x076,0x89,0x89,0x76,0x00,0x00};
char nine[8] = {0x00,0x00,0x8e,0x91,0x51,0x3e,0x00,0x00};






void app_main(void)
{
  printf("Hello MAX7219 SPI\n");
 
    spi_init(); // Set up SPI bus and device

    max7219_init();// Configure MAX7219
    // Display splash pattern
    
   
    while (1) 
    {
      for (int i = 0; i < 8; i++) {
      write_reg(i + 1, zero[i]);
    }
      vTaskDelay(1000 / portTICK_PERIOD_MS);

      for (int i = 0; i < 8; i++) {
      write_reg(i + 1, one[i]);
    }
      vTaskDelay(1000 / portTICK_PERIOD_MS);

      for (int i = 0; i < 8; i++) {
      write_reg(i + 1, two[i]);
    }
      vTaskDelay(1000 / portTICK_PERIOD_MS);


      for (int i = 0; i < 8; i++) {
      write_reg(i + 1, three[i]);
    }
      vTaskDelay(1000 / portTICK_PERIOD_MS);

       for (int i = 0; i < 8; i++) {
      write_reg(i + 1, four[i]);
    }
      vTaskDelay(1000 / portTICK_PERIOD_MS);


       for (int i = 0; i < 8; i++) {
      write_reg(i + 1, five[i]);
    }
      vTaskDelay(1000 / portTICK_PERIOD_MS);


       for (int i = 0; i < 8; i++) {
      write_reg(i + 1, six[i]);
    }
      vTaskDelay(1000 / portTICK_PERIOD_MS);

      for (int i = 0; i < 8; i++) {
      write_reg(i + 1, seven[i]);
    }
      vTaskDelay(1000 / portTICK_PERIOD_MS);

      for (int i = 0; i < 8; i++) {
      write_reg(i + 1, eight[i]);
    }
      vTaskDelay(1000 / portTICK_PERIOD_MS);

      for (int i = 0; i < 8; i++) {
      write_reg(i + 1, nine[i]);
    }
      vTaskDelay(1000 / portTICK_PERIOD_MS);


    }
}