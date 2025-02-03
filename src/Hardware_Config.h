#ifndef Hardware_Config_h
#define Hardware_Config_h
#define WIRELESS_TRACKER

#include <arduino.h>

//Dedicated i2c pins
#define I2C_SDA                7 
#define I2C_SCL                6

//Deployment trigger pins
#define droguepin              4     //pin for triggering drogue pyro channel
#define mainpin                5     //pin for triggering main pyro channel

//Onboard LED
#define LEDpin                 18

//GPS pins amd baud rate
#define VGNSS_CTRL             3 
#define GNSS_RX                33
#define GNSS_TX                34
#define GNSS_RST               35
#define GNSS_PPS               36
#define GNSS_BAUD              115200

//LoRa config
#define LORA_CS                8
#define LORA_SCK               9
#define LORA_MOSI              10
#define LORA_MISO              11
#define LORA_RESET             12
#define LORA_DIO0              -1    // a No connect on the SX1262 module
#define LORA_DIO1              14    // SX1262 IRQ
#define LORA_DIO2              13    // SX1262 BUSY
#define LORA_DIO3                    // Not connected on PCB, but internally on the TTGO SX1262, if DIO3 is high the TXCO is enabled
#define LORA_SYNC              0x34       
#define LORA_BAND              915.0 // Adjust to your region (e.g., 433E6, 868E6, or 915E6)
#define TX_OUTPUT_POWER        5     // dBm
#define TX_VOLTAGE             1.6
#define LORA_BANDWIDTH         125   // [0: 125 kHz,
#define LORA_SPREAD_FACTOR     7     // [SF7..SF12]
#define LORA_PREAMBLE          8       
#define LORA_CODING_RATE       5     // [1: 4/5, 2: 4/6, 3: 4/7, 4: 4/8]
#define RX_TIMEOUT_VALUE       1000
#define BUFFER_SIZE            30 
#define BOARD_TCXO_WAKEUP_TIME 5

//SD config
#define SD_FAT_TYPE 0 //SPI_DRIVER_SELECT must be changed to 2 in SdFatConfig.h to specify SPI pins - see SoftwareSpi example from SdFat
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(0), &softSpi) //dedicated use of SPI bus for SdFat allows better performance by using very large multi-block transfers to and from the SD card

//Required for reading battery voltage
#define ADC_CTRL 2  // Pin to enable voltage divider
#define ADC_MULTIPLIER 1.983 //Calibration adjustment

#endif