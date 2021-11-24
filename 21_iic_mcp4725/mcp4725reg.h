#ifndef mcp4725_H
#define mcp4725_H

#define MCP4725_DISABLE_SANITY_CHECK     //disable some sanity checks to increase speed, use with caution

/* dac addresses */
typedef enum
{
  MCP4725A0_ADDR_A00         = 0x60,                              //i2c address, A0 = 0
  MCP4725A0_ADDR_A01         = 0x61,                              //i2c address, A0 = 1

  MCP4725A1_ADDR_A00         = 0x62,                              //i2c address, A0 = 0
  MCP4725A1_ADDR_A01         = 0x63,                              //i2c address, A0 = 1

  MCP4725A2_ADDR_A00         = 0x64,                              //i2c address, A0 = 0
  MCP4725A2_ADDR_A01         = 0x65                               //i2c address, A0 = 1
}
MCP4725Ax_ADDRESS;

/* dac register, command bits */
typedef enum
{
  MCP4725_FAST_MODE          = 0x00,                              //writes data to DAC register
  MCP4725_REGISTER_MODE      = 0x40,                              //writes data & configuration bits to DAC register
  MCP4725_EEPROM_MODE        = 0x60                               //writes data & configuration bits to DAC register & EEPROM
}
MCP4725_COMMAND_TYPE;

/* dac register, power down bits */
typedef enum
{
  MCP4725_POWER_DOWN_OFF     = 0x00,                              //power down off
  MCP4725_POWER_DOWN_1KOHM   = 0x01,                              //power down on, with 1.0 kOhm to ground
  MCP4725_POWER_DOWN_100KOHM = 0x02,                              //power down on, with 100 kOhm to ground
  MCP4725_POWER_DOWN_500KOHM = 0x03                               //power down on, with 500 kOhm to ground
}
MCP4725_POWER_DOWN_TYPE;

/* dac library specific command */
typedef enum
{
  MCP4725_READ_SETTINGS      = 1,                                 //read 1 byte,  settings data
  MCP4725_READ_DAC_REG       = 3,                                 //read 3 bytes, DAC register data
  MCP4725_READ_EEPROM        = 5                                  //read 5 bytes, EEPROM data
}
MCP4725_READ_TYPE;

/* Some arithmatic functions */
#define lowByte(x)		((uint8_t)(x%256))
#define highByte(x)		((uint8_t)(x/256))

/* dac general call command */
#define MCP4725_GENERAL_CALL_ADDRESS 0x00                         //general call address
#define MCP4725_GENERAL_CALL_RESET   0x06                         //general call hard reset command
#define MCP4725_GENERAL_WAKE_UP      0x09                         //general call wake-up command

/* dac mics. */
#define MCP4725_RESOLUTION           12                           //resolution 12-bit
#define MCP4725_STEPS                4096 //pow(2, (MCP4725_RESOLUTION)) //quantity of DAC steps 2^12-bits = 4096
#define MCP4725_EEPROM_WRITE_TIME    25                           //non-volatile memory write time, maximum 50 msec

#define MCP4725_REFERENCE_VOLTAGE    3.30                         //supply-reference votltage
#define MCP4725_MAX_VALUE            4095 //((MCP4725_STEPS) - 1)
#define MCP4725_ERROR                0xFFFF                       //returns 65535, if communication error is occurred


#endif

