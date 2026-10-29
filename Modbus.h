/*MIT License

Copyright (c) 2026 masoud hamadi

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.*/
#ifndef MODBUS_H
#define MODBUS_H

#include "main.h"
#include <stdbool.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"

#ifndef SLAVE_ID_STRING
#define SLAVE_ID_STRING "example"
#endif


#define VENDOR_NAME        "example"
#define PRODUCT_CODE       "example "
#define MAJOR_VERSION      1
#define MINOR_VERSION      0
#define REVISION           0
#define SERIAL_NUMBER      "SN123456789"

#define OBJID_VENDOR_NAME     0x00
#define OBJID_PRODUCT_CODE    0x01
#define OBJID_MAJOR_MINOR     0x02
#define OBJID_VENDOR_URL      0x03
#define OBJID_PRODUCT_NAME    0x04
#define OBJID_MODEL_NAME      0x05
#define OBJID_USER_APP_NAME   0x06
#define OBJID_SERIAL_NUMBER   0x07
#define OBJID_FIRMWARE_REV    0x08  // Optional, but we'll use 0x02 for version

#ifndef FIRMWARE_VERSION_MAJOR
#define FIRMWARE_VERSION_MAJOR 1
#define FIRMWARE_VERSION_MINOR 0
#endif

#define FC11_COMPAT_MODE    (1)  // 1 = 6 bytes (for your Master), 0 = 5 bytes (standard)

#define MAX_BUFFER       128
#define DB_COUNT     (13U)
#define COILS_PER_DB (56U)
#define REGS_PER_DB  (32U)

/* PLC-style addressing: raw = db*1000 + offset */
#define MB_ADDR_DB_DIVISOR 1000U
#define MB_ADDR_DECODE_DB(raw)     ((uint8_t)((raw) / MB_ADDR_DB_DIVISOR))
#define MB_ADDR_DECODE_OFFSET(raw) ((uint16_t)((raw) % MB_ADDR_DB_DIVISOR))

#define MODBUS_T35_MS     (5U)
#define MODBUS_TIMEOUT_MS (100U)

typedef enum { USART_HW_DMA = 4 } mb_hardware_t;

typedef struct {
    uint8_t  uxBuffer[MAX_BUFFER];
    uint16_t u8available;
    bool     overflow;
} mb_rx_dma_buffer_t;

typedef struct {
    UART_HandleTypeDef *port;
    uint8_t  u8id;
    GPIO_TypeDef* EN_Port;
    uint16_t EN_Pin;

    uint8_t u8Buffer[MAX_BUFFER];
    uint8_t u8BufferSize;

    uint16_t u16InCnt, u16OutCnt, u16errCnt;

    TaskHandle_t myTaskModbusAHandle;
    mb_hardware_t xTypeHW;
    uint16_t u16timeOut;

    mb_rx_dma_buffer_t xBufferRX;
} modbusHandler_t;

/* Global single handler */
extern modbusHandler_t gModbusH;

/* Static DBs */
extern uint8_t  gCoils[DB_COUNT][COILS_PER_DB / 8];
extern uint16_t gRegs[DB_COUNT][REGS_PER_DB];

void ModbusInit(UART_HandleTypeDef *huart,
                uint8_t slave_id,
                GPIO_TypeDef *en_port,
                uint16_t en_pin);

void ModbusStart(void);
void StartTaskModbusSlave(void *argument);

uint16_t calcCRC(uint8_t *Buffer, uint8_t u8length);

bool     mb_read_coil(uint8_t db, int16_t address);
void     mb_write_coil(uint8_t db, int16_t address, bool state);
uint16_t mb_read_register(uint8_t db, int16_t address);
void     mb_write_register(uint8_t db, int16_t address, const uint16_t *data, uint16_t len);

#endif
