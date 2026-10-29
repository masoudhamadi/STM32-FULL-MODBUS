/******************************************************************************
 * file name: modbus.c
 * details :
 * version: 4.0
 * date:8/20/2026
 * programmer : masoud hamadi (public library)
 ******************************************************************************/
#if 1
#include "Modbus.h"
#include "string.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "cmsis_os.h"


modbusHandler_t gModbusH;
uint8_t  gCoils[DB_COUNT][COILS_PER_DB / 8] = {0};  //FC1
//uint8_t  gDiscreteInputs[DB_COUNT][COILS_PER_DB / 8] = {0};  // FC2
uint16_t gRegs[DB_COUNT][REGS_PER_DB] = {0};                  // FC3, FC6, ...
//uint16_t gInputRegs[DB_COUNT][REGS_PER_DB] = {0};             // FC4

static void transmit_response(void);
static uint16_t word(uint8_t H, uint8_t L) { return ((uint16_t)H << 8) | (uint16_t)L; }
#endif
#if 1
uint16_t calcCRC(uint8_t *buf, uint8_t len) {
    uint16_t crc = 0xFFFFU;
    for (uint8_t i = 0U; i < len; i++) {
        crc ^= (uint16_t)buf[i];
        for (uint8_t j = 0U; j < 8U; j++) {
            if (crc & 0x0001U) crc = (uint16_t)((crc >> 1) ^ 0xA001U);
            else crc >>= 1;
        }
    }
    return crc;
}

void ModbusInit(UART_HandleTypeDef *huart,uint8_t slave_id,GPIO_TypeDef *en_port, uint16_t en_pin){
    gModbusH.port    = huart;
    gModbusH.u8id    = slave_id;
    gModbusH.EN_Port = en_port;
    gModbusH.EN_Pin  = en_pin;
    gModbusH.xTypeHW = USART_HW_DMA;
    gModbusH.u16timeOut  = MODBUS_TIMEOUT_MS;
    gModbusH.u8BufferSize = 0U;
    gModbusH.u16InCnt = 0U;
    gModbusH.u16OutCnt = 0U;
    gModbusH.u16errCnt = 0U;
    memset(&gModbusH.xBufferRX, 0, sizeof(gModbusH.xBufferRX));
}

void ModbusStart(void) {
    if (gModbusH.EN_Port != NULL){HAL_GPIO_WritePin(gModbusH.EN_Port, gModbusH.EN_Pin, GPIO_PIN_RESET);}
    HAL_UARTEx_ReceiveToIdle_DMA(gModbusH.port, gModbusH.xBufferRX.uxBuffer, MAX_BUFFER);
    if (gModbusH.port->hdmarx != NULL) {__HAL_DMA_DISABLE_IT(gModbusH.port->hdmarx, DMA_IT_HT);}
}

#endif
#if 1
static void process_FC1(void) {
    uint16_t rawAddr   = word(gModbusH.u8Buffer[2], gModbusH.u8Buffer[3]);
    uint8_t  db        = MB_ADDR_DECODE_DB(rawAddr);
    uint16_t startCoil = MB_ADDR_DECODE_OFFSET(rawAddr);
    uint16_t coilCount = word(gModbusH.u8Buffer[4], gModbusH.u8Buffer[5]);

    if (coilCount == 0U || coilCount > 2000U) {
        gModbusH.u16errCnt++;
        return;
    }
    if ((db >= DB_COUNT) || ((uint32_t)startCoil + coilCount > COILS_PER_DB)) {
        gModbusH.u16errCnt++;
        return;
    }


    uint8_t byteCount = (uint8_t)((coilCount + 7U) / 8U);
    gModbusH.u8Buffer[0] = gModbusH.u8id;
    gModbusH.u8Buffer[1] = 1U;
    gModbusH.u8Buffer[2] = byteCount;
    gModbusH.u8BufferSize = 3U;
    memset(&gModbusH.u8Buffer[3], 0, byteCount);
    taskENTER_CRITICAL();
    for (uint16_t i = 0U; i < coilCount; i++) {
        uint16_t idx = startCoil + i;
        uint16_t byteIdx = idx / 8U;
        uint8_t bitPos = (uint8_t)(idx % 8U);
        uint8_t val = (gCoils[db][byteIdx] >> bitPos) & 0x01U;
        if (val) gModbusH.u8Buffer[3U + (i / 8U)] |= (uint8_t)(1U << (i % 8U));
    }
    taskEXIT_CRITICAL();
    gModbusH.u8BufferSize = (uint8_t)(3U + byteCount);
}
static void process_FC3(void) {
    uint16_t rawAddr   = word(gModbusH.u8Buffer[2], gModbusH.u8Buffer[3]);
    uint8_t  db        = MB_ADDR_DECODE_DB(rawAddr);
    uint16_t startReg  = MB_ADDR_DECODE_OFFSET(rawAddr);
    uint16_t regCount  = word(gModbusH.u8Buffer[4], gModbusH.u8Buffer[5]);

    if (regCount == 0U || regCount > 125U) { // Modbus limit: max 125 registers
        gModbusH.u16errCnt++;
        return;
    }
    if ((db >= DB_COUNT) || ((uint32_t)startReg + regCount > REGS_PER_DB)) {
        gModbusH.u16errCnt++;
        return;
    }

    uint8_t byteCount = (uint8_t)(2U * regCount);
    gModbusH.u8Buffer[0] = gModbusH.u8id;
    gModbusH.u8Buffer[1] = 3U; // FC3
    gModbusH.u8Buffer[2] = byteCount;
    gModbusH.u8BufferSize = 3U;

    for (uint16_t i = 0U; i < regCount; i++) {
        uint16_t reg_val = gRegs[db][startReg + i];
        // Big-Endian: MSB first
        gModbusH.u8Buffer[3 + i*2]     = (uint8_t)(reg_val >> 8);
        gModbusH.u8Buffer[3 + i*2 + 1] = (uint8_t)(reg_val & 0xFF);
    }

    gModbusH.u8BufferSize = (uint8_t)(3U + byteCount);
}
static void process_FC5(void) {
    uint16_t rawAddr = word(gModbusH.u8Buffer[2], gModbusH.u8Buffer[3]);
    uint8_t  db      = MB_ADDR_DECODE_DB(rawAddr);
    uint16_t coilIdx = MB_ADDR_DECODE_OFFSET(rawAddr);
    uint16_t value   = word(gModbusH.u8Buffer[4], gModbusH.u8Buffer[5]);


    if (value != 0x0000U && value != 0xFF00U) {
        gModbusH.u16errCnt++;
        return;
    }

    if ((db >= DB_COUNT) || (coilIdx >= COILS_PER_DB)) {
        gModbusH.u16errCnt++;
        return;
    }

    uint16_t byteIdx = coilIdx / 8U;
    uint8_t  bitPos  = (uint8_t)(coilIdx % 8U);
    bool state = (value == 0xFF00U);

    taskENTER_CRITICAL();
    if (state) {
        gCoils[db][byteIdx] |= (uint8_t)(1U << bitPos);
    } else {
        gCoils[db][byteIdx] &= (uint8_t)~(1U << bitPos);
    }
    taskEXIT_CRITICAL();
    gModbusH.u8Buffer[0] = gModbusH.u8id;
    gModbusH.u8Buffer[1] = 0x05;
    gModbusH.u8Buffer[2] = gModbusH.u8Buffer[2]; // Addr_H
    gModbusH.u8Buffer[3] = gModbusH.u8Buffer[3]; // Addr_L
    gModbusH.u8Buffer[4] = gModbusH.u8Buffer[4]; // Val_H
    gModbusH.u8Buffer[5] = gModbusH.u8Buffer[5]; // Val_L
    gModbusH.u8BufferSize = 6;
}



static void process_FC6(void) {
    uint16_t rawAddr = word(gModbusH.u8Buffer[2], gModbusH.u8Buffer[3]);
    uint8_t  db      = MB_ADDR_DECODE_DB(rawAddr);
    uint16_t regIdx  = MB_ADDR_DECODE_OFFSET(rawAddr);
    uint16_t value   = word(gModbusH.u8Buffer[4], gModbusH.u8Buffer[5]);

    if ((db >= DB_COUNT) || (regIdx >= REGS_PER_DB)) {
        gModbusH.u16errCnt++;
        return;
    }

    taskENTER_CRITICAL();
    gRegs[db][regIdx] = value;
    taskEXIT_CRITICAL();


    gModbusH.u8Buffer[0] = gModbusH.u8id;
    gModbusH.u8Buffer[1] = 0x06; // FC6
    gModbusH.u8Buffer[2] = gModbusH.u8Buffer[2]; // Addr_H
    gModbusH.u8Buffer[3] = gModbusH.u8Buffer[3]; // Addr_L
    gModbusH.u8Buffer[4] = gModbusH.u8Buffer[4]; // Val_H
    gModbusH.u8Buffer[5] = gModbusH.u8Buffer[5]; // Val_L
    gModbusH.u8BufferSize = 6;
}
static void process_FC15(void) {
    uint16_t rawAddr   = word(gModbusH.u8Buffer[2], gModbusH.u8Buffer[3]);
    uint8_t  db        = MB_ADDR_DECODE_DB(rawAddr);
    uint16_t startCoil = MB_ADDR_DECODE_OFFSET(rawAddr);
    uint16_t coilCount = word(gModbusH.u8Buffer[4], gModbusH.u8Buffer[5]);
    uint8_t  byteCount = gModbusH.u8Buffer[6];
    if (coilCount == 0U || coilCount > 1968U) { // Modbus limit: max 1968 coils
        gModbusH.u16errCnt++;
        return;
    }
    uint8_t expectedByteCount = (uint8_t)((coilCount + 7U) / 8U);
    if (byteCount != expectedByteCount) {
        gModbusH.u16errCnt++;
        return;
    }
    if ((db >= DB_COUNT) || ((uint32_t)startCoil + coilCount > COILS_PER_DB)) {
        gModbusH.u16errCnt++;
        return;
    }


    taskENTER_CRITICAL();
    for (uint16_t i = 0U; i < coilCount; i++) {
        uint16_t coilIdx = startCoil + i;
        uint8_t  byteIdx = coilIdx / 8U;
        uint8_t  bitPos  = (uint8_t)(coilIdx % 8U);

        uint8_t dataByte = gModbusH.u8Buffer[7 + (i / 8U)];
        bool state = (dataByte >> (i % 8U)) & 0x01U;

        if (state) {
            gCoils[db][byteIdx] |= (uint8_t)(1U << bitPos);
        } else {
            gCoils[db][byteIdx] &= (uint8_t)~(1U << bitPos);
        }
    }
    taskEXIT_CRITICAL();
    gModbusH.u8Buffer[0] = gModbusH.u8id;
    gModbusH.u8Buffer[1] = 0x0F; // FC15
    gModbusH.u8Buffer[2] = gModbusH.u8Buffer[2]; // StartAddr_H
    gModbusH.u8Buffer[3] = gModbusH.u8Buffer[3]; // StartAddr_L
    gModbusH.u8Buffer[4] = gModbusH.u8Buffer[4]; // Qty_H
    gModbusH.u8Buffer[5] = gModbusH.u8Buffer[5]; // Qty_L
    gModbusH.u8BufferSize = 6;
}

static void process_FC16(void) {
    uint16_t rawAddr   = word(gModbusH.u8Buffer[2], gModbusH.u8Buffer[3]);
    uint8_t  db        = MB_ADDR_DECODE_DB(rawAddr);
    uint16_t startReg  = MB_ADDR_DECODE_OFFSET(rawAddr);
    uint16_t regCount  = word(gModbusH.u8Buffer[4], gModbusH.u8Buffer[5]);
    uint8_t  byteCount = gModbusH.u8Buffer[6];
    if (regCount == 0U || regCount > 123U) {
        gModbusH.u16errCnt++;
        return;
    }
    if (byteCount != (2U * regCount)) {
        gModbusH.u16errCnt++;
        return;
    }


    if ((db >= DB_COUNT) || ((uint32_t)startReg + regCount > REGS_PER_DB)) {

        gModbusH.u16errCnt++;
        return;
    }

    for (uint16_t i = 0U; i < regCount; i++) {
        uint16_t val = word(gModbusH.u8Buffer[7 + i*2], gModbusH.u8Buffer[7 + i*2 + 1]);
        gRegs[db][startReg + i] = val;
    }
    gModbusH.u8Buffer[0] = gModbusH.u8id;
    gModbusH.u8Buffer[1] = 0x10; // FC16
    gModbusH.u8Buffer[2] = gModbusH.u8Buffer[2]; // آدرس High (0x03)
    gModbusH.u8Buffer[3] = gModbusH.u8Buffer[3]; // آدرس Low  (0xE9)
    gModbusH.u8Buffer[4] = gModbusH.u8Buffer[4]; // تعداد High (0x00)
    gModbusH.u8Buffer[5] = gModbusH.u8Buffer[5]; // تعداد Low  (0x01)
    gModbusH.u8BufferSize = 6;
}
static void process_FC23(void) {
    uint16_t readAddr   = word(gModbusH.u8Buffer[2], gModbusH.u8Buffer[3]);
    uint16_t readQty    = word(gModbusH.u8Buffer[4], gModbusH.u8Buffer[5]);

    uint16_t writeAddr  = word(gModbusH.u8Buffer[6], gModbusH.u8Buffer[7]);
    uint16_t writeQty   = word(gModbusH.u8Buffer[8], gModbusH.u8Buffer[9]);
    uint8_t  writeByteCount = gModbusH.u8Buffer[10];

    if (readQty == 0U || readQty > 125U) {
        gModbusH.u16errCnt++;
        return;
    }
    if (writeQty == 0U || writeQty > 121U) {
        gModbusH.u16errCnt++;
        return;
    }
    if (writeByteCount != (2U * writeQty)) {
        gModbusH.u16errCnt++;
        return;
    }

    uint8_t readDb = MB_ADDR_DECODE_DB(readAddr);
    uint16_t readStart = MB_ADDR_DECODE_OFFSET(readAddr);
    if ((readDb >= DB_COUNT) || ((uint32_t)readStart + readQty > REGS_PER_DB)) {
        gModbusH.u16errCnt++;
        return;
    }

    uint8_t writeDb = MB_ADDR_DECODE_DB(writeAddr);
    uint16_t writeStart = MB_ADDR_DECODE_OFFSET(writeAddr);
    if ((writeDb >= DB_COUNT) || ((uint32_t)writeStart + writeQty > REGS_PER_DB)) {
        gModbusH.u16errCnt++;
        return;
    }

    taskENTER_CRITICAL();
    for (uint16_t i = 0U; i < writeQty; i++) {
        uint16_t val = word(gModbusH.u8Buffer[11 + i*2], gModbusH.u8Buffer[11 + i*2 + 1]);
        gRegs[writeDb][writeStart + i] = val;
    }
    taskEXIT_CRITICAL();

    gModbusH.u8Buffer[0] = gModbusH.u8id;
    gModbusH.u8Buffer[1] = 0x17; // FC23
    uint8_t readByteCount = (uint8_t)(2U * readQty);
    gModbusH.u8Buffer[2] = readByteCount;

    for (uint16_t i = 0U; i < readQty; i++) {
        uint16_t reg_val = gRegs[readDb][readStart + i];
        // Big-Endian: MSB اول
        gModbusH.u8Buffer[3 + i*2]     = (uint8_t)(reg_val >> 8);
        gModbusH.u8Buffer[3 + i*2 + 1] = (uint8_t)(reg_val & 0xFF);
    }

    gModbusH.u8BufferSize = (uint8_t)(3U + readByteCount); // 3 = ID + FC + ByteCount
}

#endif
#if 1
static void handle_frame(void){
    if (gModbusH.xBufferRX.u8available == 0U) return;
    uint16_t size = (gModbusH.xBufferRX.u8available > MAX_BUFFER) ? MAX_BUFFER : gModbusH.xBufferRX.u8available;
    memcpy(gModbusH.u8Buffer, gModbusH.xBufferRX.uxBuffer, size);
    gModbusH.u8BufferSize = (uint8_t)size;
    gModbusH.xBufferRX.u8available = 0U;

    if (gModbusH.u8BufferSize < 4U) {
        gModbusH.u16errCnt++;
        gModbusH.u8BufferSize = 0U;
        return;
    }


     uint16_t recv_crc = (uint16_t)(gModbusH.u8Buffer[gModbusH.u8BufferSize - 2]) |
                   ((uint16_t)(gModbusH.u8Buffer[gModbusH.u8BufferSize - 1]) << 8);;
    uint16_t calc = calcCRC(gModbusH.u8Buffer, (uint8_t)(gModbusH.u8BufferSize - 2U));
    if (calc != recv_crc) {
        gModbusH.u16errCnt++;
        gModbusH.u8BufferSize = 0U;
        return;
    }

    uint8_t addr = gModbusH.u8Buffer[0];
    if ((addr != gModbusH.u8id) && (addr != 0x00U)) {
        gModbusH.u8BufferSize = 0U;
        return;
    }

    uint8_t func = gModbusH.u8Buffer[1];
    switch (func) {
        case 1U:  process_FC1(); break;  // Read Coils
        case 3U:  process_FC3(); break;  // Read Holding Registers
        case 5U:  process_FC5(); break;  // Write Single Coil
        case 6U:  process_FC6(); break;  // Write Single Register
        case 15U: process_FC15(); break; // Write Multiple Coils
        case 16U: process_FC16(); break; // Write Multiple Registers
        case 23U: process_FC23(); break; // Read/Write Multiple Registers

        default:
            gModbusH.u16errCnt++;
            break;
    }

    if (addr != 0x00U && gModbusH.u8BufferSize > 0U) {
        transmit_response();
        gModbusH.u16OutCnt++;
    } else {
        gModbusH.u8BufferSize = 0U;
    }
}

static void transmit_response(void){

    if ((uint16_t)gModbusH.u8BufferSize + 2U > MAX_BUFFER) {
        gModbusH.u16errCnt++;
        gModbusH.u8BufferSize = 0U;
        return;
    }

    uint16_t crc = calcCRC(gModbusH.u8Buffer, (uint8_t)gModbusH.u8BufferSize);
    gModbusH.u8Buffer[gModbusH.u8BufferSize++] = (uint8_t)(crc & 0xFF); // LSB اول
    gModbusH.u8Buffer[gModbusH.u8BufferSize++] = (uint8_t)(crc >> 8);   // MSB بعد

    if (gModbusH.EN_Port != NULL) {
        HAL_GPIO_WritePin(gModbusH.EN_Port, gModbusH.EN_Pin, GPIO_PIN_SET);
    }


    HAL_StatusTypeDef ret = HAL_UART_Transmit_DMA(gModbusH.port,
                                                  gModbusH.u8Buffer,
                                                  gModbusH.u8BufferSize);
    if (ret != HAL_OK) {
        if (gModbusH.EN_Port != NULL) {
            HAL_GPIO_WritePin(gModbusH.EN_Port, gModbusH.EN_Pin, GPIO_PIN_RESET);
        }
        gModbusH.u16errCnt++;
        gModbusH.u8BufferSize = 0U;
    }
}

void StartTaskModbusSlave(void *argument){
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        handle_frame();
        gModbusH.u8BufferSize = 0U;
    }
}

#endif
#if 1
bool mb_read_coil(uint8_t db, int16_t address) {
    if ((db >= DB_COUNT) || (address < 0) || (address >= (int16_t)COILS_PER_DB)) {
        return false;
    }
    uint16_t byteIdx = (uint16_t)address / 8U;
    uint8_t  bitPos  = (uint8_t)address % 8U;
    bool result;
    taskENTER_CRITICAL();
    result = ((gCoils[db][byteIdx] >> bitPos) & 0x01U) != 0U;
    taskEXIT_CRITICAL();
    return result;
}

void mb_write_coil(uint8_t db, int16_t address, bool state) {
    if ((db >= DB_COUNT) || (address < 0) || (address >= (int16_t)COILS_PER_DB)) {
        return;
    }
    uint16_t byteIdx = (uint16_t)address / 8U;
    uint8_t  bitPos  = (uint8_t)address % 8U;
    taskENTER_CRITICAL();
    if (state) gCoils[db][byteIdx] |=  (uint8_t)(1U << bitPos);
    else       gCoils[db][byteIdx] &= (uint8_t)~(1U << bitPos);
    taskEXIT_CRITICAL();

}

uint16_t mb_read_register(uint8_t db, int16_t address) {
    if ((db >= DB_COUNT) || (address < 0) || (address >= (int16_t)REGS_PER_DB)) {
        return 0U;
    }
    uint16_t val;
    taskENTER_CRITICAL();
    val = gRegs[db][address];
    taskEXIT_CRITICAL();
    return val;
}

void mb_write_register(uint8_t db, int16_t address, const uint16_t *data, uint16_t len) {
    if ((db >= DB_COUNT) || (address < 0) || (data == NULL)) {
        return;
    }
    taskENTER_CRITICAL();
    for (uint16_t i = 0U; (i < len) && ((address + i) < (int16_t)REGS_PER_DB); i++) {
        gRegs[db][address + i] = data[i];
    }
    taskEXIT_CRITICAL();

}

#endif
#if 1
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
    if (gModbusH.port == huart) {
        if (gModbusH.EN_Port != NULL) {HAL_GPIO_WritePin(gModbusH.EN_Port, gModbusH.EN_Pin, GPIO_PIN_RESET);}
        if (gModbusH.myTaskModbusAHandle != NULL) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            vTaskNotifyGiveFromISR(gModbusH.myTaskModbusAHandle, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
    if (gModbusH.port == huart) {
        HAL_UART_DMAStop(gModbusH.port);
        HAL_UARTEx_ReceiveToIdle_DMA(gModbusH.port, gModbusH.xBufferRX.uxBuffer, MAX_BUFFER);
        if (gModbusH.port->hdmarx != NULL) {__HAL_DMA_DISABLE_IT(gModbusH.port->hdmarx, DMA_IT_HT);}
    }
}
static HAL_StatusTypeDef Modbus_StartRxDMA(void){
    HAL_StatusTypeDef status;

    for (uint8_t retry = 0U; retry < 3U; retry++)
    {
        status = HAL_UARTEx_ReceiveToIdle_DMA(
            gModbusH.port,
            gModbusH.xBufferRX.uxBuffer,
            MAX_BUFFER
        );

        if (status == HAL_OK)
        {
            if (gModbusH.port->hdmarx != NULL)
            {
                __HAL_DMA_DISABLE_IT(
                    gModbusH.port->hdmarx,
                    DMA_IT_HT
                );
            }

            return HAL_OK;
        }
    }

    gModbusH.u16errCnt++;
    return HAL_ERROR;
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
    if (gModbusH.port == huart && Size > 0U) {
        gModbusH.xBufferRX.u8available = Size;
        gModbusH.xBufferRX.overflow = false;
        Modbus_StartRxDMA();
        if (gModbusH.myTaskModbusAHandle != NULL) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xTaskNotifyFromISR(gModbusH.myTaskModbusAHandle,(uint32_t)Size, eSetValueWithOverwrite,&xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}
#endif
