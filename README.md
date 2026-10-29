# STM32 Modbus RTU Slave Library

A Modbus RTU Slave implementation for STM32 microcontrollers using STM32 HAL, FreeRTOS, UART DMA, and optional RS-485 direction control.

## Features

- Modbus RTU Slave protocol over UART.
- CRC-16 calculation and received-frame validation.
- DMA-based UART reception using Receive-to-Idle.
- DMA-based UART transmission.
- Optional GPIO control for RS-485 transceiver direction.
- Multiple logical databases for coils and registers.
- FreeRTOS task notifications for received frames.
- Critical sections for selected shared-data operations.
- Communication and error counters.
- Local application APIs for reading and writing coils and holding registers.
- Support for standard Modbus function codes and selected custom functionality.

## Supported Function Codes

The following list is based on the fuller `Modbus.c` source file, which dispatches these functions from `handle_frame()`.

| Function Code | Hex | Function | Implementation |
|---|---|---|---|
| FC01 | `0x01` | Read Coils | Reads multiple coil states. |
| FC02 | `0x02` | Read Discrete Inputs | Reads multiple discrete-input states. |
| FC03 | `0x03` | Read Holding Registers | Reads multiple 16-bit holding registers. |
| FC04 | `0x04` | Read Input Registers | Reads multiple 16-bit input registers. |
| FC05 | `0x05` | Write Single Coil | Writes one coil ON or OFF. |
| FC06 | `0x06` | Write Single Register | Writes one holding register. |
| FC08 | `0x08` | Diagnostics | Implements selected diagnostic subfunctions. |
| FC11 | `0x0B` | Get Comm Event Counter | Returns communication-counter information using the implementation's response format. |
| FC14 | `0x0E` | Custom Function | Returns custom device and communication information; this is not the standard Read File Record implementation. |
| FC15 | `0x0F` | Write Multiple Coils | Writes multiple coil states. |
| FC16 | `0x10` | Write Multiple Registers | Writes multiple holding registers. |
| FC17 | `0x11` | Report Slave ID | Returns the configured slave identification string. |
| FC22 | `0x16` | Mask Write Register | Applies AND and OR masks to a holding register. |
| FC23 | `0x17` | Read/Write Multiple Registers | Writes multiple holding registers and returns the requested register values. |
| FC43 | `0x2B` | Encapsulated Interface Transport | Implements Read Device Identification using MEI type `0x0E`. |

### Function Code Details

#### FC08 — Diagnostics

The source handles the following subfunctions:

| Subfunction | Operation |
|---|---|
| `0x0000` | Return Query Data |
| `0x0001` | Restart Communications Option — the implementation returns a retained-status value |
| `0x000A` | Clear communication counters |
| `0x000B` | Return bus message count |
| `0x000C` | Return communication error count |
| `0x000E` | Return server message count |

Other subfunctions are not fully implemented.

#### FC14 — Custom Function

The source uses FC14 to return custom information selected by a Data ID, including:

- Firmware version
- Input message count
- Error count
- Output message count

This is a project-specific use of function code `0x0E`, not the standard Modbus Read File Record operation.

#### FC17 — Report Slave ID

Returns the configured `SLAVE_ID_STRING`. The default value is defined in `Modbus.h` and can be overridden by the application.

#### FC22 — Mask Write Register

Reads a holding register, applies the masks, and writes the result:

`newValue = (currentValue & andMask) | orMask`

#### FC23 — Read/Write Multiple Registers

Performs the write operation first, then returns the requested holding-register values. The implementation validates the read and write ranges and the write byte count.

#### FC43 — Read Device Identification

Uses MEI type `0x0E` and returns device identification objects such as vendor name, product code, firmware version, and serial number.

## Database Model

The fuller source defines these database dimensions:

| Setting | Default |
|---|---:|
| `DB_COUNT` | 13 |
| `COILS_PER_DB` | 1024 |
| `REGS_PER_DB` | 512 |
| `MAX_BUFFER` | 128 bytes |
| `MODBUS_TIMEOUT_MS` | 100 ms |
| `MODBUS_T35_MS` | 5 ms |

The source uses PLC-style addressing:

`rawAddress = (databaseIndex * 1000) + offset`

The database index and offset are decoded using `MB_ADDR_DECODE_DB()` and `MB_ADDR_DECODE_OFFSET()`.

Coils are stored as packed bits. Holding registers are stored as 16-bit values.

## Public API

### Initialization and Reception

```c
void ModbusInit(UART_HandleTypeDef *huart,
                uint8_t slave_id,
                GPIO_TypeDef *en_port,
                uint16_t en_pin);

void ModbusStart(void);
void StartTaskModbusSlave(void *argument);
uint16_t calcCRC(uint8_t *Buffer, uint8_t u8length);
```

- `ModbusInit()` configures the UART, slave address, optional RS-485 direction GPIO, and handler state.
- `ModbusStart()` starts UART Receive-to-Idle DMA reception.
- `StartTaskModbusSlave()` waits for task notifications and processes received frames.
- `calcCRC()` calculates the Modbus RTU CRC-16.

### Coil Access

```c
bool mb_read_coil(uint8_t db, int16_t address);
void mb_write_coil(uint8_t db, int16_t address, bool state);
```

These functions read and write individual coils in the application database.

### Holding Register Access

```c
uint16_t mb_read_register(uint8_t db, int16_t address);

void mb_write_register(uint8_t db,
                       int16_t address,
                       const uint16_t *data,
                       uint16_t len);
```

These functions read one holding register or write a sequence of holding registers.

Invalid database indices and out-of-range addresses are rejected by the local access functions. Read functions return `false` or `0` when their address arguments are invalid.

## Integration

1. Configure the UART peripheral and DMA in STM32CubeMX.
2. Configure the UART baud rate, parity, stop bits, and GPIO pins according to the Modbus network.
3. Provide the appropriate STM32 HAL and FreeRTOS configuration.
4. Include `Modbus.h` in the application.
5. Call `ModbusInit()` with the UART handle, slave address, and optional RS-485 direction GPIO.
6. Create the Modbus slave task and assign its handle to `gModbusH.myTaskModbusAHandle`.
7. Call `ModbusStart()` to start reception.
8. Access coils and holding registers through the public API.

Example initialization:

```c
ModbusInit(&huart1, 1, RS485_DE_GPIO_Port, RS485_DE_Pin);
gModbusH.myTaskModbusAHandle = modbusTaskHandle;
ModbusStart();
```

The UART instance and GPIO identifiers in this example must match the target project.

## Implementation Notes

- The implementation uses a global handler, `gModbusH`, so it is designed for a single Modbus UART instance.
- The source uses DMA and FreeRTOS task notifications to separate UART reception from frame processing.
- Broadcast requests addressed to `0x00` are processed without sending a response.
- CRC is appended low byte first, as required by Modbus RTU.
- The supplied source does not generate standard Modbus exception responses for unsupported functions or invalid requests.
- Some function handlers use custom response formats. Compatibility with a particular Modbus master should be verified before deployment.
- FC02 and FC04 depend on the `gDiscreteInputs` and `gInputRegs` arrays. The fuller source defines these arrays, but the supplied header does not expose them as public `extern` declarations.
- Validate frame lengths and response-buffer limits for the intended deployment, especially for multi-item requests and custom identification responses.

## License

This project is distributed under the MIT License. See the `LICENSE` file for details.
