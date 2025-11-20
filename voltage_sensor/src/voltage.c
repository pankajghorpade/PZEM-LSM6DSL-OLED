#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "gpio.h"
#include "uart.h"
#include "sys_clk.h"
#include "trace.h"
#include "LSM6DSL.h"
#include "i2c_sensor_lsm6dsl.h"
#include "spi_oled_ssd1309.h"

/*******************************************************************************
 *                          Static Function Prototypes
 *******************************************************************************/
void TEST_PZEM_Init(void);
void TEST_PZEM_Transmit(U8 *str1, size_t size);
void TEST_PZEM_Receive(U8 *rx_buffer, size_t size);
void TEST_TRACE_INIT(U32 hclk_frequency);

/*******************************************************************************
 *                          Static Data Definitions
 *******************************************************************************/
static SYS_CLK SysClk;

static SYS_CLK_SOURCE clock_source;

static U32 master_clk;

static I2C_SENSOR_LSM6DSL pLSM6DSL;

#define RESPONSE_SIZE 25  // Buffer size to match the response from PZEM

uint8_t rx_buffer[RESPONSE_SIZE];  // Buffer to store received data

/*******************************************************************************
 *                          Extern Data Definitions
 *******************************************************************************/
SYS_CLK clk;
UART uart;

SPI_OLED_Reg pThis;
GPIO gpio;
/*******************************************************************************
 *                          Extern Function Definitions
 *******************************************************************************/

/*******************************************************************************
 * Name       : main
 * Description: It checks firmware upgrade
 * Remarks    :
 *******************************************************************************/
void receive_data_from_pzem() {
	UART_STATUS status;

	// Initialize and start reception in MODE_0 to block until all bytes arrive
	status = UART_Rx(&uart, rx_buffer, RESPONSE_SIZE, NULL, NULL, UART_MODE_1);

	if (status == UART_STATUS_OK) {
		printf("Data received:\n");
		for (int i = 0; i < RESPONSE_SIZE; i++) {
			printf("0x%02X ", rx_buffer[i]);
		}
		printf("\n");
	} else {
		printf("Failed to receive data\n");
	}
}

void main() {
	U32 ref_clk = 1024000;
	U32 hclk_div1;
	SYS_CLK_SelectClockSource(HCLK, EXTERNAL_CLK);
	clock_source = PLL;
//	master_clk = 1000000;       /* source clock is set to this value */
	master_clk = 8000000;
	SystemCoreClock = 8000000; /* desired value of HCLK */

//	SystemCoreClock = 1000000;

	hclk_div1 = master_clk / (2 * SystemCoreClock);

	/* Set desired Clock  sources & frequencies as per the board specifics */
	SYS_CLK_Init(&SysClk, clock_source, ref_clk, master_clk);
	SYS_CLK_SelectClockSource(HCLK, clock_source);
	SYS_CLK_SetClockDivider(HCLK, hclk_div1, 0);

	HWREG_WRITE32(0x3A8000F4, 0x0000000A);  //OD, dummy divider outside VCO

	HWREG_DELAY(10);

	TRACE_Init(SystemCoreClock); // Please change UART index to AON_UART (UART2_HW_INDEX)

//	HWREG_DELAY(1000);
	GPIO_Init(&gpio);
	GPIO_Configure(GPIO_PIN_4, GPIO_OUTPUT, GPIO_INTERRUPT_DIS, NULL);

	TEST_PZEM_Init();
	I2C_SENSOR_LSM6DSL_Init(&pLSM6DSL, LSM6DSL_SLAVE_ADDR, 10);
	I2C_SENSOR_LSM6DSL_ChipID();

//    SPI_OLED_Display_Init(&pThis, SPI2_HW_INDEX, SPI_DIVIDE_16,GPIO_PIN_4);
//    SPI_OLED_Display_Configure();
	TRACE_Printf(
			"\e[0;30m" "\033[1m" "\r\n      AI Energy Meter Application Started     \r\n");
	S16 p_x_axis;
	S16 p_y_axis;
	S16 p_z_axis;
	U8 buf_x[32];
	U8 buf_y[32];
	U8 buf_z[32];

	SPI_OLED_Display_Init(&pThis, SPI2_HW_INDEX, SPI_DIVIDE_16, GPIO_PIN_4);
	SPI_OLED_Display_Configure();
	SPI_OLED_Display_fillscreen(BLACK);
	SPI_OLED_Display_SetTextSize(1);
	SPI_OLED_Display_Update_Screen();

	while (1) {
		SPI_OLED_Display_fillscreen(BLACK);
//	SPI_OLED_Display_SetTextSize(1);
		LSM6DSL_Read_TEMP_Data();
		I2C_MPU6050_Acceleration_Data(&p_x_axis, &p_y_axis, &p_z_axis);

//	 TRACE_Printf("%d  %d   %d\r\n",p_x_axis,p_y_axis,p_z_axis);

		snprintf((char*) buf_x, sizeof(buf_x), "X = %d", (int) p_x_axis);
		snprintf((char*) buf_y, sizeof(buf_y), "Y = %d", (int) p_y_axis);
		snprintf((char*) buf_z, sizeof(buf_z), "Z = %d", (int) p_z_axis);

//	 HWREG_DELAY(500);

		uint16_t calculate_crc(uint8_t *data, uint16_t length) {
			uint16_t crc = 0xFFFF;
			for (uint16_t i = 0; i < length; i++) {
				crc ^= data[i];
				for (uint8_t j = 0; j < 8; j++) {
					if (crc & 0x0001) {
						crc >>= 1;
						crc ^= 0xA001;
					} else {
						crc >>= 1;
					}
				}
			}
			return crc;
		}
		U8 buf_voltage[32];

		uint8_t response[7] = { 0 }; // Buffer to hold the response from the meter
		uint8_t response1[25]; // Buffer to hold the response from the meter
		uint8_t command[8];   // Command buffer to send Modbus request
		uint8_t command1[8];   // Command buffer to send Modbus request

		// Command to read voltage (starting register 0x0000, read 2 registers)
		command[0] = 0xF8; // Slave address
		command[1] = 0x03; // Function code (Read Input Registers)
		command[2] = 0x00; // Starting address high byte
		command[3] = 0x02; // Starting address low byte (Voltage register)
		command[4] = 0x00; // Number of registers high byte
		command[5] = 0x01; // Number of registers low byte (Read 2 registers)

		// Calculate CRC for the command
		uint16_t crc = calculate_crc(command, 6);
		command[6] = crc & 0xFF;        // CRC low byte
		command[7] = (crc >> 8) & 0xFF; // CRC high byte

		command1[0] = 0xF8; // Slave address
		command1[1] = 0x04; // Function code (Read Input Registers)
		command1[2] = 0x00; // Starting address high byte
		command1[3] = 0x00; // Starting address low byte (Voltage register)
		command1[4] = 0x00; // Number of registers high byte
		command1[5] = 0x0A; // Number of registers low byte (Read 2 registers)

		// Calculate CRC for the command
		crc = calculate_crc(command1, 6);
		command1[6] = crc & 0xFF;        // CRC low byte
		command1[7] = (crc >> 8) & 0xFF; // CRC high byte

//		HWREG_WRITE32(0x40000400, 0x0); // GPIO direction op
//		HWREG_WRITE32(0x40000000, 0xFFFFFFFF); // GPIO low

		// Send the command
		HWREG_DELAY(1000);

		TEST_PZEM_Transmit(command, sizeof(command)); // ARM_UART is used to send and receive the command

		HWREG_DELAY(1000);

		TEST_PZEM_Receive(response, (sizeof(response)));

		HWREG_DELAY(1000);

		TEST_PZEM_Transmit(command1, sizeof(command)); // ARM_UART is used to send and receive the command

		HWREG_DELAY(10000);

		// Receive the response
		TEST_PZEM_Receive(response1, sizeof(response1));

		HWREG_DELAY(1000);

		// Process the response (assuming a valid response is received)
		if (response1[1] == 0x04) {

			TRACE_Printf(" \n\r");

			uint16_t voltage = (response1[3] << 8) | response1[4];    // Voltage
			float display_voltage = voltage / 10.0;  // 220.0 V
			TRACE_Printf("\n\rVoltage: %f V\n\r", display_voltage);

			snprintf((char*) buf_voltage, sizeof(buf_voltage), "Voltage = %d",
					(int) display_voltage);

//  		    uint16_t alarm_status = (response1[21] << 8) | response1[22];           // Alarm status
		} else {
			snprintf((char*) buf_voltage, sizeof(buf_voltage),
					"Voltage reading failed");
			printf("Failed to read data from PZEM-004T-100A\n"); // error message
		}

		TRACE_Printf("%d  %d   %d\r\n", p_x_axis, p_y_axis, p_z_axis);

		/* clear and draw once, use correct lengths */

		SPI_OLED_Display_SetCursor(0, 20);
		SPI_OLED_Display_WriteString(buf_x, strlen((char*) buf_x) + 1, WHITE);
		SPI_OLED_Display_Update_Screen();

		SPI_OLED_Display_SetCursor(0, 30);
		SPI_OLED_Display_WriteString(buf_y, strlen((char*) buf_y) + 1, WHITE);
		SPI_OLED_Display_Update_Screen();

		SPI_OLED_Display_SetCursor(0, 40);
		SPI_OLED_Display_WriteString(buf_z, strlen((char*) buf_z) + 1, WHITE);
		SPI_OLED_Display_Update_Screen();

		SPI_OLED_Display_SetCursor(0, 50);
		SPI_OLED_Display_WriteString(buf_voltage,
				strlen((char*) buf_voltage) + 1, WHITE);

		/* update the screen a single time */
		SPI_OLED_Display_Update_Screen();

// 		HWREG_DELAY(10000000);
		HWREG_DELAY(1000000);

	}
}

/*******************************************************************************
 *                         Static Function Definitions
 *******************************************************************************/

/*******************************************************************************
 * Name       : TEST_UART_Init
 * Description: Initializes the UART peripheral
 * Remarks    :
 *******************************************************************************/
void TEST_PZEM_Init(void) {
	UART_Init(&uart, UART1_HW_INDEX, 9600, 8000000);
}

/*******************************************************************************
 * Name       : TEST_UART_BLE_Transmit
 * Description: Set BLE to Tx data
 * Remarks    :
 *******************************************************************************/
void TEST_PZEM_Transmit(U8 *str1, size_t size) {
	UART_Tx(&uart, str1, size, NULL, NULL, UART_MODE_0);
}

/*******************************************************************************
 * Name       : TEST_UART_BLE_Receive
 * Description: Set BLE to Rx
 * Remarks    :
 *******************************************************************************/
void TEST_PZEM_Receive(U8 *rx_buffer, size_t size) {
	UART_Rx(&uart, rx_buffer, size, NULL, NULL, UART_MODE_0);
}

/*******************************************************************************
 * Name       : TEST_UART_BLE_Receive
 * Description: Set BLE to Rx
 * Remarks    :
 *******************************************************************************/
void TEST_TRACE_INIT(U32 hclk_frequency) {
	UART_Init(&uart, UART2_HW_INDEX, 9600, hclk_frequency);
}

