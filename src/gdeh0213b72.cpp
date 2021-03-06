/* Self header */
#include "gdeh0213b72.h"

/* Arduino libraries */
#include <Arduino.h>
#include <Wire.h>

/* Config */
#ifndef CONFIG_GDEH0213B72_TIMEOUT
#define CONFIG_GDEH0213B72_TIMEOUT 5000
#endif
#if CONFIG_GDEH0213B72_DEBUG_ENABLED
#define CONFIG_GDEH0213B72_DEBUG_FUNCTION(x) Serial.println(x)
#else
#define CONFIG_GDEH0213B72_DEBUG_FUNCTION(x)
#endif
#define ALLSCREEN_GRAGHBYTES  4000

/* Macros */
#define m_send_command_and_send_data(command, ...) { m_send_command(command); const uint8_t _data[] = {__VA_ARGS__}; for(size_t i = 0; i < sizeof(_data) / sizeof(uint8_t); i++) m_send_data(_data[i]); }

/**
 *
 * @param pin_busy (8 for example)
 * @param pin_res (9 for example)
 * @param pin_dc (10 for example)
 * @param pin_cs (11 for example)
 * @param pin_sck (12 for example)
 * @param pin_sdi (13 for example)
 */
int gdeh0213b72::setup(const uint8_t pin_busy, const uint8_t pin_res, const uint8_t pin_dc, const uint8_t pin_cs, const uint8_t pin_sck, const uint8_t pin_sdi) {
	int res;

	/* Save gpios */
	m_pin_busy = pin_busy;
	m_pin_res = pin_res;
	m_pin_dc = pin_dc;
	m_pin_cs = pin_cs;
	m_pin_clock = pin_sck;
	m_pin_data = pin_sdi;

	/* Configure gpios */
	pinMode(m_pin_busy, INPUT);
	pinMode(m_pin_res, OUTPUT);
	pinMode(m_pin_dc, OUTPUT);
	pinMode(m_pin_cs, OUTPUT);
	pinMode(m_pin_clock, OUTPUT);
	pinMode(m_pin_data, OUTPUT);

	/* Perform hardware reset */
	digitalWrite(m_pin_res, LOW);
	delay(10);
	digitalWrite(m_pin_res, HIGH);
	delay(10);
	res = m_busy_wait();
	if (res < 0) {
		CONFIG_GDEH0213B72_DEBUG_FUNCTION(" [e] Timed out performing hw reset!");
		return res;
	}

	/* Perform software reset */
	m_send_command(0x12);
	res = m_busy_wait();
	if (res < 0) {
		CONFIG_GDEH0213B72_DEBUG_FUNCTION(" [e] Timed out performing sw reset!");
		return res;
	}

	/* Send configuration */
	m_send_command_and_send_data(0x74, 0x54); //set analog block control
	m_send_command_and_send_data(0x7E, 0x3B); //set digital block control
	m_send_command_and_send_data(0x01, 0xF9, 0x00, 0x00); //Driver output control
	m_send_command_and_send_data(0x11, 0x03); //data entry mode
	m_send_command_and_send_data(0x44, 0x00, (uint8_t )((m_width + 8 - 1) / 8 - 1));                                    //set Ram-X address start/end position //0x0C-->(15+1)*8=128
	m_send_command_and_send_data(0x45, 0x00, 0x00, (uint8_t )((m_height - 1) % 256), (uint8_t )((m_height - 1) / 256)); //set Ram-Y address start/end position //0xF9-->(249+1)=250
	m_send_command_and_send_data(0x3C, 0x03); //BorderWavefrom
	m_send_command_and_send_data(0x2C, 0x55); //VCOM Voltage
	m_send_command_and_send_data(0x03, k_lut_entire[70]); //
	m_send_command_and_send_data(0x04, k_lut_entire[71], k_lut_entire[72], k_lut_entire[73]);
	m_send_command_and_send_data(0x3A, k_lut_entire[74]); //Dummy Line
	m_send_command_and_send_data(0x3B, k_lut_entire[75]); //Gate time
	m_lut_use((unsigned char*) k_lut_entire); //LUT
	m_send_command_and_send_data(0x4E, 0x00);       // set RAM x address count to 0;
	m_send_command_and_send_data(0x4F, 0xF9, 0x00); // set RAM y address count to 0X127;
	res = m_busy_wait();
	if (res < 0) {
		CONFIG_GDEH0213B72_DEBUG_FUNCTION(" [e] Timed out configuring display!");
		return res;
	}

	/* Return success */
	return 0;
}

/**
 * Tries to detect the display.
 * @return true if the display was detected, or false otherwise.
 */
bool gdeh0213b72::detect(void) {

	/* Read status register */
	uint8_t reg_status;
	m_send_command_and_read_data(0x2F, &reg_status, 1);

	/* Ensure chip id is as expected */
	if ((reg_status & 0x03) != 0x01) {
		return false;
	}

	/* Return success */
	return true;
}

/**
 *
 * @note To Exit Deep Sleep mode, User required to send HWRESET to the driver
 */
int gdeh0213b72::hibernate(void) {

	/* Send deep sleep mode command */
	m_send_command_and_send_data(0x10, 0x01);

	/* Wait a bit
	 * @note I don't know why? */
	delay(100);

	/* Return success */
	return 0;
}

/**
 * Waits for at least the given number of nanoseconds.
 * @param[in] ns The minimum number of nanoseconds to wait for.
 * @note This has a lot of room for improvement.
 */
void inline __attribute__((always_inline)) gdeh0213b72::m_delay_ns(const uint32_t ns) {
	for (uint8_t cycles = 2; cycles < (ns * 1000) / (F_CPU / 1000); cycles++) {
		asm("nop");
	}
}

/**
 * Sends a command.
 * @param[in] command The command byte to send.
 * @return 0 in case of success, or a negative error code otherwise.
 * @see Datasheet section 7.2 "MCU Serial Peripheral Interface (4-wire SPI)".
 * @see Datasheet section 13 "Serial Peripheral Interface Timing".
 */
void gdeh0213b72::m_send_command(const uint8_t command) {

	/* Assert chip select */
	digitalWrite(m_pin_cs, 0);
	m_delay_ns(20); // tCSSU 20ns

	/* Send command */
	digitalWrite(m_pin_dc, 0);
	for (uint8_t i = 0; i < 8; i++) {
		digitalWrite(m_pin_clock, 0);
		digitalWrite(m_pin_data, (command & (1 << (7 - i))));
		m_delay_ns(20); // tSCLLOW 20ns
		digitalWrite(m_pin_clock, 1);
		m_delay_ns(20); // tSCLHIGH 20ns
	}

	/* Release chip select */
	digitalWrite(m_pin_cs, 1);
	m_delay_ns(250); // tCSHIGH 250ns
}

/**
 * Sends a data bye.
 * @param[in] data The data byte to send.
 * @return 0 in case of success, or a negative error code otherwise.
 * @see Datasheet section 7.2 "MCU Serial Peripheral Interface (4-wire SPI)".
 * @see Datasheet section 13 "Serial Peripheral Interface Timing".
 */
void gdeh0213b72::m_send_data(const uint8_t data) {

	/* Assert chip select */
	digitalWrite(m_pin_cs, 0);
	m_delay_ns(20); // tCSSU 20ns

	/* Send data */
	digitalWrite(m_pin_dc, 1);
	for (uint8_t i = 0; i < 8; i++) {
		digitalWrite(m_pin_clock, 0);
		digitalWrite(m_pin_data, (data & (1 << (7 - i))));
		m_delay_ns(20); // tSCLLOW 20ns
		digitalWrite(m_pin_clock, 1);
		m_delay_ns(20); // tSCLHIGH 20ns
	}

	/* Release chip select */
	digitalWrite(m_pin_cs, 1);
	m_delay_ns(250); // tCSHIGH 250ns
}

/**
 * In one transaction, sends a command and reads data from the spi shared data line.
 * @param[out] data A pointer to a variable that will be updated with the byte read.
 * @return 0 in case of success, or a negative error code otherwise.
 * @see Datasheet section 7.2 "MCU Serial Peripheral Interface (4-wire SPI)".
 * @see Datasheet section 13 "Serial Peripheral Interface Timing".
 */
void gdeh0213b72::m_send_command_and_read_data(const uint8_t command, uint8_t * const data, const size_t length) {

	/* Assert chip select */
	digitalWrite(m_pin_cs, 0);
	m_delay_ns(20); // tCSSU 20ns

	/* Send command */
	pinMode(m_pin_data, OUTPUT);
	digitalWrite(m_pin_dc, 0);
	for (uint8_t i = 0; i < 8; i++) {
		digitalWrite(m_pin_clock, 0);
		digitalWrite(m_pin_data, (command & (1 << (7 - i))));
		m_delay_ns(20); // tSCLLOW 20ns
		digitalWrite(m_pin_clock, 1);
		m_delay_ns(20); // tSCLHIGH 20ns
	}

	/* Read data */
	pinMode(m_pin_data, INPUT_PULLUP);
	digitalWrite(m_pin_dc, 1);
	for (size_t j = 0; j < length; j++) {
		data[j] = 0;
		for (uint8_t i = 0; i < 8; i++) {
			digitalWrite(m_pin_clock, 0);
			m_delay_ns(180); // tSCLLOW 180ns
			digitalWrite(m_pin_clock, 1);
			data[j] <<= 1;
			data[j] |= digitalRead(m_pin_data);
			m_delay_ns(180); // tSCLHIGH 180ns
		}
	}
	pinMode(m_pin_data, OUTPUT);

	/* Release chip select */
	digitalWrite(m_pin_cs, 1);
	m_delay_ns(250); // tCSHIGH 250ns
}

/**
 * Waits for the display to stop asserting the busy line.
 * @return 0 when the busy line is no longer asserted, or a negative error code otherwise.
 */
int gdeh0213b72::m_busy_wait(void) {
	uint32_t start = millis();
	while (1) {
		if (millis() - start > CONFIG_GDEH0213B72_TIMEOUT) {
			return -ETIMEDOUT;
		}
		if (digitalRead(m_pin_busy) == 0) {
			return 0;
		}
	}
}

/**
 *
 * @param[in] wave_data A pointer to a valid lut, stored in program memory.
 */
void gdeh0213b72::m_lut_use(const uint8_t *wave_data) {

	/* Send the write lut register command */
	m_send_command(0x32);
	for (size_t count = 0; count < 70; count++) {
		m_send_data(pgm_read_byte(&wave_data[count]));
	}
}

/**
 * Holds the waveform configuration needed to perform a entire display redraw.
 */
const PROGMEM uint8_t gdeh0213b72::k_lut_entire[] = { /*
 **/0x80, 0x60, 0x40, 0x00, 0x00, 0x00, 0x00, // LUT0: BB:   VS 0 ~7
	0x10, 0x60, 0x20, 0x00, 0x00, 0x00, 0x00, // LUT1: BW:   VS 0 ~7
	0x80, 0x60, 0x40, 0x00, 0x00, 0x00, 0x00, // LUT2: WB:   VS 0 ~7
	0x10, 0x60, 0x20, 0x00, 0x00, 0x00, 0x00, // LUT3: WW:   VS 0 ~7
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // LUT4: VCOM: VS 0 ~7
	0x03, 0x03, 0x00, 0x00, 0x02, // TP0 A~D RP0
	0x09, 0x09, 0x00, 0x00, 0x02, // TP1 A~D RP1
	0x03, 0x03, 0x00, 0x00, 0x02, // TP2 A~D RP2
	0x00, 0x00, 0x00, 0x00, 0x00, // TP3 A~D RP3
	0x00, 0x00, 0x00, 0x00, 0x00, // TP4 A~D RP4
	0x00, 0x00, 0x00, 0x00, 0x00, // TP5 A~D RP5
	0x00, 0x00, 0x00, 0x00, 0x00, // TP6 A~D RP6
	0x15, 0x41, 0xA8, 0x32, 0x30, 0x0A, };

/**
 *
 */
gdeh0213b72_fast::gdeh0213b72_fast(void) :
		GFXcanvas1(122, 250) {
}

/**
 *
 * @param[in] i
 * @note Modifications will only be visible on the display after a call to one of the draw functions.
 */
void gdeh0213b72_fast::invertDisplay(bool i) {

	/* Send display update control 1 command */
	if (i) {
		m_send_command_and_send_data(0x21, 0x08);
	} else {
		m_send_command_and_send_data(0x21, 0x00);
	}
}

/**
 * Requests the display to display the contents of the entire ram.
 * @return 0 in case of success, or a negative error code otherwise.
 */
int gdeh0213b72_fast::refresh_entire(void) {
	int res;

	/* Update display ram with local contents */
	m_send_command_and_send_data(0x4E, 0);
	m_send_command_and_send_data(0x4F, 0, 0);
	m_send_command(0x24);
	uint8_t *buffer_start = getBuffer();
	for (uint16_t i = 0; i < 4000; i++) {
		m_send_data(buffer_start[i]);
	}

	/* Send display update command */
	m_send_command_and_send_data(0x22, 0xC7);

	/* Send master activation command */
	m_send_command(0x20);

	/* Wait for end of update signaled by busy pin */
	res = m_busy_wait();
	if (res < 0) {
		CONFIG_GDEH0213B72_DEBUG_FUNCTION(" [e] Failed to perform update!");
		return res;
	}

	/* Return success */
	return 0;
}

/**
 *
 */
gdeh0213b72_slow::gdeh0213b72_slow(void) :
		Adafruit_GFX(122, 250) {
}

/**
 *
 * @param[in] i
 * @note Modifications will only be visible on the display after a call to one of the draw functions.
 */
void gdeh0213b72_slow::invertDisplay(bool i) {

	/* Send display update control 1 command */
	if (i) {
		m_send_command_and_send_data(0x21, 0x08);
	} else {
		m_send_command_and_send_data(0x21, 0x00);
	}
}

/**
 *
 * @param[in] x The number of pixels along the horizontal axis, starting from the bottom.
 * @param[in] y The number of pixels along the vertical axis, starting from the top.
 * @param[in] color 0 for black, anything else for white.
 */
void gdeh0213b72_slow::drawPixel(int16_t x, int16_t y, uint16_t color) {

	/* Handle rotation */
	uint16_t ram_y, ram_x;
	switch (rotation) {
		case 0: { // Default rotation, portrait with flex at the bottom
			ram_x = x;
			ram_y = y;
			break;
		}
		case 1: { // Rotated 90° clockwise
			ram_x = y;
			ram_y = 249 - x;
			break;
		}
		case 2: { // Rotated 180° clockwise
			ram_x = 121 - x;
			ram_y = 249 - x;
			break;
		}
		case 3: { // Rotated 270° clockwise
			ram_x = 121 - y;
			ram_y = x;
			break;
		}
		default: {
			CONFIG_GDEH0213B72_DEBUG_FUNCTION(" [e] Invalid rotation!");
			return;
		}
	}

	/* Ensure parameters are valid */
	if (x < 0 || y < 0 || ram_x >= m_width || ram_y >= m_height) {
		CONFIG_GDEH0213B72_DEBUG_FUNCTION(" [e] Invalid coordinates!");
		return;
	}

	/* Retrieve byte containing pixel value from ram
	 * @note First one is a dummy byte */
	uint8_t byte[2];
	m_send_command_and_send_data(0x4E, (uint8_t )(ram_x / 8));
	m_send_command_and_send_data(0x4F, (uint8_t )(ram_y % 256), (uint8_t )(ram_y / 256));
	m_send_command_and_read_data(0x27, byte, 2);

	/* Modify byte */
	if (color) byte[0] = byte[1] | (1 << (7 - (ram_x % 8)));
	else byte[0] = byte[1] & ~(1 << (7 - (ram_x % 8)));

	/* Write modified byte to ram */
	if (byte[0] != byte[1]) {
		m_send_command_and_send_data(0x4E, (uint8_t )(ram_x / 8));
		m_send_command_and_send_data(0x4F, (uint8_t )(ram_y % 256), (uint8_t )(ram_y / 256));
		m_send_command_and_send_data(0x24, byte[0]);
	}
}

/**
 * Fills the entire screen with the given color.
 * @note Modifications happen in ram, and will only be visible on the display after a call to one of the draw functions.
 * @return 0 in case of success, or a negative error code otherwise.
 */
void gdeh0213b72_slow::fillScreen(uint16_t color) {

	/* Send command to write into ram */
	m_send_command(0x24);

	/* Write contents of ram */
	for (size_t i = 0; i < m_height * ((m_width + 8 - 1) / 8); i++) {
		m_send_data(color ? 0xFF : 0x00);
	}
}

/**
 * Requests the display to display the contents of the entire ram.
 * @return 0 in case of success, or a negative error code otherwise.
 */
int gdeh0213b72_slow::refresh_entire(void) {
	int res;

	/* Send display update command */
	m_send_command_and_send_data(0x22, 0xC7);

	/* Send master activation command */
	m_send_command(0x20);

	/* Wait for end of update signaled by busy pin */
	res = m_busy_wait();
	if (res < 0) {
		CONFIG_GDEH0213B72_DEBUG_FUNCTION(" [e] Failed to perform update!");
		return res;
	}

	/* Return success */
	return 0;
}
