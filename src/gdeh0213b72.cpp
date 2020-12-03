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
#define EPD_W21_MOSI_0  digitalWrite(m_pin_data,LOW)
#define EPD_W21_MOSI_1  digitalWrite(m_pin_data,HIGH)
#define EPD_W21_CLK_0 digitalWrite(m_pin_clock,LOW)
#define EPD_W21_CLK_1 digitalWrite(m_pin_clock,HIGH)
#define EPD_W21_CS_0 digitalWrite(m_pin_cs,LOW)
#define EPD_W21_CS_1 digitalWrite(m_pin_cs,HIGH)
#define EPD_W21_DC_0  digitalWrite(m_pin_dc,LOW)
#define EPD_W21_DC_1  digitalWrite(m_pin_dc,HIGH)

/**
 *
 */
gdeh0213b72::gdeh0213b72() :
		Adafruit_GFX(m_width, m_height) {
}

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
	m_send_command_and_send_data(0x11, 0x01); //data entry mode
	m_send_command_and_send_data(0x44, 0x00, 0x0F);             //set Ram-X address start/end position //0x0C-->(15+1)*8=128
	m_send_command_and_send_data(0x45, 0xF9, 0x00, 0x00, 0x00); //set Ram-Y address start/end position //0xF9-->(249+1)=250
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
 * Fills the entire screen with black pixels.
 * @note Modifications happen in ram, and will only be visible on the display after a call to one of the draw functions.
 * @return 0 in case of success, or a negative error code otherwise.
 */
int gdeh0213b72::fill_black(void) {
	int res;

	/* Send command to write into ram */
	m_send_command(0x24);

	/* Write contents of ram */
	for (size_t k = 0; k < m_width; k++) {
		for (size_t i = 0; i < (m_height + 8 - 1) / 8; i++) {
			m_send_data(0x00);
		}
	}

	/* Return success */
	return 0;
}

/**
 * Fills the entire screen with white pixels.
 * @note Modifications happen in ram, and will only be visible on the display after a call to one of the draw functions.
 * @return 0 in case of success, or a negative error code otherwise.
 */
int gdeh0213b72::fill_white(void) {
	int res;

	/* Send command to write into ram */
	m_send_command(0x24);

	/* Write contents of ram */
	for (size_t k = 0; k < m_width; k++) {
		for (size_t i = 0; i < (m_height + 8 - 1) / 8; i++) {
			m_send_data(0xff);
		}
	}

	/* Return success */
	return 0;
}

/**
 *
 * @return 0 in case of success, or a negative error code otherwise.
 */
int gdeh0213b72::draw_entire(void) {
	int res;

	/* Send display update command */
	m_send_command_and_data(0x22, 0xC7);

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
 *
 * @param[in] x The number of pixels along the horizontal axis, starting from the bottom.
 * @param[in] y The number of pixels along the vertical axis, starting from the top.
 * @param[in] color 0 for black, anything else for white.
 */
void gdeh0213b72::drawPixel(int16_t x, int16_t y, uint16_t color) {

	/* Ensure parameters are valid */
	if (x < 0 || (uint16_t) x >= m_width || y < 0 || (uint16_t) y >= m_height) {
		return;
	}

	/* Retrieve byte containing pixel value from ram */
	m_send_command_and_send_data(0x4E, (uint8_t )(x / 8));
	m_send_command_and_send_data(0x4F, (uint8_t )(y / 256), (uint8_t )(y % 256));
	m_send_command_and_send_data(0x27, 0x00, 0x00); // TODO : Read
	uint8_t byte;

	/* Modify byte */
	// TODO

	/* Write byte containing pixel value to ram */
	m_send_command_and_send_data(0x4E, (uint8_t )(x / 8));
	m_send_command_and_send_data(0x4F, (uint8_t )(y / 256), (uint8_t )(y % 256));
	m_send_command_and_send_data(0x24, 0x00, byte);
}

/**
 * Fills the screen with one of the two colors.
 * @param[in] color 0 for black, anything else for white.
 */
void gdeh0213b72::fillScreen(uint16_t color) {
	if (color) fill_white();
	else fill_black();
}

/**
 *
 * @param[in] i
 * @note Modifications will only be visible on the display after a call to one of the draw functions.
 */
void gdeh0213b72::invertDisplay(bool i) {

	/* Send display update control 1 command */
	if (i) {
		m_send_command_and_send_data(0x21, 0x08);
	} else {
		m_send_command_and_send_data(0x21, 0x00);
	}
}

/**
 *
 * @param xrate
 */
void gdeh0213b72::m_spi_delay(unsigned char xrate) {
	unsigned char i;
	while (xrate) {
		for (i = 0; i < 2; i++)
			;
		xrate--;
	}
}

/**
 *
 * @param value
 */
void gdeh0213b72::m_spi_write(unsigned char value) {
	unsigned char i;
	m_spi_delay(1);
	for (i = 0; i < 8; i++) {
		EPD_W21_CLK_0;
		m_spi_delay(1);
		if (value & 0x80)
		EPD_W21_MOSI_1;
		else
		EPD_W21_MOSI_0;
		value = (value << 1);
		m_spi_delay(1);
		delayMicroseconds(1);
		EPD_W21_CLK_1;
		m_spi_delay(1);
	}
}

/**
 * Waits for at least the given number of nanoseconds.
 * @param[in] ns The minimum number of nanoseconds to wait for.
 * @note This has a lot of room for improvement as right now times are rounded up to the nearest microsecond.
 */
void inline gdeh0213b72::m_delay_ns(const uint32_t ns) {
	delayMicroseconds((ns + 999) / 1000);
}

/**
 * Sends a command.
 * @param[in] command The command byte to send.
 * @return 0 in case of success, or a negative error code otherwise.
 * @see Datasheet section 7.2 "MCU Serial Peripheral Interface (4-wire SPI)".
 * @see Datasheet section 13 "Serial Peripheral Interface Timing".
 */
void gdeh0213b72::m_send_command(const uint8_t command) {

	/* Send command code */
	m_spi_delay(1);
	EPD_W21_CS_0;
	EPD_W21_DC_0;
	m_spi_write(command);
	EPD_W21_CS_1;
}

/**
 * Sends a data bye.
 * @param[in] data The data byte to send.
 * @return 0 in case of success, or a negative error code otherwise.
 * @see Datasheet section 7.2 "MCU Serial Peripheral Interface (4-wire SPI)".
 * @see Datasheet section 13 "Serial Peripheral Interface Timing".
 */
void gdeh0213b72::m_send_data(const uint8_t data) {

	/* Send data byte */
	m_spi_delay(1);
	EPD_W21_CS_0;
	EPD_W21_DC_1;
	m_spi_write(data);
	EPD_W21_CS_1;
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
