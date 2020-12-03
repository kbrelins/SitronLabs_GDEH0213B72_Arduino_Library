#ifndef SITRONLABS_GDEH0213B72_H
#define SITRONLABS_GDEH0213B72_H

/* Arduino libraries */
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>

/* C library */
#include <errno.h>
#include <stddef.h>

/**
 *
 */
class gdeh0213b72: public Adafruit_GFX {

public:
	gdeh0213b72();
	int setup(const uint8_t pin_busy, const uint8_t pin_res, const uint8_t pin_dc, const uint8_t pin_cs, const uint8_t pin_clock, const uint8_t pin_data);
	bool detect(void);
	int fill_black(void);
	int fill_white(void);
	int hibernate(void);
	int draw_entire(void);
	void drawPixel(int16_t x, int16_t y, uint16_t color);
	void fillScreen(uint16_t color);
	void invertDisplay(bool i);

private:
	uint8_t m_pin_busy;
	uint8_t m_pin_res;
	uint8_t m_pin_dc;
	uint8_t m_pin_cs;
	uint8_t m_pin_clock;
	uint8_t m_pin_data;
	const size_t m_width = 250;
	const size_t m_height = 122;
	void inline m_delay_ns(const uint32_t ns);
	void m_spi_delay(unsigned char xrate);
	void m_spi_write(unsigned char value);
	void m_send_command(const uint8_t command);
	void m_send_data(const uint8_t data);
	void m_send_command_and_read_data(const uint8_t command, uint8_t * const data, const size_t length);
	void m_lut_use(const uint8_t *wave_data);
	int m_busy_wait(void);
	static const PROGMEM uint8_t k_lut_entire[];
};

#endif
