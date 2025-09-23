/*!
 * @file Adafruit_SH110X.h
 *
 * This is part of for Adafruit's SH110X library for monochrome
 * OLED displays: http://www.adafruit.com/category/63_98
 *
 * These displays use I2C or SPI to communicate. I2C requires 2 pins
 * (SCL+SDA) and optionally a RESET pin. SPI requires 4 pins (MOSI, SCK,
 * select, data/command) and optionally a reset pin. Hardware SPI or
 * 'bitbang' software SPI are both supported.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Limor Fried/Ladyada for Adafruit Industries, with
 * contributions from the open source community.
 *
 * BSD license, all text above, and the splash screen header file,
 * must be included in any redistribution.
 *
 */

#ifndef ZEPHYR_DRIVERS_DISPLAY_SH1107_H_
#define ZEPHYR_DRIVERS_DISPLAY_SH1107_H_

/// fit into the SH110X_ naming scheme
#define SH110X_BLACK 0   ///< Draw 'off' pixels
#define SH110X_WHITE 1   ///< Draw 'on' pixels
#define SH110X_INVERSE 2 ///< Invert pixels

/* All following bytes will contain commands */
#define SH110X_CONTROL_ALL_BYTES_CMD		0x00
/* All following bytes will contain data */
#define SH110X_CONTROL_ALL_BYTES_DATA		0x40
/* The next byte will contain a command */
#define SH110X_CONTROL_BYTE_CMD		0x80
/* The next byte will contain data */
#define SH110X_CONTROL_BYTE_DATA		0xc0
#define SH110X_READ_STATUS_MASK		0xc0
#define SH110X_READ_STATUS_BUSY		0x80
#define SH110X_READ_STATUS_ON			0x40

// Uncomment to disable Adafruit splash logo
// #define SH110X_NO_SPLASH

#define SH110X_NOOP 0xE3
#define SH110X_MEMORYMODE 0x20          ///< See datasheet
#define SH110X_COLUMNADDR 0x21          ///< See datasheet
#define SH110X_PAGEADDR 0x22            ///< See datasheet
#define SH110X_SETCONTRAST 0x81         ///< See datasheet
#define SH110X_CHARGEPUMP 0x8D          ///< See datasheet
#define SH110X_SEGREMAP 0xA0            ///< See datasheet
#define SH110X_DISPLAYALLON_RESUME 0xA4 ///< See datasheet
#define SH110X_DISPLAYALLON 0xA5        ///< Not currently used
#define SH110X_NORMALDISPLAY 0xA6       ///< See datasheet
#define SH110X_INVERTDISPLAY 0xA7       ///< See datasheet
#define SH110X_SETMULTIPLEX 0xA8        ///< See datasheet
#define SH110X_DCDC 0xAD                ///< See datasheet
#define SH110X_DISPLAYOFF 0xAE          ///< See datasheet
#define SH110X_DISPLAYON 0xAF           ///< See datasheet
#define SH110X_SETPAGEADDR                                                     \
  0xB0 ///< Specify page address to load display RAM data to page address
       ///< register
#define SH110X_COMSCANINC 0xC0         ///< Not currently used
#define SH110X_COMSCANDEC 0xC8         ///< See datasheet
#define SH110X_SETDISPLAYOFFSET 0xD3   ///< See datasheet
#define SH110X_SETDISPLAYCLOCKDIV 0xD5 ///< See datasheet
#define SH110X_SETPRECHARGE 0xD9       ///< See datasheet
#define SH110X_SETCOMPINS 0xDA         ///< See datasheet
#define SH110X_SETVCOMDETECT 0xDB      ///< See datasheet
#define SH110X_SETDISPSTARTLINE                                                \
  0xDC ///< Specify Column address to determine the initial display line or
       ///< COM0.

#define SH110X_SETLOWCOLUMN       0x00  ///< Not currently used
#define SH110X_SETLOWCOLUMN_MASK	0x0f
#define SH110X_SETHIGHCOLUMN      0x10 ///< Not currently used
#define SH110X_SETSTARTLINE       0x40  ///< See datasheet

#define SH110X_RESET_DELAY			1

#endif // !ZEPHYR_DRIVERS_DISPLAY_SH1107_H_
