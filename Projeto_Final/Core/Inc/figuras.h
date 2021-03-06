#ifndef FIGURAS_H_
#define FIGURAS_H_


//504 bytes por figura - 48x84 pixels

const unsigned char snake [] = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x80,
0xC0, 0xC0, 0xC0, 0x40, 0x60, 0x30, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x07, 0x0F, 0x0F, 0x1F, 0x1D, 0x3C, 0x38, 0x38, 0x78, 0x78, 0x78, 0x70, 0xF0, 0xF0, 0xF0, 0xF0,
0xF0, 0xF0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0x80, 0x80,
0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x70, 0xF8, 0xC8, 0xC8, 0x90, 0x00, 0xE0,
0xE0, 0x20, 0xE0, 0xC0, 0x00, 0x20, 0xA0, 0xA0, 0xE0, 0xC0, 0x00, 0xF8, 0xF8, 0x80, 0xE0, 0x60,
0x00, 0xC0, 0xE0, 0xA0, 0xE0, 0xC0, 0x00, 0x01, 0x01, 0x01, 0x03, 0x03, 0x03, 0x03, 0x03, 0x07,
0x07, 0x07, 0x87, 0x8F, 0x8F, 0xCF, 0xDF, 0xFF, 0xFF, 0xFF, 0xFE, 0xFE, 0xFE, 0xFE, 0x7C, 0x30,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x04, 0x04,
0x07, 0x03, 0x00, 0x07, 0x07, 0x00, 0x07, 0x07, 0x00, 0x03, 0x07, 0x04, 0x07, 0x07, 0x00, 0x07,
0x07, 0x01, 0x07, 0x06, 0x00, 0x03, 0x07, 0x04, 0x04, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x78, 0xFC, 0xFE, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xEF, 0xE7, 0xE7, 0xE3, 0xE3, 0xE1, 0xC1,
0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0xFC, 0xFE, 0x02, 0x02, 0x22, 0xE2, 0xE4, 0x00, 0xC8, 0xE8, 0x28, 0xF8, 0xF0, 0x00, 0xF8,
0xF8, 0x08, 0xF8, 0xF0, 0x08, 0xF8, 0xF0, 0x00, 0x00, 0xF0, 0xF8, 0x28, 0x38, 0x30, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x03,
0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07,
0x07, 0x0F, 0x0F, 0x0E, 0x1E, 0xBC, 0xF8, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x01, 0x01,
0x01, 0x01, 0x00, 0x01, 0x01, 0x00, 0x01, 0x01, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x01, 0x01,
0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x08, 0x0C, 0x04, 0x06, 0x03, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
};

const unsigned char press_button [] = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x80,
0xC0, 0xC0, 0xC0, 0x40, 0x60, 0x30, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x07, 0x0F, 0x0F, 0x1F, 0x1D, 0x3C, 0x38, 0x38, 0x78, 0x78, 0x78, 0x70, 0xF0, 0xF0, 0xF0, 0xF0,
0xF0, 0xF0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0x80, 0x80,
0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xE0, 0x20, 0xE0, 0xC0, 0x00, 0x80,
0x80, 0x80, 0x00, 0x00, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x80, 0x80, 0x80, 0x00, 0x00, 0x80,
0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x03, 0x03, 0x03, 0x03, 0x03, 0x07,
0x07, 0x07, 0x87, 0x8F, 0x8F, 0xCF, 0xDF, 0xFF, 0xFF, 0xFF, 0xFE, 0xFE, 0xFE, 0xFE, 0x7C, 0x30,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x1F, 0x02,
0x03, 0x01, 0x00, 0x1F, 0x1F, 0x00, 0x00, 0x0F, 0x1F, 0x12, 0x13, 0x13, 0x00, 0x13, 0x17, 0x1E,
0x0C, 0x00, 0x13, 0x17, 0x1E, 0x0C, 0x00, 0x18, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x78, 0xFC, 0xFE, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xEF, 0xE7, 0xE7, 0xE3, 0xE3, 0xE1, 0xC1,
0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0xF8, 0xF8, 0x48, 0x48, 0xF8, 0xB0, 0x00, 0xC0, 0xE0, 0x20, 0xE0, 0xC0, 0x20, 0xF0, 0xF0,
0x20, 0x00, 0x20, 0xA0, 0xA0, 0xE0, 0xC0, 0x00, 0xC0, 0xE0, 0x20, 0xE0, 0xC0, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x03,
0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07,
0x07, 0x0F, 0x0F, 0x0E, 0x1E, 0xBC, 0xF8, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x07, 0x04, 0x04, 0x07, 0x03, 0x00, 0x03, 0x07, 0x04, 0x07,
0x03, 0x00, 0x03, 0x07, 0x04, 0x00, 0x03, 0x07, 0x04, 0x07, 0x07, 0x00, 0x03, 0x07, 0x04, 0x07,
0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x08, 0x0C, 0x04, 0x06, 0x03, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
};


const unsigned char game_over [] = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x38, 0x70, 0xF8, 0xF8, 0xD0, 0xA0, 0x20, 0x20, 0x60, 0xE0, 0xC0, 0xC0, 0xC0, 0x00,
0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0xFC, 0x04, 0x04, 0x44, 0xC4,
0xC8, 0x00, 0xC0, 0xF0, 0x9C, 0x9C, 0xF0, 0xC0, 0x00, 0xFC, 0xFC, 0x38, 0xE0, 0x80, 0xE0, 0x38,
0xFC, 0xFC, 0x00, 0xFC, 0xFC, 0x24, 0x24, 0x24, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x40, 0x00,
0x00, 0x00, 0x00, 0x80, 0x80, 0x80, 0x00, 0x80, 0x80, 0x80, 0x83, 0x07, 0x1F, 0x1F, 0xFE, 0xFC,
0xC3, 0xC3, 0x07, 0x9E, 0x3E, 0x3E, 0x7D, 0xFE, 0xFE, 0xFE, 0xFC, 0xF0, 0xE0, 0xE0, 0x80, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03,
0x02, 0x02, 0x02, 0x03, 0x01, 0x00, 0x03, 0x03, 0x00, 0x00, 0x03, 0x03, 0x00, 0x03, 0x03, 0x00,
0x00, 0x03, 0x00, 0x00, 0x03, 0x03, 0x00, 0x03, 0x03, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x80, 0xC0, 0xC0, 0xE1, 0xB9,
0xBF, 0xBF, 0xBF, 0xDF, 0xEF, 0xEF, 0xF7, 0xF9, 0x7C, 0x7C, 0x7E, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0xFE, 0xF0, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x7E, 0xFF, 0x81, 0x81, 0x81, 0xFF, 0x7E, 0x00, 0x07, 0x3F, 0xF0, 0xF0, 0x3F, 0x07,
0x00, 0xFF, 0xFF, 0x89, 0x89, 0x89, 0x00, 0xFF, 0xFF, 0x09, 0x19, 0xFF, 0xE6, 0x00, 0x00, 0xDF,
0xDF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
0x01, 0x01, 0x01, 0x01, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0xB0,
0xB6, 0xB6, 0x36, 0x6D, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0xA0, 0xA0, 0xA4, 0x24, 0x6D, 0x6D, 0x6D, 0xDB, 0xFF, 0xFF, 0xFF, 0xFF, 0xE7, 0xE7, 0xF8, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x10, 0x10, 0x18, 0x1C, 0x1D, 0x1D, 0x1D, 0x19, 0x1B, 0x1B, 0x1E, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F,
0x1F, 0x1F, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 
};

const unsigned char dragon [] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0xC0, 0xC0, 0xE0,
	0x60, 0x30, 0x30, 0x18, 0x18, 0x08, 0x08, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x02, 0x06, 0x0C, 0x18, 0x70, 0xE0,
	0x80, 0x00, 0x00, 0x30, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00,
	0x04, 0x18, 0xE0, 0x80, 0x00, 0x00, 0xFE, 0x00, 0x80, 0xD0, 0xD8, 0xEC, 0xEE, 0xEE, 0xFF, 0xF7,
	0xF7, 0xF7, 0xF6, 0xF6, 0xF6, 0xEE, 0xEE, 0xEE, 0xEE, 0xCE, 0xDE, 0xDE, 0xBE, 0xBE, 0x7E, 0xF6,
	0xE6, 0xE6, 0xC6, 0x86, 0x04, 0x0C, 0x00, 0x08, 0x18, 0x18, 0x10, 0x20, 0x60, 0x40, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x0F, 0xFF, 0xFC, 0xC0, 0x80, 0xC3, 0xFF, 0xFE, 0xF8, 0xF0, 0xE0, 0xC0, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
	0x01, 0x02, 0x06, 0x0C, 0x18, 0x70, 0xE1, 0xDF, 0xFC, 0xF8, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0x1F, 0x07, 0x03, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x03, 0x07, 0x0F, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFE, 0xFE, 0xFD, 0xFB, 0xF7, 0xDF, 0xBE, 0xFC, 0xF0, 0xC0, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x1F, 0x3F, 0x7F, 0x07, 0x03, 0x01, 0x03,
	0x03, 0x07, 0x1F, 0xFF, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x0F, 0x3F, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xF8, 0xF0, 0xE0, 0xC0, 0xC0, 0x82, 0x86, 0x0C, 0x1C, 0x3C, 0x7C, 0x7C,
	0xFC, 0xFE, 0xFE, 0xFF, 0xEF, 0x87, 0x83, 0x00, 0x01, 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x7F, 0xFF,
	0xEF, 0x3F, 0xFC, 0xF0, 0x80, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x80, 0x80, 0xC0, 0xE0, 0xF0, 0xFC, 0x3F, 0x1F, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x03, 0x07, 0x07, 0x09, 0x1F, 0xFF, 0xFF, 0xE1, 0x80, 0x80, 0x40, 0x00, 0x01,
	0x01, 0x06, 0x00, 0x00, 0x00, 0x01, 0x01, 0x03, 0x03, 0x07, 0x07, 0x0F, 0x0E, 0x1C, 0x1C, 0x38,
	0x38, 0x30, 0x70, 0x70, 0x60, 0x60, 0x61, 0xC7, 0xFF, 0xFC, 0xC7, 0xC7, 0xC7, 0xC7, 0x8F, 0x8F,
	0x8E, 0x87, 0x87, 0x07, 0x07, 0x07, 0x03, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x1C,
	0x30, 0x60, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};
// ---------------------------------------------------------------------------------------
// Os pixeis n???o empregados na definic???o dos bytes DEVEM ser ZERO, ou seja, os pixeis que n???o devem ser impressos
// A figura ??? desenhada conforme trabalho do LCD, do LSB to MSB (bytes alinhados verticalmente, ver manual do LCD5110)
//-----------------------------------------------------------------------------------------------------------------
const struct figura_t fig1 = 
{
	4,							// largura em pixels = numero de colunas da matriz
	24,							// altura em pixels = cada 8 pixels corresponte a uma linha da matriz
	{0b11111111, 0b10000011, 0b10000011, 0b11111111,
	 0b11111110, 0b11000001, 0b11000001, 0b11111110,
	 0b11111110, 0b11000001, 0b11000001, 0b11111110,}
};


#endif /* FIGURAS_H_ */
