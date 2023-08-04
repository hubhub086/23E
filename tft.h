#ifndef _TFT_H_
#define _TFT_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
// for spi devices
#include <spidev_lib++.h>
// for delay function.
#include <chrono>
#include <unistd.h> 
#include <thread>
// for signal handling
#include <signal.h>
#include <JetsonGPIO.h>
// for img process
#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"

#include <time.h>

void LCDReset();
void sendCommand(SPI* spi, uint8_t* command, uint8_t bytes[], uint8_t length);
void sendSignalCommand(SPI* spi, uint8_t* command);
void sendManyBytes(SPI* spi, uint8_t *bytes, int length);
void st7789_init(SPI* spi);
void ILI9486_init(SPI* spi);
void setWindow(SPI* spi);
void img2rgb565(cv::Mat img, uint8_t* rgb565);
void drawImg16BitColor(SPI* spi, cv::Mat img);
void lcd_init();
void* lcd_show_process(void* ptr);


#endif