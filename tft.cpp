
#include "tft.h"

using namespace std;
using namespace GPIO;
using namespace cv;

int screenWidth = 240; //屏幕长度
int screenHeight = 240; //屏幕宽度
int PinDC = 22; //GPIO.BOARD引脚模式，第18号引脚
int PinReset = 18;  //GPIO.BOARD引脚模式，第22号引脚
int PinLED = 12; //GPIO.BOARD引脚模式，第12号引脚
vector<Mat> show_buffer;

spi_config_t spi_config;
SPI *mySPI = NULL;
uint8_t tx_buffer[32];
uint8_t rx_buffer[32];

void LCDReset()
{
    // 重置电平时序
    GPIO::output(PinReset, 1);
    usleep(1000*200);
    GPIO::output(PinReset, 0);
    usleep(1000*200);
    GPIO::output(PinReset, 1);
    usleep(1000*200);
}

void sendCommand(SPI* spi, uint8_t* command, uint8_t bytes[], uint8_t length)
{
    GPIO::output(PinDC, 0);
    spi->write(command, 1);
    GPIO::output(PinDC, 1);
    spi->write(bytes, length);
    GPIO::output(PinDC, 1);
}

void sendSignalCommand(SPI* spi, uint8_t* command)
{
    GPIO::output(PinDC, 0);
    // cout << strlen((char*)command) << endl;
    spi->write(command, 1);
}

void sendManyBytes(SPI* spi, uint8_t *bytes, int length)
{
    GPIO::output(PinDC, 1);
    spi->write(bytes, length);
}

void st7789_init(SPI* spi)
{
    GPIO::output(PinLED, 1);
    LCDReset();
    uint8_t command = 0x36;
    uint8_t bytes[1] = {0x00};
    sendCommand(spi, &command, bytes, sizeof(bytes));
    command = 0x3A;
    uint8_t bytes1[1] = {0x05};
    sendCommand(spi, &command, bytes1, sizeof(bytes1));
    command = 0xB2;
    uint8_t bytes2[5] = {0x0C,0x0C,0x00,0x33,0x33};
    sendCommand(spi, &command, bytes2, sizeof(bytes2));
    command = 0xB7;
    uint8_t bytes3[1] = {0x35};
    sendCommand(spi, &command, bytes3, sizeof(bytes3));
    command = 0xBB;
    uint8_t bytes4[1] = {0x19};
    sendCommand(spi, &command, bytes4, sizeof(bytes4));
    command = 0xC0;
    uint8_t bytes5[1] = {0x2C};
    sendCommand(spi, &command, bytes5, sizeof(bytes5));
    command = 0xC2;
    uint8_t bytes6[1] = {0x01};
    sendCommand(spi, &command, bytes6, sizeof(bytes6));
    command = 0xC3;
    uint8_t bytes7[1] = {0x12};
    sendCommand(spi, &command, bytes7, sizeof(bytes7));
    command = 0xC4;
    uint8_t bytes8[1] = {0x20};
    sendCommand(spi, &command, bytes8, sizeof(bytes8));
    command = 0xC6;
    uint8_t bytes9[1] = {0x0F};
    sendCommand(spi, &command, bytes9, sizeof(bytes9));
    command = 0xD0;
    uint8_t bytes10[2] = {0xA4,0xA1};
    sendCommand(spi, &command, bytes10, sizeof(bytes10));
    command = 0xE0;
    uint8_t bytes11[14] = {0xD0,0x04,0x0D,0x11,0x13,0x2B,0x3F,0x54,0x4C,0x18,0x0D,0x0B,0x1F,0x23};
    sendCommand(spi, &command, bytes11, sizeof(bytes11));
    command = 0xE1;
    uint8_t bytes12[14] = {0xD0,0x04,0x0C,0x11,0x13,0x2C,0x3F,0x44,0x51,0x2F,0x1F,0x1F,0x20,0x23};
    sendCommand(spi, &command, bytes12, sizeof(bytes12));
    command = 0x21;
    sendSignalCommand(spi, &command);
    command = 0x11;
    sendSignalCommand(spi, &command);
    command = 0x29;
    sendSignalCommand(spi, &command);
    cout << "finish sending" << endl;
}

void ILI9486_init(SPI* spi)
{
    GPIO::output(PinLED, 1);
    LCDReset();
    uint8_t command = 0xF1;
    uint8_t bytes[6] = {0x36,0x04,0x00,0x3c,0x0F,0x8F};
    sendCommand(spi, &command, bytes, sizeof(bytes));
    command = 0xF2;
    uint8_t bytes1[9] = {0x18,0xA3,0x12,0x02,0xB2,0x12,0xFF,0x10,0x00};
    sendCommand(spi, &command, bytes1, sizeof(bytes1));
    command = 0xF8;
    uint8_t bytes2[2] = {0x21,0x04};
    sendCommand(spi, &command, bytes2, sizeof(bytes2));
    command = 0xF9;
    uint8_t bytes3[2] = {0x00, 0x08};
    sendCommand(spi, &command, bytes3, sizeof(bytes3));
    command = 0xB4;
    uint8_t bytes4[1] = {0x00};
    sendCommand(spi, &command, bytes4, sizeof(bytes4));
    command = 0xC1;
    uint8_t bytes5[1] = {0x47};
    sendCommand(spi, &command, bytes5, sizeof(bytes5));
    command = 0xC5;
    uint8_t bytes6[4] = {0x00,0xAF,0x80,0x00};
    sendCommand(spi, &command, bytes6, sizeof(bytes6));
    command = 0xE0;
    uint8_t bytes11[15] = {0x0F,0x1F,0x1C,0x0C,0x0F,0x08,0x48,0x98,0x37,0x0A,0x13,0x04,0x11,0x0D,0x00};
    sendCommand(spi, &command, bytes11, sizeof(bytes11));
    command = 0xE1;
    uint8_t bytes12[15] = {0x0F,0x32,0x2E,0x0B,0x0D,0x05,0x47,0x75,0x37,0x06,0x10,0x03,0x24,0x20,0x00};
    sendCommand(spi, &command, bytes12, sizeof(bytes12));
    command = 0x3A;
    uint8_t bytes13[1] = {0x66};
    sendCommand(spi, &command, bytes13, sizeof(bytes13));
    command = 0x11;
    sendSignalCommand(spi, &command);
    command = 0x36;
    uint8_t bytes14[1] = {0x29};
    sendCommand(spi, &command, bytes14, sizeof(bytes14));
    usleep(1000*12);
    command = 0x29;
    sendSignalCommand(spi, &command);
    cout << "finish sending" << endl;
}


void setWindow(SPI* spi)
{
    uint8_t command = 0x2A;
    sendSignalCommand(spi, &command);
    uint8_t bytes[2] = {0>>8,0x00FF&0};
    sendManyBytes(spi, bytes, sizeof(bytes));
    uint8_t x1 = screenHeight - 1;
    bytes[0] = x1>>8; bytes[1] = x1;
    sendManyBytes(spi, bytes, sizeof(bytes));

    command = 0x2B;
    sendSignalCommand(spi, &command);
    bytes[0] = 0>>8; bytes[1] = 0;
    sendManyBytes(spi, bytes, sizeof(bytes));
    uint8_t y1 = screenWidth - 1;
    bytes[0] = y1>>8; bytes[1] = y1;
    sendManyBytes(spi, bytes, sizeof(bytes));

    command = 0x2C;
    sendSignalCommand(spi, &command);
    cout << "setWindow dnow" << endl;
}

void img2rgb565(Mat img, uint8_t* rgb565)
{
    int count = 0;
    for (int x=0; x < screenWidth; x++)
    {
        for (int y=screenHeight-1; y>=0; y--)
        {
            Vec3b pixel = img.at<Vec3b>(y, x);
            uint8_t blue = pixel[0]; 
            uint8_t green = pixel[1]; 
            uint8_t red = pixel[2]; 
            red = red >> 3;
            green = green >> 2;
            blue = blue >> 3;
            int highBit = 0| (red<<3) | (green>>3);
            int lowBit  = 0 | (green << 5) | blue;
            rgb565[count] = highBit;
            count++;
            rgb565[count] = lowBit;
            count++;
        }
    }
}

void drawImg16BitColor(SPI* spi, Mat img)
{
    uint8_t bytes[115200] = {};
    GPIO::output(PinDC, 1);
    img2rgb565(img, bytes);

    // sendManyBytes(spi, bytes, sizeof(bytes));
    clock_t start=clock();
    // for (int i=0; i < sizeof(bytes); i++)
    // {
    //     spi->write(&bytes[i], 1);
    // }
    int buffer_size = 255;
    uint8_t send_buffer[255] = {}; 
    int i = 0;
    for (; i < (int)sizeof(bytes)/buffer_size; i++)
    {
        if (i*buffer_size+buffer_size > sizeof(bytes))
        {
            copy(bytes+i*buffer_size, bytes+sizeof(bytes)-1, send_buffer);

            spi->write(send_buffer, sizeof(bytes)-i*buffer_size);
        }
        else{
            copy(bytes+i*buffer_size, bytes+i*buffer_size+buffer_size, send_buffer);
            spi->write(send_buffer, buffer_size);
        }
        
    }
    double Times=(double)(clock()-start)/CLOCKS_PER_SEC;
    cout << Times <<"send finish" << endl;
}

void lcd_init()
{
    GPIO::setmode(BOARD);
    GPIO::setwarnings(false);
    GPIO::setup(PinDC, GPIO::OUT);
    GPIO::setup(PinReset, GPIO::OUT);
    GPIO::setup(PinLED, GPIO::OUT);
    spi_config.mode=0;
    spi_config.speed=24000000;
    spi_config.delay=0;
    spi_config.bits_per_word=8;

    mySPI=new SPI("/dev/spidev0.0",&spi_config);
    cout << mySPI->begin() << endl;
    st7789_init(mySPI);
    setWindow(mySPI);
}

void* lcd_show_process(void* ptr)
{
    Mat img;
    cout << "get in lcd" << endl;
    while(1)
    {
        if (show_buffer.size() > 0)
        {
            img = show_buffer[0];
            resize(img, img, Size(240, 240));
            drawImg16BitColor(mySPI, img);
	    show_buffer.clear();
        }
    }
}


// int main( void)
// {
//     GPIO::setmode(BOARD);
//     GPIO::setwarnings(false);
//     GPIO::setup(PinDC, GPIO::OUT);
//     GPIO::setup(PinReset, GPIO::OUT);d
//     GPIO::setup(PinLED, GPIO::OUT);
//     SPI *mySPI = NULL;
//     spi_config.mode=0;
//     spi_config.speed=24000000;
//     spi_config.delay=0;
//     spi_config.bits_per_word=8;

//     mySPI=new SPI("/dev/spidev0.0",&spi_config);
//     cout << mySPI->begin() << endl;
//     st7789_init(mySPI);
//     setWindow(mySPI);

//     // Mat img = imread("../test.jpg");
//     Mat img = imread("../test_hsv.png");
//     resize(img, img, Size(240, 240));

//     drawImg16BitColor(mySPI, img);
//     delete mySPI;
//     return 1;
// }
