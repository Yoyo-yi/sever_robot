#ifndef CONTROL_HPP
#define CONTROL_HPP

#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <cstring>
#include <cstdio>

// /*定义手柄按键*/
// #define PSB_SELECT      1
// #define PSB_L3          2
// #define PSB_R3          3
// #define PSB_START       4
// #define PSB_PAD_UP      5
// #define PSB_PAD_RIGHT   6
// #define PSB_PAD_DOWN    7
// #define PSB_PAD_LEFT    8
// #define PSB_L2          9
// #define PSB_R2          10
// #define PSB_L1          11
// #define PSB_R1          12
// #define PSB_TRIANGLE    13
// #define PSB_CIRCLE      14
// #define PSB_CROSS       15
// #define PSB_SQUARE      16

/*定义手柄按键*/
#define PSB_SELECT      0b1111111111111110
#define PSB_L3          0b1111111111110101
#define PSB_R3          0b1111111111111011
#define PSB_START       0b1111111111110111
#define PSB_PAD_UP      0b1111111111101111
#define PSB_PAD_RIGHT   0b1111111111011111
#define PSB_PAD_DOWN    0b1111111110111111
#define PSB_PAD_LEFT    0b1111111101111111
#define PSB_L2          0b1111111011111111
#define PSB_R2          0b1111110111111111
#define PSB_L1          0b1111101111111111
#define PSB_R1          0b1111011111111111
#define PSB_TRIANGLE    0b1110111111111111
#define PSB_CIRCLE      0b1101111111111111
#define PSB_CROSS       0b1011111111111111
#define PSB_SQUARE      0b0111111111111111


/*回发过来的后4个数据是摇杆的数据,data中的数据位*/
#define PSS_RX 5        //右摇杆X轴数据
#define PSS_RY 6        //右摇杆Y轴数据
#define PSS_LX 7        //左摇杆X轴数据
#define PSS_LY 8        //右摇杆Y轴数据


class Controller {
public:
    Controller() {
        std::memset(data, 0, sizeof(data));
        handkey = 0;

        init();
    }

    void init() {  // 初始化手柄

        // 驱动库初始化
        if(wiringPiSetup() == -1){
		    printf("gpio init fail");
	    }

        if (wiringPiSPISetup(4, 500000) == -1) {
            printf("SPI init fail\n");
        }

        //io口初始化
        pinMode(PS2_CMD_PIN, OUTPUT);
        pinMode(PS2_CLK_PIN, OUTPUT);
        pinMode(PS2_DAT_PIN, INPUT);
        pinMode(PS2_SEL_PIN, OUTPUT);
        digitalWrite(PS2_CLK_PIN, HIGH);
        digitalWrite(PS2_SEL_PIN, HIGH);
        digitalWrite(PS2_CMD_PIN, HIGH);
    }

    unsigned char readAnalogData(unsigned char button) {  // 读取摇杆数据
        return data[button];
    }

    unsigned short getKey() {  // 获取按键类型
        clearData();
        digitalWrite(PS2_SEL_PIN, LOW);
        for (int i = 0; i < 9; i++) {
            data[i] = readData(scan[i]);
        }
        digitalWrite(PS2_SEL_PIN, HIGH);

        handkey = (data[4] << 8) | data[3];
        // printf("handkey :  %d /n" , handkey);
        // for (int index = 0; index < 16; index++) {
        //     if ((handkey & (1 << (mask[index] - 1))) == 0) {
        //         return index + 1;
        //     }
        // }
        return handkey;  // 没有按键按下
    }

private:
    void clearData() {  // 清除接收到的数据
        std::memset(data, 0, sizeof(data));
    }

    unsigned char readData(unsigned char command) {  // 读取数据
        unsigned char res = 0;
        unsigned char j = 1;

        for (int i = 0; i <= 7; i++) {
            digitalWrite(PS2_CMD_PIN, command & 0x01 ? HIGH : LOW);
            command >>= 1;
            delayMicroseconds(10);
            digitalWrite(PS2_CLK_PIN, LOW);
            delayMicroseconds(10);
            if (digitalRead(PS2_DAT_PIN) == HIGH) 
                res += j;
            j <<= 1;
            digitalWrite(PS2_CLK_PIN, HIGH);
            delayMicroseconds(10);
        }
        digitalWrite(PS2_CMD_PIN, HIGH);
        delayMicroseconds(50);

        return res;
    }

    unsigned short handkey;
    unsigned char data[9];
    const unsigned short mask[16] = {
        1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16
    };

    static const int PS2_DAT_PIN = 11;  // MOS
    static const int PS2_CMD_PIN = 12;  // MIS
    static const int PS2_SEL_PIN = 15;  // CS
    static const int PS2_CLK_PIN = 14;  // SCK
    static const unsigned char scan[9];  // 扫描指令

};

// 定义扫描指令
const unsigned char Controller::scan[9] = {
    0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

#endif // CONTROL_HPP
