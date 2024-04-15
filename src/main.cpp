#include <Arduino.h>
#include "FlexiPLCLITE.h"
#include <SerialTransfer.h>
#include <I2CTransfer.h>
#include <U8g2lib.h>
#include <Wire.h>
#include "stm32g0xx_hal.h"
#include <EEPROM.h>
#include "LXGWNeoXiHei.h"

#define SCREEN_WIDTH 128 // 使用的是 128×64 OLED 显示屏
#define SCREEN_HEIGHT 32 //
#define scl PA9
#define sda PA10

#define MAX_TICK 15000
#define MIN_TICK 1

void timerCallback();
void printMenu();
void generatePulse(bool active) ;
void pulseOut(char num,bool status);
void buttonOK();
void buttonDOWN();
void buttonUP();
void buttonCANCEL();
void InitOut();

typedef enum {
    STATE_INITIALIZED,
    STATE_WAITING_FOR_PULSE,
    STATE_GENERATING_PULSE,
    STATE_WAITING_FOR_INTERVAL
} State;

typedef enum {
    MainMenu,
    SetPulse,
    SetWidthTicks,
    SetIntervalTicks,
    ChoseMenu
} MenuState;

HardwareTimer *timer = new HardwareTimer(TIM3);
HardwareSerial Serial1(PB7,PB6);

U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R2);

struct Setting{
    int pulse_width_ticks = 100; // 脉冲宽度的计数器（以20ms为单位）
    int pulse_interval_ticks = 200; // 脉冲间隔的计数器（以20ms为单位）
    char MaxPulse = 12;//脉冲数量(继电器数量)
}setting;

MenuState menuState = MainMenu;
State state = STATE_INITIALIZED; // 当前状态

int tick = 0; // 20ms定时器的计数器
bool pulse_active = false; // 脉冲是否激活的标志
char ChoseMenuIndex =0;//菜单项
bool active_status = false;//脉冲状态
char pulse = 1;//脉冲
bool flush = true;



void setup() {
    Wire.setSCL(PA9);
    Wire.setSDA(PA10);
    u8g2.setBusClock(1000000);
    Wire.setClock(1000000);
    u8g2.begin();
    u8g2.enableUTF8Print();
    Serial1.begin(115200);



    EEPROM.get(0,setting);
    if(setting.MaxPulse>12||setting.MaxPulse<1){
        setting.MaxPulse=12;
        setting.pulse_width_ticks=100;
        setting.pulse_interval_ticks=200;
        EEPROM.put(0,setting);
    }else{
        EEPROM.get(0,setting);
    }

    InitOut();

    pinMode(BTN_OK, INPUT_PULLUP);
    pinMode(BTN_CANCEL, INPUT_PULLUP);
    pinMode(BTN_UP, INPUT_PULLUP);
    pinMode(BTN_DOWN, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(BTN_OK), buttonOK, LOW);
    attachInterrupt(digitalPinToInterrupt(BTN_CANCEL), buttonCANCEL, LOW);
    attachInterrupt(digitalPinToInterrupt(BTN_UP), buttonUP, LOW);
    attachInterrupt(digitalPinToInterrupt(BTN_DOWN), buttonDOWN, LOW);

    timer->setPrescaleFactor(6400);
    timer->setOverflow(1001);
    timer->attachInterrupt(timerCallback);

    timer->resume();
}
char strSeconds[32];
void loop() {
    if(!flush){
        return;
    }
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_LXGWNeoXiHei);
    u8g2.setCursor(0, 14);
    printMenu();
    u8g2.setCursor(0, 30);
    float seconds = (float)tick * 0.1f;
    static char buffer[10];
    dtostrf(seconds, 5, 1, buffer);
    static char statusStr[4] = "OFF";
    if (active_status){
        strcpy(statusStr,"ON");
    }else{
        strcpy(statusStr,"OFF");
    }
    sprintf(strSeconds, "脉冲%2d %3s%sS", pulse,statusStr,buffer);
    u8g2.print(strSeconds);
    u8g2.sendBuffer();
    flush = false;
}
// 定时器回调函数
void timerCallback() {
    // 状态机逻辑
    switch (state) {
        case STATE_INITIALIZED:
            // 初始化后直接进入等待脉冲状态
            state = STATE_WAITING_FOR_PULSE;
            tick = 0; // 重置计数器
            break;

        case STATE_WAITING_FOR_PULSE:
            if (tick >= setting.pulse_interval_ticks) {
                // 到达生成脉冲的时间
                tick = 0; // 重置计数器
                state = STATE_GENERATING_PULSE;
                pulse_active = true; // 激活脉冲
            }
            break;

        case STATE_GENERATING_PULSE:
            if (tick >= setting.pulse_width_ticks) {
                // 脉冲生成完成
                pulse_active = false; // 禁止脉冲
                state = STATE_WAITING_FOR_INTERVAL;
                tick = 0; // 重置计数器
            }
            break;

        case STATE_WAITING_FOR_INTERVAL:
            if (tick >= setting.pulse_interval_ticks) {
                // 等待间隔完成，准备生成下一个脉冲
                state = STATE_GENERATING_PULSE;
                pulse_active = true; // 激活下一个脉冲
                tick = 0; // 重置计数器
            }
            break;

        default:
            // 未知状态，应该不会发生
            state = STATE_INITIALIZED;
            break;
    }

    // 根据脉冲状态执行相关操作
    generatePulse(pulse_active);
    // 每次回调增加计数器
    tick++;
    flush = true;
}
void generatePulse(bool active) {
    if (active_status==active){
        return;
    }
    active_status=active;
    if(active_status){
        pulseOut(pulse,true);
    }else{
        pulseOut(pulse,false);
        pulse++;
        if(pulse>setting.MaxPulse){
            pulse=1;
        }
    }
}
void ChoseMenuFunc(){
    switch (ChoseMenuIndex) {
        case 0:
            u8g2.print("1.脉冲数设置");
            break;
        case 1:
            u8g2.print("2.脉宽设置");
            break;
        case 2:
            u8g2.print("3.脉冲间隔设置");
            break;
        case 3:
            u8g2.print("4.保存");
            break;
        case 4:
            u8g2.print("5.返回");
            break;
        case 5:
            u8g2.print("保存完成");
            break;
        default:
            u8g2.print("错误！");
            break;
    }
}
void printMenu(){
    switch (menuState){
        case MainMenu:
            u8g2.print("按确认进入菜单");
            break;
        case SetPulse:
            u8g2.printf("脉冲数:%d",setting.MaxPulse);
            break;
        case SetWidthTicks:
            {
                float seconds = (float)setting.pulse_width_ticks * 0.1f;
                static char buffer[10];
                dtostrf(seconds, 10, 1, buffer);
                u8g2.printf("脉宽:%sS",buffer);
            }
            break;
        case SetIntervalTicks:
            {
                float seconds = (float)setting.pulse_interval_ticks * 0.1f;
                static char buffer[10];
                dtostrf(seconds, 10, 1, buffer);
                u8g2.printf("间隔:%sS",buffer);
            }
            break;
        case ChoseMenu:
            ChoseMenuFunc();
            break;
    }
}
void buttonOKChoseMenu(){
    switch (ChoseMenuIndex) {
        case 0:
            menuState=SetPulse;
            break;
        case 1:
            menuState=SetWidthTicks;
            break;
        case 2:
            menuState=SetIntervalTicks;
            break;
        case 3:
            EEPROM.put(0,setting);
            ChoseMenuIndex=5;
            break;
        default:
            menuState=MainMenu;
            break;
    }
}
void buttonOK(){
    switch (menuState){
        case MainMenu:
            ChoseMenuIndex  = 0;
            menuState=ChoseMenu;
            break;
        case SetPulse:
            break;
        case SetWidthTicks:
            break;
        case SetIntervalTicks:
            break;
        case ChoseMenu:
            buttonOKChoseMenu();
            break;
    }
}
void buttonDOWN(){
    switch (menuState){
        case MainMenu:
            break;
        case SetPulse:
            if(setting.MaxPulse<=1){
                setting.MaxPulse=12;
            }else{
                setting.MaxPulse--;
            }
            break;
        case SetWidthTicks:
            if(setting.pulse_width_ticks<=0){
                setting.pulse_width_ticks=100;
            }else{
                setting.pulse_width_ticks-=1;
            }
            break;
        case SetIntervalTicks:
            if(setting.pulse_interval_ticks<=0){
                setting.pulse_interval_ticks=200;
            }else{
                setting.pulse_interval_ticks-=1;
            }
            break;
        case ChoseMenu:
            if (ChoseMenuIndex>=4){
                ChoseMenuIndex=0;
            }else{
                ChoseMenuIndex++;
            }
            break;
    }
}
void buttonUP(){
    switch (menuState){
        case MainMenu:
            break;
        case SetPulse:
            if(setting.MaxPulse==12){
                setting.MaxPulse=1;
            }else{
                setting.MaxPulse++;
            }
            break;
        case SetWidthTicks:
            if(setting.pulse_width_ticks>=MAX_TICK){
                setting.pulse_width_ticks=1;
            }else{
                setting.pulse_width_ticks+=1;
            }
            break;
        case SetIntervalTicks:
            if(setting.pulse_interval_ticks>=MAX_TICK){
                setting.pulse_interval_ticks=1;
            }else{
                setting.pulse_interval_ticks+=1;
            }
            break;
        case ChoseMenu:
            if (ChoseMenuIndex==0){
                ChoseMenuIndex=4;
            }else{
                ChoseMenuIndex--;
            }
            break;
    }
}
void buttonCANCEL(){
    switch (menuState){
        case MainMenu:
            menuState=MainMenu;
            break;
        case SetPulse :
            menuState=ChoseMenu;
            break;
        case SetWidthTicks:
            menuState=ChoseMenu;
            break;
        case SetIntervalTicks:
            menuState=ChoseMenu;
            break;
        case ChoseMenu:
            menuState=MainMenu;
            break;
    }
}
void InitOut(){
    pinMode(OUT0, OUTPUT);
    pinMode(OUT1, OUTPUT);
    pinMode(OUT2, OUTPUT);
    pinMode(OUT3, OUTPUT);
    pinMode(OUT4, OUTPUT);
    pinMode(OUT5, OUTPUT);
    pinMode(OUT6, OUTPUT);
    pinMode(OUT7, OUTPUT);
    pinMode(OUT8, OUTPUT);
    pinMode(OUT9, OUTPUT);
    pinMode(OUT10, OUTPUT);
    pinMode(OUT11, OUTPUT);
}
void closeAll(){
    digitalWrite(OUT0, LOW);
    digitalWrite(OUT1, LOW);
    digitalWrite(OUT2, LOW);
    digitalWrite(OUT3, LOW);
    digitalWrite(OUT4, LOW);
    digitalWrite(OUT5, LOW);
    digitalWrite(OUT6, LOW);
    digitalWrite(OUT7, LOW);
    digitalWrite(OUT8, LOW);
    digitalWrite(OUT9, LOW);
    digitalWrite(OUT10, LOW);
    digitalWrite(OUT11, LOW);
}
void pulseOut(char num,bool status){
    closeAll();
    Serial1.printf("当前脉冲：%2d 状态：%d\n",num,status);
    if(status){
        switch (num) {
            case 1:
                digitalWrite(OUT0, HIGH);
                break;
            case 2:
                digitalWrite(OUT1, HIGH);
                break;
            case 3:
                digitalWrite(OUT2, HIGH);
                break;
            case 4:
                digitalWrite(OUT3, HIGH);
                break;
            case 5:
                digitalWrite(OUT4, HIGH);
                break;
            case 6:
                digitalWrite(OUT5, HIGH);
                break;
            case 7:
                digitalWrite(OUT6, HIGH);
                break;
            case 8:
                digitalWrite(OUT7, HIGH);
                break;
            case 9:
                digitalWrite(OUT8, HIGH);
                break;
            case 10:
                digitalWrite(OUT9, HIGH);
                break;
            case 11:
                digitalWrite(OUT10, HIGH);
                break;
            case 12:
                digitalWrite(OUT11, HIGH);
                break;
            default:
                closeAll();
                break;
        }
    }
}

