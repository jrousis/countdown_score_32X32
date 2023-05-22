// Visual Micro is in vMicro>General>Tutorial Mode
// 
/*
    Name:       countdown_score_32X32.ino
    Created:	12/4/2023 9:23:24 πμ
    Author:     ROUSIS_FACTORY\user

    Scoreboard With only Countdown timer and 3 digits score
    With remote control (maxairidi)
*/
#include <EEPROM.h>
#define EEPROM_SIZE 100
#include <DMD32_B.h>        //
#include "fonts/SystemFont5x7.h"
#include "fonts/scoreboard.h"
#include "Chrono.h"

//EEPROM data ADDRESSER
#define EEP_COUNTDOWN_SET 0

#define TIMEOUT_SET 4
#define RS485_ENABLE 0
#define RXD2 16
#define TXD2 17
#define RS485_PIN_DIR 4
HardwareSerial rs485(1);
#define RS485_WRITE     1
#define RS485_READ      0

#define RXD2 16
#define TXD2 17

//Fire up the DMD library as dmd
#define DISPLAYS_ACROSS 1
#define DISPLAYS_DOWN 2
DMD dmd(DISPLAYS_ACROSS, DISPLAYS_DOWN, PROTOCOL_QIANGLI); // PROTOCOL_QIANGLI);
#define BUZZER 15
#define PIXELS_X (DISPLAYS_ACROSS * 32)
#define PIXELS_Y (DISPLAYS_DOWN * 16)
//-----------------------------------------------------------------------
//Timer setup
//create a hardware timer  of ESP32
hw_timer_t* timer = NULL;
hw_timer_t* flash_timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE falshMux = portMUX_INITIALIZER_UNLOCKED;
/*--------------------------------------------------------------------------------------
  Interrupt handler for Timer1 (TimerOne) driven DMD refresh scanning, this gets
  called at the period set in Timer1.initialize();
--------------------------------------------------------------------------------------*/
#define INSTR_START 0x11
#define INSTR_STOP 0x12
#define INSTR_COUNTDOWN_MODE 0x13
#define INSTR_RESET 0x14

#define COUNTDOWN_EN 1
#define COUNTDOWN_TIME 10*60 //10 Minutes

bool remote_start = false;
bool remote_stop = false;
bool remote_reset = false;

static unsigned long oldTime = 0;
uint8_t buzzer_cnt = 0;
uint8_t flash_cnt = 0;
bool flash_on = false;
bool Scan = false;
uint8_t Address;
uint8_t In_bytes_count = 0;
uint16_t Timeout_delay = 0;
unsigned int compare_elapsed = 10000;
uint16_t countdown_time;
bool countdown_on = false;
bool countdown_set_on = false;
char score[4] = { '0','0','0',0 };
char countdown_buf[6] = { '0','0',':','0','0',0 };

char  Line1_buf[10] = { 0 };
char  Line2_buf[10] = { 0 };
char  Line3_buf[10] = { 0 };
char  Line4_buf[10] = { 0 };
//------------------------------------------------------------------------------------------
Chrono chrono(Chrono::SECONDS);

void IRAM_ATTR triggerScan()
{
    Scan = true;
    portENTER_CRITICAL_ISR(&timerMux);

    dmd.scanDisplayBySPI();

    portEXIT_CRITICAL_ISR(&timerMux);
    Scan = false;
}

void IRAM_ATTR FlashInt()
{
    portENTER_CRITICAL_ISR(&falshMux);

    if (flash_cnt)
    {
        if (flash_on)
        {
            dmd.drawString(1, 0, "   > ", 5, GRAPHICS_NORMAL);
            flash_on = false;
        }
        else {
            dmd.drawString(1, 0, Line1_buf, 5, GRAPHICS_NORMAL);
            flash_on = true;
        }
        flash_cnt--;
    }

    if (buzzer_cnt) {
        buzzer_cnt--;
    }
    else {
        digitalWrite(BUZZER, LOW);
    }

    portEXIT_CRITICAL_ISR(&falshMux);
}

TaskHandle_t Task0;

//----------------------------------------------------------------------------------------
void EEPWriteArray(char address, char* arr)
{
    unsigned char k;
    for (size_t i = 0; i < sizeof(arr); i++)
    {
        k = arr[i];
        EEPROM.write(address + i, k);        
    }
    EEPROM.commit();
}

void EEPReadArray(char address, char *arr)
{
    unsigned char k;
    for (size_t i = 0; i < sizeof(arr); i++)
    {
        k = EEPROM.read(address + i);
        arr[i] = k;
    }
}

void setup()
{
    Serial.begin(115200);
    chrono.start();
    chrono.stop();

    Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2); //Transmiter for slave display prices
    pinMode(4, OUTPUT);

    pinMode(BUZZER, OUTPUT);
    digitalWrite(BUZZER, LOW);

    EEPROM.begin(EEPROM_SIZE);
    EEPReadArray(EEP_COUNTDOWN_SET, countdown_buf);
    if (countdown_buf[0] == 0xff && countdown_buf[1])
    {
        countdown_buf[0] = '0'; countdown_buf[1] = '0'; countdown_buf[2] = ':';
        countdown_buf[3] = '0'; countdown_buf[4] = '0';
    }

    delay(100);
    Serial.println("Start initialize...");

    //------------------------------------------------------------------------
    // return the clock speed of the CPU
    uint8_t cpuClock = ESP.getCpuFreqMHz();

    // Use 1st timer of 4 
    // devide cpu clock speed on its speed value by MHz to get 1us for each signal  of the timer
    timer = timerBegin(0, cpuClock, true);
    // Attach triggerScan function to our timer 
    timerAttachInterrupt(timer, &triggerScan, true);
    // Set alarm to call triggerScan function  
    // Repeat the alarm (third parameter) 
    timerAlarmWrite(timer, 300, true);
    // Start an alarm 
    timerAlarmEnable(timer);

    flash_timer = timerBegin(1, cpuClock, true);
    timerAttachInterrupt(flash_timer, &FlashInt, true);
    timerAlarmWrite(flash_timer, 100000, true);
    timerAlarmEnable(flash_timer);


    delay(100);
    //clear/init the DMD pixels held in RAM
   //printPinsStatus();
    dmd.setBrightness(160);
    dmd.selectFont(System5x7);
    dmd.drawString(1, 0, "Rousi", 5, GRAPHICS_NORMAL);
    dmd.drawString(1, 8, "LTD  ", 5, GRAPHICS_NORMAL);
    dmd.drawString(1, 16, "CntDn", 5, GRAPHICS_NORMAL);
    dmd.drawString(1, 24, "Score ", 5, GRAPHICS_NORMAL);
    delay(2000);
    dmd.drawString(1, 16, "3/23  ", 6, GRAPHICS_NORMAL);
    dmd.drawString(1, 24, "V1.1   ", 6, GRAPHICS_NORMAL);
    delay(2000);

    show_scoreboard();

    //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
    xTaskCreatePinnedToCore(
        Task0code,   /* Task function. */
        "Task0",     /* name of task. */
        10000,       /* Stack size of task */
        NULL,        /* parameter of the task */
        0,           /* priority of the task */
        &Task0,      /* Task handle to keep track of created task */
        0);          /* pin task to core 0 */
    delay(500);

    buzzer_ring(4);
}

void Task0code(void* pvParameters) {

    int minutes = 0;
    int seconds = 0;
    uint8_t laps_count = 0;
    uint8_t lap_mode = 0;    
    char buffer[5];
    char Clock_display[6];
    for (;;) {
        if (countdown_on && chrono.hasPassed(countdown_time))
        {
            chrono.restart();
            chrono.stop();
            timerAlarmDisable(flash_timer);

            digitalWrite(BUZZER, HIGH);
            dmd.clearScreen(true);
            delay(1000);
            digitalWrite(BUZZER, LOW);
            show_scoreboard();

            delay(1000);
            digitalWrite(BUZZER, HIGH);
            dmd.clearScreen(true);
            delay(1000);
            digitalWrite(BUZZER, LOW);
            show_scoreboard();

            delay(1000);
            digitalWrite(BUZZER, HIGH);
            dmd.clearScreen(true);            
            delay(1000);
            digitalWrite(BUZZER, LOW);
            show_scoreboard();

            timerAlarmEnable(flash_timer);
            countdown_on = false;
        }
        if (compare_elapsed != chrono.elapsed())
        {
            if (chrono.elapsed() > 5940)
            {
                chrono.restart();
                minutes = 0;
                seconds = 0;
            }
            int countdown_elapled;
            if (countdown_on){
                countdown_elapled = countdown_time - chrono.elapsed();
            }
            else {
                countdown_elapled = chrono.elapsed();
            }
            
            seconds = countdown_elapled % 60;
            minutes = (countdown_elapled / 60) % 60;
            /*seconds = chrono.elapsed() % 60;
            minutes = (chrono.elapsed() / 60) % 60;*/
            compare_elapsed = chrono.elapsed();

            itoa(minutes, buffer, 10);
            if (buffer[1] == 0)
            {
                buffer[1] = buffer[0];
                buffer[0] = '0';
            }
            Clock_display[0] = buffer[0]; Clock_display[1] = buffer[1];
            Clock_display[2] = ':';
            /*dmd.drawFilledBox(0, 0, PIXELS_X, 8, GRAPHICS_INVERSE);
            dmd.drawString(0, 0, buffer, 2, GRAPHICS_NORMAL);
            dmd.drawChar(14, 0, ':', GRAPHICS_NORMAL);*/
            itoa(seconds, buffer, 10);
            if (buffer[1] == 0)
            {
                buffer[1] = buffer[0];
                buffer[0] = '0';
            }
            Clock_display[3] = buffer[0]; Clock_display[4] = buffer[1];
            Clock_display[5] = 0;
            dmd.drawString(1, 1, Clock_display, 5, GRAPHICS_NORMAL);            
        }
        delay(2);
    }
    
}
// Add the main program code into the continuous loop() function
void loop()
{
    static unsigned long oldTime = 0;

    if (Serial2.available() >= 3)
    {
        byte received_bytes[10];
        uint8_t i = 0;
        while (Serial2.available())
        {
            received_bytes[i++] = Serial2.read();
        }
        Serial.print("IR Intruction: ");
        Serial.print(received_bytes[0], HEX);
        Serial.print(" ");
        Serial.print(received_bytes[1], HEX);
        Serial.print(" ");
        Serial.print(received_bytes[2], HEX);
        Serial.println();

        if (received_bytes[0] != 0xAB || received_bytes[1] != 0xFF)
        {
            Serial.print("IR Wrong instruction...");
        }
        else if (received_bytes[2] == INSTR_RESET)
        {
            Serial.println("IR command to reset..");
            // ESP.restart();
        }

        switch (received_bytes[2])
        {
        case INSTR_RESET:            
            chrono.restart();
            chrono.stop();  
            play_sympol(GRAPHICS_INVERSE);
            stop_sympol(GRAPHICS_NORMAL);
            clear_score();
            countdown_on = false;
            compare_elapsed = 10000;
            break;
        case INSTR_COUNTDOWN_MODE:
            if (!chrono.isRunning())
            {
                compare_elapsed = 10000;
                countdown_on = true;
            }           
            break;
        case INSTR_START:
            if (chrono.isRunning())
            {
                /*chrono.restart();
                stop_sympol(GRAPHICS_INVERSE);
                play_sympol(GRAPHICS_NORMAL);*/
            }
            else {
                chrono.resume();
                stop_sympol(GRAPHICS_INVERSE);
                play_sympol(GRAPHICS_NORMAL);
            }
            buzzer_ring(2);
            break;
        case 0x21:
            inc_score(0);
            break;
        case 0x22:
            inc_score_digit(0, 0);
            break;
        case 0x23:
            inc_score_digit(0, 1);
            break;
        case 0x24:
            inc_score_digit(0, 2);
            break;
        case 0x31:
            clear_score();
            break;
        case 0x32:
            dec_score_digit(1, 0);
            break;
        case 0x33:
            dec_score_digit(1, 1);
            break;
        case 0x34:
            dec_score_digit(1, 2);
            break;
//---------------------------------------------------------------------------
        case 0x81:       //instruction for coundown set Minutes +10
            if (countdown_set_on)
            {
                Timeout_delay = TIMEOUT_SET;
                countdown_buf[0]++;
                if (countdown_buf[0] > '9') { countdown_buf[0] = '0'; }
                dmd.drawString(1, 24, countdown_buf, 5, GRAPHICS_NORMAL);
            }
            else {                
                show_countdown_set();
            }            
            break;
        case 0x91:       //instruction for coundown set Minutes -10
            if (countdown_set_on)
            {
                Timeout_delay = TIMEOUT_SET;
                countdown_buf[0]--;
                if (countdown_buf[0] < '0') { countdown_buf[0] = '9'; }
                dmd.drawString(1, 24, countdown_buf, 5, GRAPHICS_NORMAL);
            }
            else {
                show_countdown_set();
            }            
            break;
        case 0x82:       //instruction for coundown set Minutes +1
            if (countdown_set_on)
            {
                Timeout_delay = TIMEOUT_SET;
                countdown_buf[1]++;
                if (countdown_buf[1] > '9') { countdown_buf[1] = '0'; }
                dmd.drawString(1, 24, countdown_buf, 5, GRAPHICS_NORMAL);
            }
            else {
                show_countdown_set();
            }            
            break;
        case 0x92:       //instruction for coundown set Minutes -1
            if (countdown_set_on)
            {
                Timeout_delay = TIMEOUT_SET;
                countdown_buf[1]--;
                if (countdown_buf[1] < '0') { countdown_buf[1] = '9'; }
                dmd.drawString(1, 24, countdown_buf, 5, GRAPHICS_NORMAL);
            }
            else {
                show_countdown_set();
            }            
            break;
        case 0x83:       //instruction for coundown set Seconds +10
            if (countdown_set_on)
            {
                Timeout_delay = TIMEOUT_SET;
                countdown_buf[3]++;
                if (countdown_buf[3] > '9') { countdown_buf[3] = '0'; }
                dmd.drawString(1, 24, countdown_buf, 5, GRAPHICS_NORMAL);
            }
            else {
                show_countdown_set();
            }
            break;
        case 0x93:       //instruction for coundown set Seconds -10
            if (countdown_set_on)
            {
                Timeout_delay = TIMEOUT_SET;
                countdown_buf[3]--;
                if (countdown_buf[3] < '0') { countdown_buf[3] = '9'; }
                dmd.drawString(1, 24, countdown_buf, 5, GRAPHICS_NORMAL);
            }
            else {
                show_countdown_set();
            }
            break;
        case 0x84:       //instruction for coundown set Seconds +1
            if (countdown_set_on)
            {
                Timeout_delay = TIMEOUT_SET;
                countdown_buf[4]++;
                if (countdown_buf[4] > '9') { countdown_buf[4] = '0'; }
                dmd.drawString(1, 24, countdown_buf, 5, GRAPHICS_NORMAL);
            }
            else {
                show_countdown_set();
            }
            break;
        case 0x94:       //instruction for coundown set Seconds -1
            if (countdown_set_on)
            {
                Timeout_delay = TIMEOUT_SET;
                countdown_buf[4]--;
                if (countdown_buf[4] < '0') { countdown_buf[4] = '9'; }
                dmd.drawString(1, 24, countdown_buf, 5, GRAPHICS_NORMAL);
            }
            else {
                show_countdown_set();
            }
            break;
//----------------------------------------------------------------------------------------------
        case INSTR_STOP:
            chrono.stop();
            play_sympol(GRAPHICS_INVERSE);
            stop_sympol(GRAPHICS_NORMAL);
            buzzer_ring(2);
            break;
        default:
            break;
        }
    }

    if (millis() - oldTime > 1000) {

        if (Timeout_delay)
        {
            Timeout_delay--;
            if (!Timeout_delay)
            {
                //Serial.print("IR Wrong instruction...");
                if (countdown_set_on) {
                    show_scoreboard();
                    countdown_set_on = false;
                    timerAlarmDisable(timer);
                    EEPWriteArray(EEP_COUNTDOWN_SET, countdown_buf);
                    timerAlarmEnable(timer);
                }
                
            }
        }
        //switchi = !switchi;
        //ESPUI.updateControlValue(switchOne, switchi ? "1" : "0");
       // Update_Label_Board(Price_Address);
       // Dots_Display();

        oldTime = millis();

    }

}

void inc_score(uint8_t row) {
    unsigned int A = atoi(score);
    A++;
    char buf[10];
    itoa(A, buf, 10);
    if (buf[1] == 0)
    {
        score[0] = '0';
        score[1] = '0';
        score[2] = buf[0];
    }
    else if (buf[2] == 0)
    {
        score[0] = '0';
        score[1] = buf[0];
        score[2] = buf[1];
    } else {
        score[0] = buf[0];
        score[1] = buf[1];
        score[2] = buf[2];
    }
    dmd.drawString(11, 21, score, 3, GRAPHICS_NORMAL);
    Serial.print("Change score ");
    Serial.print(row); Serial.print(": ");
    Serial.println(score[row]);
}

void inc_score_digit(uint8_t row, uint8_t intex) {
    score[intex]++;
    if (score[intex] > 0x39)
    {
        score[intex] = '0';
    }
    score[3] = 0;
    
    dmd.drawString(11, 21, score, 3, GRAPHICS_NORMAL);
    Serial.print("Change score ");
    Serial.print(row); Serial.print(": ");
    Serial.println(score);
}

void dec_score_digit(uint8_t row, uint8_t intex) {
    score[intex]--;
    if (score[intex] < 0x30)
    {
        score[intex] = '9';
    }
    score[3] = 0;

    dmd.drawString(11, 21, score, 3, GRAPHICS_NORMAL);
    Serial.print("Change score ");
    Serial.print(row); Serial.print(": ");
    Serial.println(score);
}

void clear_score(void) {
    score[0] = '0';
    score[1] = '0';
    score[2] = '0';
    score[3] = 0;

    dmd.drawString(11, 21, score, 3, GRAPHICS_NORMAL);
    Serial.print("Clear score: ");
    Serial.println(score);
}

void play_sympol(byte on) {
    dmd.writePixel(2, 13, on,1);
    dmd.drawLine(2, 14, 3, 14, on);
    dmd.drawLine(2, 15, 4, 15, on);
    dmd.drawLine(2, 16, 5, 16, on);
    dmd.drawLine(2, 17, 4, 17, on);
    dmd.drawLine(2, 18, 3, 18, on);
    dmd.writePixel(2, 19, on, 1);
}

void stop_sympol(byte on) {
    dmd.drawFilledBox(2, 14, 6, 18, on);
}


void buzzer_ring(uint8_t time_last) {
    buzzer_cnt = time_last;
    digitalWrite(BUZZER, HIGH);
}

void show_scoreboard() {
    countdown_time_calculate();
    dmd.selectFont(scoreboard);
    dmd.clearScreen(true);
    dmd.drawString(1, 1, "00:00", 5, GRAPHICS_NORMAL);
    dmd.drawString(11, 21, score, 3, GRAPHICS_NORMAL);
    stop_sympol(GRAPHICS_NORMAL);
}

void show_countdown_set() {
    countdown_set_on = true;
    Timeout_delay = 3; oldTime = millis();
    dmd.selectFont(SystemFont5x7);
    dmd.clearScreen(true);
    dmd.drawString(1, 8, "CDown", 5, GRAPHICS_NORMAL);
    dmd.drawString(1, 16, "Set:", 4, GRAPHICS_NORMAL);
    dmd.drawString(1, 24, countdown_buf, 5, GRAPHICS_NORMAL);
}

void countdown_time_calculate() {
    char buf[6];
    buf[0] = countdown_buf[3]; buf[1] = countdown_buf[4]; buf[2] = 0;    
    countdown_time = atoi(buf); //Calculate the Seconds
    buf[0] = countdown_buf[0]; buf[1] = countdown_buf[1]; buf[2] = 0;
    countdown_time += atoi(buf) * 60; //Calculate the Minutes
}