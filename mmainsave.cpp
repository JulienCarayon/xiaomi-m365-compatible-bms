#include <Arduino.h>
#include <limits.h>
#include <EEPROM.h>
#include <PinChangeInterrupt.h>

#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include "main.h"
#include "bq769x0.h"


M365BMS g_M365BMS;
BMSSettings g_Settings;

bool g_Debug = true;
// I2CAddress = 0x08, crcEnabled = true
bq769x0 g_BMS(bq76940, 0x08, true);
volatile bool g_interruptFlag = false;
unsigned long g_lastActivity = 0;
unsigned long g_lastUpdate = 0;
volatile bool g_uartRxInterrupted = false;
volatile bool g_wakeupFlag = false;

volatile bool g_K1Flag = false;
bool g_dischargeEnabled = true;

unsigned long g_oldMillis = 0;
int g_millisOverflows = 0;

extern volatile unsigned long timer0_millis;
volatile unsigned int g_timer2Overflows = 0;

void alertISR()
{
    g_BMS.setAlertInterruptFlag();
    g_interruptFlag = true;
    g_wakeupFlag = true;
}

#ifdef K1SWITCH
void K1ISR()
{
    g_K1Flag = true;
    g_wakeupFlag = true;
}
#endif

void uartRxISR()
{
    g_uartRxInterrupted = true;
    g_wakeupFlag = true;
}

ISR(TIMER2_OVF_vect)
{
    // only used to keep track of time while sleeping to adjust millis()
    g_timer2Overflows++;
}

void setup()
{
    Serial.begin(76800);
    Serial.println(F("BOOTED!"));
    delay(1000);
#
}

void onNinebotMessage(NinebotMessage &msg)
{
    // Enable TX
    UCSR0B |= (1 << TXEN0);

    if(msg.addr != M365BMS_RADDR)
        return;

    if(msg.mode == 0x01 || msg.mode == 0xF1)
    {
        if(msg.length != 3)
            return;

        uint16_t ofs = (uint16_t)msg.offset * 2; // word aligned
        uint8_t sz = msg.data[0];

        if(sz > sizeof(NinebotMessage::data))
            return;
            
        msg.addr = M365BMS_WADDR;
        msg.length = 2 + sz;

        if(msg.mode == 0x01)
        {
            if((ofs + sz) > sizeof(g_M365BMS))
                return;

            memcpy(&msg.data, &((uint8_t *)&g_M365BMS)[ofs], sz);
        }
        else if(msg.mode == 0xF1)
        {
            if((ofs + sz) > sizeof(g_Settings))
                return;

            memcpy(&msg.data, &((uint8_t *)&g_Settings)[ofs], sz);
        }
    
        ninebotSend(msg);

    }
    else if(msg.mode == 0x03 || msg.mode == 0xF3)
    {
        uint16_t ofs = (uint16_t)msg.offset * 2; // word aligned
        uint8_t sz = msg.length - 2;

        if(msg.mode == 0x03)
        {
            if((ofs + sz) > sizeof(g_M365BMS))
                return;

            memcpy(&((uint8_t *)&g_M365BMS)[ofs], &msg.data, sz);
        }
        else if(msg.mode == 0xF3)
        {
            if((ofs + sz) > sizeof(g_Settings))
                return;

            memcpy(&((uint8_t *)&g_Settings)[ofs], &msg.data, sz);
        }
    }
    else if(msg.mode == 0xFA)
    {
        switch(msg.offset)
        {
            } 
        }
    }


void ninebotSend(NinebotMessage &msg)
{
    msg.checksum = (uint16_t)msg.length + msg.addr + msg.mode + msg.offset;

    Serial.write(msg.header[0]);
    Serial.write(msg.header[1]);
    Serial.write(msg.length);
    Serial.write(msg.addr);
    Serial.write(msg.mode);
    Serial.write(msg.offset);
    Serial.println(msg.header[0]);
    Serial.println(msg.header[1]);
    Serial.println(msg.length);
    Serial.println(msg.addr);
    Serial.println(msg.mode);
    Serial.println(msg.offset);
    for(uint8_t i = 0; i < msg.length - 2; i++)
    {
        Serial.write(msg.data[i]);
        msg.checksum += msg.data[i];
    }

    msg.checksum ^= 0xFFFF;
    Serial.write(msg.checksum & 0xFF);
    Serial.write((msg.checksum >> 8) & 0xFF);
}

void ninebotRecv()
{
    static NinebotMessage msg;
    static uint8_t recvd = 0;
    static unsigned long begin = 0;
    static uint16_t checksum;

    while(Serial.available())
    {
        g_lastActivity = millis();

        if(millis() >= begin + 100)
        { // 100ms timeout
            recvd = 0;
        }

        uint8_t byte = Serial.read();
        recvd++;
        Serial.print("received: ");
        Serial.println(byte,HEX);
        
        switch(recvd)
        {
            case 1:
            {
                if(byte != 0x55)
                { // header1 mismatch
                    recvd = 0;
                    // Serial.println("header1 mismatch ");
                    // Serial.print("header: ");
                    // Serial.println(byte,HEX);
                    break;
                   
                }

                msg.header[0] = byte;
                begin = millis();
            } break;

            case 2:
            {
                if(byte != 0xAA)
                { // header2 mismatch
                    recvd = 0;
                    Serial.print("header2 mismatch ");
                    Serial.println(byte,HEX);
                    break;
                    
                }

                msg.header[1] = byte;
            } break;

            case 3: // length
            {
                if(byte < 2)
                { // too small
                    recvd = 0;
                    Serial.print("too small msg ");
                    Serial.println(byte,HEX);
                    break;
                }

                msg.length = byte;
                checksum = byte;
            } break;

            case 4: // addr
            {
                if(byte != M365BMS_RADDR)
                { // we're not the receiver of this message
                    recvd = 0;
                    Serial.println("wrong adress");
                    break;
                }

                msg.addr = byte;
                checksum += byte;
            } break;

            case 5: // mode
            {
                msg.mode = byte;
                checksum += byte;
                Serial.print("mode: ");
                Serial.println(byte);
            } break;

            case 6: // offset
            {
                msg.offset = byte;
                checksum += byte;
                Serial.print("offset: ");
                Serial.println(byte);
            } break;

            default:
            {
                if(recvd - 7 < msg.length - 2)
                { // data
                    msg.data[recvd - 7] = byte;
                    checksum += byte;
                    Serial.print("data: ");
                    Serial.println(byte);
                }
                else if(recvd - 7 - msg.length + 2 == 0)
                { // checksum LSB
                    msg.checksum = byte;
                    Serial.print("checksum: ");
                    Serial.println(byte);
                }
                else
                { // checksum MSB and transmission finished
                    msg.checksum |= (uint16_t)byte << 8;
                    checksum ^= 0xFFFF;

                    if(checksum != msg.checksum)
                    { // invalid checksum
                        recvd = 0;
                        break;
                        Serial.println("invalid checksum");
                    }

                    onNinebotMessage(msg);
                    recvd = 0;
                }
            } break;
        }
    }
}


void loop()
{
  ninebotRecv();
}

#if BQ769X0_DEBUG
void debug_print()
{
    g_BMS.printRegisters();
    Serial.println(F(""));

    unsigned long uptime = g_millisOverflows * (UINT_MAX / 1000UL);
    uptime += millis() / 1000;
    Serial.print(F("uptime: "));
    Serial.println(uptime);

    Serial.print(F("Battery voltage: "));
    Serial.print(g_BMS.getBatteryVoltage());
    Serial.print(F(" ("));
    Serial.print(g_BMS.getBatteryVoltage(true));
    Serial.println(F(")"));

    Serial.print(F("Battery current: "));
    Serial.print(g_BMS.getBatteryCurrent());
    Serial.print(F(" ("));
    Serial.print(g_BMS.getBatteryCurrent(true));
    Serial.println(F(")"));

    Serial.print(F("SOC: "));
    Serial.println(g_BMS.getSOC());

    Serial.print(F("Temperature: "));
    Serial.print(g_BMS.getTemperatureDegC(1));
    Serial.print(F(" "));
    Serial.println(g_BMS.getTemperatureDegC(2));

    Serial.print(F("Balancing status: "));
    Serial.println(g_BMS.getBalancingStatus());

    Serial.print(F("Cell voltages ("));
    Serial.print(g_BMS.getNumberOfConnectedCells());
    Serial.print(F(" / "));
    int numCells = g_BMS.getNumberOfCells();
    Serial.print(numCells);
    Serial.println(F("):"));
    for(int i = 0; i < numCells; i++) {
        Serial.print(g_BMS.getCellVoltage_(i));
        Serial.print(F(" ("));
        Serial.print(g_BMS.getCellVoltage_(i, true));
        Serial.print(F(")"));
        if(i != numCells - 1)
            Serial.print(F(", "));
    }
    Serial.println(F(""));

    Serial.print(F("Cell V: Min: "));
    Serial.print(g_BMS.getMinCellVoltage());
    Serial.print(F(" | Avg: "));
    Serial.print(g_BMS.getAvgCellVoltage());
    Serial.print(F(" | Max: "));
    Serial.print(g_BMS.getMaxCellVoltage());
    Serial.print(F(" | Delta: "));
    Serial.println(g_BMS.getMaxCellVoltage() - g_BMS.getMinCellVoltage());

    Serial.print(F("maxVoltage: "));
    Serial.println(g_M365BMS.max_voltage);
    Serial.print(F("maxDischargeCurrent: "));
    Serial.println(g_M365BMS.max_discharge_current);
    Serial.print(F("maxChargeCurrent: "));
    Serial.println(g_M365BMS.max_charge_current);

    Serial.print(F("XREADY errors: "));
    Serial.println(g_BMS.errorCounter_[ERROR_XREADY]);
    Serial.print(F("ALERT errors: "));
    Serial.println(g_BMS.errorCounter_[ERROR_ALERT]);
    Serial.print(F("UVP errors: "));
    Serial.println(g_BMS.errorCounter_[ERROR_UVP]);
    Serial.print(F("OVP errors: "));
    Serial.println(g_BMS.errorCounter_[ERROR_OVP]);
    Serial.print(F("SCD errors: "));
    Serial.println(g_BMS.errorCounter_[ERROR_SCD]);
    Serial.print(F("OCD errors: "));
    Serial.println(g_BMS.errorCounter_[ERROR_OCD]);
    Serial.println();
    Serial.print(F("DISCHG TEMP errors: "));
    Serial.println(g_BMS.errorCounter_[ERROR_USER_DISCHG_TEMP]);
    Serial.print(F("CHG TEMP errors: "));
    Serial.println(g_BMS.errorCounter_[ERROR_USER_CHG_TEMP]);
    Serial.print(F("CHG OCD errors: "));
    Serial.println(g_BMS.errorCounter_[ERROR_USER_CHG_OCD]);
}
#endif
