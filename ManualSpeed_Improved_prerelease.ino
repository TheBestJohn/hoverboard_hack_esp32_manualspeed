//*************************//
// PRE RELEASE//
//*************************//

// Hoverboard Manual Speed
// designed for esp32
// based on https://github.com/RoboDurden/Hoverboard-Firmware-Hack-Gen2.x-GD32/tree/main/Arduino%20Examples/TestSpeed
// version
// 0.20240302 //started adding wasd control
// 0.20240225 //added adc potentiometer support

#define _DEBUG // debug output to first hardware serial port
// #define DEBUG_RX    // additional hoverboard-rx debug output
#define REMOTE_UARTBUS

#define SEND_MILLIS 100 // send commands to hoverboard every SEND_MILLIS millisesonds
#include "Globals.h"
#include "util.h"
#include "hoverserial.h"
#include "AdcController.h"
// input method
// serial
// #define input_serial
// #define input_serial_wasd
#define input_adc
// ble
// rc receiver (PPM)
// servo (PWM)
// WiFi?
// #define input_ADC //potentiometer/twisth throthle (ADC)
// MQTT

// array for motors
// how many motors do you have
const size_t motor_count_total = 6;
// how many right motors
const size_t motor_count_right = 3;
// how many left motors
const size_t motor_count_left = 3;

// identify the motors by their slave number
int motors_all[motor_count_total] = {1, 2, 3, 4, 5, 6};
// identify the motors by their slave number
int motors_right[motor_count_right] = {0, 2, 4};
// identify the motors by their slave number
int motors_left[motor_count_left] = {1, 3, 5};

// array for speed
int motor_speed[motor_count_total];
// array for istate
int slave_state[motor_count_total];
// offset
int motoroffset = motors_all[0] - 0;



// Enum for states for better readability
enum HoverboardState
{
    LED_GREEN = 1,
    LED_ORANGE = 2,
    LED_RED = 4,
    LED_UP = 8,
    LED_DOWN = 16,
    BATTERY_3_LED = 32,
    DISABLE = 64,
    SHUT_OFF = 128
};

//
int slaveidin;
int iSpeed;
int ispeedin;
int istatein;
int count = 0;
String command;
// serial wasd
// speed steps
int speedstep = 100;
// speed difference for turn
int speedstepturn = 50;

#define oSerialHover Serial2 // ESP32

SerialHover2Server oHoverFeedback;

void setup()
{
#ifdef _DEBUG
    Serial.begin(115200);
    Serial.println("Hello Hoverbaord V2.x :-)");
#endif

#ifdef input_serial
    HoverSetupEsp32(oSerialHover, 19200, 16, 17);
#else
#endif

#ifdef input_serial_wasd
    HoverSetupEsp32(oSerialHover, 19200, 16, 17);
#else
#endif

#ifdef input_adc
    HoverSetupEsp32(oSerialHover, 19200, 16, 17);
#endif
}

unsigned long iLast = 0;
unsigned long iNext = 0;
unsigned long iTimeNextState = 3000;
uint8_t wState = 1; // 1=ledGreen, 2=ledOrange, 4=ledRed, 8=ledUp, 16=ledDown, 32=Battery3Led, 64=Disable, 128=ShutOff
// id for messages being sent
uint8_t iSendId = 0; // only ofr UartBus

void parseSerialCommand(const String &command);
void handleHoverCommand(const String &hoverCommand);
void stopMotors();
void setMotorSpeedAndState(int speed, int state);
void setMotorSpeedAndStateForGroup(int *motorGroup, size_t motorGroupSize, int speed, int state);
void debugPrintMotorStates();
bool parseSpeedAndState();
void setAllMotorsSpeedAndState();

void parseSerialCommand(const String &command)
{
    if (command.startsWith("hover|"))
    {
        handleHoverCommand(command.substring(6));
    }
    else if (command == "stop")
    {
        stopMotors();
    }
    else
    {
        Serial.println("Command not recognized");
    }
}

void debugPrintMotorStates()
{
    for (int i = 0; i < motor_count_total; ++i)
    {
        Serial.print("Motor ");
        Serial.print(motors_all[i]);
        Serial.print(" Speed is set to ");
        Serial.print(motor_speed[i]);
        Serial.print(" Slave state is set to ");
        Serial.println(slave_state[i]);
    }
}

bool parseSpeedAndState(const String &data, int &speed, int &state) {
    int numParsed = sscanf(data.c_str(), "%d|%d", &speed, &state);
    if (numParsed != 2) {
        Serial.println("Error: Command does not contain valid speed and state.");
        return false;
    }
    return true;
}

void setAllMotorsSpeedAndState(int speed, int state) {
    for (int i = 0; i < motor_count_total; ++i) {
        motor_speed[i] = speed;
        slave_state[i] = state;
    }
}

void stopMotors()
{
    setMotorSpeedAndState(0, LED_GREEN); // Assuming LED_GREEN is the default stop state
}

void setMotorSpeedAndState(int speed, int state)
{
    for (int i = 0; i < motor_count_total; ++i)
    {
        motor_speed[i] = speed;
        slave_state[i] = state;
    }
}

void setMotorSpeedAndStateForGroup(int *motorGroup, size_t motorGroupSize, int speed, int state)
{
    for (size_t i = 0; i < motorGroupSize; ++i)
    {
        int motorIndex = motorGroup[i] - motoroffset;
        motor_speed[motorIndex] = speed;
        slave_state[motorIndex] = state;
    }
}

void handleHoverCommand(const String &hoverCommand)
{
    int speed, state;
    if (hoverCommand.startsWith("all|")) {
        if (parseSpeedAndState(hoverCommand.substring(4), speed, state)) {
            setAllMotorsSpeedAndState(speed, state);
        }
    } else if (hoverCommand.startsWith("right|")) {
        if (parseSpeedAndState(hoverCommand.substring(6), speed, state)) {
            setMotorSpeedAndStateForGroup(motors_right, motor_count_right, speed, state);
        }
    } else if (hoverCommand.startsWith("left|")) {
        if (parseSpeedAndState(hoverCommand.substring(5), speed, state)) {
            setMotorSpeedAndStateForGroup(motors_left, motor_count_left, speed, state);
        }
    } else {
        Serial.println("Command not recognized: " + hoverCommand);
    }
}
void loop()
{
        unsigned long iNow = millis();
    #ifdef input_serial
        // read serial input

        if (Serial.available())
        {
            String command = Serial.readStringUntil('\n');
            parseSerialCommand(command);

            // check if input starts with hover
            // if the hover command is present parse the input data
        }
        else // single number motor specified
        {
            int numParsed = sscanf(command.c_str(), "%d|%d|%d", &slaveidin, &ispeedin, &istatein);

            if (numParsed == 3)
            {
                // set motor speed
                motor_speed[slaveidin - motoroffset] = ispeedin;
                // set motor/mcu state
                slave_state[slaveidin - motoroffset] = istatein;
            }
            else
            {
                // Handle parsing error
                Serial.println("The command doesn't meet the criteria");
            }
        }
    #endif

        // serial_WASD
    #ifdef input_serial_wasd
        // read serial input
        if (Serial.available())
        {                                                  // if there is data comming
            String command = Serial.readStringUntil('\n'); // read string until newline character
                                                        // check if input starts with hover
                                                        // if the hover command is present parse the input data

            if (command == "w") // w
            {
                ispeedin = ispeedin + speedstep while (count < motor_count_total)
                {
                    // set motor speed
                    motor_speed[count] = ispeedin;
                    // set motor/mcu state
                    slave_state[count] = istatein;
                    // increase count
                    count++;
                }
            }
            else
            {
            }
        }
        else if (command == "s") // w
        {
            ispeedin = ispeedin - speedstep while (count < motor_count_total)
            {
                // set motor speed
                motor_speed[count] = ispeedin;
                // set motor/mcu state
                slave_state[count] = istatein;
                // increase count
                count++;
            }
        }
        else
        {
        }
    }
    else if (command == "d") // d
    {
        ispeedin = ispeedin - speedstepturn while (count < motor_count_total)
        {
            // set the data
            count = 0;
            while (count < motor_count_right)
            {
                // set motor speed
                motor_speed[motors_right[count] - motoroffset] = ispeedin;
                // set motor/mcu state
                slave_state[motors_right[count] - motoroffset] = istatein;
                // increase count
                count++;
            }
        }
        else
        {
        }
    }

    else if (command == "a") // w
    {
        ispeedin = ispeedin - speedstepturn
                                // set the data
                                count = 0;
        while (count < motor_count_left)
        {
            // set motor speed
            motor_speed[motors_left[count] - motoroffset] = ispeedin;
            // set motor/mcu state
            slave_state[motors_left[count] - motoroffset] = istatein;
            // increase count
            count++;
        }
    }
    else if (command == "e") // e
        sync motors to speed of right first motoe
    {
        ispeedin = motor_speed[motors_right[0] - motoroffset]
            // set the data
            count = 0;
        while (count < motor_count_total)
        {
            // set motor speed
            motor_speed[count] = ispeedin;
            // set motor/mcu state
            slave_state[count] = istatein;
            // increase count
            count++;
        }
    }
    else if (command == "q") // q
        sync motors to speed of left first motoe
    {
        ispeedin = motor_speed[motors_left[0] - motoroffset]
            // set the data
            count = 0;
        while (count < motor_count_total)
        {
            // set motor speed
            motor_speed[count] = ispeedin;
            // set motor/mcu state
            slave_state[count] = istatein;
            // increase count
            count++;
        }
    }
    else
    {
    }
    }

    #endif

    #ifdef input_adc
    // read the input on analog pin:
    readAndProcessAdc();
    #endif

    int iSteer = 0; // repeats from +100 to -100 to +100 :-)
    // int iSteer = 0;
    // int iSpeed = 500;
    // int iSpeed = 200;
    // iSpeed = iSteer = 0;

    if (iNow > iTimeNextState)
    {
        iTimeNextState = iNow + 3000;
        wState = wState << 1;
        if (wState == 64)
            wState = 1; // remove this line to test Shutoff = 128
    }

    if (iNow > iNext)
    {
        // DEBUGLN("time",iNow)

    #ifdef REMOTE_UARTBUS
        count = 0;
        while (count < motor_count_total)
        {
            HoverSend(oSerialHover, motors_all[count], motor_speed[count], slave_state[count]);
            //           #ifdef _DEBUG
            //                Serial.print("Sent Motor ");
            //                Serial.print(motors_all[count]);
            //                Serial.print(" Speed ");
            //                Serial.print(motor_speed[count]);
            //                Serial.print (" and Slave State ");
            //                Serial.println(slave_state[count]);
            //            #endif
            count++;
            boolean bReceived;
            while (bReceived = Receive(oSerialHover, oHoverFeedback))
            {
                DEBUGT("millis", iNow - iLast);
                DEBUGT("iSpeed", iSpeed);
                // DEBUGT("iSteer",iSteer);
                HoverLog(oHoverFeedback);
                iLast = iNow;
            }
        }

        iNext = iNow + SEND_MILLIS / 2;
    #else
            // if (bReceived)  // Reply only when you receive data
            HoverSend(oSerialHover, iSteer, iSpeed, wState, wState);

            iNext = iNow + SEND_MILLIS;
    #endif
    }
}
