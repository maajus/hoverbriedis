
// INFO:
// • This sketch uses the the Serial Software interface to communicate and send commands to the hoverboard
// • The built-in (HW) Serial interface is used for debugging and visualization. In case the debugging is not needed,
//   it is recommended to use the built-in Serial interface for full speed perfomace.
// • The data packaging includes a Start Frame, checksum, and re-syncronization capability for reliable communication
//
// The code starts with zero speed and moves towards +
//
// CONFIGURATION on the hoverboard side in config.h:
// • Option 1: Serial on Right Sensor cable (short wired cable) - recommended, since the USART3 pins are 5V tolerant.
//   #define CONTROL_SERIAL_USART3
//   #define FEEDBACK_SERIAL_USART3
//   // #define DEBUG_SERIAL_USART3
// • Option 2: Serial on Left Sensor cable (long wired cable) - use only with 3.3V devices! The USART2 pins are not 5V tolerant!
//   #define CONTROL_SERIAL_USART2
//   #define FEEDBACK_SERIAL_USART2
//   // #define DEBUG_SERIAL_USART2
// *******************************************************************

#include <Arduino.h>
#include <SoftwareSerial.h>

// ########################## DEFINES ##########################
//#define GRADUAL_ACC
#define HOVER_SERIAL_BAUD   115200      // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define SERIAL_BAUD         115200      // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#define START_FRAME         0xABCD     	// [-] Start frme definition for reliable serial communication
#define LOOP_PERIOD         100         // [ms] Sending time interval
#define RX_PERIOD           500         // [ms] Sending time interval
#define SPEED_MAX_TEST      300         // [-] Maximum speed for testing
#define ACC_STEP            50          // [-] Acc step
#define ACC_STEP_DELAY      100         // [-] Acc step delay
// #define DEBUG_RX                       // [-] Debug received data. Prints all bytes to serial (comment-out to disable)
#define SPEED_MAX_INPUT 1000
#define SPEED_MIN_INPUT -1000

//Pins
#define SERIAL_RX_PIN 2
#define SERIAL_TX_PIN 3
#define PROXIMITY_SENSOR1_PIN 4
#define PROXIMITY_SENSOR2_PIN 5
#define START_SIGNAL_PIN 6
#define PAUSE_SIGNAL_PIN 7
#define SPEEED_ADC_PIN 8
#define BRAKE_PIN 9



typedef struct{
   uint16_t start;
   int16_t  steer;
   int16_t  speed;
   uint16_t checksum;
} SerialCommand;

typedef struct{
   uint16_t start;
   int16_t  cmd1;
   int16_t  cmd2;
   int16_t  speedR_meas;
   int16_t  speedL_meas;
   int16_t  batVoltage;
   int16_t  boardTemp;
   uint16_t cmdLed;
   uint16_t checksum;
} SerialFeedback;


SoftwareSerial HoverSerial(SERIAL_RX_PIN,SERIAL_TX_PIN);        // RX, TX

// Global variables
uint8_t idx = 0;                        // Index for new data pointer
uint16_t bufStartFrame;                 // Buffer Start Frame
byte *p;                                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;
SerialCommand Command;
SerialFeedback Feedback;
SerialFeedback NewFeedback;
int8_t direction = 1;
int speed = 0;
uint8_t isMoving = false;
unsigned long prevTime = 0;

// ########################## SEND ##########################
void Send(int16_t uSteer, int16_t uSpeed)
{
  // Create command
  Command.start    = (uint16_t)START_FRAME;
  Command.steer    = (int16_t)uSteer;
  Command.speed    = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);

  // Write to Serial
  HoverSerial.write((uint8_t *) &Command, sizeof(Command));
}

// ########################## RECEIVE ##########################
void Receive()
{
    // Check for new data availability in the Serial buffer
    if (HoverSerial.available()) {
        incomingByte 	  = HoverSerial.read();                                   // Read the incoming byte
        bufStartFrame	= ((uint16_t)(incomingByte) << 8) | incomingBytePrev;       // Construct the start frame
    }
    else {
        return;
    }

  // If DEBUG_RX is defined print all incoming bytes
#ifdef DEBUG_RX
        Serial.print(incomingByte);
        return;
#endif

    // Copy received data
    if (bufStartFrame == START_FRAME) {	                    // Initialize if new data is detected
        p       = (byte *)&NewFeedback;
        *p++    = incomingBytePrev;
        *p++    = incomingByte;
        idx     = 2;
    } else if (idx >= 2 && idx < sizeof(SerialFeedback)) {  // Save the new received data
        *p++    = incomingByte;
        idx++;
    }

    // Check if we reached the end of the package
    if (idx == sizeof(SerialFeedback)) {
        uint16_t checksum;
        checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas
                            ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);

        // Check validity of the new data
        if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum) {
            // Copy the new data
            memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));

            // Print data to built-in Serial
            Serial.print("1: ");   Serial.print(Feedback.cmd1);
            Serial.print(" 2: ");  Serial.print(Feedback.cmd2);
            Serial.print(" 3: ");  Serial.print(Feedback.speedR_meas);
            Serial.print(" 4: ");  Serial.print(Feedback.speedL_meas);
            Serial.print(" 5: ");  Serial.print(Feedback.batVoltage);
            Serial.print(" 6: ");  Serial.print(Feedback.boardTemp);
            Serial.print(" 7: ");  Serial.println(Feedback.cmdLed);
        } else {
          Serial.println("Non-valid data skipped");
        }
        idx = 0;    // Reset the index (it prevents to enter in this if condition in the next cycle)
    }

    // Update previous states
    incomingBytePrev = incomingByte;
}

void accelerate(int targetSpeed, int direction){

    isMoving = true;
#ifdef GRADUAL_ACC
  int i;
  for(int i = ACC_STEP; i < targetSpeed; i+=ACC_STEP){
    Send(0,i*direction);
    delay(ACC_STEP_DELAY);
  }
#else
  Send(0,targetSpeed*direction);
#endif

}

void stop(){
  Send(0,0);
  isMoving = false;
}

// ########################## SETUP ##########################
void setup()
{
  Serial.begin(SERIAL_BAUD);
  Serial.println("Hoverboard Serial v1.0");

  HoverSerial.begin(HOVER_SERIAL_BAUD);


  //init pins
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(BRAKE_PIN, OUTPUT);

  pinMode(PROXIMITY_SENSOR1_PIN, INPUT_PULLUP);
  pinMode(PROXIMITY_SENSOR2_PIN, INPUT_PULLUP);
  pinMode(START_SIGNAL_PIN, INPUT_PULLUP);
  pinMode(PAUSE_SIGNAL_PIN, INPUT_PULLUP);
  pinMode(SPEEED_ADC_PIN, INPUT_ANALOG);

}

// ########################## LOOP ##########################

void loop(void)
{
  unsigned long timeNow = millis();

  // Check for new received data
  if(timeNow - prevTime >= RX_PERIOD){
    Receive();
    prevTime = timeNow;
  }

  // Check for start signal
  if(!digitalRead(START_SIGNAL_PIN) && !isMoving){
    speed = (analogRead(SPEEED_ADC_PIN)*SPEED_MAX_INPUT+1)/1024;
    if(!digitalRead(PROXIMITY_SENSOR1_PIN)){
      direction = 1;
    }
    if(!digitalRead(PROXIMITY_SENSOR2_PIN)){
      direction = -1;
    }
    accelerate(speed, direction);
  }

  //Stop if pause signal is detected
  if(!digitalRead(PAUSE_SIGNAL_PIN) && isMoving){
    stop();
  }

  //Stop if poximity1 signal is detected
  if(!digitalRead(PROXIMITY_SENSOR1_PIN) && isMoving){
    stop();
  }

  //Stop if poximity2 signal is detected
  if(!digitalRead(PROXIMITY_SENSOR2_PIN) && isMoving){
    stop();
  }

  // Blink the LED
  digitalWrite(LED_BUILTIN, (timeNow%2000)<1000);
  delay(LOOP_PERIOD);
}

// ########################## END ##########################
