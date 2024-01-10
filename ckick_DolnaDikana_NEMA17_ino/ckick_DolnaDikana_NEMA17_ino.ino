//                                 Copyright (c) 2018 By Zumwalt Properties, LLC.                           //
//                                                                                                          //
// Includes.

#include                              <time.h>                              // for time
#include                              <WiFi.h>                              // for wifi
#include                              <WiFiUdp.h>                           // for udp

// Constants.

enum          NtpPacketMode           {REQUEST, SENT, RECEIVED};          // possible values for an NtpPacketMode variable.
const int dirPin_minute = 2;
const int stepPin_minute = 3;
const int dirPin_hour = 14;
const int stepPin_hour = 12;
const int stepsPerRevolution = 12000;
#define       DEBUG                   true                                // true for debug, false if not
#define       HOME_SWITCH_CALIBRATE   false                               // set to true to the run home switch calibration, false to run the clock
#define       LED_PIN                 13                                  // on board led drive pin
#define       MILLISECONDS_STEP_HOLD  2                                   // time in milliseconds between motor steps
#define       MOTOR_PHASE_A           32                                  // motor phase a pin
#define       MOTOR_PHASE_B           14                                  // motor phase b pin
#define       MOTOR_PHASE_C           33                                  // motor phase c pin
#define       MOTOR_PHASE_D           15                                  // motor phase d pin
#define       NTP_PACKET_LENGTH       48                                  // size, in bytes, of an ntp packet
#define       PACKET_DELAY_RESET      2                                   // maximum passes before resending an ntp packet request
#if           DEBUG
#define       SERIAL_PRINTF           Serial.printf                       // print debug information via serial
#else
#define       SERIAL_PRINTF           //                                  // do not print debug information
#endif
#define       STEPS_PER_HOUR          12000                               // steps for one full revolution
#define       SWITCH_PIN_minute       27                                  // 12:00 switch input-minute
#define       SWITCH_PIN_hour         26                                  // 12:00 switch input-hour
#define       TIME_ZONE               (+2)                                // offset from utc to your timezone
#define       UDP_PORT                4000                                // ntp time server udp port

// Global Variables.

int             nDelay =                        5000;                     // main loop default delay
NtpPacketMode   nNtpPacketMode =                REQUEST;                  // ntp packet request mode (see NtpPacketMode enum above)
bool            bNtpTimeRequestTrigger =        true;                     // ntp packet request trigger
byte            chNtpPacket[NTP_PACKET_LENGTH];                           // ntp packet
int             nPacketDelay =                  PACKET_DELAY_RESET;       // delay counter for resending ntp packet request
char            chPassword[] =                  "qawsedrf";          // your wifi network password
struct tm *     tmPointer;                                                // tm structure pointer
char            chSSID[] =                      "wiogw";              // your wifi network SSID
long            nTimeInMinutes =                0;                        // time in minutes
long            nTimeInMinutesIndicated =       0;                        // time in minutes indicated by clock
long            nTimeInHours=                   0;                        // time in hours
long            nTimeInHoursIndicated =         0;                        // time in hours indicated by clock
struct timeval  tvTimeValue;                                              // time value structure
WiFiUDP         Udp;                                                      // udp structure

// Setup.

void setup() 
{
  // Serial.
  
  Serial.begin(115200);
  while(!Serial){};

  SERIAL_PRINTF("Setup(): Starting setup.\n");

  // I/O.

    // Set motor drive pins as outputs.
    pinMode(stepPin_minute, OUTPUT);
    pinMode(dirPin_minute, OUTPUT);
    pinMode(stepPin_hour, OUTPUT);
    pinMode(dirPin_hour, OUTPUT);
    // pinMode(MOTOR_PHASE_A, OUTPUT);
    // pinMode(MOTOR_PHASE_B, OUTPUT);
    // pinMode(MOTOR_PHASE_C, OUTPUT);
    // pinMode(MOTOR_PHASE_D, OUTPUT);

    // Set on board led pin as output.

    pinMode(LED_PIN, OUTPUT);

    // Set 12:00 o'clock position switch pin as input.
  
    pinMode(SWITCH_PIN_minute, INPUT_PULLUP);
    pinMode(SWITCH_PIN_hour, INPUT_PULLUP);

#if HOME_SWITCH_CALIBRATE

    // Home switch position calibration.

      // Rotate counter clockwise 90 degrees.
      
      for(int nCount = 0; nCount < STEPS_PER_HOUR / 4; nCount ++)
      {
        Step_minute(-1);
      }

      // Rotate clockwise until the 12:00 switch activates.

        // On board led on.
        
        digitalWrite(LED_PIN, HIGH);

        // Rotate clockwise until the 12:00 switch activates.
        
        while(digitalRead(SWITCH_PIN_minute))
        {
          Step(1);
        }

        // On board led off.
        
        digitalWrite(LED_PIN, LOW);

        // Motor off.
        
        MotorOff();

        // Loop forever (reset the esp32 to restart the test).
        
        while(1){};
  
#endif

  // Wifi.

  SERIAL_PRINTF("Setup(): Initializing wifi connection.\n");
  WiFi.begin(chSSID, chPassword);

  // Begin udp service.
  
  Udp.begin(UDP_PORT);

  // Home the clock.

  Home();

  // Set esp32 rtc to 00:00:00.
  
  memset(& tvTimeValue, 0, sizeof(tvTimeValue));
  settimeofday(& tvTimeValue, NULL);

  // Check wifi status.

  SERIAL_PRINTF("Setup(): Confirming wifi connection.");
  
  while(WiFi.status() != WL_CONNECTED)
  {
      Serial.print(".");
      delay(500);
  }

  SERIAL_PRINTF("\nSetup(): WiFi connection confirmed to %s.\n", chSSID);

  // End of setup.

  SERIAL_PRINTF("Setup(): Setup complete, entering Loop().\n");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Loop.
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() 
{
  // Reset delay.
  nDelay = 5000;
  
  // Update nTimeInMinutes from the ESP32 rtc.
  
  gettimeofday(& tvTimeValue, NULL);
  
  tmPointer = localtime(& tvTimeValue.tv_sec);
  nTimeInMinutes = ((tmPointer->tm_hour % 12) * 60) + tmPointer->tm_min;
  
  SERIAL_PRINTF("Loop(): esp32 rtc: Time = %02d:%02d, nTimeInHours = %d, nTimeInMinutes = %d.\n", tmPointer->tm_hour % 12, tmPointer->tm_min, nTimeInHours, nTimeInMinutes);
  
 // Update clock.
  
  Update();
  
  // At zero minutes (the top of every hour), trigger an ntp time request in not already triggered.
  
  // if(tmPointer->tm_min == 0)
  // {
  //   // Zero minutes, check ntp time request trigger.
    
  //   if(bNtpTimeRequestTrigger)
  //   {
  //     // Time request not yet triggered, trigger ntp time request.
      
  //     SERIAL_PRINTF("Loop(): top of hour, triggering ntp time request.\n");
  //     nNtpPacketMode = REQUEST;
  //     bNtpTimeRequestTrigger = false;

  //     // At zero minutes, check for zero hours.
      
  //     if((tmPointer->tm_hour % 12) == 0)
  //     {
  //       // At zero minutes and zero hours, check home switch.
    
  //       if(digitalRead(SWITCH_PIN_minute))
  //       {
  //         // Home swich not activated at zero minutes, zero hours, home the clock.
          
  //         SERIAL_PRINTF("Loop(): **** CLOCK SLOW, GEAR TRAIN TOO TIGHT OR INCREASE MILLISECONDS_STEP_HOLD. ****\n");
  //         Home();
  //       }
  //     }
  //   }
  // }
  // else
  // {
  //   // Non-zero minutes, reset the ntp time request trigger.
    
  //   bNtpTimeRequestTrigger = true;
  // }
  
  // Check for ntp time request.

  if(nNtpPacketMode == REQUEST)
  {
    // Request is needed, initialize ntp packet.
  
      // Zero out chNtpPacket.
    
      memset(chNtpPacket, 0, NTP_PACKET_LENGTH);

      // Set the ll (leap indicator), vvv (version number) and mmm (mode) bits.
      //  
      //  These bits are contained in the first byte of chNtpPacker and are in
      // the following msb to lsb format:  llvvvmmm
      //
      // where:
      //
      //    ll  (leap indicator) = 0
      //    vvv (version number) = 3
      //    mmm (mode)           = 3
      
      chNtpPacket[0]  = 0b00011011;

    // Send the ntp packet (see https://tf.nist.gov/tf-cgi/servers.cgi for other ip addresses).
    
    //IPAddress ipNtpServer(129, 6, 15, 29);
    IPAddress ipNtpServer(87, 97, 157, 120);
    Udp.beginPacket(ipNtpServer, 123);
    Udp.write(chNtpPacket, NTP_PACKET_LENGTH);
    Udp.endPacket();

    // Packet has been sent.
    
    nNtpPacketMode = SENT;
    nPacketDelay = PACKET_DELAY_RESET;
    SERIAL_PRINTF("Loop(): ntp packet sent.\n");
  }

  // Check for ntp time request sent.
  
  if(nNtpPacketMode == SENT)
  {
    // Ntp time request sent, check for ntp time server response.
    
    if(Udp.parsePacket())
    {
      // Server responded, read the packet.
  
      Udp.read(chNtpPacket, NTP_PACKET_LENGTH);
      
      // Obtain the time from the packet, convert to Unix time, and adjust for the time zone.
      
      tvTimeValue = {0, 0};
      
      tvTimeValue.tv_sec = ((unsigned long)chNtpPacket[40] << 24) +       // bits 24 through 31 of ntp time
                           ((unsigned long)chNtpPacket[41] << 16) +       // bits 16 through 23 of ntp time
                           ((unsigned long)chNtpPacket[42] <<  8) +       // bits  8 through 15 of ntp time
                           ((unsigned long)chNtpPacket[43]) -             // bits  0 through  7 of ntp time
                           (((70UL * 365UL) + 17UL) * 86400UL) +          // ntp to unix conversion
                           (TIME_ZONE * 3600UL) +                         // time zone adjustment
                           (5);                                           // transport delay fudge factor
      
      // Set the esp32 clock to ntp time.
      
      tmPointer = localtime(& tvTimeValue.tv_sec);
      settimeofday(& tvTimeValue, NULL);
  
      // Update nNtpPacketMode.
  
      nNtpPacketMode = RECEIVED;
      
      // Debug.
      
      SERIAL_PRINTF("Loop(): ntp time received is %02d:%02d.\n", tmPointer->tm_hour % 12, tmPointer->tm_min); 
    }
    else
    {
      // Ntp packet has been sent but no server response yet, down count nPacketDelay.
      
      nPacketDelay = nPacketDelay - 1;
      
      // Check nPacketDelay.
      
      if(nPacketDelay <= 0)
      {
        // Time out.
        
        SERIAL_PRINTF("Loop(): ntp time packet timeout.\n");
        
        // nPacketDelay <= 0, reset it.
        
        nPacketDelay = PACKET_DELAY_RESET;
      
        // Request another packet send.
        
        nNtpPacketMode = REQUEST;
      }
    }
  }

  // Delay nDelay seconds.

  delay(nDelay);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Home
// Home the clock to 12:00.
// Entry   : nothing
// Returns : nothing
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void  Home()
{
  // Home clock.
  
  SERIAL_PRINTF("Setup(): Homing clock minute to 12:00.\n");

  // Turn on the on board led.
  
  digitalWrite(LED_PIN, HIGH);

  // Move off of the 12:00 switch.
  
  while(!digitalRead(SWITCH_PIN_minute))
  {
    Step_minute(1);
  }
  
  digitalWrite(LED_PIN, LOW);

  // Move onto the 12:00 switch.
  
  while(digitalRead(SWITCH_PIN_minute))
  {
    Step_minute(1);
    
  }
  
  // Turn off the on board led.
  
  digitalWrite(LED_PIN, LOW);

  SERIAL_PRINTF("Setup(): Homing clock hour to 12:00.\n");

  // Turn on the on board led.
  
  digitalWrite(LED_PIN, HIGH);

  // Move off of the 12:00 switch.
  
  while(!digitalRead(SWITCH_PIN_hour))
  {
    Step_hour(-1);
  }
  
  digitalWrite(LED_PIN, LOW);

  // Move onto the 12:00 switch.
  
  while(digitalRead(SWITCH_PIN_hour))
  {
    Step_hour(-1);
    
  }
  
  // Turn off the on board led.
  
  digitalWrite(LED_PIN, LOW);

  // Clock is now at 12:00.
  
  nTimeInMinutes = nTimeInMinutesIndicated = 0;
  nTimeInHours = nTimeInHoursIndicated = 0;

  SERIAL_PRINTF("Setup(): Clock homed at 12:00.\n");
  
  // Remove motor power.

  MotorOff();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// MotorOff
// Turn off motor drive.
// Entry   : nothing
// Returns : nothing
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void  MotorOff()
{
  digitalWrite(MOTOR_PHASE_A, LOW);
  digitalWrite(MOTOR_PHASE_B, LOW);
  digitalWrite(MOTOR_PHASE_C, LOW);
  digitalWrite(MOTOR_PHASE_D, LOW);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Step
// Step the stepper motor.
// Entry   : direction (1 = clockwise, -1 = counter clockwise)
// Returns : nothing
//
// Notes   : 1) for this stepper motor, 1 step is 1 / STEPS_PER_HOUR degrees.
//
//           2) forward clock motion is performed in 8 steps:
//
//              a) Phase d
//              b) Phase d and c
//              c) phase c
//              d) phase c and b
//              e) phase b
//              f) phase b and a
//              g) phase a
//              h) phase a and d
//
//            3) Reverse clock motion is performed in the reverse order of 2).
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void  Step(int nDirection)
{
  // Local variables.
  
  static  int nPhase = 0;

  // Update phase.
  
  nPhase = ((nDirection < 0) ? (nPhase + 1) : (nPhase - 1)) & 7;

  // Step this phase.
  
  switch(nPhase)
  
  delay(MILLISECONDS_STEP_HOLD);
  Step_minute(nDirection);
  delay(MILLISECONDS_STEP_HOLD);
  Step_hour(nDirection);
}
void Step_minute(int nDirection)
{
  {
   if(nDirection == 1)
   {
    digitalWrite(dirPin_minute, LOW);
   }
    else
   {
    digitalWrite(dirPin_minute, HIGH);
   }
    digitalWrite(stepPin_minute, HIGH);
    delayMicroseconds(800);
    digitalWrite(stepPin_minute, LOW);
    delayMicroseconds(800);
  }
}

void Step_hour(int nDirection)
{
  {
   if(nDirection == 1)
   {
    digitalWrite(dirPin_hour, LOW);
   }
    else
   {
    digitalWrite(dirPin_hour, HIGH);
   }
    digitalWrite(stepPin_hour, HIGH);
    delayMicroseconds(800);
    digitalWrite(stepPin_hour, LOW);
    delayMicroseconds(800);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// Update
// Clock update.
// Entry   : nothing
// Returns : -1 if update occurred, 0 if not.
//
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

int   Update()
{
  // Local variables.

  long              nClockwiseMinutes = 0;
  long              nCounterClockwiseMinutes = 0;
  long              nClockwiseHours = 0;
  long              nCounterClockwiseHours = 0;
  long              nStepCount = 0;
  long              nStepCount2 = 0;
  long              nStepCountHour = 0;
  long              nStepDirection = 0;
  long              nStepDirectionHour = 0;
  long              nStepPosition = 1;
  long              nStepPositionN1 = 0;
  int               br = 0;
  int               addHourSteps = 0;
  int               stepsHour = 0;
    
  // Calculate clockwise and counterclockwise time in minutes required to drive indicated time to actual time.
  
  if(nTimeInMinutes > nTimeInMinutesIndicated)
  {
    nClockwiseMinutes = nTimeInMinutes - nTimeInMinutesIndicated;
    nCounterClockwiseMinutes = (nTimeInMinutesIndicated + (12 * 60)) - nTimeInMinutes;
  }
  else if(nTimeInMinutes < nTimeInMinutesIndicated)
  {
    nClockwiseMinutes = (nTimeInMinutes + (12 * 60)) - nTimeInMinutesIndicated;
    nCounterClockwiseMinutes = nTimeInMinutesIndicated - nTimeInMinutes;
  }
  
  // Check if update is needed.
  
  if((nClockwiseMinutes) || (nCounterClockwiseMinutes))
  {
    // Update is needed, determine shortest direction.
    
    if(nClockwiseMinutes < nCounterClockwiseMinutes)
    {
      // Clockwise movement is shorter.
      
      nStepDirection = 1; 
      nStepDirectionHour = 0;
      digitalWrite(dirPin_minute, LOW);
    }
    else
    {
      // Counterclockwise movement is shorter.
      
      nStepDirection = 1;
      nStepDirectionHour = 0;
      digitalWrite(dirPin_minute, HIGH);
    }

    // Drive indicated time to time.

    while(nTimeInMinutes != nTimeInMinutesIndicated)
    {
      // Calculate n-1 step position.
      
      nStepPositionN1 = ((nTimeInMinutesIndicated % 60) * STEPS_PER_HOUR) / 60;
            
      // Update minutes.
      
      nTimeInMinutesIndicated = (nTimeInMinutesIndicated + nStepDirection) % (12 * 60);
      nTimeInMinutesIndicated = (nTimeInMinutesIndicated < 0) ? (12 * 60) + nTimeInMinutesIndicated : nTimeInMinutesIndicated;
            
      // Calculate current step position.

      nStepPosition = ((nTimeInMinutesIndicated % 60) * STEPS_PER_HOUR) / 60;
      
      // Calculate step count.
      
      nStepCount = ((nStepDirection > 0) ? nStepPosition - nStepPositionN1 : nStepPositionN1 - nStepPosition);
      nStepCount = (nStepCount < 0) ? STEPS_PER_HOUR + nStepCount : nStepCount;

            
                           
      // Step the required steps.

      nStepCount2 = nStepCount;
      
      while(nStepCount)
      {
        Step_minute(nStepDirection);
        nStepCount = nStepCount - 1;
      }

      br = 0;
      stepsHour = 0;
      while(nStepCount2)
      {
        if (br == 0){
          addHourSteps = (nStepCount2 + addHourSteps) % 12;
          br = 1;
        }
        if((nStepCount2 + addHourSteps) % 12 == 0)
        {
          stepsHour++;
          Step_hour(nStepDirectionHour);
        }
        nStepCount2 = nStepCount2 - 1;
      }

      SERIAL_PRINTF("Update(): nStepPosition = %d, nStepPositionN1 = %d, nTimeInMinutesIndicated = %d, nStepCount = %d, stepsHour = %d\n", 
                     nStepPosition, nStepPositionN1, nTimeInMinutesIndicated, nStepCount, stepsHour);
    }

  }
  else
  {
    // No update performed.
    
    return 0;
  }

  // Update was performed, remove motor power.

  MotorOff();
  
  // Update performed.
  
  return -1;
}
