// Project staat op GitHub : https://github.com/Rene081960/Wall-E
//
// PWM pins : Uno, Nano, Mini  3, 5, 6, 9, 10, 11   490 Hz (pins 5 and 6: 980 Hz)

// Include Adafruit PWM Library
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// MP3 Player
#include "SoftwareSerial.h"
SoftwareSerial mySerial(10, 11);
# define Start_Byte 0x7E
# define Version_Byte 0xFF
# define Command_Length 0x06
# define End_Byte 0xEF
# define Acknowledge 0x00 //Returns info with command 0x41 [0x01: info, 0x00: no info]
# define ACTIVATED LOW
boolean isPlaying = false;

// Define Adafruit PWM settings
#define MIN_PULSE_WIDTH       650
#define MAX_PULSE_WIDTH       2350
#define FREQUENCY             50

// Define Servo Outputs on 16 channel PCA9685 board
#define SERVO1 0
#define SERVO2 1
#define SERVO3 2
#define SERVO4 3
#define SERVO5 4
#define SERVO6 5
#define SERVO7 6

// Motor A
#define enA 9
#define in1 8
#define in2 7

// Motor B
#define enB 3
#define in3 5
#define in4 4

const int motorOffset = 25;

// Voltmeter
int sensorPin = A0; 

float voltage = 0.0; 
float sensorValue = 0.0f;  
float vout = 0.0;
float vin = 0.0;
float R1 = 100000.0; 
float R2 = 50000.0; 

long previousMillis = 0;
long interval = 5000;  // Check om de 1 seconde het voltage

// Test
#define interruptPin 2  // not used at the moment

const int ledPin = 6;
boolean isLedOn = false;

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
String commandString = "";

void setup() 
{  
  Serial.begin(9600);
  mySerial.begin (9600);

  // Setup PWM Controller object
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);
  
  pinMode(ledPin,OUTPUT);
  pinMode(interruptPin, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(interruptPin), ButtonClickedISR, CHANGE); // not used
  
  //initDisplay();
  Serial.println("Connected");
  
  delay(500);

  InitializeWallE();
}

void loop() 
{ 
  CheckActions();
  PrintVoltage();
}

void CheckActions()
{
  if(stringComplete)
  {
    stringComplete = false;
    GetCommand();

    if(commandString.equals("CONN"))
    {
      PrintText("Connected");
    } 
    else if(commandString.equals("DISC"))
    {
    }
    else if(commandString.equals("REST"))  // Reset
    {
      InitializeWallE();
    }       
    else if(commandString.equals("TEXT"))
    {
      String text = GetTextToPrint();
    }   
    else if(commandString.equals("LED1"))
    {
      ZetLedAanUit(ledPin);
    }
    else if(commandString.equals("SER1"))
    {
      ServoControlContra(SERVO1); // Arm links
    }
    else if(commandString.equals("SER2"))
    {
      ServoControl(SERVO2); // Arm Rechts
    }
    else if(commandString.equals("SER3"))
    {
      ServoControl(SERVO3); // Hoofd
    }
    else if(commandString.equals("SER4"))
    {
      ServoControl(SERVO4); // Nek onder
    }
    else if(commandString.equals("SER5"))
    {
      ServoControl(SERVO5); // Nek boven
    }
    else if(commandString.equals("SER6"))
    {
      ServoControl(SERVO6); // Oog links
      ServoControlContra(SERVO7); // Oog Rechts
    }
    else if(commandString.equals("PWM1"))
    {
      ZetLedPWM(ledPin); // Led ogen
    }
    else if(commandString.equals("MP3W"))
    {
      ZetGeluidAanUit();
    }
    else if(commandString.equals("VOOR"))
    {
      MotorControl("VOOR");
    }
    else if(commandString.equals("LINK"))
    {
      MotorControl("LINKS");
    }   
    else if(commandString.equals("RECH"))
    {
      MotorControl("RECHTS");
    }   
    else if(commandString.equals("ACHT"))
    {
      MotorControl("ACHTER");
    }   
    else if(commandString.equals("STOP"))
    {
      MotorControl("STOP");
    }   
    else if(commandString.equals("SNEL"))
    {
      MotorSpeed(enA);
      MotorSpeed(enB);
    } 
    else if(commandString.equals("VOLU"))
    {
      ZetVolume();
    }
       
    inputString = "";
  }  
}

void InitializeWallE()
{
  ServoControlContraInitialize(SERVO1, 0); // Arm links omlaag = 0, omhoog = 623
  ServoControlInitialize(SERVO2, 200); // Arm rechts omlaag = 200, omhoog = 800
  ServoControlInitialize(SERVO3, 512); // Hoofd
  ServoControlInitialize(SERVO4, 0); // Nek onder
  ServoControlInitialize(SERVO5, 0); // Nek boven
  ServoControlInitialize(SERVO6, 420); // Oog links
  ServoControlInitialize(SERVO7, 600); // Oog rechts
  ZetLedPWM(ledPin); // Led ogen
  MotorControl("STOP");
  Serial.println("Klaar initialiseren Servo's");
}

void PrintText(String melding)
{
  Serial.println(melding);
}

void ZetGeluidAanUit()
{
  if (!isPlaying)
  {
    Serial.println("Geluid aan");
    PlayFirst();
    isPlaying = true;
  }
  else
  {
    Serial.println("Geluid uit");
    Pause();
    isPlaying = false;
  }  
}

void ZetVolume()
{
  // op de juiste pin het volume zetten 0 tot 30
  String volume = inputString.substring(5,(inputString.length() - 1));
  SetVolume(volume.toInt());
}

void ZetLedPWM(int pin)
{
  int waarde = (inputString.substring(5,(inputString.length() - 1))).toInt();
  analogWrite(pin, waarde); 
}

// Zet de snelheid van de motoren (PWM)
void MotorSpeed(int pin)
{
  String motorSpeed = inputString.substring(5,(inputString.length() - 1));
  analogWrite(pin, motorSpeed.toInt());
}

// De aktie die de motoren moeten uitvoeren, voorwaards, achterwaards etc.
void MotorControl(String action)
{
  if (action == "VOOR")
  {
    MotorAction(HIGH, LOW, HIGH, LOW, "Motoren voorwaards");
  }
  else if (action == "ACHTER")
  {
    MotorAction(LOW, HIGH, LOW, HIGH, "Motoren achterwaards");
  }
  else if (action == "LINKS")
  {
    MotorAction(LOW, HIGH, HIGH, LOW, "Motoren linksom");
  }
  else if (action == "RECHTS")
  {
    MotorAction(HIGH, LOW, LOW, HIGH, "Motoren rechtsom");
  }   
  else if (action == "STOP")
  {
    MotorAction(LOW, LOW, LOW, LOW, "Motoren stop");
  }   
}

void MotorAction(bool a, bool b, bool c, bool d, String melding)
{
  digitalWrite(in1, a);
  digitalWrite(in2, b);
  
  digitalWrite(in3, c);
  digitalWrite(in4, d);    
  
  // Stuur een bericht naar de computer terug. 
  Serial.println(melding);
}

// Function to move servo to specific position
void ServoControl(int servoOut)
{
  int waarde = (inputString.substring(5,(inputString.length() - 1))).toInt();
  ServoPWM(servoOut, waarde);
}

// Function to move servo to specific position
void ServoControlContra(int servoOut)
{
  int waarde = (inputString.substring(5,(inputString.length() - 1))).toInt();
  ServoPWMContra(servoOut, waarde);
}

// Function to move servo to specific position
void ServoControlInitialize(int servoOut, int waarde)
{
  //ServoPWMRange(servoOut, 0, waarde);
  ServoPWM(servoOut, waarde);
}

void ServoControlContraInitialize(int servoOut, int waarde)
{
  ServoPWMContra(servoOut, waarde);
}

void ServoPWMRange(int servoOut, int pos1, int pos2)
{
  for (int i = pos1; i <= pos2; i++)
  {
    ServoPWM(servoOut, i);
  }
}

void ServoPWM(int servoOut, int waarde)
{
  // Definieer variabelen
  int pulse_wide, pulse_width;

  // Convert to pulse width
  pulse_wide = map(waarde, 0, 1023, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  pulse_width = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
  
  //Control Motor
  pwm.setPWM(servoOut, 0, pulse_width);  
}

void ServoPWMContra(int servoOut,int waarde)
{
  // Definieer variabelen
  int pulse_wide, pulse_width;

  // Convert to pulse width
  pulse_wide = map(waarde, 1023, 0, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  pulse_width = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
  
  //Control Motor
  pwm.setPWM(servoOut, 0, pulse_width);  
}

void ZetLedAanUit(int pin)
{
  if(!isLedOn)
  {
    digitalWrite(pin,HIGH);
    isLedOn = true;   
  }
  else
  {
    digitalWrite(pin,LOW);
    isLedOn = false;
  } 
}

void GetCommand()
{
  if(inputString.length()>0)
  {
     commandString = inputString.substring(1,5);
  }
}

String GetTextToPrint()
{
  String value = inputString.substring(5,inputString.length()-2);
  return value;
}

void serialEvent() 
{
  while (Serial.available()) 
  {
    // get the new byte:
    char inChar = (char)Serial.read();
  
    // add it to the inputString:
    inputString += inChar;
  
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') 
    {
      stringComplete = true;
    }
  }
}

void ButtonClickedISR()
{
  // Stuur een bericht naar de computer terug. 
  Serial.println("Button clicked.");
}

void PlayFirst()
{
  Execute_CMD(0x3F, 0, 0);
  delay(300);
  SetVolume(15);
  delay(300);
  Execute_CMD(0x11,0,1); 
  delay(300);
}

void Pause()
{
  Execute_CMD(0x0E,0,0);
  delay(500);
}

void Play()
{
  Execute_CMD(0x0D,0,1); 
  delay(500);
}

void PlayNext()
{
  Execute_CMD(0x01,0,1);
  delay(500);
}

void SetVolume(int volume)
{
  Execute_CMD(0x06, 0, volume); // Set the volume (0x00~0x30)
  delay(100);
}

void Execute_CMD(byte CMD, byte Par1, byte Par2)
{
  // Calculate the checksum (2 bytes)
  word checksum = -(Version_Byte + Command_Length + CMD + Acknowledge + Par1 + Par2);
  
  // Build the command line
  byte Command_line[10] = { Start_Byte, Version_Byte, Command_Length, CMD, Acknowledge,
  Par1, Par2, highByte(checksum), lowByte(checksum), End_Byte};
  
  //Send the command line to the module
  for (byte k=0; k<10; k++)
  {
    mySerial.write( Command_line[k]);
    Serial.write( Command_line[k]);
  }
}

void PrintVoltage()
{
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousMillis >= interval)
  {
    float voltage = readVoltage();
    Serial.println("Voltage : " + String(voltage));
    
    previousMillis = currentMillis;
  }
}

float readVoltage()
{
   sensorValue = analogRead(sensorPin);
   vout = (sensorValue * 5.0) / 1024.0; 
   vin = vout / (R2/(R1+R2)); 
   return vin;
}
