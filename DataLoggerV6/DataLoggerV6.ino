#include <SPI.h>
#include <SdFat.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SparkFunMPL3115A2.h>


/*
   SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 10

  MPL3115A2 attached to I2C bus as follows:
 ** Vin - 5V
 ** SCL - A5
 ** SDA - A4

  BNO055 attached to I2C bus as follows:
 ** Vin - 5V
 ** SDA - A4
 ** SCL - A5

*/

//Complier Flags
#define DEBUG false
#define DEBUG_HEX false
#define RUNONCE true
#define TIMEOUT false

//Compiler Constants
#define MAX_FILE_LEN 13
#define MAX_FILES 100
#define ACCL_ID 0x28
#define ALT_ID 0x60 // 0xC1 for 8-bit

#define BUFFER_SIZE 1 // Number of packets in the buffer
#define PACKET_LENGTH 4 * (13) // Bytes per packet

//Global objects
SdFat sd;
File fs;
char filename[MAX_FILE_LEN];
Adafruit_BNO055 bno = Adafruit_BNO055();

//unsigned long endtime = 10 * 60*1000;  //Unsigned long for the time logging should halt as a fallback
float currTime = 0;

byte *dataBuffer = new byte[BUFFER_SIZE * PACKET_LENGTH]; // Data buffer for logging data prior to flushing to SD
int bufferPos = 0;

MPL3115A2 myPressure;

float pressure;
float temperature;
float altitude;

void setup() {
  //Initialize blinky LED
  pinMode(A0, OUTPUT);
  digitalWrite(A0, HIGH);
  
  // Open serial communications and wait for port to open:
  Wire.begin();
  Serial.begin(115200);
  while (DEBUG && !Serial) {
    // Wait for serial port to connect. Blink LED. Needed for native USB only
	digitalWrite(A0, LOW);
	delay(500);
	digitalWrite(A0, HIGH);
	delay(500);
  }
  myPressure.begin();
  myPressure.setModeAltimeter();

  myPressure.setOversampleRate(1);
  myPressure.enableEventFlags();
  
  //SD card check
  Serial.print("Initializing SD card...");
  if (!sd.begin(10)) {
    Serial.println("initialization failed!");
    while(1)
	{
	  digitalWrite(A0, LOW);
	  delay(100);
	  digitalWrite(A0, HIGH);
	  delay(100);
	}
  }
  Serial.println("initialization done.");

  //Find the last flight data and make a new file after the last one ************
  
  int i = 0;
  strcpy(filename, "flight00.txt");
  while (i < MAX_FILES) { // iterate until we find a file that still hasn't been written
    char num[3];
    itoa(i, num, 10); // convert the file number to a C string/char array
    strcpy(&filename[6], num);
    if (!sd.exists(filename)) { // if the file is available...
      break; // we're done here
    }
    i++;
  }

  fs =  sd.open(filename, FILE_WRITE);
  Serial.print(filename);

  //****************************************************************************

  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1)
	
	  digitalWrite(A0, LOW);
	  delay(100);
	  digitalWrite(A0, HIGH);
	  delay(100);
	  digitalWrite(A0, LOW);
	  delay(100);
	  digitalWrite(A0, HIGH);
	  delay(500);
	
  }
  
  Serial.println();
  bno.setExtCrystalUse(true);
  
  digitalWrite(A0, LOW);
}

void loop() {

  sensors_event_t oriEvent, gyroEvent, accEvent;
  bno.getEvent(&oriEvent, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&gyroEvent, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&accEvent, Adafruit_BNO055::VECTOR_LINEARACCEL);

  //The sensors update very fast, so they have to be measure beforehand
  //to have agreement between the Debug and recorded values.
  currTime=float(millis());
  float alt = myPressure.readAltitude();
  float pressure = myPressure.readPressure();
  float temp = myPressure.readTemp();
  float oriX = oriEvent.orientation.x;
  float oriY = oriEvent.orientation.y;
  float oriZ = oriEvent.orientation.z;
  float accX = accEvent.acceleration.x;
  float accY = accEvent.acceleration.y;
  float accZ = accEvent.acceleration.z;
  float gyroX = gyroEvent.gyro.x;
  float gyroY = gyroEvent.gyro.y;
  float gyroZ = gyroEvent.gyro.z;

  store(0, currTime);
  store(4, alt);
  store(8, pressure);
  store(12, temp);
  store(16, oriX);
  store(20, oriY);
  store(24, oriZ);
  store(28, accX);
  store(32, accY);
  store(36, accZ);
  store(40, gyroX);
  store(44, gyroY);
  store(48, gyroZ);

  if (DEBUG) 
  {
    if (!DEBUG_HEX) {
      Serial.print("Time: ");
      Serial.println(currTime);
      Serial.print("Altitude: ");
      Serial.println(alt);
      Serial.print("Pressure: ");
      Serial.println(pressure);
      Serial.print("Temperature: ");
      Serial.println(temp);
      Serial.println("Orientation: ");
      Serial.println(oriX);
      Serial.println(oriY);
      Serial.println(oriZ);
      Serial.println("Linear Acceleration: ");
      Serial.println(accX);
      Serial.println(accY);
      Serial.println(accZ);
      Serial.println("Angular Acceleration: ");
      Serial.println(gyroX);
      Serial.println(gyroY);
      Serial.println(gyroZ);
    } else {
      char* buffer = new char[4];
      

      for (int j = 0; j < 4; j++) {
        char hexCar[1];

        sprintf(hexCar, "%02X", (uint8_t) buffer[j]);
        Serial.print(hexCar);
      }
    }
  }

  fs.write(dataBuffer, BUFFER_SIZE * PACKET_LENGTH);
  fs.flush();

  if (!digitalRead(6))
  {
	digitalWrite(A0, HIGH);
    fs.close();
    if (DEBUG) {
      Serial.println("Done!");
    }
    while (1);
  }
}

template <typename TYPE>
void store(int pos, TYPE data) {
  const char* bytes = (const char*) &data; // tell C that, whatever this data was, treat it as if it were raw bytes/chars
  storeBytes(bytes, sizeof(TYPE), pos); // add to the write buffer
}

void storeBytes(const char* bytes, int datasize, int pos) {
  for(int i = 0; i < datasize; i++) { // iterate over data given
    dataBuffer[pos+i] = bytes[i]; // add each byte to the buffer & increment count
  }
}
