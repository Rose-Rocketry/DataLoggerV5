#include <SdFat.h>
#include <SPI.h>

#define MAX_FILE_LEN 13
#define MAX_FILES 100
#define DEBUG_PRINT_HEX_TO_SERIAL false
#define SDFAT_FILE_TYPE 1

SdFat32 sd;
char filename[MAX_FILE_LEN];

void setup() {
  pinMode(A0, OUTPUT);
  
  Serial.begin(115200);
  if(!sd.begin(10)){
    if (DEBUG_PRINT_HEX_TO_SERIAL) {
      Serial.print("Could not connect to SD card!");
    }
    while(1) {
      //Always use blinky lights for error states
      //We should do different frequencies/patterns for different errors
      digitalWrite(A0, LOW);
      delay(100);
      digitalWrite(A0, HIGH);
      delay(100);
    }
  }

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
  
  pinMode(A0, OUTPUT);
  digitalWrite(A0, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (DEBUG_PRINT_HEX_TO_SERIAL) {
    Serial.print("Hello from loop!");
  }
  while(!Serial.available());
  
  digitalWrite(A0, LOW);
  
  Serial.write('p');
  delay(500);
  
  int i = 0;
  char tryfilename[MAX_FILE_LEN];
  char previousFilename[MAX_FILE_LEN];  
  strcpy(tryfilename, "flight00.txt");
  while (i < MAX_FILES) { // iterate until we find a file that still hasn't been written
    char num[3];
    itoa(i, num, 10); // convert the file number to a C string/char array
    strcpy(&tryfilename[6], num);
    if (!sd.exists(tryfilename)) { // if the file is available...
      break; // we're done here
    }
    i++;
    strcpy(previousFilename,tryfilename); 
  }
  
  //Serial.println(filename);
  File32 dataFile;//= sd.open(previousFilename, FILE_READ);
  dataFile.open(previousFilename, O_READ);
  emptySerial();
  //Serial.write(previousFilename, MAX_FILE_LEN);
  uint32_t fsize = dataFile.size();
  if (!DEBUG_PRINT_HEX_TO_SERIAL) {
    writeFileBytes(fsize);
  } else {
    Serial.println(fsize);
  }
  
  Serial.flush();

  char* buf = new char[4];
  float out;
  char hexCar[8];
  
  for(long int i = 0; i < fsize / 4; i++){
    dataFile.read(buf, 4);
    out = *((float*) buf);
    if (!DEBUG_PRINT_HEX_TO_SERIAL) {
      // Write the bytes as normal if we are in normal time
      writeFileBytes(out);
      //Serial.write(buf, 4);
      Serial.flush();
    } else {
      Serial.print(F("line "));
      Serial.print(i);
      Serial.print(F("\t:\t"));
      for (int j = 0; j < 4; j++) {
        sprintf(hexCar, "%02X", (uint8_t) buf[j]);
        Serial.print(hexCar);
      }
//      Serial.print(F("\tFree memory: "));
//      Serial.print(freeMemory());
      Serial.println();
    }
  }

  digitalWrite(A0, HIGH);
  dataFile.close();
  while(1) {
    delay(1000);
  }
}

void emptySerial(){
  while(Serial.available() > 0) {
    char t = Serial.read();
  }
}

template <typename TYPE>
void writeFileBytes(TYPE data){
  const char* bytes = (const char*) &data;
  Serial.write(bytes, sizeof(TYPE));
}
