/*
  SD card read/write

 This example shows how to read and write data to and from an SD card file
 The circuit:
 * SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 4 (for MKRZero SD: SDCARD_SS_PIN)

 created   Nov 2010
 by David A. Mellis
 modified 9 Apr 2012
 by Tom Igoe

 This example code is in the public domain.

 */

#include <SPI.h>
#include <SD.h>
#include "data_struct.h"
File myFile;

int write_to_sd(File file,data_struct * data ){
  
  // if the file didn't open, print an error:
  if (!file){
    Serial.println("error opening test.txt");
    return -1;//-1 means error  
  }   
  
  Serial.print("Writing to txt...");
  myFile.println("==========================================");
  myFile.println();  //print time of measurement
  file.println(data->height);
  file.println(data->temperature);
  for(int i;i<3;i++)  {
    myFile.print(data->acceleration[i]);
    myFile.print(" ");
  }
  myFile.println();
  
  for(int i;i<3;i++){  
    myFile.print(data->orientation[i]);
    myFile.print(" ");
  }
  myFile.println();
  
    
  //myFile.close();
  Serial.println("done.");
  return 0;
   
}

int read_sd_file(File file){
  //open the file for reading:
  file = SD.open("test.txt");
  if (file) {
    Serial.println("test.txt:");

    // read from the file until there's nothing else in it:
    while (file.available()) {
      Serial.write(file.read());
    }
    // close the file:
    file.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening test.txt");
  }
}


void reset_struct(data_struct * data){
    for(int i;i<3;i++) data->acceleration[i]=0;
    data->height=0;
    for(int i;i<3;i++) data->orientation[i]=0;
    data->temperature=0;
  }

void update_data(data_struct * data){
  int x[3]={1,2,3};
  
  for(int i;i<3;i++) data->acceleration[i]=x[i];
  data->height=26;
  for(int i;i<3;i++) data->orientation[i]=x[i];
  data->temperature=9;

}


void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect.
  }


  Serial.print("Initializing SD card...");

  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myFile = SD.open("mlkia.txt", FILE_WRITE);
  update_data(&Data);
  Serial.println(Data.temperature);
  write_to_sd(myFile,&Data);
  myFile.close();
}


void loop() {

}
