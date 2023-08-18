#include <Arduino.h>
#include <SPICREATE.h>
#include <LPS25HB.h>
#define CS 13
#define SCK 33
#define MISO 25
#define MOSI 26
SPICREATE::SPICreate spi;
LPS lps;
int flag=0;
void setup(){
  Serial.begin(115200);
  Serial.println("serial set done");
  delay(100);
  spi.begin(VSPI,SCK,MISO,MOSI);
  Serial.println("spi set done");
  delay(100);
  lps.begin(&spi,CS,5000000);
  Serial.println("lps set done");
}
void loop(){
  if(Serial.available()){
    if(Serial.read()=='l'){
      flag=1;
    }
  }
  if(flag>0){
    uint8_t data[3] ={};
    lps.Get(data);
    for(int i=0;i<3;i++){
      Serial.print(data[i]);
      Serial.print(",\t");
    }
    Serial.println(lps.Plessure);
  }
  delay(1000);
}