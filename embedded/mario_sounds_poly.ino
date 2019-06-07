//TMRpcm library is needed

#include "SD.h"
#define SD_ChipSelectPin 4
#include "TMRpcm.h"
#include "SPI.h"
#include "Wire.h"

//MISO -> Dpin12
//MOSI -> Dpin11
//SCK  -> 13

TMRpcm tmrpcm;
byte address = 8;
void setup()
{
  tmrpcm.speakerPin = 9;
  Wire.begin(address);
  Wire.onReceive(receiveEvent); // register event
  Serial.begin(115200);
  if (!SD.begin(SD_ChipSelectPin))
  {
    Serial.println("SD fail");
    return;
  }
  tmrpcm.setVolume(4);
  //tmrpcm.play("test.wav");
  //tmrpcm.loop(1);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()) {
    switch (Serial.read()) {
      case 'd': tmrpcm.play("test.wav"); tmrpcm.loop(1); break;
      case 'P': tmrpcm.play("test2.wav"); tmrpcm.loop(0); break;
      case 't': tmrpcm.play("catfish"); break;
      case 'p': tmrpcm.pause(); break;
      case '?': if (tmrpcm.isPlaying()) {
          Serial.println("A wav file is being played");
        } break;
      case 'S': tmrpcm.stopPlayback(); break;
      case '=': tmrpcm.volume(1); break;
      case '-': tmrpcm.volume(0); break;
      case '0': tmrpcm.quality(0); break;
      case '1': tmrpcm.quality(1); break;
      default: break;
    }
  }
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany) {
  int x = Wire.read();    // receive byte as an integer
  Serial.println(x);         // print the integer
  switch (x) {
    case 0: tmrpcm.pause(); break;
    case 1: tmrpcm.play("test.wav"); tmrpcm.loop(1); break;
    case 2: tmrpcm.play("test2.wav"); tmrpcm.loop(0); break;
    case 3: tmrpcm.pause(); break;

  }
}
