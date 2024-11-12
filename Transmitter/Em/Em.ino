//Transmitter program

#include "Mirf.h"

Nrf24l Mirf = Nrf24l(9, 10); // CE,CSN
bool motor_EN = false;
byte value[4];

void setup()
{
  Serial.begin(115200);
  pinMode(A1,INPUT);//roll
  pinMode(A2,INPUT);//pitch
  pinMode(A3,INPUT);//yaw
  pinMode(A5,INPUT);//throttle
  Mirf.spi = &MirfHardwareSpi;
  Mirf.init();
  Mirf.setOutputRF_PWR(2);
  Mirf.payload = sizeof(value);
  Mirf.channel = 90;  
  Mirf.setSpeedDataRates(1);
  Mirf.config();
  
  // Set destination address to TX_ADDR
  // Set ACK waiting address to RX_ADDR_P0
  Mirf.setTADDR((byte *)"XZXCC");

}

void loop()
{
  Mirf.send(value);
  Serial.print("Wait for sending.....");
  // Verify send was successfuly
  if (Mirf.isSend()) {
    Serial.println("Send success:");
    value[0] = map(analogRead(A1),0,1023,0,255);
    value[1] = map(analogRead(A2),0,1023,0,255);;
    value[2] = map(analogRead(A3),0,1023,0,255);;
  } else {
    Serial.println("Send fail:");
  }
  delay(20);
}

uint8_t motor_on_timer = 0;

unsigned long time_now = millis();

void loop() {
  // printf_begin();
  // radio.printDetails();
  value[0] = map(analogRead(A2),0,1023,80,175);
  value[1] = map(analogRead(A1),0,1023,90,165);
  value[2] = map(analogRead(A3),0,1023,90,165);

  if(millis()-time_now>20&&!motor_EN){
    time_now = millis();
    if(analogRead(A5)>1000){
      motor_on_timer++;
    }
    else{
      motor_on_timer = 0;
    }
  }

  if(motor_on_timer>150&&!motor_EN){
    motor_EN = true;
    value[3] = 30;
    Mirf.send(value);
    delay(5000);
  }

  if(motor_EN){
    value[3] = map(analogRead(A5),0,1023,0,255);
  }
  else{
    value[3] = 0;
  }

  Mirf.send(value);
  
}
