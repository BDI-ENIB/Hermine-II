
#include <BMP388_DEV.h>                             // Include the BMP388_DEV.h library
#include <MPU6500_WE.h>
#include <SPIFlash.h>    //get it here: https://github.com/LowPowerLab/SPIFlash
#include <SPI.h>

int good;

int cs_pre=18;
int cs_flash=14; 
int cs_imu=15;
int int_pre=17;
int int_imu=16;



//volatile float temperature, pressure, altitude;              // Create the temperature, pressure and altitude variables
//volatile boolean PreDataReady = false;   
BMP388_DEV bmp388(cs_pre);                              // Instantiate (create) a BMP388_DEV object and set-up for SPI operation on digital pin D10

//volatile xyzFloat gValue;
//volatile xyzFloat gyr;
//volatile float temp;
//volatile boolean ImuDataReady = false; 
bool useSPI = true;    // SPI use flag
MPU6500_WE myMPU6500 = MPU6500_WE(&SPI, cs_imu, useSPI);

uint32_t addresse=0;
SPIFlash flash(cs_flash);

IntervalTimer myTimer;

int count255=0;
uint32_t blockstart=0;

float ax2;
float ay2;
float az2;
float gx2;
float gy2;
float gz2;

float pressure;
float temperature;


void setup() 
{
  pinMode(cs_pre, OUTPUT);
  digitalWrite(cs_pre, HIGH);
  pinMode(cs_flash, OUTPUT);
  digitalWrite(cs_flash, HIGH);
  pinMode(cs_imu, OUTPUT);
  digitalWrite(cs_imu, HIGH);

  
  Serial.begin(115200);   
  Serial.println("start");

  
  //bmp388.setClock(400000);
  good=bmp388.begin();                                   // Default initialisation, place the BMP388 into SLEEP_MODE 
  //bmp388.enableInterrupt();
  //SPI.usingInterrupt(digitalPinToInterrupt(int_pre));       // Invoke the SPI usingInterrupt() function
  //attachInterrupt(digitalPinToInterrupt(int_pre), PreInterruptHandler, RISING);
  Serial.println(good); 
  bmp388.setTimeStandby(TIME_STANDBY_40MS);       // Set the standby time to 40 mseconds


  good=1;
  good=good & myMPU6500.init();
  Serial.println("Position you MPU6500 flat and don't move it - calibrating...");
  delay(1000);
  myMPU6500.autoOffsets();
  Serial.println("Done!");
  
  myMPU6500.enableGyrDLPF();
  myMPU6500.setGyrDLPF(MPU6500_DLPF_6);  
  myMPU6500.setGyrRange(MPU6500_GYRO_RANGE_250);
  myMPU6500.setAccRange(MPU6500_ACC_RANGE_16G);
  myMPU6500.enableAccDLPF(true);
  myMPU6500.setAccDLPF(MPU6500_DLPF_6);
  myMPU6500.setSampleRateDivider(32);

  myMPU6500.setIntPinPolarity(MPU6500_ACT_HIGH);
  myMPU6500.enableIntLatch(true);
  myMPU6500.enableClearIntByAnyRead(true);
  
  bmp388.setClock(2000000);
  //SPI.usingInterrupt(digitalPinToInterrupt(int_imu));       // Invoke the SPI usingInterrupt() function
  //attachInterrupt(digitalPinToInterrupt(int_imu), ImuInterruptHandler, RISING);





  good=good&flash.initialize();
  Serial.print("flash DeviceID: ");
  Serial.println(flash.readDeviceId(), HEX);

  



  if (good){
    noInterrupts();
    Serial.println("init ok");
    Serial.print("addresse début flash:");
    Serial.println(addresse);
    
    //myMPU6500.enableInterrupt(MPU6500_DATA_READY );
    //bmp388.startNormalConversion();         // Start BMP388 continuous conversion in NORMAL_MODE
    
    //myTimer.begin(FlashInterruptHandler, 250000);
    interrupts();
  }
  else{
    Serial.println("init cpt");
    }  
}

void loop() {
  // put your main code here, to run repeatedly:
   uint8_t data=flash.readByte(addresse);
   if (data==255){
      count255++;
    }
   else{
    count255=0;
    blockstart=addresse;
    Serial.print(blockstart);
    Serial.print(",");
    }
    if (count255<5){
      /*
     Serial.print(addresse);
     Serial.print(":");
     
     Serial.println(data);
     */
     addresse=decodageTramme(addresse);
        
        
        float resultant = 0.0;
        resultant = sqrt(sq(ax2) + sq(ay2) + sq(az2));
        Serial.print(ax2);
        Serial.print(",");
        //Serial.print(sizeof(ax2));
        Serial.print(ay2);
        Serial.print(",");
        Serial.print(az2);
        Serial.print(",");
        Serial.print(resultant);
      
        Serial.print(",");
        Serial.print(gx2);
        Serial.print(",");
        Serial.print(gy2);
        Serial.print(",");
        Serial.print(gz2);

        
        Serial.print(",");
        Serial.print(temperature);     
        Serial.print(",");
        Serial.println(pressure);    
        
    }
    else{
      addresse++;
    }
}

uint32_t decodageTramme(uint32_t addresse){
      //separateur
      if(flash.readByte(addresse)!=69){
        Serial.println("probleme debut trame");
        Serial.println(flash.readByte(addresse));
        Serial.println(addresse);
      }
      
      addresse+=1;
      //barometre
      temperature=readFlashToFloat(addresse);
      addresse+=4;

      pressure=readFlashToFloat(addresse);
      addresse+=4;
      
      //imu
      ax2=readFlashToFloat(addresse);
      addresse+=4;
      ay2=readFlashToFloat(addresse);
      addresse+=4;
      az2=readFlashToFloat(addresse);
      addresse+=4;

      gx2=readFlashToFloat(addresse);
      addresse+=4;
      gy2=readFlashToFloat(addresse);
      addresse+=4;
      gz2=readFlashToFloat(addresse);
      addresse+=4;
      return addresse;
  }

float readFlashToFloat(uint32_t  addresse){
    
    float result=0;    
    uint8_t *ptrToFloat;
    ptrToFloat = (uint8_t *)&result;
    
    ptrToFloat[0]=flash.readByte(addresse);
    ptrToFloat[1]=(flash.readByte(addresse+1));
    ptrToFloat[2]=(flash.readByte(addresse+2));
    ptrToFloat[3]=(flash.readByte(addresse+3));

    return result;
  }
