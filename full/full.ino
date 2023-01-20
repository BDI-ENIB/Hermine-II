
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



volatile float temperature, pressure, altitude;              // Create the temperature, pressure and altitude variables
volatile boolean PreDataReady = false;   
BMP388_DEV bmp388(cs_pre);                              // Instantiate (create) a BMP388_DEV object and set-up for SPI operation on digital pin D10

volatile xyzFloat gValue;
volatile xyzFloat gyr;
volatile float temp;
volatile boolean ImuDataReady = false; 
bool useSPI = true;    // SPI use flag
MPU6500_WE myMPU6500 = MPU6500_WE(&SPI, cs_imu, useSPI);


volatile uint32_t addresse;
SPIFlash flash(cs_flash);

IntervalTimer myTimer;

bool printSerial=false;


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
  bmp388.enableInterrupt();
  SPI.usingInterrupt(digitalPinToInterrupt(int_pre));       // Invoke the SPI usingInterrupt() function
  attachInterrupt(digitalPinToInterrupt(int_pre), PreInterruptHandler, RISING);
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
  SPI.usingInterrupt(digitalPinToInterrupt(int_imu));       // Invoke the SPI usingInterrupt() function
  attachInterrupt(digitalPinToInterrupt(int_imu), ImuInterruptHandler, RISING);





  good=good&flash.initialize();
  Serial.print("flash DeviceID: ");
  Serial.println(flash.readDeviceId(), HEX);

  



  if (good){
    noInterrupts();
    Serial.println("init ok");
    addresse=flash.readByte(0)+(flash.readByte(1)<<8)+(flash.readByte(2)<<16);
    Serial.print("addresse lu flash:");
    Serial.println(addresse);
    if (addresse==0xFFFFFF)
    {
      addresse=4096;
      flash.blockErase4K(0);
      flash.writeByte(0,(addresse&0x0000FF));
      flash.writeByte(1,(addresse&0x00FF00)>>8);
      flash.writeByte(2,(addresse&0xFF0000)>>16);
    }
    else{
      addresse=((addresse/4096)*4096)+4096;
      flash.blockErase4K(0);
      flash.writeByte(0,(addresse&0x0000FF));
      flash.writeByte(1,(addresse&0x00FF00)>>8);
      flash.writeByte(2,(addresse&0xFF0000)>>16);
      }
    Serial.print("addresse debut flash:");
    Serial.println(addresse);
    
    myMPU6500.enableInterrupt(MPU6500_DATA_READY );
    bmp388.startNormalConversion();         // Start BMP388 continuous conversion in NORMAL_MODE
    
    myTimer.begin(FlashInterruptHandler, 50000);
    interrupts();
  }
  else{
    Serial.println("init cpt");
    }  
}

void loop() 
{ 
  if (printSerial){
    if (PreDataReady){
        noInterrupts();
        PreDataReady=false;
        float temperature2=temperature;
        float pressure2=pressure;
        float altitude2=altitude;
        uint32_t addr=addresse;
        interrupts();
        Serial.println(good);
        Serial.print(temperature2);                        
        Serial.print(F("*C   "));
        Serial.print(pressure2);    
        Serial.print(F("hPa   "));
        Serial.print(altitude2);
        Serial.println(F("m"));
        Serial.print("derniere addresse flash:"); 
        Serial.println(addr);
        Serial.println(flash.readByte(0));
        Serial.println(flash.readByte(1));
        Serial.println(flash.readByte(2));
        
      }
  
      if (ImuDataReady){
        noInterrupts();
        ImuDataReady=false;
        float ax2=gValue.x;
        float ay2=gValue.y;
        float az2=gValue.z;
        float gx2=gyr.x;
        float gy2=gyr.y;
        float gz2=gyr.z;
        float temp2=temp;
        interrupts();
        
        //float resultantG = myMPU6500.getResultantG(g2);
        float resultant = 0.0;
        resultant = sqrt(sq(ax2) + sq(ay2) + sq(az2));
        Serial.println("Acceleration in g (x,y,z):");
        Serial.print(ax2);
        //Serial.print(sizeof(ax2));
        Serial.print("   ");
        Serial.print(ay2);
        Serial.print("   ");
        Serial.println(az2);
        Serial.print("Resultant g: ");
        Serial.println(resultant);
      
        Serial.println("Gyroscope data in degrees/s: ");
        Serial.print(gx2);
        Serial.print("   ");
        Serial.print(gy2);
        Serial.print("   ");
        Serial.println(gz2);
      
        Serial.print("Temperature in °C: ");
        Serial.println(temp2);
        //interrupts();
      }
  }
}

void PreInterruptHandler()                               // Interrupt service routine (ISR)
{   
  bmp388.getMeasurements(temperature, pressure, altitude);   // Get the data
  PreDataReady = true;                               // Set the data ready flag

}

void ImuInterruptHandler(){
  gValue = myMPU6500.getGValues();
  gyr = myMPU6500.getGyrValues();
  temp = myMPU6500.getTemperature();
  
  ImuDataReady = true; 
  }

void FlashInterruptHandler(){

      //separateur
      flash.writeByte(addresse,69);
      addresse+=1;
      //barometre
      writeFloatToFlash(temperature,addresse);
      addresse+=4;

      writeFloatToFlash(pressure,addresse);
      addresse+=4;
      
      //imu
      writeFloatToFlash(gValue.x,addresse);
      addresse+=4;
      writeFloatToFlash(gValue.y,addresse);
      addresse+=4;
      writeFloatToFlash(gValue.z,addresse);
      addresse+=4;

      writeFloatToFlash(gyr.x,addresse);
      addresse+=4;
      writeFloatToFlash(gyr.y,addresse);
      addresse+=4;
      writeFloatToFlash(gyr.z,addresse);
      addresse+=4;
      
      //ecriture addresse suivante
      flash.blockErase4K(0);
      flash.writeByte(0,(addresse&0x0000FF));
      flash.writeByte(1,(addresse&0x00FF00)>>8);
      flash.writeByte(2,(addresse&0xFF0000)>>16);
}

void writeFloatToFlash(float data,uint32_t  addresse){
    uint8_t *ptrToFloat;
    ptrToFloat = (uint8_t *)&data;
    
    flash.writeByte(addresse,ptrToFloat[0]);
    flash.writeByte(addresse+1,ptrToFloat[1]);
    flash.writeByte(addresse+2,ptrToFloat[2]);
    flash.writeByte(addresse+3,ptrToFloat[3]);
  }
