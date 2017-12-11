#include<i2c_t3.h>
#include<math.h>
#include<SoftwareSerial.h>

//bluetooth module
SoftwareSerial bt(10, 11); // RX, TX

//IMUs
//=======================================================================
#define MPU9250_ADDRESS 0x68
#define MAG_ADDRESS 0x0C
 
#define GYRO_FULL_SCALE_250_DPS 0x00 
#define GYRO_FULL_SCALE_500_DPS 0x08
#define GYRO_FULL_SCALE_1000_DPS 0x10
#define GYRO_FULL_SCALE_2000_DPS 0x18
 
#define ACC_FULL_SCALE_2_G 0x00 
#define ACC_FULL_SCALE_4_G 0x08
#define ACC_FULL_SCALE_8_G 0x10
#define ACC_FULL_SCALE_16_G 0x18

//Variable declarations
//Raw values
int16_t a_x[2], a_y[2], a_z[2];
int16_t g_x[2], g_y[2], g_z[2];
int16_t m_x, m_y, m_z;

//Scaled values
float sA_x[2], sA_y[2], sA_z[2];
float sG_x[2], sG_y[2], sG_z[2];

//Filtered values - current sample
float fA_x[2], fA_y[2], fA_z[2];
float fG_x[2], fG_y[2], fG_z[2];

//Filtered values - previous sample (Initialized to 0)
float fA_xOld[2] = {0}, fA_yOld[2] = {0}, fA_zOld[2] = {0};
float fG_xOld[2] = {0}, fG_yOld[2] = {0}, fG_zOld[2] = {0};

//Filter coefficients
float alphaAcc = 0.5;
float alphaGyr = 0.5;
float alphaMag = 0.5;

//Angle from gyroscope
float gyrAngX[2] = {0}, gyrAngY[2] = {0}, gyrAngZ[2] = {0};
//Roll, Pitch and Yaw
float Roll[2], Pitch[2], Yaw, YawU;

//For normalizing mag values
int16_t m_xMin = -434, m_xMax = 67;
int16_t m_yMin = -278, m_yMax = 215;
int16_t m_zMin = -465, m_zMax = 48;
float m_xMap, m_yMap, m_zMap;
//=======================================================================


//FSRs
#define fsr0 A9
#define slideFsr0 A8
#define slideFsr1 A7

int valFsr0, valSlideFsr0, valSlideFsr1;
float slide;

//switch array
#define sw0 2
#define sw1 3
#define sw2 4
#define sw0 5
#define sw1 6
#define sw2 7
#define sw0 8
#define sw1 9

uint8_t s[8], sArray;

// This function read Nbytes bytes from I2C device at address Address. 
// Put read bytes starting at register Register in the Data array. 
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
   
  // Read Nbytes
  Wire.requestFrom(Address, Nbytes); 
  uint8_t index=0;
  while (Wire.available())
  Data[index++]=Wire.read();
}
 
// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}

void I2Cread1(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire1.beginTransmission(Address);
  Wire1.write(Register);
  Wire1.endTransmission();
   
  // Read Nbytes
  Wire1.requestFrom(Address, Nbytes); 
  uint8_t index=0;
  while (Wire1.available())
  Data[index++]=Wire1.read();
}
 
// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte1(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire1.beginTransmission(Address);
  Wire1.write(Register);
  Wire1.write(Data);
  Wire1.endTransmission();
}

void getImuRaw(); //Collects data from the IMU
void filterRawData(); //Filter raw accel and gyro data
void computeRPY(); //Computes roll, pitch and yaw

void setup() {
  //Initialize IMUs
  //===========================================================================
  Wire.begin();
  Wire1.begin();
  Serial.begin(9600);
  Serial.println("started");
  bt.begin(38400);
  
  // Set accelerometers low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS,29,0x06);
  // Set gyroscope low pass filter at 5Hz
  I2CwriteByte(MPU9250_ADDRESS,26,0x06);
   
  // Configure gyroscope range
  I2CwriteByte(MPU9250_ADDRESS,27,GYRO_FULL_SCALE_250_DPS);
  // Configure accelerometers range
  I2CwriteByte(MPU9250_ADDRESS,28,ACC_FULL_SCALE_2_G);
  // Set sample rate to 100Hz
  I2CwriteByte(MPU9250_ADDRESS,0x19,0x09);
  // Set by pass mode for the magnetometers
  I2CwriteByte(MPU9250_ADDRESS,0x37,0x02);
     
  // Request continuous magnetometer measurements in 16 bits
  I2CwriteByte(MAG_ADDRESS,0x0A,0x16);

  //MPU9250_HIGH - second IMU. Magnetometer not enabled
  // Set accelerometers low pass filter at 5Hz
  I2CwriteByte1(MPU9250_ADDRESS,29,0x06);
  // Set gyroscope low pass filter at 5Hz
  I2CwriteByte1(MPU9250_ADDRESS,26,0x06);
   
   
  // Configure gyroscope range
  I2CwriteByte1(MPU9250_ADDRESS,27,GYRO_FULL_SCALE_250_DPS);
  // Configure accelerometers range
  I2CwriteByte1(MPU9250_ADDRESS,28,ACC_FULL_SCALE_2_G);
  // Set sample rate to 100Hz
  I2CwriteByte1(MPU9250_ADDRESS,0x19,0x09);
  //=========================================================================== 

  //Initialize switches connected to pins 2-9
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(5, INPUT);
  pinMode(6, INPUT);
  pinMode(7, INPUT);
  pinMode(8, INPUT);
  pinMode(9, INPUT);
  
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  //setup bluetooth communication
  bt.begin(9600);
  //setup complete
}

void sendValues()
{
  
}
void loop() {
  //read fsr values
  valFsr0 = map(analogRead(fsr0), 0, 1024, 0, 255);
  valSlideFsr0 = analogRead(slideFsr0);
  valSlideFsr1 = analogRead(slideFsr1);
  slide = int((float(valSlideFsr1) / 1000.0 - float(valSlideFsr0) / 1000.0 + 0.5)*127);
  
  //read from switch array. Switches connected to pins 2-9
  for(int i=2; i<10; i++)
    s[i-2] = digitalRead(i);  
  sArray = s[7]<<7 | s[6]<<6 | s[5]<<5 | s[4]<<4 | s[3]<<3 | s[2]<<2 | s[1]<<1 | s[0] ;
  
  //read IMU data
  getImuRaw();
  filterRawData();
  computeRPY();
//  Serial.print(fA_x[0]);
//  Serial.print("|");
//  Serial.print(fA_y[0]);
//  Serial.print("|");
  Serial.print(a_z[0]);
  Serial.print("|");
//  Serial.print(fA_x[1]);
//  Serial.print("|");
//  Serial.print(fA_y[1]);
//  Serial.print("|");
  Serial.print(a_z[1]);
  Serial.print("|");
  Serial.print(Roll[0]);
  Serial.print("|");
  Serial.print(Pitch[0]);
  Serial.print("|");
  Serial.print(Roll[1]);
  Serial.print("|");
  Serial.print(Pitch[1]);
  Serial.print("|");
  Serial.print(Yaw);
  Serial.print("|");
  Serial.print(sArray);
  Serial.print("|");
  Serial.print(valFsr0);
  Serial.print("|");
  Serial.print(valSlideFsr0);
  Serial.print("|");
  Serial.print(valSlideFsr1);
  Serial.print("|");
  Serial.println(slide);

  //bt.print(fA_x[0]);
  //bt.print("|");
  //bt.print(fA_y[0]);
  //bt.print("|");
  bt.print(a_z[0]);
  bt.print("|");
  //bt.print(fA_x[1]);
  //bt.print("|");
  //bt.print(fA_y[1]);
  //bt.print("|");
  bt.print(a_z[1]);
  bt.print("|");
  bt.print(Roll[0]);
  bt.print("|");
  bt.print(Pitch[0]);
  bt.print("|");
  bt.print(Roll[1]);
  bt.print("|");
  bt.print(Pitch[1]);
  bt.print("|");
  bt.print(Yaw);
  bt.print("|");
  bt.print(sArray);
  bt.print("|");
  bt.print(valFsr0);
  bt.print("|");
  bt.print(valSlideFsr0);
  bt.print("|");
  bt.print(valSlideFsr1);
  bt.print("|");
  bt.println(slide);
  //delay(300);
}


void getImuRaw()
{
  uint8_t Buf[2][14];
  for(int i = 0; i<2; i++)
  {
    // ____________________________________
    // ::: accelerometer and gyroscope :::
   
    // Read accelerometer and gyroscope
    if(i==0)  I2Cread(MPU9250_ADDRESS,0x3B,14,Buf[i]);
    else I2Cread1(MPU9250_ADDRESS,0x3B,14,Buf[i]);
     
    // Create 16 bits values from 8 bits data
    // Accelerometer
    a_x[i]=-(Buf[i][0]<<8 | Buf[i][1]);
    a_y[i]=-(Buf[i][2]<<8 | Buf[i][3]);
    a_z[i]=Buf[i][4]<<8 | Buf[i][5];
     
    // Gyroscope
    g_x[i]=-(Buf[i][8]<<8 | Buf[i][9]);
    g_y[i]=-(Buf[i][10]<<8 | Buf[i][11]);
    g_z[i]=Buf[i][12]<<8 | Buf[i][13];
  }

  uint8_t Mag[7]; 
  // _____________________
  // ::: Magnetometer :::
  // Read register Status 1 and wait for the DRDY: Data Ready
  uint8_t ST1;
  do
  {
    I2Cread(MAG_ADDRESS,0x02,1,&ST1);
  }
  while (!(ST1&0x01));
   
  // Read magnetometer data 
  I2Cread(MAG_ADDRESS,0x03,7,Mag);
  
  // Create 16 bits values from 8 bits data
   
  // Magnetometer
  m_x=-(Mag[3]<<8 | Mag[2]);
  m_y=-(Mag[1]<<8 | Mag[0]);
  m_z=-(Mag[5]<<8 | Mag[4]);  
}

void filterRawData()
{
  //scale raw data before filtering
  float accScale = 32768.0;
  float gyrScale = 16.4;
  for(int i = 0; i<2; i++)
  {
    //Scale accel and gyro values
    sA_x[i] = 2 * a_x[i] / accScale;
    sA_y[i] = 2 * a_y[i] / accScale;
    sA_z[i] = 2 * a_z[i] / accScale;

    sG_x[i] = g_x[i] / gyrScale;
    sG_y[i] = g_y[i] / gyrScale;
    sG_z[i] = g_z[i] / gyrScale;
  
    //LPF Accelerometer raw values
    fA_x[i] = fA_xOld[i] + alphaAcc * (sA_x[i] - fA_xOld[i]);
    fA_y[i] = fA_yOld[i] + alphaAcc * (sA_y[i] - fA_yOld[i]);
    fA_z[i] = fA_zOld[i] + alphaAcc * (sA_z[i] - fA_zOld[i]);
    
    fA_xOld[i] = fA_x[i];
    fA_yOld[i] = fA_y[i];
    fA_zOld[i] = fA_z[i];

    //if(i==1) fA_x[1] += 0.2;
    //if(i==1) fA_z[1] += 0.2;
         
//  //LPF Gyroscope raw values
//  fG_x[i] = fG_xOld[i] + alphaGyr * (sG_x[i] - fG_xOld[i]);
//  fG_y[i] = fG_yOld[i] + alphaGyr * (sG_y[i] - fG_yOld[i]);
//  fG_z[i] = fG_zOld[i] + alphaGyr * (sG_z[i] - fG_zOld[i]);
//
//  fG_xOld[i] = fG_x[i];
//  fG_yOld[i] = fG_y[i];
//  fG_zOld[i] = fG_z[i];  
  }
}

void computeRPY()
{
  // complementary filter to compute roll and pitch
  // both roll and pitch are converted to radians to be used with magnetometer data for tilt compensated heading.
  const float a = 0;
  for(int i = 0; i<2; i++)
  {
    Roll[i] += (sG_x[i] * 0.01);
    Pitch[i] += (sG_y[i] * 0.01);
    Roll[i] = (a * Roll[i]) + (1-a) * (atan2(fA_y[i], sqrt(sq(fA_x[i]) + sq(fA_z[i])))) * 180 / PI;
    Pitch[i] = (a * Pitch[i]) + (1-a) * (atan2(fA_x[i], sqrt(sq(fA_y[i]) + sq(fA_z[i])))) * 180 / PI;
    Roll[i] *= (PI/180);
    Pitch[i] *= (PI/180);  
  }
  Serial.println();
  
  //this part is required to normalize the magnetic vector
  if (m_x > m_xMax) {m_xMax = m_x;}
  if (m_y > m_yMax) {m_yMax = m_y;}
  if (m_z > m_zMax) {m_zMax = m_z;}
 
  if (m_x < m_xMin) {m_xMin = m_x;}
  if (m_y < m_yMin) {m_yMin = m_y;}
  if (m_z < m_zMin) {m_zMin = m_z;}
 
  float norm;
 
  m_xMap = float(map(m_x, m_xMin, m_xMax, -10000, 10000))/10000.0;
  m_yMap = float(map(m_y, m_yMin, m_yMax, -10000, 10000))/10000.0;
  m_zMap = float(map(m_z, m_zMin, m_zMax, -10000, 10000))/10000.0;
 

  //normalize the magnetic vector
  norm= sqrt( sq(m_xMap) + sq(m_yMap) + sq(m_zMap));
  m_xMap /=norm;
  m_yMap /=norm;
  m_zMap /=norm;
 
  Yaw=atan2((-m_yMap*cos(Roll[0]) + m_zMap*sin(Roll[0]) ) , m_xMap*cos(Pitch[0]) + m_yMap*sin(Pitch[0])*sin(Roll[0])+ m_zMap*sin(Pitch[0])*cos(Roll[0])) *180/PI;
  YawU=atan2(-m_yMap, m_xMap) *180/PI;
  if (Yaw < 0) Yaw += 360;
  Roll[0] = Roll[0] * 180 / PI;
  Pitch[0] = Pitch[0] * 180 / PI;
  Roll[1] = Roll[1] * 180 / PI;
  Pitch[1] = Pitch[1] * 180 / PI;
}

void unwrap()
{
  
}

