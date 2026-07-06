#include <Wire.h>
#include <math.h>

#define MPU_ADDR 0x68

void writeRegister(byte reg, byte value);

int16_t ax,ay,az;
int16_t gx,gy,gz;
int16_t tempRaw;

float accelBiasX, accelBiasY, accelBiasZ;
float gyroBiasX, gyroBiasY, gyroBiasZ;

float AxFilt, AyFilt, AzFilt;
float GxFilt, GyFilt, GzFilt;

float roll = 0, pitch = 0, yaw = 0;

unsigned long lastTime = 0;
float dt;

const float alphaAcc = 0.95;
const float alphaGyro = 0.85;

void readMPU()
{
    uint8_t buffer[14];

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);

    Wire.requestFrom(MPU_ADDR,14);

    for(int i=0;i<14;i++)
    {
        if(Wire.available())
            buffer[i] = Wire.read();
        else
            buffer[i] = 0;
    }

    ax=(buffer[0]<<8)|buffer[1];
    ay=(buffer[2]<<8)|buffer[3];
    az=(buffer[4]<<8)|buffer[5];

    tempRaw=(buffer[6]<<8)|buffer[7];

    gx=(buffer[8]<<8)|buffer[9];
    gy=(buffer[10]<<8)|buffer[11];
    gz=(buffer[12]<<8)|buffer[13];
}

void calibrateMPU()
{
    Serial.println();
    Serial.println("================================");
    Serial.println("MPU6050 Calibration");
    Serial.println("Keep the robot completely still.");
    Serial.println("Calibration starts in 3 seconds...");
    Serial.println("================================");

    delay(3000);

    long axSum = 0;
    long aySum = 0;
    long azSum = 0;

    long gxSum = 0;
    long gySum = 0;
    long gzSum = 0;

    const int samples = 1000;

    for(int i = 0; i < samples; i++)
    {
        readMPU();

        axSum += ax;
        aySum += ay;
        azSum += az;

        gxSum += gx;
        gySum += gy;
        gzSum += gz;

        delay(2);
    }

    accelBiasX = (float)axSum / samples;
    accelBiasY = (float)aySum / samples;
    accelBiasZ = (float)azSum / samples - 16384.0;

    gyroBiasX = (float)gxSum / samples;
    gyroBiasY = (float)gySum / samples;
    gyroBiasZ = (float)gzSum / samples;

    Serial.println();
    Serial.println("Calibration Complete");

    Serial.print("Accel Bias X : ");
    Serial.println(accelBiasX);

    Serial.print("Accel Bias Y : ");
    Serial.println(accelBiasY);

    Serial.print("Accel Bias Z : ");
    Serial.println(accelBiasZ);

    Serial.print("Gyro Bias X : ");
    Serial.println(gyroBiasX);

    Serial.print("Gyro Bias Y : ");
    Serial.println(gyroBiasY);

    Serial.print("Gyro Bias Z : ");
    Serial.println(gyroBiasZ);

    Serial.println();
}

void setup()
{
    Serial.begin(115200);

    Wire.setSDA(PB9);
    Wire.setSCL(PB8);

    Wire.begin();
    Wire.setClock(400000);

    // Wake up
    writeRegister(0x6B, 0x00);

    // Sample Rate = 1000Hz
    writeRegister(0x19,0x07);

    // DLPF = 44Hz
    writeRegister(0x1A,0x03);

    // Gyro ±500
    writeRegister(0x1B,0x08);

    // Accel ±2g
    writeRegister(0x1C,0x00);
    
    delay(1000);
    Serial.println("Configuration Complete");
    delay(1000);
    calibrateMPU();

    lastTime = micros();
}

void loop()
{
    unsigned long now = micros();
    dt = (now - lastTime) / 1000000.0;
    lastTime = now;

    if(dt <= 0 || dt > 0.05)
        dt = 0.01;

    readMPU();

    float Ax = (ax - accelBiasX) / 16384.0;
    float Ay = (ay - accelBiasY) / 16384.0;
    float Az = (az - accelBiasZ) / 16384.0;

    float Gx = (gx - gyroBiasX) / 65.5;
    float Gy = (gy - gyroBiasY) / 65.5;
    float Gz = (gz - gyroBiasZ) / 65.5;

    // filtering
    AxFilt = alphaAcc * AxFilt + (1 - alphaAcc) * Ax;
    AyFilt = alphaAcc * AyFilt + (1 - alphaAcc) * Ay;
    AzFilt = alphaAcc * AzFilt + (1 - alphaAcc) * Az;

    GxFilt = alphaGyro * GxFilt + (1 - alphaGyro) * Gx;
    GyFilt = alphaGyro * GyFilt + (1 - alphaGyro) * Gy;
    GzFilt = alphaGyro * GzFilt + (1 - alphaGyro) * Gz;

    // angles
    float rollAcc = atan2(AyFilt, AzFilt) * 180.0 / PI;
    float pitchAcc = atan2(-AxFilt, sqrt(AyFilt*AyFilt + AzFilt*AzFilt)) * 180.0 / PI;

    roll  = 0.98 * (roll + GxFilt * dt) + 0.02 * rollAcc;
    pitch = 0.98 * (pitch + GyFilt * dt) + 0.02 * pitchAcc;

    yaw += GzFilt * dt;

    // normalize yaw
    if(yaw > 180) yaw -= 360;
    if(yaw < -180) yaw += 360;

    Serial.print(AxFilt, 3);
    Serial.print(" ");
    Serial.print(AyFilt, 3);
    Serial.print(" ");
    Serial.print(AzFilt, 3);

    Serial.print(" | ");

    Serial.print(GxFilt, 2);
    Serial.print(" ");
    Serial.print(GyFilt, 2);
    Serial.print(" ");
    Serial.print(GzFilt, 2);

    Serial.print(" | ");

    Serial.print("Roll : ");
    Serial.print(roll);

    Serial.print("   Pitch : ");
    Serial.print(pitch);

    Serial.print("   Yaw : ");
    Serial.println(yaw);

    delay(10);
}



 


void writeRegister(byte reg, byte value)
{
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

int16_t readWord(byte reg)
{
    Wire.beginTransmission(MPU_ADDR);

    Wire.write(reg);

    Wire.endTransmission(false);

    Wire.requestFrom(MPU_ADDR,2);

    byte high = Wire.read();
    byte low = Wire.read();

    return (high<<8)|low;
}