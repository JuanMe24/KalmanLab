/**
 * K-SENSE V2 - FIRMWARE FINAL (Corregido: Variable zeroYaw)
 * R=0.8 | Q=0.02 | GyroSign=1 (Positivo)
 */
#include <Arduino.h>
#include <Wire.h>

// ==========================================
// 1. CLASE KALMAN
// ==========================================
class Kalman {
public:
    float Q_angle = 0.02f;   
    float Q_bias = 0.003f;
    float R_measure = 0.8f; 

    float angle = 0.0f;
    float bias = 0.0f;
    float P[2][2];

    Kalman() { 
        P[0][0]=0; P[0][1]=0; P[1][0]=0; P[1][1]=0; 
    }

    float getAngle(float newAngle, float newRate, float dt) {
        float rate = newRate - bias;
        angle += dt * rate;
        
        P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
        P[0][1] -= dt * P[1][1];
        P[1][0] -= dt * P[1][1];
        P[1][1] += Q_bias * dt;

        float S = P[0][0] + R_measure;
        float K[2];
        K[0] = P[0][0] / S;
        K[1] = P[1][0] / S;

        float y = newAngle - angle;
        angle += K[0] * y;
        bias += K[1] * y;

        float P00_temp = P[0][0];
        float P01_temp = P[0][1];

        P[0][0] -= K[0] * P00_temp;
        P[0][1] -= K[0] * P01_temp;
        P[1][0] -= K[1] * P00_temp;
        P[1][1] -= K[1] * P01_temp;

        return angle;
    };
};

// ==========================================
// 2. SENSOR BMI160
// ==========================================
#define BMI160_ADDR 0x69 

class BMI160_Integrated {
  public:
    float gx_off = 0, gy_off = 0, gz_off = 0;

    void begin() {
      Wire.beginTransmission(BMI160_ADDR);
      if (Wire.endTransmission() != 0) {
        Serial.println("❌ ERROR: Sensor no encontrado."); 
        while(1); 
      }
      writeReg(0x7E, 0xB6); delay(100); 
      writeReg(0x7E, 0x11); delay(100); 
      writeReg(0x7E, 0x15); delay(100); 
      writeReg(0x41, 0x03); 
      writeReg(0x43, 0x03); 
    }

    void calibrate() {
      Serial.println("⚠️ CALIBRANDO GYRO... QUIETO (3s)");
      long x=0, y=0, z=0;
      for(int i=0; i<2000; i++) {
        int16_t _gx, _gy, _gz, _ax, _ay, _az;
        readRaw(_gx, _gy, _gz, _ax, _ay, _az);
        x+=_gx; y+=_gy; z+=_gz; delay(2);
      }
      gx_off=(x/2000.0)/131.2; 
      gy_off=(y/2000.0)/131.2; 
      gz_off=(z/2000.0)/131.2;
      Serial.println("✅ Gyro Calibrado.");
    }

    void readAll(float &gx, float &gy, float &gz, float &ax, float &ay, float &az) {
      int16_t _gx, _gy, _gz, _ax, _ay, _az;
      readRaw(_gx, _gy, _gz, _ax, _ay, _az);
      
      gx = (_gx/131.2) - gx_off;
      gy = (_gy/131.2) - gy_off; 
      gz = (_gz/131.2) - gz_off;
      
      ax = _ax/16384.0; 
      ay = _ay/16384.0; 
      az = _az/16384.0;
    }

  private:
    void writeReg(uint8_t r, uint8_t v) { Wire.beginTransmission(BMI160_ADDR); Wire.write(r); Wire.write(v); Wire.endTransmission(); }
    void readRaw(int16_t &gx, int16_t &gy, int16_t &gz, int16_t &ax, int16_t &ay, int16_t &az) {
      Wire.beginTransmission(BMI160_ADDR); Wire.write(0x0C); Wire.endTransmission(false); Wire.requestFrom(BMI160_ADDR, 12);
      if(Wire.available()==12) {
        gx=Wire.read()|(Wire.read()<<8); gy=Wire.read()|(Wire.read()<<8); gz=Wire.read()|(Wire.read()<<8);
        ax=Wire.read()|(Wire.read()<<8); ay=Wire.read()|(Wire.read()<<8); az=Wire.read()|(Wire.read()<<8);
      }
    }
};

// ==========================================
// 3. VARIABLES Y SETUP
// ==========================================
BMI160_Integrated sensor;
Kalman kalmanX, kalmanY;
long lastTime;
float accPitch, accRoll;
float gyroPitch=0, gyroRoll=0;
float kPitch, kRoll;
float yaw = 0; 

// AQUI FALTABA AGREGAR zeroYaw
float zeroPitch = 0, zeroRoll = 0, zeroYaw = 0; 

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22); 
  Wire.setClock(100000); 
  
  sensor.begin(); 
  delay(500); 
  sensor.calibrate(); 
  
  // TARA PROMEDIADA
  Serial.println("⚖️ HACIENDO TARA (Auto-Zero)...");
  float sumPitch = 0;
  float sumRoll = 0;
  int muestrasTara = 100;

  for(int i=0; i<muestrasTara; i++) {
    float gx, gy, gz, ax, ay, az;
    sensor.readAll(gx, gy, gz, ax, ay, az);
    sumPitch += atan2(ay, az) * RAD_TO_DEG;
    sumRoll  += atan2(-ax, sqrt(ay*ay + az*az)) * RAD_TO_DEG;
    delay(5);
  }
  
  float startPitch = sumPitch / muestrasTara;
  float startRoll  = sumRoll / muestrasTara;
  
  kalmanX.angle = startPitch;
  kalmanY.angle = startRoll;
  gyroPitch = startPitch; 
  gyroRoll = startRoll;
  
  zeroPitch = startPitch;
  zeroRoll  = startRoll;
  zeroYaw   = 0;
  
  Serial.println("✨ ¡Listo! Iniciando...");
  lastTime = micros();
}

void loop() {
  long now = micros();
  float dt = (now - lastTime) / 1000000.0;
  lastTime = now;

  float gx, gy, gz, ax, ay, az;
  sensor.readAll(gx, gy, gz, ax, ay, az);

  accPitch = atan2(ay, az) * RAD_TO_DEG;
  accRoll  = atan2(-ax, sqrt(ay*ay + az*az)) * RAD_TO_DEG;

  float gyroX_rate = gx; 
  float gyroY_rate = gy;
  float gyroZ_rate = gz;

  gyroPitch += gyroX_rate * dt;
  gyroRoll  += gyroY_rate * dt;
  yaw       += gyroZ_rate * dt;
  
  kPitch = kalmanX.getAngle(accPitch, gyroX_rate, dt);
  kRoll  = kalmanY.getAngle(accRoll, gyroY_rate, dt);

  Serial.print(accPitch - zeroPitch); Serial.print(",");
  Serial.print(accRoll - zeroRoll);  Serial.print(",");
  Serial.print(gyroPitch - zeroPitch); Serial.print(",");
  Serial.print(gyroRoll - zeroRoll);  Serial.print(",");
  Serial.print(kPitch - zeroPitch); Serial.print(",");
  Serial.print(kRoll - zeroRoll); Serial.print(","); 
  Serial.println(yaw);

  delay(10); 
}