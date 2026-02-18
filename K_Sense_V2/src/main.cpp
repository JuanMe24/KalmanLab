/**
 * K-SENSE V2 - FIRMWARE FINAL
 * R=0.8 | Q=0.02 | GyroSign=1 (Positivo)
 */
#include <Arduino.h>
#include <Wire.h>

// ==========================================
// 1. CLASE KALMAN (El cerebro del filtro)
// ==========================================
class Kalman {
public:
    // TUS VALORES DE SINTONIZACIÓN
    float Q_angle = 0.08f;   // Rapidez de respuesta
    float Q_bias = 0.003f;   // Deriva del giroscopio
    float R_measure = 0.55f;  // Suavidad (confianza en el giroscopio)

    float angle = 0.0f; // El ángulo calculado
    float bias = 0.2f;  // El error calculado del giroscopio
    float P[2][2];      // Matriz de covarianza (Error estimado)

    // Constructor: Inicia la matriz en 0
    Kalman() { 
        P[0][0]=0; P[0][1]=0; P[1][0]=0; P[1][1]=0; 
    }

    // La función mágica que fusiona los datos
    float getAngle(float newAngle, float newRate, float dt) {
        // PASO 1: PREDICCIÓN (Matemática del Giroscopio)
        float rate = newRate - bias;
        angle += dt * rate;
        
        // Actualizamos la matriz de error (Predicción)
        P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
        P[0][1] -= dt * P[1][1];
        P[1][0] -= dt * P[1][1];
        P[1][1] += Q_bias * dt;

        // PASO 2: CORRECCIÓN (Usando el Acelerómetro)
        float S = P[0][0] + R_measure; // Estimación del error total
        
        // Calculamos la Ganancia de Kalman (K)
        float K[2];
        K[0] = P[0][0] / S;
        K[1] = P[1][0] / S;

        // "y" es la diferencia entre el Acelerómetro y nuestra Predicción
        float y = newAngle - angle;
        
        // Corregimos el ángulo y el bias
        angle += K[0] * y;
        bias += K[1] * y;

        // Actualizamos la matriz de error final
        float P00_temp = P[0][0];
        float P01_temp = P[0][1];

        P[0][0] -= K[0] * P00_temp;
        P[0][1] -= K[0] * P01_temp;
        P[1][0] -= K[1] * P00_temp;
        P[1][1] -= K[1] * P01_temp;

        return angle; // Devolvemos el ángulo "perfecto"
    };
};

// ==========================================
// 2. SENSOR BMI160 (Driver manual)
// ==========================================
#define BMI160_ADDR 0x69 // Dirección I2C del sensor

class BMI160_Integrated {
  public:
    float gx_off = 0, gy_off = 0, gz_off = 0; // Variables para guardar el error inicial

    // Configuración inicial del sensor
    void begin() {
      Wire.beginTransmission(BMI160_ADDR);
      if (Wire.endTransmission() != 0) {
        Serial.println("ERROR: Sensor no encontrado."); 
        while(1); // Se queda aquí para siempre si falla
      }
      // Comandos hexadecimales para configurar el chip
      writeReg(0x7E, 0xB6); delay(100); // Reset
      writeReg(0x7E, 0x11); delay(100); // Encender Acelerómetro
      writeReg(0x7E, 0x15); delay(100); // Encender Giroscopio
      writeReg(0x41, 0x03); // Rango Accel +-2G
      writeReg(0x43, 0x03); // Rango Gyro 250 grados/s
    }

    // Calcula el error del giroscopio al inicio
    void calibrate() {
      Serial.println("CALIBRANDO GYRO... QUIETO (3s)");
      long x=0, y=0, z=0;
      // Toma 2000 muestras y las suma
      for(int i=0; i<2000; i++) {
        int16_t _gx, _gy, _gz, _ax, _ay, _az;
        readRaw(_gx, _gy, _gz, _ax, _ay, _az);
        x+=_gx; y+=_gy; z+=_gz; delay(2);
      }
      // Saca el promedio y lo guarda como offset
      gx_off=(x/2000.0)/131.2; 
      gy_off=(y/2000.0)/131.2; 
      gz_off=(z/2000.0)/131.2;
      Serial.println("Gyro Calibrado.");
    }

    // Lee los datos y los convierte a valores reales
    void readAll(float &gx, float &gy, float &gz, float &ax, float &ay, float &az) {
      int16_t _gx, _gy, _gz, _ax, _ay, _az;
      readRaw(_gx, _gy, _gz, _ax, _ay, _az);
      
      // Convierte bits a grados/segundo y resta el error
      gx = (_gx/131.2) - gx_off;
      gy = (_gy/131.2) - gy_off; 
      gz = (_gz/131.2) - gz_off;
      
      // Convierte bits a fuerzas G
      ax = _ax/16384.0; 
      ay = _ay/16384.0; 
      az = _az/16384.0;
    }

  private:
    // Función auxiliar para escribir en el chip
    void writeReg(uint8_t r, uint8_t v) { Wire.beginTransmission(BMI160_ADDR); Wire.write(r); Wire.write(v); Wire.endTransmission(); }
    
    // Función auxiliar para leer los bits crudos
    void readRaw(int16_t &gx, int16_t &gy, int16_t &gz, int16_t &ax, int16_t &ay, int16_t &az) {
      Wire.beginTransmission(BMI160_ADDR); Wire.write(0x0C); Wire.endTransmission(false); Wire.requestFrom(BMI160_ADDR, 12);
      if(Wire.available()==12) {
        // Operaciones bitwise para unir 2 bytes en 1 numero de 16 bits
        gx=Wire.read()|(Wire.read()<<8); gy=Wire.read()|(Wire.read()<<8); gz=Wire.read()|(Wire.read()<<8);
        ax=Wire.read()|(Wire.read()<<8); ay=Wire.read()|(Wire.read()<<8); az=Wire.read()|(Wire.read()<<8);
      }
    }
};

// ==========================================
// 3. VARIABLES Y SETUP
// ==========================================
BMI160_Integrated sensor; // Objeto del sensor
Kalman kalmanX, kalmanY;  // Objetos del filtro (uno para X, otro para Y)
long lastTime;            // Para medir el tiempo (dt)
float accPitch, accRoll;  // Ángulos del acelerómetro
float gyroPitch=0, gyroRoll=0; // Ángulos del giroscopio
float kPitch, kRoll;      // Ángulos finales (Kalman)
float yaw = 0;            // Ángulo de giro (Z)

// Variables para guardar la posición inicial (Tara)
float zeroPitch = 0, zeroRoll = 0, zeroYaw = 0; 

void setup() {
  Serial.begin(115200);     // Iniciar comunicación serial
  Wire.begin(21, 22);       // Iniciar I2C en pines 21 y 22
  Wire.setClock(100000);    // Velocidad de comunicación segura
  
  sensor.begin();           // Configurar sensor
  delay(500); 
  sensor.calibrate();       // Calcular offsets internos del giroscopio
  
  // === TARA PROMEDIADA (Auto-Zero) ===
  Serial.println("HACIENDO TARA (Auto-Zero)...");
  float sumPitch = 0;
  float sumRoll = 0;
  int muestrasTara = 100;

  // Tomamos 100 fotos de la posición inicial
  for(int i=0; i<muestrasTara; i++) {
    float gx, gy, gz, ax, ay, az;
    sensor.readAll(gx, gy, gz, ax, ay, az);
    // Trigonometría para saber el ángulo
    sumPitch += atan2(ay, az) * RAD_TO_DEG;
    sumRoll  += atan2(-ax, sqrt(ay*ay + az*az)) * RAD_TO_DEG;
    delay(5);
  }
  
  // Promediamos para saber dónde es "cero" visualmente
  float startPitch = sumPitch / muestrasTara;
  float startRoll  = sumRoll / muestrasTara;
  
  // Le decimos al filtro que empiece ahí
  kalmanX.angle = startPitch;
  kalmanY.angle = startRoll;
  gyroPitch = startPitch; 
  gyroRoll = startRoll;
  
  // Guardamos ese valor para restarlo después
  zeroPitch = startPitch;
  zeroRoll  = startRoll;
  zeroYaw   = 0;
  
  Serial.println("Listo! Iniciando...");
  lastTime = micros(); // Iniciar cronómetro
}

void loop() {
  // 1. Calcular Delta Tiempo (dt)
  // Cuánto tiempo pasó desde la última vuelta (en segundos)
  long now = micros();
  float dt = (now - lastTime) / 1000000.0;
  lastTime = now;

  // 2. Leer datos frescos del sensor
  float gx, gy, gz, ax, ay, az;
  sensor.readAll(gx, gy, gz, ax, ay, az);

  // 3. Calcular Ángulos del Acelerómetro (Trigonometría)
  accPitch = atan2(ay, az) * RAD_TO_DEG;
  accRoll  = atan2(-ax, sqrt(ay*ay + az*az)) * RAD_TO_DEG;

  // 4. Preparar datos del Giroscopio
  float gyroX_rate = gx; 
  float gyroY_rate = gy; 
  float gyroZ_rate = gz;

  // 5. Integrar Giroscopio (Solo para comparar visualmente)
  gyroPitch += gyroX_rate * dt; // Angulo = Velocidad * Tiempo
  gyroRoll  += gyroY_rate * dt;
  yaw       += gyroZ_rate * dt;
  
  // 6. EJECUTAR FILTRO KALMAN (La Fusión)
  // Entra: Angulo Ruidoso (Accel) + Velocidad (Gyro) + Tiempo
  // Sale: Angulo Perfecto
  kPitch = kalmanX.getAngle(accPitch, gyroX_rate, dt);
  kRoll  = kalmanY.getAngle(accRoll, gyroY_rate, dt);

  // 7. Enviar datos a Processing
  // Restamos 'zeroPitch' y 'zeroRoll' para que visualmente empiece en 0
  Serial.print(accPitch - zeroPitch); Serial.print(",");
  Serial.print(accRoll - zeroRoll);  Serial.print(",");
  Serial.print(gyroPitch - zeroPitch); Serial.print(",");
  Serial.print(gyroRoll - zeroRoll);  Serial.print(",");
  Serial.print(kPitch - zeroPitch); Serial.print(",");
  Serial.print(kRoll - zeroRoll); Serial.print(","); 
  Serial.println(yaw);

  delay(10); // Pausa pequeña para no saturar
}