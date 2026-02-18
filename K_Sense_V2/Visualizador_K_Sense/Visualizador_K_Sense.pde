import processing.serial.*;

Serial myPort;
// 7 DATOS: [accP, accR, gyrP, gyrR, kalP, kalR, YAW]
float[] data = new float[7]; 
ArrayList<Float> rawHistory = new ArrayList<Float>();
ArrayList<Float> kalmanHistory = new ArrayList<Float>();
int maxPoints = 800; 

void setup() {
  size(1280, 720, P3D);
  surface.setTitle("K-Sense V2 - Vista Ingeniería");
  
  // Vista Ortográfica (Sin deformación, líneas paralelas)
  ortho(-width/2, width/2, -height/2, height/2); 
  
  // Posición de cámara estándar (Mirando directo al centro)
  camera(width/2.0, height/2.0, (height/2.0) / tan(PI*30.0 / 180.0), width/2.0, height/2.0, 0, 0, 1, 0);

  try {
    String portName = Serial.list()[0]; 
    myPort = new Serial(this, portName, 115200);
    myPort.bufferUntil('\n');
  } catch (Exception e) {}
  
  for(int i=0; i<maxPoints; i++) { 
    rawHistory.add(0.0); 
    kalmanHistory.add(0.0); 
  }
  textSize(16);
}

void draw() {
  background(30); 
  
  if (myPort == null) { 
    textAlign(CENTER); fill(255, 50, 50); 
    text("CONECTANDO...", width/2, height/2); 
    return; 
  }
  
  // Luces frontales fuertes para que se vea bien la cara plana
  ambientLight(120, 120, 120);
  directionalLight(255, 255, 255, 0, 0, -1);

  float currentYaw = data[6];

  // === DIBUJAR LOS 3 CUBOS ===
  
  // 1. Acelerómetro (ROJO) - Yaw Fijo en 0
  drawScene(width/6 + 30, height/3, data[0], data[1], 0, color(255, 80, 80), "ACELERÓMETRO");
  
  // 2. Giroscopio (VERDE) - Yaw Real
  drawScene(width/2, height/3, data[2], data[3], currentYaw, color(80, 255, 80), "GIROSCOPIO");
  
  // 3. Filtro Kalman (AZUL) - Yaw Real
  drawScene(5*width/6 - 30, height/3, data[4], data[5], currentYaw, color(80, 150, 255), "FILTRO KALMAN");

  drawGraph();
}

void drawScene(int x, int y, float pitch, float roll, float yaw, int c, String label) {
  pushMatrix();
  translate(x, y);
  
  // --- CAMBIO DE ORIENTACIÓN ---
  
  rotateY(radians(yaw));      // Yaw (Giro en mesa): Suele estar bien con menos
  rotateX(radians(-pitch));    // Pitch: INVERTIDO (Antes era +, ahora -)
  rotateZ(radians(roll));      // Roll: INVERTIDO (Antes era -, ahora +)
  
  // Texto (Plano)
  fill(255); textAlign(CENTER); text(label, 0, -110);
  fill(200); text("P:" + int(pitch) + " R:" + int(roll), 0, 130);
  
  // Dibujar cubo
  stroke(255); strokeWeight(2); fill(c); box(100);
  
  // Flechita para saber cuál es el "Frente" del cubo virtual
  // (Ahora apunta hacia ti)
  pushMatrix(); translate(0, 0, 51); noStroke(); fill(255); box(20, 20, 5); popMatrix();
  
  popMatrix();
}

void drawGraph() {
  noLights(); 
  int x=50, y=height-220, w=width-100, h=200;
  
  fill(0, 100); stroke(100); rect(x, y, w, h);
  stroke(150); line(x, y+h/2, x+w, y+h/2);
  
  textSize(14);
  fill(255); textAlign(LEFT); text("ESTABILIDAD PITCH (Eje X):", x, y - 10);
  fill(255, 80, 80);  text("█ Acelerómetro", x + 200, y - 10);
  fill(80, 150, 255); text("█ Kalman (Azul)", x + 320, y - 10);

  noFill(); float sY = 2.0;
  
  stroke(255, 80, 80); strokeWeight(1); beginShape(); 
  for(int i=0; i<rawHistory.size(); i++) 
    vertex(map(i,0,maxPoints,x,x+w), constrain(y+h/2-(rawHistory.get(i)*sY), y, y+h));
  endShape();
  
  stroke(80, 150, 255); strokeWeight(2); beginShape(); 
  for(int i=0; i<kalmanHistory.size(); i++) 
    vertex(map(i,0,maxPoints,x,x+w), constrain(y+h/2-(kalmanHistory.get(i)*sY), y, y+h));
  endShape();
}

void serialEvent(Serial p) {
  try {
    String s = p.readStringUntil('\n');
    if (s != null) {
      float[] vals = float(split(trim(s), ','));
      if (vals.length == 7) {
        data = vals;
        rawHistory.add(data[0]); 
        kalmanHistory.add(data[4]);
        if (rawHistory.size() > maxPoints) { 
          rawHistory.remove(0); 
          kalmanHistory.remove(0); 
        }
      }
    }
  } catch (Exception e) {}
}
