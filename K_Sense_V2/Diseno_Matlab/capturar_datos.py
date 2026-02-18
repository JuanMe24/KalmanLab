import serial
import csv
import time
import sys

# --- CONFIGURACIÓN ---
PUERTO_COM = 'COM3'  # <--- ¡Verifica tu puerto!
BAUD_RATE = 115200
NOMBRE_ARCHIVO = 'datos_imu_6dof.csv'

def guardar_datos():
    try:
        ser = serial.Serial(PUERTO_COM, BAUD_RATE, timeout=1)
        print(f" Conectado a {PUERTO_COM}")
        time.sleep(2) 
    except serial.SerialException:
        print(f" ERROR: No se puede abrir {PUERTO_COM}.")
        sys.exit()

    print(f" Grabando 6-DOF en {NOMBRE_ARCHIVO}...")
    print(" Presiona Ctrl + C para detener.")

    with open(NOMBRE_ARCHIVO, mode='w', newline='') as file:
        writer = csv.writer(file)
        # Encabezado completo
        writer.writerow(['ax', 'ay', 'az', 'gx', 'gy', 'gz', 'dt'])

        try:
            while True:
                if ser.in_waiting > 0:
                    linea = ser.readline().decode('utf-8', errors='ignore').strip()
                    if ',' in linea:
                        datos = linea.split(',')
                        # Ahora esperamos 7 datos (6 ejes + tiempo)
                        if len(datos) == 7:
                            writer.writerow(datos)
                            # Imprimir solo los primeros 3 para no saturar la pantalla
                            print(f"Graba: ax={datos[0]}, ay={datos[1]} ...") 

        except KeyboardInterrupt:
            print("\n Grabación detenida.")
            ser.close()

if __name__ == "__main__":
    guardar_datos()