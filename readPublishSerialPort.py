import serial
import pandas as pd
from pathlib import Path
import paho.mqtt.client as mqtt
import json 

### Configuración para Thingsboard ###
broker = "iot.eie.ucr.ac.cr"
puerto_TB = 1883
topic = "v1/devices/me/telemetry"
dispositivo = "Sismografo-Johander-Alexander"
contraseña = "b2qzmr40pgm7jnozydk8"

# Funciones utilizadas por el cliente
def conexion(client, userdata, flags, rc):
    if rc == 0:
        print("Cliente conectado")
        client.connected = True
    else: 
        print("Error de conexión: ", rc)
        client.loop_stop()


def desconexion(client, userdata, rc):
	client.connected = False
	if (rc == 0):
		print("El cliente se desconectó")
	else:
 		print("Ocurrió un error y el cliente se desconectó")


# Instanciamiento y configuración del cliente
cliente = mqtt.Client(dispositivo)
cliente.connected = False
cliente.on_connect=conexion
cliente.on_disconnect=desconexion

# Conexión con el broker
cliente.username_pw_set(contraseña)
cliente.connect(broker, puerto_TB)
cliente.loop_start()

# Nombre del puerto tty
nombre_puerto = '/dev/ttyACM0'

# Nombre del archivo y ruta donde se guardarán los resultados
nombre_archivo = 'resultados'
ruta_resultados = './' + nombre_archivo + '/'
Path(ruta_resultados).mkdir(parents=True, exist_ok=True)

# Definición del DataFrame para guardar los resultados
formato = { 'X':[0], 'Y':[0], 'Z':[0],'Porcentaje de batería':[0], 'Batería baja':[0] }
resultados_df = pd.DataFrame(formato)

lectura = [] # Lista donde se guarda el número leído
escritura = [] # Lista donde se guardarán los 4 valores leídos

puerto = serial.Serial(nombre_puerto, baudrate= 115200)

while True:
	caracter_leido = puerto.read()
	caracter_leido = caracter_leido.decode('utf-8')
	lectura.append(caracter_leido)

	if caracter_leido == '.':
		escritura.append(float(''.join(lectura)))
		lectura.clear()

	if len(escritura) == 4:
		caracter_leido = puerto.read()
		caracter_leido = caracter_leido.decode('utf-8')
		if caracter_leido == '1':
			escritura.append('SI')
		else:
			escritura.append('NO')
		puerto.read()
		resultados_df.loc[len(resultados_df.index)] = escritura

		# Guardar resultados en formato CSV
		resultados_df.to_csv(ruta_resultados + nombre_archivo + '.csv', encoding='utf-8', index=False)
	
		# Enviar al dashboard
		diccionario = {
  			"Eje X": escritura[0],
  			"Eje Y": escritura[1],
  			"Eje Z": escritura[2],
  			"Nivel de bateria": escritura[3],
  			"Bateria baja": escritura[4] 
		}
		escritura.clear()
		cliente.publish(topic, json.dumps(diccionario, indent = 4))
		cliente.loop()