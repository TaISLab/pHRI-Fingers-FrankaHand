import rospy
from std_msgs.msg import Float32MultiArray
import serial

class SerialPublisher:
    def __init__(self):
        rospy.init_node('angle_publisher', anonymous=True) #Inicializar nodo con nombre angle_publisher
        self.publisher = rospy.Publisher('angle_readings', Float32MultiArray, queue_size=10)  #Crear publicador para el tópico 'angle_readings' 
        self.serial_port = serial.Serial('/dev/ttyACM0', 250000)  #Configurar conexión serial con el Arduino
        self.rate = rospy.Rate(100) #Frecuencia de publicación

    #Función para publicar el topic
    def publish_serial_data(self):
        while not rospy.is_shutdown():  # Bucle principal que se ejecuta hasta que ROS se apague
            try:
                data = self.serial_port.readline().decode('utf-8').strip() # Leer la lína del puerto serial
                pot1, pot2 = map(float, data.split(',')) # Dividir la línea en dos valores flotantes separados por coma
                msg = Float32MultiArray() # Crear el mensaje de ROS
                msg.data = [pot1, pot2]  # Asignar los valores leídos al campo 'data' del mensaje
                self.publisher.publish(msg) # Publicar el mensaje en el tópico 'angle_readings'

            except Exception as e:
                rospy.logerr("Error reading from serial port: %s", e)
            self.rate.sleep()# Mantiene la frecuencia de publicación a 100 Hz

def main():
    try:
        publisher = SerialPublisher() # Crear un objeto de la clase SerialPublisher
        publisher.publish_serial_data()# Llamar al método para publicar los mensajes

    except rospy.ROSInterruptException:
        # Captura la excepción si ROS se interrumpe
        pass

if __name__ == '__main__':
    main() #Ejecutar función principal al llamar al script
