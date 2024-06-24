// Este código parte del siguiente repositorio
// https://github.com/TaISLab/dummyarm
//

// Código para la lectura de los potenciómetros y mapeo de estos valores a ángulos
// de posición de la garra.

//

#include <ros.h>                           // Incluye la biblioteca de ROS para Arduino
#include <std_msgs/Float32MultiArray.h>    // Incluye el tipo de mensaje Float32MultiArray de ROS

// Declaración de constantes y variables:
#define periodo 10   // Constante para definir el periodo de muestreo en ms.
#define POT 2        // Constante para definir el número de potenciómetros.

long nextt, t;       // Variables para el control del tiempo.

int input[POT];      // Variable para almacenar las lecturas de los sensores (lecturas del ADC).
double angle[POT];   // Variable para almacenar el ángulo de posición.


// Establecer los límites para la conversión:
int    inputmin[POT] = {   0,    0};  // Valor mínimo de lectura del ADC.
double anglemin[POT] = {   0,    0};  // Valor mínimo de ángulo.
int    inputmax[POT] = {1023, 1023};  // Valor máximo de lectura del ADC.
double anglemax[POT] = {  90,   90};  // Valor máximo de ángulo.


// Configuración de ROS
ros::NodeHandle nh;                              // Crear un nodo Ros Master para manejar la comunicación con ROS
std_msgs::Float32MultiArray pot_msg;             // Crear mensaje de tipo Float32MultiArray para enviar datos
ros::Publisher pub("angle_readings", &pot_msg);  // Declara un publicador con el nombre del topic


// Configuración inicial:
void setup() {
  Serial.begin(250000);   // Inicializar comunicación serial a 250000 bps 
  nh.initNode();          // Inicializar el nodo 
  nh.advertise(pub);      // Inicializar el publicador del topic
  nextt=millis();         // Configurar temporizador inicial: tiempo en el que la placa comienza a ejecutar el programa, tras inicilizar el entorno de ejecución.
}


// Función para mapear el rango de valores de entrada en un rango de valores de salida
double cv2Degree(double input, double inputmin, double inputmax, double outputmin, double outputmax)
{
  return outputmin+((input-inputmin)/(inputmax-inputmin))*(outputmax-outputmin); 
}


// Bucle principal
void loop() {
  // Espera activa en lugar de delay para evitar detener la ejecución del código
  while ((t=millis())<nextt);   
  nextt+=periodo;   // Incrementa nextt para la siguiente iteración      
  

  // Lecturas analógicas del valor de los potenciómetros:
  input[0] = analogRead(A1);
  input[1] = analogRead(A2);


  //Bucle para iterar sobre cada potenciómetro y obtener el valor del ángulo de posición de cada uno:
  int n;
  for (n=0; n<POT; n++){
     angle[n] = cv2Degree(input[n], inputmin[n], inputmax[n], anglemin[n], anglemax[n]);
  }


 //Envío del mensaje
  pot_msg.data_length = POT;      // Definir la longitud del mensaje

  for (n = 0; n < POT; n++) {
    pot_msg.data[n] = angle[n];   // Asignar los valores de los ángulos al mensaje
  }

  pub.publish(&pot_msg);          // Publica el mensaje en ROS
  nh.spinOnce();                  // Mantiene la comunicación con ROS

  //delay(10);               // Espera 10 ms antes de la próxima lectura
}