// TaISLAb 
// https://github.com/TaISLab/dummyarm
//
// DAQ for the dummy ARM
// Use Arduino Mega
//

// Código para la lectura de los potenciómetros y mapeo de estos valores a ángulos
// de posición de la garra.

//

// Declaración de constantes y variables:
#define periodo 10   // Constante para definir el periodo de muestreo en ms.
#define POT 2        // Constante para definir el número de potenciómetros.

long nextt, t;       // Variables para el control del tiempo.

int input[POT];      // Variable para almacenar las lecturas de los sensores (lecturas del ADC).
double angle[POT];   // Variable para almacenar el ángulo de posición.


//Cambiar estos valores:

// Establecer los límites para la conversión:
int    inputmin[POT] = {   0,    0};  // Valor mínimo de lectura del ADC.
double anglemin[POT] = {   0,    0};  // Valor mínimo de ángulo.
int    inputmax[POT] = {1023, 1023};  // Valor máximo de lectura del ADC.
double anglemax[POT] = {  90,   90};  // Valor máximo de ángulo.


// Configuración inicial:
void setup() {
  Serial.begin(250000);   // Inicializar comunicación serial a 250000 bps 
  nextt=millis();         // Configurar temporizador inicial             
 // Lee el valor del tiempo en el que la placa comienza a ejecutar el programa, tras inicilizar el entorno de ejecución.
}


// Función para mapear el rango de valores de entrada en un rango de valores de salida
double cv2Degree(double input, double inputmin, double inputmax, double outputmin, double outputmax)
{
  return outputmin+((input-inputmin)/(inputmax-inputmin))*(outputmax-outputmin); 
}

// Bucle principal
void loop() {
  while ((t=millis())<nextt);   //Espera activa
  nextt+=periodo;               
  
  // Lecturas analógicas del valor de los potenciómetros:
  input[0] = analogRead(A5);
  input[1] = analogRead(A4);

  //input[0] = 0;
  //input[1] = 1023;

  //Bucle para iterar sobre cada potenciómetro y obtener el valor del ángulo de posición de cada uno:
  int n;
  for (n=0; n<POT; n++) angle[n] = cv2Degree(input[n], inputmin[n], inputmax[n], anglemin[n], anglemax[n]);

 // Serial.print(t);    // Timestamp

 //Bucle para imprimir el valor del ángulo de cada potenciómetro
  for (n=0; n<POT; n++) {
    Serial.print(",     ");
    Serial.print(angle[n]);
    //delay(100);
  }
  Serial.println("");
}