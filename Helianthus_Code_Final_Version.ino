#include <AccelStepper.h>  //Librerías incluidas....
#include <MultiStepper.h>
#include <SoftwareSerial.h>  //Librería para comunicaciones Bluetooth

// Declaracion de la asignación de pines
int ledPin = 13;                 //Pin de led......
AccelStepper stepper1(1, 2, 5);  //motor giro horizontal en pines 2 y 5
AccelStepper stepper2(1, 3, 6);  //motor elevación en pines 3 y 6
const int muxSIG = A0;           //Lectura de señal de sensores de luz del multiplexor
const int voltage = A1;          //Lectura de la tensión de salida de los paneles
const int current = A2;          // Lectura de la corriente de salida de los paneles
const int muxS0 = 7;             // Direccioamiento del multiplexor de los sensores, 4 pines
const int muxS1 = 4;
const int muxS2 = 13;
const int muxS3 = 12;
const int BTaTx = 9;
const int BTaRx = 10;

// variables globales auxiliares
int positions[5] = { 145, 48, -48, -160, 0 };                                           // Posicioines horizontales. POSICION 0: 168 ; Posicion 2: 56 ; Poscion 5: -56 ; POSICION -8: 168
int positions_2[11] = { 0, -36, -72, -108, -155, -201, -253, -305, -346, -386, -427 };  // 11 posiciones verticales para angulos de levacion 0,15,25,35,45,55,65,75,85,95, resto
int modoAutomatico = 0;                                                                 // en modo automatico va al sensor con más luz
const int medida_baja = 200;                                                            // dependiendo de la fuente de luz
const int medida_media = 500;
const int medida_alta = 1000;

// Varibles de control del comportamiento
const int delayValue = 20;  // milisegundos de espera para leer la señal del multiplexor despues de establecer la dirección de lectura
const int cycle = 300;      // milisegundos de espera entre cada ejecución del bucle principal //antes 300
const int maxSpeedHorizontal = 300;
const float maxAccelerationHorizontal = 100.0;
const int maxSpeedVertical = 200;
const float maxAccelerationVertical = 100.0;
const float pi = M_PI;
int indexMaxVal;

// control de mensajes de depuración en el puerto serie
#define DEBUG_MOVE
//#define DEBUG_SENSORS_TOP
//#define DEBUG_SENSORS_LATERAL
//#define DEBUG_WITHIN15
//#define DEBUG_HORIZONTAL
//#define DEBUG_VERTICAL
//#define DEBUG_POWER
#define DEBUG_BT

// Creación del interfaz de comunicaciones Bluetooth e inicialización de las variables con su configuración
SoftwareSerial miBT(BTaTx, BTaRx);  // Instancia de la conexión a Bluetooth
char NOMBRE[21] = "Elite";          // Nombre 20 caracteres máximo
char BPS = '4';                     // El número 4 equivale a 9600 bps de velocidad de transmisión
char PASSWORD[5] = "6666";          // Clave del 4 caracteres para conexion a BT
String cmd = "";                    // Comando recibido por Bluetooth

// Variables para el envio de potencia generada
const unsigned long intervaloMoverse = 5000;        //Moverse cada 5 segundos
const unsigned long intervaloEnvioPotencia = 1000;  // enviamos la potencia cada 1 segundo
unsigned long tiempoAnterior = 0;                   // almacena la ultima vez que se envió la potencia
unsigned long tiempoActual = 0;                     // almacenará el tiempo actual
float power = 0.0;                                  // ultima potencia leida
float totalPower = 0.0;                             // potencia acumulada desde el encendido
const int sensitivity = 185;                        // sensibilidad del medidor de corriente ASC712 de 5A
const int offsetVoltage = 2500;                     // Correcion del voltage del medidor para tener valores positivos
int adcValue = 0;                                   // Valor del Analog to Digital Converter del mediro de corriente
double adcVoltage = 0;                              // Voltage del medidor de corriente
double currentValue = 0;                            // Valor de la corriente en amperios
double volts = 0;                                   // Voltage

// inicialización
void setup() {
  Serial.begin(9600);  // puerto serie para depuración

  // configuración del interfaz Bluetooth
  miBT.begin(9600);  // inicia la comunicación del módulo BT

  miBT.print("AT");  // inicializa la configuración con comandos AT
  delay(1000);       // tiempo para que se pueda procesar el comando

  miBT.print("AT+NAME");  // configurando el nombre
  miBT.print(NOMBRE);
  delay(1000);  // tiempo para que se pueda procesar el comando

  miBT.print("AT+BAUD");  // configurando la velocida de transmision
  miBT.print(BPS);
  delay(1000);  // tiempo para que se pueda procesar el comando

  miBT.print("AT+PIN");  // configurando la clave para la conexión
  miBT.print(PASSWORD);
  delay(1000);  // tiempo para que se pueda procesar el comando

  // configuraciones de pines
  pinMode(ledPin, OUTPUT);
  pinMode(muxSIG, INPUT);
  pinMode(voltage, INPUT);
  pinMode(current, INPUT);
  pinMode(muxS0, OUTPUT);
  pinMode(muxS1, OUTPUT);
  pinMode(muxS2, OUTPUT);
  pinMode(muxS3, OUTPUT);

  // configuració del motor de giro horizontal
  stepper1.setMaxSpeed(maxSpeedHorizontal);
  stepper1.setAcceleration(maxAccelerationHorizontal);
  stepper1.setCurrentPosition(0);  // colocar la base mirando al frente antes de encender

  // configuración del motor de elevación
  stepper2.setMaxSpeed(maxSpeedVertical);
  stepper2.setAcceleration(maxAccelerationVertical);
  stepper2.setCurrentPosition(0);  // colocar la base en su posición más baja antes de encender
}

// Funcion auxiliar: selección de la entrada del multiplexor
int SetMuxChannel(byte channel) {

  digitalWrite(muxS0, bitRead(channel, 0));
  digitalWrite(muxS1, bitRead(channel, 1));
  digitalWrite(muxS2, bitRead(channel, 2));
  digitalWrite(muxS3, bitRead(channel, 3));
}

// Funciones auxiliares: lectura de señal de sensores
int Pin(int PINnumber) {
  SetMuxChannel(PINnumber);
  delay(delayValue);
  int val = analogRead(muxSIG);
  return val;
}

// bucle principal
void loop() {

  using namespace std;

  // envia por Bluetooth cada segundo la potencia generada y la potencia total separadas por una coma
  tiempoActual = millis();                                        // en cada loop guardamos el tiempo
  if (tiempoActual - tiempoAnterior >= intervaloEnvioPotencia) {  // comprobamos si debemos enviar la potencia

    // medir la corriente, incluidas las conversiones para tener el valor en amperios
    adcValue = analogRead(current);
    adcVoltage = (adcValue / 1024.0) * 5000;
    currentValue = ((adcVoltage - offsetVoltage) / sensitivity);
    if (currentValue < 0) {
      currentValue = 0.0;  // cuando no hay corriente el medidor puede dar valores negativos
      volts = 0.0;         // cuando no hay corriente la medida de tensión no es correcta
    } else {

      //medir el voltage de salida de los paneles ahora que hay corriente
      double voltsRaw = analogRead(voltage);
      volts = 2.0 * (voltsRaw * (5.0 / 1024.0));  // Convierte el valor leido (0 - 1023) en voltios (0.0V - 5.0V). Multiplicamos por 2 por el divisor de tension
    }

    power = volts * currentValue;  // calcula la potencia actual
    //power = random(1,4);  // BORRAR ESTO ES SOLO PARA PROBAR LA GRAFICA SIN SOL
    totalPower += power;  // acumula la potencia leida

    // formatea los valores para enviar
    String cadenaParaEnviar = (String)power + "," + (String)totalPower;

    miBT.println(cadenaParaEnviar);
    tiempoAnterior = tiempoActual;

#ifdef DEBUG_POWER
    Serial.println((String) "Medida voltios y amperios: " + volts + "V, " + currentValue + "A");
    Serial.println((String) "Potencia enviada: " + cadenaParaEnviar);
#endif
  }

  // mueve los paneles hacie el sensor con mas luz si está en modo automático
  if (modoAutomatico) {

    //Lectura de los sensores de luz en la tapa superior
    int C0 = Pin(0);
    int C2 = Pin(2);
    int C5 = Pin(5);
    int C8 = Pin(8);
    int readings[] = { C0, C2, C5, C8 };  //lista con las lectura de los sensores en la tapa superior
    for (int i = 0; i < 4; i++) {
      if (readings[i] < 200) {
        readings[i] = medida_baja;                          //Lectura baja
      } else if (readings[i] > 200 && readings[i] < 500) {  //Lectura media
        readings[i] = medida_media;
      } else {
        readings[i] = medida_alta;  //Lectura alta
      }
    }
    int maxVal = 0;  // variable para guardar el valor máximo de los sensores en la tapa superior

#ifdef DEBUG_SENSORS_TOP
    Serial.print("Lectura sensores de arriba: ");
    Serial.print(readings[0]);
    Serial.print(",");
    Serial.print(readings[1]);
    Serial.print(",");
    Serial.print(readings[2]);
    Serial.print(",");
    Serial.println(readings[3]);
#endif

    //Lectura de los sensores de luz en las tapas laterales
    int C1 = Pin(1);
    int C3 = Pin(3);
    int C4 = Pin(4);
    int C6 = Pin(6);
    int C7 = Pin(7);
    int C9 = Pin(9);
    int readings_2[] = { C1, C3, C4, C6, C7, C9 };

    for (int i = 0; i < 6; i++) {
      if (readings_2[i] < 200) {
        readings_2[i] = medida_baja;                            //Lectura baja
      } else if (readings_2[i] > 200 && readings_2[i] < 500) {  //Lectura media
        readings_2[i] = medida_media;
      } else {
        readings_2[i] = medida_alta;  //Lectura alta
      }

    }                  //lista con las lectura de los sensores en las tapsa laterales
    int maxVal_2 = 0;  // variable para guardar el valor máximo de los sensores en las tapas laterales

#ifdef DEBUG_SENSORS_LATERAL
    Serial.print("Lectura sensores laterales:  ");
    Serial.print(readings_2[0]);
    Serial.print(",");
    Serial.print(readings_2[1]);
    Serial.print(",");
    Serial.print(readings_2[2]);
    Serial.print(",");
    Serial.print(readings_2[3]);
    Serial.print(",");
    Serial.print(readings_2[4]);
    Serial.print(",");
    Serial.println(readings_2[5]);
#endif

    // Buscar el valor máximo de los sensores superiores
    for (int i = 0; i < 4; i++) {
      if (maxVal < readings[i]) {
        maxVal = readings[i];
      }
    }

    // Copia al array within15 las lecturas de sensores superiores que están a menos del 15% del máximo
    float within15[4];  //INICALIZAMOS ARRAY DE FLOATS Y COUNTER
    int within15_count = 0;



#ifdef DEBUG_WITHIN15
    Serial.print("Lecturas laterales en el 15%: ");
    Serial.print(within15[0]);
    Serial.print(",");
    Serial.print(within15[1]);
    Serial.print(",");
    Serial.print(within15[2]);
    Serial.print(",");
    Serial.println(within15[3]);
#endif

    // genera el array distances con el porcentaje de distancia de la lectura al valor máximo de la lectura para los sensores superiores

#ifdef DEBUG_DISTANCES
    Serial.print("Distancias en procentaje:     ");
    Serial.print(distances[0]);
    Serial.print(",");
    Serial.print(distances[1]);
    Serial.print(",");
    Serial.print(distances[2]);
    Serial.print(",");
    Serial.println(distances[3]);
#endif

    // Mover los motores hacia la posición calculada
    //ITERA LA LISTA DE LAS DISTANCIAS, PARA CALCULAR EL INDICE DEL SENSOR SUPERIOR CON MAYOR LECTURA. VA A GIRAR HACIA ESE.
    for (int i = 0; i < 4; i++) {
      if (maxVal == readings[indexMaxVal]) {  //Si el valor maximo es el actual no moverse

      } else {
        if (readings[i] == maxVal) {  //y si no moverse al nuevo sensor predominante
          indexMaxVal = i;
        }
      }
    }
// mueve el motor en horizontal hacia el sensor indexMaxVal. Posición dada por lista positions
#ifdef DEBUG_MOVE
    stepper1.moveTo(positions[indexMaxVal]);
    stepper1.runToPosition();
#endif
#ifdef DEBUG_HORIZONTAL
    Serial.print("Nueva posición horizontal: ");
    Serial.println(positions[indexMaxVal]);
#endif


    //mueve ahora el motor vertical
    // calcula primero el angulo de elevación en función de los sensores cercanos al sensor horizontal al que está orientado
    float percent_angle = 0.0;  //porcentaje del angulo de elevacion
    float angle = 0.0;
    switch (indexMaxVal) {
      case 0:  // base orientada hacia el sensor 0, le corresponde el sensor lateral 1
        angle = (atan(float(C1) / float(C0))) * 180.0 / pi;
        break;
      case 1:  // base orientada hacia el sensor 2, le corresponden los sensores laterales 3 y 4
        if (C3 > C4) {
          angle = (atan(float(C3) / float(C2))) * 180.0 / pi;

        } else {
          angle = (atan(float(C4) / float(C2))) * 180.0 / pi;
        }
        break;
      case 2:  // base orientada hacia el sensor 5, le corresponden los sensores laterales 6 y 7
        if (C7 > C6) {
          angle = (atan(float(C7) / float(C5))) * 180.0 / pi;

        } else {
          angle = (atan(float(C6) / float(C5))) * 180.0 / pi;
        }
        break;
      case 3:  // base orientada hacia el sensor 8, le corresponde el sensor lateral 9
        angle = (atan(float(C9) / float(C8))) * 180.0 / pi;
        break;

      default:;  //No hace falta que haga nada
    }
#ifdef DEBUG_ANGULOS
    Serial.println((String) "Medida del angulo de C0 y C1 " + angle);
#endif
    // mueve el motor vertial a la posición correspondiente a ese angulo
    // la lista positions_2 tiene valores de elevación cada 10 grados
    int indexPositions_2 = angle / 9;
    if (indexPositions_2 > 9) {
      indexPositions_2 = 10;
    };  //evitar valores de indice más largo que la longitud de la lista
    if (indexPositions_2 < 0) {
      indexPositions_2 = 0;
    };  //evitar valores de indice más pequeños que la longitud de la lista
    Serial.println((String) "Numero de pasos en vertical: " + positions_2[indexPositions_2]);
#ifdef DEBUG_MOVE
    stepper2.moveTo(positions_2[indexPositions_2]);
    stepper2.runToPosition();
#endif
#ifdef DEBUG_VERTICAL
    Serial.print("Nuevo ángulo vertical: ");
    Serial.println(angle);
#endif
  }

  // lee comandos del interfaz Bluetooth
  while (miBT.available() > 0) {
    cmd += (char)miBT.read();
  }

  // determinad el comando y lo ejecuta
  if (cmd != "") {
#ifdef DEBUG_BT
    Serial.print("Comando recibido: ");
    Serial.println(cmd);
#endif
    // Comando validos: auto/manual para cambiar auto y manual,
    if (cmd == "auto") {
      modoAutomatico = 1;
    } else if (cmd == "manual") {
      modoAutomatico = 0;
    } else if (cmd == "centrado") {
      stepper1.moveTo(positions[4]);
      stepper1.runToPosition();
    } else if (cmd == "derecha") {
      stepper1.moveTo(positions[3]);
      stepper1.runToPosition();
    } else if (cmd == "izquierda") {
      stepper1.moveTo(positions[0]);
      stepper1.runToPosition();
    } else if (cmd == "arriba") {
      stepper2.moveTo(positions_2[10]);
      stepper2.runToPosition();
    } else if (cmd == "abajo") {
      stepper2.moveTo(positions_2[0]);
      stepper2.runToPosition();
    } else if ((String)cmd[0] == "*") {
      cmd.remove(0, 1);
      stepper1.moveTo(positions[cmd.toInt()]);
      stepper1.runToPosition();  // Se mueve
    }                            // Quitar asterisco para cambiar a integer (desde index 0, un caracter)
    else if ((String)cmd[0] == "/") {
      cmd.remove(0, 1);                           // Quitar asterisco para cambiar a integer
      stepper2.moveTo(positions_2[cmd.toInt()]);  // Si es negativo mover hacia el otro lado
      stepper2.runToPosition();
    } 
    else {
      // comando no soportado, no hacer nada
    };
    cmd = "";  //reset cmd
  }

  // Comentarios antiguos
  // //Buscar posicion numero maximo, buscar posicion segundo maximo -> distance fraction x pasos para girar 90º = pasos que se alejan del grande

  // //LADOS
  // //Por cada cara con reading, %distance del grande al pequeño (distancia de alejarse del grande), distance fraction x pasos girar 90º = pasos de alejar del grande

  Serial.println("---------------------------------------------------------------");
  delay(cycle);
}
