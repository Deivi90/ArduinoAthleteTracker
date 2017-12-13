#include <Wire.h>

const int MPU=0x68; // Dirección I2C del acelerometro, está definida por el protocolo.
const float accEscala=16*9.81/32768;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ; // Defino las variables de 16 bits con signo donde voy a guardar la información.
//float x,y,z;
void setup() {

  pinMode(13, OUTPUT);
  digitalWrite(13,LOW);
  
  //
  //CONFIGURACION DE INTERRUPCION POR TIEMPO
  // http://www.avrbeginners.net/architecture/timers/timers.html#timsk_tifr
  // http://www.robotshop.com/letsmakerobots/arduino-101-timers-and-interrupts
  // Seteo el tiempo en 100ms, es decir se manda los datos cada 100ms
    
  cli(); //Desactivo todas las interrupciones
  //Inicializo los registros en cero
  TCCR1A = 0;
  TCCR1B = 0;
  //El contador arranca en cero
  TCNT1 = 0;

  OCR1A = 12500;    //Compare match register = (16mhz*tiempo)/prescaler =(16mhz*0.1)/64
  TCCR1B |= (1 << WGM12); // CTC MODE. Se resetea el timer cuando se llega al valor establecido
  // Seteo el prescaler en 64
  TCCR1B |= (1 << CS10);
  TCCR1B |= (1 << CS11); 
  TIMSK1 |= (1 << OCIE1A);  // TIMSK es el registro de interrupciones. OCIE1A enables timer compare interrupt 
  sei(); //Activo todas las interrupciones
  
  //
  //CONFIGURACION ACELEROMETROS
  //  
  Wire.begin();
  // Inicio al acelerometro:
  Wire.beginTransmission(MPU); // Dirección del acelerometro, en I2C cada modulo ya tiene una dirección predeterminada.
  Wire.write(0x6B); // Registro de Power Management 1, tiene registros, está el Register Map en internet.
  Wire.write(0x00); // En ese registro escribo solo ceros, evitando que este en modo SLEEP.
  Wire.endTransmission(true); // Termíno la transmisión.
  // Modifico la sensibilidad del acelerometro:
  Wire.beginTransmission(MPU); // Dirección del acelerometro.
  Wire.write(0x1C); // Dirección del configuración de acelerometro.
  Wire.write(0x18); // En ese registro 3 (11) en los dos bits de sensibilidad, asi tenemos 16g.
  Wire.endTransmission(true); // Termíno la transmisión.
  Serial.begin(9600);
}


ISR(TIMER1_COMPA_vect)   //Funcion que se llama cuando se da la interrupcion
{
    //digitalWrite(13, digitalRead(13) ^ 1);   // toggle LED pin ^-->xor
    //Serial.print(AcX*accEscala);Serial.print(" ");
    //Serial.print(AcY*accEscala);Serial.print(" ");
    Serial.print(AcZ*accEscala);
    Serial.print("#");
}

void loop() {
  Wire.beginTransmission(MPU); // Dirección del acelerometro
  Wire.write(0x3B); // Empiezo por el registro 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false); // No termino la transmision
  Wire.requestFrom(MPU,14,true); // Al aceleremetro le pido 14 registros
  // Le pido el byte alto y se guarda en los primeros 8 bits, asi que lo muevo
  // <<8 bits a la derecha, ahora ocupa el byte mas alto de mi entero de 16 bits
  // luego con | (or de a bit) le agrego el byte bajo en los primero 8 bits del entero. 
  AcX=Wire.read()<<8|Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY=Wire.read()<<8|Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read(); // 0x41 (TEMP_OUT_H)  & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
 /* 
  Serial.print("Aceleración x:");Serial.println(AcX*accEscala);
  Serial.print("Aceleración y:");Serial.println(AcY*accEscala);
  Serial.print("Aceleración z:");Serial.println(AcZ*accEscala);
  Serial.print("-----------------------------------");Serial.print('\n');
  delay(500);
  */
/*  
 *   

   if (Serial.available()) /envia info solo cuando recibo
    {
      char VarChar;
      VarChar = 1;
      if(VarChar == '1')
    {
        Serial.print(AcZ*accEscala);
        Serial.print("#");
    }


    if(VarChar == '0')
    {
      Serial.print("LED Apagado");
      Serial.print("#"); 
            Serial.print("Apagado");
      Serial.print("#"); 
            Serial.print("L Apagado");
      Serial.print("#"); 
    }

    //Serial.println("1234568#");
 }*/

       //Serial.print(AcZ*accEscala);
       //Serial.print("#");
       //delay(100);   // Si no le pongo este delay no lo puedo leer desde el celular, despues conviene hacerlo con interrupciones 
}
