/*ESC calibration sketch; author: ELECTRONOOBS */


//#include <Arduino.h>


#include <Servo.h>

#define MAX_SIGNAL 2000
#define MIN_SIGNAL 1000
#define MOTOR_PIN1 9
#define MOTOR_PIN2 3
#define MOTOR_PIN3 6
#define MOTOR_PIN4 5



int DELAY = 1000;

Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;


void calib_esc() { //funzione IMPORTANTE per calibrazione ESC, eseguita nel SETUP  all'inizio (controlla)
  //Serial.begin(9600);
  Serial.println("CALIBRAZIONE...");
  Serial.println(" ");
  
  delay(1500);
  Serial.println("Avvio Programma.");
  delay(1000);
  Serial.println("Questo avvierà gli ESC.");

  motor1.attach(MOTOR_PIN1);
  motor2.attach(MOTOR_PIN2);
  motor3.attach(MOTOR_PIN3);
  motor4.attach(MOTOR_PIN4);
  //Scrittura massimo output
  Serial.print("Scrittura massimo output: (");Serial.print(MAX_SIGNAL);Serial.print(" us nel nostro caso)");Serial.print("\n");
  Serial.println("collega l'alimentazione, poi aspetta 2 secondi e premi un tasto qualsiasi.");
  motor1.writeMicroseconds(MAX_SIGNAL);
  motor2.writeMicroseconds(MAX_SIGNAL);
  motor3.writeMicroseconds(MAX_SIGNAL);
  motor4.writeMicroseconds(MAX_SIGNAL);

  /* Wait for input RIMUOVI QUANDO CONTROLLATO per rendere indipendente
  while (!Serial.available());
    Serial.read();
    FINO QUI*/

  Serial.print("wait..");
  delay(2000);//attende in autonomo
  // Send min output
  Serial.println("\n");
  Serial.println("\n");
  Serial.print("Sending minimum output: (");Serial.print(MIN_SIGNAL);Serial.println(" us in this case)");

  motor1.writeMicroseconds(MIN_SIGNAL);
  motor2.writeMicroseconds(MIN_SIGNAL);
  motor3.writeMicroseconds(MIN_SIGNAL);
  motor4.writeMicroseconds(MIN_SIGNAL);

  //
  Serial.println("ESC calibrati");
  Serial.println("----");
  
}


//COMANDARE I MOTORI
void motor_cmd(float cmd_1, float cmd_2, float cmd_3, float cmd_4){ //definito il valore di giri per ogni motore, impartiamo i comandi tramite queta funzione
  if(cmd_1>1000 && cmd_1<=1800){//verifico che non stia saturando
    motor1.writeMicroseconds(cmd_1);
  }
  if(cmd_2>1000 && cmd_2<=1800){
    motor2.writeMicroseconds(cmd_2);
  }
  if(cmd_3>1000 && cmd_3<=1800){
    motor3.writeMicroseconds(cmd_3);
  }
  if(cmd_4>1000 && cmd_4<=1800){
    motor4.writeMicroseconds(cmd_4);
  }
  

}

/*
void loop_test_function(int tempo){ //funzione eseguita nel SETUP, chiede di definire la velocità del test
    Serial.println("yee");
    Serial.println("inserisci valore tra 1000 e 2000");
    while (!Serial.available());
      Serial.read();
    if (Serial.available() > 0) {
      int SPEED = Serial.parseInt();
      Serial.print("avvio di tutti i motori insieme per ");Serial.print(tempo/1000);Serial.println("secondi");
      run_all_motor(SPEED);//avvia tutti i motori insieme
      delay(tempo);
      run_all_motor(1000);//li stoppa dopo (tempo) millisecondi
    }
  
}


void run_all_motor(int DELAY) { //funzione per avviare tutti i motori insieme, usato nel SETUP   
    if (DELAY > 999)
    {
      motor1.writeMicroseconds(DELAY);
      motor2.writeMicroseconds(DELAY);
      motor3.writeMicroseconds(DELAY);
      motor4.writeMicroseconds(DELAY);
      float SPEED = (DELAY-1000)/10;
      Serial.print("\n");
      Serial.println("Motor speed:"); Serial.print("  "); Serial.print(SPEED); Serial.print("%"); 
    }     
  
}
*/



