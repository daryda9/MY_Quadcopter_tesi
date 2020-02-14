

/* includo gli altri file
 #include "calibrazione_esc.ino"
 #include "MPU5060_quat.ino"
 */
//#include <Arduino.h>
#include "Main_class.h"

/*ex reset, resetta qualsiasi cosa
void(*resetFunc) (void) =0;
int resetcount=0;
*/



void setup(){

    

    Serial.begin(115200);
    
    /*
    Serial.print("resetcount= ");
    Serial.println(resetcount);
    delay(7000);
    if (resetcount==0){
        resetcount++;
        soft_restart();
       

    }
      */

    calib_esc();    //calibrazione di tutti gli esc
    delay(500);
    gyro_setup();   //inizializzazione giroscopio

    
    delay(2000);
    motor_cmd(1200,1200,1200,1200);
    delay(500);
    motor_cmd(1100,1100,1100,1100);
    delay(2000);
    //NON USO PIU'loop_test_function(2000);   //funzione TESTallmotor per 2 secondi (2000ms)
    
    Serial.print("FINE DEL SETUP, "); Serial.println("avvio a minimo throttle");
    cmd_1=min_throttle_1; cmd_2=min_throttle_2; cmd_3=min_throttle_3; cmd_4=min_throttle_4;
    motor_cmd(cmd_1,cmd_2,cmd_3,cmd_4); //tutto a minimo throttle
    
    

}


void loop_function_quat(float pitch,float roll,float yaw,bool state,float time, float elapsedTime){

    if (state==HIGH){
    //tempi per PID
    /*timePrev = time;  // the previous time is stored before the actual time read
    time = millis();  // actual time read

    elapsedTime = (time - timePrev) / 1000;
    //*/

    //Serial.print("pitch:");Serial.print(pitch);Serial.print(" roll: ");Serial.print(roll);Serial.print(" yaw: ");Serial.println(yaw);
    //Serial.print(" pitch: ");Serial.println(pitch*100);

    Serial.print("\t");
    


    // filtri per assi n=5 fisso campioni
    
    //
    //Serial.print(pitch_filtered);
    //Serial.print("\t");

    //var_filter_1(&pitch,&pitch_filtered,&n_pitch_array[0],pitch_filter_dimension);// filtro pitch ultimi n campioni
    //filter_1(&pitch,&pitch_filtered,int &var_pitch_array[0],4);// filtro pitch ultimi n campioni
    //Serial.print(pitch_filtered);
    
    /* USANDO FILTRI
    filter(&pitch,&pitch_filtered, &pitch_array[0]);//filtro pitch
    filter(&roll,&roll_filtered, &roll_array[0]);//filtro roll
    filter(&yaw,&yaw_filtered, &roll_array[0]);//filtro yaw

    error_pitch=e_pitch(&pitch_filtered);//stampa anche error_pitch
    error_roll=e_roll(&roll_filtered);//stampa anche error_pitch
    error_yaw=e_yaw(&yaw_filtered);//stampa anche error_pitch

    pid_pitch=select_PID(&I_prev[0],&error_pitch, &elapsedTime, &previous_error_pitch,0);
    pid_roll=select_PID(&I_prev[0],&error_roll, &elapsedTime, &previous_error_roll,1);
    pid_yaw=select_PID(&I_prev[0],&error_yaw, &elapsedTime, &previous_error_yaw,2);
    
    //*/
    
    //*senza filtri
    error_pitch=e_pitch(&pitch);//stampa anche error_pitch
    error_roll=e_roll(&roll);
    error_yaw=e_yaw(&yaw);
    

    pid_pitch=select_PID(&I_prev[0],&error_pitch, &elapsedTime, &previous_error_pitch,0);

    //provo a filtrare pid del pitch
    //filter(&pid_pitch,&pid_pitch_filtered, &pid_pitch_array[0]);//filtro pitch

    //Serial.print("\troll:\t");
    pid_roll=select_PID(&I_prev[0],&error_roll, &elapsedTime, &previous_error_roll,1);
    //Serial.println();
    //filter(&pid_roll,&pid_roll_filtered, &pid_roll_array[0]);//filtro pitch
    pid_yaw=select_PID(&I_prev[0],&error_yaw, &elapsedTime, &previous_error_yaw,2);
    
    //pid_yaw=select_PID(&I_prev[0],&error_yaw, &elapsedTime, &previous_error_yaw,2);
    
    //*/

    
    /*stampa a video di tutti gli errori
        Serial.print("errore pitch: ");Serial.println(error_pitch );
        Serial.print("errore roll: ");Serial.println(error_roll);
        Serial.print("errore yaw: ");Serial.println(error_yaw);   
    */

    //pitch_adjust(&cmd_1,&cmd_2,&cmd_3,&cmd_4,error_pitch); CALCOLO GREZZO
    

    //Serial.print("avg_pitch:\t "); Serial.print(pitch_filtered);
	//Serial.print("\tnormal_pitch:\t "); Serial.println(pitch);

    //Serial.print("\tPid pitch:\t");Serial.print(pid_pitch);
    /* RIMETTI, OUT dei MOTORI
    
         Serial.print("\tmotore 1: "); Serial.print(cmd_1);Serial.print(" motore 2: "); Serial.print(cmd_2);
         Serial.print(" motore 3: "); Serial.print(cmd_3);Serial.print(" motore 4: "); Serial.println(cmd_4);
    
    //*/
    thrust=1200;

    cmd_adjust(&cmd_1,&cmd_2,&cmd_3,&cmd_4,&pid_pitch,&pid_roll,&pid_yaw,&thrust);

    motor_cmd(cmd_1,cmd_2,cmd_3,cmd_4);

    

    //filter(&pid_roll,&pid_roll_filtered, &pid_roll_array[0],time);//filtro pitch

    Serial.print("\troll PID\t");Serial.print(pid_roll);
    Serial.print("\troll_angle\t");Serial.print(roll);

    Serial.print("\tpitch PID\t");Serial.print(pid_pitch);
    Serial.print("\tpitch_angle\t");Serial.print(pitch);

    Serial.print("\tyaw PID\t");Serial.print(pid_yaw);
    Serial.print("\tyaw_angle\t");Serial.print(yaw);

    Serial.println("");

    previous_error_pitch = error_pitch ;
    previous_error_roll = error_roll;
    previous_error_yaw = error_yaw ;
    //Serial.print("\t\t");
    //Serial.println(elapsedTime*1000);
    }
    else if (state==LOW)
    {
        motor_cmd(1100,1100,1100,1100);
    }
    
    
}

