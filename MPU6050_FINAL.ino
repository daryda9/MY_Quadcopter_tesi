#include <Wire.h>
#include <MPU6050.h>
#include <Math3D.h>
#include <PollTimer.h>
 
#define _DEGREES(x) (57.29578 * x)   // 57.29578 = 180/PIGRECO. funzione che trasforma da radianti a gradi.

MPU6050 MPU(400, 1, 0, 0);           // (update rate in Hz, Filtraggio 0-6 (0-> no-filtraggio 6->max-filtraggio), Gyro settings 0-3 (2g,4g,8g,16g), accel setting  0-3 (250,500,100,2000 [rad/sec]))

PollTimer samplingLoop(400UL);       // frequenza di campionamento del controllore in Hz. ES: 400 --> 1/400 = 2.5 ms <- intervallo di campionamento
PollTimer debugLoop(10UL);           // frequenza loop di debug - 10 volte al secondo

void loop_function_quat(float,float,float,bool,float,float);
bool stop_flag=HIGH;

float yaw_e=0.0, pitch_e=0.0, roll_e=0.0;
float yaw_q=0.0, pitch_q=0.0, roll_q=0.0;

float elapsedTime, time, timePrev;

//variabili
Quat AttitudeEstimateQuat;          //When used to represent an orientation (rotation relative to a reference coordinate system), quaternions are called attitude quaternions. Wikipedia.


Vec3 correction_Body, correction_World;
Vec3 Accel_Body, Accel_World;
Vec3 GyroVec;

float yaw,pitch,roll;//per gli angoli normali (Eulero)


const Vec3 VERTICAL = Vector(0.0f, 0.0f, 1.0f);  // vettore verticale


void gyro_setup() {



	Serial.begin(115200);  
	Serial.println("--- Avvio dell'IMU, Dario ti lovvo ---");
	Wire.begin();     
	Wire.setClock(400000UL);        // set speed to 400k



  

	MPU.initialize();
	MPU.accelZero();                // funzione di calibrazione dell'accellerometro
	MPU.gyroZero();	                // funzione di calibrazione del giroscopio

  //Serial.println(MPU.samplePeriod);

	samplingLoop.start();           //avvia loop principale
	debugLoop.start();              //avvia loop di debug
  
	
}

void loop() 
{

    timePrev = time;                        // the previous time is stored before the actual time read
    time = millis();                        // actual time read
    elapsedTime = (time - timePrev) / 1000; //divide by 1000 in order to obtain seconds


	if(samplingLoop.check())  // questo è il loop principale, per cambiare la sua durata modifica il numero nella variabile samplingLoop
	{
  
  		MPU.retrieve();	                                                             // get data from the sensor
  		GyroVec  = Vector(MPU.gX, MPU.gY, MPU.gZ);	                                 // move gyro data to vector structure
  		Accel_Body = Vector(MPU.aX, MPU.aY, MPU.aZ);	                               // move accel data to vector structure
  		Accel_World = Rotate(AttitudeEstimateQuat, Accel_Body);                      // rotate accel from body frame to world frame
  		correction_World = CrossProd(Accel_World, VERTICAL);                         // cross product to determine error
  		Vec3 correction_Body = Rotate(correction_World, AttitudeEstimateQuat);       // rotate correction vector to body frame
  		GyroVec = Sum(GyroVec, correction_Body);                                     // add correction vector to gyro data
  		Quat incrementalRotation = Quaternion(GyroVec, MPU.samplePeriod);            // create incremental rotation quat
  		AttitudeEstimateQuat = Mul(incrementalRotation, AttitudeEstimateQuat);       // quaternion integration (rotation composting through multiplication)
    
     
      //inserisci qui il resto del codice per il drone
        yaw_e=AttitudeEstimateQuat.z*57.29578;
        roll_e=AttitudeEstimateQuat.y*57.29578;
        pitch_e=AttitudeEstimateQuat.x*57.29578;
        
        printYPR();

        if(Serial.available()==0){
			//loop_function_quat(roll,pitch,yaw,stop_flag,time,elapsedTime);
			loop_function_quat(roll_e,pitch_e,yaw_e,stop_flag,time,elapsedTime);//try this

		}

		else
		{
			
			if (stop_flag==HIGH){
				stop_flag=LOW;
				loop_function_quat(0.0,0.0,0.0,stop_flag,time,elapsedTime);
				Serial.println("EMERGENCY STOP");
			}
		}
		

 







      roll_q=AttitudeEstimateQuat.x;
      pitch_q=AttitudeEstimateQuat.y;
      yaw_q=AttitudeEstimateQuat.z;
      
      Serial.print("Yaw: ");Serial.print(yaw_q*100);Serial.print("  Pitch: ");Serial.print(pitch_q*100);Serial.print("  Roll: ");Serial.println(roll_q*100);




    
	}
	else if(debugLoop.check())	// questo è un loop secondario che chiamiamo meno volte al secondo, serve semplicemente per stampare roba a video e quindi per fare debug.
	{ 
    
     //printQuat();
     printYPR();

     //samplingLoop.collectStats();
     //samplingLoop.displayStats();
     
	}

} // Main Loop End

void printQuat(){
  
    display(AttitudeEstimateQuat);
  }
void printYPR(){
    
    Vec3 YPR = YawPitchRoll(AttitudeEstimateQuat);   //trasforma il quaternion in yaw pitch roll. Attenzione, soffre di GIMBAL LOCK.
    yaw = _DEGREES(-YPR.x);
    pitch = _DEGREES(-YPR.y);
    roll = _DEGREES(-YPR.z);
    
    /*Serial.print("  Yaw:");  
    Serial.print(yaw);  
    Serial.print(",");
    Serial.print(pitch);  
    Serial.print(",");
    Serial.println(roll);*/
}