/* gemello gemello gemello gemello gemello gemello gemello gemello */


const int min_throttle_1=1171; //PIN 9
const int min_throttle_2=1173; //PIN 3
const int min_throttle_3=1172; //PIN 6
const int min_throttle_4=1143; //PIN 5
int thrust=0;

//PID VALUES
const double kp_pitch=10.00;//10.50;//
const double ki_pitch=0.0;//0.0048;
const double kd_pitch=1.2;//0.5;

const double kp_roll=10.0;
const double ki_roll=0.00;
const double kd_roll=1.2;//0.15;

const double kp_yaw=4.2;//1.85;
const double ki_yaw=0.00;//0.005;
const double kd_yaw=0.0;

float I_prev[3] = {0,0,0};//array in cui vengono salvate i valori integrali precedenti
                //per ogni pid


float pid_pitch=0;
float pid_roll=0;
float pid_yaw=0;


//variabili per FILTRO PITCH
volatile float pitch_array[5] = {0,0,0,0,0};

volatile float pid_pitch_array[5] = {0,0,0,0,0};

int pitch_filter_dimension=4;
//volatile float n_pitch_array[];
float pitch_filtered=0;
float pid_pitch_filtered=0;



//variabili per FILTRO ROLL
volatile float roll_array[5] = {0,0,0,0,0};

volatile float pid_roll_array[5] = {0,0,0,0,0};

int roll_filter_dimension=4;
//volatile float n_pitch_array[];
float roll_filtered=0;
float pid_roll_filtered=0;




//variabili per FILTRO YAW
volatile float yaw_array[5] = {0,0,0,0,0};
int yaw_filter_dimension=4;
//volatile float n_pitch_array[];
float yaw_filtered=0;



//inizializzo tempi
//float   elapsedTime, time, timePrev;
float   previous_error_pitch,previous_error_yaw,previous_error_roll;

float cmd_1=0; //pin 9
float cmd_2=0; //pin 3
float cmd_3=0; //pin 6
float cmd_4=0; //pin 5

float desired_yaw=0.0;
float desired_pitch=0.0;
float desired_roll=0.0;
float desired_thrust=0.0;

float error_yaw=0.0;
float error_pitch=0.0;
float error_roll=0.0;
float error_thrust=0.0;


void calib_esc(void);
void gyro_setup(void);
void run_all_motor(void);
void loop_test_function(int);
void motor_cmd(float,float,float,float);

void pitch_adjust(float *,float *,float *,float *,float); //correzione pitch GREZZO
void cmd_adjust(float *,float *,float *,float *,float*,float*,float*,int *);//somma dei PID



//FUNZIONI PER CALCOLO ERRORI



float e_roll(float *roll){  //ERRORE ROLL
    /* SE VOGLIO INSERIRE IL VALORE
        Serial.println("inserisci valore roll desiderato");
        while (!Serial.available());
        Serial.read();
        desired_roll=Serial.parseFloat();*/
    desired_roll=0.0;
    error_roll=desired_roll-*roll;//-1.1
    //Serial.print("\terrore roll=");Serial.print(error_roll);
    return error_roll; 
}

float e_pitch(float *pitch){ //ERRORE PITCH
    /*  SE VOGLIO INSERIRE IL VALORE
        Serial.println("inserisci valore pitch desiderato");
        while (!Serial.available());
        Serial.read();
        desired_pitch=Serial.parseFloat();*/
    desired_pitch=0.0;
    error_pitch=desired_pitch-*pitch;//+7.8
    //Serial.print("\terrore pitch=");Serial.print(error_pitch);
    return error_pitch;
}

float e_yaw(float *yaw){ //ERRORE YAW
    /*  SE VOGLIO INSERIRE IL VALORE
        Serial.println("inserisci valore yaw desiderato");
        while (!Serial.available());
        Serial.read();
        desired_yaw=Serial.parseFloat();*/
    desired_yaw=0.0;
    error_yaw=desired_yaw-*yaw;
    //Serial.print("\terrore yaw=");Serial.println(error_yaw);
    return error_yaw;
}


//calcolo del pid generico su asse desiderato
float select_PID(float *I_prev,float *error,float *elapsed_Time,float *prev_e,int pid_type){
    
    //pid_type Pitch=1; roll=2; yaw=3
    float Ti,Td;
    float P,D;
    float e,pid;
    

    float kp,ki,kd;
    
    switch (pid_type)
    {
    case 0: //caso del calcolo pid per pitch
        kp=kp_pitch;
        ki=ki_pitch;
        kd=kd_pitch;
        
        break;
    case 1: //caso del calcolo pid per roll
        kp=kp_roll;
        ki=ki_roll;
        kd=kd_roll;
        
        break;
    case 2: //caso del calcolo pid per yaw
        kp=kp_yaw;
        ki=ki_yaw;
        kd=kd_yaw;
        
        break;    
    
    default:
        break;
    }
    /* VERSIONE PID ALTERNATIVA
        Ti=kp_pitch/ki_pitch;
        Td=kd_pitch/ki_pitch;
        I=(I+e)/Ti;
        D=D=kd_pitch*((e-(*prev_e))/(*elapsed_Time));
        pid_pitch=kp_pitch*(1+I+D);
    
    */
    /*calcolo pid precedente

        e=*error;

    
        P=kp*e;
        I=I+(ki*e);
        D=kd*((e-(*prev_e))/(*elapsed_Time));
        pid=10*(P+I+D);
    */
    e=*error;
    /*
    
    /*
    Serial.print("\tErrore gradi:\t");
        Serial.print(e);   
    //*/ 
    P=kp*e;
    I_prev[pid_type]=I_prev[pid_type]+(ki*e); 
    D=kd*((e-(*prev_e))/(*elapsed_Time));
    pid=(P+I_prev[pid_type]-D);

    
   
    return pid;
}



void filter(float *actual_value, float *value, volatile float *array,float time){

        if (time>0){
    		//FILTRO 5 Sample Average To Smooth Out The Data
    		array[0] = array[1];
    		array[1] = array[2];
    		array[2] = array[3];
    		array[3] = array[4];
    		array[4] = *actual_value;    
   		 	//la Media degli ultimi 5 valori equivale a...
   			*value= (array[0] + array[1] +
            array[2] + array[3] + array[4]) / 5;
		}     

}

/*


//prototipo filtro con n campioni
//SISTEMA, L'ALLOCAZIONE DEL VETTORE DEVE ESSERE DINAMICA
void filter_1(float *actual_value, float *value, volatile float *array,int n){
        
        if (time>0){
    		//FILTRO 5 Sample Average To Smooth Out The Data
            for(int i=0;i<n;i++){
                
                for(int j=0;j<n-1;j++){
                    array[j]=array[j+1];
                }
                
                
            }
            array[n] = *actual_value;
            float tmp_value=0;

            for (int i = 0; i < n; i++)
            {
                tmp_value=tmp_value+array[i];
            }
            
            *value=tmp_value/n;     //media di n valori

		}     

}


//prototipo filtro con n campioni
//SISTEMA, L'ALLOCAZIONE DEL VETTORE DEVE ESSERE DINAMICA

void var_filter_1(float *actual_value, float *value, volatile float *array_n,int n){
        
        //volatile float array[n];//allocazione dinamica vettore

        if (time>0){
    		//FILTRO 5 Sample Average To Smooth Out The Data
            for(int i=0;i<n;i++){
                
                for(int j=0;j<n-1;j++){
                    array_n[j]=array_n[j+1];
                }
                
                
            }
            array_n[n] = *actual_value;
            float tmp_value=0;

            for (int i = 0; i < n; i++)
            {
                tmp_value=tmp_value+array_n[i];
            }
            
            *value=tmp_value/n;     //media di n valori

		}     

}


*/

//calcolo comandi da impartire in base a ogni PID
void cmd_adjust(float *cmd_1,float *cmd_2,float *cmd_3, 
                float *cmd_4,float *pid_pitch,float *pid_roll,
                float *pid_yaw,int *thrust){
    
    *cmd_1=*thrust+ *pid_pitch -*pid_roll-*pid_yaw;
    
    if (*cmd_1<=min_throttle_1)//evito valori non ammissibili
        {
            *cmd_1=min_throttle_1;
        }
        else if(*cmd_1>=1800){
            *cmd_1=1800;
        }

    *cmd_2=*thrust- *pid_pitch- *pid_roll+*pid_yaw;
    if (*cmd_2<=min_throttle_2)//evito saturazione
        {
            *cmd_2=min_throttle_2;
        }
        else if(*cmd_2>=1800){
            *cmd_2=1800;
        }

    *cmd_3=*thrust- *pid_pitch+ *pid_roll-*pid_yaw;
    if (*cmd_3<=min_throttle_3)//evito saturazione
        {
            *cmd_3=min_throttle_3;
        }
        else if(*cmd_3>=1800){
            *cmd_3=1800;
        }

    *cmd_4=*thrust+ *pid_pitch+ *pid_roll+*pid_yaw;
    if (*cmd_4<=min_throttle_4)//evito saturazione
        {
            *cmd_4=min_throttle_4;
        }
        else if(*cmd_4>=1800){
            *cmd_4=1800;
        }

    
}
