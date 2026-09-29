#ifdef __cplusplus
extern "C" {
#endif

#include	"bobot.h"
#include	"bobot_structure.h"
#include	<math.h>

#define D1 10.0f
#define L1 8.0f
#define L2 8.0f
#define L3 7.0f
#define	NJOINT			5

#define STEPLIMIT		0.5f
#define DELAY			5
#define STANDARDELAY	100
#define CHBASE		TIM_CHANNEL_1
#define CHSHOULDER	TIM_CHANNEL_2
#define CHELBOW		TIM_CHANNEL_3
#define CHWRIST		TIM_CHANNEL_4
#define CHEFFECTOR	TIM_CHANNEL_3

#define	BASE0		1500 // fa 60a dx e 60 sx
#define	SHOULDER0	1700 // inverti
#define	ELBOW0		2000//1700 // inverti
#define	WRIST0		1400 //


//********************************************

Robot_t	Robot;


void BOBOT_init(TIM_HandleTypeDef* timeffector,TIM_HandleTypeDef* timarm){

	Robot.joints[0].htim = timarm;
	Robot.joints[0].channel = CHBASE;
	Robot.joints[0].step_limit = STEPLIMIT;
	Robot.joints[0].center_pulse= BASE0;

	Robot.joints[0].min_angle=-60;
	Robot.joints[0].max_angle=60;
	Robot.joints[0].min_pulse=1000;
	Robot.joints[0].max_pulse=2000;
	Robot.joints[0].inverse=1;

	Robot.joints[1].htim = timarm;
	Robot.joints[1].channel = CHSHOULDER;
	Robot.joints[1].step_limit = STEPLIMIT;
	Robot.joints[1].center_pulse= SHOULDER0;
	Robot.joints[1].min_angle=-36;
	Robot.joints[1].max_angle=84;
	Robot.joints[1].min_pulse=1000;
	Robot.joints[1].max_pulse=2000;
	Robot.joints[1].inverse=-1;

	Robot.joints[2].htim = timarm;
	Robot.joints[2].channel = CHELBOW;
	Robot.joints[2].step_limit = STEPLIMIT;
	Robot.joints[2].center_pulse= ELBOW0;
	Robot.joints[2].min_angle=0;
	Robot.joints[2].max_angle=120;
	Robot.joints[2].min_pulse=1000;
	Robot.joints[2].max_pulse=2000;
	Robot.joints[2].inverse=-1;


	Robot.joints[3].htim = timarm;
	Robot.joints[3].channel = CHWRIST;
	Robot.joints[3].step_limit = STEPLIMIT;
	Robot.joints[3].center_pulse= WRIST0;
	Robot.joints[3].min_angle=-48;
	Robot.joints[3].max_angle=72;
	Robot.joints[3].min_pulse=1000;
	Robot.joints[3].max_pulse=2000;
	Robot.joints[3].inverse=1;


	Robot.joints[4].htim = timeffector;
	Robot.joints[4].channel = CHEFFECTOR;
	Robot.joints[4].step_limit = STEPLIMIT;
	Robot.joints[4].center_pulse=1000;
	Robot.joints[4].min_angle=OPENED;
	Robot.joints[4].max_angle=CLOSED;
	Robot.joints[4].min_pulse=1000;
	Robot.joints[4].max_pulse=2000;
	Robot.joints[4].inverse=1;


//inizializza PWM
	for(int i=0;i<NJOINT;i++){
	  Robot.joints[i].current_angle= 0.0f;
	  HAL_TIM_PWM_Start(Robot.joints[i].htim,Robot.joints[i].channel);
	}

	BOBOT_set_angles(0,0,0,0,CLOSED);

	HAL_Delay(STANDARDELAY);

}

void BOBOT_effector(){
	if(Robot.joints[4].current_angle==OPENED){
		Robot.joints[4].target_angle=CLOSED;

	}else{
		Robot.joints[4].target_angle=OPENED;
	}
}

uint32_t angle_to_pulse(Joint_t *j) {

    const float TICKS_PER_DEGREE = 1000.0f/120.0f; // sono  1000tick/120
    float pulse_f;

    pulse_f = (float)j->center_pulse + (j->current_angle*TICKS_PER_DEGREE);

    uint32_t pulse = (uint32_t)pulse_f;

    // Clamping di sicurezza per non uscire dal range 1-2 ms
    if (pulse < j->min_pulse) pulse = j->min_pulse;
    if (pulse > j->max_pulse) pulse = j->max_pulse;

    return pulse;
}

void BOBOT_set_angles(float a0, float a1, float a2, float a3, float a4) {
	float angle[5]={a0,a1,a2,a3,a4};

	Joint_t* j;
	for(int i=0;i<NJOINT-1;i++){

		j= &Robot.joints[i];
		j->target_angle=j->inverse*angle[i];

		if(j->inverse==1){
			if(j->target_angle < j->min_angle)j->target_angle=j->min_angle;
			if(j->target_angle > j->max_angle)j->target_angle=j->max_angle;

		}else if(j->inverse==-1){

			if(j->target_angle > j->min_angle*j->inverse)j->target_angle=j->min_angle*j->inverse;
			if(j->target_angle < j->max_angle*j->inverse)j->target_angle=j->max_angle*j->inverse;
		}

	}

	Robot.joints[4].target_angle =  Robot.joints[4].inverse*a4;

	/*
    Robot.joints[0].target_angle = Robot.joints[0].inverse*a0;
    Robot.joints[1].target_angle = Robot.joints[1].inverse*a1;
    Robot.joints[2].target_angle = Robot.joints[2].inverse*a2;
    Robot.joints[3].target_angle = Robot.joints[3].inverse*a3;
    Robot.joints[4].target_angle = Robot.joints[4].inverse*a4;
*/
}

void BOBOT_move_relative(float da0, float da1, float da2, float da3, float da4) {
	float angle[5]={da0,da1,da2,da3,da4};

	Joint_t* j;
	for(int i=0;i<NJOINT-1;i++){

		j= &Robot.joints[i];
		j->target_angle += j->inverse*angle[i];

		if(j->inverse==1){
			if(j->target_angle < j->min_angle)j->target_angle=j->min_angle;
			if(j->target_angle > j->max_angle)j->target_angle=j->max_angle;

		}else if(j->inverse==-1){
			if(j->target_angle > j->min_angle*j->inverse)j->target_angle=j->min_angle*j->inverse;
			if(j->target_angle < j->max_angle*j->inverse)j->target_angle=j->max_angle*j->inverse;
		}
	}

	//Robot.joints[4].target_angle =  Robot.joints[4].inverse*da4;

	/*
    Robot.joints[0].target_angle += Robot.joints[0].inverse*da0;
    Robot.joints[1].target_angle += Robot.joints[1].inverse*da1;
    Robot.joints[2].target_angle += Robot.joints[2].inverse*da2;
    Robot.joints[3].target_angle += Robot.joints[3].inverse*da3;
    Robot.joints[4].target_angle =  Robot.joints[4].inverse*da4;
*/
}



void BOBOT_process() {

    for (int i = 0; i < NJOINT; i++) {
        Joint_t *j = &Robot.joints[i];

        float diff = j->target_angle - j->current_angle;

        if (fabs(diff) > 0.05f) {

            if (diff > 0) j->current_angle += j->step_limit;
            else j->current_angle -= j->step_limit;

            if ((diff > 0 && j->current_angle > j->target_angle) ||
                (diff < 0 && j->current_angle < j->target_angle)) {
                j->current_angle = j->target_angle;
            }
        }
        uint32_t pulse = angle_to_pulse(j);
        __HAL_TIM_SET_COMPARE(j->htim, j->channel, pulse);
    }

    HAL_Delay(DELAY);
}



void BOBOT_inverse_cinematic_polar(float theta,float ro,float quote){
/*
 * theta  [-60°,60*]
 * ro	  =20
	*/
    float theta_rad = theta * 3.14159265f / 180.0f;
    float x = ro * cosf(theta_rad);
    float y = ro * sinf(theta_rad);
    float z = quote;
    BOBOT_inverse_cinematic(x, y, z);
}

void BOBOT_inverse_cinematic(float x, float y, float z) {

    float theta1 = 0.0f;
    float theta2 = 0.0f;
    float theta3 = 0.0f;
    float theta4 = 0.0f;

    // 1. Calcolo angolo base (theta1)
    theta1 = atan2f(y, x);

    // 2. Raggio nel piano XY dal centro della base
    float r = sqrtf(x * x + y * y);

    // 3. Posizione del polso
    // Per mantenere l'effettore finale orizzontale, la Z del polso è uguale
    // alla Z dell'effettore e la posizione radiale arretra della lunghezza L3.
    float r_w = r - L3;
    float z_w = z - D1;

    // Distanza quadratica e lineare dalla spalla al polso
    float D_sq = r_w * r_w + z_w * z_w;
    float D = sqrtf(D_sq);


    if (D > (L1 + L2) || D < fabsf(L1 - L2)) {
        // Il punto non è raggiungibile
        return;
    }

    // 4. Calcolo angolo gomito (theta3)
    // Utilizziamo la configurazione "elbow up" per ridurre la possibilità
    // che il gomito sbatta contro il piano di lavoro. (theta3 < 0)
    float cos_theta3 = (D_sq - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);

    // Saturazione per evitare errori della funzione acosf legati ai float
    if (cos_theta3 > 1.0f) cos_theta3 = 1.0f;
    if (cos_theta3 < -1.0f) cos_theta3 = -1.0f;

    float math_theta3 = -acosf(cos_theta3);

    // 5. Calcolo angolo spalla (theta2)
    float alpha = atan2f(z_w, r_w);
    float cos_beta = (D_sq + L1 * L1 - L2 * L2) / (2.0f * L1 * D);

    // Saturazione
    if (cos_beta > 1.0f) cos_beta = 1.0f;
    if (cos_beta < -1.0f) cos_beta = -1.0f;

    float beta = acosf(cos_beta);

    theta2 = alpha + beta;

    // 6. Calcolo angolo polso (theta4 standard)
    // L'angolo assoluto di L3 rispetto all'orizzontale è theta2 + math_theta3 + math_theta4 = 0
    float math_theta4 = -(theta2 + math_theta3);

    // 7. Applica la tua convenzione: gomito e polso "positivi a scendere"
    // Questo significa invertire il segno matematico standard (positivo = antiorario/a salire)
    theta3 = -math_theta3;
    theta4 = -math_theta4;

    // Convertiamo i radianti in gradi, poiché le librerie math.h in C restituiscono sempre radianti
    float rad2deg = 180.0f / 3.14159265f;
    theta1 *= rad2deg;
    theta2 *= rad2deg;
    theta3 *= rad2deg;
    theta4 *= rad2deg;

    BOBOT_set_angles(theta1, theta2, theta3, theta4, CLOSED);
}




#ifdef __cplusplus
}
#endif
