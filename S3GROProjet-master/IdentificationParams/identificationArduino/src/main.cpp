/* 
 * GRO 302 - Conception d'un robot mobile
 * Code de démarrage
 * Auteurs: Roundnet
 * date: 24 juin 2021
*/

/*------------------------------ Librairies ---------------------------------*/
#include <LibS3GRO.h>
#include <ArduinoJson.h>
#include <Tests.h> // Vos propres librairies
/*------------------------------ Constantes ---------------------------------*/
using namespace std;

#define BAUD                            115200    // Frequence de transmission serielle
#define UPDATE_PERIODE                  100       // Periode (ms) d'envoie d'etat general

#define MAGPIN                          32        // Port numerique pour electroaimant J-16
#define POTPIN                          A5        // Port analogique pour le potentiometre

#define PASPARTOUR                      64        // Nombre de pas par tour du moteur
#define RAPPORTVITESSE                  50        // Rapport de vitesse du moteur
#define RAYON_ROUE                      0.06      // Diamètres des roues
#define MOTOR_ID                        1         //Permet de choisir l'ID du moteur utiliser
#define GEAR_RATIO                      2         //Permet de choisir un gearRatio en fonction des engrenage choisit
#define FACTEUR_MAGIQUE                 1.1       //Ajoute 10% de distance pour compenser l'arondissement des ratio
#define DISTANCE_AVANT_OBSTACLE         0.095     //Permet de savoir la position que le robot doit prendre pour commencer son oscillation
#define PID_KP_LENT                     1.5
#define PID_KI_LENT                     0.25
#define PID_KD_LENT                     0.2
#define PID_KP_RAPIDE                   13
#define COMPTEUR                        4

//Différent cas pour la séquence du projet du sapin
#define START                           0
#define ATTENTE                         1
#define AVANCE_INITIAL                  2
#define OSCILLATION_DEBUT               3
#define ARRET_OSCILLATION               4
#define PASSE_OBSTACLE                  5
#define AVANCE_ALLER                    6
#define LACHE_SAPIN                     7
#define AVANCE_RETOUR                   8
#define PREND_SAPIN                     9
#define ARRET_TOTAL                     100

/*---------------------------- variables globales ---------------------------*/

ArduinoX AX_;                                     // objet arduinoX
MegaServo servo_;                                 // objet servomoteur
VexQuadEncoder vexEncoder_;                       // objet encodeur vex
IMU9DOF imu_;                                     // objet imu
PID pid_x;                                        // objet PID x
PID pid_q;                                        // objet PID q
Tests tests;

volatile bool shouldSend_ =             false;    // drapeau prêt à envoyer un message
volatile bool shouldRead_ =             false;    // drapeau prêt à lire un message
volatile bool shouldPulse_ =            false;    // drapeau pour effectuer un pulse
volatile bool isInPulse_ =              false;    // drapeau pour effectuer un pulse

SoftTimer timerSendMsg_;                          // chronometre d'envoie de messages
SoftTimer timerPulse_;                            // chronometre pour la duree d'un pulse

uint16_t pulseTime_ =                   0;        // temps dun pulse en ms

int time =                              0;        //timer pour la loop
int32_t compteur_encodeur =             0;        //Encodeur du moteur

int choix =                             START;  //sert pour le switch case

bool goal_position_atteint =            false;    //Permet de savoir si la positon est atteinte
bool goal_angle_atteint =               false;    //Permet de savoir si l'anlge du pendule est atteinte
bool prendre_sapin =                    false;    //Permet de savoir si à la fin de l'arrêt de l'oscillation, il faut prendre ou laisser le sapin
bool sapinLacher =                      false;    //Permet de savoir si le sapin à été laché
bool casZero =                          false;    //Permet de savoir si le cas START est actif
bool oscillation_finis =                false;

double fonction =                       0.0;      //fonction de tests dans la loop
double goal_voulu_angle =               0.0;      //Permet de dire l'angle voulue
double position_depart =                0.0;      //Permet de savoir la position initial du robot
double position_obstacle =              0.5;      //Permet de savoir la position de l'obstacle
double position_depot =                 1.0;      //Permet de savoir la position du dépot du sapin
double distance_ins =                   0.0;      //Permet de savoir la distance instantanné du véhicule pour calculer la vitesse
double distance_old =                   0.0;      //Permet de savoir la distance précédente pour le calcul de la vitesse
double temps_ins =                      0.0;      //Permet de savoir le temps instantanné du véhicule pour calculer la vitesse
double temps_old =                      0.0;      //Permet de savoir le temps précédente pour le calcul de la vitesse

float pulsePWM_ =                       0.1;      // Amplitude de la tension au moteur pour la position[-1,1]
float pulsePWM_angle =                  0.1;      //Amplitude de la tension au moteur pour l'angle [-1,1]
float Axyz[3];                                    // tableau pour accelerometre
float Gxyz[3];                                    // tableau pour giroscope
float Mxyz[3];                                    // tableau pour magnetometre
float Potentio_zero =                   0.0;      //permet de savoir la valeur initiale du pendule
float angle_pendule =                   0.0;      //Permet de savoir l'agle actuelle du pendule
float cur_pos =                         0.0;      //Permet de savoir la position en temps réelle du pendule
float cur_vel =                         0.0;      //Permet de savoir la vitesse en temps réelle du pendule
float cur_angle =                       0.0;      //Permet de savoir l'angle en temps réelle du pendule
int i = 0;

/*------------------------- Prototypes de fonctions -------------------------*/
void timerCallback();
void startPulse();
void endPulse();
void sendMsg(); 
void readMsg();
void serialEvent();
void digitalWrite(uint8_t pin, uint8_t val);

// Fonctions pour le PID de position
double PIDmeasurement();
void PIDcommand(double cmd);
void PIDgoalReached();

// Fonctions pour le PID de l'angle
double PIDmeasurement_angle();
void PIDcommand_angle(double cmd);
void PIDgoalReached_angle();

double Calculangle();

/*---------------------------- fonctions "Main" -----------------------------*/

void setup() {
  Serial.begin(BAUD);               // initialisation de la communication serielle
  AX_.init();                       // initialisation de la carte ArduinoX 
  imu_.init();                      // initialisation de la centrale inertielle
  vexEncoder_.init(2,3);            // initialisation de l'encodeur VEX
  // attache de l'interruption pour encodeur vex
  attachInterrupt(vexEncoder_.getPinInt(), []{vexEncoder_.isr();}, FALLING);
  
  // Chronometre envoie message
  timerSendMsg_.setDelay(UPDATE_PERIODE);
  timerSendMsg_.setCallback(timerCallback);
  timerSendMsg_.enable();

  // Chronometre duration pulse
  timerPulse_.setCallback(endPulse);
  
  // Initialisation du PID de position
  pid_x.setGains(PID_KP_LENT, PID_KI_LENT ,PID_KD_LENT);
  // Attache des fonctions de retour
  pid_x.setMeasurementFunc(PIDmeasurement);
  pid_x.setCommandFunc(PIDcommand);
  pid_x.setAtGoalFunc(PIDgoalReached);
  pid_x.setEpsilon(0.01);
  pid_x.setPeriod(100);
  //pid_x.enable();

  // Initialisation du PID d'angle
  pid_q.setGains(5, 0 ,0);
  // Attache des fonctions de retour
  pid_q.setMeasurementFunc(PIDmeasurement_angle);
  pid_q.setCommandFunc(PIDcommand_angle);
  pid_q.setAtGoalFunc(PIDgoalReached_angle);
  pid_q.setEpsilon(5);
  pid_q.setPeriod(100);
  //pid_q.enable();


  //Defenition du IO pour l'ÉLECTRO-AIMANT
  pinMode(MAGPIN, OUTPUT); 

  //Initialise l'état initiale du pendule comme étant 0 degree
  Potentio_zero = analogRead(POTPIN);
}


// Boucle principale (infinie) 
void loop() {

  if(shouldRead_){
    readMsg();
  }
  if(shouldSend_){
    sendMsg();
  }
  if(shouldPulse_){
    startPulse();
  }
  

  

  // Mise à jour des chronometres
  timerSendMsg_.update();
  timerPulse_.update();


  switch(choix) 
  {
    case ATTENTE : //Cas pour l'activation de l'électroaimant et attente du commencement de la séquence
    //Serial.println("Case 1");
      pinMode(MAGPIN, HIGH);
      AX_.setMotorPWM(MOTOR_ID, 0);
      //delay(3000);
      //choix = START;

    break;

    case START: //Cas pour l'initialisation des variables
    //Serial.println("Case 2");
      casZero = true;
      goal_voulu_angle = 60;
      goal_position_atteint = false;
      goal_angle_atteint = false;
      pid_x.setGoal(position_obstacle-DISTANCE_AVANT_OBSTACLE);
      pinMode(MAGPIN, HIGH);
      i = 0;

      sapinLacher = false;
      
      choix = AVANCE_INITIAL;
      AX_.setMotorPWM(MOTOR_ID, 0.0);  
      pid_x.enable();
    
    break;

    case AVANCE_INITIAL: //Cas pour aller  la position initiale avant d'osciller
    //Serial.println("Case 3");
      casZero = false;
      
      pid_x.setGains(PID_KP_LENT, PID_KI_LENT ,PID_KD_LENT);
      
      
      
      AX_.setMotorPWM(MOTOR_ID, pulsePWM_);
      //Serial.println("case = pid_x.getGoal()");
      //Serial.println(pid_x.getGoal());


     if(goal_position_atteint)
      {
        choix = OSCILLATION_DEBUT;
        goal_position_atteint = false;
        pid_x.setGoal(position_depot);
        pid_q.enable(); 
        pid_x.enable();
        //Serial.println("avance initial fini");
      }

    break;

    case OSCILLATION_DEBUT: //Cas pour osciller le pendule a environ 60 degree pour passer par dessus l'obstacle
    //Serial.println("Case 4");
      if(!oscillation_finis)
      {
      fonction = 0.9*sin(5.0*(millis()/1000.0));
      pid_q.setGoal(goal_voulu_angle);
      AX_.setMotorPWM(MOTOR_ID,fonction);
      }

      if(goal_angle_atteint)
      {
        oscillation_finis = true;
        pid_x.setGains(PID_KP_RAPIDE,0,0);
        AX_.setMotorPWM(MOTOR_ID, pulsePWM_);

        if(goal_position_atteint)
        {
          oscillation_finis = false;
          goal_angle_atteint = false;
          choix = AVANCE_ALLER;
          goal_position_atteint = false;
          pid_x.enable();
        }

        /*
          choix = PASSE_OBSTACLE;
          goal_angle_atteint = false;
          pid_q.disable();
          pid_x.enable();
          */
      }

    break;

      /*
    case PASSE_OBSTACLE: //Cas pour passer par dessus l'obstacle
      pid_x.setGoal(position_obstacle);
      pid_x.setGains(PID_KP_RAPIDE,0,0);
      
      AX_.setMotorPWM(MOTOR_ID, pulsePWM_);

      if(goal_position_atteint)
      {
        choix = AVANCE_ALLER;
        goal_position_atteint = false;
        pid_x.enable();
        
      }

    break;
    */

    case AVANCE_ALLER : //Cas pour se rendre au dessus du panier à sapin
    //Serial.println("Case 5");
    
        // Serial.println("pos depot");
        // Serial.println(position_depot);
        // Serial.println(" pos cur");
        // Serial.println(cur_pos);

      pid_x.setGoal(position_depot);
      pid_x.setGains(PID_KP_LENT, PID_KI_LENT ,PID_KD_LENT);

      AX_.setMotorPWM(MOTOR_ID, pulsePWM_);

      // if(millis()>25000)
      // {
      //   Serial.println(" Timeout");
      //   choix = ARRET_TOTAL;
      //   goal_position_atteint = false;
      //   pid_q.enable();
      //   pid_x.enable();
      // } 

      if(goal_position_atteint)
      {
         
        choix = ARRET_OSCILLATION;
        goal_position_atteint = false;
        pid_q.enable();
        pid_x.enable();
      }

    break;

    case ARRET_OSCILLATION: //Cas pour arreter le pendule vers 0 degree au dessus du panier
      //On fait un sinus qui tend vers 0 ???---------------------------------------------------------------
      //problème avec le case 
      pid_q.setGains(5,0,0);
      pid_q.setGoal(0);
     
      AX_.setMotorPWM(MOTOR_ID,pulsePWM_angle);

      //Serial.println(angle_pendule);
      pid_q.enable();

      if(goal_angle_atteint)
      {
        //i++;
        delay(1000);
        pid_q.enable();
        choix = LACHE_SAPIN;
        pid_x.enable();
        goal_angle_atteint = false;
      }
    /*  else
      {
        i = 0;
      }

      if(i == COMPTEUR)
      {
        //Serial.println(" allo");
        choix = LACHE_SAPIN;
        pid_x.enable();
        /*
        if(prendre_sapin == true)
        {
          choix = PREND_SAPIN;
        }
        else
        {
          choix = LACHE_SAPIN;
        }
        
        goal_angle_atteint = false;
        
      }*/

    break;

    case LACHE_SAPIN: //Cas pour lacher le pendule dans le panier
      sapinLacher = true;

      AX_.setMotorPWM(MOTOR_ID,0);
      pinMode(MAGPIN, LOW);
      delay(500);

      choix = AVANCE_RETOUR;
      pid_x.enable();

    break;

    case AVANCE_RETOUR: //Cas de passer par dessus l'obstacle pour le retour
      pid_x.setGoal(position_depart);
      pid_x.setGains(PID_KP_LENT, PID_KI_LENT ,PID_KD_LENT);
      AX_.setMotorPWM(MOTOR_ID,pulsePWM_);
      if(goal_position_atteint)
      {
        goal_position_atteint = false;
        prendre_sapin = true;
        pid_x.enable();
        choix = ARRET_OSCILLATION;
      }
      
      

    break;

    case PREND_SAPIN: //Cas d'arret du pendule au dessus du sapin = 0 degree
        choix = ARRET_TOTAL;
        goal_angle_atteint = false;
        prendre_sapin = false;
        // AX_.resetEncoder(MOTOR_ID);
        pinMode(MAGPIN, HIGH);

    break;

    case ARRET_TOTAL: //Cas pour mettre les moteurs à 0. Donc, arret du moteur et de l'électroaimant
      AX_.setMotorPWM(MOTOR_ID,0);
      pinMode(MAGPIN, LOW);
      
    break;


    

  }//Fin du switch case
 
 //PID update
  pid_x.run();
  pid_q.run();

  Calculangle();

}

/*---------------------------Definition de fonctions ------------------------*/

void serialEvent(){shouldRead_ = true;}

void timerCallback(){shouldSend_ = true;}

void startPulse(){
  /* Demarrage d'un pulse */
  timerPulse_.setDelay(pulseTime_);
  timerPulse_.enable();
  timerPulse_.setRepetition(1);
  AX_.setMotorPWM(0, pulsePWM_);
  AX_.setMotorPWM(1, pulsePWM_);
  shouldPulse_ = false;
  isInPulse_ = true;
}

void endPulse(){
  /* Rappel du chronometre */
  AX_.setMotorPWM(0,0);
  AX_.setMotorPWM(1,0);
  timerPulse_.disable();
  isInPulse_ = false;
}

void sendMsg(){
  /* Envoit du message Json sur le port seriel */
  StaticJsonDocument<500> doc;
  // Elements du message

  doc["time"]      = (millis()/1000.0);
  doc["goal"]      = pid_x.getGoal();
  doc["pulsePWM"]  = pulsePWM_;
  doc["cur_vel"]   = cur_vel;
  doc["cur_pos"]   = cur_pos;
  doc["cur_angle"] = cur_angle;
  doc["Etat"]      = choix;
  doc["actualTime"] = pid_x.getActualDt();
  doc["position_obstacle"] = position_obstacle;
  doc["position_depot"] = position_depot;
  doc["sapin_lacher"] = sapinLacher;
  doc["casZero"] = casZero;

  //doc["potVex"] = analogRead(POTPIN);
  //doc["encVex"] = vexEncoder_.getCount();
  //doc["measurements"] = PIDmeasurement();
  //doc["voltage"] = AX_.getVoltage();
  //doc["current"] = AX_.getCurrent(); 
  //doc["measurements"] = PIDmeasurement();
  //doc["voltage"] = AX_.getVoltage();
  //doc["current"] = AX_.getCurrent(); 
  //doc["pulseTime"] = pulseTime_;
  //doc["inPulse"] = isInPulse_;
  //doc["accelX"] = imu_.getAccelX();
  //doc["accelY"] = imu_.getAccelY();
  //doc["accelZ"] = imu_.getAccelZ();
  //doc["gyroX"] = imu_.getGyroX();
  //doc["gyroY"] = imu_.getGyroY();
  //doc["gyroZ"] = imu_.getGyroZ();
  //doc["temp"] = imu_.getTemp(); //Température
  //doc["isGoal"] = pid_x.isAtGoal();

  // Serialisation
  serializeJson(doc, Serial);
  // Envoit
  //Serial.println();
  shouldSend_ = false;
}

void readMsg(){
  // Lecture du message Json
  StaticJsonDocument<500> doc;
  JsonVariant parse_msg;

  // Lecture sur le port Seriel
  DeserializationError error = deserializeJson(doc, Serial);
  shouldRead_ = false;

  // Si erreur dans le message
  if (error) {
    //Serial.print("deserialize() failed: ");
    //Serial.println(error.c_str());
    return;
  }
  
  //À LAISSER DANS LA PREMIÈRE FENETRE QT------------------------------------------------------------À FAIRE-------------------------------------

  // Analyse des éléments du message message
  parse_msg = doc["pulsePWM"];
  if(!parse_msg.isNull()){
     pulsePWM_ = doc["pulsePWM"].as<float>();
  }

  parse_msg = doc["pulseTime"];
  if(!parse_msg.isNull()){
     pulseTime_ = doc["pulseTime"].as<float>();
  }

  parse_msg = doc["pulse"];
  if(!parse_msg.isNull()){
     shouldPulse_ = doc["pulse"];
  }

  parse_msg = doc["setGoal"];
  if(!parse_msg.isNull()){
    pid_x.disable();
    pid_x.setGains(doc["setGoal"][0], doc["setGoal"][1], doc["setGoal"][2]);
    pid_x.setEpsilon(doc["setGoal"][3]);
    pid_x.setGoal(doc["setGoal"][4]);
    pid_x.enable();
  }

  parse_msg = doc["Start"];
  if(!parse_msg.isNull())
  {
    choix = START;
  }

  parse_msg = doc["Stop"];
  if(!parse_msg.isNull())
  {
    //pinMode(MAGPIN, LOW);
    //AX_.setMotorPWM(MOTOR_ID, 0);
    choix = ARRET_TOTAL;
  }

  parse_msg = doc["position_obstacle"];
  if(!parse_msg.isNull())
  {
    position_obstacle = (doc["Distance"][0]);
  }

  parse_msg = doc["position_depot"];
  if(!parse_msg.isNull())
  {
    position_depot = (doc["Distance"][1]);
  }  
  //--------------------------------------------------------------------------------------------------------------
}


// Fonctions pour le PID
double vitesse(){

  double distancetot = 0;
  double dt = 0;

  temps_ins = millis()/1000.0;//temps en sec

  PIDmeasurement();// calcul de la variable distance_ins
  distancetot = cur_pos-distance_old;
  dt = temps_ins-temps_old;
  cur_vel = distancetot /dt;

  //Serial.println("vitesse:");
  //Serial.println(cur_vel);

  distance_old = cur_pos;
  temps_old = temps_ins;

  return cur_vel;
}

// Fonctions pour le PID de position
double PIDmeasurement(){
  double pulse;
  double distance;
  double tour;

  pulse = AX_.readEncoder(MOTOR_ID);

  tour = pulse/(3200.0*GEAR_RATIO);
  distance = (tour * RAYON_ROUE * 2.0 * PI ) / FACTEUR_MAGIQUE;

  cur_pos = distance;

  return distance;
}

void PIDcommand(double cmd){
  pulsePWM_ = cmd;
}

void PIDgoalReached(){
  goal_position_atteint = true;
}


/////////////////////////////////////////////////////////

void PIDAngle()
{
  
}

// Fonctions pour le PID d'angle
double PIDmeasurement_angle(){
  
  //angle_pendule = (analogRead(POTPIN)-Potentio_zero)*(180.0/880.0);

  //cur_angle = angle_pendule;

  //Serial.print("angle_pendule");
  //Serial.println(angle_pendule);

  return Calculangle();
}

double Calculangle()
{
  angle_pendule = (analogRead(POTPIN)-Potentio_zero)*(180.0/880.0);

  cur_angle = angle_pendule;

  return angle_pendule;
}


void PIDcommand_angle(double cmd_angle){
  pulsePWM_angle = cmd_angle;
  //Serial.println("cmd_angle");
  //Serial.println(cmd_angle);
}


void PIDgoalReached_angle(){
  goal_angle_atteint = true;
}