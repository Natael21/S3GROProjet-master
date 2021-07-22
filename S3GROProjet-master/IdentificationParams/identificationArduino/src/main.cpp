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

#define BAUD            115200      // Frequence de transmission serielle
#define UPDATE_PERIODE  100         // Periode (ms) d'envoie d'etat general

#define MAGPIN          32          // Port numerique pour electroaimant J-16
#define POTPIN          A5          // Port analogique pour le potentiometre

#define PASPARTOUR      64          // Nombre de pas par tour du moteur
#define RAPPORTVITESSE  50          // Rapport de vitesse du moteur
#define RAYON_ROUE   0.06        // Diamètres des roues
#define MOTOR_ID        1
#define GEAR_RATIO      2
#define FACTEUR_MAGIQUE 1.1

/*---------------------------- variables globales ---------------------------*/

ArduinoX AX_;                       // objet arduinoX
MegaServo servo_;                   // objet servomoteur
VexQuadEncoder vexEncoder_;         // objet encodeur vex
IMU9DOF imu_;                       // objet imu
PID pid_x;                           // objet PID x
PID pid_q;                           // objet PID q
Tests tests;

volatile bool shouldSend_ = false;  // drapeau prêt à envoyer un message
volatile bool shouldRead_ = false;  // drapeau prêt à lire un message
volatile bool shouldPulse_ = false; // drapeau pour effectuer un pulse
volatile bool isInPulse_ = false;   // drapeau pour effectuer un pulse

SoftTimer timerSendMsg_;            // chronometre d'envoie de messages
SoftTimer timerPulse_;              // chronometre pour la duree d'un pulse

uint16_t pulseTime_ = 0;             // temps dun pulse en ms
float pulsePWM_ = 0.1;               // Amplitude de la tension au moteur pour la position[-1,1]
float pulsePWM_angle = 0.1;          //Amplitude de la tension au moteur pour l'angle [-1,1]

float Axyz[3];                       // tableau pour accelerometre
float Gxyz[3];                       // tableau pour giroscope
float Mxyz[3];                       // tableau pour magnetometre

int time = 0;                        //timer pour la loop
int32_t compteur_encodeur = 0;       //Encodeur du moteur

int choix = 0;                  //sert pour le switch case
//int choix = 10000;                  //sert pour le switch case
double fonction = 0;                 //fonction de tests dans la loop
bool goal_position_atteint = false;  //Permet de savoir si la positon est atteinte
bool goal_angle_atteint = false;     //Permet de savoir si l'anlge du pendule est atteinte
double goal_voulu_position_aller = 0;//Permet de dire la distance voulue pour l'aller
double goal_voulu_position_retour = 0;//Permet de dire la distance voulue pour le retour
double goal_voulu_angle = 0;         //Permet de dire l'angle voulue
double distance_oscillation = 0;     //Permet savoir l'endroit d'oscillation
float Potentio_zero = 0;             //permet de savoir la valeur initiale du pendule
float deg = 0;                       //Permet de savoir l'agle actuelle du pendule
float cur_pos = 0;
float cur_vel = 0;
float cur_angle = 0;
double distance_ins;
double distance_old = 0.0;
double temps_ins;
double temps_old = 0.0;
double goal_voulu_position_aller_cas2 = 0;

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
  pid_x.setGains(1.5, 0.25 ,0.2);
  // Attache des fonctions de retour
  pid_x.setMeasurementFunc(PIDmeasurement);
  pid_x.setCommandFunc(PIDcommand);
  pid_x.setAtGoalFunc(PIDgoalReached);
  pid_x.setEpsilon(0.01);
  pid_x.setPeriod(200);
  pid_x.enable();

    // Initialisation du PID d'angle
  pid_q.setGains(0.25,0.1 ,0);
  // Attache des fonctions de retour
  pid_q.setMeasurementFunc(PIDmeasurement_angle);
  pid_q.setCommandFunc(PIDcommand_angle);
  pid_q.setAtGoalFunc(PIDgoalReached_angle);
  pid_q.setEpsilon(5);
  pid_q.setPeriod(200);
  pid_q.enable();


  //Defenition du IO pour l'ÉLECTRO-AIMANT
  pinMode(MAGPIN, OUTPUT); 


  Potentio_zero = analogRead(POTPIN);

  //pid_x.setGoal(0.6);
  //pid_x.setGains(1.5, 0.25 ,0.2);
}

/*
void loop() {
  //tests.Tests_unitaire();
  //Serial.println(imu_.getTemp());
  //delay(750);

    //pinMode(MAGPIN, HIGH);

  if(pid_x.isAtGoal())
  {
      AX_.setMotorPWM(MOTOR_ID, 0);
  }
  else
  {
    AX_.setMotorPWM(MOTOR_ID, pulsePWM_);
  }

  pid_x.run(); 
}
*/



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
    case 0: // initialisation des variables
      pinMode(MAGPIN, HIGH);
      delay(3000);
      goal_voulu_position_aller = 0.4;
      goal_voulu_position_aller_cas2 = 0.3;
      goal_voulu_position_retour = -0.7;
      goal_voulu_angle = -60;
      distance_oscillation = 0;
      goal_position_atteint = false;
      goal_angle_atteint = false;
      choix = 1;
    break;

    case 1: //squence 1 aller (oscillation du pendule)
      fonction = 0.9*sin(5.0*(millis()/1000.0))+distance_oscillation;
      //fonction = 0.8*sin(5.0*(millis()/1000.0))+0.5;
      pid_q.setGoal(goal_voulu_angle);
      pid_q.run();
      AX_.setMotorPWM(MOTOR_ID,fonction);
      //AX_.setMotorPWM(MOTOR_ID,pulsePWM_angle);
      //Serial.print("sin = ");
      //Serial.println(fonction);

      if(goal_angle_atteint)
      {
        choix = 2;
        goal_angle_atteint = false;
        pid_x.setGoal(goal_voulu_position_aller);
        pid_x.setGains(13,0,0);
        Serial.println("cas 1 finis");
      }
    break;

    case 2: //séquence 2 aller (passer par dessus l'obstacle)
      AX_.setMotorPWM(MOTOR_ID, pulsePWM_);
      pid_x.run();
      if(goal_position_atteint)
      {
        choix = 15;
        goal_position_atteint = false;
        pid_x.setGoal(goal_voulu_position_aller_cas2);
        pid_x.setGains(1.5, 0.25, 0.2);
        Serial.println("cas 2 finis");
      }
    break;

    case 15 :
      AX_.setMotorPWM(MOTOR_ID, pulsePWM_);
      pid_x.run();
      //Serial.println("case 2 entree");
      if(goal_position_atteint)
      {
        choix = 100;
        goal_position_atteint = false;
        Serial.println("cas 15 finis");
      }

      break;

    case 3: //séquence 3 aller (arret du pendule vers 0 degree)
      pid_q.setGains(5,0,0);
      pid_q.setGoal(0);
      pid_q.run();
      AX_.setMotorPWM(MOTOR_ID,pulsePWM_angle);
      if(goal_angle_atteint)
      {
        choix = 4;
        goal_angle_atteint = false;
      }
    break;

    case 4: //séquence 4 aller (lache le pendule dans le panier)
      AX_.setMotorPWM(MOTOR_ID,0);
      pinMode(MAGPIN, LOW);
      delay(750);
      choix = 6;
    break;

    case 5: //séquence 5 aller (passer par dessus l'obstacle pour le retour)
      pid_x.setGoal(goal_voulu_position_retour);
      pid_x.setGains(5,0,0);
      pid_x.run();
      AX_.setMotorPWM(MOTOR_ID,pulsePWM_);
      if(goal_position_atteint)
      {
        choix = 6;
        goal_position_atteint = false;
      }
    break;

    case 6: //séquence 6 aller (arret du pendule au dessus du sapin = 0 degree)
      pid_q.setGains(5,0,0);
      pid_q.setGoal(0);
      pid_q.run();
      AX_.setMotorPWM(MOTOR_ID,pulsePWM_angle);
      //Serial.println(fonction);
      if(goal_angle_atteint)
      {
        choix = 0;
        goal_angle_atteint = false;
      }
    break;

    case 10: // Test du PID de position
    //Serial.println(pulsePWM_);
    pid_x.setGoal(goal_voulu_position_aller);
    pid_x.setGains(5,0,0);
    pid_x.run();
    AX_.setMotorPWM(MOTOR_ID,pulsePWM_);
    //Serial.println("sortit");
      break;
    
    case 11: // Test 2 du PID de position
      //fonction = 0.8*sin(5.0*millis());
      pid_x.setGoal(goal_voulu_position_aller);
      pid_x.setGains(13,0,0);
      pid_x.run();
      //Serial.println(fonction);
      AX_.setMotorPWM(MOTOR_ID,pulsePWM_);      
      if(goal_position_atteint)
      { 
       choix = 100;
      }
      
    break;

    case 12: // Test du PID du pendule
      //code
      pid_q.setGoal(goal_voulu_angle);
      pid_q.setGains(13,0,0);
      pid_q.run();
      //Serial.println(30);
      //AX_.setMotorPWM(0,pulsePWM_angle);
      //delay(200);
    break;

    case 100: //mets des moteurs à 0. Donc, arret du moteur et de l'électroaimant
      AX_.setMotorPWM(MOTOR_ID,0);
      pinMode(MAGPIN, LOW);
    break;
  }//Fin du switch case

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
  //doc["potVex"] = analogRead(POTPIN);
  //doc["encVex"] = vexEncoder_.getCount();
  doc["goal"]      = pid_x.getGoal();
  //doc["measurements"] = PIDmeasurement();
  //doc["voltage"] = AX_.getVoltage();
  //doc["current"] = AX_.getCurrent(); 
  doc["pulsePWM"]  = pulsePWM_;
  doc["Etat"]      = choix;
  doc["cur_vel"]   = cur_vel;
  doc["cur_pos"]   = cur_pos;
  doc["cur_angle"] = cur_angle;
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
  doc["actualTime"] = pid_x.getActualDt();

  // Serialisation
  serializeJson(doc, Serial);
  // Envoit
  Serial.println();
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
    Serial.print("deserialize() failed: ");
    Serial.println(error.c_str());
    return;
  }
  
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
    choix = doc["Start"];
  }

  parse_msg = doc["Stop"];
  if(!parse_msg.isNull())
  {
    pinMode(MAGPIN, LOW);
    AX_.setMotorPWM(MOTOR_ID,0);
    doc["time"] = 0;
    choix = doc["Stop"];
  }
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
// Fonctions pour le PID d'angle
double PIDmeasurement_angle(){
  
  deg = (analogRead(POTPIN)-Potentio_zero)*(180.0/880.0);

  cur_angle = deg;

  //Serial.print("deg");
  //Serial.println(deg);

  return deg;
}


void PIDcommand_angle(double cmd_angle){
  pulsePWM_angle = cmd_angle;
}


void PIDgoalReached_angle(){
  goal_angle_atteint = true;
}