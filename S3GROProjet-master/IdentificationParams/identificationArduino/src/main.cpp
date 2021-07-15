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

#define MAGPIN          32         // Port numerique pour electroaimant J-16
#define POTPIN          A5          // Port analogique pour le potentiometre

#define PASPARTOUR      64          // Nombre de pas par tour du moteur
#define RAPPORTVITESSE  50          // Rapport de vitesse du moteur

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

uint16_t pulseTime_ = 0;            // temps dun pulse en ms
float pulsePWM_ = 0.1;              // Amplitude de la tension au moteur pour la position[-1,1]
float pulsePWM_angle = 0.1;         //Amplitude de la tension au moteur pour l'angle [-1,1]

float Axyz[3];                      // tableau pour accelerometre
float Gxyz[3];                      // tableau pour giroscope
float Mxyz[3];                      // tableau pour magnetometre

int time = 0;                       //timer pour la loop
int32_t compteur_encodeur = 0;      //Encodeur du moteur

int choix = 0;                      //sert pour le switch case
double fonction = 0;                 //fonction de tests dans la loop
bool goal_position_atteint = false;  //Permet de savoir si la positon est atteinte
bool goal_angle_atteint = false;     //Permet de savoir si l'anlge du pendule est atteinte
double goal_voulu_position = 0;         //Permet de dire la distance voulue
double goal_voulu_angle = 0;            //Permet de dire l'angle voulue
float Potentio_zero = 0;             //permet de savoir la valeur initiale du pendule
float deg = 0;

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
  pid_x.setGains(0.25,0.1 ,0);
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
  pid_q.setEpsilon(0.01);
  pid_q.setPeriod(200);
  pid_q.enable();


  //Defenition du IO pour l'ÉLECTRO-AIMANT
  pinMode(MAGPIN, OUTPUT); 


  Potentio_zero = analogRead(POTPIN);

  goal_voulu_position = 0.7;
}


void loop() {
  //tests.Tests_unitaire();
  //Serial.println(imu_.getTemp());
  //delay(750);
}


/*
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
  

  // mise a jour des chronometres
  timerSendMsg_.update();
  timerPulse_.update();

  switch(choix) 
  {
    case 0: // init de l'électroaimant
      pinMode(MAGPIN, HIGH);
      delay(2000);
     choix = 11;
    break;

    case 1: //squence 1 aller
      //code ici
      fonction = 0.8*sin(5.0*millis());
      AX_.setMotorPWM(0,fonction);
      Serial.println(fonction);
    break;

    case 2: //séquence 2 aller
      //code ici
    break;

    case 3: //séquence 3 aller
      //code ici
    break;

    case 4: //séquence 4 aller
      //code ici
    break;

    case 10: // test du PID de position
    //Serial.println(pulsePWM_);
    pid_x.setGoal(goal_voulu_position);
    pid_x.setGains(5,0,0);
    pid_x.run();
    AX_.setMotorPWM(0,pulsePWM_);
    //Serial.println("sortit");
      break;
    
    case 11: // Tests 2 du PID de position
      //fonction = 0.8*sin(5.0*millis());
      pid_x.setGoal(goal_voulu_position);
      pid_x.setGains(13,0,0);
      pid_x.run();
      //Serial.println(fonction);
      AX_.setMotorPWM(0,pulsePWM_);      
      if(goal_position_atteint)
      { 
       choix = 100;
      }
      
    break;

    case 12:
      //code
      pid_q.setGoal(goal_voulu_angle);
      pid_q.setGains(13,0,0);
      pid_q.run();
      //Serial.println(30);
      //AX_.setMotorPWM(0,pulsePWM_angle);
      //delay(200);
    break;

    case 100: //mets des moteurs à 0. Donc, arret du moteur et de l'électroaimant
      AX_.setMotorPWM(0,0);
      pinMode(MAGPIN, LOW);
    break;
  }//Fin du switch case
}

*/
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

  doc["time"] = millis();
  doc["potVex"] = analogRead(POTPIN);
  doc["encVex"] = vexEncoder_.getCount();
  doc["goal"] = pid_x.getGoal();
  doc["measurements"] = PIDmeasurement();
  doc["voltage"] = AX_.getVoltage();
  doc["current"] = AX_.getCurrent(); 
  doc["pulsePWM"] = pulsePWM_;
  doc["pulseTime"] = pulseTime_;
  doc["inPulse"] = isInPulse_;
  doc["accelX"] = imu_.getAccelX();
  doc["accelY"] = imu_.getAccelY();
  doc["accelZ"] = imu_.getAccelZ();
  doc["gyroX"] = imu_.getGyroX();
  doc["gyroY"] = imu_.getGyroY();
  doc["gyroZ"] = imu_.getGyroZ();
  doc["temp"] = imu_.getTemp();
  doc["isGoal"] = pid_x.isAtGoal();
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
}


// Fonctions pour le PID de position
double PIDmeasurement(){
  double pulse;
  double distance;
  double tour;
  pulse = AX_.readEncoder(0);
  tour = pulse/3200;
  distance = tour * 0.06 * 2 * PI;
  Serial.println(distance);

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

  return deg;
}


void PIDcommand_angle(double cmd_angle){
  pulsePWM_angle = cmd_angle;
}


void PIDgoalReached_angle(){
  goal_angle_atteint = true;
}