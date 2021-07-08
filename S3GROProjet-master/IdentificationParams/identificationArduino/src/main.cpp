/* 
 * GRO 302 - Conception d'un robot mobile
 * Code de démarrage
 * Auteurs: Roundnet
 * date: 24 juin 2021
*/

/*------------------------------ Librairies ---------------------------------*/
#include <LibS3GRO.h>
#include <ArduinoJson.h>
#include <libExample.h> // Vos propres librairies
/*------------------------------ Constantes ---------------------------------*/

#define BAUD            115200      // Frequence de transmission serielle
#define UPDATE_PERIODE  100         // Periode (ms) d'envoie d'etat general

#define MAGPIN          32         // Port numerique pour electroaimant
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

volatile bool shouldSend_ = false;  // drapeau prêt à envoyer un message
volatile bool shouldRead_ = false;  // drapeau prêt à lire un message
volatile bool shouldPulse_ = false; // drapeau pour effectuer un pulse
volatile bool isInPulse_ = false;   // drapeau pour effectuer un pulse

SoftTimer timerSendMsg_;            // chronometre d'envoie de messages
SoftTimer timerPulse_;              // chronometre pour la duree d'un pulse

uint16_t pulseTime_ = 0;            // temps dun pulse en ms
float pulsePWM_ = 0;                // Amplitude de la tension au moteur [-1,1]


float Axyz[3];                      // tableau pour accelerometre
float Gxyz[3];                      // tableau pour giroscope
float Mxyz[3];                      // tableau pour magnetometre

int time = 0;                       //timer pour la loop
int32_t compteur_encodeur = 0;      //Encodeur du moteur

int choix = -1;                      //sert pour le switch case

/*------------------------- Prototypes de fonctions -------------------------*/
void timerCallback();
void startPulse();
void endPulse();
void sendMsg(); 
void readMsg();
void serialEvent();
void digitalWrite(uint8_t pin, uint8_t val);

// Fonctions pour le PID
double PIDmeasurement();
void PIDcommand(double cmd);
void PIDgoalReached();

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
  
  // Initialisation du PID
  pid_x.setGains(0.25,0.1 ,0);
  // Attache des fonctions de retour
  pid_x.setMeasurementFunc(PIDmeasurement);
  pid_x.setCommandFunc(PIDcommand);
  pid_x.setAtGoalFunc(PIDgoalReached);
  pid_x.setEpsilon(0.001);
  pid_x.setPeriod(200);

  //Defenition du IO pour l'ÉLECTRO-AIMANT
  pinMode(MAGPIN, OUTPUT); 
  }

/* Boucle principale (infinie)*/
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

  //----------------------------------SECTION MOTEUR-------------------------------------------//

  //Allume les moteur
  //shouldPulse_ = false;

  //Éteindre les moteurs
  //shouldPulse_ = true;

  //-------------------------------SECTION ÉLECTRO-AIMANT-------------------------------------//

  //TEST : Allumer et éteindre l'électro-aimant
  /*
  pinMode(MAGPIN, HIGH); // Activation electroAimant
  if(time > 10000)
  {
      pinMode(MAGPIN, LOW); // Desactivation electroAimant
      time = 0;
  }
  else
  {
    time += 1;
    //delay(1000); //Délai de 1 secondes
  }
  */


  //----------------------------------SECTION ENCODEUR------------------------------------------//

  compteur_encodeur = AX_.readEncoder(0);
  Serial.println();


  //--------------------------SECTION CAPTEUR TENSION/COURANT----------------------------------//

  //Voir section void sendMsg();


  //------------------------------SECTION ENCODEUR OPTIQUE-------------------------------------//

  //vexEncoder_.init(2,3); // initialisation de l'encodeur VEX
  // attache de l'interruption pour encodeur vex
  //attachInterrupt(vexEncoder_.getPinInt(),[]{vexEncoder_.isr();},FALLING);
  //Voir section void sendMsg(); pour la fonction en dessous 
  //doc["encVex"] = vexEncoder_.getCount();


  //-----------------------------SECTION POTENTIOMÈTRE GROVE-----------------------------------//

  //Voir section void sendMsg(); pour la fonction en dessous
  //pour doc["potVex"] = analogRead(POTPIN);


  //--------------------------SECTION CENTRALE INERTIELLE GROVE--------------------------------//

  //Voir section void sendMsg();  pour pour les fonctions en dessous
  //doc["accelZ"] = imu_.getAccelZ();
  //doc["gyroX"] = imu_.getGyroX();


  //-------------------------------------SECTION PID-------------------------------------------//

  
  switch(choix) 
  {
    case -1:
    pinMode(MAGPIN, HIGH);
    choix = 0;
    break;

    case 0:
    pid_x.setGoal(0.7);
    pid_x.setGains(13,0,0);
    pid_x.run();
    AX_.setMotorPWM(0,pulsePWM_);//changer valeur vitesse avec MG
    
    break;
    
    case 1:
      //code
      break;

    case 2:
      //code
      break;
  }
  // mise à jour du PID
  pid_x.run();


  //-------------------------------------SECTION TESTS------------------------------------------//
  
  //TEST #1 : Permet d'avoir en console la valeur de tous les capteurs.
  //sendMsg();
  //readMsg();

  //TEST #2 : Fait avancer le moteur 0 à une vitesse de 0.1
  AX_.setMotorPWM(0,1);
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


// Fonctions pour le PID
double PIDmeasurement(){
  double pulse;
  double distance;
  double tour;
  pulse = AX_.readEncoder(0);
  tour = pulse/3200;
  distance = tour * (60/1000) * 2 * PI;

  return distance;
}
void PIDcommand(double cmd){
  pulsePWM_= cmd;
}
void PIDgoalReached(){
  // To do
}