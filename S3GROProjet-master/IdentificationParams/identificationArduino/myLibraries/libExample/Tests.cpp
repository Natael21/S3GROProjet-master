/*
Exemple de librairie pouvant etre ajoute au projet
*/
#include <Tests.h>

// Class constructor
MyClass::MyClass(){
    
}

// Class desstructor
MyClass::~MyClass(){
    
}

// Public Functions
void MyClass::Tests_unitaire(){


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

  //compteur_encodeur = AX_.readEncoder(0);
  //Serial.println();


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
  pid_x.run();
}

// Private Functions
void MyClass::myPrivateFunction(){

}

// Protected Functions
void MyClass::myProtectedFunction(){

}