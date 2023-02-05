// #define TINKERCAD 1

#include "LiquidCrystal_I2C.h"

#ifndef TINKERCAD 
#include "EmonLib.h"
#endif 

const bool SIMULATION_FOR_DEBUG = false;

unsigned long REQUEST_DELAY_MS = 3000;
const double THEORICAL_MAX_LOAD_W = 2000;
const double MAX_LOAD_REQUEST_W = 750;
const double MIN_LOAD_REQUEST_FOR_START = 150;
const double MIN_LOAD_REQUEST_FOR_STOP = 100;
const double POWER_INJECTION_TARGET_W = 15;

int sct013_pin = 1;
int basseTensionAC_pin = 2;
int ledOptocoupleur_pin = 9; 
int button_pin = 3; 

double powerRequest_W = 0; 
double lastpowerRequest_W = 0;
double basePowerRequestForSimulation_W = 0;
int ledRequest = 0;
int lastLedRequest = 0;
double baseLedRequestForSimulation = 0;

double injectedEnergy_Wh = 0.0; 
double consumedEnergy_Wh = 0.0;
double lastLoopInjectedEnergy_Ws = 0;
double lastLoopConsumedEnergy_Ws = 0;
int injectedPower_W = 0;
int debug_injectedPower = -1;
int simulatedPower_W = 0;

unsigned long lastLoopTime_ms;
unsigned long lastAppliedRequestTime_ms = 0;
bool hasRecentRequest = false;
int loopCount=0;
int screenNumber = 1;

#ifndef TINKERCAD 
EnergyMonitor emon1;
#endif
LiquidCrystal_I2C lcd(0x27,20,4); 

void setup()
{
  pinMode(ledOptocoupleur_pin, OUTPUT);
  pinMode(button_pin, INPUT_PULLUP);

  Serial.begin(9600);
  
  int sct013_calibration = 46;
  double basseTensionAC_calibration = 200;
    
#ifndef TINKERCAD 
  emon1.current(sct013_pin, sct013_calibration);    
  emon1.voltage(basseTensionAC_pin, basseTensionAC_calibration, 1.7);  
#endif
  lcd.init();                      
  lcd.backlight();

  // init lastLoopTime_ms pour première loop
  lastLoopTime_ms = millis();  
  
}
/*
   * mesure
   * calcul consigne
   * application consigne
   * attente
   * loop
  */ 
void loop(){
  loop_directTarget();
  //loop_incrementalTracking();
  //loop_debug();
}

void loop_directTarget()
{

#ifndef TINKERCAD 
  emon1.calcVI(20,2000); 
  injectedPower_W = -1 * int(emon1.realPower);
#endif
  // faire durer la boucle au moins 0.5 seconde
  delay(max(500 - (millis() - lastLoopTime_ms), 0));
  long lastLoopDuration_ms = (millis() - lastLoopTime_ms);
  

  // Pour debug, forcer la valeur de injectedPower_W depuis l'interface série
  // Pour sortir du débug, envoyer "-1".
  // si la valeur n'est pas multiple de 10, on ajoute du bruit
  if(Serial.available() > 0) { debug_injectedPower = double(Serial.parseInt());}
  if(debug_injectedPower != -1) { 
    injectedPower_W = debug_injectedPower; 
    if(debug_injectedPower%10 != 0) { injectedPower_W = injectedPower_W + (random(50)-25); }
  }

  // Pour debug (DEBUG==true), simuler une charge qui suit la consigne
  injectedPower_W = injectedPower_W - simulateLoadFromPowerRequest();
  
  // Traitement / asservissement
  // Seulement si la dernière consigne a été appliquée depuis plus de 3 secondes
  hasRecentRequest = ( millis()-lastAppliedRequestTime_ms < REQUEST_DELAY_MS );
  if( ! hasRecentRequest ) {

    // si injectedPower_W est positif
    //   alors on augmente la consigne avec cette puissance (en valeur absolue)
    //   en l'occurence on soustrait cette valeur negative -> augmentation
    // si injectedPower_W est negatif
    //   alors on baisse la consigne avec cette puissance 
    //   en l'occurence on soustrait cette valeur positive -> baisse
    // en résumé, on applique toujours la même formule :
    double delta_W = injectedPower_W - POWER_INJECTION_TARGET_W;
    powerRequest_W = powerRequest_W + (delta_W / 2);
    
    // si la consigne est superieure au max de la charge, on plafonne
    powerRequest_W = min(powerRequest_W, MAX_LOAD_REQUEST_W);
    
    // si la consigne est inferieure à la puissance de declechement ET que la charge n'est pas allumée, on met à zero 
    // si la consigne est inferieure à la puissance de coupure on met à zero  
    bool isLoadActive = (lastpowerRequest_W > 0);
    if( powerRequest_W < MIN_LOAD_REQUEST_FOR_START && !isLoadActive){ powerRequest_W = 0; }
    if( powerRequest_W < MIN_LOAD_REQUEST_FOR_STOP){ powerRequest_W = 0; }

    // Application de la consigne
    // (seulement si elle est différente de la précédente)
    if( powerRequest_W != lastpowerRequest_W ){
      basePowerRequestForSimulation_W = lastpowerRequest_W;
      lastpowerRequest_W = powerRequest_W;

      // Traitement
      ledRequest = powerToLed(powerRequest_W);
      analogWrite(ledOptocoupleur_pin, ledRequest);
      lastAppliedRequestTime_ms = millis();
    }
  }

  // calculs cumuls energie 
  if(injectedPower_W>0.0) {
    lastLoopInjectedEnergy_Ws = injectedPower_W * ((double)lastLoopDuration_ms/1000);
    injectedEnergy_Wh = injectedEnergy_Wh + (lastLoopInjectedEnergy_Ws/3600);
  }
  lastLoopConsumedEnergy_Ws = powerRequest_W * ((double)lastLoopDuration_ms/1000);
  consumedEnergy_Wh = consumedEnergy_Wh + (lastLoopConsumedEnergy_Ws/(3600));

  // Affichages LCD
  displaytoLCD();

  // affichage Serie
  displayToSerial();

  lastLoopTime_ms = millis();  
  loopCount++;

}  

// f telle que f(ledRequest) = powerRequest_W subit un HYSTERESIS quand ledRequest descend sous k=16, jusqu'à remonter au dessus de k=32
// sinon, dans le plage k=[18..64] (soit P~[70w..900w]), on a une approximation satisfaisante avec :
// pour la tension : g(k)=2.1*k-30
// pour la puissance : f(k)=(2000*230)*g(k) = 18.2*k - 261
//   et son opposée : k = (P+261)/18.2 = 0.055*P + 14.28
// couple(P,k)min = (122;14)
int powerToLed(double power_w){
  if(power_w<=0) {return 0;}
  else {return 0.055*power_w + 14.28;}
}
double ledToPower(int ledIntensity){
  return max(0, 18.2*ledIntensity - 261);
}
double powerToVolts(int power_w){
  return power_w * (230/THEORICAL_MAX_LOAD_W);
}

/**
 * Simulation : 
 * la charge appplique sa consigne progressivement, en une seconde de moins que le delai impartit
*/
int simulateLoadFromPowerRequest() {
  simulatedPower_W = 0;
  if(SIMULATION_FOR_DEBUG){
    double progressRate = min(1, (double)(millis()-lastAppliedRequestTime_ms) / (double)(REQUEST_DELAY_MS-1000));
    //simulatedPower_W = int(progressRate * powerRequest_W);
    simulatedPower_W = int(basePowerRequestForSimulation_W 
      + (progressRate * (powerRequest_W-basePowerRequestForSimulation_W)));
    
  }
  return simulatedPower_W;
}

void displaytoLCD() {

  int screenChangeRequest = digitalRead(button_pin);
  int voltRequest_V = int(powerToVolts(powerRequest_W));

  if(screenChangeRequest==0) { screenNumber++; }
  if(screenNumber>2) { screenNumber=1; }

  //lcd.clear();
  //char message0[16];   sprintf(message0, "%d/ Pi:%4dW", screenNumber, injectedPower_W);
  //lcd.setCursor(0,0);  lcd.print(String(message0));
  //lcd.setCursor(0,0);  lcd.print(String(screenNumber) + "-"+ String(hasRecentRequest ? "-" : " ")+" Pi=" + injectedPower_W + "W ");
    

  switch (screenNumber)
  {
  case 1:lcd.setCursor(0,1);  
    lcd.setCursor(0,0);  lcd.print("Pi:" + String(injectedPower_W) + "W "+String(hasRecentRequest ? "*" : " ")+"    ");
    //char message0[16];   snprintf(message0, 16, "Pi:%5dW %s", int(powerRequest_W), (hasRecentRequest ? "*" : " "));
    //lcd.setCursor(0,0);  lcd.print(String(message0));
    char message1[16];   snprintf(message1, 16, "R:%3dW %3dV %d ", int(powerRequest_W), int(voltRequest_V), ledRequest);
    lcd.setCursor(0,1);  lcd.print(String(message1));
    
    break;
  
  case 2:
    lcd.setCursor(0,0);  lcd.print(String("Ei=" + String(injectedEnergy_Wh / 1000, 3) + "KWh    "));
    lcd.setCursor(0,1);  lcd.print(String("Ea=" + String(consumedEnergy_Wh / 1000, 3) + "KWh    "));
    break;
    
  default:
    lcd.setCursor(0,1);  lcd.print(String("*****"));
    break;
  }
  //lcd.setCursor(0,0);  lcd.print(String((millis() - lastLoopTime_ms)));
  //lcd.setCursor(0,1);  lcd.print(String("Ei=" + String(injectedEnergy_Wh / 1000, 3) + "KWh"));

}

void displayToSerial() {
  Serial.print("Pi:");   Serial.print(injectedPower_W);
  //Serial.print(",Ei:"); Serial.print(injectedEnergy_Wh);
  Serial.print(",Preq:"); Serial.print(powerRequest_W);
  //Serial.print(",PbaseSim:"); Serial.print(basePowerRequestForSimulation_W);
  //Serial.print(",Psim:");  Serial.print(simulatedPower_W);
  Serial.print(",hRR:");  Serial.print(hasRecentRequest);
  //Serial.print(",e:");  Serial.print(lastLoopInjectedEnergy_Ws);
  Serial.print(",t:");  Serial.print((millis() - lastLoopTime_ms));
  Serial.print(",c:");  Serial.print(loopCount);
  Serial.println();

}


void loop_debug()
{

  // faire durer la boucle au moins 0.5 seconde
  delay(max(500 - (millis() - lastLoopTime_ms), 0));
  long lastLoopDuration_ms = (millis() - lastLoopTime_ms);
  

  // Pour debug, forcer la valeur de injectedPower_W depuis l'interface série
  // Pour sortir du débug, envoyer "-1".
  if(Serial.available() > 0) { debug_injectedPower = double(Serial.parseInt());}
  if(debug_injectedPower != -1) { ledRequest = debug_injectedPower; }
  
  // Traitement / asservissement
  // Seulement si la dernière consigne a été appliquée depuis plus de 3 secondes
  hasRecentRequest = ( millis()-lastAppliedRequestTime_ms < REQUEST_DELAY_MS );
  
  analogWrite(ledOptocoupleur_pin, ledRequest);
  lastAppliedRequestTime_ms = millis();

  // Affichages LCD
  displaytoLCD();

  // affichage Serie
  displayToSerial();

  lastLoopTime_ms = millis();  
  loopCount++;

}  
