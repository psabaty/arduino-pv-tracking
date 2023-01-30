#define TINKERCAD 1

#include "LiquidCrystal_I2C.h"

#ifndef TINKERCAD 
#include "EmonLib.h"
#endif 

const bool DEBUG_ENABLED = true;

unsigned long REQUEST_DELAY_MS = 3000;
const double PUISSANCE_MAX_CHAUFFE_EAU__W = 2000;
const double PUISSANCE_MIN_CHAUFFE_EAU__W = 0;
const int LED_MAX_255 = 122;

int sct013_pin = 1;
int basseTensionAC_pin = 2;
int ledOptocoupleur_pin = 9; 
int button_pin = 3; 

double consignePuissance_W = 0; 
double lastConsignePuissance_W = 0;
double baseSimulationConsignePuissance_W = 0;
int consigne_255 = 0;

double energieInjecteeVersEDF_Wh = 0.0; 
//double energieTheoriqueAutoConsommee_Wh = 0.0;
double lastLoopEnergy_Ws = 0;
int puissanceFournieParEDF_W = 0;
int debug_puissanceEdf = -1;
int puissanceSimulee = 0;

unsigned long lastLoopTime_ms;
unsigned long lastAppliedRequestTime_ms = 0;
bool hasRecentRequest = false;
int loopCount=0;
int screenNumber = 2;

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
  //loop_debugPlotter();
}

void loop_directTarget()
{

#ifndef TINKERCAD 
  emon1.calcVI(20,2000); 
  puissanceFournieParEDF_W = int(emon1.realPower);
#endif
  // faire durer la boucle au moins 0.5 seconde
  delay(max(500 - (millis() - lastLoopTime_ms), 0));
  long lastLoopDuration_ms = (millis() - lastLoopTime_ms);
  

  // Pour debug, forcer la valeur de puissanceFournieParEDF_W depuis l'interface série
  // Pour sortir du débug, envoyer "-1".
  if(Serial.available() > 0) { debug_puissanceEdf = double(Serial.parseInt());}
  if(debug_puissanceEdf != -1) { puissanceFournieParEDF_W = debug_puissanceEdf; }

  // Pour debug (DEBUG==true), simuler une chauffe-eau qui suit la consigne
  puissanceFournieParEDF_W = puissanceFournieParEDF_W + simulationPuissanceChauffeEau();

  // calculs cumuls energie 
  if(puissanceFournieParEDF_W<0.0) {
    lastLoopEnergy_Ws = -1.0 * puissanceFournieParEDF_W * ((double)lastLoopDuration_ms/1000);
    energieInjecteeVersEDF_Wh = energieInjecteeVersEDF_Wh + (lastLoopEnergy_Ws/3600);
  }
  // energieTheoriqueAutoConsommee_Wh = energieTheoriqueAutoConsommee_Wh + (consignePuissance_W * lastLoopDuration_ms/(3600*1000));

  
  // Traitement / asservissement
  // Seulement si la dernière consigne a été appliquée depuis plus de 3 secondes
  hasRecentRequest = ( millis()-lastAppliedRequestTime_ms < REQUEST_DELAY_MS );
  if( ! hasRecentRequest ) {

    // si puissanceFournieParEDF_W est négatif
    //   alors on augmente la consigne avec cette puissance (en valeur absolue)
    //   en l'occurence on soustrait cette valeur negative -> augmentation
    // si puissanceFournieParEDF_W est positif
    //   alors on baisse la consigne avec cette puissance 
    //   en l'occurence on soustrait cette valeur positive -> baisse
    // en résumé, on applique toujours la même formule :
    consignePuissance_W = consignePuissance_W - puissanceFournieParEDF_W;
    consignePuissance_W = min(consignePuissance_W, PUISSANCE_MAX_CHAUFFE_EAU__W);
    
    // si la consigne est inferieure à la puissance MIN qu'accepte le chauffe-eau, on met à zero
    if(consignePuissance_W < PUISSANCE_MIN_CHAUFFE_EAU__W){
      consignePuissance_W = 0;
    }

    // Appliation de la consigne
    // (seulement si elle est différente de la précédente)
    if( consignePuissance_W != lastConsignePuissance_W ){
      baseSimulationConsignePuissance_W = lastConsignePuissance_W;
      lastConsignePuissance_W = consignePuissance_W;
      consigne_255 = int(LED_MAX_255*(consignePuissance_W/PUISSANCE_MAX_CHAUFFE_EAU__W));
      analogWrite(ledOptocoupleur_pin, consigne_255);
      lastAppliedRequestTime_ms = millis();
    }
  }

  // Affichages LCD
  displaytoLCD();

  // affichage Serie
  displayToSerial();

  lastLoopTime_ms = millis();  
  loopCount++;

}  

/**
 * Simulation : 
 * Le chauffe-eau appplique sa consigne progressivement, en une seconde de moins que le delai impartit
*/
int simulationPuissanceChauffeEau() {
  puissanceSimulee = 0;
  if(DEBUG_ENABLED){
    double progressRate = min(1, (double)(millis()-lastAppliedRequestTime_ms) / (double)(REQUEST_DELAY_MS-1000));
    //puissanceSimulee = int(progressRate * consignePuissance_W);
    puissanceSimulee = int(baseSimulationConsignePuissance_W 
      + (progressRate * (consignePuissance_W-baseSimulationConsignePuissance_W)));
    
  }
  return puissanceSimulee;
}

void displaytoLCD() {

  int screenChangeRequest = digitalRead(button_pin);

  if(screenChangeRequest==0) { screenNumber++; }
  if(screenNumber>2) { screenNumber=1; }

  //lcd.clear();
  lcd.setCursor(0,0);  lcd.print(String(String(screenNumber) + "-  Pedf=" + puissanceFournieParEDF_W + "W "));
  //char message0[16];   sprintf(message0, "%d/ EDF:%4dW", screenNumber, puissanceFournieParEDF_W);
  //lcd.setCursor(0,0);  lcd.print(String(message0));
    

  switch (screenNumber)
  {
  case 1:
    lcd.setCursor(0,1);  lcd.print(String("Ei=" + String(energieInjecteeVersEDF_Wh / 1000, 3) + "KWh"));
    break;
  
  case 2:
    //char message2[16];   sprintf(message2, "Req: %4dW %s(%d)", int(consignePuissance_W), delayedMsg, consigne_255);
    //lcd.setCursor(0,1);  lcd.print(String(message2));
    lcd.setCursor(0,1);  
    lcd.print("Req:"
      + String(int(consignePuissance_W))+"W"
      + String(hasRecentRequest ? "." : " ")
      +"(" + String(consigne_255) + ")");
    break;
  
  default:
    lcd.setCursor(0,1);  lcd.print(String("*****"));
    break;
  }
  //lcd.setCursor(0,0);  lcd.print(String((millis() - lastLoopTime_ms)));
  //lcd.setCursor(0,1);  lcd.print(String("Ei=" + String(energieInjecteeVersEDF_Wh / 1000, 3) + "KWh"));

}

void displayToSerial() {
  Serial.print("Pedf:");   Serial.print(puissanceFournieParEDF_W);
  //Serial.print(",Ei:"); Serial.print(energieInjecteeVersEDF_Wh);
  Serial.print(",Preq:"); Serial.print(consignePuissance_W);
  Serial.print(",PbaseSim:"); Serial.print(baseSimulationConsignePuissance_W);
  Serial.print(",Psim:");  Serial.print(puissanceSimulee);
  Serial.print(",hRR:");  Serial.print(hasRecentRequest);
  //Serial.print(",e:");  Serial.print(lastLoopEnergy_Ws);
  Serial.print(",t:");  Serial.print((millis() - lastLoopTime_ms));
  Serial.print(",c:");  Serial.print(loopCount);
  Serial.println();

}
