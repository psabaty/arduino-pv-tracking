#include "LiquidCrystal_I2C.h"
#include "EmonLib.h"

const double PUISSANCE_MAX_CHAUFFE_EAU__W = 2000;
const double PUISSANCE_MIN_CHAUFFE_EAU__W = 0;
//const int INCREMENT_CONSIGNE__W = 50;
const int LED_MAX_255 = 122;

int sct013_pin = 1;
int basseTensionAC_pin = 2;
int ledOptocoupleur_pin = 9; 

double consignePuissanceChauffeEau_W = 0; 
int consigne_255 = 0;

double energieInjecteeVersEDF_Wh = 0.0; 
//double energieTheoriqueAutoConsommee_Wh = 0.0;
double puissanceFournieParEDF_W = 0;
double debug_puissanceEdf = -1;

unsigned long lastLoopTime_ms;
int loopCount=0;
EnergyMonitor emon1;
LiquidCrystal_I2C lcd(0x27,20,4); 

void setup()
{
  pinMode(ledOptocoupleur_pin, OUTPUT);

  Serial.begin(9600);
  
  int sct013_calibration = 46;
  double basseTensionAC_calibration = 200;
    
  emon1.current(sct013_pin, sct013_calibration);    
  emon1.voltage(basseTensionAC_pin, basseTensionAC_calibration, 1.7);  

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
  double lastLoopEnergy_Ws = 0;
  emon1.calcVI(20,2000); 
  puissanceFournieParEDF_W = emon1.realPower;

  // faire durer la boucle au moins une seconde
  delay(max(1000 - (millis() - lastLoopTime_ms), 0));

  // durée de la précedente boucle, à mettre APRES le emon.calc() qui peut durer longtemps
  long lastLoopDuration_ms = (millis() - lastLoopTime_ms);
  

  // Pour debug, forcer la valeur de puissanceFournieParEDF_W depuis l'interface série
  // Pour sortir du débug, envoyer "-1".
  if(Serial.available() > 0) {
    debug_puissanceEdf = double(Serial.parseInt());
  }
  if(debug_puissanceEdf != -1) {
    puissanceFournieParEDF_W = debug_puissanceEdf;
  }

  // calculs (tres approximatifs) cumuls energie 
  if(puissanceFournieParEDF_W<0.0) {
    lastLoopEnergy_Ws = -1.0 * puissanceFournieParEDF_W * ((double)lastLoopDuration_ms/1000);
    energieInjecteeVersEDF_Wh = energieInjecteeVersEDF_Wh + (lastLoopEnergy_Ws/3600);
  }
  // energieTheoriqueAutoConsommee_Wh = energieTheoriqueAutoConsommee_Wh + (consignePuissanceChauffeEau_W * lastLoopDuration_ms/(3600*1000));

  
  // Traitement / asservissement
  // si puissanceFournieParEDF_W est négatif
  //   alors on augmente la consigne avec cette puissance (en valeur absolue)
  //   en l'occurence on soustrait cette valeur negative -> augmentation
  // si puissanceFournieParEDF_W est positif
  //   alors on baisse la consigne avec cette puissance 
  //   en l'occurence on soustrait cette valeur positive -> baisse
  // en résumé, on applique toujours la même formule :
  consignePuissanceChauffeEau_W = consignePuissanceChauffeEau_W - puissanceFournieParEDF_W;
  consignePuissanceChauffeEau_W = min(consignePuissanceChauffeEau_W, PUISSANCE_MAX_CHAUFFE_EAU__W);
  
  // si la consigne est inferieure à la puissance MIN qu'accepte le chauffe-eau, on met à zero
  if(consignePuissanceChauffeEau_W < PUISSANCE_MIN_CHAUFFE_EAU__W){
    consignePuissanceChauffeEau_W = 0;
  }

  // Appliation de la consigne
  // BUG BUG BUG : 
  // - K augmente trop vite, sans laisser le temps à la consigne de s'apppliquer (mettre un delay dynamique en fin de loop ?)
  // - Le LCD s'arrete quand K arrive autour de 122 (int overflow ?...)
  // on applique la consigne au variateur de puissance (led en serie avec une  R=2.2Kohm)
  consigne_255 = int(LED_MAX_255*(consignePuissanceChauffeEau_W/PUISSANCE_MAX_CHAUFFE_EAU__W));
  //analogWrite(ledOptocoupleur_pin, consigne_255);

  // Affichages LCD
  // (on affiche les consignes de la loop précédente, qui ont mené à la valeur qu'on mesure maintenant)
  //char message1[16];   sprintf(message1, "EDF:%4dW k=%3d", int(puissanceFournieParEDF_W), consigne_255);
  //char message1[16];   sprintf(message1, "EDF:%4dW", int(puissanceFournieParEDF_W));
  String message1 = String("Ps=" + String(puissanceFournieParEDF_W, 0) + "W    ");
  char message2[16];   sprintf(message2, "Consigne: P=%3d", int(consignePuissanceChauffeEau_W));
  String message3 = String("Ei=" + String(energieInjecteeVersEDF_Wh / 1000, 3) + "KWh");

  lcd.clear();
  lcd.setCursor(0,0);  lcd.print(String(message1));
  //lcd.setCursor(0,0);  lcd.print(String((millis() - lastLoopTime_ms)));
  //lcd.setCursor(0,1);  lcd.print(String(message2));
  lcd.setCursor(0,1);  lcd.print(String(message3));
  
  // affichage Serie
  char messagePlotter[64];
  //sprintf(messagePlotter, "EDF:%d,consigneW:%d,consigneKx10:%d", int(puissanceFournieParEDF_W), int(consignePuissanceChauffeEau_W), 10*consigne_255);
  //Serial.println(messagePlotter);
  Serial.print("P:");
  Serial.print(puissanceFournieParEDF_W);
  Serial.print(",Ei:");
  Serial.print(energieInjecteeVersEDF_Wh);
  Serial.print(",e:");
  Serial.print(lastLoopEnergy_Ws);
  Serial.print(",t:");
  Serial.print((millis() - lastLoopTime_ms));
  Serial.print(",c:");
  Serial.print(loopCount);
  Serial.println();
  //emon1.serialprint();


  lastLoopTime_ms = millis();  
  loopCount++;

}  
/*
void loop_debugPlotter(){

  emon1.calcVI(20,2000); 
  double puissanceFournieParEDF_W = emon1.realPower;

  // Affichages (on affiche les consignes de la loop précédente, qui ont mené à la valeur qu'on mesure maintenant)
  char message1[16];
  sprintf(message1, "EDF:%4dW k=%3d", int(puissanceFournieParEDF_W), consigne_255);
  lcd.setCursor(0,0);
  lcd.print(String(message1));
  char message2[16];
  sprintf(message2, "Consigne P=%4d", int(consignePuissanceChauffeEau_W));
  lcd.setCursor(0,1);
  lcd.print(String(message2));
  
  // affichage Serie
  //emon1.serialprint();
  char messagePlotter[64];
  sprintf(messagePlotter, "EDF:%d,consigneW:%d,consigneKx10:%d", int(puissanceFournieParEDF_W), int(consignePuissanceChauffeEau_W), 10*consigne_255);
  Serial.println(messagePlotter);
  
  // Traitement : increment constant pour plotter
  //consignePuissanceChauffeEau_W = consignePuissanceChauffeEau_W + 40;
  if(Serial.available() > 0) {
    consignePuissanceChauffeEau_W = double(Serial.parseInt());
  }  
  // on applique la consigne au variateur de puissance (led en serie avec une  R=2.2Kohm)
  consigne_255 = int(LED_MAX_255*(consignePuissanceChauffeEau_W/PUISSANCE_MAX_CHAUFFE_EAU__W));
  analogWrite(ledOptocoupleur_pin, consigne_255);

  // on attend que l'effet de la consigne se stabilise
  delay(2*1000);

}
*/

/*
void loop_increment()
{
  emon1.calcVI(20,2000); 
  
  double intensiteFournieParEDF_I = emon1.calcIrms(1480);
  double puissanceFournieParEDF_W = emon1.realPower;

  // DEBUG DEBUG DEBUG DEBUG DEBUG DEBUG 
  puissanceFournieParEDF_W = -60;
  
  if (puissanceFournieParEDF_W<(-1*INCREMENT_CONSIGNE__W)) {
    // Pas de consomation depuis EDF => on augmente la charge du chauffe-eau
    consignePuissanceChauffeEau_W = consignePuissanceChauffeEau_W + INCREMENT_CONSIGNE__W;
    consignePuissanceChauffeEau_W = min(consignePuissanceChauffeEau_W, PUISSANCE_MAX_CHAUFFE_EAU__W);
  }else{
    // Consomation depuis EDF => on baisse la charge du chauffe-eau
    consignePuissanceChauffeEau_W = consignePuissanceChauffeEau_W - INCREMENT_CONSIGNE__W;
    consignePuissanceChauffeEau_W = max(consignePuissanceChauffeEau_W, 0);
  }
  
  
  // on applique la consigne au variateur de puissance (led en serie avec une  R=2.2Kohm)
  int consigne_255 = LED_MIN_255 + int((LED_MAX_255-LED_MIN_255)*(consignePuissanceChauffeEau_W/PUISSANCE_MAX_CHAUFFE_EAU__W));
  // si la consigne est inferieure à la puissance min qu'accepte le chauffe-eau, on laisse à zero
  if(consignePuissanceChauffeEau_W < PUISSANCE_MIN_CHAUFFE_EAU__W){
    consigne_255 = 0;
  }
  analogWrite(ledOptocoupleur_pin, consigne_255);

  // affichage Serie
  //String message1 = String("EDF:" + String(int(puissanceFournieParEDF_W))+"W  ");
  //String message1bis = String("U=" + String(int(emon1.Vrms))+"V  ");
  // "EDF:ddddW k=ddd"
  // "0123456789ABCDEF"
  char message1[16];
  sprintf(message1, "EDF:%4dW k=%3d", int(puissanceFournieParEDF_W), consigne_255);
  //String message2 = String("K=" + String(consigne_255) + " P=" + String(int(consignePuissanceChauffeEau_W))+"W  ");
  String message2 = String("Consigne: P=" + String(int(consignePuissanceChauffeEau_W))+"W  ");
  
  char message3[16];
  sprintf(message3, "EDF:%d,consigneW:%d,consigneKx10:%d", int(puissanceFournieParEDF_W), int(consignePuissanceChauffeEau_W), 10*consigne_255);
  
  //emon1.serialprint();
  Serial.println(message3);
  
  // affichage LCD
  lcd.setCursor(0,0);
  lcd.print(message1);
  lcd.setCursor(0,1);
  lcd.print(message2);

  delay(4000);
}  
*/

/*
 * 
 * NOTES
 * 
K W
1 0
2 0
3 470
4 700
5 950
6 1040
7 1200
8 1250
9 1300
10  1400
11  1400
12  1440
13  1450
14  1500
15  1526
16  1583
17  1640
18  1662
19  1700
29  1800
43  1900
 */
