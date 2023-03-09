// #define TINKERCAD 1

#include "LiquidCrystal_I2C.h"

#ifndef TINKERCAD 
#include "EmonLib.h"
#endif 


const double MIN_LED_REQUEST = 14;
const double MAX_LED_REQUEST = 99;
const double POWER_INJECTION_TARGET_W = 25;

int injection_sct013_pin = 1;
int consumption_sct013_pin = 0;
int basseTensionAC_pin = 2;
int ledOptocoupleur_pin = 9; 
int button_pin = 3; 

int ledRequest = 0;
int injectedPower_W = 0;
int debug_injectedPower = -1;
int consumedPower_W = 0;
double injectedEnergy_Wh = 0.0; 
double consumedEnergy_Wh = 0.0; 
double overConsumedEnergy_Wh = 0.0;
double lastLoopInjectedEnergy_Ws = 0;
double lastLoopConsumedEnergy_Ws = 0;
unsigned long lastLoopTime_ms;
int screenNumber = 1;
int loopCount=0;

#ifndef TINKERCAD 
EnergyMonitor emon1;
EnergyMonitor emon2;
#endif
LiquidCrystal_I2C lcd(0x27,20,4); 

void setup()
{
  pinMode(ledOptocoupleur_pin, OUTPUT);
  pinMode(button_pin, INPUT_PULLUP);

  Serial.begin(9600);
  
#ifndef TINKERCAD 
  double basseTensionAC_calibration = 200;
    
  emon1.current(injection_sct013_pin, 46);    
  emon1.voltage(basseTensionAC_pin, basseTensionAC_calibration, 1.7);  
 
  emon2.current(consumption_sct013_pin, 19);    
  emon2.voltage(basseTensionAC_pin, basseTensionAC_calibration, 1.7);  
#endif
  lcd.init();                      
  lcd.backlight();

  // init lastLoopTime_ms pour première loop
  lastLoopTime_ms = millis();  
  
}

void loop()
{
#ifndef TINKERCAD 
  // Mesures
  emon1.calcVI(20,2000); 
  injectedPower_W = -1 * int(emon1.realPower);
  emon2.calcVI(20,2000); 
  consumedPower_W = int(emon2.realPower);
#endif

  // Attente : faire durer la boucle au moins 1 seconde
  delay(max(1000 - (millis() - lastLoopTime_ms), 0));
  long lastLoopDuration_ms = (millis() - lastLoopTime_ms);

  // Pour debug, forcer la valeur de injectedPower_W depuis l'interface série
  // Pour sortir du débug, envoyer "-1".
  // si la valeur n'est pas multiple de 10, on ajoute du bruit
  if(Serial.available() > 0) { debug_injectedPower = double(Serial.parseInt());}
  if(debug_injectedPower != -1) { 
    injectedPower_W = debug_injectedPower; 
    if(debug_injectedPower%10 != 0) { injectedPower_W = injectedPower_W + (random(50)-25); }
  }

  // Traitement / asservissement
  // (Pour les explication, POWER_INJECTION_TARGET_W est noté PIT)
  // Si la puissance injectée est negative, on baisse la led rapidement (-5)
  // Sinon, si la puissance injectée est inférieure à (PIT-50%), on baisse la led (-1)
  // Sinon, si la puissance injectée est entre (PIT-50%) et (PIT+50%), on ne change rien (+0) [c'est "else" final]
  // Sinon, si la puissance injectée est superieure à (PIT+50%), on augmente la led (+1)
  //
  // La création d'un intervale (PIT+-50%) permet de créer une stabilité de la led, sinon elle changerai à chaque loop
  if(injectedPower_W < 0) {ledRequest = max(0, ledRequest - 5);} 
  else if(injectedPower_W <= POWER_INJECTION_TARGET_W*0.5) {ledRequest = max(0, ledRequest - 1);} 
  else if(injectedPower_W >= POWER_INJECTION_TARGET_W*1.5) {ledRequest = min(MAX_LED_REQUEST, ledRequest + 1);} 
  else {ledRequest = ledRequest ;} 
  
  // Application de la consigne
  analogWrite(ledOptocoupleur_pin, (ledRequest<MIN_LED_REQUEST)? 0: ledRequest);

  // calculs cumuls energie 
  lastLoopInjectedEnergy_Ws = injectedPower_W * ((double)lastLoopDuration_ms/1000);
  if(lastLoopInjectedEnergy_Ws>0.0) {
    injectedEnergy_Wh = injectedEnergy_Wh + (lastLoopInjectedEnergy_Ws/3600);
  }
  lastLoopConsumedEnergy_Ws = consumedPower_W * ((double)lastLoopDuration_ms/1000);
  consumedEnergy_Wh = consumedEnergy_Wh + (lastLoopConsumedEnergy_Ws/(3600));
  // si on consomation EDF (injection negative) alors que la charge est allumée (consumedPower>~0),
  // alors on est en "sur-consomation" (conso EDF qu'on aurait pas eu sans le systeme)
  if(lastLoopInjectedEnergy_Ws<0.0 && (consumedPower_W>4)) {
    overConsumedEnergy_Wh = overConsumedEnergy_Wh - (lastLoopInjectedEnergy_Ws/3600);
  }
  
  // Affichages 
  displaytoLCD();
  if(loopCount%3==0){ displayToSerial(); }
  

  lastLoopTime_ms = millis();  
  loopCount++;
}  

void displaytoLCD() {

  int screenChangeRequest = digitalRead(button_pin);

  if(screenChangeRequest==0) { screenNumber++; }
  if(screenNumber>3) { screenNumber=1; }

  switch (screenNumber)
  {
  case 1:  
    lcd.setCursor(0,0);  lcd.print("Pi :" + String(injectedPower_W) + "W "+"     ");
    char message1[16];   snprintf(message1, 16, "Pac:%3dW  k:%d ", int(consumedPower_W), ledRequest);
    lcd.setCursor(0,1);  lcd.print(String(message1));
    
    break;
  
  case 2:
    lcd.setCursor(0,0);  lcd.print(String("Ei =" + String(injectedEnergy_Wh / 1000, 3) + "KWh   "));
    lcd.setCursor(0,1);  lcd.print(String("Eac=" + String(consumedEnergy_Wh / 1000, 3) + "KWh   "));
    break;
    
  case 3:
    lcd.setCursor(0,0);  lcd.print(String("Esurconso = "));
    lcd.setCursor(0,1);  lcd.print(String(overConsumedEnergy_Wh / 1000, 3) + "KWh    ");
    break;
  default:
    lcd.setCursor(0,1);  lcd.print(String("*****"));
    break;
  }
}

void displayToSerial() {
  Serial.println(
    //'{"Pi":'+String(injectedPower_W)+',"Pac":'+String(consumedPower_W)+",'k':"+String(ledRequest)+"}"
    "{\"Pi\":"+String(injectedPower_W)+",\"Pac\":"+String(consumedPower_W)+",\"k\":"+String(ledRequest)+"}"
  
  );

}
