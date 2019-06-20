/* Michal Kielbratowski, Dawid Gamulka 2019 */ 

#include <stdlib.h>
#include <LiquidCrystal.h>

// Serwonapędy
#define X_STEP_PIN         54
#define X_DIR_PIN          55
#define X_ENABLE_PIN       38
#ifndef X_CS_PIN
  #define X_CS_PIN         53
#endif

#define Y_STEP_PIN         60
#define Y_DIR_PIN          61
#define Y_ENABLE_PIN       56
#ifndef Y_CS_PIN
  #define Y_CS_PIN         49
#endif

#define Z_STEP_PIN         46
#define Z_DIR_PIN          48
#define Z_ENABLE_PIN       62
#ifndef Z_CS_PIN
  #define Z_CS_PIN         40
#endif

// Krańcówki
#define X_MIN_PIN           3
#ifndef X_MAX_PIN
  #define X_MAX_PIN         2
#endif
#define Y_MIN_PIN          14
#define Y_MAX_PIN          15
#define Z_MIN_PIN          18
#define Z_MAX_PIN          19

// Grzałka

#define MOSFET_D_PIN       10
#define TEMP_0_PIN         13
#define HEATER_1_PIN   MOSFET_D_PIN
#define RAMPS_D10_PIN      10

#define a -1.51922044997230e-06
#define b 0.00275858993083322
#define c -1.55501496673548
#define d 325.645485874563

// Wyświetlacz LCD
LiquidCrystal lcd(16,17,23,25,27,29);

String input;
bool dir = 0;
int dirX = 0;
int dirY = 0;
int dirZ = 0;
int stepX = 0;
int stepY = 0;
int stepZ = 0;
bool reply = 0;
int temp;  
double x, y;
double e;
double PID_value; 
double integ = 0;
double elapsed;
double prev_error;
double P,I,D;

void setup() {
  Serial.begin(115200);

  // X
  pinMode (X_ENABLE_PIN, OUTPUT);
  digitalWrite(X_ENABLE_PIN, LOW);
  pinMode (X_DIR_PIN, OUTPUT);
  pinMode(X_STEP_PIN, OUTPUT);
  
  // Y
  pinMode (Y_ENABLE_PIN, OUTPUT);
  digitalWrite(Y_ENABLE_PIN, LOW);
  pinMode (Y_DIR_PIN, OUTPUT);
  pinMode(Y_STEP_PIN, OUTPUT);

  // Z
  pinMode (Z_ENABLE_PIN, OUTPUT);
  digitalWrite(Z_ENABLE_PIN, LOW);
  pinMode (Z_DIR_PIN, OUTPUT);
  pinMode(Z_STEP_PIN, OUTPUT);

  // Krańcówki
  pinMode(X_MAX_PIN, INPUT_PULLUP);
  pinMode(Y_MAX_PIN, INPUT_PULLUP);
  pinMode(Z_MAX_PIN, INPUT_PULLUP);

  // Wybór kierunku
  digitalWrite(X_DIR_PIN, 0);
  digitalWrite(Y_DIR_PIN, 0);
  digitalWrite(Z_DIR_PIN, 0);

  lcd.begin( 20, 4 );
  pinMode(HEATER_1_PIN, OUTPUT);
}

void loop() {
  digitalWrite(X_STEP_PIN, HIGH);
  digitalWrite(Y_STEP_PIN, HIGH);
  digitalWrite(Z_STEP_PIN, HIGH);
  delay(1);
  
  if ((digitalRead(X_MAX_PIN) == 0) && (stepX > 0))
  {
    digitalWrite(X_STEP_PIN, LOW);
    stepX--;
  }
  if ((digitalRead(Y_MAX_PIN) == 0) && (stepY > 0))
  {
    digitalWrite(Y_STEP_PIN, LOW);
    stepY--;
  }
  if ((digitalRead(Z_MAX_PIN) == 0) && (stepZ > 0))
  {
    digitalWrite(Z_STEP_PIN, LOW);
    stepZ--;
  }
  
  delay(1);
  
  while (Serial.available() > 0)
  {
    input = Serial.readStringUntil('\n');
    sscanf(input.c_str(), "krokX%dkrokY%dkrokZ%d", &stepX, &stepY, &stepZ);
    reply=1;

    dirX = stepX < 0; 				// wybor kierunku - 1 dol, 0  gora
    stepX = abs(stepX); 			// zmiana minusa na plus, liczba krokow
    digitalWrite(X_DIR_PIN, stepX); // przypisanie kierunku

    dirY = stepY < 0; 				// wybor kierunku - 1 dol, 0  gora
    stepY = abs(stepY); 			// zmiana minusa na plus, liczba krokow
    digitalWrite(Y_DIR_PIN, stepY); // przypisanie kierunku

    dirZ = stepZ < 0; 				// wybor kierunku - 1 dol, 0  gora
    stepZ = abs(stepZ); 			// zmiana minusa na plus, liczba krokow
    digitalWrite(Z_DIR_PIN, stepZ); // przypisanie kierunku
	
	//Serial.println(dirX);
    //Serial.println(dirY);
    //Serial.println(dirZ);
  }

  if(stepX==0 && stepY==0 && stepZ==0 && reply==1)
  {
    Serial.println("OK");
    reply=0;
  }
  
  lcd.setCursor(0,0);
  temp = analogRead(13); 				// Odczytanie obecnej temperatury
  // lcd.print("temp = ");
  // lcd.print(temp);
  lcd.print(PID_value);
  x=(double) temp;
  y=a*x*x*x+b*x*x+c*x+d;				// Wyliczenie błędu
  lcd.print(" ");
  lcd.print(y);
  
  e = 100 - y;
  elapsed = (y - prev_error)/ 0.1;
  
  if (y < 100)
  integ = integ + e*0.1;

  // Stałe PID
  P = 255/80;
  I = 0.3;
  D = 1.8;

  PID_value = round(P*e + I*integ + D*elapsed);

  // Definicja zakresu PWM między 0 a 255
  if (PID_value < 0)
  {	  PID_value = 0;   }
  if (PID_value > 255)
  {	  PID_value = 255; }
  
  analogWrite(HEATER_1_PIN, PID_value);		// Podanie sygnału PWM do nagrzewacza

  prev_error = y;							// Przepisanie poprzedniego błędu dla kolejnej pętli
  delay(300);
}
