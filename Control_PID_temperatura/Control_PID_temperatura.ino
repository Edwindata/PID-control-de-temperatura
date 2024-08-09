#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Configuración de la pantalla OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET    -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define OLED_I2C_ADDRESS 0x3C // Dirección I2C del SSD1306
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// I/O
int PWM_pin = 3; // Pin para la señal PWM al controlador MOSFET (cambiado a 3)
int PWM_pin2 = 5; // Nuevo pin para la señal PWM al controlador MOSFET (cambiado a 5)
int clk = 2; // Pin 1 del encoder rotatorio (cambiado a 2)
int data = 4; // Pin 2 del encoder rotatorio (cambiado a 4)
int buttonPin = 7; // Pin del botón (cambiado a 7)

// Variables
float set_temperature = 0; // Valor predeterminado del punto de ajuste de temperatura
float temperature_read = 0.0;
float PID_error = 0;
float previous_error = 0;
float elapsedTime, Time, timePrev;
float PID_value = 0;
int button_pressed = 0;
int menu_activated = 0;
float last_set_temperature = 0;

// Variables para la detección del estado del encoder rotatorio
int clk_State;
int Last_State;
bool dt_State;
int buttonState;
int lastButtonState = HIGH; // Asume que el botón no está presionado inicialmente

// Constantes PID
float kp = 4;
float ki = 1;
float kd = 0.25;

int PID_p = 0; 
int PID_i = 0; 
int PID_d = 0;
float last_kp = 0;
float last_ki = 0;
float last_kd = 0;

int PID_values_fixed = 0;

// Pines para el SPI con MAX6675
#define MAX6675_CS 10
#define MAX6675_SO 12
#define MAX6675_SCK 13

void setup() {
  pinMode(PWM_pin, OUTPUT);
  pinMode(PWM_pin2, OUTPUT); // Configura el nuevo pin como salida
  Time = millis();
  Last_State = digitalRead(clk);

  pinMode(clk, INPUT);
  pinMode(data, INPUT);
  pinMode(buttonPin, INPUT_PULLUP); // Habilita el resistor pull-up interno

  attachInterrupt(digitalPinToInterrupt(clk), readEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(data), readEncoder, CHANGE);

  // Inicializar OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDRESS)) {
    Serial.println(F("Fallo en la inicialización de SSD1306"));
    for (;;);
  }
  display.display();
  delay(2000); // Pausa de 2 segundos
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
}

void loop() {
  readButton();

  if (menu_activated == 0) {
    temperature_read = (readThermocouple());
    PID_error = set_temperature - temperature_read + 3;
    PID_p = kp * PID_error;
    PID_i = PID_i + (ki * PID_error);

    timePrev = Time;
    Time = millis();
    elapsedTime = (Time - timePrev) / 1000;
    PID_d = kd * ((PID_error - previous_error) / elapsedTime);
    PID_value = PID_p + PID_i + PID_d;

    if (PID_value < 0) { PID_value = 0; }
    if (PID_value > 255) { PID_value = 255; }
    analogWrite(PWM_pin, 255 - PID_value);
    analogWrite(PWM_pin2, 255 - PID_value); // Añade esta línea para el nuevo pin
    previous_error = PID_error;

    delay(250); // Frecuencia de actualización + retardo de impresión OLED
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("PID TEMP control");
    display.setCursor(0, 10);
    display.print("S:");
    display.setCursor(20, 10);
    display.print(set_temperature, 1);
    display.setCursor(80, 10);
    display.print("R:");
    display.setCursor(100, 10);
    display.print(temperature_read, 1);
    display.display();
  }

  if (menu_activated == 1) {
    analogWrite(PWM_pin, 255);
    analogWrite(PWM_pin2, 255); // Añade esta línea para el nuevo pin
    if (set_temperature != last_set_temperature) {
      display.clearDisplay();
      display.setCursor(0, 0);
      display.print("Set temperature");
      display.setCursor(0, 10);
      display.print(set_temperature);
      display.display();
    }
    last_set_temperature = set_temperature;
  }

  if (menu_activated == 2) {
    if (kp != last_kp) {
      display.clearDisplay();
      display.setCursor(0, 0);
      display.print("Set P value ");
      display.setCursor(0, 10);
      display.print(kp);
      display.display();
    }
    last_kp = kp;
  }

  if (menu_activated == 3) {
    if (ki != last_ki) {
      display.clearDisplay();
      display.setCursor(0, 0);
      display.print("Set I value ");
      display.setCursor(0, 10);
      display.print(ki);
      display.display();
    }
    last_ki = ki;
  }

  if (menu_activated == 4) {
    if (kd != last_kd) {
      display.clearDisplay();
      display.setCursor(0, 0);
      display.print("Set D value ");
      display.setCursor(0, 10);
      display.print(kd);
      display.display();
    }
    last_kd = kd;
  }
}

double readThermocouple() {
  uint16_t v;
  pinMode(MAX6675_CS, OUTPUT);
  pinMode(MAX6675_SO, INPUT);
  pinMode(MAX6675_SCK, OUTPUT);
  digitalWrite(MAX6675_CS, LOW);
  delay(1);

  v = shiftIn(MAX6675_SO, MAX6675_SCK, MSBFIRST);
  v <<= 8;
  v |= shiftIn(MAX6675_SO, MAX6675_SCK, MSBFIRST);
  digitalWrite(MAX6675_CS, HIGH);
  if (v & 0x4) {
    return NAN;
  }

  v >>= 3;
  return v * 0.25;
}

void readEncoder() {
  if (menu_activated == 1) {
    clk_State = digitalRead(clk);
    dt_State = digitalRead(data);
    if (clk_State != Last_State) {
      if (dt_State != clk_State) {
        set_temperature = set_temperature + 0.5;
      } else {
        set_temperature = set_temperature - 0.5;
      }
    }
    Last_State = clk_State;
  }

  if (menu_activated == 2) {
    clk_State = digitalRead(clk);
    dt_State = digitalRead(data);
    if (clk_State != Last_State) {
      if (dt_State != clk_State) {
        kp = kp + 1;
      } else {
        kp = kp - 1;
      }
    }
    Last_State = clk_State;
  }

  if (menu_activated == 3) {
    clk_State = digitalRead(clk);
    dt_State = digitalRead(data);
    if (clk_State != Last_State) {
      if (dt_State != clk_State) {
        ki = ki + 1;
      } else {
        ki = ki - 1;
      }
    }
    Last_State = clk_State;
  }

  if (menu_activated == 4) {
    clk_State = digitalRead(clk);
    dt_State = digitalRead(data);
    if (clk_State != Last_State) {
      if (dt_State != clk_State) {
        kd = kd + 1;
      } else {
        kd = kd - 1;
      }
    }
    Last_State = clk_State;
  }
}

void readButton() {
  buttonState = digitalRead(buttonPin);

  // Detectar cambio de estado del botón
  if (buttonState != lastButtonState) {
    if (buttonState == LOW) {
      button_pressed = 1;
    } else {
      if (button_pressed == 1) {
        if (menu_activated == 4) {
          menu_activated = 0;
          PID_values_fixed = 1;
          button_pressed = 0;
          delay(500);
        } else {
          menu_activated = menu_activated + 1;
          button_pressed = 0;
          delay(500);
        }
      }
    }
  }
  lastButtonState = buttonState; // Actualiza el estado del botón
}
