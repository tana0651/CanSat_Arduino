/*
    This is CanSat.
*/

// set before flight
#define LAT_GOAL 40.142723
#define LON_GOAL 139.987457
#define GOAL_DISTANCE 2.0

// constant for calculation
#define R 6378137

// pin configration
#define MLIN1  4
#define MLIN2  5
#define MLVREF 6
#define MRIN1  7
#define MRIN2  8
#define MRVREF 9
#define FLIGHT 10
#define SEP    11

// time constants
#define SEP_TIME 5000
#define TIME_STEP 1000

float t_0;
int flag, f_stack;
float lat_0, lon_0;
float lat, lon;
float distance, angle;

void hold();
void front();
void left();
void right();
void back();
void brake();
float calc_distance(float lat_0, float lon_0, float lat, float lon);
float calc_angle(float lat_0, float lon_0, float lat, float lon);
void record();

void setup() {
  Serial.begin(9600);
  Serial.print("[ INFO ] setup routine executing...");

  pinMode(MLIN1,  OUTPUT);
  pinMode(MLIN2,  OUTPUT);
  pinMode(MLVREF, OUTPUT);
  pinMode(MRIN1,  OUTPUT);
  pinMode(MRIN2,  OUTPUT);
  pinMode(MRVREF, OUTPUT);
  pinMode(FLIGHT, INPUT_PULLUP);
  pinMode(SEP,    OUTPUT);

  Serial.println("done");
}

void loop() {
  if (millis() - t_0 > TIME_STEP)
  {
    Serial.println("[ INFO ] loop routine.");
    record();
    t_0 = millis();
  }

  if (flag == 0) {
    Serial.println("[ DEBUG ] flag : 0");
    if (digitalRead(FLIGHT) == HIGH) {
      digitalWrite(SEP, HIGH);
      delay(SEP_TIME);
      digitalWrite(SEP, LOW);
      flag = 1;
    }
  }

  if (flag == 1)
  {
    Serial.println("[ DEBUG ] flag : 1");
    lat_0 = 1;//;gps.get_lat();
    lon_0 = 1;//gps.get_lon();
    Serial.println("[ DEBUG ] lat_0 : " + String(lat_0));
    Serial.println("[ DEBUG ] lon_0 : " + String(lon_0));

    front();
    delay(5000);
    brake();
    hold();

    lat = 1;//gps.get_lat();
    lon = 1;//gps.get_lon();
    Serial.println("[ DEBUG ] lat : " + String(lat));
    Serial.println("[ DEBUG ] lon : " + String(lon));

    if (calc_distance(lat_0, lon_0, lat, lon) < 2) {
      Serial.println("[ WARN ] stack detected.");
      f_stack = 1;
    }

    distance = calc_distance(lat, lon, LAT_GOAL, LON_GOAL);
    Serial.println("[ DEBUG ] distance : " + String(distance));
    if (distance < GOAL_DISTANCE) {
      flag = 2;
    }

    angle = calc_angle(lat, lon, LAT_GOAL, LON_GOAL) - calc_angle(lat_0, lon_0, lat, lon);
    if (angle > 180) {
      angle -= 360;
    }
    if (angle < -180) {
      angle += 360;
    }

    Serial.println("[ DEBUG ] angle : " + String(angle));
    if (angle > 0) {
      left();
      if (angle > 90) {
        Serial.println("[ DEBUG ] large left turn.");
        delay(2000);
      } else {
        Serial.println("[ DEBUG ] small left turn.");
        delay(1000);
      }
      brake();
      hold();
    }
    else {
      right();
      if (angle < -90) {
        Serial.println("[ DEBUG ] large right turn.");
        delay(2000);
      } else {
        Serial.println("[ DEBUG ] small right turn.");
        delay(1000);
      }
      brake();
      hold();
    }
  }

  if (flag == 2)
  {
    Serial.println("[ DEBUG ] flag : 2");
  }
}

void hold() {
  Serial.println("[ DEBUG ] motor : hold");
  digitalWrite(MLIN1, LOW);
  digitalWrite(MLIN2, LOW);
  analogWrite(MLVREF, 0);
  digitalWrite(MRIN1, LOW);
  digitalWrite(MRIN2, LOW);
  analogWrite(MRVREF, 0);
}

void front() {
  Serial.println("[ DEBUG ] motor : front");
  digitalWrite(MLIN1, HIGH);
  digitalWrite(MLIN2, LOW);
  analogWrite(MLVREF, 255);
  digitalWrite(MRIN1, HIGH);
  digitalWrite(MRIN2, LOW);
  analogWrite(MRVREF, 255);
}

void left() {
  Serial.println("[ DEBUG ] motor : left");
  digitalWrite(MLIN1, LOW);
  digitalWrite(MLIN2, LOW);
  analogWrite(MLVREF, 0);
  digitalWrite(MRIN1, HIGH);
  digitalWrite(MRIN2, LOW);
  analogWrite(MRVREF, 255);
}

void right() {
  Serial.println("[ DEBUG ] motor : right");
  digitalWrite(MLIN1, HIGH);
  digitalWrite(MLIN2, LOW);
  analogWrite(MLVREF, 0);
  digitalWrite(MRIN1, LOW);
  digitalWrite(MRIN2, LOW);
  analogWrite(MRVREF, 255);
}

void back() {
  Serial.println("[ DEBUG ] motor : back");
  digitalWrite(MLIN1, LOW);
  digitalWrite(MLIN2, HIGH);
  analogWrite(MLVREF, 255);
  digitalWrite(MRIN1, LOW);
  digitalWrite(MRIN2, HIGH);
  analogWrite(MRVREF, 255);
}

void brake() {
  Serial.println("[ DEBUG ] motor : brake");
  digitalWrite(MLIN1, HIGH);
  digitalWrite(MLIN2, HIGH);
  analogWrite(MLVREF, 0);
  digitalWrite(MRIN1, HIGH);
  digitalWrite(MRIN2, HIGH);
  analogWrite(MRVREF, 0);
}

float calc_distance(float lat_0, float lon_0, float lat, float lon) {
  float dx = R * (lon - lon_0) * (PI / 180) * cos(lat * (PI / 180));
  float dy = R * (lat - lat_0) * (PI / 180);
  return sqrt(dx * dx + dy * dy);
}

float calc_angle(float lat_0, float lon_0, float lat, float lon) {
  float dx = R * (lon - lon_0) * (PI / 180) * cos(lat * (PI / 180));
  float dy = R * (lat - lat_0) * (PI / 180);
  return atan2(dy, dx) * (180 / PI);
}

void record(){
  
}
