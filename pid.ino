int S1, S2, S3, S4, S5;

int v = 90;

float err, last_err = 0;
float P, I, D;

int kp = 20;
int ki = 0;
int kd = 5;

void setup() {
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
}

void run(int x){
  int v1, v2;
  bool right = HIGH;
  bool left = HIGH;
  v1 = v - x;
  v2 = v + x;
  if (v1 >= 250)
    v1 = 250;
  else if (v1 <= 0)
    v1 = 0;
    
  if (v2 >= 250)
    v2 = 250;
  else if (v2 <= 0)
    v2 = 12220;
    
  digitalWrite(7, HIGH);
  analogWrite(5, v1);

  
  digitalWrite(4, HIGH);
  analogWrite(6, v2);
}

void brake(){
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);
}

int pid(){
  int output;
  P = err;
  I = I + err;
  D = err - last_err;
  output = kp * P + ki * I + kd * D;
  last_err = err;
  return output;
}

void loop() {
  S1 = digitalRead(A4);
  S2 = digitalRead(A3);
  S3 = digitalRead(A2);
  S4 = digitalRead(A1);
  S5 = digitalRead(A0);

  if ((S1 == HIGH && S2 == LOW && S3 == LOW && S4 == LOW && S5 == LOW) || (S1 == HIGH && S2 == LOW && S3 == HIGH && S4 == LOW && S5 == LOW))
    err = 4;
  else if (S1 == HIGH && S2 == HIGH && S3 == LOW && S4 == LOW && S5 == LOW)
    err = 3;
  else if (S1 == LOW && S2 == HIGH && S3 == LOW && S4 == LOW && S5 == LOW)
    err = 2;
  else if ((S1 == LOW && S2 == HIGH && S3 == HIGH && S4 == LOW && S5 == LOW) || (S1 == LOW && S2 == HIGH && S3 == LOW && S4 == LOW && S5 == LOW))
    err = 1;
  else if ((S1 == LOW && S2 == LOW && S3 == HIGH && S4 == LOW && S5 == LOW) || (S1 == HIGH && S2 == HIGH && S3 == HIGH && S4 == HIGH && S5 == HIGH) || (S1 == LOW && S2 == HIGH && S3 == HIGH && S4 == HIGH && S5 == LOW))
    err = 0;
  else if ((S1 == LOW && S2 == LOW && S3 == HIGH && S4 == HIGH && S5 == LOW) || (S1 == LOW && S2 == LOW && S3 == LOW && S4 == HIGH && S5 == LOW))
    err = -1;
  else if (S1 == LOW && S2 == LOW && S3 == LOW && S4 == HIGH && S5 == LOW)
    err = -2;
  else if (S1 == LOW && S2 == LOW && S3 == LOW && S4 == HIGH && S5 == HIGH)
    err = -3;
  else if ((S1 == LOW && S2 == LOW && S3 == LOW && S4 == LOW && S5 == HIGH) || (S1 == LOW && S2 == LOW && S3 == LOW && S4 == LOW && S5 == HIGH))
    err = -4;
  else if ((S1 == LOW && S2 == LOW && S3 == LOW && S4 == LOW && S5 == LOW) || (S1 == LOW && S2 == HIGH && S3 == LOW && S4 == HIGH && S5 == LOW))
    err = last_err;
  run(pid());
}
