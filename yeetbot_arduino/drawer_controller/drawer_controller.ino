const byte numMotors = 1;
const byte motorPinA[numMotors] = {2};
const byte motorPinB[numMotors] = {3};
const byte motorSenseA[numMotors] = {14};
const byte motorSenseB[numMotors] = {15};
const byte rfidPin[numMotors] = {4};

int voltageA[numMotors] = {0};
int voltageB[numMotors] = {0};
int state[numMotors];
int target[numMotors];
int content[numMotors];
unsigned long timer[numMotors];

void receive_command();

void setup() {
  // put your setup code here, to run once:
  for (int i = 0; i < numMotors; i++) {
    pinMode(motorPinA[i], OUTPUT);
    pinMode(motorPinB[i], OUTPUT);
    pinMode(rfidPin[i], INPUT);
    pinMode(motorSenseA[i], INPUT);
    pinMode(motorSenseB[i], INPUT);
    state[i] = 2;
    timer[i] = 0;
    content[i] = 0;
  }
  analogReference(EXTERNAL);
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  receive_command();
  for (int i = 0; i < numMotors; i++) {
    voltageA[i] = analogRead(motorSenseA[i]);
    voltageB[i] = analogRead(motorSenseB[i]);
    content[i] = digitalRead(rfidPin[i]);
//        Serial.print(state[i]);
//        Serial.print(" ");
//        Serial.print(voltageA[i]);
//        Serial.print(" ");
//        Serial.println(voltageB[i]);
    switch (state[i]) {
      case 0: //open
        digitalWrite(motorPinA[i], HIGH);
        digitalWrite(motorPinB[i], LOW);
        if (voltageA[i] < 900 && millis() - timer[i] > 500) {
          state[i] = 4;
          timer[i] = millis();
          //Serial.println("0 -> 4");
        }
        if (target[i] == 2) {
          state[i] = 1;
          //Serial.println("0 -> 1 (ext)");
          target[i] = 0;
        }
        break;

      case 1: //closing
        digitalWrite(motorPinA[i], LOW);
        digitalWrite(motorPinB[i], HIGH);
        if (voltageB[i] > 1000 && millis() - timer[i] > 1000) {
          state[i] = 2;
          //Serial.println("1 -> 2");
        }
        break;

      case 2: //closed
        digitalWrite(motorPinA[i], LOW);
        digitalWrite(motorPinB[i], HIGH);
//        if (voltageB[i] < 900 && millis() - timer[i] > 500) {
//          state[i] = 5;
//          timer[i] = millis();
//          //Serial.println("2 -> 5");
//        }
        if (target[i] == 1) {
          state[i] = 3;
          //Serial.println("2 -> 3 (ext)");
          target[i] = 0;
        }
        break;

      case 3: //opening
        digitalWrite(motorPinA[i], HIGH);
        digitalWrite(motorPinB[i], LOW);
        if (voltageA[i] > 1000 && millis() - timer[i] > 1000) {
          state[i] = 6;
          timer[i] = millis();
          //Serial.println("3 -> 0");
        }
        break;

      case 4: //fighting to keep open
        digitalWrite(motorPinA[i], HIGH);
        digitalWrite(motorPinB[i], LOW);
        if (millis() - timer[i] > 100) {
          if (voltageA[i] < 900) {
            state[i] = 1;
            timer[i] = millis();
            //Serial.println("4 -> 1");
          }
          else {
            timer[i] = millis();
          }
        }
        if (target[i] == 2) {
          state[i] = 1;
          //Serial.println("4 -> 1 (ext)");
          target[i] = 0;
        }
        break;

      case 5: //fighting to keep closed
        digitalWrite(motorPinA[i], LOW);
        digitalWrite(motorPinB[i], HIGH);
        if (millis() - timer[i] > 100) {
          if (voltageB[i] < 900) {
            state[i] = 3;
            timer[i] = millis();
            //Serial.println("5 -> 3");
          }
          else {
            timer[i] = millis();
          }
        }
        if (target[i] == 1) {
          state[i] = 3;
          //Serial.println("5 -> 3 (ext)");
          target[i] = 0;
        }
        break;

      case 6: //stop oscillations when open
        digitalWrite(motorPinA[i], LOW);
        digitalWrite(motorPinB[i], LOW);
        if (millis() - timer[i] > 500) {
          state[i] = 0;
          timer[i] = millis();
          //Serial.println("6 -> 0");
        }
        break;
    }
  }
}

void receive_command() {
  char cmd[6];
  cmd[5] = '\0';
  if (Serial.available()) {
    for (int i = 0; i < 5; i++) {
      char temp = Serial.read();
      delay(10);
      cmd[i] = temp;
    }
    if (cmd[0] == '>' && cmd[3] == '<') {
      //Serial.println("Successful reception!!");
      Serial.println(cmd);
      int tgt = cmd[1] - '1';
      //Serial.println(tgt);
      if (tgt <= numMotors) {
        switch (cmd[2]) {
          case 'o': //open drawer
            target[tgt] = 1;
            break;

          case 'c': //close drawer
            target[tgt] = 2;
            break;

          case 'i': //info about current state of drawer
            Serial.print('>');
            Serial.print(tgt + 1);
            Serial.print(state[tgt]);
            Serial.print(content[tgt]);
            Serial.println('<');
            break;
        }
      }
      else{
        Serial.println(">000<"); //code for "drawer does not exist"
      }
    }
  }
}
