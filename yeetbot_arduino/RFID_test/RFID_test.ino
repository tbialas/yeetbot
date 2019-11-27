//#include <SoftwareSerial.h>

const unsigned long tagNo = 0x5a7ca405;
const unsigned int tagSe = 0xf6;

const unsigned long tagNo2 = 0x35850e;
const unsigned int tagSe2 = 0x55;

byte ctrlPins[3] = {16, 15, 14};
boolean state = 0;
int lvl = 0;
boolean history[256];
int history_t[256];
int message[128];
int i = 0;
unsigned long timer = 0;
boolean xorout = false;
boolean delayout = false;
boolean ffout = false;
byte cc = 0;
unsigned long codes[6];
unsigned int seriess[6];
unsigned int strength[6];
//SoftwareSerial rcv(7, 8);

void next_coil();

void setup() {
  // put your setup code here, to run once:
  for (int i = 0; i < 3; i++) {
    pinMode(ctrlPins[i], OUTPUT);
    digitalWrite(ctrlPins[i], 0);
  }
  pinMode(3, OUTPUT);
  pinMode(4, INPUT);
  pinMode(5, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(17, INPUT);
  Serial.begin(115200);
  //  rcv.begin(2400);
  TCCR2B = TCCR2B & B11110000 | B00001001;
  TCCR2A = TCCR2A & B11111100 | B00000011;
  OCR2A = 127;
  analogWrite(3, OCR2A / 2);
  timer = micros();
}

void loop() {
  boolean oldst = state;
  state = digitalRead(4);
  if (state != oldst) {
    if (i < 256) {
      history[i] = state;
      history_t[i] = micros() - timer;
      i++;
      timer = micros();
    }
  }
  if (i >= 256 || micros() - timer > 1000000) {
    //    for (int j = 0; j < 256; j++) {
    //      Serial.print(history[j]);
    //      Serial.print(" ");
    //      Serial.println(history_t[j]);
    //    }
    byte temp = 0;
    int skip = 0;
    long test = 0;
    byte data[8];
    int k = 0;
    //    Serial.println("=================================");
    //    Serial.print("Coil ");
    //    Serial.println(cc);
    //    Serial.println("Decoding...");
    while (history_t[skip] < 380) skip++;
    //    Serial.println(skip);
    for (int j = skip; j < 256; j++) {
      int round_bit = 2;
      //      Serial.print(j);
      //      Serial.print(" ");
      //      Serial.print(history_t[j]);
      //      Serial.println(" ");
      if (history_t[j] < 100) {
        //        Serial.println("No tag!");
        i = 0;
        //next_coil();
        goto START;
      }
      if (history_t[j] > 380) {
        if (history[j] == 0) temp = 1;
        else if (history[j] == 1) temp = 0;
        round_bit = temp;
      }
      else if (history_t[j] < 380) {
        if (history[j] != temp) {
          round_bit = temp;
        }
      }
      if ((round_bit != 2)) {
        if (k < 128) message[k] = round_bit;
        k++;
      }
    }
    i = 0;
    k = 0;
    uint16_t frame = 0;
    int strt = 128;
    for (int j = 0; j < 128; j++) {
      frame = (frame << 1) & 1023;
      bitWrite(frame, 0, message[j]);
      if (frame == 511) {
        strt = j;
        break;
      }
    }
    int tst = 0;
//    for (int j = strt - 8; j < strt + 56; j++) {
//      Serial.print(message[j]);
//      tst++;
//      if (tst == 9 || tst == 15) {
//        Serial.println();
//        tst = 10;
//      }
//    }
//    Serial.println();
    //    Serial.println();
    //    Serial.println(strt);
    //    Serial.println();
START:
    if (strt < 64) {
      int ctr = 1;
      uint8_t series = 0;
      uint32_t code = 0;
      byte parity = 0;
      for (int e = 1; e >= 0; e--) {
        for (int g = 3; g >= 0; g--) {
//          Serial.print(strt + ctr);
//          Serial.print(" ");
//          Serial.print(parity);
//          Serial.print(" ");
//          Serial.println(message[strt + ctr]);
          bitWrite(series, g + e * 4, message[strt + ctr]);
          if (message[strt + ctr] == 1) parity ++;
          ctr++;
        }
        if (message[strt + ctr] == 1) parity ++;
//        Serial.println(parity);
//        Serial.println(parity%2);
//        Serial.println();
        parity = 0;
        ctr++;
      }
      for (int e = 7; e >= 0; e--) {
        for (int g = 3; g >= 0; g--) {
          //          Serial.print(strt + ctr);
          //          Serial.print(" ");
          //          Serial.println(message[strt + ctr]);
          bitWrite(code, g + e * 4, message[strt + ctr]);
          ctr++;
        }
        ctr++;
      }
      //      Serial.print("Code: ");
      //      Serial.print(series, HEX);
      //      Serial.print(" ");
      //      Serial.println(code, HEX);
      codes[cc] = code;
      seriess[cc] = series;
    }
    //Serial.println();
    next_coil();
    if (cc == 0) {
      int maxim = 0;
      int mx = 0;
      Serial.println("><><><><><><><><><><><><><><><><><><><><");
      for (int z = 0; z < 6; z++) {
        Serial.print(z);
        Serial.print(" ");
        Serial.print(seriess[z], HEX);
        Serial.print(" ");
        Serial.print(codes[z], HEX);
        Serial.print(" ");
        Serial.println(strength[z]);
        if (strength[z] > maxim) {
          maxim = strength[z];
          mx = z;
        }
      }
      if (maxim > 350) {
        Serial.print("Most likely code: ");
        Serial.print(seriess[mx], HEX);
        Serial.print(" ");
        Serial.println(codes[mx], HEX);
      }
      else {
        Serial.println("No tag found!");
      }
      Serial.println("><><><><><><><><><><><><><><><><><><><><");
      if ((seriess[mx] == tagSe && codes[mx] == tagNo)
          || (seriess[mx] == tagSe2 && codes[mx] == tagNo2)) {
        digitalWrite(13, HIGH);
      }
      else {
        digitalWrite(13, LOW);
      }
    }
    timer = micros();
  }
}

void next_coil() {
  //  Serial.println("NEXT");
  //  Serial.println("COIL");
  unsigned long average = 0;
  for (int s = 0; s < 100; s ++) {
    average += analogRead(17);
    delayMicroseconds(100);
  }
  //  Serial.println(average / 100);
  strength[cc] = average / 100;
  if (cc < 5) {
    //    Serial.println('a');
    //    Serial.println(cc);
    cc++;
    //    Serial.println(cc);
  }
  else {
    //    Serial.println('b');
    cc = 0;
  }
  for (int bb = 0; bb < 3; bb++) {
    digitalWrite(ctrlPins[bb], bitRead(cc, bb));
  }
  for (int z = 0; z < 128; z++) {
    message[z] = 0;
  }
  for (int z = 0; z < 256; z++) {
    history[z] = 0;
    history_t[z] = 0;
  }
  //delay(500);
}

