// assign pin num
//Pins that control locomotive motors
const int M11 = A2;
const int M12 = A3;
const int M21 = A4;
const int M22 = A5;
/*int fd = 20;
const int trigPin1 = A0;
const int echoPin1 = A1;
float duration,cm;
*/
// duration for output
int time = 100;
// initial command
int command = 0;

/*float microsecondsToCentimeters(float microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29 / 2;
}
*/


void right(int time){
  digitalWrite(M11, HIGH);
  digitalWrite(M12, LOW);
  delay(time);
}

void left(int time){
  digitalWrite(M21, HIGH);
  digitalWrite(M22, LOW);
  delay(time);
}

void forward(int time){
  digitalWrite(M11, HIGH);
  digitalWrite(M12, LOW);
  digitalWrite(M21, HIGH);
  digitalWrite(M22, LOW);
  delay(time);
}

void reverse(int time){
  digitalWrite(M11, LOW);
  digitalWrite(M12, HIGH);
  digitalWrite(M21, LOW);
  digitalWrite(M22, HIGH);
  delay(time);
}

void forward_right(int time){
  digitalWrite(M11,LOW);
  digitalWrite(M12,HIGH);
  digitalWrite(M21,HIGH);
  digitalWrite(M22,LOW);
  delay(150);
  /*
  analogWrite(M11,0);
  analogWrite(M12,0);
  analogWrite(M21,0);
  analogWrite(M22,0);
  */
  digitalWrite(M11,HIGH);
  digitalWrite(M12,LOW);
  digitalWrite(M21,HIGH);
  digitalWrite(M22,LOW);
  delay(60);
  digitalWrite(M11,LOW);
  digitalWrite(M12,LOW);
  digitalWrite(M21,LOW);
  digitalWrite(M22,LOW);
  //delay(time);
}

void reverse_right(int time){
  digitalWrite(M11,HIGH);
  digitalWrite(M12,LOW);
  digitalWrite(M21,LOW);
  digitalWrite(M22,HIGH);
  delay(150);
  /*
  analogWrite(M11,0);
  analogWrite(M12,0);
  analogWrite(M21,0);
  analogWrite(M22,0);
  */
  digitalWrite(M11,LOW);
  digitalWrite(M12,HIGH);
  digitalWrite(M21,LOW);
  digitalWrite(M22,HIGH);
  delay(60);
  digitalWrite(M11,LOW);
  digitalWrite(M12,LOW);
  digitalWrite(M21,LOW);
  digitalWrite(M22,LOW);
  //delay(time);
}

void forward_left(int time){
  digitalWrite(M11,HIGH);
  digitalWrite(M12,LOW);
  digitalWrite(M21,LOW);
  digitalWrite(M22,HIGH);
  delay(150);
  /*
  analogWrite(M11,0);
  analogWrite(M12,0);
  analogWrite(M21,0);
  analogWrite(M22,0);
  */
  digitalWrite(M11,HIGH);
  digitalWrite(M12,LOW);
  digitalWrite(M21,HIGH);
  digitalWrite(M22,LOW);
  delay(60);
  digitalWrite(M11,LOW);
  digitalWrite(M12,LOW);
  digitalWrite(M21,LOW);
  digitalWrite(M22,LOW);
  //delay(time);
}

void reverse_left(int time){
  digitalWrite(M11,LOW);
  digitalWrite(M12,HIGH);
  digitalWrite(M21,HIGH);
  digitalWrite(M22,LOW);
  delay(150);
  /*
  analogWrite(M11,0);
  analogWrite(M12,0);
  analogWrite(M21,0);
  analogWrite(M22,0);
  */
  digitalWrite(M11,LOW);
  digitalWrite(M12,HIGH);
  digitalWrite(M21,LOW);
  digitalWrite(M22,HIGH);
  delay(60);
  digitalWrite(M11,LOW);
  digitalWrite(M12,LOW);
  digitalWrite(M21,LOW);
  digitalWrite(M22,LOW);
  //delay(time);
}

void reset(){
  digitalWrite(M11, LOW);
  digitalWrite(M12, LOW);
  digitalWrite(M21, LOW);
  digitalWrite(M22, LOW);
}

void send_command(int command, int time){

  
  switch (command){

     //reset command
     case 0: reset(); break;

     // single command
     case 1: forward(time); break;
     case 2: reverse(time); break;
     case 3: right(time); break;
     case 4: left(time); break;

     //combination command
     case 6: forward_right(time); break;
     case 7: forward_left(time); break;
     case 8: reverse_right(time); break;
     case 9: reverse_left(time); break;

     default: Serial.print("InValid Command\n");
    }
}

void setup() {
  pinMode(M11, OUTPUT);
  pinMode(M12, OUTPUT);
  pinMode(M21, OUTPUT);
  pinMode(M22, OUTPUT);
  Serial.begin(9600);
}

void loop() {
/*
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  duration = pulseIn(echoPin1, HIGH);
  cm = microsecondsToCentimeters(duration);
  Serial.println(cm);
  
  if (cm < fd)
  {
    reset();
//    continue;
  }
*/
  //receive command
  if (Serial.available() > 0){
    command = Serial.read();
  }
  else{
    reset();
  }
   send_command(command,time);
}


