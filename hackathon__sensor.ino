const int FORWARD_ECHO_PIN = 8;
const int FORWARD_TRIG_PIN = 9;
const int LEFT_ECHO_PIN = 10;
const int LEFT_TRIG_PIN = 11;
const int RIGHT_ECHO_PIN = 12;
const int RIGHT_TRIG_PIN = 13;
const int MLEFT_FORWARD = 2;
const int MLEFT_BACKWARD = 3;
const int MRIGHT_FORWARD = 4;
const int MRIGHT_BACKWARD = 5;

float f_duration, f_distance;
float l_duration, l_distance; 
float r_duration, r_distance; 





void setup() {  
  pinMode(FORWARD_TRIG_PIN, OUTPUT);  
  pinMode(FORWARD_ECHO_PIN, INPUT);  
  pinMode(LEFT_TRIG_PIN, OUTPUT);  
  pinMode(LEFT_ECHO_PIN, INPUT); 
  pinMode(RIGHT_TRIG_PIN, OUTPUT);  
  pinMode(RIGHT_ECHO_PIN, INPUT); 
  pinMode(MLEFT_FORWARD, OUTPUT); 
  pinMode(MLEFT_BACKWARD, OUTPUT); 
  pinMode(MRIGHT_FORWARD, OUTPUT); 
  pinMode(MRIGHT_BACKWARD, OUTPUT); 
  Serial.begin(9600);  
}  


void loop()
{
  go_forward();
  turn_left();


  
  
  
}

void f_dist(){
  digitalWrite(FORWARD_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(FORWARD_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(FORWARD_TRIG_PIN, LOW);

  f_duration = pulseIn(FORWARD_ECHO_PIN, HIGH);
  f_distance = (f_duration*.0343)/2;
  Serial.print("Front Distance: ");
  Serial.println(f_distance);
}
  
void l_dist(){
  digitalWrite(LEFT_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(LEFT_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(LEFT_TRIG_PIN, LOW);

  l_duration = pulseIn(LEFT_ECHO_PIN, HIGH);
  l_distance = (l_duration*.0343)/2;
  Serial.print("Left Distance: ");
  Serial.println(l_distance);
}

void r_dist(){
  digitalWrite(RIGHT_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(RIGHT_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(RIGHT_TRIG_PIN, LOW);


  r_duration = pulseIn(RIGHT_ECHO_PIN, HIGH);
  r_distance = (r_duration*.0343)/2;
  Serial.print("Right Distance: ");
  Serial.println(r_distance);
  delay(100);

}

void turn_left(){
  digitalWrite(MLEFT_FORWARD,LOW);
  digitalWrite(MRIGHT_BACKWARD,LOW);
  digitalWrite(MLEFT_BACKWARD,HIGH);
  digitalWrite(MRIGHT_FORWARD,HIGH);
  delayMicroseconds(3000);
  digitalWrite(MLEFT_BACKWARD,LOW);
  digitalWrite(MRIGHT_FORWARD,LOW);
  
}

void turn_right(){
  
  digitalWrite(MLEFT_FORWARD,HIGH);
  digitalWrite(MRIGHT_BACKWARD,HIGH);
  delayMicroseconds(3000);
  digitalWrite(MLEFT_FORWARD,LOW);
  digitalWrite(MRIGHT_BACKWARD,LOW);
  
}


void go_forward(){
  
  digitalWrite(MLEFT_FORWARD,HIGH);
  digitalWrite(MRIGHT_FORWARD,HIGH);
  delayMicroseconds(3000);


}

void go_back(){
  
  digitalWrite(MLEFT_BACKWARD,HIGH);
  digitalWrite(MRIGHT_BACKWARD,HIGH);
  delayMicroseconds(3000);

 
}

void STOP(){
  digitalWrite(MLEFT_BACKWARD,LOW);
  digitalWrite(MRIGHT_FORWARD,LOW);
  digitalWrite(MRIGHT_BACKWARD,LOW);
  digitalWrite(MLEFT_FORWARD,LOW);
  
}
