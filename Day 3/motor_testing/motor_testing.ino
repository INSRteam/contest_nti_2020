const int motor1_1 = 2;
const int motor1_2 = 3;
const int motor_depth = 5;


void setup() {
  // initialize serial:
  Serial.begin(9600);
  // make the pins outputs:
  pinMode(motor1_1, OUTPUT);
  pinMode(motor1_2, OUTPUT);  
  pinMode(motor_depth, OUTPUT);
  

}

void loop() {
  // if there's any serial available, read it:
  while (Serial.available() > 0) {

    // look for the next valid integer in the incoming serial stream:
    //int motor_1 = Serial.parseInt();
    //int motor_2 = Serial.parseInt();
    int power = Serial.parseInt();
    
    // do it again

    // look for the newline. That's the end of your sentence:
    if (Serial.read() == '\n') {
      // constrain the values to 0 - 255 and invert
      //motor_1 = constrain(motor_1, 0, 1);
    // motor_2 = constrain(motor_2, 0, 1);
      //if(motor_power_12 < 0) {
        analogWrite(motor_depth, power);
        digitalWrite(motor1_1, HIGH);
        digitalWrite(motor1_2, LOW);
        
        Serial.println(power);
       /* }
       // else {
          analogWrite(motor_pwm_12, motor_power_12);
          digitalWrite(motor1, 0);
        digitalWrite(motor2, 1);
         }*/
    }
  }
}
