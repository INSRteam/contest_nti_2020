const int con_compression = 4;  
const int con_release = 7; 

void setup() {
  
pinMode(con_compression,INPUT);
pinMode(con_release,INPUT);

Serial.begin(9600);
}

void loop() {

  
  bool con1 = digitalRead(con_compression);  // чтение концевика 
  bool con2 = digitalRead(con_release);  // чтение концевика 
  String s = "Value con1 = " + String(con1) + "; Value con2 = " + String(con2);
  Serial.println(s);    // Выводим в монитор порта значение концевика
  delay(100);


/*


    // look for the next valid integer in the incoming serial stream:
    bool r = analogRead(A6); 

    // look for the newline. That's the end of your sentence:
   
      Serial.println(analogRead(A6));
      if (r>0) { 
        digitalWrite(4,HIGH);
        digitalWrite(5,LOW); 
  }  
  if (r<0) { 
    digitalWrite(4,LOW);
  digitalWrite(5,HIGH); 
  }
  if (!r) {
  digitalWrite(4,LOW);
  digitalWrite(5,LOW); 
  }
    */
  
  


}
