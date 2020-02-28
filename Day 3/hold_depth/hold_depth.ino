#include <MS5837.h>  //датчик глубины
#include <Wire.h>  //взаимодействие с I2C
#include <PID_v1.h> 

MS5837 sensor;

//движение
const int motor1_1 = 2;
const int motor1_2 = 3; 
const int motor_depth = 5;

class P_regulator {

private:
  // ПИД переменные
  double kp;
  double ki;
  double kd;
 
  unsigned long currentTime, previousTime;
  double elapsedTime;
  double error;
  double lastError;
  double input, output, setPoint;

public:
  // конструктор ПИД класса
  P_regulator(int k_p, int k_i, int k_d) {
    kp = k_p;
    ki = k_i;
    kd = k_d;
  }
  //вычисление П регулятора
  double regulateDepth(double target_depth) {
    sensor.read();
    currentTime = millis();  // получить текущее время
    elapsedTime = (double)(currentTime - previousTime);        // вычислительное время, прошедшее с момента предыдущего вычисления
          
    error = target_depth - sensor.depth();                                // определение ошибки
    //cumError += error * elapsedTime;                // вычесление интеграла
    // rateError = (error - lastError)/elapsedTime;    // вычесление производной
  
    double out = kp * error; //+ ki*cumError +   kd*rateError;                // выход П регулятора            
   
    lastError = error;                                // запомнить текущую ошибку
    previousTime = currentTime;                        // запомнить текущую время
   
     return out;                                        // выход П функции
  
    }
}

P_regulator pid = new P_regulator(2, 0, 0);




// мощность для глубины
void setPowerDepth(int power) {
  digitalWrite(motor2_1, HIGH);
  digitalWrite(motor2_2, LOW);
  analogWrite(motor_depth, power);
}

void setup() {
  pinMode(motor1_1,OUTPUT); 
  pinMode(motor1_2,OUTPUT); 
  pinMode(motor_depth,OUTPUT);

  Serial.begin(9600);

 
  while (!sensor.init()) {
    Serial.println("Инициализация датчика глубины не удалась!");
    Serial.println("\n\n\n");
    delay(5000);
  }

  sensor.setModel(MS5837::MS5837_30BA);
  sensor.setFluidDensity(1000);
  
}

int time_task = millis();
seconds = 10;

void loop() {

   sensor.read();  // Обновление показаний

   Serial.print("Глубина: "); 
   Serial.print(sensor.depth()); 
   Serial.println(" м");

   while (sensor.depth() < 1.3) {
   setPowerDepth(pid.regulate_depth(1.4));
    
    }
   while (time_task - millis() <= seconds * 1000) {
    setPowerDepth(pid.regulate_depth(1.4));
    }  
    
   while (true) setPowerDepth(pid.regulate_depth(0));

}
