#include <MS5837.h>  //датчик глубины
#include <Wire.h>  //взаимодействие с I2C
#include <PID_v1.h> 

MS5837 sensor;

//концевики
const int con1 = 7;  
const int con2 = 8; 

//движение
const int motor1_1 = 2;
const int motor1_2 = 3;
const int motor1_speed = 5;

//манипулятор
const int motor2_1 = 10;
const int motor2_2 = 11;
const int motor2_speed = 6;

class P_regulator {

private:
  // ПИД переменные
  double kp = 2;
  double ki = 0;
  double kd = 0;
 
  unsigned long currentTime, previousTime;
  double elapsedTime;
  double error;
  double lastError;
  double input, output, setPoint;
  //double cumError, rateError;

/*  
public setKp(int kp) {
  this.kp = kp;
}
*/

//вычисление П регулятора
double computeP() {
 currentTime = millis();  // получить текущее время
 elapsedTime = (double)(currentTime - previousTime);        // вычислительное время, прошедшее с момента предыдущего вычисления
        
  error = Setpoint - inp;                                // определение ошибки
  //cumError += error * elapsedTime;                // вычесление интеграла
  // rateError = (error - lastError)/elapsedTime;    // вычесление производной

  double out = kp*error //+ ki*cumError +   kd*rateError;                // выход П регулятора            
 
  lastError = error;                                // запомнить текущую ошибку
  previousTime = currentTime;                        // запомнить текущую время
 
   return out;                                        // выход П функции

  }
}

P_regulator pid = new P_regulator();

void setup() {

  setPoint = 0;                          // точка отсчета на нуле градусов
  Serial.begin(9600);  //Инициируем передачу данных в монитор последовательного порта, на скорости 9600 бит/сек
  Wire.begin();  //Инициализирует библиотеку Wire и подключается к шине I2C

  while (!sensor.init()) {
    Serial.println("Иницилизация датчика глубины не удалась!");
    Serial.println("\n\n\n");
    delay(5000);
  }

  sensor.setModel(MS5837::MS5837_30BA);
  sensor.setFluidDensity(997); // кг/м**3 (пресная вода, 1029 для морской воды)

  
  pinMode(motor1_1,OUTPUT);  
  pinMode(motor1_2,OUTPUT); 
  pinMode(motor1_speed,OUTPUT);
  
  pinMode(motor1_1,OUTPUT); 
  pinMode(motor1_2,OUTPUT); 
  pinMode(motor1_speed,OUTPUT);
  
  pinMode(motor2_1,OUTPUT);
  pinMode(motor2_2,OUTPUT);
  pinMode(motor2_speed,OUTPUT);
} 

void loop() {

  sensor.read();  // Обновление показаний 

  Serial.print("Глубина: "); 
  Serial.print(sensor.depth()); 
  Serial.println(" м");

  currentTime = millis();
  
  while (millis() - currentTime < seconds * 1000) {
    digitalWrite(motor1_1, HIGH);
    digitalWrite(motor1_2, HIGH);
    analogWrite(motor2_speed, pid.computeP());
  }
   digitalWrite(motor2_1, HIGH);
   digitalWrite(motor2_2, HIGH);

   
  }

 
  bool con = analogRead(A6);  // чтение концевика 
  
  Serial.print("Value = ");    // Выводим в монитор порта текст
  Serial.println(digitalRead(conPin));  // Считываем значение сигнала с вывода, к которому подключен концевик и выводим значение в монитор порта

}
