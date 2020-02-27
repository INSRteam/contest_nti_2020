#include <MS5837.h>  //датчик глубины
#include <Wire.h>  //взаимодействие с I2C
#include <PID_v1.h> 

MS5837 sensor;

//концевики
const int con_compression = 7;  
const int con_release = 8; 

//движение
const int motor1_1 = 2;
const int motor1_2 = 3;
const int motor_depth = 5;

//манипулятор
const int motor2_1 = 10;
const int motor2_2 = 11;
const int motor_manipulator = 6;
const int power = 150;

int seconds;

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

void setPowerDepth(int power) {
  digitalWrite(motor2_1, HIGH);
  digitalWrite(motor2_2, LOW);
  analogWrite(motor_depth, power);
}

void setPowerManipulator(int power) {
  digitalWrite(motor2_1, HIGH);
  digitalWrite(motor2_2, LOW);
  analogWrite(motor_manipulator, power);
}

void compressionManipulator(double depth) {
  while (!digitalRead(con_compression)) {
    setPowerManipulator(power);
    setPowerDepth(pid.regulateDepth(depth));
  }
  setPowerManipulator(0);
}

void releaseManipulator(double depth) {
  while (!digitalRead(con_release)) {
     setPowerManipulator(-power);
     setPowerDepth(pid.regulateDepth(depth));
  }
  setPowerManipulator(0);
}

void setup() {

  setPoint = 0;                          // точка отсчета на нуле градусов
  Serial.begin(9600);  //Инициируем передачу данных в монитор последовательного порта, на скорости 9600 бит/сек
  Wire.begin();  //Инициализирует библиотеку Wire и подключается к шине I2C

  while (!sensor.init()) {
    Serial.println("Инициализация датчика глубины не удалась!");
    Serial.println("\n\n\n");
    delay(5000);
  }

  sensor.setModel(MS5837::MS5837_30BA);
  sensor.setFluidDensity(997); // кг/м**3 (пресная вода, 1029 для морской воды)
  pinMode(con_compression, INPUT);
  pinMode(con_release, INPUT);
  
  pinMode(motor1_1,OUTPUT); 
  pinMode(motor1_2,OUTPUT); 
  pinMode(motor_depth,OUTPUT);
  
  pinMode(motor2_1,OUTPUT);
  pinMode(motor2_2,OUTPUT);
  pinMode(motor_manipulator,OUTPUT);
} 

int time_task = millis()
void loop() {
  
  sensor.read();  // Обновление показаний 

  Serial.print("Глубина: "); 
  Serial.print(sensor.depth()); 
  Serial.println(" м");

  bool con = digitalRead(con_compression);  // чтение концевика 
  String s = "Value = " + string(con);
  Serial.println(s);    // Выводим в монитор порта значение концевика

  /*
   *
   * while(sensor.depth() < 1.95) {
   *  releaseManipulator(2);
   *  setPowerDepth(pid.regulate_depth(2));     
   * }
   * compressionManipulator(2);
   * while(sensor.depth() > 0.1) {
   *  setPowerDepth(pid.regulate_depth(0)); 
   * }
   * releaseManipulator(0);
   */
}
