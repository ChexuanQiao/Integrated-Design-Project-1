#include <ArduinoQueue.h>
const int DS_receive;
const int main_loop_delay_time = 100; // main loop delay.

const int sweep_queue_length = 10; // Distance sensor sweep queue length.
const int front_queue_length = 5;
const int back_queue_length = 5;

const int debounce = 1000;

float front_avg = 0;
float back_avg = 0;

float front_front;
float mid_front;
float back_front;

const int avg_l = 5;
float dip_threshold = 4;

int block_found = 0;
unsigned long search_time = 0;
int DS_data;
int front = 0;
int back = 0;


ArduinoQueue<int> Q = ArduinoQueue<int>(sweep_queue_length);
ArduinoQueue<int> F = ArduinoQueue<int>(front_queue_length);
ArduinoQueue<int> B = ArduinoQueue<int>(back_queue_length);

void setup(){
  Serial.begin(9600);
  pinMode(DS_receive, INPUT);
}

void BlockDetect(){
  
  while (!F.isFull()){
    DS_data = analogRead(DS_receive);
    F.enqueue( DS_data );
    front_avg += DS_data / front_queue_length;
  }
  while (!Q.isFull()){
    DS_data = analogRead(DS_receive);
    Q.enqueue( DS_data );
  }
  while (!B.isFull()){
    DS_data = analogRead(DS_receive);
    B.enqueue( DS_data );
    back_avg += DS_data / back_queue_length;
  }
  
  while (true){
    DS_data = analogRead(DS_receive);
    front_front = F.dequeue();
    mid_front = Q.dequeue();
    back_front = B.dequeue();
    
    F.enqueue(mid_front);
    Q.enqueue(back_front);
    B.enqueue(DS_data);
    
    front_avg -= front_front / front_queue_length;
    front_avg += mid_front / front_queue_length;
    back_avg -= back_front / back_queue_length;
    back_avg += DS_data / back_queue_length;
    
    Serial.println("Front Average: " + String(front_avg));
    Serial.println("Back  Average: " + String(back_avg));
    
    if (front_avg - back_avg > dip_threshold){
      Serial.println("Block Found.");
      break;
    
  }
}
}

void loop(){
  current_time = millis();
  BlockDetect();
  delay(500);
}
  
