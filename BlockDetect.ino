#include <ArduinoQueue.h>
const int DS_receive;
const int main_loop_delay_time = 100; // main loop delay.

const int sweep_queue_length = 10; // Distance sensor sweep queue length.
const int front_queue_length = 5;
const int back_queue_length = 5;

float front_avg = 0;
float end_avg = 0;


const int avg_l = 5;
float threshold = 4;

int block_found = 0;
unsigned long search_time = 0;
int DS_data;
int front = 0;
int back = 0;

ArduinoQueue<int> Q = ArduinoQueue<int>(sweep_queue_length);
ArduinoQueue<int> F = ArduinoQueue<int>(front_queue_length);
ArduinoQueue<int> B = ArduinoQueue<int>(back_queue_length);

void setup(){
  pinMode(DS_receive, INPUT);
}

void BlockDetect(){
  DS_data = analogRead(DS_receive);
  while (!F.isFull()){
    F.enqueue( DS_data );
    front_avg += DS_data / front_queue_length;
  }
  while (!F.isFull()){
    Q.enqueue( DS_data );
  }
  while (!F.isFull()){
    B.enqueue( DS_data );
    back_avg += DS_data / back_queue_length;
  }
  
  while (true){
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

void loop(){
  BlockDetect();
  delay(500);
}
  
