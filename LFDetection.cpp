#include <LFDetection.h>
#include <Arduino.h>

void LFDetection::BlockDetection(){
    // Not robust.
    if (DS_data < min_dist){
        min_dist = DS_data;
    }
    if (DS_data > max_dist){
        max_dist = DS_data;
    }
    if (max_dist - min_dist >= dip_threshold){
        block_found = 1;
    }
    /*
    int sweep_deq = sweepQueue.dequeue();
    if (sweep_deq < min_dist){
        min_dist = sweep_deq;
    }
    sweepQueue.enqueue(DS_data);
    if (DS_data > max_dist){
        max_dist = DS_data;
    }
    */

}


void LFDetection::LFDataRead()
{
    L1LF_data = digitalRead(L1LF_receive);
    R1LF_data = digitalRead(R1LF_receive);

    if (main_loop_counter % 10 == 0){
      Serial.println("Sensor Data 1 2: " + String(L1LF_data) + " " + String(R1LF_data));
    }
}

void LFDetection::DSDataRead()
{
    DS_data = digitalRead(DS_receive);

    if (main_loop_counter % 10 == 0){
      Serial.println("Distance sensor: " + String(DS_data));
    }
}

void LFDetection::EdgeDetection() // Testing using L1 and R1.
{
    int Ldeq = L2Queue.dequeue();
    int Rdeq = R2Queue.dequeue();
    if (l_intxn_deb == 0){ // Debounce not finished.
        L2Queue.enqueue(0);
    }
    else{
        L2Queue.enqueue(L1LF_data); // to be changed to L2
    }
    if (r_intxn_deb == 0){ // Debounce not finished.
        R2Queue.enqueue(0);
    }
    else{
        R2Queue.enqueue(R1LF_data); // to be changed to L2
    }
    // L2Queue.enqueue(L1LF_data); // to be changed to L2
    // R2Queue.enqueue(R1LF_data); // to be changed to R2

    if (L1LF_data == 1){
        left_white_counter++;
        // Serial.println("L1 ENqueued 1");
    }
    if (Ldeq == 1){
        left_white_counter--;
        // Serial.println("L1 DEqueued 1");
    }
    if (R1LF_data == 1){
        right_white_counter++;
        // Serial.println("R1 ENqueued 1");
    }
    if (Rdeq == 1){
        right_white_counter--;
        // Serial.println("R1 DEqueued 1");
    }
    /*
    if (main_loop_counter % 10 == 0){
        Serial.println("left_White_Counter: " + String(left_white_counter));
        Serial.println("right_White_Counter: " + String(right_white_counter));
    }
    */
}

/*
void LFDetection::TurnDetection()
{
    return;
}
*/

void LFDetection::IntersectionDetection()
{
    if (current_time - l_intxn_deb_prev > intxn_deb_time){
        l_intxn_deb = 1;
    }
    if (current_time - r_intxn_deb_prev > intxn_deb_time){
        r_intxn_deb = 1;
    }
    if (left_white_counter == intxn_detection_threshold) {
        // Debounce after recorded an intersection.
        l_intxn_deb_prev = current_time;
        l_intxn_deb = 0;

        // Record the intersection.
        left_intxn_counter++;
        Serial.println("Left Intersection " + String(left_intxn_counter) + " recorded. ");

        // Reset Line follower counter.
        left_white_counter = 0;
        for (int i=0;i<intxn_queue_length;i++){
            L2Queue.dequeue();
            L2Queue.enqueue(0);
        }
    }
    if (right_white_counter == intxn_detection_threshold) {
        // Debounce after recorded an intersection.
        r_intxn_deb_prev = current_time;
        r_intxn_deb = 0;

        // Record the intersection.
        right_intxn_counter++;
        Serial.println("Right Intersection " + String(right_intxn_counter) + " recorded. ");

        // Reset Line follower counter.
        right_white_counter = 0;
        for (int i=0;i<intxn_queue_length;i++){
            R2Queue.dequeue();
            R2Queue.enqueue(0);
        }
    }
}
