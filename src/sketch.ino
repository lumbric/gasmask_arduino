#include <Arduino.h>
#include <Servo.h>
#include <NewPing.h>


#define LED_PIN       13
#define SERVO_PIN     9
#define SENSOR_TRIG   7
#define SENSOR_ECHO   7


/***** Config ******/
// one nuckle is a servo movement in and out again
const int min_pos           = 0;         // start point of servo in degree (between 0 and 180)
const int max_pos           = 75;        // end point of servo in degree (between 0 and 180, must be larger than min_pos)
const int delay_servo       = 1;         // increase value to make servo slower
const int delay_time        = 50;        // time in milliseconds between 2 nuckels
const long nuckle_dist      = 150;       // nuckels if distance is nobody is closer than nuckle_dist
long wait_between_min       = 1 * 1000;  // time between to wait between two nuckle sets in milliseconds
long wait_between_max       = 10 * 1000; // time between to wait between two nuckle sets in milliseconds
int number_nuckles_min      = 2;         // nummber of nuckles in one nuckle set
int number_nuckles_max      = 4;         // nummber of nuckles in one nuckle set
/***** Config ******/

#define DEBUG 1   // set this to 0 to turn off debug output

Servo servo;
// see also: http://code.google.com/p/arduino-new-ping/wiki/NewPing_Single_Pin_Sketch
NewPing sonar(SENSOR_TRIG, SENSOR_TRIG, nuckle_dist);
void nuckel();
long get_distance();
long last_distance = 0;
unsigned long last_measured = 0;


void setup() {
    Serial.begin (9600);
    servo.attach(SERVO_PIN);
    pinMode(LED_PIN, OUTPUT);
    pinMode(SENSOR_TRIG, OUTPUT);
    pinMode(SENSOR_ECHO, INPUT);
    digitalWrite(LED_PIN, LOW);
}


void loop() {
    int nuckel_times = (int) random(number_nuckles_min, number_nuckles_max + 1);
    for (int i = 0; i < nuckel_times; i++) {
        if (delay_until(delay_time) != 0) {
            return;
        }
        nuckel();
    }
    int wait_time = (int) random(wait_between_min, wait_between_max);
    delay_until(wait_time);
}


/**
 * Delays delay_time if in nuckle distance and returns 0. Aborts if
 * not in nuckle distance and returns -1.
 */
int delay_until(unsigned long delay_time) {
    unsigned long start = millis();
    while (millis() - start < delay_time){
        if (nuckle_dist > get_distance()) {
            return -1;
        }
    }
    return 0;
}


/**
 * pacifier once in and out again
 */
void nuckel() {
    digitalWrite(LED_PIN, HIGH);
    for (int i = min_pos; i <= max_pos; i++) {
        servo.write(i);
        delay(delay_servo);
    }
    for (int i = max_pos; i >= min_pos; i--) {
        servo.write(i);
        delay(delay_servo);
    }
    digitalWrite(LED_PIN, LOW);
}


long get_distance() {
    if (millis() - last_measured < 50)
        return last_distance;

    unsigned int uS = sonar.ping();
    last_measured = millis();
    last_distance = uS / US_ROUNDTRIP_CM;
    Serial.print("Distance: ");
    Serial.println(last_distance);

    // 0 means max distance exceeded
    if (last_distance == 0)
        return nuckle_dist;
    // Convert ping time to distance and print result (0 = outside set distance range, no ping echo)
    return last_distance;
}
