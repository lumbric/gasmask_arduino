#include <Arduino.h>
#include <Servo.h>

#define LED_PIN       13
#define SERVO_PIN     9
#define SENSOR_TRIG   7
#define SENSOR_ECHO   8

int min_pos           = 0;     // start point of servo in degree (between 0 and 180)
int max_pos           = 75;    // end point of servo in degree (between 0 and 180, must be larger than min_pos)
int delay_servo       = 1;     // increase value to make servo slower
int delay_time        = 50;    // time in milliseconds between 2 nuckels
long nuckle_dist_min  = 150;   // nuckels if distance is between min..
long nuckle_dist_max  = 600;   // max...
//long wait_between_min = 10

#define DEBUG 1   // set this to 0 to turn off debug output

Servo servo;
void nuckel();
long get_distance();

void setup() {
    Serial.begin (9600);
    servo.attach(SERVO_PIN);
    pinMode(LED_PIN, OUTPUT);
    pinMode(SENSOR_TRIG, OUTPUT);
    pinMode(SENSOR_ECHO, INPUT);
    digitalWrite(LED_PIN, LOW);
}

void loop() {
    DEBUG && Serial.println("Start nuckel...");
    int nuckel_times = (int) random(2, 5);
    for (int i = 0; i < nuckel_times; i++) {
        if (delay_until(delay_time) != 0) {
            return;
        }
        nuckel();
    }
    int wait_time = (int) random(1000, 10000);
    delay_until(wait_time);
}


/**
 * Delays delay_time if in nuckle distance and returns 0. Aborts if
 * not in nuckle distance and returns -1.
 */
int delay_until(unsigned long delay_time) {
    unsigned long start = millis();
    while (millis() - start < delay_time){
        long distance = get_distance();
        if (! (nuckle_dist_min < distance && distance < nuckle_dist_max ) ) {
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


/* Based on:
 HC-SR04 Ping distance sensor:
 VCC to arduino 5v 
 GND to arduino GND
 Echo to Arduino pin 7 
 Trig to Arduino pin 8
 
 This sketch originates from Virtualmix: http://goo.gl/kJ8Gl
 Has been modified by Winkle ink here: http://winkleink.blogspot.com.au/2012/05/arduino-hc-sr04-ultrasonic-distance.html
 And modified further by ScottC here: http://arduinobasics.blogspot.com/
 on 10 Nov 2012.
 */
long get_distance() {
    /* The following SENSOR_TRIG/SENSOR_ECHO cycle is used to determine the
    distance of the nearest object by bouncing soundwaves off of it. */
    long duration, distance;
    digitalWrite(SENSOR_TRIG, LOW); 
    delayMicroseconds(2); 

    digitalWrite(SENSOR_TRIG, HIGH);
    delayMicroseconds(10); 

    digitalWrite(SENSOR_TRIG, LOW);
    duration = pulseIn(SENSOR_ECHO, HIGH);

    //Calculate the distance (in cm) based on the speed of sound.
    distance = duration/58.2;

    DEBUG && Serial.println(String("Distance: ") + String(distance));
    return distance;
}
