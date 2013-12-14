#include <Arduino.h>
#include <Servo.h>

#define LED_PIN       13
#define SERVO_PIN     9
#define SENSOR_TRIG   2
#define SENSOR_ECHO   3

int min_pos      = 0;
int max_pos      = 75;
int delay_servo  = 1;     // increase value to make servo slower
int delay_time   = 50;
long nuckle_dist = 30;

Servo servo;
void nuckel();
long get_distance();

void setup() {
    Serial.begin (9600);
    servo.attach(SERVO_PIN);
    pinMode(LED_PIN, OUTPUT);
    pinMode(SENSOR_TRIG, OUTPUT);
    pinMode(SENSOR_ECHO, INPUT);
}

void loop() {
    long distance = get_distance();
    Serial.println(String("Distance: ") + String(distance));

    if (distance < nuckle_dist) {
        int nuckel_times = (int) random(2, 5);
        for (int i = 0; i < nuckel_times; i++) {
            nuckel();
        }
        int wait_time = (int) random(1000, 10000);
        delay(wait_time);
    }
    else
        delay(500);
}


void nuckel() {
    for (int i = min_pos; i <= max_pos; i++) {
        servo.write(i);
        delay(delay_servo);
    }
    for (int i = max_pos; i >= min_pos; i--) {
        servo.write(i);
        delay(delay_servo);
    }
    delay(delay_time);
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

    return distance;
}
