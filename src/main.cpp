#include <Arduino.h>
#include "pins.hpp"
#include "kart.hpp"
#include "ld06.hpp"
#include "dma.hpp"
#include "logger.hpp"
#include "pure_pursuit.hpp"
#include "f1tenth_gap_follow.hpp"
#include "naive_gap_follow.hpp"

// Robot control
TinyKart *tinyKart;

// LiDAR
LD06 ld06{};

// Scan processor
ScanBuilder scan_builder{180, 360, ScanPoint{0.1524, 0}};

/// Starts/stops the kart
void estop() {
    logger.printf("Toggle Pause\n");

    tinyKart->toggle_pause();
    digitalToggle(LED_YELLOW);
}

void setup() {
    // LEDs
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_YELLOW, OUTPUT);

    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_YELLOW, HIGH);

    // Setup blue user button on the board to stop the kart
    pinMode(USER_BTN, INPUT);
    attachInterrupt(digitalPinToInterrupt(USER_BTN), estop, FALLING);

    // Init PWM
    analogWriteResolution(PWM_BITS); // Range of 0-4096
    analogWriteFrequency(PWM_FREQ);

    // Prepare kart for motion
    ESC esc{THROTTLE_PIN, PWM_MAX_DUTY, PWM_FREQ};
    tinyKart = new TinyKart{STEERING_PIN, esc, 0.3, 4.5};

    // Init DMA and UART for LiDAR
    dmaSerialRx5.begin(230'400, [&](volatile LD06Buffer buffer) {
        // On each packet received, copy over to driver.
        ld06.add_buffer(buffer, 47);
    });

    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_GREEN, LOW);
}

int status = 0;

void loop() {
    noInterrupts();
    auto res = ld06.get_scan();
    interrupts();

    // Check if we have a scan frame
    if (res) {
        auto scan_res = *res;

        // Check if frame erred
        if (scan_res) {
            auto maybe_scan = scan_builder.add_frame(scan_res.scan);

            // Check if we have a 180 degree scan built
            if (maybe_scan) {
                auto scan = *maybe_scan;

                auto front_obj_dist = scan[scan.size() / 2].dist(ScanPoint::zero());

                //Find point and set appropriate throttle and steering
                auto maybe_target_pt = gap_follow::find_gap_naive(scan,10,2);
                auto target_pt = *maybe_target_pt;
                
                auto command = pure_pursuit::calculate_command_to_point(tinyKart, target_pt, 1.0);

                    // Set throttle proportional to distance to point in front of kart
                command.throttle_percent = mapfloat(front_obj_dist, 0.1, 10.0, 0.17, tinyKart->get_speed_cap());

                if(target_pt.x > .05)
                {
                    command.steering_angle = -1*command.steering_angle;
                }

                //Scan is OK, begin driving logic
                switch (status)
                {
                case 0: //STANDBY
                    if(front_obj_dist != 0.0 && front_obj_dist > .60)
                    {
                        status = 1;
                    } 
                    break;
                case 1://NO OBSTACLE; DRIVING STRAIGHT    
                    digitalWrite(LED_GREEN, HIGH);
                    digitalWrite(LED_RED, LOW); 
                    tinyKart->set_steering(0);
                    tinyKart->set_forward(.20);   
                    if(front_obj_dist != 0.0 && front_obj_dist < 0.45 + 0.1524) //STOP at dist .45
                    {
                        logger.printf("Stopping because of object: %himm in front! \n", (int16_t) (front_obj_dist * 1000));
                        tinyKart->set_reverse(.17);
                        delay(100);
                        tinyKart->set_neutral();
                        status = 0;
                        digitalWrite(LED_RED, HIGH);
                        digitalWrite(LED_GREEN, LOW);
                    }
                    if(front_obj_dist != 0.0 && front_obj_dist < 1.0 && status == 1) //STEER AWAY at dist < 1
                    {
                        status = 2;
                        
                        digitalWrite(LED_YELLOW, HIGH);
                        digitalWrite(LED_GREEN, LOW);
                    }
                    break;
                case 2: //OBSTACLE DETECTED; GAP FOUND; STEERING TOWARDS GAP
                    {
                    // Actuate kart
                        tinyKart->set_forward(command.throttle_percent);
                        tinyKart->set_steering(command.steering_angle);
                    }
                    if(front_obj_dist != 0.0 && front_obj_dist < 0.35 + 0.1524) //STOP at dist .35
                    {
                        logger.printf("Stopping because of object: %himm in front! \n", (int16_t) (front_obj_dist * 1000));
                        tinyKart->set_reverse(.17);
                        delay(100);
                        tinyKart->set_neutral();
                        status = 0;
                        digitalWrite(LED_RED, HIGH);
                        digitalWrite(LED_GREEN, LOW);
                    }
                    if (front_obj_dist != 0.0 && front_obj_dist > 1) //IF no obstacle in 1m, set back to status 1
                    {
                        status = 1;
                        digitalWrite(LED_YELLOW, LOW);
                    }  
                    break;
                default:
                    break;
                }
            }

        //Something has gone wrong with the scan
        } else {
            switch (scan_res.error) {
                case ScanResult::Error::CRCFail:
                    logger.printf("CRC error!\n");
                    break;

                case ScanResult::Error::HeaderByteWrong:
                    logger.printf("Header byte wrong!\n");
                    break;
            }
        }
    }
}