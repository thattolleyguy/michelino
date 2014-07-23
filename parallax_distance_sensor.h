/**
 * @file newping_distance_sensor.h
 * @brief distance sensor driver for distance sensors supported by the NewPing library.
 * @author Miguel Grinberg
 */

#include "distance_sensor.h"

namespace Michelino {

    class DistanceSensor : public DistanceSensorDriver {
    public:

        DistanceSensor(int signalPinInit, int maxDistance)
        : DistanceSensorDriver(maxDistance), signalPin(signalPinInit) {
        }
        
        void initialize()
        {
            // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
            // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
//            pinMode(signalPin, OUTPUT);
//            digitalWrite(signalPin, LOW);
//            delayMicroseconds(2);
//            digitalWrite(signalPin, HIGH);
//            delayMicroseconds(5);
//            digitalWrite(signalPin, LOW);
//            pinMode(signalPin, INPUT);            
        }

        virtual unsigned int getDistance() {
//            long duration = pulseIn(signalPin, HIGH);

//            return microsecondsToCentimeters(duration);
                        return 0;
        }

        long microsecondsToCentimeters(long microseconds) {
            // The speed of sound is 340 m/s or 29 microseconds per centimeter.
            // The ping travels out and back, so to find the distance of the
            // object we take half of the distance travelled.
            return microseconds / 29 / 2;
        }
    private:
        int signalPin;
    };
};
