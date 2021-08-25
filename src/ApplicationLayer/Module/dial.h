#pragma once

#include <stdint.h>
#include "pidController.h"

class Dial {
      protected:
        Dial();
        virtual ~Dial() {}

        virtual float getAngle() = 0;
        virtual float getVelocity() = 0;
      public:
        void setEnable(bool enable);
        bool getEnable();
        void update();
        float getDuty();
        void incrementPosition();        
        void decrementPosition();
        void setDivisionNum(uint8_t num);
        uint8_t getDialPosition();
        uint8_t getDivisionNum();
        void setPiGain(float p_gain, float i_gain, float i_limit, float limit);
        void reset();
        void debug();
      private:        
        static constexpr float RAD2DEG = 180.0f / 3.14159265f;
        static constexpr float DEG2RAD = 3.14159265f / 180.0f;
        AngPidfController _ang_pidf;
        bool _enable;
        uint8_t _dial_position;
        uint8_t _division_num;
        float _duty;
        float _p_gain;
        float _i_gain;
        float _i_limit;
        float _limit;

    };
