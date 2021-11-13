#include "searchRunActivity.h"

// Hal
#include "hal_timer.h"


namespace activity{    
    constexpr float DEG2RAD = 3.14159265f / 180.0f;

    std::string SearchRunActivity::getModeName()
    {
        std::string mode_name = "SearchRunActivity";
        return mode_name;
    }

    void SearchRunActivity::onStart(){
/*
    	Intent intent = Intent();
        intent.uint8_t_param["SUB_MODE_NUM"] = 5;
        auto activity = ActivityFactory::createSubModeSelect();
        activity->start(intent);
        
        intent = activity->getIntent();
        PRINTF_ASYNC("SUB MODE SELECT RESULT = %d \n", intent.uint8_t_param["SUB_MODE"]);        
        uint8_t mode = intent.uint8_t_param["SUB_MODE"];
        if(mode == (uint8_t)ESearchMode::BACK_MODE_SELECT) return;
        
        hal::waitmsec(1000);
        module::ImuDriver::getInstance().calibrateGyro(1000);
        module::TrajectoryCommander::getInstance().reset(0.045f, 0.045f, 90.0f * DEG2RAD);
        module::PositionEstimator::getInstance().reset(0.045f, 0.045f, 90.0f * DEG2RAD);
*/
    }
    
    
    void SearchRunActivity::onFinish(){

    }


    SearchRunActivity::ELoopStatus SearchRunActivity::loop() {

        return ELoopStatus::CONTINUE;
    }
/*
    void SearchRunActivity::_slalom90(int8_t rot_times){
        ETurnDir turn_dir;
        if(rot_times > 0) turn_dir = ETurn_dir::CCW;
        else turn_dir = ETurn_dir::CW;
        CurveFactory::pushWithStraight(ETurnParamSet::SEARCH, ETurnType::TURN_90, turn_dir);
    }

    void SearchRunActivity::_spin90(int8_t rot_times){
        float target_ang = 45.0f * (float)rot_times * DEG2RAD;
        StraightFactory::push(ETurnType::STRAIGHT_CENTER, 0.045, _v, _v, 0.0f, _a, _a);
        StopFactory::push(0.1f);
        SpinTurnFactory::push(target_ang, yawrate_max, yawacc);
        StopFactory::push(0.1f);
        StraightFactory::push(ETurnType::STRAIGHT_CENTER, 0.045, 0.0f, _v, _v, _a, _a);
    }


    void SearchRunActivity::_straight(uint8_t x_next, uint8_t y_next){
        UMouse& m = UMouse::getInstance();
        WallSensor& ws = WallSensor::getInstance();
        ParameterManager& pm = ParameterManager::getInstance();
        uint8_t block_count = 1;

        while(1) {
            if (m.direction == direction_e::E) x_next++;
            else if (m.direction == direction_e::N) y_next++;
            else if (m.direction == direction_e::W) x_next--;
            else if (m.direction == direction_e::S) y_next--;
            if(!m.maze.isReached(x_next, y_next) ||
                (x_next == m.goal.x && y_next == m.goal.y) ||
                (x_next == 0 && y_next == 0)){
                break;
            }
            direction_e dest_dir_next = m.maze.getSearchDirection2(x_next, y_next, m.direction);
            int8_t rot_times = m.maze.calcRotTimes(dest_dir_next, m.direction);
            if(rot_times == 0) {
                block_count ++;
            }else {
                break;
            }
        }
        if(block_count == 1){
            auto traj0 = StraightTrajectory::createAsWallCenter(0.09f * block_count, v);
            m.trajCommander.push(std::move(traj0));
        }
        else{
            auto traj0 = StraightTrajectory::createAsWallCenter(0.09f * block_count - 0.045, v, v_max, v, a, a);
            m.trajCommander.push(std::move(traj0));
            while(!m.trajCommander.empty()) {waitmsec(1);}
            auto traj1 = StraightTrajectory::createAsWallCenter(0.045f, v);
            m.trajCommander.push(std::move(traj1));
        }
    }


    void SearchRunActivity::_spin180(){        
        StraightFactory::push(ETurnType::STRAIGHT_CENTER, 0.02f, _v, _v, 0.0f, _a, _a);
        StopFactory::push(0.1f);
        SpinTurnFactory::push(180.0f * DEG2RAD, _yawrate_max, _yawacc);
        StopFactory::push(0.1f);
        StraightFactory::push(ETurnType::STRAIGHT_CENTER, 0.02f, 0.0f, _v, _v, _a, _a);        
    }

    void _destiMove(){

    }
*/

}

