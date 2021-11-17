#include "trajectoryFactory.h"
#include "trajectoryCommander.h"
#include "trajectoryInitializer.h"

// StraightFactory
std::unique_ptr<BaseTrajectory> StraightFactory::create(ETurnType turn_type, float target_dist, float v_0){
    std::unique_ptr<BaseTrajectory> traj = std::make_unique<StraightTrajectory>(turn_type, target_dist, v_0);
    return traj;
}

std::unique_ptr<BaseTrajectory> StraightFactory::create(ETurnType turn_type, float target_dist, float v_0, float v_max, float v_end, float a_acc, float a_dec){
    std::unique_ptr<BaseTrajectory> traj = std::make_unique<StraightTrajectory>(turn_type, target_dist, v_0, v_max, v_end, a_acc, a_dec);
    return traj;
}

void StraightFactory::push(ETurnType turn_type, float target_dist, float v_0){
	auto traj = create(turn_type, target_dist, v_0);
	module::TrajectoryCommander::getInstance().push(std::move(traj));
}

void StraightFactory::push(ETurnType turn_type, float target_dist, float v_0, float v_max, float v_end, float a_acc, float a_dec){
	auto traj = create(turn_type, target_dist, v_0, v_max, v_end, a_acc, a_dec);
	module::TrajectoryCommander::getInstance().push(std::move(traj));
}

// SpinTurnFactory
std::unique_ptr<BaseTrajectory> SpinTurnFactory::create(float target_cumulative_yaw, float abs_yawrate_max, float abs_yawacc){
    std::unique_ptr<BaseTrajectory> traj = std::make_unique<SpinTurnTrajectory>(target_cumulative_yaw, abs_yawrate_max, abs_yawacc);
    return traj;
}

void SpinTurnFactory::push(float target_cumulative_yaw, float abs_yawrate_max, float abs_yawacc){
    auto traj = create(target_cumulative_yaw, abs_yawrate_max, abs_yawacc);
    module::TrajectoryCommander::getInstance().push(std::move(traj));
}

// StopFactory
std::unique_ptr<BaseTrajectory> StopFactory::create(float stop_time){
    std::unique_ptr<BaseTrajectory> traj = std::make_unique<StopTrajectory>(stop_time);
    return traj;
}

void StopFactory::push(float stop_time){
    auto traj = create(stop_time);
    module::TrajectoryCommander::getInstance().push(std::move(traj));
}

// CurveFactory
std::unique_ptr<BaseTrajectory> CurveFactory::create(ETurnParamSet param_set , ETurnType turn_type, ETurnDir turn_dir){
    std::unique_ptr<BaseTrajectory> traj = std::make_unique<CurveTrajectory>(param_set, turn_type, turn_dir);
    return traj;
}

void CurveFactory::push(ETurnParamSet param_set , ETurnType turn_type, ETurnDir turn_dir){
    auto traj = create(param_set , turn_type, turn_dir);
    module::TrajectoryCommander::getInstance().push(std::move(traj));
}

void CurveFactory::pushWithStraight(ETurnParamSet param_set , ETurnType turn_type, ETurnDir turn_dir){
    float v = module::TrajectoryInitializer::getInstance().getV(param_set, turn_type);
    float pre_dist = module::TrajectoryInitializer::getInstance().getPreDist(param_set, turn_type);
    float fol_dist = module::TrajectoryInitializer::getInstance().getFolDist(param_set, turn_type);
    ETurnType turn_type_pre;
    ETurnType turn_type_fol;

    switch (turn_type) {
		case ETurnType::TURN_90:
			turn_type_pre = ETurnType::STRAIGHT;
			turn_type_fol = ETurnType::STRAIGHT;
			break;
		case ETurnType::TURN_L_90:
            turn_type_pre = ETurnType::STRAIGHT;
            turn_type_fol = ETurnType::STRAIGHT;
            break;
        case ETurnType::TURN_180:
            turn_type_pre = ETurnType::STRAIGHT;
            turn_type_fol = ETurnType::STRAIGHT;
            break;
        case ETurnType::TURN_S2D_45:
            turn_type_pre = ETurnType::STRAIGHT;
            turn_type_fol = ETurnType::DIAGONAL;
            break;
        case ETurnType::TURN_S2D_135:
            turn_type_pre = ETurnType::STRAIGHT;
            turn_type_fol = ETurnType::DIAGONAL;
            break;
        case ETurnType::TURN_D2S_45:
            turn_type_pre = ETurnType::DIAGONAL;
            turn_type_fol = ETurnType::STRAIGHT;
            break;
        case ETurnType::TURN_D2S_135:
            turn_type_pre = ETurnType::DIAGONAL;
            turn_type_fol = ETurnType::STRAIGHT;            
            break;
        case ETurnType::TURN_D_90:
            turn_type_pre = ETurnType::DIAGONAL;
            turn_type_fol = ETurnType::DIAGONAL;
            break;
        default:
            turn_type_pre = ETurnType::STRAIGHT;
            turn_type_fol = ETurnType::STRAIGHT;
    }

    auto traj1 = StraightFactory::create(turn_type_pre , pre_dist, v);
    auto traj2 = create(param_set , turn_type, turn_dir);
    auto traj3 = StraightFactory::create(turn_type_fol , fol_dist, v);
    module::TrajectoryCommander::getInstance().push(std::move(traj1));
    module::TrajectoryCommander::getInstance().push(std::move(traj2));
    module::TrajectoryCommander::getInstance().push(std::move(traj3));
}

void CurveFactory::pushWithStraight(ETurnParamSet param_set , ETurnType turn_type, ETurnDir turn_dir, float offset_pre, float offset_fol){
    float v = module::TrajectoryInitializer::getInstance().getV(param_set, turn_type);
    float pre_dist = module::TrajectoryInitializer::getInstance().getPreDist(param_set, turn_type) + offset_pre;
    float fol_dist = module::TrajectoryInitializer::getInstance().getFolDist(param_set, turn_type) + offset_fol;
    ETurnType turn_type_pre;
    ETurnType turn_type_fol;

    switch (turn_type) {
		case ETurnType::TURN_90:
			turn_type_pre = ETurnType::STRAIGHT;
			turn_type_fol = ETurnType::STRAIGHT;
			break;
		case ETurnType::TURN_L_90:
            turn_type_pre = ETurnType::STRAIGHT;
            turn_type_fol = ETurnType::STRAIGHT;
            break;
        case ETurnType::TURN_180:
            turn_type_pre = ETurnType::STRAIGHT;
            turn_type_fol = ETurnType::STRAIGHT;
            break;
        case ETurnType::TURN_S2D_45:
            turn_type_pre = ETurnType::STRAIGHT;
            turn_type_fol = ETurnType::DIAGONAL;
            break;
        case ETurnType::TURN_S2D_135:
            turn_type_pre = ETurnType::STRAIGHT;
            turn_type_fol = ETurnType::DIAGONAL;
            break;
        case ETurnType::TURN_D2S_45:
            turn_type_pre = ETurnType::DIAGONAL;
            turn_type_fol = ETurnType::STRAIGHT;
            break;
        case ETurnType::TURN_D2S_135:
            turn_type_pre = ETurnType::DIAGONAL;
            turn_type_fol = ETurnType::STRAIGHT;            
            break;
        case ETurnType::TURN_D_90:
            turn_type_pre = ETurnType::DIAGONAL;
            turn_type_fol = ETurnType::DIAGONAL;
            break;
        default:
            turn_type_pre = ETurnType::STRAIGHT;
            turn_type_fol = ETurnType::STRAIGHT;
    }

    auto traj1 = StraightFactory::create(turn_type_pre , pre_dist, v);
    auto traj2 = CurveFactory::create(param_set , turn_type, turn_dir);
    auto traj3 = StraightFactory::create(turn_type_fol , fol_dist, v);
    module::TrajectoryCommander::getInstance().push(std::move(traj1));
    module::TrajectoryCommander::getInstance().push(std::move(traj2));
    module::TrajectoryCommander::getInstance().push(std::move(traj3));

};
