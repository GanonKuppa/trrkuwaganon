#include "activityFactory.h"

#include "modeSelectActivity.h"

#include "shortestRunActivity.h"
#include "debugActivity.h"
#include "deleteMazeActivity.h"
#include "searchRunActivity.h"
#include "calibrateFeedForwardActivity.h"
#include "fullAutoRunActivity.h"
#include "radioControlActivity.h"
#include "subModeSelectActivity.h"

namespace activity {

    std::unique_ptr<BaseActivity> ActivityFactory::create(EActivityColor name) {
        switch(name) {
            case EActivityColor::BLACK:
                return std::make_unique<ModeSelectActivity>();
            case EActivityColor::RED:
                return std::make_unique<ShortestRunActivity>();
            case EActivityColor::GREEN:
                return std::make_unique<DebugActivity>();
            case EActivityColor::YELLOW:
                return std::make_unique<DeleteMazeActivity>();
            case EActivityColor::BLUE:
                return std::make_unique<SearchRunActivity>();
            case EActivityColor::MAGENTA:
                return std::make_unique<CalibrateFeedForwardActivity>();
            case EActivityColor::CYAN:
                return std::make_unique<FullAutoRunActivity>();
            case EActivityColor::WHITE:
                return std::make_unique<RadioControlActivity>();
            default:
                return std::make_unique<ModeSelectActivity>();
        }
    }
    
    std::unique_ptr<BaseActivity> ActivityFactory::createSubModeSelect() {
        return std::make_unique<SubModeSelectActivity>();
    }
    

    std::unique_ptr<BaseActivity> ActivityFactory::createModeSelect() {
        return std::make_unique<ModeSelectActivity>();
    }


}
