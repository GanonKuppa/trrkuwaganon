#pragma once

#include <functional>

namespace scheduler{

void pushTask(std::function<void(void)> f);
void doTask();

}