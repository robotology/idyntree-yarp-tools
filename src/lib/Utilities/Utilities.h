#ifndef IDYNTREE_YARP_UTILITIES_H
#define IDYNTREE_YARP_UTILITIES_H

#include <functional>

namespace idyntree_yarp_tools {

void handleSignals(std::function<void()> customHandler);

void handleSignals();


}

#endif // IDYNTREE_YARP_UTILITIES_H
