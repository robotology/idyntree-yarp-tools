#ifndef IDYNTREE_YARP_UTILITIES_H
#define IDYNTREE_YARP_UTILITIES_H

#include <atomic>

namespace idyntree_yarp_tools {

std::atomic<bool> isClosing{false};

void handleSigInt();

}

#endif // IDYNTREE_YARP_UTILITIES_H
