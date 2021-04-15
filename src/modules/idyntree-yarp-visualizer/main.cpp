/*
     * Copyright (C) 2020 Fondazione Istituto Italiano di Tecnologia
     *
     * Licensed under either the GNU Lesser General Public License v3.0 :
     * https://www.gnu.org/licenses/lgpl-3.0.html
     * or the GNU Lesser General Public License v2.1 :
     * https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html
     * at your option.
     */

#include "Visualizer.h"


int main(int argc, char * argv[])
{
    yarp::os::Network yarp; //to initialize the network

    yarp::os::ResourceFinder& rf = yarp::os::ResourceFinder::getResourceFinderSingleton();

    rf.configure(argc, argv);

    idyntree_yarp_tools::Visualizer viz;

    if (viz.neededHelp(rf))
    {
        return EXIT_SUCCESS;
    }

    if (!viz.configure(rf))
    {
        viz.close();
        return EXIT_FAILURE;
    }

    return viz.run();
}
