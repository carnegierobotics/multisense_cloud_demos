/**
 * @file CloutDemoVoxels.cpp
 *
 * Simple MultiSense demo.
 * Meant to be quick and easy to modify demo code.
 * This is not production code -- do not rely on for real applications.
 *
 * Copyright 2014
 * Carnegie Robotics, LLC
 * 4501 Hatfield Street, Pittsburgh, PA 15201
 * http://www.carnegierobotics.com
 *
 * Significant history (date, user, job code, action):
 *   2014-05-21, dlr@carnegierobotics.com, PR1044, Created file from
 *               Bill Ross's code.
 *   2014-07-31, ewestman@carnegierobotics.com, PR1044, cleaned up and
 *               optimized code for improved performance.
 **/

#include <cstring>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <unistd.h>

#include <opencv2/opencv.hpp>

#include <GL/freeglut.h>

#include <CloudDemo.h>

using namespace std;


// Anonymous namespace for locally scoped symbols.
namespace {

    class VoxelDisplayDemo : public CloudDemo {

    public:
        VoxelDisplayDemo(BaseMultiSenseWrapper *MultiSenseArg);
        virtual  ~VoxelDisplayDemo();
        void DrawDemo(cv::Mat *Cloud, cv::Mat *Color);

    private:
        VoxelArray m_voxelArray;
    };

} // namespace

void usage(char** argv)
{
    std::cerr << "Usage: " << argv[0] << " [-i <ip-address>]" << std::endl;
    std::cerr << "\t-i default: 10.66.171.21" << std::endl;
}

// Start for this demo
int main(int argc, char *argvPP[])
{
    std::string ipAddress = "10.66.171.21";

    int result = -1;
    while ((result = getopt(argc, argvPP, "i:h")) != -1)
    {
        switch (result)
        {
            case 'i':
                ipAddress = optarg;
                break;
            case 'h':
                usage(argvPP);
                return 0;
        }
    }

    // Initialize the demo object and set the self pointer
    // to itself so that the static callbacks may call
    // the non-static methods which actually run the demo
    BaseMultiSenseWrapper *multiSenseWrapperP = new MultiSenseWrapper(ipAddress);


    // This try block is to prevent a memory leak if one of the
    // enclosed function calls throws an exception.
    try {

        VoxelDisplayDemo demo(multiSenseWrapperP);
        demo.p_self = &demo;

        // Initialize the MultiSense sensor and 3D display
        demo.SetupMultisense();
        demo.SetupUserInterface(argc, argvPP);

        // Set a function to call periodically where all of our processing happens
        glutIdleFunc(demo.IdleFuncCallback);

        // Enter the infinite event-processing loop
        glutMainLoop();

    } catch(...) {

        // No harm done if multisenseWrapperP ==0.
        delete multiSenseWrapperP;

    }

    return 0;
}

// Definitions of locally scoped symbols.
namespace {

    VoxelDisplayDemo::VoxelDisplayDemo(BaseMultiSenseWrapper *MultiSenseArg)
        : CloudDemo(MultiSenseArg),
          // Arguments are XSize, YSize, ZSize, VoxelDimension.
          m_voxelArray(370, 230, 650, 0.035)
    {
        // Empty.
    }


    VoxelDisplayDemo::~VoxelDisplayDemo()
    {
        // Empty.
    }


    // Simple demo -- fill 3D grid with points
    void VoxelDisplayDemo::DrawDemo(cv::Mat *Cloud, cv::Mat *Color)
    {
        // Make sure there's valid data to process.
        if(0 == Cloud || 0 == Color) {
            return;
        }

        this->PopulateVoxels(*Cloud, *Color, this->m_voxelArray);
        this->DrawVoxels(this->m_voxelArray);

        // Draw info on the screen
        Draw2DScreenText(20, 20, 2.0, 0.0, 0.7, 0.0,
                         (const unsigned char *) "CARNEGIE ROBOTICS LLC");
        Draw2DScreenText(20, -50, 2.0, 0.0, 0.7, 0.0,
                         (const unsigned char *) "MultiSense 3D Voxel Display Demo");

        return;
    }

} // namespace
