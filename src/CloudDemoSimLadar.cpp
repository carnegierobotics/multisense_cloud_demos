/**
 * @file CloudDemoSimLadar.cpp
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

    class SimLadarDemo : public CloudDemo {
    public:

        SimLadarDemo(BaseMultiSenseWrapper *MultiSenseArg);
        virtual ~SimLadarDemo();
        void DrawDemo(cv::Mat *Cloud, cv::Mat *Color);

    private:

        void DrawVoxelsWithLaser(const VoxelArray& voxelArray);

        VoxelArray m_voxelArray;

        // Height of laser plane relative to sensor in meters.
        float m_laserHeight;

        // Thickness of laser plane in meters.
        float m_laserThick;

        // How far to advance the laser at each interval.
        float m_laserStep;
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

        SimLadarDemo demo(multiSenseWrapperP);
        demo.p_self = &demo;

        // Adjust some settings for this specific demo
        demo.SetFieldOfView(55.0);

        // Initialize the MultiSense sensor and 3D display
        demo.SetupMultisense();
        demo.SetupUserInterface(argc, argvPP);

        // Set a function to call periodically where all of our processing happens
        glutIdleFunc(demo.IdleFuncCallback);

        // Enter the infinite event-processing loop
        glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION);
        glutMainLoop();

    } catch(...) {

        // No harm done if multisenseWrapperP ==0.
        delete multiSenseWrapperP;
    }

    return 0;
}

// Definitions of locally scoped symbols.
namespace {

    // Constructor.
    SimLadarDemo::SimLadarDemo(BaseMultiSenseWrapper *MultiSenseArg)
        : CloudDemo(MultiSenseArg),
          // Arguments are XSize, YSize, ZSize, VoxelDimension.
          m_voxelArray(250, 250, 350, 0.025),
          m_laserHeight(-1.0),
          m_laserThick(0.03),
          m_laserStep(0.08)
    {
        // Empty.
    }


    SimLadarDemo::~SimLadarDemo()
    {
        // Empty.
    }


    // Simple demo -- simulate a planer laser scanner.
    void SimLadarDemo::DrawDemo(cv::Mat *Cloud, cv::Mat *Color)
    {
        // Make sure there's valid data to process.
        if(0 == Cloud || 0 == Color) {
            return;
        }

        this->PopulateVoxels(*Cloud, *Color, this->m_voxelArray);

        // Draw mesh to illustrate position of laser scan
        DrawMeshWall(0.04, // Spacing (meters)
                     // XMin, XMax, YMin, YMax, ZMin, ZMax
                     -5.0, 5.0, this->m_laserHeight, this->m_laserHeight, -8.0, -2.0,
                     0.0, 0.0, 0.6, // RGB color
                     0.5);          // Point size

        // Draw mesh to illustrate position of laser scan
        DrawMeshWall(0.04, // Spacing (meters)
                     // XMin, XMax, YMin, YMax, ZMin, ZMax
                     (float) (2.5*this->m_laserHeight), (float) (2.5*this->m_laserHeight), -5.0, 5.0, -8.0, -2.0,
                     0.0, 0.0, 0.6,  // RGB color
                     0.5);           // Point size


        this->DrawVoxelsWithLaser(this->m_voxelArray);

        // Update the laser position.
        this->m_laserHeight += this->m_laserStep;
        if ( (this->m_laserHeight > 1.0) || (this->m_laserHeight < -1.0) ){
            this->m_laserStep = -this->m_laserStep;
        }

        // Draw info on the screen
        Draw2DScreenText(20, 20, 2.0, 0.0, 0.7, 0.0, (const unsigned char *) "CARNEGIE ROBOTICS LLC");
        Draw2DScreenText(20, -50, 2.0, 0.0, 0.7, 0.0, (const unsigned char *) "MultiSense Simulated Planar LADAR");
        return;
    }


    void SimLadarDemo::DrawVoxelsWithLaser(const VoxelArray& VoxelArrayArg)
    {
        // Display the voxels
        uint32_t VIndex = VoxelArrayArg.FirstVoxel;
        while (VIndex != 0) {

            // Unpack the flat index into X, Y and Z indices into the
            // 3D voxel array.
            int32_t VX;
            int32_t VY;
            int32_t VZ;
            VoxelArrayArg.IndexToXyz(VIndex, VX, VY, VZ);

            // Convert to X right, Y up, Z out-of-screen coordinate sytem.
            float XX;
            float YY;
            float ZZ;
            this->ConvertCloudCoordsToDisplay(
                VX, VY, VZ, VoxelArrayArg, XX, YY, ZZ);

            uint8_t iR = VoxelArrayArg.Voxels[VIndex].R;
            uint8_t iG = VoxelArrayArg.Voxels[VIndex].G;
            uint8_t iB = VoxelArrayArg.Voxels[VIndex].B;

            float BB = static_cast<float>((iB / 255.0) * CloudGain);
            float GG = static_cast<float>((iG / 255.0) * CloudGain);
            float RR = static_cast<float>((iR / 255.0) * CloudGain);

            // Check if this voxel is part of our "laser scan"
            if ((YY > (this->m_laserHeight))
                && (YY < (this->m_laserHeight + this->m_laserThick))) {
                RR = 0.0;
                GG = 0.8;
                BB = 0.0;
                ZZ += 0.05; // Pull the point forward a little
            }
            else if ((XX > 2.5 * (this->m_laserHeight))
                     && (XX < (2.5 * this->m_laserHeight) + this->m_laserThick)) {
                RR = 0.7;
                GG = 0.0;
                BB = 0.0;
                ZZ += 0.05; // Pull the point forward a little
            }

            // Apply bounding regions
            if ((ZZ < -0.05) && (YY < YClipping)){
                DrawCube(XX, YY, ZZ, VoxelArrayArg.VoxelDimension, RR, GG, BB);
            }

            // Follow to the next element of the linked list.
            VIndex = VoxelArrayArg.Voxels[VIndex].Next;
        }
    }

} // namespace
