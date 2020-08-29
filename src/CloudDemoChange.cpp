/**
 * @file CloutDemoChange.cpp
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

    // This class implements the CloudDemoChange demo
    class VoxelChangeDemo : public CloudDemo {

    public:
        VoxelChangeDemo(BaseMultiSenseWrapper *MultiSenseArg);
        virtual ~VoxelChangeDemo();
        void DrawDemo(cv::Mat *Cloud, cv::Mat *Color);
        void DiscardOrphanedSegments(VoxelArray& voxelArray);

    private:
        VoxelArray m_voxelArray;
        int32_t m_frames;
        int32_t m_trainingFrames;
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

        VoxelChangeDemo demo(multiSenseWrapperP);
        demo.p_self = &demo;

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
    VoxelChangeDemo::VoxelChangeDemo(BaseMultiSenseWrapper *MultiSenseArg)
        : CloudDemo(MultiSenseArg),
          // Arguments are XSize, YSize, ZSize, VoxelDimension.
          m_voxelArray(300, 300, 350, 0.035),
          m_frames(0),
          m_trainingFrames(50)
    {
        // Empty.
    }


    VoxelChangeDemo::~VoxelChangeDemo()
    {
        // Empty.
    }


    // Simple demo -- detect changes in original scene
    void VoxelChangeDemo::DrawDemo(cv::Mat *Cloud, cv::Mat *Color)
    {
        // Make sure there's valid data to process.
        if(0 == Cloud || 0 == Color) {
            return;
        }

        if(this->m_frames < this->m_trainingFrames) {

            // Set doErase argument to false, so that voxels accumulate.
            this->PopulateVoxels(*Cloud, *Color, this->m_voxelArray, 1, false);

            // Display voxels.
            this->DrawVoxels(this->m_voxelArray);
            Draw2DScreenText((int)(WinWidth/2.0)-240, 150, 2.5, 0.0, 0.0, 0.7,
                             (const unsigned char *) "Learning 3D scene...");

        } else {

            // From this frame on, we'll erase voxels at each frame.
            // Just once, we manually tweak the FirstVoxel index so
            // that our accumulated points aren't erased.
            if(this->m_frames == this->m_trainingFrames) {
                this->m_voxelArray.FirstVoxel = 0;
                this->m_voxelArray.LastVoxel = 0;
            }

            // Populate just the voxels from this frame that haven't
            // been previously accumulated during training.  This
            // time, we'll flag the voxels with value 2, to
            // distinguish them from the accumulated voxels.
            this->PopulateVoxels(*Cloud, *Color, this->m_voxelArray, 2);

            // Remove small groups of voxels that are most likely noise.
            this->DiscardOrphanedSegments(this->m_voxelArray);

            // And display.
            this->DrawVoxels(this->m_voxelArray);
            Draw2DScreenText(
              (int)(WinWidth/2.0)-320, 150, 2.5, 0.0, 0.0, 0.7,
              (const unsigned char *) "Displaying 3D scene changes");
        }

        // Draw info on the screen
        Draw2DScreenText(
          20, 20, 2.0, 0.0, 0.7, 0.0,
          (const unsigned char *) "CARNEGIE ROBOTICS LLC");
        Draw2DScreenText(
          20, -50, 2.0, 0.0, 0.7, 0.0,
          (const unsigned char *) "3D Scene Change Detection Demonstration");

        (this->m_frames)++;
    }


    void VoxelChangeDemo::DiscardOrphanedSegments(VoxelArray& voxelArray)
    {
        const int32_t XVoxels = voxelArray.XSize;
        const int32_t YVoxels = voxelArray.YSize;
        const int32_t ZVoxels = voxelArray.ZSize;
        const int32_t XYVoxels = XVoxels * YVoxels;
        const int32_t XYZVoxels = XYVoxels * ZVoxels;

        int32_t VIndex = voxelArray.FirstVoxel;
        uint32_t* PreviousNextP = &(voxelArray.FirstVoxel);
        while(VIndex != 0) {
            // Simplified segmentation - count adjacent pixels
            int32_t Count=0;

            int32_t VXStart = (VIndex % XVoxels) - 1;
            int32_t VXEnd = VXStart + 2;

            int32_t VYStart = ((VIndex / XVoxels) % YVoxels) - 1;
            int32_t VYEnd = VYStart + 2;

            int32_t VZStart = (VIndex / XYVoxels) - 1;
            int32_t VZEnd = VZStart + 2;

            for (int32_t VX = VXStart; VX <= (int) VXEnd ; VX++) {
                for (int32_t VY = VYStart ; VY <= (int) VYEnd ; VY++) {
                    for (int32_t VZ = VZStart ; VZ <= (int) VZEnd ; VZ++) {
                        int32_t ii = (VX + (VY * XVoxels) + (VZ * XYVoxels));
                        if ((ii > 0) && (ii < XYZVoxels)) {
                            if ((voxelArray.Voxels[ii].Flag) == 2) {
                                ++Count;
                            }
                        }
                    }
                }
            }

            if (Count < 6) {
                // This is an orphan segment -- change the flag and
                // remove it from the list.
                voxelArray.Voxels[VIndex].Flag = 0;
                *PreviousNextP = voxelArray.Voxels[VIndex].Next;
            } else {
                PreviousNextP = &(voxelArray.Voxels[VIndex].Next);
            }

            // Advance to the next voxel.
            VIndex = *PreviousNextP;
        }
    }

} // namespace
