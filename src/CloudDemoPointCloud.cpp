/**
 * @file CloutDemoPointCloud.cpp
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

#include <stdio.h>
#include <fstream>
#include <iostream>
#include <cstring>

#include <opencv2/opencv.hpp>

#include <GL/freeglut.h>

#include <CloudDemo.h>

using namespace std;


// Anonymous namespace for locally scoped symbols.
namespace {

    class PointCloudDemo : public CloudDemo {
    public:
        PointCloudDemo(BaseMultiSenseWrapper *MultiSenseArg);
        void DrawDemo(cv::Mat *Cloud, cv::Mat *Color);
    };

} // namespace


// Start for this demo
int main(int argc, char *argvPP[])
{
    // Initialize the demo object and set the self pointer
    // to itself so that the static callbacks may call
    // the non-static methods which actually run the demo
    BaseMultiSenseWrapper *multiSenseWrapperP = new MultiSenseWrapper();



    // This try block is to prevent a memory leak if one of the
    // enclosed function calls throws an exception.
    try {

        PointCloudDemo demo(multiSenseWrapperP);
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

    PointCloudDemo::PointCloudDemo(BaseMultiSenseWrapper *MultiSenseArg)
        : CloudDemo(MultiSenseArg)
    {
        // Empty.
    }


    // Simple demo -- just display point cloud
    void PointCloudDemo::DrawDemo(cv::Mat *Cloud, cv::Mat *Color)
    {
        cv::Vec3b ColorPt;
        cv::Vec3f CloudPt;
        float X, Y, Z;
        float B, G, R;

        // Make sure there's valid data to process.
        if(0 == Cloud || 0 == Color) {
            return;
        }

        // Set to process points
        glPointSize(CloudPtSize);
        glBegin(GL_POINTS);

        // Draw the point cloud
        // Ignore the first few rows with green noise
        for (int r = 20 ; r < ImgRows ; r++) {
            for (int c = 0 ; c < ImgCols ; c++) {

                // Figure out true world location (OpenGL coordinates)
                CloudPt = Cloud->at<cv::Vec3f>(r, c);
                X = (float) (CloudPt[0]);           // Left-Right in camera frame
                Y = (float) (-CloudPt[1]);          // Up-down in camera frame
                Z = (float) (-CloudPt[2]);          // More negative is farther from the camera

                // Figure out proper RGB color with gains
                ColorPt = Color->at<cv::Vec3b>(r, c);

                // Display RGB camera color for each 3D point
                B = (float) (ColorPt[0]) / 255.0;
                G = (float) (ColorPt[1]) / 255.0;
                R = (float) (ColorPt[2]) / 255.0;

                glColor3f(CloudGain*R, CloudGain*G, CloudGain*B);

                // Filter out very close points which are often noise
                if (Z > -0.05)
                    continue;

                // Don't bother to draw perfectly black points (probably filtered noise)
                if ( (R == 0.0) && (G == 0.0) && (B == 0.0) )
                    continue;

                // Display the 3D point on the screen
                glVertex3f(X, Y, Z);
            }
        }

        glEnd();

        // Draw info on the screen
        Draw2DScreenText(20, 20, 2.0, 0.0, 0.7, 0.0, (const unsigned char *) "CARNEGIE ROBOTICS LLC");
        Draw2DScreenText(20, -50, 2.0, 0.0, 0.7, 0.0, (const unsigned char *) "MultiSense Point Cloud Display");
        return;
    }

} // namespace
