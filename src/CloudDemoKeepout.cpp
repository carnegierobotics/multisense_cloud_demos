/**
 * @file CloutDemoKeepout.cpp
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

    class KeepOutDemo : public CloudDemo {
    public:
        // Keepout is a rectangular volume with WARNING and DANGER zones
        float WarnXMin, WarnXMax, WarnYMin, WarnYMax, WarnZMin, WarnZMax;
        float DangerXMin, DangerXMax, DangerYMin, DangerYMax, DangerZMin, DangerZMax;

        // Constructor.
        KeepOutDemo(BaseMultiSenseWrapper *MultiSenseArg);

        // Destructor.
        ~KeepOutDemo();

        // Demo-specific code.
        void DrawDemo(cv::Mat *Cloud, cv::Mat *Color);

    private:
        float Pulse, PulseStep;

    };

} // namespace



int main(int argc, char *argvPP[])
{
    // Initialize the demo object and set the self pointer
    // to itself so that the static callbacks may call
    // the non-static methods which actually run the demo
    BaseMultiSenseWrapper *multiSenseWrapperP;

    // Read our command line arguments
    switch (argc) {
        case 1:
        case 4:
            // No arguments -- just open live display
            // or Three arguments -- live display with user-defined zone borders
            multiSenseWrapperP = new MultiSenseWrapper();
            break;
        default:
            fprintf(stderr, "Optional arguments are three distances to border of each zone.\n\n");
            fprintf(stderr, "   %s [<near_danger> <far_danger> <far_warning>]\n", argvPP[0]);
            exit(0);
    }


    // This try block is to prevent a memory leak if one of the
    // enclosed function calls throws an exception.
    try {

        KeepOutDemo demo(multiSenseWrapperP);
        demo.p_self = &demo;


        // Read arguments which set ranges for keepout zones
        if (argc > 3) {
            demo.DangerZMax = -fabsf(atof(argvPP[1]));                // Ranges are always negative
            demo.DangerZMin = demo.WarnZMax = -fabsf(atof(argvPP[2]));
            demo.WarnZMin = -fabsf(atof(argvPP[3]));
        }


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

    // Constructor to initialize member variables
    KeepOutDemo::KeepOutDemo(BaseMultiSenseWrapper *MultiSenseArg)
        : CloudDemo(MultiSenseArg),

        // Setup extents of 3D warning and danger zones
        // Values are in meters.  Z is more negative farther from the sensor.
        WarnXMin(-1.0),
        WarnXMax(1.0),
        WarnYMin(-0.50),
        WarnYMax(0.50),
        WarnZMin(-2.75),
        WarnZMax(-1.5),
        DangerXMin(-1.0),
        DangerXMax(1.0),
        DangerYMin(-0.50),
        DangerYMax(0.50),
        DangerZMin(-1.5),
        DangerZMax(-0.5),

        Pulse(1.0),
        PulseStep(0.05)
    {
        // Empty.
    }


    KeepOutDemo::~KeepOutDemo()
    {
        // Empty.
    }


    void KeepOutDemo::DrawDemo(cv::Mat *Cloud, cv::Mat *Color)
    {
        // Declare and initialize variables needed for this demo

        cv::Vec3b ColorPt;
        cv::Vec3f CloudPt;
        float X, Y, Z;
        float B, G, R;
        int Warning=0, Danger=0;

        // Make sure there's valid data to process.
        if(0 == Cloud || 0 == Color) {
            return;
        }

        // Now back to processing points
        glPointSize(CloudPtSize);
        glBegin(GL_POINTS);

        // Draw the point cloud
        // Ignore the first few rows with green noise
        for (int r = 20 ; r < ImgRows ; r++) {
            for (int c = 0 ; c < ImgCols ; c++) {
                // Figure out proper RGB color with gains
                ColorPt = Color->at<cv::Vec3b>(r, c);
                B = (float) (ColorPt[0]) / 255.0;
                G = (float) (ColorPt[1]) / 255.0;
                R = (float) (ColorPt[2]) / 255.0;
                glColor3f(CloudGain*R, CloudGain*G, CloudGain*B);

                // Figure out true world location (OpenGL coordinates)
                CloudPt = Cloud->at<cv::Vec3f>(r, c);
                X = (float) (CloudPt[0]);                   // Left-Right in camera frame
                Y = (float) (-CloudPt[1]);                  // Up-down in camera frame
                Z = (float) (-CloudPt[2]);                  // More negative is farther from the camera

                // Filter out very close points which are often noise
                if (Z > -0.05) {
                    continue;
                }

                // Check if this point should have warning or danger colors
                if (((Z > WarnZMin) && (Z < WarnZMax)) &&
                    ((Y > WarnYMin) && (Y < WarnYMax)) &&
                    ((X > WarnXMin) && (X < WarnXMax)) ) {
                    // Warning (change point to yellow)
                    B = 0.0; R = 1.0; G = 1.0;
                    glColor3f(R, G, B);
                    Warning++;
                }

                if (((Z > DangerZMin) && (Z < DangerZMax)) &&
                    ((Y > DangerYMin) && (Y < DangerYMax)) &&
                    ((X > DangerXMin) && (X < DangerXMax)) ) {
                    // Danger (change point to red)
                    G = 0.0; B = 0.0; R = 1.0;
                    glColor3f(R, G, B);
                    Danger++;
                }

                glVertex3f(X, Y, Z);
            }
        }

        glEnd();

        // The pulsing of the warning and danger zones
        Pulse = Pulse + PulseStep;
        if (Pulse > 1.0) {
            Pulse = 1.0;
            PulseStep = -PulseStep;
        }
        if (Pulse < 0.3) {
            Pulse = 0.3;
            PulseStep = -PulseStep;
        }

        // Set colors for warning zone.
        R = Pulse *0.8;
        G = Pulse *0.8;
        B = Pulse *0.0;

        // Left, right, top, and bottom walls, respectively
        DrawMeshWall(0.02, WarnXMin, WarnXMin, WarnYMin, WarnYMax, WarnZMin, WarnZMax, R, G, B, 0.15);
        DrawMeshWall(0.02, WarnXMax, WarnXMax, WarnYMin, WarnYMax, WarnZMin, WarnZMax, R, G, B, 0.15);
        DrawMeshWall(0.02, WarnXMin, WarnXMax, WarnYMax, WarnYMax, WarnZMin, WarnZMax, R, G, B, 0.15);
        DrawMeshWall(0.02, WarnXMin, WarnXMax, WarnYMin, WarnYMin, WarnZMin, WarnZMax, R, G, B, 0.15);

        // Draw start of warning zone
        glBegin(GL_LINE_LOOP);
        glVertex3f(WarnXMin, WarnYMin, WarnZMin);
        glVertex3f(WarnXMax, WarnYMin, WarnZMin);
        glVertex3f(WarnXMax, WarnYMax, WarnZMin);
        glVertex3f(WarnXMin, WarnYMax, WarnZMin);
        glEnd();

        // Set colors for danger zone.
        R = Pulse *1.0;
        G = Pulse *0.0;
        B = Pulse *0.0;

        // Left, right, top, and bottom walls, respectively
        DrawMeshWall(0.02, DangerXMin, DangerXMin, DangerYMin, DangerYMax, DangerZMin, DangerZMax, R, G, B, 0.05);
        DrawMeshWall(0.02, DangerXMax, DangerXMax, DangerYMin, DangerYMax, DangerZMin, DangerZMax, R, G, B, 0.05);
        DrawMeshWall(0.02, DangerXMin, DangerXMax, DangerYMax, DangerYMax, DangerZMin, DangerZMax, R, G, B, 0.05);
        DrawMeshWall(0.02, DangerXMin, DangerXMax, DangerYMin, DangerYMin, DangerZMin, DangerZMax, R, G, B, 0.05);

        // Draw near end of danger zone
        glBegin(GL_LINE_LOOP);
        glVertex3f(DangerXMin, DangerYMin, DangerZMin);
        glVertex3f(DangerXMax, DangerYMin, DangerZMin);
        glVertex3f(DangerXMax, DangerYMax, DangerZMin);
        glVertex3f(DangerXMin, DangerYMax, DangerZMin);
        glEnd();

        // Draw far end of danger zone
        glBegin(GL_LINE_LOOP);
        glVertex3f(DangerXMin, DangerYMin, DangerZMax);
        glVertex3f(DangerXMax, DangerYMin, DangerZMax);
        glVertex3f(DangerXMax, DangerYMax, DangerZMax);
        glVertex3f(DangerXMin, DangerYMax, DangerZMax);
        glEnd();

        // Post warning/danger text
        if (Danger>30) {
            Draw2DScreenText((int) 20, 70, 3.5, 0.9, 0.0, 0.0,
                             (const unsigned char *) "3D DANGER ZONE VIOLATED!");
        }
        else if (Warning>30) {
            Draw2DScreenText((int) 20, 70, 3.5, 0.8, 0.8, 0.0,
                             (const unsigned char *) "3D WARNING ZONE VIOLATED!");
        }

        // Draw info on the screen
        Draw2DScreenText(20, 20, 2.0, 0.0, 0.7, 0.0,
                         (const unsigned char *) "CARNEGIE ROBOTICS LLC");
        Draw2DScreenText(20, -50, 2.0, 0.0, 0.7, 0.0,
                         (const unsigned char *) "3D Safety Zone Demonstration");

        return;
    }

} // namespace
