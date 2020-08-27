/**
 * Copyright 2014
 * Carnegie Robotics, LLC
 * Pittsburgh, PA 15201
 * http://www.carnegierobotics.com
 **/


#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <pthread.h>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <MultiSenseWrapper.h>

using namespace cv;

// Test grabbing from CRL Sensor Pod
int main(int argc, char *argv[])
{
    Mat CvImg;

    // Setup Comms with CRL stereo pod
    MultiSenseWrapper MultiSense;

    int c;
    float CurGain = 1.4;
    float CurFPS = 15.0;
    int CurMaxExp = 60000;
    uint32_t CurDisparities = 128;
    float CurExpThresh = 0.75;
    float CurPostFilt = 0.65;


    // cvInitSystem(0, NULL);
    //cvNamedWindow("Rectified", CV_WINDOW_AUTOSIZE);
    namedWindow("Disparity", WINDOW_AUTOSIZE);
    namedWindow("Left", WINDOW_AUTOSIZE);
    // namedWindow("Right", WINDOW_AUTOSIZE);
    namedWindow("Color", WINDOW_AUTOSIZE);
    namedWindow("Cloud", WINDOW_AUTOSIZE);


    // Ask camera to start sending images
    MultiSense.StartDisparityStream();
    MultiSense.StartLumaLeftStream();
    MultiSense.StartChromaLeftStream();

    MultiSense.SetGain(CurGain);
    MultiSense.SetFPS(CurFPS);
    MultiSense.SetMaxExp(CurMaxExp);
    MultiSense.SetDisparities(CurDisparities);
    MultiSense.SetExpThresh(CurExpThresh);
    MultiSense.SetPostFilt(CurPostFilt);


    printf("Type q in any display window to quit.\n");


    while (1) {

        Mat DisparityImg;
        CvImg = MultiSense.CopyCloud(DisparityImg);
        if (!CvImg.empty()) {
            imshow("Disparity", DisparityImg);
            imshow("Cloud", CvImg);
        }

        CvImg = MultiSense.CopyLeftLuma();
        if (!CvImg.empty()) {
            imshow("Left", CvImg);
        }

        CvImg = MultiSense.CopyLeftRectifiedRGB();
        if (!CvImg.empty()) {
            imshow("Color", CvImg);
        }


        c = waitKey(1);
        if ((char) c == 'q')
            break;

        if ((char) c == 'g') {
            CurGain -= 0.1;
            if (CurGain < 0.1)
                CurGain = 0.1;
            MultiSense.SetGain(CurGain);
            printf("New Gain = %0.2f\n", CurGain);
        }
        if ((char) c == 'G') {
            CurGain += 0.1;
            if (CurGain > 4.0)
                CurGain = 4.0;
            MultiSense.SetGain(CurGain);
            printf("New Gain = %0.2f\n", CurGain);
        }

        if ((char) c == 'f') {
            CurFPS -= 1.0;
            if (CurFPS < 1.0)
                CurFPS = 1.0;
            MultiSense.SetFPS(CurFPS);
            printf("New FPS = %0.1f\n", CurFPS);
        }
        if ((char) c == 'F') {
            CurFPS += 1.0;
            if (CurFPS > 30.0)
                CurFPS = 30.0;
            MultiSense.SetFPS(CurFPS);
            printf("New FPS = %0.1f\n", CurFPS);
        }

        if ((char) c == 'e') {
            CurMaxExp -= 5000;
            if (CurMaxExp < 10000)
                CurMaxExp = 10000;
            MultiSense.SetMaxExp(CurMaxExp);
            printf("New MaxExp = %d\n", CurMaxExp);
        }
        if ((char) c == 'E') {
            CurMaxExp += 5000;
            if (CurMaxExp > 500000)
                CurMaxExp = 500000;
            MultiSense.SetMaxExp(CurMaxExp);
            printf("New MaxExp = %d\n", CurMaxExp);
        }

        if ((char) c == 'd') {
            CurDisparities -= 128;
            if (CurDisparities < 128)
                CurDisparities = 128;
            MultiSense.SetDisparities(CurDisparities);
            printf("New Disparities = %d\n", CurDisparities);
        }
        if ((char) c == 'D') {
            CurDisparities += 128;
            if (CurDisparities > 256)
                CurDisparities = 256;
            MultiSense.SetDisparities(CurDisparities);
            printf("New Disparities = %d\n", CurDisparities);
        }

        if ((char) c == 't') {
            CurExpThresh -= 0.05;
            if (CurExpThresh < 0.05)
                CurExpThresh = 0.05;
            MultiSense.SetExpThresh(CurExpThresh);
            printf("New ExpThresh = %0.2f\n", CurExpThresh);
        }
        if ((char) c == 'T') {
            CurExpThresh += 0.05;
            if (CurExpThresh > 1.0)
                CurExpThresh = 1.0;
            MultiSense.SetExpThresh(CurExpThresh);
            printf("New ExpThresh = %0.2f\n", CurExpThresh);
        }

        if ((char) c == 'p') {
            CurPostFilt -= 0.05;
            if (CurPostFilt < 0.05)
                CurPostFilt = 0.05;
            MultiSense.SetPostFilt(CurPostFilt);
            printf("New PostFilt = %0.2f\n", CurPostFilt);
        }
        if ((char) c == 'P') {
            CurPostFilt += 0.05;
            if (CurPostFilt > 1.0)
                CurPostFilt = 1.0;
            MultiSense.SetPostFilt(CurPostFilt);
            printf("New PostFilt = %0.2f\n", CurPostFilt);
        }


    }

    // Stop image streams
    MultiSense.StopDisparityStream();
    MultiSense.StopLumaLeftStream();
    MultiSense.StopChromaLeftStream();

    return 0;
}
