/**
 * @file CloudDemo.cpp
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
 *   2014-05-14, br@carnegierobotics.com, PR1044, Created file.
 *   2014-07-31, ewestman@carnegierobotics.com, PR1044, cleaned up and
 *               optimized code for improved performance.
 **/

#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <ctype.h>
#include <pthread.h>
#include <sys/time.h>
#include <opencv2/opencv.hpp>
#include <CloudDemo.h>


#define USE_AUTO_MOTION 0       // Set to 1 in order to automatically pitch/roll 3D display


using namespace std;
using namespace cv;

CloudDemo* CloudDemo::p_self = NULL;


// Constructor to initialize member functions
CloudDemo::CloudDemo(BaseMultiSenseWrapper *MultiSenseArg)
    : MultiSense(MultiSenseArg),
      ImgCols(1024),            // These dimensions are just initial values -- reset to sensor dimensions below
      ImgRows(544),
      CurGain(2.6),
      CurFPS(15.0),
      CurMaxExp(60000),
      CurDisparities(128),
      CurExpThresh(0.65),
      CurExpDecay(3),
      CurPostFilt(0.05),

      // Setup local state.
      ElapsedMSInitialTime(0),

      // Setup for display
      GlutFOV(65.0),                  // Initial virtual camera field of view for OpenGL render
      title("CRL Multisense"),
      EyeX(0.0),
      EyeY(0.0),
      EyeZ(0.0),
      ViewX(0.0),
      ViewY(0.0),
      ViewZ(-1.0),
      ViewYaw(0.0),
      ViewPitch(3.14159/2.0),
      CloudGain(1.2),
      WinHeight(1080.0),   // Initial dimensions of on-screen display window
      WinWidth(1920.0),
      CloudPtSize(3.0),    // Size of points to draw when rendering point cloud
      XClipping(150.0),    // Dont display points farther to the left or right than this (meters).
      YClipping(50.0),     // Dont display points higher than this (meters).
      ZClipping(50.0),     // Dont display points farther than this (meters).
      DisplayMode(0),      // How to remap colors
      LeftMouseDown(0),
      RightMouseDown(0),
      MousePrevX(-1),
      MousePrevY(-1)
{
    // Setup CloudDemo to use the resolution chosen by the sensor itself
    ImgRows = this->MultiSense->m_grabbingRows;
    ImgCols = this->MultiSense->m_grabbingCols;
}


// Destructor.
CloudDemo::~CloudDemo()
{
    this->MultiSense->StopDisparityStream();
    this->MultiSense->StopLumaLeftStream();
    this->MultiSense->StopChromaLeftStream();
}


// Configure the field of view with which demo graphics will be
// displayed.
void CloudDemo::SetFieldOfView(float FieldOfView)
{
    this->GlutFOV = FieldOfView;
}


// Configure the aggressiveness with which noisy pixels will be
// pruned from the disparity image.
void CloudDemo::SetStereoPostFilter(float FilterLevel)
{
    // Enforce bounds on the argument.
    FilterLevel = std::max(FilterLevel, float(0.0));
    FilterLevel = std::min(FilterLevel, float(1.0));
    this->CurPostFilt = FilterLevel;
}


// Configure the MultiSense unit, and start image streams.
void CloudDemo::SetupMultisense()
{
    // Ask camera to start sending the appropriate images
    this->MultiSense->StartDisparityStream();
    this->MultiSense->StartLumaLeftStream();
    this->MultiSense->StartChromaLeftStream();

    // Set parameters for cameras
    this->MultiSense->SetGain(CurGain);
    this->MultiSense->SetFPS(CurFPS);
    this->MultiSense->SetMaxExp(CurMaxExp);
    this->MultiSense->SetDisparities(CurDisparities);
    this->MultiSense->SetExpThresh(CurExpThresh);
    this->MultiSense->SetExpDecay(CurExpDecay);
    this->MultiSense->SetPostFilt(CurPostFilt);
}


// Setup GLUT 3D display
void CloudDemo::SetupUserInterface(int argc, char** argvPP)
{
    glutInit(&argc, argvPP);                    // Initialize GLUT
    glutInitDisplayMode(GLUT_DOUBLE);           // Enable double buffered mode
    glutInitWindowSize((int) WinWidth, (int) WinHeight);   // Set the window's initial width & height
    glutInitWindowPosition(50, 50);             // Position the window's initial top-left corner
    glutCreateWindow(title);                    // Create window with the given title
    glutReshapeFunc(GlutReshapeCallback);       // Register callback handler for window re-size event
    glutKeyboardFunc(GlutKeyCallback);          // Register callback handler for the keyboard
    glutMouseFunc(GlutMouseClickCallback);      // Register callback handler for the rodent clicks
    glutMotionFunc(GlutMouseDownMotionCallback);// Register callback handler for the rodent clicks
    initGL();                         // Our own OpenGL initialization

    printf("\nCommon demo commands:\n");
    printf("\n");
    printf("-------MOUSE---------\n");
    printf("Left button:  Pitch and yaw view\n");
    printf("Scroll wheel: Fly forward and back\n");
    printf("Right button: Pan view\n");
    printf("\n");
    printf("------KEYBOARD-------\n");
    printf("Move camera:     w = move forwards             r = move up\n");
    printf("              a  &  d = move left and right\n");
    printf("                 s = move backwards            f = move down\n\n");
    printf("Tilt camera:     u = pitch up\n");
    printf("              h  &  k = yaw left and right\n");
    printf("                 j = pitch down\n\n");
    printf("g,G = Adjust live camera gain\n");
    printf("n,N = Adjust live camera noise (stereo post filtering)\n");
    printf("p,P = Adjust 3D point display size\n");
    printf("x,X = Adjust X clipping (removes left and right sides)\n");
    printf("y,Y = Adjust Y clipping (removes the ceiling)\n");
    printf("z,Z = Adjust Z clipping (removes distant objects)\n");
    printf("B,b = Start/Stop saving an image Bundle\n");
    printf("m,M = Step through variety of color display modes\n");
    printf(" ,  = Toggle overlay of range and color data in same display\n");
    printf(" .  = Change which axis (Z or Y) is used for color range display\n");
    printf("+,- = Brighten/darken on-screen display\n");
    printf(" q  = Quit program\n\n");
}


// This routine uses one of a variety of approaches to remap the colors to range values, etc...
void CloudDemo::RemapColors(const cv::Mat& Cloud, cv::Mat& Color, int32_t Mode)
{
    float X, Y, Z;
    float R, G, B;
    float Extent, ExtHalf, Offset, Range;
    float ZoneMin, ZoneMax;

    // Deconstruct display mode information
    int32_t ColorMap =   (Mode & 0x0000FF);             // Lowest byte of Mode indicates color remapping method to use
    int32_t Overlay =   ((Mode & 0x00FF00) > 0);        // Second byte indicates whether to use overlay or not
    int32_t ColorAxis = ((Mode & 0xFF0000) >> 16);    // Third byte indicates which axis to use for color gradient (Z, Y, or X)

    // Operate on the destired axis
    if (ColorAxis == 1) {
        // Y axis
        ZoneMin = -2.0;         // Fixed for now at a sensible value - may want to be changable in future
        ZoneMax = YClipping;
    }
    else if (ColorAxis == 2) {
        // X axis
        ZoneMin = 1.0;
        ZoneMax = XClipping;
    }
    else  {
        // Z axis
        ZoneMin = 1.25;
        ZoneMax = ZClipping;
    }

    Extent = ZoneMax - ZoneMin;   // Length, in meters, of the display area
    ExtHalf = Extent/2.0;         // Half way

    // Go through image and remap colors
    cv::Size ImageSize = Color.size();
    for (int rr = 0 ; rr < ImageSize.height; rr++) {
        for (int cc = 0 ; cc < ImageSize.width ; cc++) {

            // Get position of point
            cv::Vec3f WorldPosition = Cloud.at<cv::Vec3f>(rr, cc);
            X = WorldPosition[0];
            Y = WorldPosition[1]; // In this instance, Y is negative in the upwards direction
            Z = WorldPosition[2]; // Note -- Z range is already positive here

            // Get color of point
            cv::Vec3b ColorBgr = Color.at<cv::Vec3b>(rr, cc);
            B = ColorBgr[0] / 255.0;
            G = ColorBgr[1] / 255.0;
            R = ColorBgr[2] / 255.0;

            // Ad-hoc, simple noise filtering
            //if ((Y < -1.0) && (Z < 3.0))
                //Z = 0.0;
            //else if (Y < -20.0)
                //Z = 0.0;
            //if ((Z < 10.0) && (Y < -1.0) && (fabs(X) < 3.5))
                //Z = 0.0;
            //if ((B > 0.95) && (R < 0.4) && (G < 0.8))
                //Z = 0.0;

            // Operate on the destired axis
            if (ColorAxis == 1)
                Range = -Y;
            else if (ColorAxis == 2)
                Range = X;
            else
                Range = Z;

            Offset = Range - ZoneMin;   // Offset from minimum of this point

            switch (ColorMap & 0x000F) {
                case 0:
                    // Display pure RGB color
                    if ((Range > ZoneMax) || (Range < ZoneMin))
                        B = G = R = 0.0;
                    break;

                case 1:
                    // Display greyscale range -- far is dark
                    if (! Overlay) {
                        B = (float) 1.0 - (Offset/Extent);
                        G = (float) 1.0 - (Offset/Extent);
                        R = (float) 1.0 - (Offset/Extent);
                    }
                    else {
                        B *= (float) 1.0 - (Offset/Extent);
                        G *= (float) 1.0 - (Offset/Extent);
                        R *= (float) 1.0 - (Offset/Extent);
                    }
                    // Enforce bounds
                    if ((Range > ZoneMax) || (Range < ZoneMin))
                        B = G = R = 0.0;
                    break;

                case 2:
                    // Display greyscale range -- far is light
                    if (! Overlay) {
                        B = (float) (Offset/Extent);
                        G = (float) (Offset/Extent);
                        R = (float) (Offset/Extent);
                    }
                    else {
                        B *= (float) (Offset/Extent);
                        G *= (float) (Offset/Extent);
                        R *= (float) (Offset/Extent);
                    }
                    // Enforce bounds
                    if ((Range > ZoneMax) || (Range < ZoneMin))
                        B = G = R = 0.0;
                    break;

                case 3:
                    // Display range as shading from Red to Blue
                    if (! Overlay) {
                        B = (float) Offset/Extent;
                        G = (float)  0.0;
                        R = (float) (Extent - Offset)/Extent;
                    }
                    else {
                        B *= (float) Offset/Extent;
                        G *= (float)  0.0;
                        R *= (float) (Extent - Offset)/Extent;
                    }
                    if ((Range > ZoneMax) || (Range < ZoneMin))
                        B = G = R = 0.0; // Enforce bounds
                    break;

                case 4:
                    // Display range as shading from Green to Blue
                    if (! Overlay) {
                        B = (float) Offset/Extent;
                        G = (float) (Extent - Offset)/Extent;
                        R = (float)  0.0;
                    }
                    else {
                        B *= (float) Offset/Extent;
                        G *= (float) (Extent - Offset)/Extent;
                        R *= (float)  0.0;
                    }
                    if ((Range > ZoneMax) || (Range < ZoneMin))
                        B = G = R = 0.0; // Enforce bounds
                    break;

                case 5:
                    // Display range as orange->purple->blue ("heat map style")
                    if (! Overlay) {
                        B = (float) Offset/Extent;
                        G = (float)  fabs((Extent/2.0)-Offset) / Extent;
                        R = (float) (Extent - Offset)/Extent;
                    }
                    else {
                        B *= (float) Offset/Extent;
                        G *= (float)  fabs((Extent/2.0)-Offset) / Extent;
                        R *= (float) (Extent - Offset)/Extent;
                    }
                    if ((Range > ZoneMax) || (Range < ZoneMin))
                        B = G = R = 0.0; // Enforce bounds
                    break;

                case 6:
                    // Display as full rainbow of range
                    ExtHalf = Extent/2.0;
                    if (! Overlay) {
                        R = 1.0 - (Offset / ExtHalf);
                        if (R < 0.0) R = 0.0;
                        B = (Offset / ExtHalf) - 1.0;
                        if (B < 0.0) B = 0.0;
                        G = 1.0 - B - R;
                    }
                    else {
                        float RR, GG, BB;
                        RR = 1.0 - (Offset / ExtHalf);
                        if (RR < 0.0) RR = 0.0;
                        BB = (Offset / ExtHalf) - 1.0;
                        if (BB < 0.0) BB = 0.0;
                        GG = 1.0 - BB - RR;
                        R *= RR;
                        G *= GG;
                        B *= BB;
                    }
                    if ((Range > ZoneMax) || (Range < ZoneMin))
                        B = G = R = 0.0; // Enforce bounds
                    break;

                default:
                    break;
            }   // switch

            // Enforce display clipping
            if ((fabs(X) > XClipping) || (-Y > YClipping) || (Z > ZClipping))
                B = G = R = 0.0; // Clip it

            // Set new point color
            ColorBgr[0] = MIN(255, B * 255.0 * CloudGain);
            ColorBgr[1] = MIN(255, G * 255.0 * CloudGain);
            ColorBgr[2] = MIN(255, R * 255.0 * CloudGain);
            Color.at<cv::Vec3b>(rr, cc) = ColorBgr;
        }
    }
}



void CloudDemo::PopulateVoxels(const cv::Mat& Cloud, const cv::Mat& Color,
                               VoxelArray& voxelArray, uint8_t flag,
                               bool doErase)
{
    // Zero out the voxel array by traversing the linked list of
    // populated voxels.
    if(doErase) {
        int32_t VIndex = voxelArray.FirstVoxel;
        while (VIndex != 0) {
            voxelArray.Voxels[VIndex].Flag = 0;
            VIndex = voxelArray.Voxels[VIndex].Next;
        }
        voxelArray.FirstVoxel = voxelArray.LastVoxel = 0;
    }

    // Interpret the point cloud
    // Ignore the first few rows with green noise
    cv::Size ImageSize = Color.size();
    float OriginX = voxelArray.XSize / 2.0;
    float OriginY = voxelArray.YSize / 2.0;
    for (int rr = 20 ; rr < ImageSize.height; rr++) {
        for (int cc = 0 ; cc < ImageSize.width ; cc++) {

            // Figure out true world location (in camera coordinates)
            cv::Vec3f WorldPosition = Cloud.at<cv::Vec3f>(rr, cc);

            // Convert to voxel grid coordinates.  We choose the
            // origin of the voxel grid to have Z coordinate == 0.
            int32_t VX = static_cast<int32_t>(
                WorldPosition[0] / voxelArray.VoxelDimension + OriginX);
            int32_t VY = static_cast<int32_t>(
                WorldPosition[1] / voxelArray.VoxelDimension + OriginY);
            int32_t VZ = static_cast<int32_t>(
                WorldPosition[2] / voxelArray.VoxelDimension);

            // Bounds checking.
            if ((VX < 0) || (VX >= static_cast<int32_t>(voxelArray.XSize)))
                continue;
            if ((VY < 0) || (VY >= static_cast<int32_t>(voxelArray.YSize)))
                continue;
            if ((VZ < 0) || (VZ >= static_cast<int32_t>(voxelArray.ZSize)))
                continue;

            // Index into voxelArray.Voxels, which is a flat array.
            int32_t VIndex = (VX + (VY*voxelArray.XSize)
                              + (VZ*voxelArray.XSize*voxelArray.YSize));

            if (VIndex == 0) {
                // Ignore this point so we can use 0 for end of linked list
                continue;
            }

            if (voxelArray.Voxels[VIndex].Flag != 0) {
                // A pixel already hit this voxel
                continue;
            }

            // Figure out proper RGB color.  Note that opencv
            // color convention is BGR.
            cv::Vec3b ColorBgr = Color.at<cv::Vec3b>(rr, cc);
            voxelArray.Voxels[VIndex].R = ColorBgr[2];
            voxelArray.Voxels[VIndex].G = ColorBgr[1];
            voxelArray.Voxels[VIndex].B = ColorBgr[0];
            voxelArray.Voxels[VIndex].Flag = flag;

            // Setup simple linked list of all valid voxels
            if (voxelArray.FirstVoxel == 0) {
                voxelArray.FirstVoxel = VIndex;
            } else {
                voxelArray.Voxels[voxelArray.LastVoxel].Next = VIndex;
            }

            voxelArray.LastVoxel = VIndex;
            voxelArray.Voxels[VIndex].Next = 0;
        }
    }
}



// This callback is invoked periodically by FreeGlut.
void CloudDemo::IdleFunc()
{
    // Declare the OpenCV Mats that are required for this demo
    Mat Cloud;
    Mat Color;
    Mat Disparity;
    Mat* CloudP = 0;
    Mat* ColorP = 0;

    // Call library function to manage display, keyboard, etc...
    this->GlutIdleWorker();

    // Get latest color image
    // or wait until one is available
    while (Color.empty()) {
        Color = CloudDemo::MultiSense->CopyLeftRectifiedRGB();
        usleep(1000);
    }
    if (!Color.empty()) {
        ColorP = &Color;
    }

    // Get latest image of corresponding 3D points
    // or wait until one is available
    while (Cloud.empty()) {
        Cloud = CloudDemo::MultiSense->CopyCloud(Disparity);
        usleep(1000);
    }
    if (!Cloud.empty()) {
        CloudP = &Cloud;
    }

    // Run our demo
    if ((CloudP != 0) && (ColorP != 0)) {
        // Change display colormapping depending on display mode
        // Last two arguments are Z clipping min range and max range
        RemapColors(*CloudP, *ColorP, DisplayMode);

        // Draw points on screen using this demo's method
        this->DrawDemo(CloudP, ColorP);

        // Swap the front and back frame buffers (double buffering)
        glutSwapBuffers();
    }

#if USE_AUTO_MOTION
    // Automatic, continous camera motion to show 3D effect
    static uint32_t InitMotion = 1;
    static uint32_t Step = 0;

    // Initial location for auto motion
    if (InitMotion) {
        // Upper right corner of motion square
        EyeY += 30.0 * (1.0/40.0);
        EyeX -= ViewZ * (30.0 * (1.0/25.0));
        EyeZ += ViewX * (30.0 * (1.0/25.0));
    }

    // Auto motion of camera
    if (Step < 30) {            // Down
        EyeY -= (1.0/40.0);
    }
    else if (Step < 60) {       // Left
        EyeX += ViewZ * (1.0/25);
        EyeZ -= ViewX * (1.0/25);
    }
    else if (Step < 90) {       // Up
        EyeY += (1.0/40.0);
    }
    else {                      // Right
        EyeX -= ViewZ * (1.0/25);
        EyeZ += ViewX * (1.0/25);
    }

    // Initial pointing for auto pointing of the camera
    if (InitMotion) {
        // Pitch down
        ViewPitch += 15.0 * (1.0/90.0);
        ViewY = cos(ViewPitch);

        // Yaw left
        ViewYaw -= 15.0 * (1.0/90.0);
        ViewX = sin(ViewYaw);
        ViewZ = -cos(ViewYaw);
    }

    // Auto pointing of camera
    if (Step < 30) {            // Pitch upwards
        ViewPitch -= (1.0/90.0);
        ViewY = cos(ViewPitch);
    }
    else if (Step < 60) {       // Yaw rightwards
        ViewYaw += (1.0/90.0);
        ViewX = sin(ViewYaw);
        ViewZ = -cos(ViewYaw);
    }
    else if (Step < 90) {       // Pitch downwards
        ViewPitch += (1.0/90.0);
        ViewY = cos(ViewPitch);
    }
    else {                      // Yaw leftwards
        ViewYaw -= (1.0/90.0);
        ViewX = sin(ViewYaw);
        ViewZ = -cos(ViewYaw);
    }

    // Step to next location in auto motion / pointing
    Step++;
    if (Step >= 120)
        Step = 0;

    if (InitMotion)
        InitMotion = 0;
#endif

    return;
}



// Get milliseconds since first call to this routine.
uint64_t CloudDemo::ElapsedMS()
{
    struct timeval TV;
    struct timezone TZ;

    TZ.tz_dsttime = 0;
    TZ.tz_minuteswest = 0;
    gettimeofday(&TV, &TZ);

    uint64_t CurrentTime = static_cast<uint64_t>(TV.tv_sec) * 1000;
    CurrentTime += (TV.tv_usec / 1000);

    if (this->ElapsedMSInitialTime == 0) {
      this->ElapsedMSInitialTime = CurrentTime;
    }

    return(CurrentTime - this->ElapsedMSInitialTime);
}


// Cloud coordinates have X pointing left, Y pointing down, and Z
// pointing in the camera view direction, with the origin at the
// camera.
void CloudDemo::ConvertCloudCoordsToDisplay(
    int32_t CloudX, int32_t CloudY, int32_t CloudZ,
    VoxelArray const& VoxelArrayArg,
    float& DisplayX, float& DisplayY, float& DisplayZ)
{
    // Assume the camera is at Z = 0, and otherwise centered on the
    // pointcloud.
    int32_t OriginX = VoxelArrayArg.XSize >> 1;
    int32_t OriginY = VoxelArrayArg.YSize >> 1;

    // Cloud coordinates have X pointing left, Y pointing
    // down, and Z pointing in the camera view direction, with
    // the origin at the camera.
    //
    // Default OpenGL coordinates have X pointing left, Y
    // pointing up, and Z pointing against the view direction,
    // with the origin at the camera, so that all visible Z
    // coordinates are negative.
    //
    // We make the transformation from camera coordinates to
    // OpenGL coordinates by simply negating Y and Z.
    DisplayX = ((CloudX - OriginX) * VoxelArrayArg.VoxelDimension);
    DisplayY = -((CloudY - OriginY) * VoxelArrayArg.VoxelDimension);
    DisplayZ = -(CloudZ * VoxelArrayArg.VoxelDimension);
}


// Draw 2D text on screen.
// Scale is scaling of nominal 24-point font.
void CloudDemo::Draw2DScreenText(
    int XPos,       // True raster pixel column - if negative then this is an offset from the RHS of window.
    int YPos,       // True raster pixel row (from bottom) - if negative then this is an offset from the TOP of window.
    float Scale,    // rough size of text
    float R, float G, float B,
    const unsigned char *Text)
{

    // Make scale smaller so it matches bitmap 24 point font
    Scale *= 0.15;

    // Setup 2D drawing mode to draw pixels right on the raster screen
    glDisable(GL_TEXTURE_2D);
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0.0, WinWidth, 0.0, WinHeight);
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    glColor3f(R, G, B); // Set text color

    // Allow negative positions to provide offset from "far" edges of window.
    // There is no bounds checking.
    if (XPos < 0) {
        XPos = (int) WinWidth + XPos;
    }

    if (YPos < 0) {
        YPos = (int) WinHeight + YPos;
    }

    // Scale the points
    glTranslatef((float) XPos, (float) YPos, 0.0f);
    glScalef(Scale, Scale, 1.0f);
    glLineWidth(1.5 * Scale / 0.15);
    glutStrokeString(GLUT_STROKE_MONO_ROMAN, (const unsigned char *) Text);

    // Back to 3D rendering mode.
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();
    glEnable(GL_TEXTURE_2D);
    return;
}


// Draw a wireframe box
void CloudDemo::DrawBox(float X, float Y, float Z,
                        float XDim, float YDim, float ZDim,
                        float R, float G, float B)
{
    glColor3f(R, G, B);

    // Draw all 6 faces of a wireframe cube
    // FIXME - this draws more lines than are required...
    glBegin(GL_LINE_LOOP);
    glLineWidth(3.0);
    glVertex3f(X, Y, Z);
    glVertex3f(X+XDim, Y, Z);
    glVertex3f(X+XDim, Y+YDim, Z);
    glVertex3f(X, Y+YDim, Z);
    glEnd();

    glBegin(GL_LINE_LOOP);
    glLineWidth(3.0);
    glVertex3f(X, Y, Z+ZDim);
    glVertex3f(X+XDim, Y, Z+ZDim);
    glVertex3f(X+XDim, Y+YDim, Z+ZDim);
    glVertex3f(X, Y+YDim, Z+ZDim);
    glEnd();

    glBegin(GL_LINE_LOOP);
    glLineWidth(3.0);
    glVertex3f(X, Y, Z);
    glVertex3f(X, Y+YDim, Z);
    glVertex3f(X, Y+YDim, Z+ZDim);
    glVertex3f(X, Y, Z+ZDim);
    glEnd();

    glBegin(GL_LINE_LOOP);
    glLineWidth(3.0);
    glVertex3f(X+XDim, Y, Z);
    glVertex3f(X+XDim, Y+YDim, Z);
    glVertex3f(X+XDim, Y+YDim, Z+ZDim);
    glVertex3f(X+XDim, Y, Z+ZDim);
    glEnd();

    glBegin(GL_LINE_LOOP);
    glLineWidth(3.0);
    glVertex3f(X, Y, Z);
    glVertex3f(X+XDim, Y, Z);
    glVertex3f(X+XDim, Y, Z+ZDim);
    glVertex3f(X, Y, Z+ZDim);
    glEnd();

    glBegin(GL_LINE_LOOP);
    glLineWidth(3.0);
    glVertex3f(X, Y+YDim, Z);
    glVertex3f(X+XDim, Y+YDim, Z);
    glVertex3f(X+XDim, Y+YDim, Z+ZDim);
    glVertex3f(X, Y+YDim, Z+ZDim);
    glEnd();

    return;
}


// Draw a cube
void CloudDemo::DrawCube(float X, float Y, float Z, float Dim, float R, float G, float B)
{
    glColor3f(R, G, B);

    // Draw all 6 faces of a cube
    glBegin(GL_POLYGON);
    glVertex3f(X, Y, Z); glVertex3f(X+Dim, Y, Z);
    glVertex3f(X+Dim, Y+Dim, Z); glVertex3f(X, Y+Dim, Z);
    glEnd();

    glBegin(GL_POLYGON);
    glVertex3f(X, Y, Z+Dim); glVertex3f(X+Dim, Y, Z+Dim);
    glVertex3f(X+Dim, Y+Dim, Z+Dim); glVertex3f(X, Y+Dim, Z+Dim);
    glEnd();

    glBegin(GL_POLYGON);
    glVertex3f(X, Y, Z); glVertex3f(X, Y+Dim, Z);
    glVertex3f(X, Y+Dim, Z+Dim); glVertex3f(X, Y, Z+Dim);
    glEnd();

    glBegin(GL_POLYGON);
    glVertex3f(X+Dim, Y, Z); glVertex3f(X+Dim, Y+Dim, Z);
    glVertex3f(X+Dim, Y+Dim, Z+Dim); glVertex3f(X+Dim, Y, Z+Dim);
    glEnd();

    glBegin(GL_POLYGON);
    glVertex3f(X, Y, Z); glVertex3f(X+Dim, Y, Z);
    glVertex3f(X+Dim, Y, Z+Dim); glVertex3f(X, Y, Z+Dim);
    glEnd();

    glBegin(GL_POLYGON);
    glVertex3f(X, Y+Dim, Z); glVertex3f(X+Dim, Y+Dim, Z);
    glVertex3f(X+Dim, Y+Dim, Z+Dim); glVertex3f(X, Y+Dim, Z+Dim);
    glEnd();

    return;
}


// This fills the specified volume (which may have one dimension zero)
// with spaced points.  Volume is cubic and orthogonal to axes
void CloudDemo::DrawMeshWall(float Spacing, float MinX, float MaxX,
                             float MinY, float MaxY, float MinZ, float MaxZ,
                             float R, float G, float B, float PtSize)
{
    float X, Y, Z;

    glPointSize(PtSize);
    glBegin(GL_POINTS);

    glColor3f(R, G, B);

    for (X = MinX ; X <= MaxX ; X += Spacing) {
        for (Y = MinY ; Y <= MaxY ; Y += Spacing) {
            for (Z = MinZ ; Z <= MaxZ ; Z += Spacing) {
                glVertex3f(X, Y, Z);
            }
        }
    }

    glEnd();

    return;
}


void CloudDemo::DrawVoxels(const VoxelArray& VoxelArrayArg,
                           float YMax,
                           float ZMax)
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

        //float BB = static_cast<float>((iB / 255.0) * CloudGain);
        //float GG = static_cast<float>((iG / 255.0) * CloudGain);
        //float RR = static_cast<float>((iR / 255.0) * CloudGain);
        float BB = static_cast<float>((iB / 255.0));
        float GG = static_cast<float>((iG / 255.0));
        float RR = static_cast<float>((iR / 255.0));

        // Ignore voxels that are within 10cm of the camera, as they
        // fill up too much of the field of view!
        if ((YY < YMax) && (ZZ < ZMax)){
            DrawCube(XX, YY, ZZ, VoxelArrayArg.VoxelDimension, RR, GG, BB);
        }

        // Follow to the next element of the linked list.
        VIndex = VoxelArrayArg.Voxels[VIndex].Next;
    }
}


// Initialize OpenGL Graphics
void CloudDemo::initGL() {
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);   // Set background color to black and opaque
    glClearDepth(1.0f);                     // Set background depth to farthest
    glEnable(GL_DEPTH_TEST);                // Enable depth testing for z-culling
    glShadeModel(GL_SMOOTH);                // Enable smooth shading
    glHint(GL_PERSPECTIVE_CORRECTION_HINT,
           GL_NICEST);                      // Nice perspective corrections
}


// Called periodically at GLUT's leisure
void CloudDemo::GlutIdleWorker()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear color and depth buffers
    glMatrixMode(GL_MODELVIEW);     // To operate on model-view matrix

    // This resets the view to the default
    // Below we move the view back to where we want it
    glLoadIdentity();                 // Reset the model-view matrix

    // Set view
    gluLookAt(EyeX, EyeY, EyeZ,                     // X,Y,Z of "eye point"
              EyeX+ViewX, EyeY+ViewY, EyeZ+ViewZ,     // X,Y,Z of what we're looking at
              0, 1, 0);                               // UP vector (Roll)

    glutPostRedisplay();
    glFlush();
    return;
}


// Handler for window re-size event. Called back when the window first appears and
// whenever the window is re-sized with its new width and height.
void CloudDemo::GlutReshape(GLsizei width, GLsizei height)
{
    WinHeight = (double) height;
    WinWidth = (double) width;

    // GLsize for non-negative integer
    // Compute aspect ratio of the new window
    if (height == 0){
        height = 1;                // To prevent divide by 0
    }

    GLfloat aspect = (GLfloat)width / (GLfloat)height;

    // Set the viewport to cover the new window
    glViewport(0, 0, width, height);

    // Set the aspect ratio of the clipping volume to match the viewport
    glMatrixMode(GL_PROJECTION);  // To operate on the Projection matrix
    glLoadIdentity();             // Reset

    // Enable perspective projection with fovy, aspect, zNear and zFar
    gluPerspective(GlutFOV, aspect, 0.1f, 100.0f);
}


// Handles keyboard callback routine
void CloudDemo::GlutKey(unsigned char key, int x, int y)
{
    (void) x;
    (void) y;

    // Strafe in view frame
    if (key == 'a') {
        EyeX += ViewZ * 0.10;
        EyeZ -= ViewX * 0.10;
    }

    if (key == 'd') {
        EyeX -= ViewZ * 0.10;
        EyeZ += ViewX * 0.10;
    }


    // Fly forward in view frame
    if (key == 'w') {
        EyeX += ViewX * 0.10;
        EyeZ += ViewZ * 0.10;
        EyeY += ViewY * 0.10;
    }

    if (key == 's') {
        EyeX -= ViewX * 0.10;
        EyeZ -= ViewZ * 0.10;
        EyeY -= ViewY * 0.10;
    }

    // Up and down in Y
    if (key == 'r') {
        EyeY += 0.10;
    }

    if (key == 'f') {
        EyeY -= 0.10;
    }

    // Pitch view frame
    if (key == 'u') {
        ViewPitch -= 0.10;
        ViewY = cos(ViewPitch);
    }

    if (key == 'j') {
        ViewPitch += 0.10;
        ViewY = cos(ViewPitch);
    }

    // Yaw view frame
    if (key == 'h') {
        ViewYaw -= 0.10;
        ViewX = sin(ViewYaw);
        ViewZ = -cos(ViewYaw);
    }

    if (key == 'k') {
        ViewYaw += 0.10;
        ViewX = sin(ViewYaw);
        ViewZ = -cos(ViewYaw);
    }

    if (key == 'q') {
        glutLeaveMainLoop();
    }

    // Adjust 3D point size for cloud render
    if (key == 'p') {
        if (CloudPtSize > 0.2)
            CloudPtSize -= 0.2;
        printf("New point size = %0.2f\n", CloudPtSize);
    }
    if (key == 'P') {
        CloudPtSize += 0.2;
        printf("New point size = %0.2f\n", CloudPtSize);
    }


    // Control camera sensor gain
    if (key == 'g') {
        CurGain -= 0.2;
        if (CurGain < 0.2) {
            CurGain = 0.2;
        }

        this->MultiSense->SetGain(CurGain);
        printf("New Gain = %0.2f\n", CurGain);
    }
    if (key == 'G') {
        CurGain += 0.2;
        if (CurGain > 6.0){
            CurGain = 6.0;
        }
        std::cout << "Exiting" << std::endl;

        this->MultiSense->SetGain(CurGain);
        printf("New Gain = %0.2f\n", CurGain);
    }

    // Control pixel screen display gain / brightness
    if ((key == '-') || (key == '_'))
        CloudGain *= 0.9;

    if ((key == '=') || (key == '+'))
        CloudGain *= 1.1;


    // Control post-filtering
    if (key == 'n') {
        CurPostFilt -= 0.05;
        if (CurPostFilt < 0.05) {
            CurPostFilt = 0.05;
        }

        this->MultiSense->SetPostFilt(CurPostFilt);
        printf("New PostFilt = %0.2f\n", CurPostFilt);
    }
    if (key == 'N') {
        CurPostFilt += 0.05;
        if (CurPostFilt > 1.0) {
            CurPostFilt = 1.0;
        }

        this->MultiSense->SetPostFilt(CurPostFilt);
        printf("New PostFilt = %0.2f\n", CurPostFilt);
    }

    // Control X clipping (display of pixels to left and right)
    if (((key == 'x') || (key == 'X')) && (XClipping == 150.0)) {
        // Turn on X clipping by setting it to a reasonable initial value
        XClipping = 10.1;
    }
    if (key == 'x') {
        XClipping -= 1.00;
        printf("New XClipping = %0.2f\n", XClipping);
    }
    if (key == 'X') {
        XClipping += 1.00;
        printf("New XClipping = %0.2f\n", XClipping);
    }

    // Control Y clipping (display of pixels above a certain height)
    if (((key == 'y') || (key == 'Y')) && (YClipping == 50.0)) {
        // Turn on Y clipping by setting it to a reasonable initial value
        YClipping = 3.0;
    }
    if (key == 'y') {
        YClipping -= 0.10;
        printf("New YClipping = %0.2f\n", YClipping);
    }
    if (key == 'Y') {
        YClipping += 0.10;
        printf("New YClipping = %0.2f\n", YClipping);
    }

    // Control Z clipping (display of pixels beyond a certain distance)
    if (((key == 'z') || (key == 'Z')) && (ZClipping == 50.0)) {
        // Turn on Z clipping by setting it to a reasonable initial value
        ZClipping = 25.1;
    }
    if (key == 'z') {
        ZClipping -= 1.0;
        printf("New ZClipping = %0.2f\n", ZClipping);
    }
    if (key == 'Z') {
        ZClipping += 1.0;
        printf("New ZClipping = %0.2f\n", ZClipping);
    }

    // Control display mode (color remapping)
    // DisplayMode variable encodes color mode in lowest byte
    if (key == 'm') {
        int32_t Mode = DisplayMode & 0x0000FF;
        Mode++;
        if (Mode > 6)
            Mode = 0;
        DisplayMode = (DisplayMode & 0xFFFF00) + Mode;
    }
    if (key == 'M') {
        int32_t Mode = DisplayMode & 0x0000FF;
        if (Mode == 0)
            Mode = 6;
        else
            Mode--;
        DisplayMode = (DisplayMode & 0xFFFF00) + Mode;
    }

    // Or... Number keys switch directly to desired mode
    if (key == '0')
        DisplayMode = (DisplayMode & 0xFFFF00) + 0;
    if (key == '1')
        DisplayMode = (DisplayMode & 0xFFFF00) + 1;
    if (key == '2')
        DisplayMode = (DisplayMode & 0xFFFF00) + 2;
    if (key == '3')
        DisplayMode = (DisplayMode & 0xFFFF00) + 3;
    if (key == '4')
        DisplayMode = (DisplayMode & 0xFFFF00) + 4;
    if (key == '5')
        DisplayMode = (DisplayMode & 0xFFFF00) + 5;
    if (key == '6')
        DisplayMode = (DisplayMode & 0xFFFF00) + 6;

    // Display mode - toggle overlay
    // Overlay mode is encoded in 2nd byte if DisplayMode
    if (key == ',') {
        if (DisplayMode & 0x00FF00)
            DisplayMode = DisplayMode & 0xFF00FF;
        else
            DisplayMode = DisplayMode | 0x000100;
    }

    // Display mode - choose axis for range display
    // Axis is encoded in 3nd byte if DisplayMode
    if (key == '.') {
        if (DisplayMode & 0xFF0000)
            DisplayMode = DisplayMode & 0xFF00FFFF;
        else
            DisplayMode = DisplayMode | 0x00010000;
    }

    return;
}


// Handles mouse events
void CloudDemo::GlutMouseClick(int32_t Button, int32_t State, int32_t X, int32_t Y)
{
    // Button values are: GLUT_LEFT_BUTTON GLUT_MIDDLE_BUTTON GLUT_RIGHT_BUTTON
    // State values are: GLUT_DOWN GLUT_UP
    // X & Y are relative to upper left corner of window

    if ((Button == GLUT_LEFT_BUTTON) && (State == GLUT_DOWN))
        LeftMouseDown = 1;
    if ((Button == GLUT_LEFT_BUTTON) && (State == GLUT_UP))
        LeftMouseDown = 0;

    if ((Button == GLUT_RIGHT_BUTTON) && (State == GLUT_DOWN))
        RightMouseDown = 1;
    if ((Button == GLUT_RIGHT_BUTTON) && (State == GLUT_UP))
        RightMouseDown = 0;


    // Check for mouse wheel action -- zoom in and out
    if (Button == 3) {
        // On many machines this will be mouse wheel up
        // Fly forward in view frame
        EyeX += ViewX * 0.10;
        EyeZ += ViewZ * 0.10;
        EyeY += ViewY * 0.10;
    }
    if (Button == 4) {
        // On many machines this will be mouse wheel down
        EyeX -= ViewX * 0.10;
        EyeZ -= ViewZ * 0.10;
        EyeY -= ViewY * 0.10;
    }

    MousePrevX = X;
    MousePrevY = Y;

    return;
}


// Handles mouse-down motion events
void CloudDemo::GlutMouseDownMotion(int32_t X, int32_t Y)
{
    int32_t dX, dY;

    // X & Y are relative to upper left corner of window
    dX = X - MousePrevX;
    dY = Y - MousePrevY;

    // Pan view with right mouse button
    if (RightMouseDown) {
        // Pan view frame up-down
        EyeY += (float) dY/200.0;
        // Pan view frame left-right
        EyeX += ViewZ * ((float) dX/200.0);
        EyeZ -= ViewX * ((float) dX/200.0);
    }

    // Pitch and yaw view with left mouse button
    if (LeftMouseDown) {
        // Pitch view frame
        ViewPitch += (float) dY/200.0;
        ViewY = cos(ViewPitch);
        // Yaw view frame
        ViewYaw += (float) dX/200.0;
        ViewX = sin(ViewYaw);
        ViewZ = -cos(ViewYaw);
    }

    MousePrevX = X;
    MousePrevY = Y;
    return;
}


// Handles mouse-wheel motion events
void CloudDemo::GlutMouseWheel(int32_t Button, int32_t dir, int32_t x, int32_t y)
{
    (void) Button;
    (void) x;
    (void) y;

    if (dir > 0) {
        printf("Zoom in!\n");
    }
    else {
        printf("Zoom out!\n");
    }

    return;
}


// Callback which invokes non-static method GlueIdleWorker()
void CloudDemo::GlutIdleWorkerCallback()
{
    p_self->GlutIdleWorker();
}


// Callback which invokes non-static method GlueDisplay())
void CloudDemo::GlutReshapeCallback(GLsizei width, GLsizei height)
{
    p_self->GlutReshape(width, height);
}


// Callback which invokes non-static method GlutReshape()
void CloudDemo::GlutKeyCallback(unsigned char key, int x, int y)
{
    p_self->GlutKey(key, x, y);
}


// Callback which invokes non-static method GlutMouseClick()
void CloudDemo::GlutMouseClickCallback(int32_t Button, int32_t State, int32_t X, int32_t Y)
{
    p_self->GlutMouseClick(Button, State, X, Y);
}


// Callback which invokes non-static method GlutMouseDownMotion()
void CloudDemo::GlutMouseDownMotionCallback(int32_t X, int32_t Y)
{
    p_self->GlutMouseDownMotion(X, Y);
}


// Callback which invokes non-static method GlutMouseWheel()
void CloudDemo::GlutMouseWheelCallback(int32_t Button, int32_t Dir, int32_t X, int32_t Y)
{
    p_self->GlutMouseWheel(Button, Dir, X, Y);
}
