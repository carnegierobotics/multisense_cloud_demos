/**
 * @file CloudDemos.h
 *
 * Simple MultiSense demos.
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

#ifndef CRL_CLOUDDEMO_H
#define CRL_CLOUDDEMO_H

#include <stdint.h>

#include <limits>
#include <string>

#include <GL/freeglut.h>                // FreeGlut is preferred version

#include <MultiSenseWrapper.h>


/**
 ** Abstract base class from which to derive CloudDemo demonstration
 ** programs.
 **/
class CloudDemo {
public:

    // GLUT doesn't allow us to pass a userdata pointer to our idle
    // function, so we record the address of the demo instance in a
    // static member variable, and provide a static member function to
    // use for the callback itself.
    static CloudDemo* p_self;
    static void IdleFuncCallback() {CloudDemo::p_self->IdleFunc();}


    /**
     * Constructor.
     *
     * @param MultiSenseArg This argument is an interface to the
     * MultiSense unit from which data should be taken.
     */
    CloudDemo(BaseMultiSenseWrapper *MultiSenseArg);


    /**
     * Destructor.
     */
    virtual ~CloudDemo();


    /**
     * Configure the field of view with which demo graphics will be
     * displayed.
     *
     * @param FieldOfView This argument is the field of view in
     * degrees.
     */
    void SetFieldOfView(float FieldOfView);


    /**
     * Configure the aggressiveness with which noisy pixels will be
     * pruned from the disparity image.  Higher numbers are more
     * aggressive.
     *
     * @param FieldOfView This argument is the desired filter level in
     * the range [0 ... 1].
     */
    void SetStereoPostFilter(float FilterLevel);


    /**
     * Configure the MultiSense unit, and start image streams.
     */
    void SetupMultisense();


    /**
     * Initializes a simple GLUT UI.
     *
     * @param argc Number of command-line arguments you'd like to pass to
     * the underlying graphics library.
     *
     * @param argvPP Vector of command-line arguments you'd like to pass
     * to the underlying graphics library.
     *
     * @return The return value is always 1.
     */
    void SetupUserInterface(int argc, char** argvPP);


protected:

    // Structure to hold one 3D "voxel"
    struct VoxStruct {
        uint8_t R;
        uint8_t G;
        uint8_t B;
        uint8_t Flag;       // Bit 0 is occupied or not
        uint32_t Next;      // Index to next occupied voxel
    };


    // Structure to hold an array of maybe-full, maybe-empty voxels in
    // which populated voxels are joined in a linked list.
    struct VoxelArray {
        VoxStruct* Voxels;
        uint32_t XSize;
        uint32_t YSize;
        uint32_t ZSize;
        float VoxelDimension;
        uint32_t FirstVoxel;
        uint32_t LastVoxel;

        VoxelArray(uint32_t XSizeArg, uint32_t YSizeArg, uint32_t ZSizeArg,
                   float VoxelDimensionArg)
            : Voxels(new VoxStruct[XSizeArg * YSizeArg * ZSizeArg]),
              XSize(XSizeArg),
              YSize(YSizeArg),
              ZSize(ZSizeArg),
              VoxelDimension(VoxelDimensionArg),
              FirstVoxel(0),
              LastVoxel(0) {}
        ~VoxelArray() {delete[] Voxels;}

        void IndexToXyz(uint32_t Index,
                        int32_t& XX, int32_t& YY, int32_t& ZZ) const {
            XX = Index % this->XSize;
            Index /= XSize;
            YY = Index % this->YSize;
            Index /= this->YSize;
            ZZ = Index;
        }

        void XyzToIndex(int32_t XX, int32_t YY, int32_t ZZ,
                        uint32_t& Index) const {
            Index = XX + (this->XSize * (YY + this->YSize * ZZ));
        }
    };


    /**
     * Given input point cloud and rectified color image, remap colors.
     *
     * @param Cloud This argument is the input point cloud, a cv::Mat
     * of cv::Vec3f.
     *
     * @param Color This argument is the input rectified color image,
     * a CV_8UC3 cv::Mat.  It must have the same shape as Cloud.
     *
     * @param Mode This argument specifies how to remap the colors.
     *
     */
    void RemapColors(const cv::Mat& Cloud, cv::Mat& Color, int32_t Mode=0);


    /**
     * Given input point cloud and rectified color image, construct a
     * linked list of VoxStruct instances, in which each instance is
     * stored in one cell of VoxArray.
     *
     * @param Cloud This argument is the input point cloud, a cv::Mat
     * of cv::Vec3f.
     *
     * @param Color This argument is the input rectified color image,
     * a CV_8UC3 cv::Mat.  It must have the same shape as Cloud.
     *
     * @param voxelArray This argument represents the voxel grid to be
     * populated.
     *
     * @param flag This argument specifies what the flag value of any
     * populated pixels should be set to.
     *
     * @param doErase If this argument is true, then the current
     * contents of voxelArray will be cleared prior to populating,
     * otherwise the new voxels will simply accumulate.
     */
    void PopulateVoxels(const cv::Mat& Cloud, const cv::Mat& Color,
                        VoxelArray& voxelArray, uint8_t flag = 1,
                        bool doErase = true);


    /**
     * This pure virtual member function implements the demo,
     * including any OpenGL output.  It implemented for each specific
     * demo in its respective derived class)
     *
     * @return Nothing is returned
     */
    virtual void DrawDemo(cv::Mat *Cloud, cv::Mat *Color)=0;


    /**
     * This callback is invoked periodically by FreeGlut to
     * capture new images, draw the demo, etc.  It calls this->DrawDemo().
     *
     * @return Nothing is returned
     */
    virtual void IdleFunc();


    /**
     * Get milliseconds since first call to this routine.
     *
     * @return The return value is elapsed time in milliseconds.
     */
    uint64_t
    ElapsedMS();


    // -------- OpenGL drawing functions for use by subclasses --------

    /**
     * Cloud coordinates have X pointing left, Y pointing down, and Z
     * pointing in the camera view direction, with the origin at the
     * camera.
     *
     * Our display coordinates have X pointing left, Y pointing up,
     * and Z pointing against the view direction, with the origin at
     * the camera, so that all visible Z coordinates are negative.
     *
     * This routine makes the transformation from Cloud Coordinates to
     * display coordinates.  At some time in the future we may skip
     * this step by simply updating the OpenGL ModelView Matrix.
     */
    void
    ConvertCloudCoordsToDisplay(
        int32_t CloudX, int32_t CloudY, int32_t CloudZ,
        VoxelArray const& VoxelArrayArg,
        float& DisplayX, float& DisplayY, float& DisplayZ);


    /**
     * Draw 2D text on the screen
     */
    void
    Draw2DScreenText(int XPos, int YPos, float Scale,
                     float R, float G, float B, const unsigned char *Text);


    /**
     * Draw a wireframe box.
     */
    void
    DrawBox(float X, float Y, float Z,
            float XDim, float YDim, float ZDim,
            float R, float G, float B);

    /**
     * Draw a 3D cube on the screen
     */
    void
    DrawCube(float X, float Y, float Z, float Dim, float R, float G, float B);


    /**
     * Draw a 3D wall on the screen made out of small dots.
     */
    void
    DrawMeshWall(float Spacing, float MinX, float MaxX,
                 float MinY, float MaxY, float MinZ, float MaxZ,
                 float R, float G, float B, float PtSize);


    /**
     * This function draws a voxel array to the OpenGL window.
     *
     * @param voxelArray This argument represents the voxel grid to be
     * drawn.
     */
    void
    DrawVoxels(const VoxelArray& voxelArray,
               float YMax = std::numeric_limits<float>::max(),
               float ZMax = -0.1);


    /* Initialize OpenGL Graphics */
    void
    initGL();


    // -------- Non-static functions to which --------
    // -------- GLUT callbacks will dispatch  --------


    /**
     * Handle housekeeping
     *
     * @return Nothing is returned
     */
    void GlutIdleWorker();


    // Handler for window re-size event. Called back when the window first appears and
    // whenever the window is re-sized with its new width and height.
    void GlutReshape(GLsizei width, GLsizei height);


    // Handles keyboard callback routine
    void GlutKey(unsigned char key, int x, int y);


    // Handles mouse click events
    void GlutMouseClick(int32_t Button, int32_t State, int32_t X, int32_t Y);


    // Handles mouse-down motion events
    void GlutMouseDownMotion(int32_t X, int32_t Y);


    // Handles mouse-wheel motion events
    void GlutMouseWheel(int32_t Button, int32_t Dir, int32_t X, int32_t Y);


    // -------- Static functions for use as GLUT callbacks --------

    /**
     * Callback which invokes non-static method GlueIdleWorker()
     *
     * @return Nothing is returned
     */
    static void
    GlutIdleWorkerCallback();


    /**
     * Callback which invokes non-static method GlutReshape()
     *
     * @return Nothing is returned
     */
    static void
    GlutReshapeCallback(GLsizei width, GLsizei height);


    /**
     * Callback which invokes non-static method GlutKey()
     *
     * @return Nothing is returned
     */
    static void
    GlutKeyCallback(unsigned char key, int x, int y);


    /**
     * Callback which invokes non-static method GlutMouseClick()
     *
     * @return Nothing is returned
     */
    static void
    GlutMouseClickCallback(int32_t Button, int32_t State, int32_t X, int32_t Y);


    /**
     * Callback which invokes non-static method GlutMouseDownMotion()
     *
     * @return Nothing is returned
     */
    static void
    GlutMouseDownMotionCallback(int32_t X, int32_t Y);


    /**
     * Callback which invokes non-static method GlutMouseWheel()
     *
     * @return Nothing is returned
     */
    static void
    GlutMouseWheelCallback(int32_t Button, int32_t Dir, int32_t X, int32_t Y);


    // ---- Data members ----

    BaseMultiSenseWrapper *MultiSense;

    // Setup for multisense
    int32_t ImgCols, ImgRows;
    float CurGain;
    float CurFPS;
    int32_t CurMaxExp;
    uint32_t CurDisparities;
    float CurExpThresh;
    uint32_t CurExpDecay;
    float CurPostFilt;

    // State to enable ElapsedMS() functionality.
    uint64_t ElapsedMSInitialTime;

    // Setup for display
    float GlutFOV;
    const char * title;
    float EyeX, EyeY, EyeZ;
    float ViewX, ViewY, ViewZ;
    float ViewYaw;
    float ViewPitch;
    float CloudGain;
    double WinHeight, WinWidth;
    float CloudPtSize;
    float XClipping;
    float YClipping;
    float ZClipping;
    int32_t DisplayMode;

    // Variables related to user interaction.
    int32_t LeftMouseDown;
    int32_t RightMouseDown;
    int32_t MousePrevX, MousePrevY;

};

#endif /* #ifndef CRL_CLOUDDEMO_H */
