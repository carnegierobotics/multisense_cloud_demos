/**
 * @file MultiSenseWrapper.cpp
 *
 * MultiSense interface utility library.
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


#include <stdint.h>
#include <sys/time.h>
#include <limits>
#include <string>
#include <fstream>      // std::fstream
#include <iostream>     // std::cout

#include <errno.h>
#include <unistd.h>

#include <opencv2/opencv.hpp>

#include <MultiSenseWrapper.h>



using namespace std;
using namespace crl::multisense;
using namespace cv;

// Anonymous namespace for locally scoped symbols.
namespace {

// Simple pthread-based lock class with RAII semantics.
class ScopedLock
{
public:
    ScopedLock(pthread_mutex_t* mutexP)
        : m_mutexP(mutexP) { if(m_mutexP) {pthread_mutex_lock(m_mutexP);} }

    ~ScopedLock() { if(m_mutexP) {pthread_mutex_unlock(m_mutexP);} }

private:
    pthread_mutex_t *m_mutexP;
};

inline std::string createErrorString(const std::string &info, const Status &status)
{
    return info + Channel::statusString(status);
}


} // namespace



// Public functions for BaseMultiSenseWrapper

// Constructor
BaseMultiSenseWrapper::BaseMultiSenseWrapper()
    : m_grabbingRows(0),
      m_grabbingCols(0),
      m_chromaSupported(true),
      m_leftCalibrationMapX(),
      m_leftCalibrationMapY(),
      m_rightCalibrationMapX(),
      m_rightCalibrationMapY(),
      m_qMatrix(),
      m_sensorRows(0),
      m_sensorCols(0),
      curFPS(0.0)
{

}


// Destructor
BaseMultiSenseWrapper::~BaseMultiSenseWrapper()
{
}





// ---------------------------------------------------------------------------
// Here are the public functions for MultiSenseWrapper
// MultiSenseWrapper is a class which inherits from BaseMultiSenseWrapper
// These functions talk to live MultiSense hardware
// ---------------------------------------------------------------------------


// Setup communications with CRL sensor pod.
MultiSenseWrapper::
MultiSenseWrapper(const std::string& IP)
    : m_channelP(0),
      m_chromaLeftHeader(),
      m_disparityHeader(),
      m_lumaLeftHeader(),
      m_chromaLeftBufferP(0),
      m_disparityBufferP(0),
      m_lumaLeftBufferP(0),
      m_matchedChromaLeftHeader(),
      m_matchedLumaLeftHeader(),
      m_matchedChromaLeftBufferP(0),
      m_matchedLumaLeftBufferP(0),
      m_disparityMutex(),
      m_lumaAndChromaLeftMutex()
{
    std::string currentAddress = IP;


    // Set up control structures for coordinating threads.
    if (0 != pthread_mutex_init(&m_disparityMutex, NULL)) {
        std::runtime_error(std::string("pthread_mutex_init() failed: ")+ strerror(errno));
    }
    if (0 != pthread_mutex_init(&m_lumaAndChromaLeftMutex, NULL)) {
        std::runtime_error(std::string("pthread_mutex_init() failed: ") + strerror(errno));
    }

    // Initialize communications.
    m_channelP = Channel::Create(currentAddress);
    if (NULL == m_channelP) {
        cerr << "Could not start communications with MultiSense sensor.\n";
        cerr << "Check network connections and settings?\n";
        cerr << "Consult ConfigureNetwork.sh script for hints.\n";
        exit(1);
    }

    // Query version
    Status status;
    system::VersionInfo versionInfo;
    status = m_channelP->getVersionInfo(versionInfo);
    if (Status_Ok != status) {
        std::runtime_error("failed to query sensor version:" + status);
    }

    system::DeviceInfo deviceInfo;
    status = m_channelP->getDeviceInfo(deviceInfo);
    if (Status_Ok != status) {
        std::runtime_error("failed to query device info: " + status);
    }
    m_sensorRows = deviceInfo.imagerHeight;
    m_sensorCols = deviceInfo.imagerWidth;

    if (deviceInfo.imagerType == system::DeviceInfo::IMAGER_TYPE_CMV2000_GREY ||
        deviceInfo.imagerType == system::DeviceInfo::IMAGER_TYPE_CMV4000_GREY) {
        m_chromaSupported = false;
    }

    // Configure the sensor.
    image::Config cfg;
    status = m_channelP->getImageConfig(cfg);
    if (Status_Ok != status) {
        std::runtime_error("Failed to query image config: " + status);
    }

    cfg.setResolution(m_grabbingCols, m_grabbingRows);
    status = m_channelP->setImageConfig(cfg);
    if (Status_Ok != status) {
        std::runtime_error("Failed to configure sensor resolution and framerate\n");
    }

    // Change MTU
    if (Status_Ok != m_channelP->setMtu(7200))
        fprintf(stderr, "failed to set MTU to 7200\n");

    // Change trigger source
    status = m_channelP->setTriggerSource(Trigger_Internal);
    if (Status_Ok != status)
        fprintf(stderr, "Failed to set trigger source, Error %d\n", status);


    // Read calibration data and compute rectification maps.
    InitializeTransforms();

    // Initialize frameId's so image data can be copied properly
    // on startup. I.e. prevent the case where the image frame ids and
    // the matched frame ids are both 0.
    m_lumaLeftHeader.frameId = -1;
    m_chromaLeftHeader.frameId = -1;
    m_matchedLumaLeftHeader.frameId = -1;
    m_matchedChromaLeftHeader.frameId = -1;

    // Setup callbacks
    m_channelP->addIsolatedCallback(disparityCallback, Source_Disparity, this);
    m_channelP->addIsolatedCallback(lumaChromaLeftCallback, Source_Luma_Left | Source_Chroma_Left, this);

}


// Destructor.
MultiSenseWrapper::~MultiSenseWrapper()
{
    m_channelP->removeIsolatedCallback(disparityCallback);
    m_channelP->removeIsolatedCallback(lumaChromaLeftCallback);

    Channel::Destroy(m_channelP);
}


// This asks the Sensor Pod to start sending the chroma_left stream
void MultiSenseWrapper::StartChromaLeftStream()
{
    Status status;

    status = m_channelP->startStreams(Source_Chroma_Left);
    if (status != Status_Ok)
        std::runtime_error(createErrorString("Unable to start Source_Chroma_Left stream: ", status));
}


// This asks the Sensor Pod to start sending the disparity stream
void MultiSenseWrapper::StartDisparityStream()
{
    Status status;

    status = m_channelP->startStreams(Source_Disparity) != Status_Ok;
    if (status != Status_Ok)
        std::runtime_error(createErrorString("Unable to start Source_Disparity stream: ", status));
}


// This asks the Sensor Pod to start sending the luma_left stream
void MultiSenseWrapper::StartLumaLeftStream()
{
    Status status;

    status = m_channelP->startStreams(Source_Luma_Left);
    if (status != Status_Ok)
        std::runtime_error(createErrorString("Unable to start Source_Luma_Left stream: ", status));
}


// This asks the Sensor Pod to stop sending the chroma_left stream
void MultiSenseWrapper::StopChromaLeftStream()
{
    Status status;

    status = m_channelP->stopStreams(Source_Chroma_Left);
    if (status != Status_Ok)
        std::runtime_error(createErrorString("Unable to stop Source_Chroma_Left stream: ", status));
}


// This asks the Sensor Pod to stop sending the disparity stream
void MultiSenseWrapper::StopDisparityStream()
{
    Status status;

    status = m_channelP->stopStreams(Source_Disparity) != Status_Ok;
    if (status != Status_Ok)
        std::runtime_error(createErrorString("Unable to stop Source_Disparity stream: ", status));
}


// This asks the Sensor Pod to stop sending the luma_left stream
void MultiSenseWrapper::StopLumaLeftStream()
{
    Status status;

    status = m_channelP->stopStreams(Source_Luma_Left);
    if (status != Status_Ok)
        std::runtime_error(createErrorString("Unable to stop Source_Luma_Left stream: ", errno));
}


// Get calibration parameters from the camera
void MultiSenseWrapper::GetCalibration(float LeftM[3][3], float LeftD[8],
                                       float LeftR[3][3], float LeftP[3][4],
                                       float RightM[3][3], float RightD[8],
                                       float RightR[3][3], float RightP[3][4])
{
    image::Calibration Cal;
    int i, j;
    float XScale, YScale;

    // Calibration is for full-size image -- scale for our image size
    XScale = (float) m_grabbingCols/(float) m_sensorCols;
    YScale = (float) m_grabbingRows/(float) m_sensorRows;

    Status status = m_channelP->getImageCalibration(Cal);
    if (Status_Ok != status) {
        std::runtime_error(createErrorString("Failed to query image config: ", status));
    }

    // Scale for image size vs. imager size
    Cal.left.M[0][0] *= XScale;  Cal.left.M[1][1] *= YScale;
    Cal.left.M[0][2] *= XScale;  Cal.left.M[1][2] *= YScale;
    Cal.left.P[0][0] *= XScale;  Cal.left.P[1][1] *= YScale;
    Cal.left.P[0][2] *= XScale;  Cal.left.P[1][2] *= YScale;
    Cal.left.P[0][3] *= XScale;  Cal.left.P[1][3] *= YScale;

    // Put everything into OpenCV-friendly formats
    for (j = 0 ; j < 3 ; j++) {
        for (i = 0 ; i < 3 ; i++) {
            LeftM[j][i] =  Cal.left.M[j][i];
            RightM[j][i] = Cal.right.M[j][i];
            LeftR[j][i] =  Cal.left.R[j][i];
            RightR[j][i] = Cal.right.R[j][i];
        }
    }
    for (j = 0 ; j < 3 ; j++) {
        for (i = 0 ; i < 4 ; i++) {
            LeftP[j][i] =  Cal.left.P[j][i];
            RightP[j][i] = Cal.right.P[j][i];
        }
    }

    for (i = 0 ; i < 8 ; i++) {
        LeftD[i] = Cal.left.D[i];
        RightD[i] = Cal.right.D[i];
    }

    return;
}


Mat MultiSenseWrapper::CopyCloud(Mat& disparityMat)
{
    // Default constructor initializes to zero size.
    Mat cloudMat;

    disparityMat = this->CopyDisparity();
    if (!disparityMat.empty()) {
        reprojectImageTo3D(disparityMat, cloudMat, m_qMatrix, true);
    }
    return cloudMat;
}


Mat MultiSenseWrapper::CopyDisparity()
{
    // Lock access to the local data pointer so we don't read from it
    // while libMultiSense callbacks are writing.  The lock will be
    // released when when ScopedLock::~ScopedLock() is called on exit
    // from this function.
    ScopedLock lock(&(this->m_disparityMutex));

    // Make sure there's valid data to copy.
    Mat disparityFloatMat;
    if ((0 != m_disparityBufferP)) {

        // Construct a cv::Mat instance that references the disparity
        // data.  This line does not actually copy the data.  Variable
        // disparityMat exists only so that we can call its
        // convertTo() member function.
        image::Header const& header = this->m_disparityHeader;
        Mat disparityMat(header.height, header.width, CV_16UC1,
                         const_cast<void*>(header.imageDataP));

        // Convert to float, as promised to the calling context.
        disparityMat.convertTo(disparityFloatMat, CV_32FC1, 1.0/16.0);
    }

    // The returned Mat will have zero size if the test of
    // m_disparityBufferP failed above.
    return disparityFloatMat;
}


Mat MultiSenseWrapper::CopyLeftLuma()
{
    // Lock access to the local data pointers so we don't read from
    // them while libMultiSense callbacks are writing.  The lock will
    // be released when when ScopedLock::~ScopedLock() is called on
    // exit from this set of curly braces.
    ScopedLock lock(&(this->m_lumaAndChromaLeftMutex));

    // Is there data to copy?
    Mat lumaMat;
    if (0 != m_matchedLumaLeftBufferP) {
        // ...Yes.  Construct a cv::Mat instance that references the
        // luma data.  This line does not actually copy the data.
        image::Header const& lumaHeader   = this->m_matchedLumaLeftHeader;
        Mat lumaReference(lumaHeader.height, lumaHeader.width, CV_8UC1,
                          const_cast<void*>(lumaHeader.imageDataP));

        // Deep copy the data into lumaMat.
        lumaMat = lumaReference.clone();
    }

    // The returned Mat will have zero size if we weren't able to get
    // image data.
    return lumaMat;
}


Mat MultiSenseWrapper::CopyLeftRectifiedRGB()
{
    // Default constructor initializes to zero size.
    Mat rgbImage;
    Mat rectifiedRgbImage;

    {
        // Lock access to the local data pointers so we don't read from
        // them while libMultiSense callbacks are writing.  The lock will
        // be released when when ScopedLock::~ScopedLock() is called on
        // exit from this set of curly braces.
        ScopedLock lock(&(this->m_lumaAndChromaLeftMutex));

        if (m_chromaSupported) {
            // Is there data to copy?
            if ((0 != m_matchedLumaLeftBufferP) && (0 != m_matchedChromaLeftBufferP)) {

                // ...Yes.  Construct cv::Mat instances that reference the
                // luma and chroma data.  These lines do not actually copy the data.
                image::Header const& lumaHeader   = this->m_matchedLumaLeftHeader;
                image::Header const& chromaHeader = this->m_matchedChromaLeftHeader;
                Mat lumaMat(lumaHeader.height, lumaHeader.width, CV_8UC1,
                              const_cast<void*>(lumaHeader.imageDataP));
                Mat chromaMat(chromaHeader.height, chromaHeader.width, CV_8UC1,
                              const_cast<void*>(chromaHeader.imageDataP));

                // Create an unrectified color image.
                rgbImage = this->makeRGBImage(lumaMat, chromaMat);

            }
        } else {
            // Is there data to copy?
            if (0 != m_matchedLumaLeftBufferP) {
                // ...Yes.  Construct cv::Mat instances that reference the
                // luma data.  These lines do not actually copy the data.

                image::Header const& lumaHeader   = this->m_matchedLumaLeftHeader;
                Mat lumaMat(lumaHeader.height, lumaHeader.width, CV_8UC1,
                              const_cast<void*>(lumaHeader.imageDataP));

                // Create an unrectified "color" image (RGB channels will be greyscale)
                rgbImage = this->makeMonoImage(lumaMat);
            }
        }
    }  // Releases the lock on m_lumaAndChromaLeftMutex.

    // If we successfully generated a color image, rectify it now.
    if (!rgbImage.empty()) {

        Scalar OutlierColor = CV_RGB(0.0, 0.0, 0.0);
        remap(rgbImage, rectifiedRgbImage, this->m_leftCalibrationMapX, this->m_leftCalibrationMapY,
              INTER_LINEAR, BORDER_CONSTANT, OutlierColor);
    }

    // The returned Mat will have zero size if we weren't able to get
    // image data.
    return rectifiedRgbImage;
}


// Set stereo disparities
void MultiSenseWrapper::SetDisparities(uint32_t Disparities)
{
    image::Config cfg;
    Status status;

    status = m_channelP->getImageConfig(cfg);
    if (Status_Ok != status) {
        std::runtime_error(createErrorString("Failed to query image config: ", status));
    }

    cfg.setDisparities(Disparities);  // Can be 128 or 256
    status = m_channelP->setImageConfig(cfg);
    if (Status_Ok != status) {
        std::runtime_error(createErrorString("Failed to configure sensor number of disparities: ", status));
    }
}


// Set camera exposure threshold
void MultiSenseWrapper::SetExpThresh(float ExpThresh)
{
    image::Config cfg;
    Status status;

    status = m_channelP->getImageConfig(cfg);
    if (Status_Ok != status) {
        std::runtime_error(createErrorString("Failed to query image config: ", status));
    }

    cfg.setAutoExposureThresh(ExpThresh);  // Can be 0.0 -> 1.0 ?
    status = m_channelP->setImageConfig(cfg);
    if (Status_Ok != status) {
        std::runtime_error(createErrorString("Failed to configure sensor autoexposure threshold: ",  status));
    }
}



// Set camera exposure decay
void MultiSenseWrapper::SetExpDecay(uint32_t ExpDecay)
{
    image::Config cfg;
    Status status;

    status = m_channelP->getImageConfig(cfg);
    if (Status_Ok != status) {
        std::runtime_error(createErrorString("Failed to query image config: ", status));
    }

    cfg.setAutoExposureDecay(ExpDecay);  // seconds
    status = m_channelP->setImageConfig(cfg);
    if (Status_Ok != status) {
        std::runtime_error(createErrorString("Failed to configure sensor autoexposure decay: ", status));
    }
}


// Set camera frames per second
void MultiSenseWrapper::SetFPS(float FPS)
{
    image::Config cfg;
    // This routine also scales the camera values.
    Status status;

    curFPS = FPS;

    status = m_channelP->getImageConfig(cfg);
    if (Status_Ok != status) {
        std::runtime_error(createErrorString("Failed to query image config: ", status));
    }

    cfg.setFps(FPS);  // Can be 1.0 -> 30.0
    status = m_channelP->setImageConfig(cfg);
    if (Status_Ok != status) {
        std::runtime_error(createErrorString("Failed to configure sensor framerate: ", status));
    }
}


// Set camera gains
void MultiSenseWrapper::SetGain(float Gain)
{
    image::Config cfg;
    Status status;

    status = m_channelP->getImageConfig(cfg);
    if (Status_Ok != status) {
        std::runtime_error(createErrorString("Failed to query image config: ", status));
    }

    cfg.setGain(Gain);  // Can be 0.0 -> 6.0 ?
    status = m_channelP->setImageConfig(cfg);
    if (Status_Ok != status) {
        std::runtime_error(createErrorString("Failed to configure sensor gain: ", status));
    }
}


// Set camera maximum autoexposure milliseconds
void MultiSenseWrapper::SetMaxExp(int MaxExpMilS)
{
    image::Config cfg;
    Status status;

    status = m_channelP->getImageConfig(cfg);
    if (Status_Ok != status) {
        std::runtime_error(createErrorString("Failed to query image config: ", status));
    }

    cfg.setAutoExposureMax(MaxExpMilS);
    status = m_channelP->setImageConfig(cfg);
    if (Status_Ok != status) {
        std::runtime_error(createErrorString("Failed to configure sensor autoexposure: ", status));
    }
}


// Set stereo post filter strength
void MultiSenseWrapper::SetPostFilt(float PostFilt)
{
    image::Config cfg;
    Status status;

    status = m_channelP->getImageConfig(cfg);
    if (Status_Ok != status) {
        std::runtime_error(createErrorString("Failed to query image config: ", status));
    }

    cfg.setStereoPostFilterStrength(PostFilt);  // Can be 0.0 -> 1.0
    status = m_channelP->setImageConfig(cfg);
    if (Status_Ok != status) {
        std::runtime_error(createErrorString("Failed to configure sensor post filter level: ", status));
    }
}



// ---------------------------------------------------------------------------
// Here are the PRIVATE functions for MultiSenseWrapper
// MultiSenseWrapper is a class which inherits from BaseMultiSenseWrapper
// These functions talk to live MultiSense hardware
// ---------------------------------------------------------------------------

Mat MultiSenseWrapper::makeMonoImage(const Mat& LumaImage)
{
    const uint32_t height    = LumaImage.rows;
    const uint32_t width     = LumaImage.cols;

    const uint8_t *lumaP     = reinterpret_cast<const uint8_t*>(LumaImage.ptr(0));

    Mat            GreyImage  (height, width, CV_8UC3);
    uint8_t       *rgbP      = reinterpret_cast<uint8_t*>(GreyImage.ptr(0));
    const uint32_t rgbStride = width * 3;

    // Copied from the MultiSense ROS driver
    for(uint32_t y=0; y<height; y++) {
        for(uint32_t x=0; x<width; x++) {

            const uint32_t lumaOffset   = (y * width) + x;
            const uint32_t rgbOffset    = (y * rgbStride) + (3 * x);

            rgbP[rgbOffset + 2] = lumaP[lumaOffset];
            rgbP[rgbOffset + 1] = lumaP[lumaOffset];
            rgbP[rgbOffset + 0] = lumaP[lumaOffset];
        }
    }

    return GreyImage;
}

// Make RGB image from YCbCr image
Mat MultiSenseWrapper::makeRGBImage(const Mat& LumaImage,
        const Mat& ChromaImage)
{
    const uint32_t height    = LumaImage.rows;
    const uint32_t width     = LumaImage.cols;

    const uint8_t *lumaP     = reinterpret_cast<const uint8_t*>(LumaImage.ptr(0));
    const uint8_t *chromaP   = reinterpret_cast<const uint8_t*>(ChromaImage.ptr(0));

    Mat            ColorImage  (height, width, CV_8UC3);
    uint8_t       *rgbP      = reinterpret_cast<uint8_t*>(ColorImage.ptr(0));
    const uint32_t rgbStride = width * 3;

    // Copied from the MultiSense ROS driver
    for(uint32_t y=0; y<height; y++) {
        for(uint32_t x=0; x<width; x++) {

            const uint32_t lumaOffset   = (y * width) + x;
            const uint32_t chromaOffset = 2 * (((y/2) * (width/2)) + (x/2));

            const float px_y  =
                static_cast<float>(lumaP[lumaOffset]);
            const float px_cb =
                static_cast<float>(chromaP[chromaOffset+0]) - 128.0f;
            const float px_cr =
                static_cast<float>(chromaP[chromaOffset+1]) - 128.0f;

            float px_r  = px_y + 1.402f   * px_cr;
            float px_g  = px_y - 0.34414f * px_cb - 0.71414f * px_cr;
            float px_b  = px_y + 1.772f   * px_cb;

            if (px_r < 0.0f)        px_r = 0.0f;
            else if (px_r > 255.0f) px_r = 255.0f;
            if (px_g < 0.0f)        px_g = 0.0f;
            else if (px_g > 255.0f) px_g = 255.0f;
            if (px_b < 0.0f)        px_b = 0.0f;
            else if (px_b > 255.0f) px_b = 255.0f;

            const uint32_t rgbOffset = (y * rgbStride) + (3 * x);

            rgbP[rgbOffset + 2] = static_cast<uint8_t>(px_r);
            rgbP[rgbOffset + 1] = static_cast<uint8_t>(px_g);
            rgbP[rgbOffset + 0] = static_cast<uint8_t>(px_b);
        }
    }

    return ColorImage;
}


// Pick an image size that is supported by the sensor, and is as
// close as possible to the requested image size.
void MultiSenseWrapper::selectDeviceMode(int32_t RequestedWidth,
                                         int32_t RequestedHeight,
                                         DataSource RequiredSources,
                                         int32_t& SelectedWidth,
                                         int32_t& SelectedHeight)
{
    // Query the sensor to see what resolutions are supported.
    std::vector<system::DeviceMode> modeVector;
    Status status = m_channelP->getDeviceModes(modeVector);
    if (Status_Ok != status) {
        std::runtime_error(createErrorString("Failed to query device modes: ", status));
    }

    // Check each mode in turn, and pick the one that's closest to the
    // requested image size.
    typedef std::vector<system::DeviceMode>::iterator Iter;
    int32_t bestResidual = -1;
    system::DeviceMode bestMode;
    for(Iter iter = modeVector.begin(); iter != modeVector.end(); ++iter) {

        // Only consider modes that support the required data streams.
        if((iter->supportedDataSources & RequiredSources) == RequiredSources) {
            int32_t modeWidth = static_cast<int32_t>(iter->width);
            int32_t modeHeight = static_cast<int32_t>(iter->height);
            int32_t residual = (std::abs(RequestedWidth - modeWidth)
                                + std::abs(RequestedHeight - modeHeight));
            if ((bestResidual < 0) || residual < bestResidual) {
                bestResidual = residual;
                bestMode = *iter;
            }
        }
    }

    if(bestResidual < 0) {
        std::runtime_error("Device does not support the required data sources "
                      "(left luma, left chroma, and disparity)");
    }

    SelectedWidth = bestMode.width;
    SelectedHeight = bestMode.height;
}


// Load calibration information from S-7 camera and calculate
// transform matrices
void MultiSenseWrapper::InitializeTransforms()
{
    float LeftM[3][3], LeftD[8], LeftR[3][3], LeftP[3][4];
    float RightM[3][3], RightD[8], RightR[3][3], RightP[3][4];

    cv::Mat M1, D1, M2, D2;
    cv::Mat R, T;
    cv::Mat R1, R2, P1, P2;

    int i, j;

    image::Config c;

    Status status;

    status = m_channelP->getImageConfig(c);
    if (status != Status_Ok) {
        std::runtime_error("Failed to getImageConfig() in "
                      "MultiSenseWrapper::InitializeTransforms()");
    }

    uint32_t ImgRows = c.height();
    uint32_t ImgCols = c.width();

    m_leftCalibrationMapX = Mat(ImgRows, ImgCols, CV_32F);
    m_leftCalibrationMapY = Mat(ImgRows, ImgCols, CV_32F);
    m_rightCalibrationMapX = Mat(ImgRows, ImgCols, CV_32F);
    m_rightCalibrationMapY = Mat(ImgRows, ImgCols, CV_32F);

    // Allocate space for matricies
    // Mat takes Rows, Cols
    // One-D matricies are setup with 1 row and N columns
    M1 = Mat(3, 3, CV_32F);
    M2 = Mat(3, 3, CV_32F);
    D1 = Mat(1, 8, CV_32F);
    D2 = Mat(1, 8, CV_32F);
    R1 = Mat(3, 3, CV_32F);
    R2 = Mat(3, 3, CV_32F);
    P1 = Mat(3, 4, CV_32F);
    P2 = Mat(3, 4, CV_32F);
    T = Mat(1, 3, CV_32F);
    m_qMatrix = Mat(4, 4, CV_32F, 0.0);

    // Load values from camera
    // This routine also scales the camera values.
    MultiSenseWrapper::GetCalibration(LeftM, LeftD, LeftR, LeftP,
                                      RightM, RightD, RightR, RightP);

    // Copy camera values into cvMats
    for (j = 0 ; j < 3 ; j++) {
        for (i = 0 ; i < 3 ; i++) {
            M1.at<float>(j, i) = LeftM[j][i];
            M2.at<float>(j, i) = RightM[j][i];
            R1.at<float>(j, i) = LeftR[j][i];
            R2.at<float>(j, i) = RightR[j][i];
        }
    }
    for (j = 0 ; j < 3 ; j++) {
        for (i = 0 ; i < 4 ; i++) {
            P1.at<float>(j, i) = LeftP[j][i];
            P2.at<float>(j, i) = RightP[j][i];
        }
    }
    for (i = 0 ; i < 8 ; i++) {
        // OpenCV only wants 5 elements for D1 but camera returns 8
        D1.at<float>(0, i) = LeftD[i];
        D2.at<float>(0, i) = RightD[i];
    }

    //
    // Compute the Q reprojection matrix for non square pixels. Setting
    // fx = fy will result in the traditional Q matrix
    m_qMatrix.at<float>(0, 0) =  c.fy() * c.tx();
    m_qMatrix.at<float>(1, 1) =  c.fx() * c.tx();
    m_qMatrix.at<float>(0, 3) = -c.fy() * c.cx() * c.tx();
    m_qMatrix.at<float>(1, 3) = -c.fx() * c.cy() * c.tx();
    m_qMatrix.at<float>(2, 3) =  c.fx() * c.fy() * c.tx();
    m_qMatrix.at<float>(3, 2) = -c.fy();
    m_qMatrix.at<float>(3, 3) =  c.fy() * (0.0);

    // Compute rectification maps
    initUndistortRectifyMap(M1, D1, R1, P1, m_leftCalibrationMapX.size(), CV_32FC1,
                            m_leftCalibrationMapX, m_leftCalibrationMapY);
    initUndistortRectifyMap(M2, D2, R2, P2, m_rightCalibrationMapX.size(), CV_32FC1,
                            m_rightCalibrationMapX, m_rightCalibrationMapY);
}


void MultiSenseWrapper::updateImage(const image::Header& sourceHeader,
                                    image::Header& targetHeader,
                                    pthread_mutex_t* mutexP,
                                    void** bufferP)
{
    // Lock access to the local data pointer so we don't clobber it
    // while client code is reading from it.  The lock will be
    // released when when ScopedLock::~ScopedLock() is called on exit
    // from this function.
    ScopedLock lock(mutexP);

    // Return any previously reserved image data to the libMultiSense
    // library.
    if(0 != (*bufferP)) {
        this->m_channelP->releaseCallbackBuffer(*bufferP);
    }

    // Reserve the data that's backing the new image header.
    *bufferP = this->m_channelP->reserveCallbackBuffer();

    // And make a local copy, so that client code can access the image
    // later.
    targetHeader = sourceHeader;
}


void MultiSenseWrapper::updateLumaAndChroma(const image::Header& header)
{
    // Lock access to the local data pointers so we don't clobber them
    // while client code is reading from them, or other callbacks are
    // writing to them.
    ScopedLock lock(&(this->m_lumaAndChromaLeftMutex));


    // Copy a reference to the data that was just passed in.  This
    // lets us buffer the incoming luma or chroma data until matching
    // chroma or luma data is available.  Passing 0 as the mutex
    // pointer prevents updateImage() from trying to lock
    // m_lumaAndChromaLeftMutex again, which would result in a
    // deadlock.
    if(Source_Luma_Left == header.source) {
        this->updateImage(header, this->m_lumaLeftHeader,
                          0, &this->m_lumaLeftBufferP);
    } else if (Source_Chroma_Left == header.source) {
        this->updateImage(header, this->m_chromaLeftHeader,
                          0, &this->m_chromaLeftBufferP);
    } else {
        std::runtime_error("Unexpected source type in "
                           "MultiSenseWrapper::updateLumaAndChroma()");
    }

    if (m_chromaSupported) {

        // Now that we've buffered a reference to the incoming data
        // reference, check to see if the incoming data makes a complete
        // set of luma/chroma data for a new image.  If so, we'll copy it
        // to secondary storage.
        if (this->m_lumaLeftHeader.frameId == this->m_chromaLeftHeader.frameId) {

            // We have a matching luma/chroma pair.  Sanity check to make
            // sure we're not getting repeat frame IDs, or double-copying
            // somewhere.
            if((this->m_lumaLeftHeader.frameId
                == this->m_matchedLumaLeftHeader.frameId)
               || (this->m_chromaLeftHeader.frameId
                == this->m_matchedChromaLeftHeader.frameId)) {


                // Sanity check: this should never happen.
                std::runtime_error(
                    "Logic exception in MultiSenseWrapper::updateLumaAndChroma():\n"
                    "frame IDs appear to be double-counted.\n");
            }

            // Release any previously saved (and now obselete) luma/chroma data.
            if (0 != this->m_matchedLumaLeftBufferP) {
                this->m_channelP->releaseCallbackBuffer(
                    this->m_matchedLumaLeftBufferP);
            }
            if (0 != this->m_matchedChromaLeftBufferP) {
                this->m_channelP->releaseCallbackBuffer(
                    this->m_matchedChromaLeftBufferP);
            }

            // Transfer the new luma and chroma component into secondary
            // storage, where they will be available to the calling
            // context.  save the matched pair.
            this->m_matchedLumaLeftBufferP = this->m_lumaLeftBufferP;
            this->m_matchedLumaLeftHeader = this->m_lumaLeftHeader;
            this->m_lumaLeftBufferP = 0;

            this->m_matchedChromaLeftBufferP = this->m_chromaLeftBufferP;
            this->m_matchedChromaLeftHeader = this->m_chromaLeftHeader;
            this->m_chromaLeftBufferP = 0;

        }
    } else {

        // Release any previously saved (and now obsolete) luma data.
        if (0 != this->m_matchedLumaLeftBufferP) {
            this->m_channelP->releaseCallbackBuffer(
                this->m_matchedLumaLeftBufferP);
        }

        // Unit is monochrome, so all we need to use is transfer the new luma component
        // into secondary storage, where it will be available to the calling context.
        this->m_matchedLumaLeftBufferP = this->m_lumaLeftBufferP;
        this->m_matchedLumaLeftHeader = this->m_lumaLeftHeader;
        this->m_lumaLeftBufferP = 0;
    }
}


// Calls non-static method updateImage()
void MultiSenseWrapper::disparityCallback(const image::Header& header,
                                          void *userDataP)
{
    MultiSenseWrapper* pod = (MultiSenseWrapper *)userDataP;
    pod->updateImage(header, pod->m_disparityHeader,
                     &pod->m_disparityMutex, &pod->m_disparityBufferP);
    ScopedLock lock(&(pod->m_disparityMutex));
}


// Calls non-static method updateLumaAndChroma()
void MultiSenseWrapper::lumaChromaLeftCallback(const image::Header& header,
                                         void *userDataP)
{
    MultiSenseWrapper* pod = (MultiSenseWrapper *)userDataP;
    pod->updateLumaAndChroma(header);
}







