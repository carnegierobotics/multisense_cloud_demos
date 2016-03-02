/**
 * @file MultiSenseWrapper.h
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

#ifndef CRL_MULTISENSEWRAPPER_H
#define CRL_MULTISENSEWRAPPER_H

#include <LibMultiSense/MultiSenseChannel.hh>
#include <LibMultiSense/details/utility/Exception.hh>

#include <opencv2/core/core.hpp>



// This is a parent class for (at least) two inherited children.
// The first child is MultiSenseWrapper -- this talks to MultiSense hardware.
// The second child is PlayMultiSenseBundle which reads from a bundle on disk.
class BaseMultiSenseWrapper {

public:

    // Size of image we're setup to grab
    int m_grabbingRows;
    int m_grabbingCols;

    // Is the luma channel supported or does the unit have monochrome imagers?
    bool m_chromaSupported;

protected:
    // Data members for rectifying images and reprojecting disparities.
    cv::Mat m_leftCalibrationMapX;
    cv::Mat m_leftCalibrationMapY;
    cv::Mat m_rightCalibrationMapX;
    cv::Mat m_rightCalibrationMapY;
    cv::Mat m_qMatrix;

    // Data members for interacting with and controlling the sensor.
    int m_sensorRows;
    int m_sensorCols;

    float curFPS;


public:


    // Constructor
    BaseMultiSenseWrapper();

    // Destructor.
    virtual ~BaseMultiSenseWrapper();

    // ---- Functions for getting access to sensor data ----

    // Returns the most recent point cloud as a matrix of cv::Vec3f.
    // If no point cloud data is available, the returned cv:Mat will
    // be empty.  Internally, this function calls member function
    // CopyDisparity(), and the result of that call is returned
    // through reference argument disparityMat.
    virtual cv::Mat CopyCloud(cv::Mat& disparityMat) = 0;

    // Returns the most recent disparity image as a CV_32FC1 cv::Mat
    // representing disparity in pixels.  If no disparity data is
    // available, the returned cv:Mat will be empty.
    virtual cv::Mat CopyDisparity() = 0;

    // Returns the most recent unrectified luma image as a CV_8UC1
    // cv::Mat.  If no luma data is available, the returned cv:Mat
    // will be empty.
    virtual cv::Mat CopyLeftLuma() = 0;

    // Returns the most recent rectified color image as a CV_8UC3
    // cv::Mat.  If no color image data is available, the returned
    // cv:Mat will be empty.
    virtual cv::Mat CopyLeftRectifiedRGB() = 0;

    // Request calibration info from MultiSense unit.
    virtual void GetCalibration(float LeftM[3][3], float LeftD[8],
                                float LeftR[3][3], float LeftP[3][4],
                                float RightM[3][3], float RightD[8],
                                float RightR[3][3], float RightP[3][4]) = 0;

    // ---- Functions for turning data streams on and off ----

    // This asks the Sensor Pod to start sending the disparity stream
    virtual void StartDisparityStream(){};

    // This asks the Sensor Pod to start sending the chroma_left stream
    virtual void StartChromaLeftStream(){};

    // This asks the Sensor Pod to start sending the luma_left stream
    virtual void StartLumaLeftStream(){};

    // This asks the Sensor Pod to stop sending the disparity stream
    virtual void StopDisparityStream(){};

    // This asks the Sensor Pod to stop sending the chroma_left stream
    virtual void StopChromaLeftStream(){};

    // This asks the Sensor Pod to stop sending the luma_left stream
    virtual void StopLumaLeftStream(){};


    // ---- Functions for controlling sensor parameters ----

    // Set number of stereo disparities
    virtual void SetDisparities(uint32_t){};

    // Set autoexposure threshold
    virtual void SetExpThresh(float){};

    // Set autoexposure decay
    virtual void SetExpDecay(uint32_t){};

    // Set camera frames per second
    virtual void SetFPS(float){};

    // Set camera gain
    virtual void SetGain(float){};

    // Set MAX exposure time in milliseconds
    virtual void SetMaxExp(int){};

    // Set strength of post filtering
    virtual void SetPostFilt(float){};




private:

};



/**
 ** This class wraps the libMultiSense C++ API to provide a simplified
 ** interface suitable for making demonstration code.
 **/
class MultiSenseWrapper: public BaseMultiSenseWrapper {

public:

    /**
     * Constructor establishes communications with a MultiSense unit.
     *
     * @param IP This argument specifies the IP address of the MultiSense unit.
     *
     * @param Cols Indicates the desired image width.
     *
     * @param Rows Indicates the desired image height.
     *
     * @param FPS Indicates the desired framerate.
     */

    // Here is where you can set the IP address and other parameters to start your MultiSense.
    // You should not have to get these dimensions perfect -- the software should choose the closest size.
    // Default factory IP address for multisense units is: "10.66.171.21"
    // Typical dimensions are 1024x512 or 2048x1024 (others are possible).
    MultiSenseWrapper(const std::string& IP = "10.66.171.21",
                      int Cols = 1024, int Rows = 512, float FPS = 15.0);

    // Destructor.
    ~MultiSenseWrapper();


    // ---- Functions for turning data streams on and off ----

    // This asks the Sensor Pod to start sending the disparity stream
    virtual void StartDisparityStream();

    // This asks the Sensor Pod to start sending the chroma_left stream
    virtual void StartChromaLeftStream();

    // This asks the Sensor Pod to start sending the luma_left stream
    virtual void StartLumaLeftStream();

    // This asks the Sensor Pod to stop sending the disparity stream
    virtual void StopDisparityStream();

    // This asks the Sensor Pod to stop sending the chroma_left stream
    virtual void StopChromaLeftStream();

    // This asks the Sensor Pod to stop sending the luma_left stream
    virtual void StopLumaLeftStream();


    // ---- Functions for getting access to sensor data ----

    // Returns the most recent point cloud as a matrix of cv::Vec3f.
    // If no point cloud data is available, the returned cv:Mat will
    // be empty.  Internally, this function calls member function
    // CopyDisparity(), and the result of that call is returned
    // through reference argument disparityMat.
    virtual cv::Mat CopyCloud(cv::Mat& disparityMat);

    // Returns the most recent disparity image as a CV_32FC1 cv::Mat
    // representing disparity in pixels.  If no disparity data is
    // available, the returned cv:Mat will be empty.
    virtual cv::Mat CopyDisparity();

    // Returns the most recent unrectified luma image as a CV_8UC1
    // cv::Mat.  If no luma data is available, the returned cv:Mat
    // will be empty.
    virtual cv::Mat CopyLeftLuma();

    // Returns the most recent rectified color image as a CV_8UC3
    // cv::Mat.  If no color image data is available, the returned
    // cv:Mat will be empty.
    virtual cv::Mat CopyLeftRectifiedRGB();

    // Request calibration info from MultiSense unit.
    virtual void GetCalibration(float LeftM[3][3], float LeftD[8],
                                float LeftR[3][3], float LeftP[3][4],
                                float RightM[3][3], float RightD[8],
                                float RightR[3][3], float RightP[3][4]);


    // ---- Functions for controlling sensor parameters ----

    // Set number of stereo disparities
    virtual void SetDisparities(uint32_t);

    // Set autoexposure threshold
    virtual void SetExpThresh(float);

    // Set autoexposure decay
    virtual void SetExpDecay(uint32_t);

    // Set camera frames per second
    virtual void SetFPS(float);

    // Set camera gain
    virtual void SetGain(float);

    // Set MAX exposure time in milliseconds
    virtual void SetMaxExp(int);

    // Set strength of post filtering
    virtual void SetPostFilt(float);


private:

    // ---- General helper functions ----

    // Pick an image size that is supported by the sensor, and is as
    // close as possible to the requested image size.
    void selectDeviceMode(int32_t RequestedWidth, int32_t RequestedHeight,
                          crl::multisense::DataSource RequiredSources,
                          int32_t& SelectedWidth, int32_t& SelectedHeight);

    // Make RGB image from Luma image
     cv::Mat makeMonoImage(const cv::Mat& LumaImage);

    // Make RGB image from YCbCr image
     cv::Mat makeRGBImage(const cv::Mat& LumaImage, const cv::Mat& ChromaImage);

    // Read calibration from Multisense and setup Q matrix and
    // rectification transformation matrices
    void InitializeTransforms();


    // ---- Non-static functions that act as callbacks ----

    // Reserve the image buffer associated with sourceHeader, and copy
    // its metadata into targetHeader, so that copy*() functions above
    // can access libMultiSense's internal image buffers.
    void updateImage(const crl::multisense::image::Header& sourceHeader,
                     crl::multisense::image::Header& targetHeader,
                     pthread_mutex_t* mutexP,
                     void** bufferP);

    // Reserve the image buffer associated with header by calling
    // updateImage(), above.  If the data in header completes a new
    // matched pair of luma and chroma data, then copy those data into
    // the m_matched* member variables (below) for easy access from
    // the copy*() member functions (above).
    void updateLumaAndChroma(const crl::multisense::image::Header& header);


    // ---- Static callbacks that dispatch to nonstatic update functions ----

    static void disparityCallback(const crl::multisense::image::Header& header,
                                  void *userDataP);
    static void lumaChromaLeftCallback(const crl::multisense::image::Header& header,
                                       void *userDataP);


    // ---- Private data members ----


    // Data members for interacting with and controlling the sensor.
    crl::multisense::Channel* m_channelP;

    // Data members that maintain local pointers to image data that is
    // managed by libMultiSense.
    crl::multisense::image::Header m_chromaLeftHeader;
    crl::multisense::image::Header m_disparityHeader;
    crl::multisense::image::Header m_lumaLeftHeader;

    // Pointers used to coordinate buffer use with libMultiSense.
    // These pointers "reserve" the image data that backs each of the
    // header members above.
    void* m_chromaLeftBufferP;
    void* m_disparityBufferP;
    void* m_lumaLeftBufferP;

    // Additional headers and buffers that always reference data with
    // matching frame IDs.  This lets member function
    // copyLeftRectifiedRGB() access matching luma and chroma data
    // even if the most recent incoming luma header had a different
    // frameID from the most recent incoming chroma header.
    crl::multisense::image::Header m_matchedChromaLeftHeader;
    crl::multisense::image::Header m_matchedLumaLeftHeader;
    void* m_matchedChromaLeftBufferP;
    void* m_matchedLumaLeftBufferP;

    // Mutexes to coordinate image access between the callback
    // functions (above) and the copy*() functions (also above).
    pthread_mutex_t m_disparityMutex;
    pthread_mutex_t m_lumaAndChromaLeftMutex;
};




#endif /* #ifndef CRL_MULTISENSEWRAPPER_H */
