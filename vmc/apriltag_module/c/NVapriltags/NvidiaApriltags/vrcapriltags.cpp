 #include <opencv2/core/version.hpp>
  
 #if CV_MAJOR_VERSION >= 3
 #    include <opencv2/imgcodecs.hpp>
 #else
 #    include <opencv2/highgui/highgui.hpp>
 #endif

#include <iostream>
#include "nvapriltags/nvAprilTags.h"
#include "cuda.h"
#include "cuda_runtime.h"
#include <opencv2/opencv.hpp>
#include <chrono>

#include <opencv2/imgproc/imgproc.hpp>
#include <vpi/OpenCVInterop.hpp>

#include <string.h> // for basename(3) that doesn't modify its argument
#include <unistd.h> // for getopt
#include <vpi/Image.h>
#include <vpi/LensDistortionModels.h>
#include <vpi/Status.h>
#include <vpi/Stream.h>
#include <vpi/algo/ConvertImageFormat.h>
#include <vpi/algo/Remap.h>

 #include <sstream>


 #define CHECK_STATUS(STMT)                                    \
     do                                                        \
     {                                                         \
         VPIStatus status = (STMT);                            \
         if (status != VPI_SUCCESS)                            \
         {                                                     \
             char buffer[VPI_MAX_STATUS_MESSAGE_LENGTH];       \
             vpiGetLastStatusMessage(buffer, sizeof(buffer));  \
             std::ostringstream ss;                            \
             ss << vpiStatusGetName(status) << ": " << buffer; \
             throw std::runtime_error(ss.str());               \
         }                                                     \
     } while (0);

    //g++ -I/usr/include/opencv4/ -I/usr/local/cuda-10.2/include -I/opt/nvidia/vpi/include/vpi  -I/opt/nvidia/vpi1/include/vpi -L./lib_aarch64_jetpack44 -L/usr/local/cuda-10.2/lib64  -L/opt/nvidia/vpi1/lib64 -L/opt/nvidia/vpi/lib helloworld.cpp  -lapril_tagging -lcudart -lcuda -lcublas -lopencv_core -lopencv_videoio -lopencv_imgproc -lopencv_highgui -lnvvpi

// copy from https://github.com/NVIDIA-AI-IOT/ros2-nvapriltags/blob/main/src/AprilTagNode.cpp

struct AprilTagsImpl {
    // Handle used to interface with the stereo library.
    nvAprilTagsHandle april_tags_handle = nullptr;
    // Camera intrinsics
    nvAprilTagsCameraIntrinsics_t cam_intrinsics;

    // Output vector of detected Tags
    std::vector<nvAprilTagsID_t> tags;

    // CUDA stream
    cudaStream_t main_stream = {};

    // CUDA buffers to store the input image.
    nvAprilTagsImageInput_t input_image;

    // CUDA memory buffer container for RGBA images.
    uchar4 *input_image_buffer = nullptr;

    // Size of image buffer
    size_t input_image_buffer_size = 0;

    int max_tags;

    void initialize(const uint32_t width,
                    const uint32_t height, 
                    const size_t image_buffer_size,
                    const size_t pitch_bytes,
                    const float fx, const float fy, const float cx, const float cy,
                    float tag_edge_size_, int max_tags_) {
        assert(!april_tags_handle), "Already initialized.";

        // Get camera intrinsics
        cam_intrinsics = {fx, fy, cx, cy};

        // Create AprilTags detector instance and get handle
        const int error = nvCreateAprilTagsDetector(
                &april_tags_handle, width, height, nvAprilTagsFamily::NVAT_TAG36H11,
                &cam_intrinsics, tag_edge_size_);
        if (error != 0) {
            throw std::runtime_error(
                    "Failed to create NV April Tags detector (error code " +
                    std::to_string(error) + ")");
        }

        // Create stream for detection
        cudaStreamCreate(&main_stream);

        // Allocate the output vector to contain detected AprilTags.
        tags.resize(max_tags_);
        max_tags = max_tags_;
        // Setup input image CUDA buffer.
        const cudaError_t cuda_error =
                cudaMalloc(&input_image_buffer, image_buffer_size);
        if (cuda_error != cudaSuccess) {
            throw std::runtime_error("Could not allocate CUDA memory (error code " +
                                     std::to_string(cuda_error) + ")");
        }

        // Setup input image.
        input_image_buffer_size = image_buffer_size;
        input_image.width = width;
        input_image.height = height;
        input_image.dev_ptr = reinterpret_cast<uchar4 *>(input_image_buffer);
        input_image.pitch = pitch_bytes;
    }

    ~AprilTagsImpl() {
        if (april_tags_handle != nullptr) {
            cudaStreamDestroy(main_stream);
            nvAprilTagsDestroy(april_tags_handle);
            cudaFree(input_image_buffer);
        }
    }
};




int main() {

    //############################################# SETUP VIDEO CAPTURE ##################################################################################################
    cv::VideoCapture capture("nvarguscamerasrc ! video/x-raw(memory:NVMM), width=1280, height=720,format=NV12, framerate=60/1 ! nvvidconv ! video/x-raw,format=BGRx !  videoconvert ! videorate ! video/x-raw,format=BGR,framerate=10/1 ! appsink", cv::CAP_GSTREAMER);
    std::cout<<"made it past cap device"<<std::endl;
    int width = 1280;
    int height = 720;

    float fx = 784.0756786399139;
    float fy = 784.9009527658286;
    float ppx = 677.124825443364;
    float ppy = 385.33983488708003;

    cv::Mat frame;
    cv::Mat img_rgba8;

    capture.read(frame);
    cv::cvtColor(frame, img_rgba8, cv::COLOR_BGR2RGBA);
    
    auto *impl_ = new AprilTagsImpl();
    impl_->initialize(img_rgba8.cols, img_rgba8.rows,
                      img_rgba8.total() * img_rgba8.elemSize(),  img_rgba8.step,
                      fx,fy,ppx,ppy, //camera params
                      17.4, //tag edge length
                      6); //max number of tags 

    //###################################################################### V P I ####################################################################

    // VPI objects that will be used
     VPIStream stream = NULL;
     VPIPayload remap = NULL;
     VPIImage tmpIn = NULL, tmpOut = NULL;
     VPIImage vimg = nullptr;

    // Camera intrinsic parameters, initially identity (will be estimated by calibration process).
    using Mat3     = cv::Matx<double, 3, 3>;
    Mat3 camMatrix = Mat3::eye();

    camMatrix(0,0) = (double)fx;
    camMatrix(1,1) = (double)fy;
    camMatrix(0,2) = (double)ppx;
    camMatrix(1,2) = (double)ppy;
  
    // Allocate a dense map.
    VPIWarpMap map            = {};
    map.grid.numHorizRegions  = 1;
    map.grid.numVertRegions   = 1;
    map.grid.regionWidth[0]   = img_rgba8.cols;
    map.grid.regionHeight[0]  = img_rgba8.rows;
    map.grid.horizInterval[0] = 1;
    map.grid.vertInterval[0]  = 1;
    CHECK_STATUS(vpiWarpMapAllocData(&map));

    // Initialize the fisheye lens model with the coefficients given by calibration procedure.
    VPIFisheyeLensDistortionModel distModel = {};
    distModel.mapping                       = VPI_FISHEYE_EQUIDISTANT;
    distModel.k1                            = -0.013826167055651659;
    distModel.k2                            = -0.11999744016996756;
    distModel.k3                            = 0.2825695466585381;
    distModel.k4                            = -0.22616481734332383;

    // Fill up the camera intrinsic parameters given by camera calibration procedure.
    VPICameraIntrinsic K;
    for (int i = 0; i < 2; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            K[i][j] = camMatrix(i, j);
        }
    }

    // Camera extrinsics is be identity.
    VPICameraExtrinsic X = {};
    X[0][0] = X[1][1] = X[2][2] = 1;

    // Generate a warp map to undistort an image taken from fisheye lens with
    // given parameters calculated above.
    vpiWarpMapGenerateFromFisheyeLensDistortionModel(K, X, K, &distModel, &map);

    // Create the Remap payload for undistortion given the map generated above.
    CHECK_STATUS(vpiCreateRemap(VPI_BACKEND_CUDA, &map, &remap));

    // Now that the remap payload is created, we can destroy the warp map.
    vpiWarpMapFreeData(&map);

    // Create a stream where operations will take place. We're using CUDA
    // processing.
    CHECK_STATUS(vpiStreamCreate(VPI_BACKEND_CUDA, &stream));

    // Temporary input and output images in NV12 format.
    CHECK_STATUS(vpiImageCreate(1280, 720, VPI_IMAGE_FORMAT_NV12_ER, 0, &tmpIn));
    CHECK_STATUS(vpiImageCreate(1280, 720, VPI_IMAGE_FORMAT_NV12_ER, 0, &tmpOut));



    //################################################################### MAIN LOOP ##########################################################################################
    while (capture.isOpened())
    {

        auto start = std::chrono::system_clock::now();

        capture.read(frame);
        ///////////////////////////////// UNDISTORT THE FRAME ////////////////////////////////////////////
        // Wrap it into a VPIImage
        if (vimg == nullptr)
        {
            // Now create a VPIImage that wraps it.
            CHECK_STATUS(vpiImageCreateOpenCVMatWrapper(frame, 0, &vimg));
        }
        else
        {
            CHECK_STATUS(vpiImageSetWrappedOpenCVMat(vimg, frame));
        }

        // Convert BGR -> NV12
        CHECK_STATUS(vpiSubmitConvertImageFormat(stream, VPI_BACKEND_CUDA, vimg, tmpIn, NULL));

        // Undistorts the input image.
        CHECK_STATUS(vpiSubmitRemap(stream, VPI_BACKEND_CUDA, remap, tmpIn, tmpOut, VPI_INTERP_CATMULL_ROM,
                                    VPI_BORDER_ZERO, 0));

        // Convert the result NV12 back to BGR, writing back to the input image.
        CHECK_STATUS(vpiSubmitConvertImageFormat(stream, VPI_BACKEND_CUDA, tmpOut, vimg, NULL));

        // Wait until conversion finishes.
        CHECK_STATUS(vpiStreamSync(stream));
        /////////////////////////////////////////////////////////////////////////////////////////////////////


        /////////////////////////// RUN THE APRILTAG DETECTOR ///////////////////////////////////////////////
        cv::cvtColor(frame, img_rgba8, cv::COLOR_BGR2RGBA);
        
        const cudaError_t cuda_error =
                cudaMemcpy(impl_->input_image_buffer,
                           (uchar4 *)img_rgba8.ptr<unsigned char>(0),
                           impl_->input_image_buffer_size,
                           cudaMemcpyHostToDevice);
        
        if (cuda_error != cudaSuccess) {
            throw std::runtime_error(
                    "Could not memcpy to device CUDA memory (error code " +
                    std::to_string(cuda_error) + ")");
        }

        uint32_t num_detections;
        const int error = nvAprilTagsDetect(
            impl_->april_tags_handle,
            &(impl_->input_image), 
            impl_->tags.data(),
            &num_detections, 
            impl_->max_tags, 
            impl_->main_stream
        );

    
        if (error != 0) {
            throw std::runtime_error("Failed to run AprilTags detector (error code " +
                                     std::to_string(error) + ")");
        }

        for (int i = 0; i < num_detections; i++) {
            const nvAprilTagsID_t &detection = impl_->tags[i];

            printf("ID: %d\n", detection.id);
            printf("X dist: %f\n", detection.translation[0]);
            printf("Y dist: %f\n", detection.translation[1]);
            printf("Z dist: %f\n", detection.translation[2]);
            
            // corners
            for (auto corner : detection.corners) {
               float x = corner.x;
               float y = corner.y;
               //printf("x: %f, y: %f\n", x,y);
            }
        }
        auto end = std::chrono::system_clock::now();
        int fps = int(1000 / ( std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() + 1));
        cv::putText(frame, "FPS: "+ std::to_string(fps), cv::Point(100,100),
                    cv::FONT_HERSHEY_PLAIN, 5, cv::Scalar(0xFF, 0xFF, 0), 2);        
        
        std::cout<<"num_detections: "<<num_detections<<std::endl;

        std::cout<<"FPS: "<<std::to_string(fps)<<std::endl;

        // cv::namedWindow("frame", 0);
        // cv::resizeWindow("frame", 1280,800);
        // cv::imshow("frame", frame);
        // if (cv::waitKey(10)==27)
        //     break;
    }
    delete(impl_);
    return 0;
}



    // while(capture.isOpened())
    // {
    //     std::cout<<"while"<<std::endl;
    //     capture.read(frame);
    //     cv::namedWindow("frame", 0);
    //     cv::resizeWindow("frame", 1280,720);
    //     cv::imshow("frame", frame);
    //     if (cv::waitKey(1)==27)
    //         break;
    // }