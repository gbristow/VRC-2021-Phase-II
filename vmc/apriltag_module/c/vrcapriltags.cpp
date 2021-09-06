#include "apriltags.hpp"
#include "undistort.hpp"
#include <string.h> // for basename(3) that doesn't modify its argument
#include <unistd.h> // for getopt
#include <sstream>

#include <nlohmann/json.hpp>

// for convenience
using json = nlohmann::json;


int main() {

    //############################################# SETUP VIDEO CAPTURE ##################################################################################################
    cv::VideoCapture capture("nvarguscamerasrc ! video/x-raw(memory:NVMM), width=1280, height=720,format=NV12, framerate=60/1 ! nvvidconv ! video/x-raw,format=BGRx !  videoconvert ! videorate ! video/x-raw,format=BGR,framerate=5/1 ! appsink", cv::CAP_GSTREAMER);
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
                      0.174, //tag edge length
                      6); //max number of tags 




    //################################################################### MAIN LOOP ##########################################################################################
    while (capture.isOpened())
    {

        auto start = std::chrono::system_clock::now();

        capture.read(frame);
        undistort_frame(frame);


        /////////////////////////// RUN THE APRILTAG DETECTOR ///////////////////////////////////////////////

        //convert the frame to rgba
        cv::cvtColor(frame, img_rgba8, cv::COLOR_BGR2RGBA);
        

        //copy the image to cuda mem
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

        //run the detector
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
        //handle the detections
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