#include <string.h> // for basename(3) that doesn't modify its argument
#include <unistd.h> // for getopt
#include <sstream>

#include "cam_properties.hpp"

#include "apriltags.hpp"
#include "undistort.hpp"

#include <nlohmann/json.hpp>

// for convenience
using json = nlohmann::json;

json jsonify_tag(nvAprilTagsID_t detection)
{
    // create an empty structure (null)
    json j; 

    json["id"] = detection.id;

    json["pos"]["x"] = detection.translation[0];
    json["pos"]["y"] = detection.translation[1];
    json["pos"]["z"] = detection.translation[2];

    json["rotation"] = {{detection.translation[0],detection.translation[3],detection.translation[6]},
                        {detection.translation[1],detection.translation[4],detection.translation[7]},
                        {detection.translation[2],detection.translation[5],detection.translation[8]}};

    return j;
}


int main() {

    //############################################# SETUP VIDEO CAPTURE ##################################################################################################
    cv::VideoCapture capture("nvarguscamerasrc ! video/x-raw(memory:NVMM), width=1280, height=720,format=NV12, framerate=60/1 ! nvvidconv ! video/x-raw,format=BGRx !  videoconvert ! videorate ! video/x-raw,format=BGR,framerate=5/1 ! appsink", cv::CAP_GSTREAMER);
    std::cout<<"made it past cap device"<<std::endl;

    cv::Mat frame;
    cv::Mat img_rgba8;

    //capture a frame and hand it to VPI to initialize it
    capture.read(frame);
    cv::cvtColor(frame, img_rgba8, cv::COLOR_BGR2RGBA);
    setup_vpi(img_rgba8);

    //create the apriltag handler
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

        //capture a frame
        capture.read(frame);

        //undistort it
        undistort_frame(frame);

        //convert the frame to rgba
        cv::cvtColor(frame, img_rgba8, cv::COLOR_BGR2RGBA);
        
        //send the frame to GPU memory and run the detections
        uint32_t num_detections = process_frame(img_rgba8, impl_);

        //handle the detections
        for (int i = 0; i < num_detections; i++) {
            const nvAprilTagsID_t &detection = impl_->tags[i];

            json j = jsonify_tag(detection);

            std::cout << j.dump(4) << std::endl;

            // printf("ID: %d\n", detection.id);
            // printf("X dist: %f\n", detection.translation[0]);
            // printf("Y dist: %f\n", detection.translation[1]);
            // printf("Z dist: %f\n", detection.translation[2]);
            
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