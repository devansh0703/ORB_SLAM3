/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include <signal.h>
#include <stdlib.h>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <ctime>
#include <sstream>
#include <thread>
#include <condition_variable>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <System.h>

using namespace std;

bool b_continue_session;

void exit_loop_handler(int s){
    cout << "Finishing session" << endl;
    b_continue_session = false;
}

class V4L2Camera {
private:
    int fd;
    struct v4l2_buffer bufferinfo;
    void* buffer_start;
    size_t buffer_length;
    
public:
    int width, height;
    
    V4L2Camera() : fd(-1), buffer_start(nullptr), width(640), height(480) {}
    
    ~V4L2Camera() {
        close_camera();
    }
    
    bool open_camera(const string& device_path) {
        fd = open(device_path.c_str(), O_RDWR);
        if (fd == -1) {
            cerr << "Error opening device " << device_path << endl;
            return false;
        }
        
        // Query capabilities
        struct v4l2_capability cap;
        if (ioctl(fd, VIDIOC_QUERYCAP, &cap) == -1) {
            cerr << "Error querying capabilities" << endl;
            close(fd);
            return false;
        }
        
        cout << "Camera driver: " << cap.driver << endl;
        cout << "Camera card: " << cap.card << endl;
        
        // Set format
        struct v4l2_format format;
        format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        format.fmt.pix.width = width;
        format.fmt.pix.height = height;
        format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
        format.fmt.pix.field = V4L2_FIELD_INTERLACED;
        
        if (ioctl(fd, VIDIOC_S_FMT, &format) == -1) {
            cerr << "Error setting format" << endl;
            close(fd);
            return false;
        }
        
        width = format.fmt.pix.width;
        height = format.fmt.pix.height;
        
        cout << "Set format: " << width << "x" << height << endl;
        
        // Request buffer
        struct v4l2_requestbuffers req;
        req.count = 1;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;
        
        if (ioctl(fd, VIDIOC_REQBUFS, &req) == -1) {
            cerr << "Error requesting buffer" << endl;
            close(fd);
            return false;
        }
        
        // Query buffer
        bufferinfo.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        bufferinfo.memory = V4L2_MEMORY_MMAP;
        bufferinfo.index = 0;
        
        if (ioctl(fd, VIDIOC_QUERYBUF, &bufferinfo) == -1) {
            cerr << "Error querying buffer" << endl;
            close(fd);
            return false;
        }
        
        // Map buffer
        buffer_start = mmap(NULL, bufferinfo.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, bufferinfo.m.offset);
        buffer_length = bufferinfo.length;
        
        if (buffer_start == MAP_FAILED) {
            cerr << "Error mapping buffer" << endl;
            close(fd);
            return false;
        }
        
        // Queue buffer
        if (ioctl(fd, VIDIOC_QBUF, &bufferinfo) == -1) {
            cerr << "Error queuing buffer" << endl;
            munmap(buffer_start, buffer_length);
            close(fd);
            return false;
        }
        
        // Start streaming
        int type = bufferinfo.type;
        if (ioctl(fd, VIDIOC_STREAMON, &type) == -1) {
            cerr << "Error starting stream" << endl;
            munmap(buffer_start, buffer_length);
            close(fd);
            return false;
        }
        
        cout << "Camera opened and streaming started successfully!" << endl;
        return true;
    }
    
    bool capture_frame(cv::Mat& output) {
        // Dequeue buffer
        if (ioctl(fd, VIDIOC_DQBUF, &bufferinfo) == -1) {
            return false;
        }
        
        // Convert YUYV to RGB
        cv::Mat yuyv_image(height, width, CV_8UC2, buffer_start);
        cv::Mat bgr_image;
        cv::cvtColor(yuyv_image, bgr_image, cv::COLOR_YUV2BGR_YUYV);
        
        // Convert to grayscale
        cv::cvtColor(bgr_image, output, cv::COLOR_BGR2GRAY);
        
        // Queue buffer again
        if (ioctl(fd, VIDIOC_QBUF, &bufferinfo) == -1) {
            return false;
        }
        
        return true;
    }
    
    void close_camera() {
        if (fd != -1) {
            int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            ioctl(fd, VIDIOC_STREAMOFF, &type);
            
            if (buffer_start != nullptr) {
                munmap(buffer_start, buffer_length);
            }
            
            close(fd);
            fd = -1;
        }
    }
};

int main(int argc, char **argv) {

    if (argc < 3 || argc > 4) {
        cerr << endl
             << "Usage: ./mono_logitech_c270 path_to_vocabulary path_to_settings (trajectory_file_name)"
             << endl;
        return 1;
    }

    string file_name;

    if (argc == 4) {
        file_name = string(argv[argc - 1]);
    }

    struct sigaction sigIntHandler;

    sigIntHandler.sa_handler = exit_loop_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;

    sigaction(SIGINT, &sigIntHandler, NULL);
    b_continue_session = true;

    // Initialize camera using direct V4L2
    cout << "Opening camera using direct V4L2 interface..." << endl;
    V4L2Camera camera;
    
    if (!camera.open_camera("/dev/video0")) {
        cerr << "Failed to open camera with V4L2" << endl;
        return 1;
    }

    // Test frame capture
    cout << "Testing frame capture..." << endl;
    cv::Mat test_frame;
    if (!camera.capture_frame(test_frame)) {
        cerr << "Failed to capture test frame" << endl;
        return 1;
    }
    
    cout << "SUCCESS: Frame capture test passed" << endl;
    cout << "Frame size: " << test_frame.cols << "x" << test_frame.rows << endl;

    // Camera callback (exactly like RealSense)
    std::mutex imu_mutex;
    std::condition_variable cond_image_rec;

    cv::Mat imCV;
    int width_img = camera.width, height_img = camera.height;
    double timestamp_image = -1.0;
    bool image_ready = false;
    int count_im_buffer = 0; // count dropped frames

    auto camera_callback = [&]()
    {
        cv::Mat frame;
        auto start_time = std::chrono::steady_clock::now();
        
        while(b_continue_session) {
            bool ret = camera.capture_frame(frame);
            
            if(!ret || frame.empty()) {
                cout << "Failed to capture frame, retrying..." << endl;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            
            std::unique_lock<std::mutex> lock(imu_mutex);
            count_im_buffer++;

            auto now = std::chrono::steady_clock::now();
            double new_timestamp_image = std::chrono::duration_cast<std::chrono::microseconds>(now - start_time).count() * 1e-6;
            
            if(abs(timestamp_image-new_timestamp_image)<0.001){
                count_im_buffer--;
                continue;
            }

            imCV = frame.clone();
            timestamp_image = new_timestamp_image;
            image_ready = true;

            lock.unlock();
            cond_image_rec.notify_all();
        }
        cout << "Camera thread finished" << endl;
    };

    // Start camera thread
    cout << "Starting camera thread..." << endl;
    std::thread camera_thread(camera_callback);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    cout << "Initializing SLAM system..." << endl;
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR, true, 0, file_name);
    float imageScale = SLAM.GetImageScale();

    double timestamp;
    cv::Mat im;

    double t_resize = 0.f;
    double t_track = 0.f;

    cout << "Starting SLAM processing..." << endl;

    while (!SLAM.isShutDown() && b_continue_session)
    {
        {
            std::unique_lock<std::mutex> lk(imu_mutex);
            if(!image_ready)
                cond_image_rec.wait(lk);

#ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point time_Start_Process = std::chrono::steady_clock::now();
#else
            std::chrono::monotonic_clock::time_point time_Start_Process = std::chrono::monotonic_clock::now();
#endif

            if(count_im_buffer>1)
                cout << count_im_buffer -1 << " dropped frs\n";
            count_im_buffer = 0;

            timestamp = timestamp_image;
            im = imCV.clone();

            image_ready = false;
        }

        if(imageScale != 1.f)
        {
#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t_Start_Resize = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t_Start_Resize = std::chrono::monotonic_clock::now();
    #endif
#endif
            int width = im.cols * imageScale;
            int height = im.rows * imageScale;
            cv::resize(im, im, cv::Size(width, height));

#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t_End_Resize = std::chrono::steady_clock::now();
    #else
            std::chrono::monotonic_clock::time_point t_End_Resize = std::chrono::monotonic_clock::now();
    #endif
            t_resize = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t_End_Resize - t_Start_Resize).count();
            SLAM.InsertResizeTime(t_resize);
#endif
        }

#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t_Start_Track = std::chrono::steady_clock::now();
    #else
        std::chrono::monotonic_clock::time_point t_Start_Track = std::chrono::monotonic_clock::now();
    #endif
#endif
        // Pass image to SLAM system
        SLAM.TrackMonocular(im, timestamp);
#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t_End_Track = std::chrono::steady_clock::now();
    #else
        std::chrono::monotonic_clock::time_point t_End_Track = std::chrono::monotonic_clock::now();
    #endif
        t_track = t_resize + std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t_End_Track - t_Start_Track).count();
        SLAM.InsertTrackTime(t_track);
#endif
    }
    
    // Stop camera thread
    cout << "Stopping camera thread..." << endl;
    b_continue_session = false;
    camera_thread.join();
    
    cout << "System shutdown!\n";
    
    return 0;
}
