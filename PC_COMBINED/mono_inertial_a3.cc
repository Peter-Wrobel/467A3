/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
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


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <ctime>
#include <sstream>
#include <cassert>

#include <condition_variable>

#include <stdio.h>
#include <lcm/lcm-cpp.hpp>
#include "../../lcmtypes/mbot_imu_t.hpp"
#include "../../lcmtypes/mbot_image_t.hpp"

#include<opencv2/core/core.hpp>

#include<System.h>
#include "ImuTypes.h"

using namespace std;

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps);

void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro);

void lcm_imu_handler(void);

void lcm_photo_handler(void);



// Thread Synchronizers
std::mutex lcm_mutex;
std::condition_variable condv;
bool found_image;

// global values updated constantly
double vAccX, vAccY, vAccZ, vRotX, vRotY, vRotZ, vTime;
int16_t p_image[144][192][3]; 
uint32_t ni;




class Handler 
{
    public:
        ~Handler() {}
        void handleIMUMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const mbot_imu_t* msg)
        {
            std::lock_guard<std::mutex> lk(lcm_mutex);
            
            printf("Received message on channel \"%s\":\n", chan.c_str());


            vAccX  =  msg->gyro[0];
            vAccY  =  msg->gyro[1];
            vAccZ  =  msg->gyro[3];
            vRotX  =  msg->mag[0];
            vRotY  =  msg->mag[1];
            vRotZ  =  msg->mag[2];
            vTime  =  msg->utime;

        }

    void handleImageMessage(const lcm::ReceiveBuffer* rbuf,
        const std::string& chan, 
        const mbot_image_t *msg)
        {
            std::lock_guard<std::mutex> lk(lcm_mutex);
            found_image = true;
            ni = msg->utime;
            condv.notify_all();

        }



    
};


void lcm_imu_handler(void){

    // Makes lcm o
    lcm::LCM lcm;
    if(!lcm.good())
        return;
    Handler handlerObject;
    lcm.subscribe("MBOT_IMU", &Handler::handleIMUMessage, &handlerObject);



    while(0 == lcm.handle());

}


void lcm_image_handler(void){

    // Makes lcm o
    lcm::LCM lcm;
    if(!lcm.good())
        return;
    Handler handlerObject;
    lcm.subscribe("MBOT_IMAGE", &Handler::handleImageMessage, &handlerObject);



    while(0 == lcm.handle());

}







double ttrack_tot = 0;
int main(int argc, char *argv[])
{

    if(argc < 7)
    {
        cerr << endl << "Usage: ./mono_inertial_a3\n  path_to_vocabulary\n  path_to_settings\n  path_to_sequence_folder_1\n  path_to_times_file_1\n  (path_to_image_folder_2\n    path_to_times_file_2 ...\n   path_to_image_folder_N path_to_times_file_N)\n  DO_IMU " << endl;
        return 1;
    }



    bool DO_IMU = atoi(argv[argc-1]);
    if (DO_IMU){

        std::thread lcm_thread(lcm_imu_handler);
    }


    std::thread lcm_thread(lcm_image_handler);



    const int num_seq = (argc-3)/2;
    assert(num_seq == 0); // Just to make sure the command args are correct -Bryce
    
    cout << "num_seq = " << num_seq << endl;
    bool bFileName= (((argc-4) % 2) == 1);
    string file_name;
    if (bFileName)
    {
        file_name = string(argv[argc-2]);
        cout << "file name: " << file_name << endl;
    }

    // Load all sequences:
    int seq;
    

    vector< vector<double> > vTimestampsImu;

    vector<int> first_imu(num_seq,0);


    vTimestampsImu.resize(num_seq);

    

    int tot_images = 0;
    


    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(tot_images);

    cout.precision(8);



    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(
                        argv[1],
                        argv[2],
                        DO_IMU ? ORB_SLAM3::System::IMU_MONOCULAR : ORB_SLAM3::System::MONOCULAR,
                        true);

    int proccIm=0;
    for (seq = 0; seq<num_seq; seq++)
    {
        // we wait for next image to be handled
        std::unique_lock<std::mutex> lk(lcm_mutex);
        condv.wait(lk, []{return found_image == true;});
        found_image = false;



        // Main loop
        cv::Mat im;
        vector<ORB_SLAM3::IMU::Point> vImuMeas;
        proccIm = 0;

            
            
        std::cout << "-----------a" << std::endl;
        
        // Read image from file
        //im = cv::imread(p_image,CV_LOAD_IMAGE_UNCHANGED);
        std::cout << "-----------b" << std::endl;
        

        //double tframe = vTimestampsCam[seq][ni];
        double tframe = ni * 0.1;


        std::cout << "-----------c" << std::endl;

        // Load imu measurements from previous frame
        vImuMeas.clear();
        std::cout << "-----------c.5" << std::endl;


        vImuMeas.push_back(ORB_SLAM3::IMU::Point(vAccX, vAccY, vAccZ, vRotX, vRotY, vRotZ, vTime));

        std::cout << "-----------d" << std::endl;
        
        #ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        #else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
        #endif
        
        std::cout << "-----------e" << std::endl;
        
        if (DO_IMU)
            SLAM.TrackMonocular(im,tframe,vImuMeas);
        else
            SLAM.TrackMonocular(im,tframe);
        std::cout << "-----------f" << std::endl;
            

        #ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        #else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
        #endif

        double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        ttrack_tot += ttrack;
        

        std::cout << "-----------g" << std::endl;

        std::cout << "-----------h" << std::endl;

        if(seq < num_seq - 1)
        {
            cout << "Changing the dataset" << endl;

            SLAM.ChangeDataset();
        }
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    if (bFileName)
    {
        const string kf_file =  "kf_" + string(argv[argc-2]) + ".txt";
        const string f_file =  "f_" + string(argv[argc-2]) + ".txt";
        SLAM.SaveTrajectoryEuRoC(f_file);
        SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);
    }
    else
    {
        SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
        SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
    }

    return 0;
}

/*
void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(10000);
    vstrImages.reserve(10000);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImages.push_back(strImagePath + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t/1e9);

        }
    }
}

void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro)
{
    ifstream fImu;
    fImu.open(strImuPath.c_str());
    vTimeStamps.reserve(10000);
    vAcc.reserve(10000);
    vGyro.reserve(10000);

    while(!fImu.eof())
    {
        string s;
        getline(fImu,s);
        if (s[0] == '#')
            continue;

        if(!s.empty())
        {
            string item;
            size_t pos = 0;
            double data[7];
            int count = 0;
            while ((pos = s.find(',')) != string::npos) {
                item = s.substr(0, pos);
                data[count++] = stod(item);
                s.erase(0, pos + 1);
            }
            item = s.substr(0, pos);
            data[6] = stod(item);

            vTimeStamps.push_back(data[0]/1e9);
            vAcc.push_back(cv::Point3f(data[4],data[5],data[6]));
            vGyro.push_back(cv::Point3f(data[1],data[2],data[3]));
        }
    }
}
*/
