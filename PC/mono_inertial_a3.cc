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

#include<opencv2/core/core.hpp>

#include<System.h>
#include "ImuTypes.h"

using namespace std;

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps);

void LoadIMU(const string &strImuPath, vector<double> &vTimeStamps, vector<cv::Point3f> &vAcc, vector<cv::Point3f> &vGyro);

double ttrack_tot = 0;
int main(int argc, char *argv[])
{

    if(argc < 7)
    {
        cerr << endl << "Usage: ./mono_inertial_a3\n  path_to_vocabulary\n  path_to_settings\n  path_to_sequence_folder_1\n  path_to_times_file_1\n  (path_to_image_folder_2\n    path_to_times_file_2 ...\n   path_to_image_folder_N path_to_times_file_N)\n  DO_IMU " << endl;
        return 1;
    }

    bool DO_IMU = atoi(argv[argc-1]);

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
    
    //vector< vector<string> > vstrImageFilenames;
    //vector< vector<double> > vTimestampsCam;
    //vector< vector<cv::Point3f> > vAcc, vGyro;
    vector< vector<double> > vTimestampsImu;
    //vector<int> nImages;
    //vector<int> nImu;
    vector<int> first_imu(num_seq,0);

    //vstrImageFilenames.resize(num_seq);
    //vTimestampsCam.resize(num_seq);
    //vAcc.resize(num_seq);
    //vGyro.resize(num_seq);
    vTimestampsImu.resize(num_seq);
    //nImages.resize(num_seq);
    //nImu.resize(num_seq);
    

    int tot_images = 0;
    
    /*
    for (seq = 0; seq<num_seq; seq++)
    {
        
        cout << "Loading images for sequence " << seq << "...";

        string pathSeq(argv[(2*seq) + 3]);
        string pathTimeStamps(argv[(2*seq) + 4]);

        string pathCam0 = pathSeq + "/mav0/cam0/data";
        string pathImu = pathSeq + "/mav0/imu0/data.csv";

        LoadImages(pathCam0, pathTimeStamps, vstrImageFilenames[seq], vTimestampsCam[seq]);
        cout << "LOADED!" << endl;

        cout << "Loading IMU for sequence " << seq << "...";
        LoadIMU(pathImu, vTimestampsImu[seq], vAcc[seq], vGyro[seq]);
        cout << "LOADED!" << endl;

        nImages[seq] = vstrImageFilenames[seq].size();
        tot_images += nImages[seq];
        nImu[seq] = vTimestampsImu[seq].size();

        if((nImages[seq]<=0)||(nImu[seq]<=0))
        {
            cerr << "ERROR: Failed to load images or IMU for sequence" << seq << endl;
            return 1;
        }

        // Find first imu to be considered, supposing imu measurements start first

        while(vTimestampsImu[seq][first_imu[seq]]<=vTimestampsCam[seq][0])
            first_imu[seq]++;
        first_imu[seq]--; // first imu measurement to be considered

    }
    */

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(tot_images);

    cout.precision(8);

    /*cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl;
    cout << "IMU data in the sequence: " << nImu << endl << endl;*/

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(
                        argv[1],
                        argv[2],
                        DO_IMU ? ORB_SLAM3::System::IMU_MONOCULAR : ORB_SLAM3::System::MONOCULAR,
                        true);

    int proccIm=0;
    for (seq = 0; seq<num_seq; seq++)
    {

        // Main loop
        cv::Mat im;
        vector<ORB_SLAM3::IMU::Point> vImuMeas;
        proccIm = 0;
        for(int ni=0; ni<9999999999; ni++, proccIm++)
        {
        
            std::string p_image = "stream_image/" + std::to_string(ni) + ".png";
            std::string p_imu   = "stream_imu/"   + std::to_string(ni) + ".imu";
            while (true) {
                
                bool ie, pe;
                
                {
                    fstream ifs(p_image), pfs(p_imu);
                    ie = !ifs.fail();
                    pe = !pfs.fail();
                }
            
                
                if (ie && pe)
                    break;
                    
                std::cout << "p_image p_imu : " << ie << " " << pe << std::endl;
                usleep(1000000);
                
            }
            
            std::cout << "-----------a" << std::endl;
            
            // Read image from file
            im = cv::imread(p_image,CV_LOAD_IMAGE_UNCHANGED);
            std::cout << "-----------b" << std::endl;
            
            std::fstream imu_stream(p_imu);
            double vAccX, vAccY, vAccZ, vRotX, vRotY, vRotZ, vTime;
            imu_stream >> vAccX >> vAccY >> vAccZ >> vRotX >> vRotY >> vRotZ >> vTime;

            //double tframe = vTimestampsCam[seq][ni];
            double tframe = ni * 0.1;

            if(im.empty())
            {
                cerr << endl << "Failed to load image at: "
                     <<  /*vstrImageFilenames[seq][ni]*/ p_image << endl;
                return 1;
            }
            std::cout << "-----------c" << std::endl;

            // Load imu measurements from previous frame
            vImuMeas.clear();
            std::cout << "-----------c.5" << std::endl;

            //if(ni>0)
            //{
                // cout << "t_cam " << tframe << endl;

                //while(vTimestampsImu[seq][first_imu[seq]]<=vTimestampsCam[seq][ni])
                //{
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(vAccX, vAccY, vAccZ, vRotX, vRotY, vRotZ, vTime));
                    //first_imu[seq]++;
                //}
            //}
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
            //vTimesTrack[ni]=ttrack;

            /*
            // Wait to load the next frame
            double T=0;
            if(ni<nImages[seq]-1)
                T = vTimestampsCam[seq][ni+1]-tframe;
            else if(ni>0)
                T = tframe-vTimestampsCam[seq][ni-1];

            if(ttrack<T)
                usleep((T-ttrack)*1e6); // 1e6
            */
            std::cout << "-----------h" << std::endl;
            
        }
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
