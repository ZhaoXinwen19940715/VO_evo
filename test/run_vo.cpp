/*
 * This file is part of VO
 * created by ZhaoXinwen
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#include <boost/timer.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/viz.hpp> 

#include "myslam/config.h"
#include "myslam/visual_odometry.h"
#include <myslam/orbextractor.h>
#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>

#include<opencv2/core/core.hpp>

using namespace std;

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps);
void SaveTrajectory(const string &filename);
 list<SE3> FramePoses;
int main(int argc, char **argv)
{
    if(argc != 3)
    {
        cerr << endl << "usage: run_vo parameter_file" << endl;
        return 1;
    }
    myslam::Config::setParameterFile ( argv[1] );

    myslam::VisualOdometry::Ptr vo ( new myslam::VisualOdometry );
    
    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimestamps;
    LoadImages(string(argv[2]), vstrImageLeft, vstrImageRight, vTimestamps);    
    const int nImages = vstrImageLeft.size();  
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);    
    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;   
    
    myslam::Camera::Ptr camera ( new myslam::Camera );
    
    
    
    cv::Mat imLeft, imRight;
    for(int ni=0; ni<nImages; ni++)
    {
	 cout<<"****** loop "<<ni+1<<" ******"<<endl;
        imLeft = cv::imread(vstrImageLeft[ni],CV_LOAD_IMAGE_UNCHANGED);
        imRight = cv::imread(vstrImageRight[ni],CV_LOAD_IMAGE_UNCHANGED);

	if ( imLeft.data==nullptr || imRight.data==nullptr )
            break;
	myslam::Frame::Ptr pFrame = myslam::Frame::createFrame();

	pFrame->camera_ = camera;
	pFrame->image_left_ = imLeft;
	pFrame->image_right_= imRight;
	pFrame->time_stamp_ = vTimestamps[ni];
	boost::timer timer;

	vo->addFrame ( pFrame );

        cout<<"VO costs time: "<<timer.elapsed()<<endl;
	cout << "*******************************" << endl;
	if ( vo->state_ == myslam::VisualOdometry::LOST )
        break;
        SE3 Twc = pFrame->T_c_w_.inverse();
        FramePoses.push_back(Twc);
    }
    SaveTrajectory("result.txt");
    return 0;
}


void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);

        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";
    string strPrefixRight = strPathToSequence + "/image_1/";

    const int nTimes = vTimestamps.size();
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
    }
}

void SaveTrajectory(const string &filename)
{
  cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    ofstream f;
    f.open(filename.c_str());
    f << fixed;
    for(list<SE3>::iterator lit=FramePoses.begin();lit!=FramePoses.end();lit++)
    {
      f << setprecision(9) << (*lit).rotation_matrix()(0,0) << " " << (*lit).rotation_matrix()(0,1)  << " " << (*lit).rotation_matrix()(0,2) << " "  << (*lit).translation()(0) << " " <<
             (*lit).rotation_matrix()(1,0) << " " << (*lit).rotation_matrix()(1,1)  << " " << (*lit).rotation_matrix()(1,2) << " "  << (*lit).translation()(1) << " " <<
             (*lit).rotation_matrix()(2,0) << " " << (*lit).rotation_matrix()(2,1)  << " " << (*lit).rotation_matrix()(2,2) << " "  << (*lit).translation()(2) << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}







