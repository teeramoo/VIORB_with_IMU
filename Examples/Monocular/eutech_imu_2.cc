/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>
#include<string>
#include<System.h>

#include "../../src/IMU/imudata.h"
#include "../../src/IMU/configparam.h"


using namespace std;

class Position
{
public:
    double X;
    double Y;
    double Z;
    double pitch;
    double roll;
    double yaw;
    double timestamp;
};



void tokenize(const string &str, vector<string> &vTokens);

void tokenize_space(const string &str, vector<string> &vTokens);

void readFrameData(const string &strFrameDataFile, vector<double> &vTimestamps,
        vector<int> &vFrameNames, vector<Position> &vIMUPositions);

void LoadImages(const string &strPathTimes, vector<double> &vTimeStamps);
                
void LoadIMUData(const string &strIMUPath, vector<Position> &vIMUPositions);

int main(int argc, char **argv)
{
   	if (argc != 10)
    {
        cerr << endl << "Usage: " << argv[0]
                << " <path_to_vocabulary> <path_to_settings_bottom_camera> <path_to_settings_top_camera> <path_to_video_bottom> "
                        "<path_to_video_top> <path_to_vid_log_file> <path_to_imu_log_file> <path_to_output_folder_top> <path_to_output_folder_bottom>"
                << endl;
        return 1;
    }
    
    string strVideoBottomFile = string(argv[4]);
    string strVideoTopFile = string(argv[5]);
    string strFrameDataFile = string(argv[6]);
    string strIMUDataFile = string(argv[7]);
    string sPathToOutputFolderTop = string(argv[8]);
    string sPathToOutputFolderBottom = string(argv[9]);

    // Retrieve paths to images
    vector<double> vTimestamps;
    LoadImages(strFrameDataFile, vTimestamps);

	vector<Position> vIMUPositions;
	LoadIMUData(strIMUDataFile, vIMUPositions);
	

    int nImages = vTimestamps.size();
	int nIMU = vIMUPositions.size();
	cout << "Images: " << nImages << endl;
	cout << "IMU: " << nIMU << endl;

	cv::VideoCapture videoCaptureBottom(strVideoBottomFile);
    if (!videoCaptureBottom.isOpened())
    {
        ostringstream ostream;
        ostream << "Could not open video file " << strVideoBottomFile;
        throw runtime_error(ostream.str());
    }

    cv::VideoCapture videoCaptureTop(strVideoTopFile);
    if (!videoCaptureTop.isOpened())
    {
        ostringstream ostream;
        ostream << "Could not open video file " << strVideoTopFile;
        throw runtime_error(ostream.str());
    }


    // Create SLAM system. It initializes all system threads and gets ready to process frames.

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
	cv::Mat im;
    
    double epsilon = 1e-6;
    
    int nIMUc = 0;
    int skip = 0;
    string mTransform = "TransformMatrix/transform.yaml" ;
    bool bInitMat = false;
    std::vector<cv::Mat> vCamPoses;
    vector<ORB_SLAM2::KeyFrame*> vKeyFrames;
    vector<int> vKeyFrameID;
    cv::Mat blankMat = cv::Mat();
    cv::Mat mTrcBotCam;
    cv::Mat mTrcTopCam;
    {
        ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,sPathToOutputFolderBottom, true);
        double imageMsgDelaySec = SLAM.imageDelaySec;
        mTrcBotCam = SLAM.GetInitCamPose();

        cout << "mTrcBotCam is : " << mTrcBotCam << endl;
        

    for(int ni=0; ni< nImages; ni++)         // Start point of 1st SLAM //    for(int ni=0; ni<nImages; ni++)
    {

        if (!videoCaptureBottom.read(im))         // Read image from file
		{
		  throw runtime_error("Premature end of bottom video file.");
		}

        double tframe = vTimestamps[ni];	//Time is in millisecond so divide by 1000
        double tempFrame = tframe/1e6;
		if(ni < skip)
		{
			nIMUc++;
			continue;
		}
        if(im.empty())
        {
            cerr << endl << "Failed to load image at: "
                 <<  ni << endl;
            return 1;
        }

		double tempIMU = vIMUPositions[nIMUc].timestamp/1e6;
        std::vector<ORB_SLAM2::IMUData> vimuData;

        cout << endl;
        cout << "--------------------------------------" << endl;
        cout << "Frame: "<< ni << " of " <<  nImages << endl;

        cout << "Checking IMU data at frame "  << ni << endl;
        while((tempFrame-tempIMU) > epsilon)
        {

        	Position currentPos = vIMUPositions[nIMUc];
        	double ax = currentPos.X;
        	double ay = currentPos.Y;
        	double az = currentPos.Z;
        	double wx = currentPos.roll;
        	double wy = currentPos.pitch;
        	double wz = currentPos.yaw;
        	double dTime = currentPos.timestamp/1e6;

        	ORB_SLAM2::IMUData imudata(wx,wy,wz,ax,ay,az,dTime);

           	vimuData.push_back(imudata);

           	nIMUc++;
            if(nIMUc <= vIMUPositions.size())
            {
                tempIMU = vIMUPositions[nIMUc].timestamp/1e6;
            }
            else
                break;

        }
        cout << "Finish checking IMU data at frame :" << ni<< endl;
        cout << "Size of vimuData is " << vimuData.size() << endl;

		if(vimuData.size() == 0)
		{
			cout<<"Hit blank IMU slot ###############################" << endl;
			cout<<"Skipping this frame (Specially if before initializing)!" << endl;
            vCamPoses.push_back(blankMat); // Add empty Matrix
           	continue;
		}

        if(ni < 200)
            continue;
        //Fast skip
        if(ni <0)
        {
            vCamPoses.push_back(blankMat); // Add empty Matrix
            continue;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

       SLAM.SetFrameNumber(ni);

       cv::Mat mCamPoseCurrent = SLAM.TrackMonoVI(im, vimuData, tempFrame - imageMsgDelaySec, blankMat );
       vCamPoses.push_back(mCamPoseCurrent); 


#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;
      
    }
	
    // Stop all threads
        cout << "Executing shutdown process SLAM.Shutdown() " << endl;

        vKeyFrames = SLAM.GetAllKeyFrames();
        cout << "Total vKeyframe " << vKeyFrames.size() << endl;


        for(int i = 0; i < vKeyFrames.size(); i++)
        {
            vKeyFrameID.push_back(vKeyFrames[i]->iFrameNumber);
        }
        cout << "Total vKeyFrameID " << vKeyFrameID.size() << endl;

        SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;
/*
        cout << "Total number of KeyFrame : " << vKeyFrames.size() << endl;

        cout << "First KeyFrame is : " << vKeyFrames[0]->iFrameNumber << endl;
        cout << "with Matrix " << vKeyFrames[0]->GetPose() << endl;
        cout << "Second KeyFrame is : " << vKeyFrames[1]->iFrameNumber << endl;
        cout << "with Matrix " << vKeyFrames[1]->GetPose() << endl;

        cout << "Size of vCamposes is : " << vCamPoses.size() <<endl;
        cout << "Matrix of Frame according to 1st keyframe is : " << vCamPoses[vKeyFrames[0]->iFrameNumber] << endl;
        cout << "Matrix of Frame according to 2nd keyframe is : " << vCamPoses[vKeyFrames[1]->iFrameNumber] << endl;
        cout << "Matrix of Frame according to 2nd keyframe +1 is : " << vCamPoses[vKeyFrames[1]->iFrameNumber +1] << endl;
        cout << "Matrix of Frame according to 2nd keyframe -1 is : " << vCamPoses[vKeyFrames[1]->iFrameNumber -1 ] << endl;

        int EmptyFrame = 0;
        vector<int> TestCamPoses;
        for(int i=0; i < vCamPoses.size(); i++)
        {
            if(vCamPoses[i].empty())
            {
                EmptyFrame++;
                TestCamPoses.push_back(i);
            }

        }
        cout << "Total empty frame is : " << EmptyFrame << endl;
        cout << "First Empty frame is : " << TestCamPoses.front() << endl;
        cout << "Last emptry frame is : " << TestCamPoses.back() << endl;
*/
    //    cout << "SLAM 1 was initialized at : " << vKeyFrames[1]->iFrameNumber << endl;

    }

    // Save camera trajectory and keyframe info

    cout << "----------------------------------------------------------------------------" << endl;

    cout << "First SLAM process ended. Starting second SLAM...." << endl;

 //   return 0 ;
    sleep(3);

    nIMUc = 0;
    skip = 0;
    cout << "Images: " << nImages << endl;
    cout << "IMU: " << nIMU << endl;

    {
        ORB_SLAM2::System SLAM(argv[1],argv[3],ORB_SLAM2::System::MONOCULAR,sPathToOutputFolderTop, true);
        double imageMsgDelaySec = SLAM.imageDelaySec;
        SLAM.SetSecondSLAM(true); // Set 2nd SLAM
        mTrcTopCam = SLAM.GetInitCamPose();

        cout << "mTrcTopCam is : " << mTrcTopCam << endl;

        int FirstFrameCounter = 0;
        cv::Mat mForcedKF;


        cv::Mat mCamBotToTop = cv::Mat::eye(4,4,CV_32F);
        cv::Mat t1, t2, R1, R2;

        mTrcBotCam(cv::Rect(3,0,1,3)).copyTo(t1);
        mTrcBotCam(cv::Rect(0,0,3,3)).copyTo(R1);

        mTrcTopCam(cv::Rect(3,0,1,3)).copyTo(t2);
        mTrcTopCam(cv::Rect(0,0,3,3)).copyTo(R2);

        cv::Mat tempMat1 = R2.inv()*R1;
        cv::Mat tempMat2 = R2.inv()* (t1 - t2);
        cv::Mat tempMat3 = t1-t2;


        tempMat1.copyTo(mCamBotToTop(cv::Rect(0,0,3,3)));
        tempMat2.copyTo(mCamBotToTop(cv::Rect(3,0,1,3)));


        cout << " R2.inv()*R1 is : " << R2.inv()*R1 << endl;

        cout << "  R2.inv()* (t1 - t2) is : " <<  R2.inv()* (t1 - t2) << endl;

        cout << "t1-t2 is : " << t1-t2 << endl;

        cout << "R2 is : " << R2 << endl;

        cout << "t2 is : " << t2 << endl;

        cout << "R1 is : " << R1 << endl;

        cout << "t1 is : " << t1 << endl;

        cout << "R2.inv() is : " << R2.inv() << endl;

        cout << "mCamBotToTop is : " << mCamBotToTop << endl;

        cout << "Check mTrcTop * mTrcBot.inv() " << mTrcTopCam * mTrcBotCam.inv()<< endl;

        sleep(10);
        SLAM.SetCamTopToBot(mCamBotToTop);

        for(int ni=0; ni< nImages; ni++)  //     for(int ni=0; ni<nImages; ni++)
        {
            // Read image from file
            if (!videoCaptureTop.read(im)) {
                throw runtime_error("Premature end of top video file.");
            }

            
            double tframe = vTimestamps[ni];	//Time is in millisecond so divide by 1000
            double tempFrame = tframe/1e6;
            if(ni < skip)
            {
                nIMUc++;
                continue;
            }
            if(im.empty())
            {
                cerr << endl << "Failed to load image at: "
                     <<  ni << endl;
                return 1;
            }


            if(!SLAM.CheckInitialization())  //Check initialization
            {
                if(ni > vKeyFrameID[FirstFrameCounter])
                {
                    while(vKeyFrameID[FirstFrameCounter] < ni)
                    {
                        FirstFrameCounter++;
                    }

                    if(FirstFrameCounter < vKeyFrameID.size())
                        mForcedKF = vKeyFrames[FirstFrameCounter]->GetPose();
                    else
                        mForcedKF = vKeyFrames[FirstFrameCounter-1]->GetPose();


                    SLAM.SetForceKeyFrame(vKeyFrameID[FirstFrameCounter], mForcedKF);

                }
            }


            double tempIMU = vIMUPositions[nIMUc].timestamp/1e6;
            std::vector<ORB_SLAM2::IMUData> vimuData;

            cout << endl;
            cout << "--------------------------------------" << endl;
            cout << "Frame: "<< ni << " of " <<  nImages << endl;

            while((tempFrame-tempIMU) > epsilon)
            {

                Position currentPos = vIMUPositions[nIMUc];
                double ax = currentPos.X;
                double ay = currentPos.Y;
                double az = currentPos.Z;
                double wx = currentPos.roll;
                double wy = currentPos.pitch;
                double wz = currentPos.yaw;
                double dTime = currentPos.timestamp/1e6;
                cout << "Checking IMU data at frame "  << ni << endl;
                ORB_SLAM2::IMUData imudata(wx,wy,wz,ax,ay,az,dTime);
                cout << "Finish checking IMU data at frame :" << ni<< endl;

                vimuData.push_back(imudata);
                cout << "Size of vimuData is " << vimuData.size() << endl;


                nIMUc++;
                if(nIMUc <= vIMUPositions.size())
                {
                    tempIMU = vIMUPositions[nIMUc].timestamp/1e6;
                    cout << "Current IMU data size is : " << nIMUc << " of "<< vIMUPositions.size() << endl;
                }
                else
                    break;

            }

            if(vimuData.size() == 0)
            {
                cout<<"Hit blank IMU slot ###############################" << endl;
                cout<<"Skipping this frame (Specially if before initializing)!" << endl;
                continue;
            }


#ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
            std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

            // Quick skip to Initialized frame - 20
            if (ni < vKeyFrameID[FirstFrameCounter] - 1)
                continue;
            
            SLAM.SetFrameNumber(ni);
//            cv::Mat mCamPoseCurrent = SLAM.TrackMonoVI(im, vimuData, tempFrame - imageMsgDelaySec);

            cout << "SLAM_1 was initialized at frame : " << vKeyFrameID[FirstFrameCounter] << endl;

            cv::Mat mCamPoseCurrent = SLAM.TrackMonoVI(im, vimuData, tempFrame - imageMsgDelaySec, vCamPoses[ni]);



            if(!mCamPoseCurrent.empty())
            {
                cout << "Frame number : " << ni << endl;
                cout << "current camera pose is : " << endl << mCamPoseCurrent << endl;
           }



#ifdef COMPILEDWITHC11
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
            std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
            vTimesTrack[ni]=ttrack;

        }

        // Stop all threads
        SLAM.Shutdown();

        // Tracking time statistics
        sort(vTimesTrack.begin(),vTimesTrack.end());
        float totaltime = 0;
        for(int ni=0; ni<nImages; ni++)
        {
            totaltime+=vTimesTrack[ni];
        }
        cout << "-------" << endl << endl;
        cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
        cout << "mean tracking time: " << totaltime/nImages << endl;

    }
    return 0;
}


void tokenize(const string &str, vector<string> &vTokens)
{
    int iPos = 0;
    int iTokBeg = 0;
    while (iPos < (int) str.length())
    {
        if (str[iPos] == ',')
        {
            if (iTokBeg < iPos)
            {
                vTokens.push_back(str.substr(iTokBeg, iPos - iTokBeg));
                iTokBeg = iPos + 1;
            }
        }
        iPos++;
    }
    if (iTokBeg < (int) str.length())
        vTokens.push_back(str.substr(iTokBeg));
}


void LoadImages(const string &strPathTimes, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    string s;
//    getline(fTimes,s);
//    getline(fTimes,s);
//    getline(fTimes,s);
//    getline(fTimes,s);
    cout<<"Read vid timestamp:" << strPathTimes<<endl;
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
       
        if(!s.empty())
        {
        	vector<string> vTokens;
	        
	        /*
	        stringstream ss;
	        ss.str(s);
	        string token;
	        
	        while(getline(ss,token,delim))
	        {
	        	vTokens.push_back(token);
	        }
	        */
        	
        	tokenize(s, vTokens);
        	if(vTokens.size() == 2)
        	{
        		double t = std::stod(vTokens[0]);
        		vTimeStamps.push_back(t);
        	}
        }
    }
}

void LoadIMUData(const string &strIMUPath, vector<Position> &vIMUPositions)
{
	ifstream fTimes;
    fTimes.open(strIMUPath.c_str());
    // Skip first line
    string s;
    getline(fTimes,s);
    
    Position positionLatest;
    positionLatest.X = 0;
    positionLatest.Y = 0;
    positionLatest.Z = 0;
    positionLatest.roll = 0;
    positionLatest.pitch = 0;
    positionLatest.yaw = 0;
    positionLatest.timestamp = 0;
    
//    double offsetAcl = 9.8/8192.0;
//    double offsetGy = 131.0/250.0;
//    double offsetGy = 1.0/131.0;
      double offsetAcl = 0.0;
      double offsetGy = 0.0;
//      double offsetGy = 1.0*180/M_PI;

    int countLine = 0;
    while(!fTimes.eof())
    {   countLine++;
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
        	vector<string> vTokens;
        	tokenize(s, vTokens);
        	if(vTokens.size() == 8)
        	{
		    	positionLatest.timestamp = std::stod(vTokens[0]);

		        //positionLatest.timestamp /= 1e9;

				positionLatest.X = std::stod(vTokens[1]);
		    	positionLatest.Y = std::stod(vTokens[2]);
		    	positionLatest.Z = std::stod(vTokens[3]);
		    	
		    	positionLatest.roll = std::stod(vTokens[4]);
		    	positionLatest.pitch = std::stod(vTokens[5]);
		    	positionLatest.yaw = std::stod(vTokens[6]);
		    	
		    	positionLatest.X *= offsetAcl;
		    	positionLatest.Y *= offsetAcl;
		    	positionLatest.Z *= offsetAcl;

                //Conversion
	//	    	positionLatest.Z -= 9.81;

		    	positionLatest.roll *= offsetGy;
		    	positionLatest.pitch *= offsetGy;
		    	positionLatest.yaw *= offsetGy;
		    			    	
		    	vIMUPositions.push_back(positionLatest);
		    }

            if(vTokens.size() > 0 && vTokens.size() < 8)
                cout << "Skip line number from token < 8: " << countLine << endl;
            if(vTokens.size() > 8)
                cout << "Skip line number from token > 8: " << countLine << endl;
        }
    }
    cout << "Total line read : " << countLine << endl;
}


void tokenize_space(const string &str, vector<string> &vTokens)
{
    int iPos = 0;
    int iTokBeg = 0;
    while (iPos < (int) str.length())
    {
        if (str[iPos] == ' ')
        {
            if (iTokBeg < iPos)
            {
                vTokens.push_back(str.substr(iTokBeg, iPos - iTokBeg));
                iTokBeg = iPos + 1;
            }
        }
        iPos++;
    }
    if (iTokBeg < (int) str.length())
        vTokens.push_back(str.substr(iTokBeg));
}


void readFrameData(const string &strFrameDataFile, vector<double> &vTimestamps,
        vector<int> &vFrameNames, vector<Position> &vIMUPositions)
{
    ifstream f;
    f.open(strFrameDataFile.c_str());
    if (!f.is_open())
    {
        ostringstream ostream;
        ostream << "Could not open frame data file " << strFrameDataFile;
        throw runtime_error(ostream.str());
    }

    Position positionLatest;
    positionLatest.X = 0;
    positionLatest.Y = 0;
    positionLatest.Z = 0;
    positionLatest.roll = 0;
    positionLatest.pitch = 0;
    positionLatest.yaw = 0;
    positionLatest.timestamp = 0;
    while (f.is_open() && !f.eof())
    {
        string s;
        getline(f, s);
        if (s.length() == 0)
            continue;
        if (s[0] == '#')
            continue;
        vector<string> vTokens;
        tokenize_space(s, vTokens);
        if (vTokens[0] == "Output")
        {
            // Frame descriptor
            vTimestamps.push_back(std::stod(vTokens[8]));
            vFrameNames.push_back(vIMUPositions.size());
        }
        else if (vTokens[1] == "xacc" && vTokens[3] == "yacc")
        {
            // linear acceleration
            positionLatest.timestamp = std::stof(vTokens[0]);
            positionLatest.X = std::stof(vTokens[2]);
        	positionLatest.Y = std::stof(vTokens[4]);
        	positionLatest.Z = std::stof(vTokens[6]);
        	vIMUPositions.push_back(positionLatest);
        }
        else if (vTokens[1] == "roll" && vTokens[3] == "pitch")
        {
            // ATTITUDE
            positionLatest.roll = std::stof(vTokens[2]);
            positionLatest.pitch = std::stof(vTokens[4]);
            positionLatest.yaw = std::stof(vTokens[6]);
        }
    }
}


