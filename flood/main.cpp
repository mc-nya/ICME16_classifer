#include"headfile.h";
#include"Confirm.h"
#include<io.h>
float GetDist(cv::Mat &img, Point2i p1, Point2i p2){		//output:mm    input: poing2i(i,j);
	float dist = 0;
	float x1 = img.at<cv::Vec3f>(p1.y, p1.x)[0];
	float y1 = img.at<cv::Vec3f>(p1.y, p1.x)[1];
	float z1 = img.at<cv::Vec3f>(p1.y, p1.x)[2];
	float x2 = img.at<cv::Vec3f>(p2.y, p2.x)[0];
	float y2 = img.at<cv::Vec3f>(p2.y, p2.x)[1];
	float z2 = img.at<cv::Vec3f>(p2.y, p2.x)[2];
	dist = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2) *(z1 - z2));
	return dist;
}

// frok from PCL, fix the const factor for **KINECT ONLY**!!!
int GetSizeInImageBySizeIn3D(const int iSizeIn3D, const int iDistance)
{
	if (iDistance == 0 || iSizeIn3D == 0)
	{
		return 0;
	}

	static double dConstFactor = 0.0; // 常数，表示空间中？长度的距离相机为1毫米的直线将投影成一个像素宽
	static bool bIsFactorComputed = false; // 因为只想在第一次计算一次常数，以后不再计算，本变量保存计算状态

	if (!bIsFactorComputed)
	{
		// 在深度图上假定两点
		//pPoint3D[2] = { { 0, 0, 1000 }, { 100, 0, 1000 } }; // {列, 行, 深度}
		//float X1, X2, Y1, Y2, D1, D2;
		//float rX1, rX2, rY1, rY2, rZ1, rZ2;
		//X1 = 0; //1的列
		//Y1 = 0; //1的行
		//D1 = 1000;
		//X2 = 100; //2的列
		//Y2 = 0; //2的行
		//D2 = 1000;
		//g_openNi.m_DepthGenerator.ConvertProjectiveToRealWorld(2, pPoint3D, pPoint3D); // 可以得到点在空间中的实际坐标
		//CoordinateConverter::convertDepthToWorld(g_openNi.streamDepth, X1, Y1, D1, &rX1, &rY1, &rZ1);
		//CoordinateConverter::convertDepthToWorld(g_openNi.streamDepth, X2, Y2, D2, &rX2, &rY2, &rZ2);
		// 两点在空间中的实际距离
		double d3DDistance = 173.66668978426750;//std::sqrt((long double)((rX1 - rX2) * (rX1 - rX2) + (rY1 - rY2) * (rY1 - rY2) + (rZ1 - rZ2) * (rZ1 - rZ2)));
		dConstFactor = d3DDistance / (1000 * 100);
		bIsFactorComputed = true;
	}

	return (int)(((double)iSizeIn3D / (double)iDistance) / dConstFactor);

}
int findz();

// ALL THE ARRAY HERE CAN BE CHANGED TO VECTOR AND BE PUT IN TO FINDZ()
// THEY ARE UGLY GLOBAL VARIBLES
// The queue in BFS
int quei[500000], quej[500000];
// recorder of pixel sets
int seti[500000], setj[500000], setnum[500000];		
// union-find set for each pixel
int flag[480][640];		//集合标志
// head pointer, tail pointer and set count in BFS
int h, t, setcount;		


fstream fout;
string outfile;
int pixelNum;
int main(){
	
	for (int i = 50800; i < 51400; i++){
		if ((i + 5) % 1 == 0){
			string number = to_string(i);
			outfile = "H:\\result\\flood\\47\\" + number + ".png";
			findz(number, outfile);
		}

	}

}

int totalNum = 0;
int totalCount = 0;

const double camera_factor = 1;
const double camera_cx = 325.5;
const double camera_cy = 253.5;
const double camera_fx = 518.0;
const double camera_fy = 519.0;


int findz(const string &number, const string &outfile){

	string sfileDepth = "E:\\lab\\dataset\\47\\depth\\depth_" + number + ".png";
	string sfileRGB = "E:\\lab\\dataset\\47\\rgb\\rgb_" + number + ".png";


	char cfile1[200], cfile2[200];
	strncpy(cfile1, sfileDepth.c_str(), sfileDepth.length());
	strncpy(cfile2, sfileRGB.c_str(), sfileRGB.length());
	cfile1[sfileDepth.length()] = '\0';
	cfile2[sfileRGB.length()] = '\0';
	if (access(cfile1, 0) == -1 || access(cfile2, 0) == -1){
		return 0;
	}
	Mat rawRGB=imread(sfileRGB, CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);
	Mat rawDepth = imread(sfileDepth, CV_LOAD_IMAGE_ANYCOLOR | CV_LOAD_IMAGE_ANYDEPTH);
	Mat properRGB = rawRGB.clone();
	Mat headCloud;			
	headCloud = Mat(480, 640, CV_32FC3);

	// initialize union-find set of pixels
	for (int j = 0; j < 480; j++){
		for (int i = 0; i < 640; i++){
			flag[j][i] = 0;
		}
	}


	//generate point cloud
	for (int j = 1; j < rawDepth.rows; ++j){
		for (int i = 1; i < rawDepth.cols; ++i){
			int d = rawDepth.at<unsigned short>(j, i);
			// skip too near and too far pixels
			if (rawDepth.at<unsigned short>(j, i)<800 || rawDepth.at<unsigned short>(j, i)>10000){
				headCloud.at<cv::Vec3f>(j, i)[0] = 0;
				headCloud.at<cv::Vec3f>(j, i)[1] = 0;
				headCloud.at<cv::Vec3f>(j, i)[2] = 0;
				continue;
			}
			// calculate x,y,z and store them in position 0,1,2
			headCloud.at<cv::Vec3f>(j, i)[2] = double(d) / camera_factor;
			headCloud.at<cv::Vec3f>(j, i)[0] = (i - camera_cx) * headCloud.at<cv::Vec3f>(j, i)[2] / camera_fx;
			headCloud.at<cv::Vec3f>(j, i)[1] = (j - camera_cy) * headCloud.at<cv::Vec3f>(j, i)[2] / camera_fy;
			
		}
	}

	

	setcount = 0;
	for (int j = 0; j < rawDepth.rows; j++){
		for (int i = 0; i < rawDepth.cols; i++){
			// flag[j][i]==0 means the pixel isn't in a set, so start search here
			if (flag[j][i] == 0 && rawDepth.at<unsigned short>(j, i)>800 && rawDepth.at<unsigned short>(j, i)<8000){
				
				// add a new set
				setcount++;
				//record start point of this set
				seti[setcount] = i; setj[setcount] = j;
				// record number of pixels in this set
				setnum[setcount] = 1;
				// change set number in flag arrary
				flag[j][i] = setcount;

				// initialize BFS queue
				h = 1; t = 1;
				quei[1] = i;	quej[1] = j;
				while (h <= t){
					int tempi = quei[h];
					int tempj = quej[h];
					int pixelNum = 80;
					
					//explore different direction
					// if a pixel is not explored, put it in the queue and change it's flag
					if (tempi>0 && flag[tempj][tempi-1]==0){
						double dist = GetDist(headCloud, Point2i(tempi, tempj), Point2i(tempi - 1, tempj));
						if (dist < pixelNum){
							t++;
							quei[t] = tempi-1;
							quej[t] = tempj;
							flag[tempj][tempi - 1] = setcount;
							setnum[setcount]++;
						}
					}

					if (tempj<479 && flag[tempj + 1][tempi] == 0){
						double dist = GetDist(headCloud, Point2i(tempi, tempj), Point2i(tempi, tempj + 1));
						if (dist < pixelNum){
							t++;
							quei[t] = tempi;
							quej[t] = tempj + 1;
							flag[tempj + 1][tempi] = setcount;
							setnum[setcount]++;
						}
					}

					if (tempi<639  && flag[tempj][tempi + 1] == 0){
						double dist = GetDist(headCloud, Point2i(tempi, tempj), Point2i(tempi + 1, tempj));
						if (dist < pixelNum){
							t++;
							quei[t] = tempi + 1;
							quej[t] = tempj;
							flag[tempj][tempi + 1] = setcount;
							setnum[setcount]++;
						}
					}
					h++;

				}
			}
		}
	}



	// generate RGB colors for visualization
	for (int j = 0; j < properRGB.rows; j++){
		for (int i = 0; i < properRGB.cols; i++){
			int pixelNum = GetSizeInImageBySizeIn3D(150, rawDepth.at<unsigned short>(j, i));
			int curset = flag[j][i];
			int dep = rawDepth.at<unsigned short>(setj[curset], seti[curset]);
			if (setnum[flag[j][i]]*dep>1200000){
				properRGB.at<Vec3b>(j, i)[0] = (flag[j][i] * 20) % 255;
				properRGB.at<Vec3b>(j, i)[1] = (flag[j][i] * 30) % 255;
				properRGB.at<Vec3b>(j, i)[2] = (flag[j][i] * 40) % 255;
			}
		}
	}




	Mat drawRGB = rawRGB.clone();
	int ccount = 0;
	int ncount = 0;
	vector<Point> validPoint;
	for (int k = 1; k <= setcount; k++){
		int i = seti[k];
		int j = setj[k];
		int pixelNum = GetSizeInImageBySizeIn3D(150, rawDepth.at<unsigned short>(j, i));

		int dep = rawDepth.at<unsigned short>(j, i);
	
		// select imageblock by # pixels in the set
		// it may change to other threshold such as depth*setnum[k]< *a number*
		if ((setnum[k] / (pixelNum ^ 2)  > 80) || setnum[k]>150){


			// ***********second step here*******************
			// use another classfier to test the head top region
			//if (!Confirm::HOTRGBDconfirm(rawDepth, rawRGB, i, j, pixelNum)){
			//	continue;
			//}

			ccount++;
			Point p(i, j);

			circle(drawRGB, Point(i, j), 3, Scalar(0, 69, 255), -1, 8);
			circle(properRGB, Point(i, j), 3, Scalar(0, 69, 255), -1, 8);
			rectangle(drawRGB, cv::Rect(i - 1.5*pixelNum, j - 0.7*pixelNum, 3 * pixelNum, 4 * pixelNum), cv::Scalar(47, 230, 173), 5, 8);

		}
	}


	imshow("result of first step", drawRGB);

	//cv::imwrite(outfile, drawRGB);

	cvWaitKey(200);

	return 0;
}