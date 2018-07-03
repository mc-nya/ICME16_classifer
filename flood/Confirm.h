#include "headfile.h"
#include"MatHelper.h"
#include"ForExp.h"
class Confirm{
public:


	//保存提取出来的小矩形，包含深度、RGB、点云图像
	static bool saveRect(const Mat &depthFrame, const Mat &rgbFrame, const Mat &pointCloud, int x, int y, int pixelNum, string path){
		
		Mat RectDepth, RectRGB, RectCloud;
		if (int(x - 1.5*pixelNum - 1) < 0 || int(x + 1.5*pixelNum + 1) > 640 || int(y + 3.3*pixelNum + 1) > 480 || int(y - 0.7*pixelNum - 1) < 0)
		{
			return false;
		}
		//设定输出参数
		vector<int> compression_params;
		compression_params.push_back(CV_IMWRITE_PNG_STRATEGY_RLE);
		compression_params.push_back(0);//这就是质量  默认值是3
		//获取深度图和彩色图RECT
		RectDepth = Mat(4 * pixelNum + 2, 3 * pixelNum + 2, CV_16U);
		RectRGB = Mat(4 * pixelNum + 2, 3 * pixelNum + 2, CV_8UC3);
		Mat tempDepth;
		Mat tempRGB;
		depthFrame.copyTo(tempDepth);
		rgbFrame.copyTo(tempRGB);
		MatHelper::GetRectDepthMat(tempDepth, RectDepth, int(x - 1.5*pixelNum - 1), int(y - 0.7*pixelNum - 1), 3 * pixelNum + 2, 4 * pixelNum + 2);//width height 
		MatHelper::GetRectMat(tempRGB, RectRGB, int(x - 1.5*pixelNum - 1), int(y - 0.7*pixelNum - 1), 3 * pixelNum + 2, 4 * pixelNum + 2);//width height 
		//获取点云RECT
		RectCloud = Mat(4 * pixelNum + 2, 3 * pixelNum + 2, CV_32FC3);
		Mat tempCloud;
		pointCloud.copyTo(tempCloud);
		MatHelper::GetRectMatF(tempCloud, RectCloud, int(x - 1.5*pixelNum - 1), int(y - 0.7*pixelNum - 1), 3 * pixelNum + 2, 4 * pixelNum + 2);//width height 
		//确定头顶点位置
		Point2i rectHeadPoint = Point2i(x - int(x - 1.5*pixelNum - 1), y - int(y - 0.7*pixelNum - 1));

		//save head position in first two pixels in depth image
		RectDepth.at<unsigned short>(0, 0) = rectHeadPoint.x;
		RectDepth.at<unsigned short>(0, 1) = rectHeadPoint.y;

		//debug
		//cout << RectDepth.at<unsigned short>(0, 0) << " " << RectDepth.at<unsigned short>(0, 1) << endl;

		//save pointcloud in txtfile
		fstream fout;
		fout.open(path + "raw_pointcloud.txt", ios::out);
		for (int q = 0; q < RectDepth.rows; q++){
			for (int p = 0; p < RectDepth.cols; p++){
				fout << RectCloud.at<Vec3f>(q, p)[0] << " " << RectCloud.at<Vec3f>(q, p)[1] << " " << RectCloud.at<Vec3f>(q, p)[2] << endl;
			}
		}
		fout.close();
		imwrite(path + "raw_Depth.png", RectDepth, compression_params);
		//cancel point_cloud output
		//imwrite(path + "raw_pointcloud.png",RectCloud, compression_params);
		imwrite(path + "raw_RGB.png", RectRGB, compression_params);
		for (int j = 0; j < RectRGB.rows; j++){
			for (int i = 0; i < RectRGB.cols; i++){
				double dist = MatHelper::GetDist(RectCloud, rectHeadPoint, Point2i(i, j));
				if (dist>500){
					RectRGB.at<cv::Vec3b>(j, i) = { 0, 255, 20 };
					RectDepth.at<unsigned short>(j, i) = 0;
				}
			}
		}

	}

	//SVM classifier : HOT on depth
	//NOT USED
	static bool SVMconfirm(const Mat &depthFrame, int x, int y, int pixelNum){
		Mat RectDepth;
		CvSVM cvSVM;
		if (int(x - 1.5*pixelNum - 1) < 0 || int(x + 1.5*pixelNum + 1) > 640 || int(y + 3.3*pixelNum + 1) > 480 || int(y - 0.7*pixelNum - 1) < 0)
		{
			return false;
		}
		RectDepth = Mat(4 * pixelNum + 2, 3 * pixelNum + 2, CV_16U);
		Mat tempDepth;
		depthFrame.copyTo(tempDepth);
		MatHelper::GetRectDepthMat(tempDepth, RectDepth, int(x - 1.5*pixelNum - 1), int(y - 0.7*pixelNum - 1), 3 * pixelNum + 2, 4 * pixelNum + 2);//width height 
		vector<float> hotFeature = ForExp::getHoTfeature(RectDepth);
		Mat responseMat = Mat(1, 1121, CV_32FC1);
		for (int i = 0; i < hotFeature.size(); i++){
			responseMat.at<float>(0, i) = hotFeature[i];
		}
		float response = cvSVM.predict(responseMat);
		if (response>0)
		{
			return true;
		}
		else{
			return false;
		}

	}

	//SVM classifier : HOT on Depth and Color
	//NOT USED
	static bool SVMconfirm(const Mat &depthFrame, const Mat &rgbFrame, int x, int y, int pixelNum){
		Mat RectDepth, RectRGB;
		CvSVM cvSVM;
		if (int(x - 1.5*pixelNum - 1) < 0 || int(x + 1.5*pixelNum + 1) > 640 || int(y + 3.3*pixelNum + 1) > 480 || int(y - 0.7*pixelNum - 1) < 0)
		{
			return false;
		}
		RectDepth = Mat(4 * pixelNum + 2, 3 * pixelNum + 2, CV_16U);
		RectRGB = Mat(4 * pixelNum + 2, 3 * pixelNum + 2, CV_8UC3);
		Mat tempDepth;
		depthFrame.copyTo(tempDepth);
		MatHelper::GetRectDepthMat(tempDepth, RectDepth, int(x - 1.5*pixelNum - 1), int(y - 0.7*pixelNum - 1), 3 * pixelNum + 2, 4 * pixelNum + 2);//width height 
		Mat tempRGB;
		rgbFrame.copyTo(tempRGB);
		MatHelper::GetRectMat(tempRGB, RectRGB, int(x - 1.5*pixelNum - 1), int(y - 0.7*pixelNum - 1), 3 * pixelNum + 2, 4 * pixelNum + 2);//width height 
		Mat rgbResized;
		Mat depthResized;

		cv::resize(RectDepth, RectDepth, cv::Size(50, 66));
		cv::resize(RectRGB, RectRGB, cv::Size(50, 66));
		vector<float> hotFeature = ForExp::getHoTfeature(RectRGB, RectDepth);
		Mat responseMat = Mat(1, 1121, CV_32FC1);
		for (int i = 0; i < hotFeature.size(); i++){
			responseMat.at<float>(0, i) = hotFeature[i];
			//cout << hotFeature[i] << " ";
		}
		//cout << endl;
		float response = cvSVM.predict(responseMat, true);

		//cout << response << endl;
		if (response<-0.2)
		{
			return true;
		}
		else{
			return false;
		}


	}

	//SVM classifier : HOT on Depth
	static bool HOTconfirm(const Mat &depthFrame, const Mat &rgbFrame, int x, int y, int pixelNum){
		Mat RectDepth, RectRGB;
		CvSVM cvSVM;
		cvSVM.load("E:\\lab\\train\\HOTOnlyDepth_10+18.xml");
		if (int(x - 1.5*pixelNum - 1) < 0 || int(x + 1.5*pixelNum + 1) > 640 || int(y + 3.3*pixelNum + 1) > 480 || int(y - 0.7*pixelNum - 1) < 0)
		{
			return false;
		}
		RectDepth = Mat(4 * pixelNum + 2, 3 * pixelNum + 2, CV_16U);
		RectRGB = Mat(4 * pixelNum + 2, 3 * pixelNum + 2, CV_8UC3);
		Mat tempDepth;
		depthFrame.copyTo(tempDepth);
		MatHelper::GetRectDepthMat(tempDepth, RectDepth, int(x - 1.5*pixelNum - 1), int(y - 0.7*pixelNum - 1), 3 * pixelNum + 2, 4 * pixelNum + 2);//width height 
		Mat tempRGB;
		rgbFrame.copyTo(tempRGB);
		MatHelper::GetRectMat(tempRGB, RectRGB, int(x - 1.5*pixelNum - 1), int(y - 0.7*pixelNum - 1), 3 * pixelNum + 2, 4 * pixelNum + 2);//width height 
		Mat rgbResized;
		Mat depthResized;

		cv::resize(RectDepth, RectDepth, cv::Size(50, 66));
		cv::resize(RectRGB, RectRGB, cv::Size(50, 66));
		vector<float> hotFeature = ForExp::getHoTfeatureDepth(RectRGB, RectDepth);
		Mat responseMat = Mat(1, 1121, CV_32FC1);
		for (int i = 0; i < hotFeature.size(); i++){
			responseMat.at<float>(0, i) = hotFeature[i];
			//cout << hotFeature[i] << " ";
		}
		//cout << endl;
		float response = cvSVM.predict(responseMat, true);

		cout << response << endl;
		if (response <= -0.3)
		{
			return true;
		}
		else{
			return false;
		}


	}

	//SVM classifier : HOT on Depth and Color
	static bool HOTRGBDconfirm(const Mat &depthFrame, const Mat &rgbFrame, int x, int y, int pixelNum){
		Mat RectDepth, RectRGB;
		CvSVM cvSVM;
		cvSVM.load("E:\\lab\\train\\HOTRGBD_10+18.xml");
		//if (int(x - 1.5*pixelNum - 1) < 0 || int(x + 1.5*pixelNum + 1) > 640 || int(y + 3.3*pixelNum + 1) > 480 || int(y - 0.7*pixelNum - 1) < 0)
		//{
		//	return false;
		//}
		RectDepth = Mat(4 * pixelNum + 2, 3 * pixelNum + 2, CV_16U);
		RectRGB = Mat(4 * pixelNum + 2, 3 * pixelNum + 2, CV_8UC3);
		Mat tempDepth;
		depthFrame.copyTo(tempDepth);
		MatHelper::GetRectDepthMat(tempDepth, RectDepth, int(x - 1.5*pixelNum - 1), int(y - 0.7*pixelNum - 1), 3 * pixelNum + 2, 4 * pixelNum + 2);//width height 
		Mat tempRGB;
		rgbFrame.copyTo(tempRGB);
		MatHelper::GetRectMat(tempRGB, RectRGB, int(x - 1.5*pixelNum - 1), int(y - 0.7*pixelNum - 1), 3 * pixelNum + 2, 4 * pixelNum + 2);//width height 
		Mat rgbResized;
		Mat depthResized;

		cv::resize(RectDepth, RectDepth, cv::Size(50, 66));
		cv::resize(RectRGB, RectRGB, cv::Size(50, 66));
		GaussianBlur(RectDepth, RectDepth, Size(5, 5), 1.5, 0);
		vector<float> hotFeature = ForExp::getHoTfeature(RectRGB, RectDepth);
		Mat responseMat = Mat(1, 1121, CV_32FC1);
		for (int i = 0; i < hotFeature.size(); i++){
			responseMat.at<float>(0, i) = hotFeature[i];
			//cout << hotFeature[i] << " ";
		}
		//cout << endl;
		float response = cvSVM.predict(responseMat, true);

		//cout << response << endl;
		if (response <= -0.0)
		{
			return true;
		}
		else{
			return false;
		}


	}

	//SVM classifier : Only JHCH
	static bool JHCHconfirm(const Mat &depthFrame, const Mat &pointCloud, const Mat &rgbFrame, int x, int y, int pixelNum){
		CvSVM cvSVM;
		cvSVM.load("E:\\lab\\train\\HS_38+47.xml");
		Mat RectDepth, RectRGB, RectCloud;
		if (int(x - 1.5*pixelNum - 1) < 0 || int(x + 1.5*pixelNum + 1) > 640 || int(y + 3.3*pixelNum + 1) > 480 || int(y - 0.7*pixelNum - 1) < 0)
		{
			return false;
		}
		RectDepth = Mat(4 * pixelNum + 2, 3 * pixelNum + 2, CV_16U);
		RectRGB = Mat(4 * pixelNum + 2, 3 * pixelNum + 2, CV_8UC3);
		Mat tempDepth;
		Mat tempRGB;
		depthFrame.copyTo(tempDepth);
		rgbFrame.copyTo(tempRGB);
		MatHelper::GetRectDepthMat(tempDepth, RectDepth, int(x - 1.5*pixelNum - 1), int(y - 0.7*pixelNum - 1), 3 * pixelNum + 2, 4 * pixelNum + 2);//width height 
		MatHelper::GetRectMat(tempRGB, RectRGB, int(x - 1.5*pixelNum - 1), int(y - 0.7*pixelNum - 1), 3 * pixelNum + 2, 4 * pixelNum + 2);//width height 
		//获取点云RECT
		RectCloud = Mat(4 * pixelNum + 2, 3 * pixelNum + 2, CV_32FC3);
		Mat tempCloud;
		pointCloud.copyTo(tempCloud);
		MatHelper::GetRectMatF(tempCloud, RectCloud, int(x - 1.5*pixelNum - 1), int(y - 0.7*pixelNum - 1), 3 * pixelNum + 2, 4 * pixelNum + 2);//width height 
		//确定头顶点位置
		Point2i rectHeadPoint = Point2i(x - int(x - 1.5*pixelNum - 1), y - int(y - 0.7*pixelNum - 1));
		RectDepth.at<unsigned short>(0, 0) = rectHeadPoint.x;
		RectDepth.at<unsigned short>(0, 1) = rectHeadPoint.y;
		Mat responseMat = Mat(1, 236, CV_32FC1);
		vector<float> JHCHFeature = ForExp::getJHCHfeatureHS(RectDepth, RectCloud, RectRGB);
		for (int i = 0; i < JHCHFeature.size(); i++){
			responseMat.at<float>(0, i) = JHCHFeature[i];
			//cout << hotFeature[i] << " ";
		}
		//cout << endl;

		float response = cvSVM.predict(responseMat, true);
		cout << response << endl;
		if (response<=-0.0015)
		{
			return true;
		}
		else{
			return false;
		}

	}

	//SVM classifier : HOT + JHCH
	static bool HOTJHCHconfirm(const Mat &depthFrame, const Mat &pointCloud, const Mat &rgbFrame, int x, int y, int pixelNum){
		CvSVM cvSVM;
		cvSVM.load("E:\\lab\\train\\HOTH_8.xml");
		Mat RectDepth, RectRGB, RectCloud;
		//if (int(x - 1.5*pixelNum - 1) < 0 || int(x + 1.5*pixelNum + 1) > 640 || int(y + 3.3*pixelNum + 1) > 480 || int(y - 0.7*pixelNum - 1) < 0)
		//{
		//	return false;
		//}
		RectDepth = Mat(4 * pixelNum + 2, 3 * pixelNum + 2, CV_16U);
		RectRGB = Mat(4 * pixelNum + 2, 3 * pixelNum + 2, CV_8UC3);
		Mat tempDepth;
		Mat tempRGB;
		depthFrame.copyTo(tempDepth);
		rgbFrame.copyTo(tempRGB);
		MatHelper::GetRectDepthMat(tempDepth, RectDepth, int(x - 1.5*pixelNum - 1), int(y - 0.7*pixelNum - 1), 3 * pixelNum + 2, 4 * pixelNum + 2);//width height 
		MatHelper::GetRectMat(tempRGB, RectRGB, int(x - 1.5*pixelNum - 1), int(y - 0.7*pixelNum - 1), 3 * pixelNum + 2, 4 * pixelNum + 2);//width height 
		//获取点云RECT
		RectCloud = Mat(4 * pixelNum + 2, 3 * pixelNum + 2, CV_32FC3);
		Mat tempCloud;
		pointCloud.copyTo(tempCloud);
		MatHelper::GetRectMatF(tempCloud, RectCloud, int(x - 1.5*pixelNum - 1), int(y - 0.7*pixelNum - 1), 3 * pixelNum + 2, 4 * pixelNum + 2);//width height 
		//确定头顶点位置
		Point2i rectHeadPoint = Point2i(x - int(x - 1.5*pixelNum - 1), y - int(y - 0.7*pixelNum - 1));
		RectDepth.at<unsigned short>(0, 0) = rectHeadPoint.x;
		RectDepth.at<unsigned short>(0, 1) = rectHeadPoint.y;

		//********************************************************************************************************
		Mat responseMat = Mat(1, 1120 + 11 * 5 + 1, CV_32FC1);
		vector<float> JHCHFeature = ForExp::getJHCHfeatureH(RectDepth, RectCloud, RectRGB);
		//**********************************************************************************************************

		resize(RectDepth, RectDepth, cv::Size(50, 66));
		resize(RectRGB, RectRGB, cv::Size(50, 66));
		vector<float> HOTFeature = ForExp::getHoTfeatureDepth(RectRGB, RectDepth);
		for (int i = 0; i < HOTFeature.size(); i++){
			responseMat.at<float>(0, i) = HOTFeature[i];
		}
		for (int i = HOTFeature.size(); i < JHCHFeature.size() + HOTFeature.size(); i++){
			responseMat.at<float>(0, i) = HOTFeature[i - HOTFeature.size()];
		}
		float response = cvSVM.predict(responseMat, true);
		cout << response << endl;
		if (response <= -1)
		{
			return true;
		}
		else{
			return false;
		}

	}
};