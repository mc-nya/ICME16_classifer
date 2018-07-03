

#ifndef ForExp_H
#define ForExp_H
#include "headfile.h"
#include"MatHelper.h"
class ForExp{
public: 

	//get gradient of image
	static void getGra(const Mat &image, Mat &Gra){
		Gra = Mat(66, 50, CV_32FC1);
		double gx, gy;
		for (int j = 1; j < image.rows - 1; j++){
			for (int i = 1; i < image.cols - 1; i++){
				gx = image.at<unsigned short>(j, i + 1) - image.at<unsigned short>(j, i - 1);
				gy = image.at<unsigned short>(j - 1, i) - image.at<unsigned short>(j + 1, i);
				Gra.at<float>(j, i) = sqrt(gx*gx + gy*gy);
				//cout << "gra" << endl;
			}
		}
	}

	//get gradient of image for diffent datatype
	static Mat getGra(const Mat &image){
		Mat Gra = Mat(66, 50, CV_32FC1);
		double gx, gy;
		for (int j = 1; j < image.rows - 1; j++){
			for (int i = 1; i < image.cols - 1; i++){
				gx = image.at<unsigned short>(j, i + 1) - image.at<unsigned short>(j, i - 1);
				gy = image.at<unsigned short>(j - 1, i) - image.at<unsigned short>(j + 1, i);
				Gra.at<float>(j, i) = sqrt(gx*gx + gy*gy);
				//cout << "gra" << endl;
			}
		}
		return Gra;
	}
	static Mat getGraB(const Mat &image){
		Mat Gra = Mat(66, 50, CV_32FC1);
		double gx, gy;
		for (int j = 1; j < image.rows - 1; j++){
			for (int i = 1; i < image.cols - 1; i++){
				gx = image.at<unsigned char>(j, i + 1) - image.at<unsigned char>(j, i - 1);
				gy = image.at<unsigned char>(j - 1, i) - image.at<unsigned char>(j + 1, i);
				Gra.at<float>(j, i) = sqrt(gx*gx + gy*gy);
				//cout << "gra" << endl;
			}
		}
		return Gra;
	}
	static Mat getGraF(const Mat &image){
		Mat Gra = Mat(66, 50, CV_32FC1);
		double gx, gy;
		for (int j = 1; j < image.rows - 1; j++){
			for (int i = 1; i < image.cols - 1; i++){
				gx = image.at<float>(j, i + 1) - image.at<float>(j, i - 1);
				gy = image.at<float>(j - 1, i) - image.at<float>(j + 1, i);
				Gra.at<float>(j, i) = sqrt(gx*gx + gy*gy);
				//cout << "gra" << endl;
			}
		}
		return Gra;
	}

	//get features in each bin     store result in ret
	static void computeBin(vector<float> &ret, const Mat &rawDepth, const Mat &depthGra, const Mat &rgbGra, const Mat &rgbGra2, int binNum, int y, int x, int j1, int i1, int j2, int i2){
		if (rawDepth.at<unsigned short>(y, x)>rawDepth.at<unsigned short>(y + j1, x + i1) && rawDepth.at<unsigned short>(y, x) > rawDepth.at<unsigned short>(y + j2, x + i2))
		{
			ret[binNum]++;
		}
		if (depthGra.at<float>(y, x) > depthGra.at<float>(y + j1, x + i1) && depthGra.at<float>(y, x) > depthGra.at<float>(y + j2, x + i2)){
			ret[8 + binNum]++;
		}
		if (rgbGra.at<float>(y, x) > rgbGra.at<float>(y + j1, x + i1) && rgbGra.at<float>(y, x) > rgbGra.at<float>(y + j2, x + i2)){
			ret[16 + binNum]++;
		}
		if (rgbGra2.at<float>(y, x) > rgbGra2.at<float>(y + j1, x + i1) && rgbGra2.at<float>(y, x) > rgbGra2.at<float>(y + j2, x + i2)){
			ret[24 + binNum]++;
		}
	}
	static void computeBin(vector<float> &ret, double &maxI, double &maxG, int &TmaxI, int &TmaxG, const Mat &image, const Mat &gra, int binNum, int y, int x, int j1, int i1, int j2, int i2){
		if (image.at<unsigned short>(y, x) > image.at<unsigned short>(y + j1, x + i1) && image.at<unsigned short>(y, x) > image.at<unsigned short>(y + j2, x + i2))
		{
			ret[binNum]++;
		}
		if (gra.at<float>(y, x) > gra.at<float>(y + j1, x + i1) && gra.at<float>(y, x) > gra.at<float>(y + j2, x + i2)){
			ret[16 + binNum]++;
		}
		double sumI = image.at<unsigned short>(y, x) + image.at<unsigned short>(y + j1, x + i1) + image.at<unsigned short>(y + j2, x + i2);
		double sumG = gra.at<float>(y, x) + gra.at<float>(y + j1, x + i1) + gra.at<float>(y + j2, x + i2);
		if (sumI > maxI){
			maxI = sumI;
			TmaxI = binNum;
		}
		if (sumG > maxG){
			maxG = sumG;
			TmaxG = binNum;
		}
	}


	//get features in each block 
	//collect all bin features and return a vector<float>
	static vector<float> blockfeature(const Mat &rawDepth, const Mat &depthGra, const Mat &rgbGra, const Mat &rgbGra2, int startj, int starti, int height, int width){
		vector<float> ret;
		ret.resize(32);
		double maxI, maxG;
		int TmaxI, TmaxG;
		for (int j = startj; j < startj + height; j++){
			for (int i = starti; i < starti + width; i++){
				maxI = -1; maxG = -1; TmaxI = -1; TmaxG = -1;
				//bin0
				computeBin(ret, rawDepth, depthGra, rgbGra, rgbGra2, 0, j, i, -1, -1, 1, 1);
				//bin1
				computeBin(ret, rawDepth, depthGra, rgbGra, rgbGra2, 1, j, i, -1, 1, 1, -1);
				//bin2			
				computeBin(ret, rawDepth, depthGra, rgbGra, rgbGra2, 2, j, i, 0, -1, 0, 1);
				//bin3			
				computeBin(ret, rawDepth, depthGra, rgbGra, rgbGra2, 3, j, i, -1, 0, 1, 0);
				//bin4			
				computeBin(ret, rawDepth, depthGra, rgbGra, rgbGra2, 4, j, i, 0, -1, -1, 0);
				//bin5			
				computeBin(ret, rawDepth, depthGra, rgbGra, rgbGra2, 5, j, i, -1, 0, 0, 1);
				//bin6			
				computeBin(ret, rawDepth, depthGra, rgbGra, rgbGra2, 6, j, i, 0, 1, 1, 0);
				//bin7			
				computeBin(ret, rawDepth, depthGra, rgbGra, rgbGra2, 7, j, i, 1, 0, 0, -1);
			}
		}
		return ret;
	}
	static vector<float> blockfeature(const Mat &image, const Mat &gra, int startj, int starti, int height, int width){
		vector<float> ret;
		ret.resize(32);
		double maxI, maxG;
		int TmaxI, TmaxG;
		for (int j = startj; j < startj + height; j++){
			for (int i = starti; i < starti + width; i++){
				maxI = -1; maxG = -1; TmaxI = -1; TmaxG = -1;
				//bin0
				computeBin(ret, maxI, maxG, TmaxI, TmaxG, image, gra, 0, j, i, -1, -1, 1, 1);
				//bin1
				computeBin(ret, maxI, maxG, TmaxI, TmaxG, image, gra, 1, j, i, -1, 1, 1, -1);
				//bin2
				computeBin(ret, maxI, maxG, TmaxI, TmaxG, image, gra, 2, j, i, 0, -1, 0, 1);
				//bin3
				computeBin(ret, maxI, maxG, TmaxI, TmaxG, image, gra, 3, j, i, -1, 0, 1, 0);
				//bin4
				computeBin(ret, maxI, maxG, TmaxI, TmaxG, image, gra, 4, j, i, 0, -1, -1, 0);
				//bin5
				computeBin(ret, maxI, maxG, TmaxI, TmaxG, image, gra, 5, j, i, -1, 0, 0, 1);
				//bin6
				computeBin(ret, maxI, maxG, TmaxI, TmaxG, image, gra, 6, j, i, 0, 1, 1, 0);
				//bin7
				computeBin(ret, maxI, maxG, TmaxI, TmaxG, image, gra, 7, j, i, 1, 0, 0, -1);
				ret[8 + TmaxI]++;
				ret[24 + TmaxG]++;
			}
		}
		return ret;
	}

	//get final HOT feature 
	//collect all block features and return a vector<float>
	static vector<float> getHoTfeature(const Mat &rowImage){
		Mat image;
		Mat gra;
		cv::resize(rowImage, image, cv::Size(50, 66));
		getGra(image, gra);
		//imshow("test", image);
		//imshow("test2", gra);
		//cvWaitKey();
		vector<float>feature;
		vector<float>temp;
		feature.resize(1121);
		for (int j = 1; j < (image.rows - 10); j = j + 8){
			for (int i = 1; i < (image.cols - 10); i = i + 8){
				//cout << image.rows << "  " << image.cols << endl;
				//cout <<"out  "<< j << "  " << i << endl;
				temp = blockfeature(image, gra, j, i, 16, 16);
				for (int k = 0; k < 32; k++){
					feature[(((j - 1) / 8) * 5 + (i - 1) / 8) * 32 + k] = temp[k];
					//cout << "in  " << (((j - 1) / 8) * 5 + (i - 1) / 8) * 32 + k << "  " << temp[k] << endl;
				}
			}
		}
		return feature;
	}
	static vector<float> getHoTfeature(const Mat &rawRGB, const Mat &rawDepth){
		Mat grayImage;
		Mat depthGra, rgbGra, rgbGra2;
		cvtColor(rawRGB, grayImage, CV_RGB2GRAY);
		depthGra = getGra(rawDepth);
		rgbGra = getGraB(grayImage);
		rgbGra2 = getGraF(rgbGra);
		//imshow("test", rawRGB);
		//imshow("test2", rawDepth);
		//imshow("test3", rgbGra2);
		//cvWaitKey(200);
		vector<float>feature;
		vector<float>temp;
		feature.resize(1121);
		for (int j = 1; j < (rawRGB.rows - 10); j = j + 8){
			for (int i = 1; i < (rawRGB.cols - 10); i = i + 8){
				//cout << image.rows << "  " << image.cols << endl;
				//cout <<"out  "<< j << "  " << i << endl;
				temp = blockfeature(rawDepth, depthGra, rgbGra, rgbGra2, j, i, 16, 16);
				for (int k = 0; k < 32; k++){
					feature[(((j - 1) / 8) * 5 + (i - 1) / 8) * 32 + k] = temp[k];
					//cout << "in  " << (((j - 1) / 8) * 5 + (i - 1) / 8) * 32 + k << "  " << temp[k] << endl;
				}
			}
		}
		return feature;
	}


	//nearly the same functions above, but only use on depth image
	static void computeBinDepth1(vector<float> &ret, double &maxD0, double &maxD1, int &TmaxD0, int &TmaxD1, const Mat &rawDepth, const Mat &depthGra, int binNum, int y, int x, int j1, int i1, int j2, int i2){
		if (rawDepth.at<unsigned short>(y, x)>rawDepth.at<unsigned short>(y + j1, x + i1) && rawDepth.at<unsigned short>(y, x)>rawDepth.at<unsigned short>(y + j2, x + i2))
		{
			ret[binNum]++;
		}
		if (depthGra.at<float>(y, x) > depthGra.at<float>(y + j1, x + i1) && depthGra.at<float>(y, x) > depthGra.at<float>(y + j2, x + i2)){
			ret[16 + binNum]++;
		}
		double sumD0 = rawDepth.at<unsigned short>(y, x) + rawDepth.at<unsigned short>(y + j1, x + i1) + rawDepth.at<unsigned short>(y + j2, x + i2);
		double sumD1 = depthGra.at<float>(y, x) + depthGra.at<float>(y + j1, x + i1) + depthGra.at<float>(y + j2, x + i2);
		if (sumD0 > maxD0){
			maxD0 = sumD0;
			TmaxD0 = binNum;
		}
		if (sumD1 > maxD1){
			maxD1 = sumD1;
			TmaxD1 = binNum;
		}
	}
	static vector<float> getHoTfeatureDepth(const Mat &rawRGB, const Mat &rawDepth){
		Mat grayImage;
		Mat depthGra, rgbGra, rgbGra2, depthGra2;
		cvtColor(rawRGB, grayImage, CV_RGB2GRAY);
		depthGra = getGra(rawDepth);
		depthGra2 = getGraF(depthGra);
		rgbGra = getGraB(grayImage);
		rgbGra2 = getGraF(rgbGra);
		vector<float>feature;
		vector<float>temp;
		feature.resize(1120);
		for (int j = 1; j < (rawRGB.rows - 10); j = j + 8){
			for (int i = 1; i < (rawRGB.cols - 10); i = i + 8){
				//cout << image.rows << "  " << image.cols << endl;
				//cout <<"out  "<< j << "  " << i << endl;
				temp = blockfeatureDepth1(rawDepth, depthGra, j, i, 16, 16);
				for (int k = 0; k < 32; k++){
					feature[(((j - 1) / 8) * 5 + (i - 1) / 8) * 32 + k] = temp[k];
					//cout << "in  " << (((j - 1) / 8) * 5 + (i - 1) / 8) * 32 + k << "  " << temp[k] << endl;
				}
			}
		}
		return feature;
	}
	static vector<float> blockfeatureDepth1(const Mat &rawDepth, const Mat &depthGra, int startj, int starti, int height, int width){
		vector<float> ret;
		ret.resize(32);
		double maxD0, maxD1;
		int TmaxD0, TmaxD1;
		for (int j = startj; j < startj + height; j++){
			for (int i = starti; i < starti + width; i++){
				maxD0 = -1; maxD1 = -1; TmaxD0 = -1; TmaxD1 = -1;
				//bin0
				computeBinDepth1(ret, maxD0, maxD1, TmaxD0, TmaxD1, rawDepth, depthGra, 0, j, i, -1, -1, 1, 1);
				//bin1
				computeBinDepth1(ret, maxD0, maxD1, TmaxD0, TmaxD1, rawDepth, depthGra, 1, j, i, -1, 1, 1, -1);
				//bin2			
				computeBinDepth1(ret, maxD0, maxD1, TmaxD0, TmaxD1, rawDepth, depthGra, 2, j, i, 0, -1, 0, 1);
				//bin3			
				computeBinDepth1(ret, maxD0, maxD1, TmaxD0, TmaxD1, rawDepth, depthGra, 3, j, i, -1, 0, 1, 0);
				//bin4			
				computeBinDepth1(ret, maxD0, maxD1, TmaxD0, TmaxD1, rawDepth, depthGra, 4, j, i, 0, -1, -1, 0);
				//bin5			
				computeBinDepth1(ret, maxD0, maxD1, TmaxD0, TmaxD1, rawDepth, depthGra, 5, j, i, -1, 0, 0, 1);
				//bin6			
				computeBinDepth1(ret, maxD0, maxD1, TmaxD0, TmaxD1, rawDepth, depthGra, 6, j, i, 0, 1, 1, 0);
				//bin7			
				computeBinDepth1(ret, maxD0, maxD1, TmaxD0, TmaxD1, rawDepth, depthGra, 7, j, i, 1, 0, 0, -1);
				ret[8 + TmaxD0]++;
				ret[24 + TmaxD1]++;
			}
		}
		return ret;
	}


	//get JHCH features
	static vector<float> getJHCHfeatureHS(Mat &rawDepth, Mat &rawCloud, Mat &rawRGB){
		Point2i head = Point2i(rawDepth.at<unsigned short>(0, 0), rawDepth.at<unsigned short>(0, 1));
		vector<float> feature;
		feature.resize(236);
		Mat HSV;
		int totalPoint = 0;
		cvtColor(rawRGB, HSV, CV_RGB2HSV);
		for (int j = 0; j < rawDepth.rows; j++){
			for (int i = 0; i < rawDepth.cols; i++){
				double dist = MatHelper::GetDist(rawCloud, head, Point2i(i, j));
				int hue = HSV.at<cv::Vec3b>(j, i)[0];
				int sat = HSV.at<cv::Vec3b>(j, i)[1];
				int val = HSV.at<cv::Vec3b>(j, i)[2];
				// Hue取值0,1,2,3,4,5,6,7,8
				hue = (hue == 180) ? 179 : hue;
				hue = hue / 20;    // 20=180/9
				// Saturation取值0,1,2,3,4
				sat = sat / 52;    // 52=256/5
				if (dist < 500){
					totalPoint++;
					int pos = (((int)dist) / 100) * 47;
					int extend;
					if (val < 40) // 黑色做特殊处理
					{
						extend = 45;
					}
					else if (val > 230 && sat < 30) // 白色做特殊处理
					{
						extend = 46;
					}
					else
					{
						extend = hue * 5 + sat;
					}
					feature[pos + extend]++;
				}
			}
		}
		for (int i = 0; i < 235; i++){
			feature[i] = feature[i] / ((double)totalPoint);
		}
		return feature;
	}
	static vector<float> getJHCHfeatureH(Mat &rawDepth, Mat &rawCloud, Mat &rawRGB){
		Point2i head = Point2i(rawDepth.at<unsigned short>(0, 0), rawDepth.at<unsigned short>(0, 1));
		vector<float> feature;
		feature.resize(56);
		Mat HSV;
		int totalPoint = 0;
		cvtColor(rawRGB, HSV, CV_RGB2HSV);
		for (int j = 0; j < rawDepth.rows; j++){
			for (int i = 0; i < rawDepth.cols; i++){
				double dist = MatHelper::GetDist(rawCloud, head, Point2i(i, j));
				int hue = HSV.at<cv::Vec3b>(j, i)[0];
				int sat = HSV.at<cv::Vec3b>(j, i)[1];
				int val = HSV.at<cv::Vec3b>(j, i)[2];
				// Hue取值0,1,2,3,4,5,6,7,8
				hue = (hue == 180) ? 179 : hue;
				hue = hue / 20;    // 20=180/9
				// Saturation取值0,1,2,3,4
				sat = sat / 52;    // 52=256/5
				if (dist < 500){
					totalPoint++;
					int pos = (((int)dist) / 100) * 11;
					int extend;
					if (val < 40) // 黑色做特殊处理
					{
						extend = 9;
					}
					else if (val > 230 && sat < 30) // 白色做特殊处理
					{
						extend = 10;
					}
					else
					{
						extend = hue;
					}
					feature[pos + extend]++;
				}
			}
		}
		for (int i = 0; i < 55; i++){
			feature[i] = feature[i] / ((double)totalPoint);
		}
		return feature;
	}
	static void fillEdge(const Mat &img, Mat &result){
		result = Mat(480, 640, CV_8U);
		result = img.clone();
		//for (int j = 1; j < img.rows; j++)		//效果测试
		//{
		//	for (int i = 1; i < img.cols; i++)
		//	{
		//		cout << int(img.at<unsigned short>(j, i)) << " ";
		//	}
		//	cout << endl;
		//}
		for (int j = 2; j < (img.rows - 4); ++j){
			for (int i = 2; i < (img.cols - 4); ++i){
				//cout << "1" << j << " " << i << endl;
				if (img.at<unsigned short>(j, i)<800 || img.at<unsigned short>(j, i)>7000){
					int countover = 0;
					int countin = 0;
					for (int dy = -1; dy < 2; dy++){
						for (int dx = -1; dx < 2; dx++){
							//cout << "2" << j << " " << i << " " << dy << " " << dx << " " << endl;
							if (dx == 0 && dy == 0) continue;
							if (img.at<unsigned short>(j + dy, i + dx)<800 || img.at<unsigned short>(j + dy, i + dx)>7000)
								countover += 1;
							else{
								countin += 1;
								result.at<unsigned short>(j, i) = img.at<unsigned short>(j, i);
							}

						}
					}
					if (countover >= 5)
						result.at<unsigned short>(j, i) = 0;
				}
			}
		}
	}
};

#endif