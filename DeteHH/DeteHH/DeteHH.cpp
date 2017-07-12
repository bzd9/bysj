// DeteHH.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include"DDFCL.h"

int _tmain(int argc, _TCHAR* argv[])
{
	DDFCL df;
	Mat img=imread("E:\\testImg\\20160111-1\\a\\P01101524401.jpg");
	Mat img1 = imread("E:\\testImg\\20160111-1\\a\\P01101524403.jpg");
	df.setSrcImg1(img);
	df.setSrcImg2(img1);
	df.CalCircle();//计算是不是圆
	df.GetIs8DegreeFace();//计算是不是八度斜面
	df.DefectDetectJXH(1,img1);
	waitKey(0);
	return 0;
}

 