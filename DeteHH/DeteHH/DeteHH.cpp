// DeteHH.cpp : �������̨Ӧ�ó������ڵ㡣
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
	df.CalCircle();//�����ǲ���Բ
	df.GetIs8DegreeFace();//�����ǲ��ǰ˶�б��
	df.DefectDetectJXH(1,img1);
	waitKey(0);
	return 0;
}

 