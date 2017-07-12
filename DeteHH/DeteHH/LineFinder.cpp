#include"LineFinder.h"
void LineFinder:: setAccResolution(double dRho, double dTheta)
{
	deltaRho = dRho;
	deltaTheta = dTheta;
}

//设置最小的投票数  
void LineFinder::setMinVote(int minv)
{
	minVote = minv;
}

//设置线长和间隔  
void LineFinder::setLineLengthAndGap(double length, double gap)
{
	minLength = length;
	maxGap = gap;
}

//封装的概率huogh变换程序  
std::vector<Vec4i> LineFinder::findLines(Mat &binary)
{
	lines.clear();
	HoughLinesP(binary, lines, deltaRho, deltaTheta, minVote, minLength, maxGap);
	return lines;
}

//在图像上画出检测的线  
void LineFinder::drawDetectedLines(Mat &image, Scalar color)
{
	//画线  
	color = Scalar(0,0,255);
	std::vector<Vec4i>::const_iterator it2 = lines.begin();
	while (it2 != lines.end())
	{
		Point pt1((*it2)[0], (*it2)[1]);
		Point pt2((*it2)[2], (*it2)[3]);
		line(image, pt1, pt2, color, 2);
		++it2;
	}
}