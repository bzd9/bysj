#include<opencv2/opencv.hpp>
#include<stack>
#include <vector>
using namespace cv;
class DDFCL{
private:
	Mat srcImg1, srcImg2;//定义了源图像1和源图像2
	int Radious = 380;//定义圆的半径为380
	Point m_center;
	int m_radius;
	Point g_center;                       //自聚焦透镜圆心
	std::vector<Vec4i> li;
	float a, b, c;             //台阶上直线系数
	int m_flag;
	bool Is8DegreeFace;                   //true--8度斜面，false--直角面
	Mat ddjc1;
	Mat ddjc2;
public:
	void setSrcImg1(Mat img);//通过方法设置源图像1
	void setSrcImg2(Mat img);//通过方法设置源图像2
	//程序运行第一步，首先从srcImg1中找出圆。
	bool CalCircle();
	//nMaxIter：最大迭代次数；nDiffRec：使用给定阀值确定的亮区与暗区平均灰度差异值 
	int IteratSegThreshold(Mat& img, int nMaxIter, int& iDiffRec); //阀值分割：迭代法 
    int	Calc3yuan2ciFCforCircle(Point& CTop, Point& CLeft, Point& CRight, int &radius, Point& center);
	int CalcZCXforCircle(Point& CTop, Point& CLeft, Point& CRight, int &radius, Point& center);
	bool GetIs8DegreeFace(void);
	int DefectDetectJXH(int num, Mat &ShowImg);//检测精细划痕
	Mat ImageFastFilter(Mat pImg, int t);//参数1，分割出来的图像。参数2，像素最小的检测。
	Mat reverseImg(Mat img);
	vector< vector<Point> > DDFCL::seedFilling(Mat& srcImg, Mat& dstImg, const ushort& minArea);
};