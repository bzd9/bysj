#include<opencv2/opencv.hpp>
#include<stack>
#include <vector>
using namespace cv;
class DDFCL{
private:
	Mat srcImg1, srcImg2;//������Դͼ��1��Դͼ��2
	int Radious = 380;//����Բ�İ뾶Ϊ380
	Point m_center;
	int m_radius;
	Point g_center;                       //�Ծ۽�͸��Բ��
	std::vector<Vec4i> li;
	float a, b, c;             //̨����ֱ��ϵ��
	int m_flag;
	bool Is8DegreeFace;                   //true--8��б�棬false--ֱ����
	Mat ddjc1;
	Mat ddjc2;
public:
	void setSrcImg1(Mat img);//ͨ����������Դͼ��1
	void setSrcImg2(Mat img);//ͨ����������Դͼ��2
	//�������е�һ�������ȴ�srcImg1���ҳ�Բ��
	bool CalCircle();
	//nMaxIter��������������nDiffRec��ʹ�ø�����ֵȷ���������밵��ƽ���ҶȲ���ֵ 
	int IteratSegThreshold(Mat& img, int nMaxIter, int& iDiffRec); //��ֵ�ָ������ 
    int	Calc3yuan2ciFCforCircle(Point& CTop, Point& CLeft, Point& CRight, int &radius, Point& center);
	int CalcZCXforCircle(Point& CTop, Point& CLeft, Point& CRight, int &radius, Point& center);
	bool GetIs8DegreeFace(void);
	int DefectDetectJXH(int num, Mat &ShowImg);//��⾫ϸ����
	Mat ImageFastFilter(Mat pImg, int t);//����1���ָ������ͼ�񡣲���2��������С�ļ�⡣
	Mat reverseImg(Mat img);
	vector< vector<Point> > DDFCL::seedFilling(Mat& srcImg, Mat& dstImg, const ushort& minArea);
};