#include<opencv2/opencv.hpp>
using namespace cv;
#define PI 3.1415926 
class LineFinder
{
private:
	//Դͼ��  
	Mat img;
	//�����Ž������������  
	std::vector<Vec4i> lines;
	//���۵ķֱ���  
	double deltaRho;
	double deltaTheta;
	//�ж�Ϊ�ߵ���С������  
	int minVote;
	//�ߵ���С����  
	double minLength;
	//��������������  
	double maxGap;
public:
	//Ĭ�����������Ϊ��1�����أ�1��Ϊ�뾶������û�м��Ҳû����С����  
	LineFinder() :deltaRho(1), deltaTheta(PI / 180), minVote(10), minLength(0.), maxGap(0.){}

	//************��ص����ú���************//  
	//���û������ķֱ���  
	void setAccResolution(double dRho, double dTheta);

	//������С��ͶƱ��  
	void setMinVote(int minv);

	//�����߳��ͼ��  
	void setLineLengthAndGap(double length, double gap);

	//��װ�ĸ���huogh�任����  
	std::vector<Vec4i> findLines(Mat &binary);

	//��ͼ���ϻ���������  
	void drawDetectedLines(Mat &image, Scalar color = Scalar(0, 0, 255));
};