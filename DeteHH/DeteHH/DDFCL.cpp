#include"DDFCL.h"
#include"LineFinder.h"
using namespace std;
void DDFCL::setSrcImg1(Mat img){   //����Դͼ��1
	srcImg1 = img;
}
void DDFCL::setSrcImg2(Mat img){  //����Դͼ��2
	cvtColor(img,srcImg2,CV_RGB2GRAY);
}
bool DDFCL::CalCircle(){   //��srcImg1���ҳ�Բ������ҵ�����true,����Ҳ����򷵻�false
	Mat YSGrayImg(srcImg1.size().height, srcImg1.size().width, CV_8U, Scalar(0));
	cvtColor(srcImg1, YSGrayImg, CV_BGR2GRAY);          //�Ҷ�ͼ��  CV_BGR2GRAY
	int m_w = YSGrayImg.size().width;//�õ�ͼ��Ŀ�
	int m_h = YSGrayImg.size().height;//�õ�ͼ��ĸ�
	Mat MatImg8u(YSGrayImg.size().height, YSGrayImg.size().width, CV_8U, Scalar(0));    //�ָ��ͼ��Ķ���
	int iDiffRec = 0;
	int th = IteratSegThreshold(YSGrayImg, 5, iDiffRec);
	if (th>95)
	{
		cv::threshold(YSGrayImg, MatImg8u, th - 40, 255, CV_THRESH_BINARY);
	}
	else if (th >= 85 && th<95)
		cv::threshold(YSGrayImg, MatImg8u, th - 40, 255, CV_THRESH_BINARY);
	else if (th >= 65 && th<85)
		cv::threshold(YSGrayImg, MatImg8u, th - 10, 255, CV_THRESH_BINARY);
	else if (th >= 50 && th<65)
		cv::threshold(YSGrayImg, MatImg8u, th, 255, CV_THRESH_BINARY);
	else if (th >= 40 && th<50)
		cv::threshold(YSGrayImg, MatImg8u, th - 10, 255, CV_THRESH_BINARY);
	else if (th>30 && th<40)
		cv::threshold(YSGrayImg, MatImg8u, th - 5, 255, CV_THRESH_BINARY);
	/*namedWindow("����ֵ�ָ�ͼ��", 0);
	imshow("����ֵ�ָ�ͼ��", MatImg8u);*/
	std::vector<std::vector<Point>> contours;
	std::vector<Vec4i> hierarchy;
	//��������
	cv::findContours(MatImg8u, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));  //CV_RETR_EXTERNAL   CV_RETR_TREE
	std::vector<std::vector<Point>> contours_poly(contours.size());          //���ƺ�������㼯
	std::vector<Point2f>center(contours.size());                               //��Χ�㼯����СԲ��vector
	std::vector<float>radius(contours.size());
	std::vector<float>m_area(contours.size());
	int flag = -100;
	int min = 100;
	for (int i = 0; i < contours.size(); i++)
	{
		if (contours[i].size()>300)//()ֻ�е���������300����ʱ�Ŷ��������и��١�
		{
			/*if (i == 144)
			int aa = 0;*/
			//minRect[i] = minAreaRect(Mat(contours[i]));
			approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);      //�Զ�����������ʵ����ƣ�contours_poly[i]������Ľ��Ƶ㼯
			minEnclosingCircle(contours_poly[i], center[i], radius[i]);     //���㲢���ذ�Χ�����㼯����СԲ�μ���뾶
			m_area[i] = contourArea(Mat(contours[i]));
			if ((int)radius[i] > 320 && (int)radius[i] <400 && center[i].x<1000)    //б�棨������û����
				if (abs((int)radius[i] - Radious) < min && center[i].x<1000)    //б�棬
				{
				flag = i;
				min = abs((int)radius[i] - Radious);//�뾶��380�Ĳ�ֵ����¼��380����С��ֵ
				}
		}
	}
	if (flag == -100)
	{
		return false;
	}
	else{
		m_center = center[flag];//������380��ӽ��İ뾶ΪԲ
		m_radius = radius[flag];
	}
	circle(YSGrayImg, m_center, m_radius, cv::Scalar(255, 255, 255));//����Բ�Ĵ��λ��
	/*namedWindow("Բ�Ĵ��λ��",0);
	imshow("Բ�Ĵ��λ��",YSGrayImg);*/
	/*********************************Բ�ľ�ϸ��λ*******************************************************/
	float PII = 3.14159;
	int avg = 0;
	int angle = 0;
	for (int i = 0; i < 8; i++)
	{
		avg += YSGrayImg.at<uchar>(m_center.y + 250 * sin(angle), m_center.x + 250 * cos(angle));
		angle += PII / 4.0;
	}
	avg /= 8;//�����ƽ������,Ŀ�������ƽ������
	int BGavg = 0;     //����ƽ������
	BGavg = (YSGrayImg.at<uchar>(1000, 1300) + YSGrayImg.at<uchar>(10, 1300)) / 2.0;//���������ƽ������
	//std::cout << "Բ�����Ⱦ�ֵ: " << avg << "   ";
	//��ȷ��λԲ,ʹ����������Բ��4�����������ݶ������
	int maxGrad = 0;//���������ݶ�ֵ
	Point Left;
	int last = 0;
	int first = 0;//�߽�ĻҶ�ֵ

	if (BGavg < 25)     //ʹ��С�ع�ʱ�䣬���ݱ����������ж��ع�ʱ��ĳ���
	{
		if (avg >= 170)   //��
			first = 30;
		else if (avg < 170 && avg >= 140)
			first = 25;
		else if (avg < 140 && avg >= 70)
			first = 25;
		else if (avg>20 && avg < 70)
			first = 30;
	}
	if (BGavg >= 25)     //ʹ�ô��ع�ʱ��
	{
		if (avg >= 160)
			first = 80;
		else if (avg < 160 && avg >= 100)
			first = 70;
		else if (avg < 100 && avg >= 70)
			first = 40;
		else if (avg>40 && avg < 70)
			first = 20;
	}
	//���ĸ���������������ҵ��ݶ�����λ��
	//Left
	for (int x = (int)(m_center.x - m_radius + 50); x >(int)(m_center.x - m_radius - 50); x--)//������������
	{
		if (x == 2)//�����̫���
			break;
		if (x == (int)(m_center.x - m_radius + 50))
		{
			last = YSGrayImg.at<uchar>(m_center.y, x);//last���ڲ�����㣬Բ����һ���㣬�պú��������귳��
		}
		else
		{
			int f = YSGrayImg.at<uchar>(m_center.y, x - 2);//fΪ�ô�������ֵ
			if (maxGrad >(f - last) && f < first)//�ݶ�����������
			{
				maxGrad = f - last;
				Left = cv::Point(x, m_center.y);
			}
			last = f;
		}
	}
	maxGrad = 0;
	Point Right;
	last = 0;
	//Right
	for (int x = (int)(m_center.x + m_radius - 50); x < (int)(m_center.x + m_radius + 50); x++)
	{
		if (x > YSGrayImg.size().width - 1)
			break;
		if (x == (int)(m_center.x + m_radius - 50))
		{
			last = YSGrayImg.at<uchar>(m_center.y, x + 2);
		}
		else
		{
			int f = YSGrayImg.at<uchar>(m_center.y, x);
			if (maxGrad > (f - last) && f < first)
			{
				maxGrad = f - last;
				Right = cv::Point(x, m_center.y);
			}
			last = f;
		}
	}
	maxGrad = 0;
	Point Top;
	last = 0;
	//Top
	for (int y = (int)(m_center.y - m_radius + 50); y > (int)(m_center.y - m_radius - 50); y--)
	{
		if (y < 2)
			break;
		if (y == (int)(m_center.y - m_radius + 50))
		{
			last = YSGrayImg.at<uchar>(y, m_center.x);
		}
		else
		{
			int f = YSGrayImg.at<uchar>(y - 2, m_center.x);
			if (maxGrad >(f - last) && f < first)
			{
				maxGrad = f - last;
				Top = cv::Point(m_center.x, y);
			}
			last = f;
		}
	}
	maxGrad = 0;
	Point Bottom;
	last = 0;
	//Bottom
	for (int y = (int)(m_center.y + m_radius - 50); y < (int)(m_center.y + m_radius + 50); y++)
	{
		if (y > YSGrayImg.size().height - 1)
			break;
		if (y == (int)(m_center.y + m_radius - 50))
		{
			last = YSGrayImg.at<uchar>(y, m_center.x);
		}
		else
		{
			int f = YSGrayImg.at<uchar>(y + 2, m_center.x);
			if (maxGrad > (f - last) && f < first)
			{
				maxGrad = f - last;
				Bottom = cv::Point(m_center.x, y);
			}
			last = f;
		}
	}
	//ͨ������ķ����ҵ����ݶȲ������ĸ�������ĸ���
	if ((Left.x == 0 && Left.y == 0) || (Right.x == 0 && Right.y == 0) || (Top.x == 0 && Top.y == 0) || (Bottom.x == 0 && Bottom.y == 0))
	{
		//����ɨ�跨�����ϡ���3�������ҳ��Ӱ������ݶȱ仯������ΪԲ����3���㣬�������ĺͰ뾶
		int CmaxGrad = 0;
		Point CLeft;
		int Clast = 0;

		//Left
		for (int x = 50; x <m_w / 3.0; x++)
		{
			if (x == 50)
			{
				Clast = YSGrayImg.at<uchar>(m_h / 2.0, x);
			}
			else
			{
				int f = YSGrayImg.at<uchar>(m_h / 2.0, x);
				if (CmaxGrad <(f - Clast))
				{
					CmaxGrad = f - Clast;
					CLeft = cv::Point(x, m_h / 2.0);
					if (CmaxGrad>5)
						break;
				}
				Clast = f;
			}
		}
		CmaxGrad = 0;
		Point CRight;
		Clast = 0;

		//Right
		for (int x = m_w - 50; x >m_w*2.0 / 3.0; x--)
		{
			if (x == m_w - 50)
			{
				Clast = YSGrayImg.at<uchar>(m_h / 2.0, x);
			}
			else
			{
				int f = YSGrayImg.at<uchar>(m_h / 2.0, x);
				if (CmaxGrad <(f - Clast))
				{
					CmaxGrad = f - Clast;
					CRight = cv::Point(x, m_h / 2.0);
					if (CmaxGrad>5)
						break;
				}
				Clast = f;
			}
		}
		CmaxGrad = 0;
		Point CTop;
		Clast = 0;

		//Top
		for (int y = 50; y <m_h / 3.0; y++)
		{
			if (y == 50)
			{
				Clast = YSGrayImg.at<uchar>(y, m_w / 2.0);
			}
			else
			{
				int f = YSGrayImg.at<uchar>(y, m_w / 2.0);
				if (CmaxGrad <(f - Clast))
				{
					CmaxGrad = f - Clast;
					CTop = cv::Point(m_w / 2.0, y);
					if (CmaxGrad>5)
						break;
				}
				Clast = f;
			}
		}
		CmaxGrad = 0;
		Point CBottom;
		Clast = 0;

		//Bottom
		for (int y = m_h - 50; y >m_h / 3.0; y--)
		{
			if (y == m_h - 50)
			{
				Clast = YSGrayImg.at<uchar>(y, m_w / 2.0);
			}
			else
			{
				int f = YSGrayImg.at<uchar>(y, m_w / 2.0);
				if (CmaxGrad <(f - Clast))
				{
					CmaxGrad = f - Clast;
					CBottom = cv::Point(m_w / 2.0, y);
					if (CmaxGrad>5)
						break;
				}
				Clast = f;
			}
		}//Ϊʲô���������������㣿
		Point m_center1;
		int m_radius1 = 0;
		Calc3yuan2ciFCforCircle(CTop, CLeft, CRight, m_radius1, m_center1);
		if (abs(m_radius1 - 328) < abs(m_radius - 328))
		{
			m_center = m_center1;
			m_radius = m_radius1;
		}
	}
	else
	{//////////////////////////////////8.9/////////////////////////////////
		//��Բ�İ뾶
		int LR = 0, RR = 0, TR = 0, BR = 0, SumR = 0;
		int lr = 0, rr = 0, tr = 0, br = 0;
		LR = sqrt((Left.x - m_center.x)*(Left.x - m_center.x) + (Left.y - m_center.y)*(Left.y - m_center.y));
		lr = abs(m_radius - sqrt((Left.x - m_center.x)*(Left.x - m_center.x) + (Left.y - m_center.y)*(Left.y - m_center.y)));
		rr = abs(m_radius - sqrt((Right.x - m_center.x)*(Right.x - m_center.x) + (Right.y - m_center.y)*(Right.y - m_center.y)));
		RR = sqrt((Right.x - m_center.x)*(Right.x - m_center.x) + (Right.y - m_center.y)*(Right.y - m_center.y));
		tr = abs(m_radius - sqrt((Top.x - m_center.x)*(Top.x - m_center.x) + (Top.y - m_center.y)*(Top.y - m_center.y)));
		TR = sqrt((Top.x - m_center.x)*(Top.x - m_center.x) + (Top.y - m_center.y)*(Top.y - m_center.y));
		br = abs(m_radius - sqrt((Bottom.x - m_center.x)*(Bottom.x - m_center.x) + (Bottom.y - m_center.y)*(Bottom.y - m_center.y)));
		BR = sqrt((Bottom.x - m_center.x)*(Bottom.x - m_center.x) + (Bottom.y - m_center.y)*(Bottom.y - m_center.y));
		//����ȡ��λ��
		std::vector<int> sort(4);
		sort.at(0) = lr;

		if (rr > sort.at(0) || rr == sort.at(0))
			sort.at(1) = rr;
		else
		{
			sort.at(1) = sort.at(0);
			sort.at(0) = rr;
		}

		if (tr > sort.at(0))
		{
			if (tr > sort.at(1) || tr == sort.at(1))
				sort.at(2) = tr;
			else
			{
				sort.at(2) = sort.at(1);
				sort.at(1) = tr;
			}
		}
		else if (tr == sort.at(0))
		{
			sort.at(2) = sort.at(1);
			sort.at(1) = tr;
		}
		else
		{
			sort.at(2) = sort.at(1);
			sort.at(1) = sort.at(0);
			sort.at(0) = tr;
		}
		if (br > sort.at(0))
		{
			if (br > sort.at(1))
			{
				if (br > sort.at(2) || br == sort.at(2))
					sort.at(3) = br;
				else
				{
					sort.at(3) = sort.at(2);
					sort.at(2) = br;
				}
			}
			else if (br == sort.at(1))
			{
				sort.at(3) = sort.at(2);
				sort.at(2) = br;
			}
			else
			{
				sort.at(3) = sort.at(2);
				sort.at(2) = sort.at(1);
				sort.at(1) = br;
			}
		}
		else if (BR == sort.at(0))
		{
			sort.at(3) = sort.at(2);
			sort.at(2) = sort.at(1);
			sort.at(1) = br;
		}
		else
		{
			sort.at(3) = sort.at(2);
			sort.at(2) = sort.at(1);
			sort.at(1) = sort.at(0);
			sort.at(0) = br;
		}
		//int  zws = (sort.at(0) + sort.at(1)) / 2.0 + m_radius;
		//���ĸ������������
		int zws1 = 0, zws2 = 0;
		if (sort.at(0) == lr)
			zws1 = LR;
		if (sort.at(0) == rr)
			zws1 = RR;
		if (sort.at(0) == tr)
			zws1 = TR;
		if (sort.at(0) == br)
			zws1 = BR;
		if (sort.at(1) == lr)
			zws2 = LR;
		if (sort.at(1) == rr)
			zws2 = RR;
		if (sort.at(1) == tr)
			zws2 = TR;
		if (sort.at(1) == br)
			zws2 = BR;
		int  zws = (zws1 + zws2) / 2.0;

		Point p1, p2, p3, p4;//�����ĸ���
		std::vector<Point> sp;
		int js = 0;
		if (abs(LR - Radious) < 5 || (LR > (zws - 8) && LR < (zws + 21) && abs(LR - Radious) < 14))
		{
			p1 = Left;
			js++;
			SumR += LR;
			sp.push_back(Left);
		}
		if (abs(RR - Radious) < 5 || (RR > (zws - 8) && RR < (zws + 21) && abs(RR - Radious) < 14))
		{
			if (p1.x != 0 && p1.y != 0)//Ϊʲô��p1
				p2 = Right;
			else
				p1 = Right;
			SumR += RR;
			js++;
			sp.push_back(Right);
		}
		if (abs(TR - Radious) < 5 || (TR > (zws - 8) && TR < (zws + 21) && abs(TR - Radious) < 14))
		{
			if (p1.x != 0 && p1.y != 0)
			{
				if (p2.x != 0 && p2.y != 0)
					p3 = Top;
				else
					p2 = Top;
			}
			else
				p1 = Top;
			SumR += TR;
			js++;
			sp.push_back(Top);
		}
		if (abs(BR - Radious) < 5 || (BR > (zws - 8) && BR < (zws + 21) && abs(BR - Radious) < 14))
		{
			if (p1.x != 0 && p1.y != 0)
			{
				if (p2.x != 0 && p2.y != 0)
				{
					if (p3.x == 0 && p3.y == 0)
						p3 = Bottom;
					else
						p4 = Bottom;
				}
				else
					p2 = Bottom;
			}
			else
				p1 = Bottom;
			SumR += BR;
			js++;
			sp.push_back(Bottom);
		}
		//�жϷ��������ĵ㼯

		//������֪3����Բ�ķ���,����2���д����󽻷�
		//�������ݼ����һ��Բ������
		//���һ��ֱ�߷���Top-Left
		if (js == 3)//��֪��3������Ҫ��õ㣬ȥ��Բ
		{
			float k1 = -1.0 / ((p1.y - p3.y) / ((p1.x - p3.x)*1.0));
			Point M1((p1.x + p3.x) / 2.0, (p1.y + p3.y) / 2.0);
			//��ڶ���ֱ�߷���  left-Bottom
			float k2 = -1.0 / ((p2.y - p3.y) / ((p2.x - p3.x)*1.0));
			Point M2((p3.x + p2.x) / 2.0, (p3.y + p2.y) / 2.0);

			m_center.x = ((k1*M1.x - k2*M2.x) - (M1.y - M2.y)) / ((k1 - k2)*1.0);
			m_center.y = k1*(m_center.x - M1.x) + M1.y;
			LR = sqrt((p1.x - m_center.x)*(p1.x - m_center.x) + (p1.y - m_center.y)*(p1.y - m_center.y));
			RR = sqrt((p2.x - m_center.x)*(p2.x - m_center.x) + (p2.y - m_center.y)*(p2.y - m_center.y));
			TR = sqrt((p3.x - m_center.x)*(p3.x - m_center.x) + (p3.y - m_center.y)*(p3.y - m_center.y));
			m_radius = (int)(LR + RR + TR) / 3.0;
		}
		if (js == 4 || js == 1)
		{
			//ȷ����ѵ�3����ѡ�㣬�������Բ
			float lenLR = 0, lenBT = 0;
			lenLR = sqrtf((Left.x - Right.x)*(Left.x - Right.x) + (Left.y - Right.y)*(Left.y - Right.y));
			lenBT = sqrtf((Top.x - Bottom.x)*(Top.x - Bottom.x) + (Top.y - Bottom.y)*(Top.y - Bottom.y));
			Point CTop, CLeft, CRight;
			if (lenLR >= lenBT)
			{
				CLeft = Left;
				CRight = Right;
			}
			else
			{
				CLeft = Top;
				CRight = Bottom;
			}

			//����
			Point intsect;
			intsect.x = Top.x;
			intsect.y = Left.y;
			if (lenLR >= lenBT)
			{
				float len1 = 0, len2 = 0;
				len1 = sqrtf((Top.x - intsect.x)*(Top.x - intsect.x) + (Top.y - intsect.y)* (Top.y - intsect.y));
				len2 = sqrtf((Bottom.x - intsect.x)*(Bottom.x - intsect.x) + (Bottom.y - intsect.y)* (Bottom.y - intsect.y));
				if (len1 >= len2)
					CTop = Top;
				else
					CTop = Bottom;
			}
			else
			{
				float len1 = 0, len2 = 0;
				len1 = sqrtf((Left.x - intsect.x)*(Left.x - intsect.x) + (Left.y - intsect.y)* (Left.y - intsect.y));
				len2 = sqrtf((Right.x - intsect.x)*(Right.x - intsect.x) + (Right.y - intsect.y)* (Right.y - intsect.y));
				if (len1 >= len2)
					CTop = Left;
				else
					CTop = Right;
			}
			Calc3yuan2ciFCforCircle(CTop, CLeft, CRight, m_radius, m_center);
		}
		if (js == 2)
		{
			///////////////////////////////////////////8.9////////////////////////////////////////////////////
			TR = sqrt((sp.at(0).x - m_center.x)*(sp.at(0).x - m_center.x) + (sp.at(0).y - m_center.y)*(sp.at(0).y - m_center.y));
			BR = sqrt((sp.at(1).x - m_center.x)*(sp.at(1).x - m_center.x) + (sp.at(1).y - m_center.y)*(sp.at(1).y - m_center.y));
			//���Ͻ�
			maxGrad = 0;
			Point RU;
			last = 0;
			int cou = 80;
			//Rightup
			float a = sin(PII / 4.0);
			for (int y = (int)(m_center.y - (Radious - 80)* sin(a)); y > (int)(m_center.y - (Radious + 80)* sin(a)); y--)
			{
				if (y < 0)
					break;
				if (y == (int)(m_center.y - (Radious - 80)* sin(a)))
					last = YSGrayImg.at<uchar>(y, (int)(m_center.x + (Radious - 80)* cos(a)));
				else
				{
					/*int x = m_center.x + (m_radius - cou)* cos(a);*/
					int f = YSGrayImg.at<uchar>(y, m_center.x + (Radious - cou)* cos(a));
					if (maxGrad >(f - last) && f < first)
					{
						maxGrad = f - last;
						RU = cv::Point(m_center.x + (Radious - cou)* cos(a), y);
					}
					last = f;
				}
				cou--;
			}
			//���Ͻ�
			maxGrad = 0;
			Point LU;
			last = 0;
			cou = 80;
			//Rightup
			for (int y = (int)(m_center.y - (Radious - 80)* sin(a)); y > (int)(m_center.y - (Radious + 80)* sin(a)); y--)
			{
				if (y < 0)
					break;
				if (y == (int)(m_center.y - (Radious - 80)* sin(a)))
					last = YSGrayImg.at<uchar>(y, (int)(m_center.x + (Radious - 80)* cos(a)));
				else
				{
					/*int x = m_center.x + (m_radius - cou)* cos(a);*/
					int f = YSGrayImg.at<uchar>(y, m_center.x - (Radious - cou)* cos(a));
					if (maxGrad >(f - last) && f < first)
					{
						maxGrad = f - last;
						LU = cv::Point(m_center.x - (Radious - cou)* cos(a), y);
					}
					last = f;
				}
				cou--;
			}
			//���½�
			maxGrad = 0;
			Point LB;
			last = 0;
			cou = 80;
			//LeftBottom
			for (int y = (int)(m_center.y + (Radious - 80)* sin(a)); y < (int)(m_center.y + (Radious + 80)* sin(a)); y++)
			{
				if (y > YSGrayImg.size().height)
					break;
				if (y == (int)(m_center.y + (Radious - 80)* sin(a)))
					last = YSGrayImg.at<uchar>(y, (int)(m_center.x - (Radious + 80)* cos(a)));
				else
				{
					//int x = m_center.x - (m_radius - cou)* cos(a);
					int f = YSGrayImg.at<uchar>(y, m_center.x - (Radious - cou)* cos(a));
					if (maxGrad > (f - last) && f < first)
					{
						maxGrad = f - last;
						LB = cv::Point(m_center.x - (Radious - cou)* cos(a), y);
					}
					last = f;
				}
				cou--;
			}
			//���½�
			maxGrad = 0;
			Point RB;
			last = 0;
			cou = 80;
			//LeftBottom
			for (int y = (int)(m_center.y + (Radious - 80)* sin(a)); y < (int)(m_center.y + (Radious + 80)* sin(a)); y++)
			{
				if (y > YSGrayImg.size().height)
					break;
				if (y == (int)(m_center.y + (Radious - 80)* sin(a)))
					last = YSGrayImg.at<uchar>(y, (int)(m_center.x - (Radious + 80)* cos(a)));
				else
				{
					//int x = m_center.x - (m_radius - cou)* cos(a);
					int f = YSGrayImg.at<uchar>(y, m_center.x + (Radious - cou)* cos(a));
					if (maxGrad > (f - last) && f < first)
					{
						maxGrad = f - last;
						RB = cv::Point(m_center.x + (Radious - cou)* cos(a), y);
					}
					last = f;
				}
				cou--;
			}
			int RUR = 0, LBR = 0, LUR = 0, RBR = 0;
			RUR = sqrt((RU.x - m_center.x)*(RU.x - m_center.x) + (RU.y - m_center.y)*(RU.y - m_center.y));
			LBR = sqrt((LB.x - m_center.x)*(LB.x - m_center.x) + (LB.y - m_center.y)*(LB.y - m_center.y));
			LUR = sqrt((LU.x - m_center.x)*(LU.x - m_center.x) + (LU.y - m_center.y)*(LU.y - m_center.y));
			RBR = sqrt((RB.x - m_center.x)*(RB.x - m_center.x) + (RB.y - m_center.y)*(RB.y - m_center.y));
			int k = 0;
			if (abs(RUR - Radious) < abs(LBR - Radious))
			{
				p1 = sp.at(0);
				p2 = sp.at(1);
				p3 = RU;
				k = RUR;
			}
			else
			{
				p1 = sp.at(0);
				p2 = sp.at(1);
				p3 = LB;
				k = LBR;
			}
			if (abs(LUR - Radious) < abs(k - Radious))
			{
				p3 = LU;
				k = LUR;
			}
			if (abs(RBR - Radious) < abs(k - Radious))
			{
				p3 = RB;
				//k = LUR;
			}
			//////////////////////////////////////////////////////////////////////////////////////////////////////
			float k1 = -1.0 / ((p1.y - p3.y) / ((p1.x - p3.x)*1.0));
			Point M1((p1.x + p3.x) / 2.0, (p1.y + p3.y) / 2.0);
			//��ڶ���ֱ�߷���  left-Bottom
			float k2 = -1.0 / ((p2.y - p3.y) / ((p2.x - p3.x)*1.0));
			Point M2((p3.x + p2.x) / 2.0, (p3.y + p2.y) / 2.0);

			m_center.x = ((k1*M1.x - k2*M2.x) - (M1.y - M2.y)) / ((k1 - k2)*1.0);
			m_center.y = k1*(m_center.x - M1.x) + M1.y;
			LR = sqrt((p1.x - m_center.x)*(p1.x - m_center.x) + (p1.y - m_center.y)*(p1.y - m_center.y));
			RR = sqrt((p2.x - m_center.x)*(p2.x - m_center.x) + (p2.y - m_center.y)*(p2.y - m_center.y));
			TR = sqrt((p3.x - m_center.x)*(p3.x - m_center.x) + (p3.y - m_center.y)*(p3.y - m_center.y));
			m_radius = (int)(LR + RR + TR) / 3.0;
		}
		if (js == 0)
		{
			//����ɨ�跨�����ϡ���3�������ҳ��Ӱ������ݶȱ仯������ΪԲ����3���㣬�������ĺͰ뾶
			int CmaxGrad = 0;
			Point CLeft;
			int Clast = 0;

			//Left
			for (int x = 50; x <m_w / 3.0; x++)
			{
				if (x == 50)
				{
					Clast = YSGrayImg.at<uchar>(m_h / 2.0, x);
				}
				else
				{
					int f = YSGrayImg.at<uchar>(m_h / 2.0, x);
					if (CmaxGrad <(f - Clast))
					{
						CmaxGrad = f - Clast;
						CLeft = cv::Point(x, m_h / 2.0);
						if (CmaxGrad>5)
							break;
					}
					Clast = f;
				}
			}
			CmaxGrad = 0;
			Point CRight;
			Clast = 0;

			//Right
			for (int x = m_w - 50; x >m_w*2.0 / 3.0; x--)
			{
				if (x == m_w - 50)
				{
					Clast = YSGrayImg.at<uchar>(m_h / 2.0, x);
				}
				else
				{
					int f = YSGrayImg.at<uchar>(m_h / 2.0, x);
					if (CmaxGrad <(f - Clast))
					{
						CmaxGrad = f - Clast;
						CRight = cv::Point(x, m_h / 2.0);
						if (CmaxGrad>5)
							break;
					}
					Clast = f;
				}
			}
			CmaxGrad = 0;
			Point CTop;
			Clast = 0;

			//Top
			for (int y = 50; y <m_h / 3.0; y++)
			{
				if (y == 50)
				{
					Clast = YSGrayImg.at<uchar>(y, m_w / 2.0);
				}
				else
				{
					int f = YSGrayImg.at<uchar>(y, m_w / 2.0);
					if (CmaxGrad <(f - Clast))
					{
						CmaxGrad = f - Clast;
						CTop = cv::Point(m_w / 2.0, y);
						if (CmaxGrad>5)
							break;
					}
					Clast = f;
				}
			}
			CalcZCXforCircle(CTop, CLeft, CRight, m_radius, m_center);
		}
	}
	g_center = m_center;
	circle(YSGrayImg, m_center, m_radius, cv::Scalar(255, 255, 255));
	namedWindow("Բ�����¶�λ",0);
	imshow("Բ�����¶�λ",YSGrayImg);
	return true;
}
int DDFCL::IteratSegThreshold(Mat& img, int nMaxIter, int& iDiffRec){ //ʹ�õ�����ȥ������ֵ
	//ͼ����Ϣ  
	int height = img.rows;
	int width = img.cols;

	//unsigned char *np; // ͼ��ָ��

	iDiffRec = 0;
	int F[256] = { 0 }; //ֱ��ͼ����  
	int iTotalGray = 0;//�Ҷ�ֵ��  
	int iTotalPixel = 0;//��������  
	unsigned char bt;//ĳ�������ֵ  

	uchar iThrehold, iNewThrehold;//��ֵ���·�ֵ  
	uchar iMaxGrayValue = 0, iMinGrayValue = 255;//ԭͼ���е����Ҷ�ֵ����С�Ҷ�ֵ  
	uchar iMeanGrayValue1, iMeanGrayValue2;

	//��ȡ(i,j)��ֵ������ֱ��ͼ����F  
	for (int i = 0; i<height; i++)
	{
		for (int j = 0; j<width; j++)
		{
			bt = img.at<uchar>(i, j);
			if (bt<iMinGrayValue)
				iMinGrayValue = bt;
			if (bt>iMaxGrayValue)
				iMaxGrayValue = bt;
			F[bt]++;
		}
	}

	iThrehold = 0;
	iNewThrehold = (iMinGrayValue + iMaxGrayValue) / 2;//��ʼ��ֵ  
	iDiffRec = iMaxGrayValue - iMinGrayValue;

	for (int a = 0; (abs(iThrehold - iNewThrehold)>0.5) && a<nMaxIter; a++)//������ֹ����  
	{
		iThrehold = iNewThrehold;
		//С�ڵ�ǰ��ֵ���ֵ�ƽ���Ҷ�ֵ  
		for (int i = iMinGrayValue; i<iThrehold; i++)
		{
			iTotalGray += F[i] * i;//F[]�洢ͼ����Ϣ  
			iTotalPixel += F[i];
		}

		iMeanGrayValue1 = (uchar)(iTotalGray / iTotalPixel);
		//���ڵ�ǰ��ֵ���ֵ�ƽ���Ҷ�ֵ  
		iTotalPixel = 0;
		iTotalGray = 0;
		for (int j = iThrehold + 1; j<iMaxGrayValue; j++)
		{
			iTotalGray += F[j] * j;//F[]�洢ͼ����Ϣ  
			iTotalPixel += F[j];
		}
		iMeanGrayValue2 = (uchar)(iTotalGray / iTotalPixel);

		iNewThrehold = (iMeanGrayValue2 + iMeanGrayValue1) / 2; //�·�ֵ  
		iDiffRec = abs(iMeanGrayValue2 - iMeanGrayValue1);
	}
	return iThrehold;
}
/*����1������Բ��3Ԫ2�η������������ķ��������Բ�ĺͰ뾶
CTop\CLeft\ CRight��Բ����3����*/
int DDFCL::Calc3yuan2ciFCforCircle(Point& CTop, Point& CLeft, Point& CRight, int &radius, Point& center)
{
	float a = 0, b = 0, xx = 0, yy = 0, r1 = 0, r2 = 0, r3 = 0;
	a = (CTop.x*CTop.x - CLeft.x*CLeft.x + CTop.y*CTop.y - CLeft.y*CLeft.y)*(CRight.x - CLeft.x) - (CRight.x*CRight.x - CLeft.x*CLeft.x + CRight.y*CRight.y - CLeft.y*CLeft.y)*(CTop.x - CLeft.x);
	b = 2 * ((CTop.y - CLeft.y)*(CRight.x - CLeft.x) - (CRight.y - CLeft.y)*(CTop.x - CLeft.x));
	yy = a / b;
	xx = ((CRight.y - CLeft.y)*(CTop.y*CTop.y - CLeft.y*CLeft.y + CTop.x*CTop.x - CLeft.x*CLeft.x) + (CTop.y - CLeft.y)*(CLeft.y*CLeft.y - CRight.y*CRight.y + CLeft.x*CLeft.x - CRight.x*CRight.x)) / (2 * (CTop.x - CLeft.x)*(CRight.y - CLeft.y) - 2 * (CRight.x - CLeft.x)*(CTop.y - CLeft.y));
	radius = sqrt((xx - CLeft.x)*(xx - CLeft.x) + (yy - CLeft.y)*(yy - CLeft.y));
	center.x = xx;
	center.y = yy;
	return 1;
}
/*����2������Բ��3����2���д����󽻵�ķ��������Բ�ĺͰ뾶
CTop\CLeft\ CRight��Բ����3����*/
int DDFCL::CalcZCXforCircle(Point& CTop, Point& CLeft, Point& CRight, int &radius, Point& center)
{
	//���һ���д���ֱ�߷���  left-Top
	float k1 = -1.0 / ((CLeft.y - CTop.y) / ((CLeft.x - CTop.x)*1.0));
	Point M1((CLeft.x + CTop.x) / 2.0, (CLeft.y + CTop.y) / 2.0);
	//��ڶ����д���ֱ�߷���  left-Bottom
	float k2 = -1.0 / ((CTop.y - CRight.y) / ((CTop.x - CRight.x)*1.0));
	Point M2((CRight.x + CTop.x) / 2.0, (CRight.y + CTop.y) / 2.0);

	center.x = ((k1*M1.x - k2*M2.x) - (M1.y - M2.y)) / ((k1 - k2)*1.0);
	center.y = k1*(center.x - M1.x) + M1.y;
	int CLR = sqrt(float((CLeft.x - center.x)*(CLeft.x - center.x) + (CLeft.y - center.y)*(CLeft.y - center.y)));
	int CRR = sqrt(float((CTop.x - center.x)*(CTop.x - center.x) + (CTop.y - center.y)*(CTop.y - center.y)));
	int CTR = sqrt(float((CRight.x - center.x)*(CRight.x - center.x) + (CRight.y - center.y)*(CRight.y - center.y)));
	radius = (int)(CLR + CRR + CTR) / 3.0;
	return 1;
}
//�ж�Դͼ���ǲ���8��б��ͼ��
bool DDFCL::GetIs8DegreeFace(void)
{
	Mat YSGrayImg(srcImg1.size().height, srcImg1.size().width, CV_8U, Scalar(0));         //�Ҷ�ͼ��
	cv::cvtColor(srcImg1, YSGrayImg, CV_BGR2GRAY);          //�Ҷ�ͼ��  CV_BGR2GRAY
	//YSGrayImg = m_SrcImg.clone();        //�Ҷ�ͼ��
	//����ƽ����������ֱ�����8��б�棬150
	int m_avg = 0;
	int m_avgYS = 0;
	int count = 0;
	//ȥ����������
	for (int y = 0; y < YSGrayImg.size().height; y++)
		for (int x = 0; x < YSGrayImg.size().width; x++)
		{
		if (sqrt((x - m_center.x)*(x - m_center.x) + (y - m_center.y)*(y - m_center.y))>m_radius)   //-14
		{
			YSGrayImg.at<uchar>(y, x) = 255;
		}
		else
		{
			m_avgYS += (int)YSGrayImg.at<uchar>(y, x);
			count++;
		}
		}
	//namedWindow("ȥ�����������ͼ��", 0);
	//imshow("ȥ�����������ͼ��", YSGrayImg); 
	//waitKey(0);
	//m_avg /= count;
	m_avgYS /= count; //���ƽ������
	//����Canny��Ե���õ�ֱ�����8��б�����ֲ��õ�8��б���ֱ��
	int ys = 0;
	//if (m_avgYS > 40 && m_avgYS <= 80)
	//	ys = m_avgYS - 1000 /*+25*/;
	//if (m_avgYS > 80 && m_avgYS<140)
	//	ys = m_avgYS - 120 /*+25    70*/;
	//if (m_avgYS >= 140 && m_avgYS<200)
	//	ys = m_avgYS - 130 /*+25    70*/;
	//if (m_avgYS >= 200 && m_avgYS<250)
	//	ys = m_avgYS - 140 /*+25    70*/;

	if (m_avgYS >= 100 && m_avgYS <= 110)
	{
		ys = m_avgYS - 25;
	}
	if (m_avgYS>110 && m_avgYS <= 130)
	{
		ys = m_avgYS - 41;
	}
	if (m_avgYS>130 && m_avgYS <= 150)
	{
		ys = m_avgYS - 35;
	}
	if (m_avgYS>150 && m_avgYS <= 170)
	{
		ys = m_avgYS - 55;
	}
	if (m_avgYS>170 && m_avgYS <= 180)
	{
		ys = m_avgYS - 65;
	}
	Mat MatImg8u(YSGrayImg.size().height, YSGrayImg.size().width, CV_8U, Scalar(0));
	cv::Canny(YSGrayImg, MatImg8u, 50, ys, 3);
	LineFinder finder;
	//����Huogh�任�Ĳ���  
	finder.setLineLengthAndGap(50, 10);
	finder.setMinVote(80);   //50  80   70
	//���ֱ�߲�������  
	li = finder.findLines(MatImg8u);
	m_flag = -1;
	a = 0, b = 0, c = 0;
	Is8DegreeFace = false;          //�������ֵ�ǰ����ֱ���滹��8��б��
	if (li.size() <= 0)
	{
		Is8DegreeFace = false;//��ƽ��ͼ��
		/* ::MessageBox(NULL, L"δ����ϳ�ֱ��!", L"Operation tips", MB_OK);
		return 0;*/
	}
	else
	{
		//������ֱ��  

		int lenmax = 50;
		std::vector<float>m_a;
		std::vector<float>m_b;
		std::vector<float>m_c;
		count = 0;

		std::vector<Vec4i>::const_iterator it2 = li.begin();
		while (it2 != li.end())
		{
			float len = 0;
			m_a.push_back((*it2)[3] - (*it2)[1]);
			m_b.push_back((*it2)[0] - (*it2)[2]);
			m_c.push_back(((*it2)[2])*((*it2)[1]) - ((*it2)[0])*((*it2)[3]));
			/*Point pt1, pt2;
			pt1 = Point(li.at(count)[0], li.at(count)[1]);
			pt2 = Point(li.at(count)[2], li.at(count)[3]);
			line(m_SrcImg, pt1, pt2, Scalar(0, 0, 255), 1);*/
			int Dis2P = sqrt((li.at(count)[0] - li.at(count)[2])*(li.at(count)[0] - li.at(count)[2]) + (li.at(count)[1] - li.at(count)[3])*(li.at(count)[1] - li.at(count)[3]));
			//Բ�ĵ�ֱ�ߵľ���
			len = abs(m_a[count] * m_center.x + m_b[count] * m_center.y + m_c[count]) / sqrtf(m_a[count] * m_a[count] + m_b[count] * m_b[count]);
			if (Dis2P > lenmax&&len>180 && len < 290)
			{
				lenmax = Dis2P;
				m_flag = count;

			}

			++it2;
			count++;
		}
		if (m_flag > -1)
		{
			a = m_a.at(m_flag);  //���abc
			b = m_b.at(m_flag);//���abc
			c = m_c.at(m_flag);
			Point pt1, pt2;//���abc
			pt1 = Point(li.at(m_flag)[0], li.at(m_flag)[1]);
			pt2 = Point(li.at(m_flag)[2], li.at(m_flag)[3]);
			line(srcImg1, pt1, pt2, Scalar(0, 0, 255), 1); //����ֱ��
			Is8DegreeFace = true;
		}
		else
			Is8DegreeFace = false;
	}

	/*namedWindow("����ֱ��", 0);
	imshow("����ֱ��", srcImg1);*/
	return Is8DegreeFace;
}
//�ָ��ͼ��
int DDFCL::DefectDetectJXH(int num, Mat &ShowImg){
	Mat YSGrayImg(srcImg2.size().height, srcImg2.size().width, CV_8U, Scalar(0));         //�Ҷ�ͼ��
	//cv::cvtColor(m_SrcImg, YSGrayImg, CV_BGR2GRAY);          //�Ҷ�ͼ��    ///////////////////////////12.28//////////////////////////
	/*namedWindow("����ԭʼͼ",0);
	imshow("����ԭʼͼ",srcImg2);*/
	YSGrayImg = srcImg2.clone();
	circle(srcImg2,m_center,380,Scalar(255));
	Mat CUTImg(2 * m_radius + 50, 2 * m_radius + 50,CV_8UC1, Scalar(0));     //���ü��ûҶ�ͼ��Ĵ�С
	int signal = 0;
	float NewC = 0.0;
	//Is8DegreeFace = false;
	if (Is8DegreeFace == true)  //�����Լ��ſ��ģ�
	{
		//cout << "******" << Is8DegreeFace << endl;
		//���¼���cֵ
		NewC = (li.at(m_flag)[2] - (g_center.x - m_radius - 25))*(li.at(m_flag)[1] - (g_center.y - m_radius - 25)) - (li.at(m_flag)[0] - (g_center.x - m_radius - 25))*(li.at(m_flag)[3] - (g_center.y - m_radius - 25));
		signal = a*m_center.x + b*m_center.y + c;
		if (signal > 0)
			signal = 1;
		else
			signal = -1;
	}//�Լ��ſ���
	//ȥ��������ͼ��
	int j = 0;
	for (int y = m_center.y - m_radius - 25; y < m_center.y + m_radius + 25; y++)
	{
		int i = 0;
		for (int x = m_center.x - m_radius - 25; x < m_center.x + m_radius + 25; x++)
		{
			if (sqrt((x - m_center.x)*(x - m_center.x) + (y - m_center.y)*(y - m_center.y)) > m_radius)   //-14
			{
				CUTImg.at<uchar>(j, i)= 0;
				//srcImg2.at<uchar>(y, x) = 0;
			}
			else{
				////CUTImg.at<uchar>(j, i) = (int)YSGrayImg.at<uchar>(y, x);
				//CUTImg.at<Vec3b>(j, i)[0] = 0;
				//CUTImg.at<Vec3b>(j, i)[0] = 0;
				//CUTImg.at<Vec3b>(j, i)[0] = 255;
				//�����һ������
				if (Is8DegreeFace == true)//�����8��б��
				{
					//cout << "sssss";
					//��̨�ײ���ͼ����Ϊ����ɫ����д�뱣��̨��ͼ����
					if ((a*x + b*y + c)*signal>0)//abc�Ƕ���  /*����ͼ��*/
					{
						CUTImg.at<uchar>(j, i) = (int)YSGrayImg.at<uchar>(y, x);
						//srcImg2.at<uchar>(y, x) = 255;

					}
					else
					{
						CUTImg.at<uchar>(j, i) = 0;
						//srcImg2.at<uchar>(y, x) = 0;
					}
				}
				else{
					CUTImg.at<uchar>(j, i) = (int)YSGrayImg.at<uchar>(y, x);//����8��б�棬�Ͳ�����ɾ��
				}
			}
			i++;
		}
		j++;
	}
	/*namedWindow("�ָ����ͼ��", 0);
    imshow("�ָ����ͼ��", CUTImg);*/
	////7��6��
	
	//namedWindow("xx",0);
	//imshow("xx",CUTImg);
	ddjc1=ImageFastFilter(CUTImg,2);
	Mat CUTImg2;
	cvtColor(CUTImg,CUTImg2,CV_GRAY2BGR);
	namedWindow("�����˲�ͼ��",0);
	imshow("�����˲�ͼ��",ddjc1);
	/*dilate(ddjc1, ddjc1, Mat(2, 2, CV_8U), Point(-1, -1), 2);
	erode(ddjc1, ddjc1, Mat(2, 2, CV_8U), Point(-1, -1), 2);*/
	//namedWindow("������ֳ���ͼ��", 0);
	//imshow("������ֳ���ͼ��",ddjc1);
	//imshow("����",srcImg2);
	ddjc2 = reverseImg(ddjc1);
	namedWindow("����ͼ��",0);
	imshow("����ͼ��",ddjc2);
	int x = (2 * m_radius + 50) / 2;
	int y = (2 * m_radius + 50) / 2;
	for (int i = 0; i < ddjc2.rows; i++){
		for (int j = 0; j < ddjc2.cols; j++){
			if (sqrt((x - i)*(x - i) + (y - j)*(y - j))>m_radius -20){
				ddjc2.at<uchar>(j, i) = 0;
			}
		}
	}
	
	//2017��7��6��
	/*erode(ddjc2, ddjc2, Mat(2, 2, CV_8U), Point(-1, -1), 2);
	dilate(ddjc2, ddjc2, Mat(2, 2, CV_8U), Point(-1, -1), 2);

	namedWindow("����2", 0);
	imshow("����2", ddjc2);*/
	Mat ddjc3=Mat::zeros(ddjc2.size(),ddjc2.type());
	vector<vector<Point>>  region = seedFilling(ddjc2, ddjc3, 100);
	//�ҵ�ÿһ�������������
	namedWindow("Ч��ͼ",0);
	imshow("Ч��ͼ",ddjc3);
	cout << region.size() << endl;
	for (int i = 0; i < region.size(); i++){
		vector<Point>pp;
		for (int j = 0; j < region[i].size(); j++){
			Point p;
			p.y = region[i][j].x;
			p.x = region[i][j].y;
			pp.push_back(p);
		}
		RotatedRect rect = minAreaRect(pp);
		Point2f pt[4];
		rect.points(pt);
		for (int i = 0; i < 4; i++)
		{
			line(ddjc3, pt[i], pt[(i + 1) % 4], Scalar(255));
			line(CUTImg2, pt[i], pt[(i + 1) % 4], Scalar(255,255,0));
		}
		cout << i + 1 <<"  ";
		cout << rect.size <<"  ";
		cout << rect.size.width <<"  ";
		cout << rect.size.height<<endl;
	}
	namedWindow("Ч��ͼ22", 0);
	imshow("Ч��ͼ22", ddjc3);
	imshow("����ͼ",CUTImg2);
	return 1;
}
Mat DDFCL::ImageFastFilter(Mat pImg, int t){
	Mat pDstImg = Mat::zeros(pImg.size(), CV_8UC1);
	Mat medianImagex = Mat::zeros(pImg.size(), CV_8UC1);
	Mat medianImagey = Mat::zeros(pImg.size(), CV_8UC1);

	int height = pImg.rows, width = pImg.cols;
	//float k[5] = { 1.0, 0.0, -2.0,0.0, 1.0 };
	//float k[7] = { 1.0,0.0, 0.0, -2.0, 0.0, 0.0, 1.0 };
	//float k[9] = { 1.0,0.0, 0.0,0.0, -2.0, 0.0,0.0, 0.0, 1.0 };
	//float k[11] = { 1.0, 0.0, 0.0, 0.0, 0.0, -2.0, 0.0, 0.0, 0.0, 0.0, 1.0 };
	//float k[19] = { 1.0,0.0,0.0,0.0,0.0, 0.0,0.0, 0.0,0.0, -2.0,0.0, 0.0,0.0,0.0,0.0, 0.0,0.0, 0.0, 1.0 };
	//�����ϵ��
	float k[33] = { 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0 };
	Mat kx = Mat(1, 33, CV_32FC1, k);   //ˮƽ�˲��ˡ�kxΪˮƽ�˲���һ��33��
	Mat ky = Mat(33, 1, CV_32FC1, k);   //��ֱ�˲��ˡ�kyΪ��ֱ�˲���33��1�С�
	//float k[9] = { 2.0, 1.0, 0.0, 0.0, -6.0, 0.0, 0.0,1.0, 2.0 };
	// kx = Mat(1, 9, CV_32FC1, k);   //ˮƽ�˲���
	//Mat ky = Mat(9, 1, CV_32FC1, k);   //��ֱ�˲���
	filter2D(pImg, medianImagex, pImg.depth(), kx, cvPoint(-1, -1));   //ˮƽ�˲�ͼ��
	filter2D(pImg, medianImagey, pImg.depth(), ky, cvPoint(-1, -1));   //��ֱ�˲�ͼ��
	int M = 4, T = 22;//M��t����ʲô��˼
	T = t;
	int p, px, py;
	for (int i = 0; i < height; i++)//����ͼ��
	{
		for (int j = 0; j < width; j++)
		{
			//******************�ĸ���������******************//
			if ((i >= 0) && (i < M) && (j >= 0) && (j < M))   //���Ͻ�����
			{
				px = medianImagex.at<uchar>(i, j);
				py = medianImagey.at<uchar>(i, j);
				p = px< py ? px : py;//ȡС�ĵ�ֵ
			}
			else if ((i >= 0) && (i < M) && (j >= width - M) && (j < width))   //���Ͻ�����
			{
				px = medianImagex.at<uchar>(i, j);
				py = medianImagey.at<uchar>(i, j);
				p = px< py ? px : py;
			}
			else if ((i >= height - M) && (i < height) && (j >= 0) && (j < M))   //���½�����
			{
				px = medianImagex.at<uchar>(i, j);
				py = medianImagey.at<uchar>(i, j);
				p = px < py ? px : py;
			}
			else if ((i >= height - M) && (i < height) && (j >= width - M) && (j < width))   //���½�����
			{
				px = medianImagex.at<uchar>(i, j);
				py = medianImagey.at<uchar>(i, j);
				p = px< py ? px : py;
			}
			//******************ˮƽ��Ե����******************//j >= width
			else if ((((i >= 0) && (i < M)) || ((i >= height - M) && (i < height))) && ((j >= width) && (j < width - M)))
				p = medianImagex.at<uchar>(i, j);
			//******************��ֱ��Ե����******************//
			else if (((i >= M) && (i < height - M)) && (((j >= 0) && (j < M)) || ((j >= width - M) && (j < width))))
				p = medianImagey.at<uchar>(i, j);
			else
			{
				px = medianImagex.at<uchar>(i, j);
				py = medianImagey.at<uchar>(i, j);
				p = (px + py) / 2;
			}
			p = p> T ? 255 : 0;//T��Ҫ�Ƚϵ�����
			pDstImg.at<uchar>(i, j) = p;
		}
	}
	//medianBlur(pDstImg, pDstImg,5);
	return pDstImg;
}
Mat DDFCL::reverseImg(Mat img){
	Mat img1=Mat(img.rows,img.cols,CV_8UC1,Scalar(255));
	int j = 0;
	for (int y = m_center.y - m_radius - 25; y < m_center.y + m_radius + 25; y++)
	{
		int signal = 0;
		float NewC = 0.0;
		//Is8DegreeFace = false;
		if (Is8DegreeFace == true)  //�����Լ��ſ��ģ�
		{
			//cout << "******" << Is8DegreeFace << endl;
			//���¼���cֵ
			NewC = (li.at(m_flag)[2] - (g_center.x - m_radius - 25))*(li.at(m_flag)[1] - (g_center.y - m_radius - 25)) - (li.at(m_flag)[0] - (g_center.x - m_radius - 25))*(li.at(m_flag)[3] - (g_center.y - m_radius - 25));
			signal = a*m_center.x + b*m_center.y + c;
			if (signal > 0)
				signal = 1;
			else
				signal = -1;
		}//�Լ��ſ���
		int i = 0;
		for (int x = m_center.x - m_radius - 25; x < m_center.x + m_radius + 25; x++)
		{
			if (sqrt((x - m_center.x)*(x - m_center.x) + (y - m_center.y)*(y - m_center.y)) > m_radius)   //-14
			{
				img1.at<uchar>(j, i) = 0;
				//srcImg2.at<uchar>(y, x) = 0;
			}
			else{
				////CUTImg.at<uchar>(j, i) = (int)YSGrayImg.at<uchar>(y, x);
				//CUTImg.at<Vec3b>(j, i)[0] = 0;
				//CUTImg.at<Vec3b>(j, i)[0] = 0;
				//CUTImg.at<Vec3b>(j, i)[0] = 255;
				//�����һ������
				if (Is8DegreeFace == true)//�����8��б��
				{
					//cout << "sssss";
					//��̨�ײ���ͼ����Ϊ����ɫ����д�뱣��̨��ͼ����
					if ((a*x + b*y + c)*signal>0)//abc�Ƕ���  /*����ͼ��*/
					{
						img1.at<uchar>(j, i) =255;
						//srcImg2.at<uchar>(y, x) = 255;

					}
					else
					{
						img1.at<uchar>(j, i) = 0;
						//srcImg2.at<uchar>(y, x) = 0;
					}
				}
				else{
					img1.at<uchar>(j, i) = 255;//����8��б�棬�Ͳ�����ɾ��
				}
			}
			i++;
		}
		j++;
	}
	Mat dst=img1-img;
	erode(dst, dst, Mat(2, 2, CV_8U), Point(-1, -1), 2);
	return dst;
};
vector<vector<Point> > DDFCL::seedFilling(Mat& srcImg, Mat& dstImg, const ushort& minArea) //��ͨ����
{
	vector<Point> m_Blob;   //һ����ļ���
	vector< vector<Point> >  m_Region;//�㼯������

	const ushort imgHeight = srcImg.rows, imgWidth = srcImg.cols;  //ͼ��ĸߺͿ�
	Mat srcCopy = srcImg.clone();   //ͼ��Ŀ�¡

	for (ushort r = 0; r < imgHeight; r++)   //����ͼ��
	{
		uchar* data = srcCopy.ptr<uchar>(r);   //ÿһ�е�ָ��
		for (ushort c = 0; c < imgWidth; c++)
		{
			if (data[c] != 0)   //��ǰ���ػҶ�ֵ��Ϊ0
			{
				stack< pair<ushort, ushort> >  neighborPixels;  //���ڵ����ء����ڵ�����Ҳ��һ�����ϡ�
				neighborPixels.push(pair<ushort, ushort>(r, c));  //����ǰ���ش�������֮��
				data[c] = 0;   //����ɨ�����ػҶ�ֵ��0

				while (!neighborPixels.empty())   //�����Ϊ��
				{
					pair<ushort, ushort> curPixel = neighborPixels.top();
					ushort curX = curPixel.first, curY = curPixel.second;  //ȡx�����y����
					neighborPixels.pop();   //pop the top pixel   ����ա�
					m_Blob.push_back(Point(curX, curY));//����������Ȥ������

					// push the 4-neighbors(foreground pixels)    
					if ((curY != imgWidth - 1) && (srcCopy.at<uchar>(curX, curY + 1) != 0))   //right pixel
					{
						neighborPixels.push(pair<ushort, ushort>(curX, curY + 1));
						srcCopy.at<uchar>(curX, curY + 1) = 0;//����Ժ��Ϊ0
					}
					if ((curX != 0) && (srcCopy.at<uchar>(curX - 1, curY) != 0))   //up pixel
					{
						neighborPixels.push(pair<ushort, ushort>(curX - 1, curY));
						srcCopy.at<uchar>(curX - 1, curY) = 0;
					}
					if ((curY != 0) && (srcCopy.at<uchar>(curX, curY - 1) != 0))   //left pixel
					{
						neighborPixels.push(pair<ushort, ushort>(curX, curY - 1));
						srcCopy.at<uchar>(curX, curY - 1) = 0;
					}
					if ((curX != imgHeight - 1) && (srcCopy.at<uchar>(curX + 1, curY) != 0))   //down pixel
					{
						neighborPixels.push(pair<ushort, ushort>(curX + 1, curY));
						srcCopy.at<uchar>(curX + 1, curY) = 0;
					}
				}
				if (m_Blob.size() >= minArea)   //�����>=minArea����ͨ������άvector����
				{
					m_Region.push_back(m_Blob);  //���ҵ�������������С���
					for (ushort k = 0; k < m_Blob.size(); k++)   //��srcͼ�����ݸ��Ƶ�dst
					{
						ushort h = m_Blob.at(k).x, w = m_Blob.at(k).y;
						dstImg.at<uchar>(h, w) = srcImg.at<uchar>(h, w);
					}
				}
				else
				{
					for (ushort k = 0; k < m_Blob.size(); k++)   //��srcͼ�����ݸ��Ƶ�dst
					{
						ushort h = m_Blob.at(k).x, w = m_Blob.at(k).y;
						dstImg.at<uchar>(h, w) = 0;
					}
				}
				m_Blob.clear();   //ÿѭ��һ�����һ�ε�ļ���
			}
		}
	}
	return m_Region;
}