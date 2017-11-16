/*
 * util.cpp
 *
 *      Author: JYF
 */

#include "util.h"

//used only during training
Rect findDigitBndBox(Mat& image){
	Mat gray;
	if(image.channels()==3){
		cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
	}
	else
		image.copyTo(gray);
	Mat biMat;

	cv::threshold(gray, biMat, 100, 255, cv::THRESH_OTSU);
	cv::bitwise_not(biMat, biMat);
	//cv::imshow("bi", biMat);
	IplImage ipl=biMat;
	CvMemStorage* pStorage=cvCreateMemStorage(0);
	CvSeq * pContour=NULL;
	cvFindContours(&ipl, pStorage, &pContour, sizeof(CvContour), CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

	double max_area=0;
	Rect bndbox;
	for(;pContour;pContour=pContour->h_next){
		for(CvSeq* inner=pContour;inner;inner=inner->v_next){
			CvRect bbox=cvBoundingRect(inner,0);
			double real_area=fabs(cvContourArea(inner));
			if(real_area>max_area){
				max_area=real_area;
				bndbox=Rect(bbox);
			}
		}
	}
	cvReleaseMemStorage(&pStorage);
	return bndbox;
}

int Compare (const int& a, const int& b)
{
  return a<b;
}

void nms(vector<Rect>& rects){
	vector<int> deleteIdx;
	vector<char> flags(rects.size());
	for(size_t i=0;i<flags.size();++i)
		flags[i]=0;
	int areai,areaj,nms_area,nms_rbx,nms_rby,nms_ltx,nms_lty, nms_w, nms_h;
	for(size_t i=0;i<rects.size();++i){
		Rect& reci=rects[i];
		areai=reci.width*reci.height;
		for(size_t j=i+1;j<rects.size();++j){
			//if(i==j)	continue;
			Rect& recj=rects[j];
			areaj=recj.width*recj.height;
			nms_rbx=min(recj.x+recj.width-1, reci.x+reci.width-1);
			nms_rby=min(recj.y+recj.height-1, reci.y+reci.height-1);
			nms_ltx=max(recj.x,reci.x);
			nms_lty=max(recj.y,reci.y);
			nms_w=max(nms_rbx-nms_ltx,0);
			nms_h=max(nms_rby-nms_lty,0);
			nms_area=nms_w*nms_h;
			float ratio2i=1.0*nms_area/areai;
			float ratio2j=1.0*nms_area/areaj;
			if(ratio2i>0.8 and not flags[i]){
				deleteIdx.push_back(i);
				flags[i]=1;
			}
			else if(ratio2j>0.8 and not flags[j]){
				deleteIdx.push_back(j);
				flags[j]=1;
			}
		}
	}
	if(deleteIdx.size()>0){
		auto beginner=rects.begin();
		/*
		cout<<rects.size()<<": ";
		for(size_t i=0;i<deleteIdx.size();++i)
			cout<<deleteIdx[i]<<",";
		cout<<endl;
		*/
		sort(deleteIdx.begin(), deleteIdx.end(), Compare);
		int k=0;
		for(size_t i=0;i<deleteIdx.size();++i){
			k=deleteIdx[i]-i;
			rects.erase(beginner+k);
		}
	}
}
