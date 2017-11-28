/*
 * TwoLayerNNFaster.cpp
 *
 *  Created on: 2017年10月23日
 *      Author: yfji
 */

#include "TwoLayerNNFaster.h"

TwoLayerNNFaster::TwoLayerNNFaster(int mode_, int minibatchIfTrain) {
	srand(time(NULL));
	mode=mode_;
	if(mode==TRAIN or mode==FINETUNE)	minibatch=minibatchIfTrain;
	else if(mode==TEST)	minibatch=1;
	else{
		cerr<<"unrecognized mode"<<endl;
		exit(0);
	}
	sample_file_lst="./file_lst.txt";
	alloc();
	if(mode==TRAIN or mode==FINETUNE){
		reset();
		loadSamplePathAndLabels();
		shuffle();
	}
	curIndex=0;
	step=2;
	display=10;
	gamma=1.0;
	lr=0.1;
	reg=1e-4;
}

TwoLayerNNFaster::~TwoLayerNNFaster() {
	release();
}

void TwoLayerNNFaster::reset(double std){
	std::mt19937 gen(time(0));
	std::normal_distribution<double> n(0,1);
	int i;
	for(i=0;i<nLayers[1];++i)
		b1[i]=0.0;
	for(i=0;i<nLayers[2];++i)
		b2[i]=0.0;
	for(i=0;i<nLayers[0]*nLayers[1];++i){
		W1[i]=std*n(gen);
	}
	for(i=0;i<nLayers[1]*nLayers[2];++i){
		W2[i]=std*n(gen);
	}
}

void TwoLayerNNFaster::alloc(){
	cout<<"allocating memory..."<<endl;
	W1=new double[nLayers[0]*nLayers[1]];
	W2=new double[nLayers[1]*nLayers[2]];
	b1=new double[nLayers[1]];
	b2=new double[nLayers[2]];
	batchSamples=new double[minibatch*nLayers[0]];
	output=new double[minibatch*nLayers[2]];
	yLayer1=new double[minibatch*nLayers[1]];
	yLayer2=new double[minibatch*nLayers[2]];

	if(mode==TRAIN or mode==FINETUNE){
		gW1=new double[nLayers[0]*nLayers[1]];
		gW2=new double[nLayers[1]*nLayers[2]];
		gb1=new double[nLayers[1]];
		gb2=new double[nLayers[2]];
		dLayer1=new double[minibatch*nLayers[1]];
		dLayer2=new double[minibatch*nLayers[2]];
		batchLabels=new int[minibatch];
	}
}

void TwoLayerNNFaster::release(){
	delete W1;
	delete W2;
	delete batchSamples;
	delete output;
	delete yLayer1;
	delete yLayer2;
	delete b1;
	delete b2;
	if(mode==TRAIN or mode==FINETUNE){
		delete gW1;
		delete gW2;
		delete gb1;
		delete gb2;
		delete dLayer1;
		delete dLayer2;
		delete batchLabels;
	}
}

void TwoLayerNNFaster::loadSamplePathAndLabels(){
	ifstream in;
	in.open(sample_file_lst.c_str(), ios::in);
	string line;
	int cnt=0;
	cout<<"loading samples"<<endl;
	while(!in.eof()){
		std::getline(in, line);
		if(line.length()<5)	continue;
		int i=line.find_last_of('/')+1;
		int index=int(line[i])-48;
		if(line[i+1]!='_'){
			char num[2]={line[i],line[i+1]};
			index=atoi(num);
		}
		sampleFiles.push_back(line);
		labelMat.push_back(index);
		if(index==10)
			cout<<line<<": "<<index<<endl;
		++cnt;
	}
	numSamples=cnt;
	for(int i=0;i<numSamples;++i){
		ind.push_back(i);
	}
	cout<<numSamples<<" samples are loaded"<<endl;
}

void TwoLayerNNFaster::shuffle(){
	for(size_t i=0;i<ind.size();++i){
		int swap_ind=rand()%(ind.size());
		swap(ind[i],ind[swap_ind]);
	}
}

double TwoLayerNNFaster::forward(){
	double sum_loss=0.0;
	int offset=0;
	//yLayer1=batchSamples*W1: [N*H]=[N*D].[D*H]
	//alpha*A*B+beta*C-->C
	cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, minibatch, nLayers[1], nLayers[0], \
			1.0, batchSamples, nLayers[0], W1, nLayers[1], 0.0, yLayer1, nLayers[1]);
	//yLayer2
	for(int i=0;i<minibatch*nLayers[1];++i){
		if(yLayer1[i]<0.0)
			yLayer1[i]=0.0;
	}
	cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, minibatch, nLayers[2], nLayers[1],\
			1.0, yLayer1, nLayers[1], W2, nLayers[2], 0.0, yLayer2, nLayers[2]);

	offset=0;
	for(int i=0;i<minibatch;++i){
		double max_out=-1.0*1e3;
		double sum_out=0.0;
		for(int j=0;j<nLayers[2];++j){
			if(yLayer2[j+offset]>max_out)
				max_out=yLayer2[j+offset];
		}
		for(int j=0;j<nLayers[2];++j){
			output[j+offset]=std::exp(yLayer2[j+offset]-max_out);
			sum_out+=output[j+offset];
		}
		for(int j=0;j<nLayers[2];++j){
			output[j+offset]/=sum_out;
		}
		sum_loss += -1.0*std::log(output[offset+batchLabels[i]]);
		output[offset+batchLabels[i]]-=1.0;
		for(int j=0;j<nLayers[2];++j){
			dLayer2[j+offset]= output[j+offset]/minibatch;
			//cout<<dLayer2[j+offset]<<",";
		}
		//cout<<endl;
		offset+=nLayers[2];
	}
	//gW2=yLayer1T*dLayer2: [H*O]=[H*N].[N*O]
	cblas_dgemm(CblasRowMajor, CblasTrans, CblasNoTrans, nLayers[1], nLayers[2], minibatch, \
			1.0, yLayer1, nLayers[1], dLayer2, nLayers[2], 0.0, gW2, nLayers[2]);

	//dLayer1=dLayer2*W2T: [N*H]=[N*O].[O*H]
	cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasTrans, minibatch, nLayers[1], nLayers[2],\
			1.0, dLayer2, nLayers[2], W2, nLayers[2], 0.0, dLayer1, nLayers[1]);
	//gW1=batchSamplesT*dLayer1: [D*H]=[D*N].[N*H]
	cblas_dgemm(CblasRowMajor, CblasTrans, CblasNoTrans, nLayers[0], nLayers[1], minibatch, \
			1.0, batchSamples, nLayers[0], dLayer1, nLayers[1], 0.0, gW1, nLayers[1]);
	for(int j=0;j<nLayers[1];++j){
		gb1[j]=0.0;
		for(int i=0;i<minibatch;++i){
			gb1[j]+=dLayer1[j+i*nLayers[1]];
		}
	}
	for(int j=0;j<nLayers[2];++j){
		gb2[j]=0.0;
		for(int i=0;i<minibatch;++i)
			gb2[j]+=dLayer2[j+i*nLayers[2]];
	}
	sum_loss/=minibatch;
	return sum_loss;
}

void TwoLayerNNFaster::getFeature(Mat& image, int batchIndex){
	Mat digit;
	if(image.channels()==3)
		cv::cvtColor(image, digit, cv::COLOR_BGR2GRAY);
	else
		image.copyTo(digit);
	cv::threshold(digit, digit, 50,255, cv::THRESH_OTSU);
	resize(digit, digit, Size(resz_w, resz_h), cv::INTER_CUBIC);
	assert(digit.rows*digit.cols==nLayers[0]);
	cv::threshold(digit, digit, 50, 255, cv::THRESH_OTSU);
	uchar* data=digit.data;
	int val;
	int offset=batchIndex*nLayers[0];
	for(int i=0;i<digit.rows*digit.cols;++i){
		val=(int)data[i];
		batchSamples[offset+i]=val>50?0.0:1.0;
	}
}

void TwoLayerNNFaster::getFeatureX(Mat& image, int batchIndex){
	//Mat digit;image.copyTo(digit);
	Mat digit;
	if(image.channels()==3)
		cv::cvtColor(image, digit, cv::COLOR_BGR2GRAY);
	else
		image.copyTo(digit);
	cv::threshold(digit, digit, 50,255, cv::THRESH_OTSU);
	uchar* digitdata=digit.data;
	int h=digit.rows;
	int w=digit.cols;
	int max_cnt=-100;
	double bin_size_h=static_cast<double>(h)/(pool_h);
	double bin_size_w=static_cast<double>(w)/(pool_w);

	int offset=batchIndex*nLayers[0];
	for(int ph=0;ph<pool_h;++ph){
		int start_h=std::max<int>(0,round(ph*bin_size_h));
		int end_h=std::min<int>(h,round((ph+1)*bin_size_h));
		for(int pw=0;pw<pool_w;++pw){
			int start_w=std::max<int>(0,round(pw*bin_size_w));
			int end_w=std::min<int>(w,round((pw+1)*bin_size_w));
			int cnt=0;
			for(int y=start_h;y<end_h;++y){
				for(int x=start_w;x<end_w;++x){
					if(digitdata[y*w+x]==0)	++cnt;
				}
			}
			if(cnt>max_cnt)	max_cnt=cnt;
			batchSamples[offset+(ph*pool_w+pw)]=static_cast<double>(cnt);
		}
	}
}

void TwoLayerNNFaster::backward(){
	for(int i=0;i<nLayers[0]*nLayers[1];++i){
		W1[i] -= (lr*gW1[i]+reg*W1[i]);
	}
	for(int i=0;i<nLayers[1]*nLayers[2];++i){
		W2[i] -= (lr*gW2[i]+reg*W2[i]);
	}
	for(int i=0;i<nLayers[1];++i)
		b1[i] -= lr*gb1[i];
	for(int i=0;i<nLayers[2];++i)
		b2[i] -= lr*gb2[i];
}

void TwoLayerNNFaster::train(int iters){
	int iter=0;
	int step_duration=std::max(1,iters/step);
	cout<<"start train"<<endl;
	while(iter<iters){
		Rect rect;
		for(int k=curIndex;k<curIndex+minibatch;++k){
			Mat image=imread(sampleFiles[ind[k]]);
			if(labelMat[ind[k]]!=10)
				rect=findDigitBndBox(image);
			else
				rect=Rect(0,0,image.cols,image.rows);
			Mat digit=image(rect);
			getFeatureX(digit, k-curIndex);
			batchLabels[k-curIndex]=labelMat[ind[k]];
		}
		float loss=forward();
		backward();
		if(iter%display==0 or iter==iters-1)
			cout<<"iteration: "<<iter<<", loss: "<<loss<<endl;
		curIndex+=minibatch;
		if(curIndex>=numSamples){
			shuffle();
			curIndex=0;
		}
		++iter;
		if(iter%step_duration==0){
			lr*=gamma;
		}
	}
	cout<<endl<<"finished"<<endl;
}

void TwoLayerNNFaster::predict(Mat& image, int& pred, double& prob){
	//assert(mode==TEST);
	getFeatureX(image, 0);
	cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, minibatch, nLayers[1], nLayers[0], \
				1.0, batchSamples, nLayers[0], W1, nLayers[1], 0.0, yLayer1, nLayers[1]);
	//yLayer2
	for(int i=0;i<minibatch*nLayers[1];++i){
		if(yLayer1[i]<0.0)
			yLayer1[i]=0.0;
	}
	cblas_dgemm(CblasRowMajor, CblasNoTrans, CblasNoTrans, minibatch, nLayers[2], nLayers[1],\
			1.0, yLayer1, nLayers[1], W2, nLayers[2], 0.0, yLayer2, nLayers[2]);

	double max_out=-1.0*1e3;
	double sum_out=0.0;
	for(int j=0;j<nLayers[2];++j){
		if(yLayer2[j]>max_out){
			max_out=yLayer2[j];
			pred=j;
		}
	}
	for(int j=0;j<nLayers[2];++j){
		output[j]=std::exp(yLayer2[j]-max_out);
		sum_out+=output[j];
	}
	for(int j=0;j<nLayers[2];++j){
		output[j]/=sum_out;
	}
	prob=output[pred];
	//cout<<"prob: "<<output[pred]<<endl;
	//cout<<pred<<endl;
	if(output[pred]<0.8)
		pred=-1;
}

void TwoLayerNNFaster::saveParams(const char* filename){
	ofstream out;
	out.open(filename, ios::out);
	out<<"params"<<endl;
	for(int i=0;i<nLayers[0]*nLayers[1];++i){
		out<<W1[i];
		if((i+1)%nLayers[1]==0)	out<<endl;
		else 	out<<' ';
	}
	out<<"params"<<endl;
	for(int i=0;i<nLayers[1]*nLayers[2];++i){
		out<<W2[i];
		if((i+1)%nLayers[2]==0)	out<<endl;
		else	out<<' ';
	}
	out<<"bias"<<endl;
	for(int i=0;i<nLayers[1];++i){
		out<<b1[i];
		if(i<nLayers[1]-1)	out<<' ';
	}
	out<<endl;
	out<<"bias"<<endl;
	for(int i=0;i<nLayers[2];++i){
		out<<b2[i];
		if(i<nLayers[2]-1)	out<<' ';
	}
	out.close();
}

void TwoLayerNNFaster::loadParams(const char* filename){
	cout<<"loading params..."<<endl;
	ifstream in;
	in.open(filename, ios::in);
	if(!in){
		cerr<<"no parameter file found, reset randomly"<<endl;
		reset();
	}
	string line;
	int layerIndexParams=0;
	int layerIndexBias=0;
	int paramRow=0;
	while(!in.eof()){
		getline(in, line);
		if(line.length()<=1)	continue;
		if(line=="params"){
			++layerIndexParams;
			paramRow=0;
		}
		else if(line=="bias"){
			++layerIndexBias;
		}
		else if(layerIndexParams==1 && layerIndexBias==0){
			stringstream ss(line);
			double param;
			for(int i=0;i<nLayers[1];++i){
				ss>>param;
				W1[paramRow*nLayers[1]+i]=param;
			}
			++paramRow;
		}
		else if(layerIndexParams==2 && layerIndexBias==0){
			stringstream ss(line);
			double param;
			for(int i=0;i<nLayers[2];++i){
				ss>>param;
				W2[paramRow*nLayers[2]+i]=param;
			}
			++paramRow;
		}
		else if(layerIndexBias==1){
			stringstream ss(line);
			double param;
			for(int i=0;i<nLayers[1];++i){
				ss>>param;
				b1[i]=param;
			}
		}
		else if(layerIndexBias==2){
			stringstream ss(line);
			double param;
			for(int i=0;i<nLayers[2];++i){
				ss>>param;
				b2[i]=param;
			}
		}
	}
	in.close();
}

