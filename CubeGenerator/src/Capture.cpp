/**
  @file Capture.cpp
  @brief Provides functionality to handle plugin input data.
  
  Declaration: Capture.h
  Provides methods to convert image data from/to stacked planes format (e.g. DevEnviro Image)
  
  @author Andre Halim
  @author Fraunhofer IOSB VID
  @date 2016, Apr.-Sept.
 */
//==================================================================================================

#include "Capture.h"

using namespace std;
using namespace cv;

//==================================================================================================

//____Instance__________________________________________________________________

Capture::Capture()
  : mHyp(0), mResolution(0, 0), mIsBGR(true), mIn() {}

Capture::Capture(const InputData &src, bool isBGR, bool preserve)
  : mIsBGR(isBGR)
{
  if (preserve){
    mIn.m = src.m.clone();
    mIn.angles[PAN] = src.angles[PAN];
    mIn.angles[TIL] = src.angles[TIL];
    mIn.angles[ROT] = src.angles[ROT];
    mIn.zoom = src.zoom;
  }else
    mIn = src;
  
  int w = src.m.cols;
  int h = src.m.rows;
  mResolution = Size(w, h);
  mHyp = (long)floor(sqrt((double)w*w+(double)h*h));
}


Capture::Capture(bool isBGR, float zoom) : mIsBGR(isBGR)
{
  mIn.zoom = zoom;
}

//____Output____________________________________________________________________

void Capture::writePlanar(unsigned char* pData) const
{//Accepts only uchar formatted memory. Correct pre-allocation expected
  int w = mIn.m.cols;
  int h = mIn.m.rows;
  int len = w*h;
  const unsigned char* pIn;
  
  if (mIn.m.channels()==3){
    unsigned char* pg = pData + len;
    unsigned char* pb = pData + 2 * len;
    MatConstIterator_<Vec3b> end=mIn.m.end<Vec3b>();
    MatConstIterator_<Vec3b> it;
    //normal cv interpretation
    if (this->mIsBGR){
      for (it = mIn.m.begin<Vec3b>(); it != end; it++){
        *pData++ = (*it)[0];  //b
        *pg++ = (*it)[1];     //g
        *pb++ = (*it)[2];     //r
      }
    //other interpretation
    }else{
      for (it = mIn.m.begin<Vec3b>(); it != end; it++){
        *pData++ = (*it)[2];
        *pg++ = (*it)[1];
        *pb++ = (*it)[0];
      }
    }
  //direct copy
  }else{
    int j;
    //is it like one line array without padding?
    if (mIn.m.isContinuous()){
      w *= h; h = 1;
    }
    
    for (int i = 0; i < h; i++){
      //ptr(row) ensures correct entry and row span
      pIn = mIn.m.ptr(i);
      for (j = 0; j < w; j++)
        *pData++ = saturate_cast<uchar>(*pIn++);
    }
  }
}


char Capture::channels() const
{
  return mIn.m.channels();
}


float Capture::pan() const
{
  return mIn.angles[PAN];
}


float Capture::tilt() const
{
  return mIn.angles[TIL];
}


float Capture::zoom() const
{
  return mIn.zoom;
}


const float& Capture::angle(Angles idx) const
{
  return mIn.angles[idx];
}


Capture Capture::clone() const
{
  //old: Capture left(this->mIn,this->mIsBGR,true);
  Capture lh;
  lh.mHyp = this->mHyp;
  lh.mResolution = this->mResolution;
  lh.mIsBGR = this->mIsBGR;
  
  lh.mIn.m = this->mIn.m.clone();
  lh.mIn.angles[PAN] = this->mIn.angles[PAN];
  lh.mIn.angles[TIL] = this->mIn.angles[TIL];
  lh.mIn.angles[ROT] = this->mIn.angles[ROT];
  lh.mIn.zoom = this->mIn.zoom;
  return lh;
}


const uchar* Capture::dataStart() const
{// [todo] is continous?
  return mIn.m.ptr();
}


//____Methods___________________________________________________________________

bool Capture::readPlanar(unsigned char* pData, int w, int h, int pl,bool cpy)
{/*
  mat constructor can take a datapointer and construct a image header only
  without the need of memcpy. But cv offers no const pointer constructor.
  Only uchar input is assumed.
  */
  
  mIn.m.release();
  
  switch ( pl )
  {
    case 1:
      cpy ? mIn.m = Mat( Size( w, h ), CV_8UC1, pData ).clone() :
        mIn.m = Mat( Size( w, h ), CV_8UC1, pData );

      break;

    case 3:
    {
      Mat in( w, h, CV_8UC3 );
      long len = w*h;
      Mat r( Size( w, h ), CV_8UC1, pData );
      Mat g( Size( w, h ), CV_8UC1, pData + len );
      Mat b( Size( w, h ), CV_8UC1, pData + 2 * len );

      // Mat is a fixed sized header. cv std color format is GBR
      Mat channels[] = { b, g, r };
      // Alternative: vector<Mat_<uchar>> channels(3);      
      if ( !this->mIsBGR ){
        channels[0] = r;
        channels[2] = b;
      }

      merge( channels, 3, in );
      // copy data into new instance (if needed)
      cpy ? mIn.m = in.clone() : mIn.m = in;

      break;
    }

    default:
      return false;
    
  }

  if (mIn.m.data){
    mResolution = Size(w, h);
    mHyp = (long)floor(sqrt( (double)w*w + (double)h*h) );
    return true;
  }else
    return false;
}


void Capture::setAngles(float pan, float tilt, float rot)
{
  mIn.angles[PAN] = pan;
  mIn.angles[TIL] = tilt;
  mIn.angles[ROT] = rot;
}


const Mat& Capture::getMat()
{
  return mIn.m;
}


void Capture::setMat(Mat &&src)
{
  mIn.m = src;
  int w = src.cols;
  int h = src.rows;
  mResolution = Size(w, h);
  mHyp = (long)floor(sqrt((double)w*w + (double)h*h));
}


void Capture::setMat(const Mat &src)
{
  mIn.m = src.clone();
  int w = src.cols;
  int h = src.rows;
  mResolution = Size(w, h);
  mHyp = (long)floor(sqrt((double)w*w + (double)h*h));
}


bool Capture::clearMat()
{
  mIn.m.release();
  return mIn.m.empty();
}

/*___disabled stuff_____________________________________________________________

Capture::~Capture()
{
mIn.m.deallocate(); //or m.free, but done automatically
}
unsigned char* Capture::getDataPtr()
{
return mIn.m.ptr();
}
float& Capture::operator[] (int idx)
{
return mIn.angles[idx];
}
const float& Capture::operator[] (const int idx) const
{
return mIn.angles[idx];
}
Size Capture::getSize()
{
return Size(mIn.m.cols ,mIn.m.rows);
}
double Capture::getDiagonalLen()
{
return mHyp;
}
double Capture::getFoc()
{
return mIn.nFoc*mHyp;
}
Capture::Capture(const InputData &src, bool isBGR) : mIsBGR(isBGR)
{
mIn.m = src.m.clone();
mIn.angles[PAN] = src.angles[PAN];
mIn.angles[TIL] = src.angles[TIL];
mIn.angles[ROT] = src.angles[ROT];
mIn.zoom = src.zoom;
int w = src.m.cols;
int h = src.m.rows;
mResolution = Size(w, h);
mHyp = (long)floor(sqrt((double)w*w + (double)h*h));
}
Capture& Capture::operator= (Capture &rh)
{
Capture(rh.mIn, rh.mIsBGR,false);
return *this;
}
/*
Capture& Capture::operator= (const Capture &rh)
{
Capture(rh.mIn, rh.mIsBGR,false);  //mat shared memory should handle instances
return *this;
}
*/