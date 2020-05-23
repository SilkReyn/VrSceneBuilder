/**
  @file Cubegen.cpp
  @brief Classes to prepare cubemap textures
  
  Declaration: Cubegen.h
  
  @author Andre Halim
  @author Fraunhofer IOSB VID
  @date 2016, Apr.-Sept.
 */

//==================================================================================================

//#define CG_DPHI 0.04

#include "Cubegen.h"
#include <QDebug>

using namespace std;
using namespace cv;

template<typename T>
T unsign(T val)
{
  return val<0 ? -val : val;
}

/*
inline long lRound(double val)
{
  return val - floor(val) >= 0.5 ? static_cast<long>(ceil(val)) : static_cast<long>(floor(val));
}
*/

//==================================================================================================

#pragma region baseclass

//____Instance__________________________________________________________________

Cubegen::Cubegen()
  : mMaxH(0), mMaxV(0), mSectorAngle(0.0f)
{
  // Camera params fx,fy,cx,cy
  mCamMat[0] = 1.; mCamMat[1] = 1.; mCamMat[2] = 0.; mCamMat[3] = 0.;
  // Pivot offset dx dy dz
  mCamMat[4] = 0.; mCamMat[5] = 0.; mCamMat[6] = 0.;
  mInitialized = false;
  mArrSegIdx = NULL;
  mTransforms.clear();
  mLUTs.clear();
  mScale = 1.;
  mAngPrecision = 0.006f;
}


Cubegen::Cubegen(double arrCam[], double arrCoeff[], int w, int h, Byte chann)
{
  Cubegen();
  init(arrCam, arrCoeff, w, h, chann);
}


Cubegen::~Cubegen()
{
  if (mArrSegIdx)
    delete[] mArrSegIdx;
}


int Cubegen::init(double arrCam1x9[], double arrCoeff1x5[], int w, int h, Byte chann, int maxSize)
{
  mSectors.clear();  // content becomes outdated
  mSectorAngle = 0.f;

  // biggest capacity demand for samplermap:
  // width*height*2channel*4Byte(float)
  long long bytes = w * h * 8;
  if (bytes < 1 || bytes > (1<<28)){
    qDebug( "Cubegen::init(): Bad image size, init failed" );
    return 0;
  }

  // fx, fy, cx, cy:
  mCamMat[0] = arrCam1x9[0];
  mCamMat[1] = arrCam1x9[4];
  mCamMat[2] = arrCam1x9[2];
  mCamMat[3] = arrCam1x9[5];

  if ((arrCam1x9[0] * arrCam1x9[4]) <= 0.0){
    qDebug( "Cubegen::init(): invalid focal lenght, init failed" );
    return 0;
  }

  //midFoc = (mCamMat[CAM_FX] + mCamMat[CAM_FY]) * 0.5;
  double aperture = 0.;
  double d = 0;
  // zero division guard
  if (arrCam1x9[0] > 0.0 && arrCam1x9[4] > 0.0){
    // One sector should span the cameras smallest field of view
    // Here aperture is half spanned angle in radian
    // d is the hypotenuse, used for precision calculation
    if (h < w){  //v_aperture in radians
      midFoc = 0.5*w*arrCam1x9[0] / arrCam1x9[2];
      aperture = atan( (double)h / (2. * arrCam1x9[4]) );
      d = sqrt( midFoc*midFoc + w*w * 0.25 );
    }
    else{      //h_aperture
      midFoc = 0.5*h*arrCam1x9[4] / arrCam1x9[5];
      aperture = atan( (double)w / (2. * arrCam1x9[0]) );
      d = sqrt( midFoc*midFoc + h*h * 0.25 );
    }
    
    aperture = unsign( aperture );
  }
  if (aperture >= CV_2PI || aperture == 0.){
    qDebug( "Cubegen::init(): Unreal View, init failed. (Camera parameters set?)" );
    return 0;
  }

  // change in angle at border areas, to differ more than 1pxl
  // a rough estimation is mSectorAngle/w
  if (d != 0)
    mAngPrecision = unsign<float>( (asin( 1. / d - sin( aperture ) ) + aperture) / CV_PI * 180. );
  else
    mAngPrecision = aperture / (CV_PI*w) * 180.;

  // full span aperture
  mSectorAngle = (float)(aperture / CV_PI * 360.);

  // Horizontal& vertical sphere rasterization
  mMaxH = static_cast<int>(ceil(360.f / mSectorAngle));
  mMaxV = static_cast<int>(ceil(180.f / mSectorAngle));
  int cnt = mMaxH*mMaxV;
  if (mArrSegIdx)
    delete[] mArrSegIdx;

  mArrSegIdx = new float[mMaxH];
  if (!mArrSegIdx){
    qDebug( "Cubegen::init(): failed to allocate index" );
    return 0;
  }

  // Marking first range exclusive angle per sector
  for (int i = 0; i < mMaxH; i++)
    mArrSegIdx[i] = (i + 1)*mSectorAngle;

  mSectors.resize(cnt);  //adds n empty sectors (we expect existing!)
  mLUTs.resize( cnt );
  mTransforms.resize(cnt);
  //mEdges.resize(cnt);

  // Side of a cubeplane 2*focallenght*cos(45)
  int cubeLen = qRound(sqrt(2.)*midFoc);
  if (maxSize > 1 && cubeLen > maxSize){
    mScale = (double)maxSize / cubeLen;
    cubeLen = maxSize;
  }else{//reset
    mScale = 1.;
  }
  qDebug() << "BG Tex. Size: " << cubeLen << " Scale: " << mScale;
  setupCube(cubeLen, chann);
  
  /* Sample map for corrected image space.
  // Fill initial indices (with subpixel precision).
  // One pixel is missing at right&bottom (osz-0.5).
  mSampleMap.create(Size(2*w,2*h), CV_32FC2);
  for (int y = 0;y < mSampleMap.rows; y++){
    float* p = mSampleMap.ptr<float>(y);
    for (int x = 0; x < mSampleMap.cols; x++){
      *p++ = x / 2.f;
      *p++ = y / 2.f;
    }
  }
  */
  // indices to source image. used as map for LUT generation
  // function 'perspectiveTransform' expects floating point, mandatory
  mSampleMap.create( Size( w, h ), CV_32FC2 );
  for (int y = 0; y < mSampleMap.rows; y++){
    float* p = mSampleMap.ptr<float>( y );
    for (int x = 0; x < mSampleMap.cols; x++){
      *p++ = x;
      *p++ = y;
    }
  }

  // Must be called to enable distortion
  undistortMaps(w, h, mCamMat, arrCoeff1x5); 
  
  mInitialized = true;
  return cubeLen;
}


//____Output____________________________________________________________________

Mat Cubegen::applyUndistort(const Mat &distImg) const
{
  Mat dst(mDistMaps[0].size(),distImg.type());
  remap(distImg, dst, mDistMaps[0], mDistMaps[1], INTER_LINEAR, BORDER_TRANSPARENT);
  return dst;
}


int Cubegen::findSector(float pan, float tilt) const
{// accepts +/-angles relative to front = 0, tilt max +/-90°
  if (!mInitialized) return 0;
  // Enough sectors to make this reasonable?
  if (mMaxH < 2) return 0;  
  
  // Start in middle of sector (offset)
  pan += mSectorAngle / 2.0f;
  
  //if multiple of 360
  if (unsign(pan) >= 360.0f) pan -= (int)(pan / 360.0f) * 360;
  // unnötig und negativ fehlt:
  //if (pan > 180.0f) pan = -(360.0f - pan);
  
  int i,j;
  // Positive
  if (pan >= 0){
    for (i = 0; i < mMaxH; i++)
      if (pan < mArrSegIdx[i]) break;
  // Negative
  }else{
    // assume only positve
    pan += 360.0f;
    // mMaxH must be >1 !
    for (i = mMaxH-1; i > 0; i--)
      if (pan >= mArrSegIdx[i-1]) break;
  }
  
  // bottom==0°;
  // tilt >=180 -> in last sector
  tilt += 90.0f;
  for (j = 0; mArrSegIdx[j] <= 180.0f; j++)
    if (tilt < mArrSegIdx[j]) break;

  return j * mMaxH + i;
}


Cubegen::Sides Cubegen::findSide(double pan, double tilt) const
{// Similar to findSector, but for Cubesides
  Sides idx=CUBE_PX;
  //if multiple of 90
  if (unsign(tilt) >90.) tilt -= (int)(tilt / 90.) * 90;
  
  if (tilt >= 45.)
    return CUBE_PY;  //above
  
  if (tilt <= -45.)
    return CUBE_NY;  //below
  else{
    //if multiple of 360
    if (unsign(pan) >= 360.) pan -= (int)(pan / 360.) * 360;
    //assume only positve
    if (pan < 0.) pan += 360.;
    
    Byte k = 0;
    for (double phi = 45.; phi < 360.; phi += 90.)
      if (pan < phi) break; else k++;    //borders are next plane
    
    switch (k)
    {
      case 1: idx = CUBE_NX; break;  //left
      case 2: idx = CUBE_PZ; break;  //back
      case 3: idx = CUBE_PX; break;  //right
      default: idx = CUBE_NZ; break;  //front
    }
  }
  return idx;
}


int Cubegen::getCubeLen() const
{//height = width. This is not width of mPlanes
  return mPlanes.rows;
}


double Cubegen::getFoc() const
{
  return midFoc;
}


//____Methods___________________________________________________________________

Cubegen::Sides Cubegen::renderThat(Capture &src, bool borders, bool warp, bool undistort, bool skip)
{
  //mPlanes must be set before!
  if (!mInitialized) return CUBE_NONE;
  
  float p = src.pan(), t = src.tilt();
  int n = findSector(p, t);
  
  //compare old angles
  float dP = unsign(p - mSectors[n].pan());
  float dT = unsign(t - mSectors[n].tilt());
  
  Sides k = findSide(p, t);
  Mat dst = matchedPlane(k);
  
  mSectors[n] = src;

  // update translation matrix if angles differ
  if (max( dP, dT ) > mAngPrecision || !skip){
    //Mat_<double> transform = prepareTransform( n );
    mTransforms[n] = prepareTransform( n );

    // heavy part: LUT generation
    if (warp && undistort)
      generateLut( mTransforms[n], n, true );
  }

  if (borders){
    // borders of original picture
    Vec<Point2d, 4> frame;
    // vertices of transformed frame
    Vec<Point2d, 4> edges;
    Size sze = src.mResolution;

    // paint couter-clockwise (OGL)
    frame[0] = Point2d( 0, 0 );
    frame[1] = Point2d( sze.width, 0 );
    frame[3] = Point2d( 0, sze.height );
    frame[2] = Point2d( sze.width, sze.height );

    perspectiveTransform( frame, edges, mTransforms[n] );
    drawOutline( edges, dst );
  }

  // second heavy part: pixel transfer
  if (warp){
    if (undistort)
      drawPerspective( n, dst );
    else
      warpPerspective( mSectors[n].getMat(), dst, mTransforms[n], dst.size(), INTER_LINEAR, BORDER_TRANSPARENT );
  }

  // delete source image to free memory, it is not used later.
  if (!(mSectors[n].clearMat()))
    qDebug( "Cubegen: Memory persist" );

  return k;
}


void Cubegen::addFrame(Capture& src)
{
  int n = findSector( src.pan(), src.tilt() );
  mSectors[n] = src;
}


Mat_<double> Cubegen::prepareTransform( int sect )
{// generate projected edges and translation matrix of sector

  //halbe bildbreite
  double hw = mSectors[sect].mResolution.width*0.5;
  double hh = mSectors[sect].mResolution.height*0.5;
  //halbe Cubeseite
  double hl = mPlanes.rows * 0.5;
  
  //applied zoom on average focal lenght
  double z = (double)mSectors[sect].zoom() * midFoc;
  //new camera focal lenght
  double f = 1. / sqrt(2.) * midFoc;
  //double fx = hl*mCamMat[0] / mCamMat[2];
  //double fy = hl*mCamMat[1] / mCamMat[3];
  double phi = mSectors[sect].pan();
  double alpha = mSectors[sect].tilt();
  //pan, tilt, rot (new cam)
  double a, b, c;
  
  // new cam rotation relative to coordinate axes
  // the cube-sides are those used in opengl. PX:right, NZ:front, PY:top
  // this function utilized cv::remap, whose y-axis is reversed (pointed down),
  // as the origin is in top/left edge of the picture (Image still upside).
  // all y-axis rotation must be reversed using right-hand-system.
  // Here positive z-axis points into picture plane and considered front,
  // where in GL, NZ is front side (reversed)
  switch (findSide( phi, alpha ))
  {
    case CUBE_PX:
      a = 0.5 * CV_PI;
      c = b = 0;
      break;

    case CUBE_NX:
      a = -0.5 * CV_PI;
      c = b = 0;
      break;

    case CUBE_PY:
      b = 0.5 * CV_PI;
      a = c = 0.;
      break;

    case CUBE_NY:
      b = -0.5 * CV_PI;
      a = c = 0;
      break;

    case CUBE_PZ:
      a = CV_PI;
      c = b = 0;
      break;

    case CUBE_NZ:
      a = b = c = 0.;
      break;

    default:
      a = b = c = 0.;
  }

  //in case a kinda gyroscope thing is used...
  if (unsign(alpha) > 90.){
    if (alpha >= 0)
      alpha = 180. - alpha;
    else
      alpha = -180. - alpha;
  }

  //in rad
  phi = phi / 180. * CV_PI;
  alpha = alpha / 180. * CV_PI;

  // move 2D image into 3D space {4x3 * (x,y,1) = 4x1}
  // positive Z-Axis directs to viewer, front is -Z
  // Lens is at world origin
  Mat M = (Mat_<double>(4, 3) <<
    1, 0, -hw,  //x (move image left)
    0, 1, -hh,  //y (move up)
    0, 0, z,   //z (move away)
    0, 0, 1);

  // Lens position in World (Offset to Origin)
  Mat O = (Mat_<double>(4, 4) <<
    1, 0, 0, mCamMat[4],  //dx
    0, 1, 0, -mCamMat[5],  //dy
    0, 0, 1, mCamMat[6],  //dz
    0, 0, 0, 1);

  // New intrinsic matrix (camera params)
  // calibrated y face down, x right, optical axis in z direction
  Mat C = (Mat_<double>(3, 4) <<
    f, 0, hl, 0,
    0, f, hl, 0,
    0, 0, 1, 0);

  // 2D Scaling Matrix
  Mat S = (Mat_<double>(3, 3) <<
    mScale, 0, (1.-mScale)*hl,
    0, mScale, (1.-mScale)*hl,
    0, 0, 1);

  /* If new cam positioned at real cam position
  Mat_<double> R, P, H, T1;
  Mat_<double> T2 = Mat::eye( 4, 4, CV_64F );
  Mat_<double> E(4, 4);
  // R=New Cam Orientation
  mkRotMat(R, c, b, a);
  // world rotation relative to cam
  E = R.t();

  // R=picture rotation  
  mkRotMat(R, 0., alpha, phi);

  // rotate offset, copy only tranlation vector
  T1 = (R*O);
  T2( 0, 3 ) = T1( 0, 3 );
  T2( 1, 3 ) = T1( 1, 3 );
  T2( 2, 3 ) = T1( 2, 3 );
  T1 = E*T2;
  
  // E=Extrinsic matrix. Only copy offset
  E(0, 3) = T1(0,3);
  E(1, 3) = T1(1,3);
  E(2, 3) = T1(2,3);
  */

  Mat_<double> R, E, P, H;
  // R=New Cam Orientation
  mkRotMat( R, c, b, a );
  // E=extrinsic matrix, world rotation relative to cam
  E = R.t();
  // P=camera matrix
  P = C * E;

  // R=picture rotation (z-axis ignored)
  mkRotMat( R, 0., alpha, -phi );

  // homography matrix
  H = S*P*R*O*M;
  return H;
}


void Cubegen::mkRotMat(Mat_<double> &m, double z1, double x2, double y3)
{// Helper function to produce a rotation 4x4matrix. Parameter are angles in radian,
 // right-hand system used (positve angles are leftrotation).
 // parameters state the rotated axis (z,x,y) and order in which they rotated (1,2,3)

  // Rotation x-axis (leftrotation)
  Mat RX = (Mat_<double>(4, 4) <<
    1, 0, 0, 0,
    0, cos(x2), -sin(x2), 0,
    0, sin(x2), cos(x2), 0,
    0, 0, 0, 1);

  // Rotation y-axis (leftrotation)
  Mat RY = (Mat_<double>(4, 4) <<
    cos(y3), 0, sin(y3), 0,
    0, 1, 0, 0,
    -sin(y3), 0, cos(y3), 0,
    0, 0, 0, 1);

  // Rotation z-axis (leftrotation)
  Mat RZ = (Mat_<double>(4, 4) <<
    cos(z1), -sin(z1), 0, 0,
    sin(z1), cos(z1), 0, 0,
    0, 0, 1, 0,
    0, 0, 0, 1);

  m = RY*(RX*RZ);
}


void Cubegen::transformationToAngle( const cv::Mat_<float> &tMat, float &xRot, float &yRot, float &zRot )
{
  // code based on Michael Trapp: Lighthouse Plugin::TrackedDevice.cpp; Sept.2016
  double rot[3];
  rot[2] = asin( (double)tMat( 0, 2 ) );  // Y
  rot[1] = -asin( (double)tMat( 1, 2 ) / cos( (double)rot[2] ) );  // X
  rot[0] = -asin( (double)tMat( 0, 1 ) / cos( (double)rot[2] ) );  // Z

  for (Byte i = 0; i < 3; i++){
    // the corresponding axis is rotated more than 90 degree
    if (tMat(i,i) < 0.)
      rot[i] = CV_PI - rot[i];
    
    // the corresponding axis is rotated more than 270 degree
    if (rot[i] < 0.)
      rot[i] += CV_2PI;

    // convert to degree
    rot[i] = rot[i] / CV_PI * 180.;
  }

  zRot = (float)rot[0];
  xRot = (float)rot[1];
  yRot = (float)rot[2];
}

void Cubegen::drawOutline(Vec<Point2d,4> &src,Mat& dst)
{
  //Mat src = matchedPlane(sect);
  //line(src, mEdges[sect][0], mEdges[sect][1], Scalar(0, 0, 255), 10);
  line(dst, src[0], src[1], Scalar(0, 0, 255), 10);
  line(dst, src[1], src[2], Scalar(0, 0, 255), 10);
  line(dst, src[2], src[3], Scalar(0, 0, 255), 10);
  line(dst, src[3], src[0], Scalar(0, 0, 255), 10);
}


void Cubegen::generateLut( cv::Mat_<double> &m, int sect, bool undist )
{
  // not calculated?
  //if (mTransforms[sect].empty()) return;
  if (m.empty()) return;

  // new size
  Size nsz( getCubeLen(), getCubeLen() );

  // translation map, will be of type CV_32FC2, dimension of source
  Mat t_map;

  // round&combined map of dst_map
  Mat imap1, imap2;

  // calculates coordinates in source image space
  perspectiveTransform( mSampleMap, t_map, m );

  // discrete index for linking. cast float-> int as alternative (but does only floor)
  convertMaps( t_map, Mat(), imap1, imap2, CV_16SC2, true );

  if (undist){
  
    // maps original position to new position
    // negative values marks nonexistant (transparent)
    Mat_<float> dst_Xmap( nsz.height, nsz.width, -1.f );
    Mat_<float> dst_Ymap( nsz.height, nsz.width, -1.f );

    /*double size sample map:
    int j, i, x, y;
    j = i = -1;
    // link translation and distortion maps for remap
    for (y = 0; y < imap1.rows; y++){
    short* p = imap1.ptr<short>(y);
    for (x = 0; x < imap1.cols; x++, p += 2){
    // check if repeated write (only previous)
    if (p[0] == i){
    if (p[1] == j)
    continue;
    }

    // cols & rows
    i = p[0]; j = p[1];
    // skip if a index is out of range
    if (i < 0 || j < 0 || i >= nsz.width || j >= nsz.height) continue;

    // discrete! (floor, makes sure index is in range)
    int m = y / 2, n = x / 2;
    dst_Xmap(j, i) = (mDistMaps[0])(m, n);
    dst_Ymap(j, i) = (mDistMaps[1])(m, n);
    }
    }
    */

    // single size sample map (less operations and mem, but some coordinates
    // are lost due to rounding indices)
    int i, j, x;
    for (int y = 0; y < imap1.rows; y++){
      short* p = imap1.ptr<short>( y );
      for (x = 0; x < imap1.cols; x++, p += 2){
        // coordinates of transformed pixel i=column, j=rows
        i = p[0]; j = p[1];
        // skip if a index is out of range
        if (i < 0 || j < 0 || i >= nsz.width || j >= nsz.height) continue;

        dst_Xmap( j, i ) = (mDistMaps[0])(y, x);
        dst_Ymap( j, i ) = (mDistMaps[1])(y, x);
      }
    }

    // pre-converted fixed point map runs faster
    convertMaps( dst_Xmap, dst_Ymap, imap1, imap2, CV_16SC2, true );
    mLUTs[sect] = imap1;
    //changed to inter_nearest and nninterpol true, to omit map2
    //mLUTs[sect+1].map2 = imap2;

  }else{
    //just copy the transformed coordinates into the bigger plane map
    Mat dst_XYmap( nsz, CV_16SC2, Scalar::all(-1) );
    int x,i,j;
    for (int y = 0; y < imap1.rows; y++){
      short* pi = imap1.ptr<short>( y );
      for (x = 0; x < imap1.cols; x++){
        i = *pi++; j = *pi++;
        if (i < 0 || j < 0 || i >= nsz.width || j >= nsz.height) continue;
        dst_XYmap.at<Vec2s>( j, i ) = Vec2s(x,y);
      }
    }
    mLUTs[sect] = dst_XYmap;
  }
}


void Cubegen::drawPerspective(int sect, Mat &dst)
{
  // not calculated?
  if (mLUTs[sect].empty()) return;

  remap( mSectors[sect].getMat(), dst,
    mLUTs[sect], Mat(),
    INTER_NEAREST, BORDER_TRANSPARENT );
}


void Cubegen::undistortMaps(int x, int y, double arrCam1x4[], double arrCoeff1x5[])
{
  Mat mDistCoeff = (Mat_<double>(5, 1) <<  //must be a cv-array for function compability
    arrCoeff1x5[0], arrCoeff1x5[1], arrCoeff1x5[2], arrCoeff1x5[3], arrCoeff1x5[4]);

  // size of distorted image
  Size osz(x, y);
  // camera matrix from cali-file
  Mat cm = (Mat_<double>(3, 3) <<
    arrCam1x4[0], 0, arrCam1x4[2],
    0, arrCam1x4[1], arrCam1x4[3],
    0, 0, 1);
    
  /*
  // camera for undistored virtual capture
  Mat ncm = (Mat_<double>(3, 3) <<
  midFoc, 0, osz.width/2,
  0, midFoc, osz.height/2,
  0, 0, 1);
  
  // finds params, so that ncm results similar to original image (considering distortion)
  Mat ncm = getOptimalNewCameraMatrix(cm, mDistCoeff, osz, 0, osz);
  mDistMaps[0].create(osz);
  mDistMaps[1].create(osz);
  */

  initUndistortRectifyMap( cm, mDistCoeff, Mat_<double>::eye( 3, 3 ), cm, osz, mDistMaps[0].type(), mDistMaps[0], mDistMaps[1] );
}


void Cubegen::writeAsPlanar(Sides k, unsigned char* pDst)
{// Planar Image Format. Only uchar. pDst must have allocated size of src: x*y*c
  if (k == CUBE_NONE) return;

  Mat src;
  int w, h, i, j, x, y;

  // [todo]:correct positional write if pdst.size > src.size
  if (k == CUBE_ALL)
    src = mPlanes;
  else
    src = matchedPlane(k);
  
  w = mPlanes.cols;
  h = mPlanes.rows;
  x = src.cols;
  y = src.rows;
  
  unsigned char* pIn;

  switch (src.channels())
  {// writes src left aligned into memory
    case 3:
    {
      unsigned char* pg = pDst + w * h;
      unsigned char* pb = pDst + 2 * w * h;
      
      for ( i = 0; i < y; i++ ){
        pIn = src.ptr<uchar>( i );
        // use cv::satureate_cast if different depth
        for ( j = 0; j < x; j++ ){
          *pb++ = *pIn++;    //b
          *pg++ = *pIn++;    //g
          *pDst++ = *pIn++;  //r
        }
      }
      break;
    }
    case 1:
      // direct copy
      x *= y;
      pIn = src.ptr<uchar>(0);
      for (i = 0; i < x; i++)
        *pDst++ = *pIn++;

      break;
    
    default: qDebug("Unsupported channelcount");
  }
}


// sets size of cubeplane, based on radius to imageplane
void Cubegen::setupCube(int l,Byte chann)
{
  switch (chann)
  {
    // row,col order
    case 1: mPlanes.create(l, 6*l, CV_8UC1); break;
    case 3: mPlanes.create(l, 6*l, CV_8UC3); break;
    default: qDebug("Unsupported channelcount");
  }
  // set white BG (create doesnt support inital values)
  mPlanes = Scalar_<uchar>::all(255);
}


void Cubegen::clearPlane(Sides k)
{
  if (!mInitialized) return;
  if (k == CUBE_ALL){
    // set white paper
    mPlanes = Scalar_<uchar>::all(255);
    mSectors.clear();
    // dangerous if max not defined!
    mSectors.resize(mMaxH*mMaxV);
  }else{
    int l = mPlanes.rows;
    Rect roi(k*l, 0, l, l);
    Mat plane = Mat(mPlanes, roi);
    plane = Scalar_<uchar>::all(255);
    //one side consists of several sectors. they are not cleared here
  }
}


void Cubegen::printPlane( Sides k )
{
  if (!mInitialized || k>6) return;
  if (k == CUBE_ALL)
  {
    imwrite( "CUBEMAP_full.png", mPlanes );
  } else{

    imwrite( "CUBEMAP_" + QString::number((int)k).toStdString() + ".png", matchedPlane( k ) );
  }
}


Mat Cubegen::matchedPlane(Sides k)
{// updates ROI based on sector and returns it
  int x, l = mPlanes.rows;
  k > 5 ? x = 0 : x = k*l;
  Rect roi(x, 0, l, l);
  return Mat(mPlanes, roi); //sROI;
}


void Cubegen::setPivot(int dx, int dy, int dz)
{
  mCamMat[4]=dx;
  mCamMat[5]=dy;
  mCamMat[6]=dz;
}

#pragma endregion baseclass

//==================================================================================================
/*
 * Asynchronous Implementation of Cubegen
 */

AsyncCg::AsyncCg(QObject *parent)
    : QThread(parent), Cubegen(), mSetting()
{
  //make the enum known to QObject::connect
  //qRegisterMetaType<Cubegen::Sides>("Cubegen::Sides");

  mAbort = false;
  mNewData = false;
  mFetched = true;
  
  /*
  mEnBorders = true;
  mEnWarp = false;
  mEnUndistort = false;
  mSkip = false;
  */
}

AsyncCg::~AsyncCg()
{
  mutex.lock();
  mAbort = true;
  mCond.wakeOne();
  mutex.unlock();

  //wait until run() returns
  wait();
}


void AsyncCg::setOptions(bool borders, bool warp, bool undistort, bool NoSkip)
{
  //QMutexLocker locker(&mutex);
  Options o;
  o._borders = borders;
  o._warp = warp;
  o._undistort = undistort;
  o._skip = !NoSkip;
  mSetting = o;
  /*
  this->mSkip = !NoSkip;
  this->mEnBorders = borders;
  this->mEnWarp = warp;
  this->mEnUndistort = undistort;
  */
}


void AsyncCg::setOptions(Options setting)
{
  mSetting = setting;
}


void AsyncCg::runWorkerAsync(Capture &src)
{// make temporary copy and invoke worker
  QMutexLocker locker(&mutex);
  
  // previous work is lost if no time to process
  this->mInput = src.clone();
  // signals new work
  this->mNewData = true;
  
  if (mInitialized){
    if (!isRunning())
      start();
    else
      mCond.wakeOne();
      // waiting for either fetch or newData
      // wake if condition sleeping
  }
}

void AsyncCg::run()
{
  while (!mAbort)
  {
    mutex.lock();
    //Capture src = mInput.clone();
    mNewData = false;
    //Sides k =renderThat(mInput, mEnBorders, mEnWarp, mEnUndistort, mSkip);
    Sides k=renderThat(mInput,mSetting._borders, mSetting._warp, mSetting._undistort, mSetting._skip);
    mutex.unlock();

    //make sure we can read plane k and not plane k+1
    while (!(mFetched || mAbort)) { msleep(1); }
      //mCond.wait(&mutex, 10);
    
    mutex.lock();  
    mOutput = matchedPlane(k);  //mirrored to output!
    mFetched = false;
    emit workerCompleted(static_cast<unsigned char>(k));
    
    //is new Work available?
    if (!mNewData && !mAbort)
      mCond.wait(&mutex);  //manages lock while sleeping
    mutex.unlock();
  }
  qDebug("AsyncCg::run Background-Worker quit");
}


void AsyncCg::getResult(Mat& dst)
{
  QMutexLocker locker(&mutex);
  //mOutput holds last processed cubeside
  dst = mOutput.clone();
  //assure only 1 sucessful fetch
  mFetched = true;
  
  /*possible to wake when at end of loop (undesired)
  mCond.wakeOne();*/
}


bool AsyncCg::isBusy() const
{//can it accept new Data?
  if (isRunning() && mNewData)
    return true;
  else
    return false;
}