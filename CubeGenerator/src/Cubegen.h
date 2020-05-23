/**
  @file Cubegen.h
  @brief Classes to prepare cubemap textures
  
  Implementation: Cubegen.cpp
  Content:
  - Cubegen
  + Cubegen::Byte
  - AsyncCg
  
  @author Andre Halim
  @author Fraunhofer IOSB VID
  @date 2016, Apr.-Sept.
 */
 
//==================================================================================================

#ifndef CUBEGEN_H_
#define CUBEGEN_H_

#include "Capture.h"       // required argument and storage format for this class
#include <QThread>         // required for AsyncCg class
#include <QWaitCondition>  // required a background worker thread
//#include <QReadWriteLock>

//==================================================================================================

/// Prepares cubemaps based on image and positional input
/**
  Details, usage, links...
 */
class Cubegen
{
public:
  //____Declarations____________________________________________________________
  /// A small basic value type
  typedef unsigned char Byte;
  
  /// Indexes used in reference to a cubes side. Order of first 6 depend on OpenGL
  enum Sides{ CUBE_PX=0, CUBE_NX,
              CUBE_PY, CUBE_NY,
              CUBE_PZ, CUBE_NZ,
              CUBE_ALL, CUBE_NONE };
  
  //____Instance________________________________________________________________
  /// Creates a empty uninitialized instance
  Cubegen();
  
  /// Creates instance with all needed parameters
  Cubegen(double arrCam[], double arrCoeff[], int w, int h, Byte chann);
  
  ~Cubegen();
  
  /// initializes instance with non obvious parameters and ready it for operation.
  int init(double arrCam[], double arrCoeff1x5[], int w, int h, Byte chann, int maxSize=-1);
  
  //____Output__________________________________________________________________
  /// Returns edge lenght of the mapped cube
  int getCubeLen() const;
  
  /// Returns the used average focal lenght
  double getFoc() const;
  
  /// Returns index of the storage sector, holding the image taken with given angles.
  int findSector(float pan, float tilt) const;
  
  /// Returns index of the cubes side, seen at given angles.
  Sides findSide(double pan, double tilt) const;
  
  /// Applies undistortion on specified image, based on current initialized parameters.
  cv::Mat applyUndistort(const cv::Mat &distImg) const;
  
  //____Methods_________________________________________________________________
  /// Sets offset coordinates of a camera, not in worlds origin 0,0
  void setPivot(int dx, int dy, int dz);
  
  /// Process input image and updates cubemap upon.
  Sides renderThat(Capture &src, bool borders, bool warp, bool undistort=false, bool skip=true);
  
  /// Resets cubemap to a white sheet.
  void clearPlane(Sides k = CUBE_ALL);
  
  /// Exports cubemap to external memory space
  void writeAsPlanar(Sides k, unsigned char* data);
  
  /// Attempts to save cubemap as file.
  void printPlane( Sides k = CUBE_ALL );

  /// Construct a combined rotation matrix from rotation angles.
  static void mkRotMat( cv::Mat_<double> &m, double z1, double x2, double y3 );

  /// Helper function to calculate rotation angles in degree out of a transformation matrix
  static void transformationToAngle( const cv::Mat_<float> &tMat3x4, float &xRot, float &yRot, float &zRot );
  
  /*___Disabled_________________________________________________________________
  Cubegen(int apt, int w, int h, Byte chann);
  double init(int apt, int w, int h, Byte chann);
  int mLastAdded;  //only public mirror, shouldnt be used for internal cubegencode
  float getDistance() const;
  void setDistance(float dist);
  int getMaxSize() const;
  int setMaxSize(unsigned short size);
  */
  
protected:
  enum CamMatIdx{ CAM_FX=0, CAM_FY, CAM_CX, CAM_CY, CAM_DX, CAM_DY, CAM_DZ };

  //____Fields__________________________________________________________________
  std::vector<Capture>          mSectors;      ///< Image storage, sectors of a \
                                                    sphere. Size depends on FOV
  cv::Mat_<float>               mDistMaps[2];  ///< Maps for undistortion ([0]x, [1]y)
  std::vector<cv::Mat_<double>> mTransforms;   ///< transformation matrix to all \
                                                    processed sectors.
  std::vector<cv::Mat> mLUTs;  ///< precalculated pixel coordinate maps
  cv::Mat mPlanes;       ///< The cubemap, holding canvas of all cubesides.
  cv::Mat mSampleMap;    ///< Container holding indices of pixels to rasterize.
  double  mCamMat[7];    ///< Values describing a cameras properties
  float*  mArrSegIdx;    ///< LUT for each sectors angles span.
  int     mMaxH;         ///< Count of horizontal segments (logical columns)
  int     mMaxV;         ///< Count of vertical segments (logical rows)
  float   mSectorAngle;  ///< Angle that a sector spans (FOV)
  double  midFoc;        ///< Average focal lenght of the used camera.
  bool    mInitialized;  ///< Hints whether all required properties were set up.
  double  mScale;        ///< Scaling factor, with size limited cubemaps.
  float   mAngPrecision;  ///< Angle tolerance used to skip new transformation
  

  //____Methods_________________________________________________________________
  /// Feeds input data to process
  void addFrame(Capture &src);
  
  /// Command to draw borders onto a specified image
  void drawOutline(cv::Vec<cv::Point2d, 4> &src, cv::Mat& dst);
  
  void drawPerspective(int sect, cv::Mat &dst);
  
  /// Creates headers of cubemaps and initialize with white color
  void setupCube(int l, Byte chann);
  
  /// Calculates transformation matrix for a given sector
  cv::Mat_<double> prepareTransform( int sect );
  
  /// Calculates transformed indices to draw
  void Cubegen::generateLut( cv::Mat_<double> &m, int sect, bool undist );

  /// Returns canvas of a cubes side.
  cv::Mat matchedPlane(Sides k);
  
  /// Prepares mapping values for undistortion
  void undistortMaps(int x, int y, double arrCam1x4[], double arrCoeff1x5[]);
  
  /*___Disabled_________________________________________________________________
  //std::vector<cv::Vec<cv::Point2d, 4>> mEdges;  //atm used only for drawing borders
  cv::Point3i toCartesian(float pan, float tilt) const;  //[WIP] private pxl coordinates
  cv::Mat_<double> mDistCoeff;  //distortion coeff. for all sectors
  cv::Mat mDistMaps[2];
  float mDist;  //distance to near plane, normalized on span (hypotenuse of w&h)
  long mCubeLen;
  cv::Mat mROI;  //temp. mat header for ROIs (easier return), moved to function static
  int mMaxSize;
  */
};


/// Prepares cubemaps based on image and positional input in own thread
/**
  Details, usage, links...
 */
class AsyncCg : public QThread, protected Cubegen
{
  //____Declarations____________________________________________________________
  Q_OBJECT
  
public:
  using Cubegen::Sides;
  using Cubegen::init;
  using Cubegen::setPivot;
  using Cubegen::getCubeLen;
  using Cubegen::getFoc;
  using Cubegen::applyUndistort;
  using Cubegen::printPlane;
  //using Cubegen::getMaxSize;
  //using Cubegen::setMaxSize;
  
  struct Options{
    bool _borders;
    bool _warp;
    bool _undistort;
    bool _skip;
    Options() : _borders( false ), _warp( false ), _undistort( false ), _skip( false ){}
  };
  
  //____Instance________________________________________________________________
  AsyncCg(QObject *parent = 0);
  ~AsyncCg();

  //____Output__________________________________________________________________
  /// Returns if the tread is not ready for new input.
  bool isBusy() const;
  
  //____Methods_________________________________________________________________
  /// Configures the creation of cubemaps.
  void setOptions(bool borders=false, bool warp=false, bool undistort = false, bool NoSkip = false);
  
  /// Overloaded method, requires to always set all options.
  void setOptions(Options setting);
  
  /// Starts background thread to work on a input.
  void runWorkerAsync(Capture &src);
  
  /// Fetches processed image from thread.
  void getResult(cv::Mat& dst);
  
  //____Events__________________________________________________________________
signals:
  /// Signals a finished image is ready to fetch.
  void workerCompleted(unsigned char side);

protected:
  //____Methods_________________________________________________________________
  /// Used to start the threads loop for the first time.
  void run() Q_DECL_OVERRIDE;

private:
  //____Fields__________________________________________________________________
  QWaitCondition mCond; ///< Threads sleep state.
  QMutex  mutex;     ///< Threads synchronisation flag.
  bool    mAbort;    ///< Flag used to cancel the thread.
  bool    mNewData;  ///< Flag for unprocessed input existant.
  bool    mFetched;  ///< Flag for processed image has been fetched (at least once).
  Options mSetting;  ///< Holds option settings.
  Capture mInput;    ///< Copy of last given imput. No ringbuffer as calc is too slow.
  cv::Mat mOutput;   ///< Mirror of current processed image.
  
  /*___Disabled_________________________________________________________________
  bool mEnBorders;
  bool mEnWarp;
  bool mEnUndistort;
  bool mSkip;
  QReadWriteLock lock;
  */
};

//==================================================================================================

#endif //Cubegen_H_