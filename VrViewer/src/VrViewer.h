/**
  @file VrViewer.h
  @brief Classes to display 3D content using opengl functionality.
  
  Implementation: VrViewer.cpp
  Content:
  - VrViewer
  + VrViewer::VrVertex
  + VrViwer::Texture
  
  @author Andre Halim
  @author Fraunhofer IOSB VID
  @date 2016, Apr.-Sept.
 */
 
//==================================================================================================

#ifndef VRVIEWER_H
#define VRVIEWER_H

/// Provides OpenGL functions in code, targeted at a certain GL Version.
#include <QOpenGLFunctions_2_1>  //Qt5.1
/// Qt API to access GL functions
#include <QOpenGLWidget>  //Qt5.4
/// Needed to use index/ vertex buffers for more efficent GL procedures
#include <QOpenGLBuffer>
/// Qt Wrapper for GL shader functionality
#include <QOpenGLShaderProgram>
/// Only partially used. Raw GL access is favored here.
#include <QOpenGLTexture>
#include <QTimer>
#include <QOpenGlDebugLogger>

//==================================================================================================

/// Qt Widget to display a skybox using OpenGl v2.1
/**
  Extends QOpenGLWidget and implements a "skypbox-viewer" utilizing the OpenGL cubemap shader.
 */
class VrViewer : public QOpenGLWidget, protected QOpenGLFunctions_2_1
{
  Q_OBJECT

public:
  //____Public fields_______________________________________________________________________________
  bool mDrawFrame;  ///< Flag whether a overlay should be drawn.
  QVector<double> mFrameData;  ///< Buffer for current Overlay-Vertices.

  //____Declarations________________________________________________________________________________
  
  /// Simple struct to align data in memory
  struct VrVertex
  {
    GLfloat _x, _y, _z;  //vertex position
    GLfloat _s, _t;    //texture position
    VrVertex() : _x(0.f), _y(0.f), _z(0.f), _s(0.f), _t(0.f){};
    VrVertex(GLfloat x, GLfloat y, GLfloat z, GLfloat s, GLfloat t, float scale=1.f)
    : _x(x*scale), _y(y*scale), _z(z*scale), _s(s), _t(t){}
  };

  /// Texture header for...something, currently unused.
  struct VrTexture
  {
    const uchar* p;
    int w;
    int h;
    int chan;
    VrTexture() : p(NULL), w(0), h(0), chan(0){};
    VrTexture(const uchar* pxl, int width, int height, short channels)
    : p(pxl), w(width), h(height), chan(channels){};
  };

  //____Instance____________________________________________________________________________________
  VrViewer(QWidget *parent = 0);
  ~VrViewer();

  //____Public methods______________________________________________________________________________
  /// Makes a specific viewing angle visible on the presentation (Observer view).
  void setView(float jaw, float pitch, float roll = 0);
  void setView( const QVector<float>* m );

  /// Sets a specific texture (rgb or gray) for one side of the background cube.
  void setTex(const uchar* pImg, int w, int h, int channels, unsigned char cubeface);
  
  /// Sets near and farplane for clipping distances.
  void setRange(int nearf, int farf);
  
  /// Used to set size of the overlay, relative to background.
  void setFramesize(int w, int h, double focalLen);
  
  /// Sets a specific 2D texture for the overlay.
  void setFrame(const uchar* pImg, int w, int h, int channels);
  
  /// Overloaded method to use a qimage as texture for the overlay.
  void setFrame(const QImage &Img, int channels);
  
  /// Sets the position of overlay to given angle coordinates (Camera view).
  void setRotation(float pan, float tilt, float rotation=0.f);
  
  /// Load background textures from file.
  int loadCubeMapFile( QString path, QString name_, QString dotExt );
  
  //void setTex(QImage &rgbImg, char cubeface, bool isMirrored=false, bool forceRGB888=false);
  //QImage getView();

  /*
signals:
  void verticeChanged( QVector<double>* data);
  */

public slots:
  //there is also a update() slot, that schedules paintGL
  
  /// Sets vertical view in degrees.
  void setFov(int fov);
  
  /// Sets refresh rate of presentation to given rate in Hertz.
  void setRate(int hz);
  
protected:
  //____Fields______________________________________________________________________________________
  QOpenGLShaderProgram *mpFrameShade;  ///< shader handler (Frame).
  QOpenGLShaderProgram *mpCubeShade;   ///< shader handler (BG).
  QOpenGLDebugLogger* mLog;
  QOpenGLTexture* mpTex;  ///< pixel container.
  QTimer* mFrameserv;  ///< updates incomming textures.
  QOpenGLBuffer mVbo;  ///< vertex buffer object.
  QOpenGLBuffer mIbo;  ///< index buffer object.
  QMatrix4x4 mV;  ///< view matrix (world).
  QMatrix4x4 mP;  ///< projection matrix.
  QMatrix4x4 mR;  ///< Rotation matrix (frame).
  unsigned char mChangedFace;  ///< Mask for announcing changed textures.
  GLfloat mQuad[12];  ///< Holds 3D points of a static 2D frame.
  int mMaxCubeSize;   ///< Value for approx. maximal supported 3D texture size.
  int mMaxTexSize;  ///< Value for maximal supported 2D texture size.
  int mCurrSize[2];    ///< Value for current alllocated cubeside size
  GLuint mFrameId;  ///< Holds GL texture id for a generated 2D texture.
  GLuint mTexId;    ///< Holds GL texture id for a generated cubemap.
  float mNear;  ///< Value for near clipping plane.
  float mFar;   ///< Value for far clipping plane.
  float mFov;   ///< Vertical viewing angle of GL viewport.
  bool mWait;   ///< Used to halt rendering into mainbuffer.
  //____Methods_____________________________________________________________________________________
  
  // GL enabled methods
  /// API method for gl specific initialization.
  void initializeGL() Q_DECL_OVERRIDE;
  
  /// API method for settings to apply after sceen size change.
  void resizeGL(int width, int height) Q_DECL_OVERRIDE;
  
  /// API method to update/refresh view.
  void paintGL() Q_DECL_OVERRIDE;

  // non GL enabled methods. Use makeCurrent() to get GL-Context
  /// GL specific garbage collection
  void freeGL();
  
  /// Transferes 3D texture data to hardware.
  /**
    Context must be set pre-call!
   */
  void uploadCubeMap(const void* src, int w, int h, QOpenGLTexture::PixelFormat pf, short face);
  
  /// Treansferes 2D texture data to hardware.
  /**
    Context must be set pre-call!
   */
  void uploadTex(const void* src, int w, int h, QOpenGLTexture::PixelFormat pf, GLuint id);
  
  /// Method dedicated to constructing vertex data.
  virtual void construct();

protected slots:
  /// Callback function for timer to update changed textures.
  void checkTex();
  void onMessageLogged( QOpenGLDebugMessage );
  void onCompose();
  void onSwapped();
};

#endif //VRVIEWER_H