// [TODO]
// uploadTex -> background thread (pixeltransfer)
// setTex || uploadCubeMap -> background thread (mirror & pixeltransfer)
// make use of glfunction: SubTex
// utilize mChangedFace - checkTex more

/**
  @file VrViewer.cpp
  @brief Classes to display 3D content using opengl functionality.
  
  Declaration: VrViewer.h
  
  @author Andre Halim
  @author Fraunhofer IOSB VID
  @date 2016, Apr.-Sept.
 */
//==================================================================================================

#include "VrViewer.h"

//#include <qopenglpixeltransferoptions.h>

/// Count of indexes referencing cube vertices.
#define VR_IDX_CNT 14
/// Switch to use raw opengl texturing or to use qt's textures.
#define VR_RAW_TEX
/// enable GL Log messages
#ifndef NDEBUG
  #define VR_LOG
#endif

// now unused
//#define VR_MAX_TEX_SIZE 1672
//#define VR_MAX_FRAME_SIZE 4096

/// Texture xy-coordinates used for the overlay.
const GLfloat VR_2D_CORNERS[] = {
  1.f, 0.f,
  0.f, 0.f,
  0.f, 1.f,
  1.f, 1.f };

//==================================================================================================

  
VrViewer::VrViewer(QWidget *parent)
  : QOpenGLWidget(parent), mIbo(QOpenGLBuffer::IndexBuffer)
{
  // no GL ressource here! use initializeGL instead
  mChangedFace = 0;
  mpCubeShade = NULL;
  mpFrameShade = NULL;
  mNear = 0.23f; mFar = sqrt(3.f);
  mFov = 45.f;
  mFrameserv = NULL;
  mTexId = 0;
  mFrameId = 0;
  mpTex = NULL;
  mDrawFrame = true;
  mMaxTexSize = 2048;
  mFrameData.resize( 12 );
  mLog = NULL;
  mCurrSize[0] = 0;
  mCurrSize[1] = 0;
  // mQuad normalized frame vertice pts (x,y,z)
  // init here, time of call initializeGL() is uncertain and may overwrite setFormatsize()
  // z is lenght from origin to farthest cube-edge
  float z = -sqrt( 2.f ); // -1.424f;
  mQuad[0] = 1.f; mQuad[1] = 1.f; mQuad[2] = z;  //TR
  mQuad[3] = -1.f; mQuad[4] = 1.f; mQuad[5] = z;  //TL
  mQuad[6] = -1.f; mQuad[7] = -1.f; mQuad[8] = z;  //BL
  mQuad[9] = 1.f; mQuad[10] = -1.f; mQuad[11] = z;  //BR

  //initializeGL(); crashes with "context" assert
}


VrViewer::~VrViewer()
{// if GL ressource, call makeCurrent()...doneCurrent()
  if (mFrameserv != NULL){
    if (mFrameserv->isActive())
      mFrameserv->stop();
    delete mFrameserv;
    mFrameserv = NULL;
  }
  
  freeGL();
}


void VrViewer::freeGL()
{
  makeCurrent();

  mVbo.destroy();
  mIbo.destroy();
  
#ifdef VR_RAW_TEX
  if ( mTexId != 0 && mFrameId != 0 ){
    GLuint textures[] = { mTexId, mFrameId };
    glDeleteTextures(2, textures);
    mTexId = mFrameId = 0;
  }else{
    if ( mTexId != 0 ){
      glDeleteTextures( 1, &mTexId );
      mTexId = 0;
    }
    if ( mFrameId != 0 ){
      glDeleteTextures( 1, &mFrameId );
      mFrameId = 0;
    }
  }
#else
  if (mpTex != NULL)
  {
    if (mpTex->isCreated())
      mpTex->destroy();
    delete mpTex;  // calls QOpenGlTexture::destroy()
    mpTex = NULL;
  }
#endif
  if (mpCubeShade != NULL){
    delete mpCubeShade;
    mpCubeShade = NULL;
  }
  if (mpFrameShade != NULL){
    delete mpFrameShade;
    mpFrameShade = NULL;
  }

  if (mLog){
    delete mLog;
    mLog = NULL;
  }

  doneCurrent();
  qDebug("VrViewer::freeGL() done");
}


void VrViewer::initializeGL()
{
  initializeOpenGLFunctions();
  //connect(context(), &QOpenGLContext::aboutToBeDestroyed, this, &VrViewer::freeGL);
  connect( this, &QOpenGLWidget::aboutToCompose, this, &VrViewer::onCompose );
  connect( this, &QOpenGLWidget::frameSwapped, this, &VrViewer::onSwapped );

#ifdef VR_LOG
  mLog = new QOpenGLDebugLogger( this );
  if (mLog->initialize()) {
    connect( mLog, SIGNAL( messageLogged( QOpenGLDebugMessage ) ),
      this, SLOT( onMessageLogged( QOpenGLDebugMessage ) ),
      Qt::DirectConnection );
    mLog->startLogging( QOpenGLDebugLogger::SynchronousLogging );
    mLog->enableMessages();
  }
#endif

  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);
  glEnable(GL_CULL_FACE);
  //glCullFace(GL_FRONT);

  mVbo.create();
  mIbo.create();
  construct();

  QImage src,bg;
  if (src.load(":/Resources/BG.png"))
    bg = src.convertToFormat(QImage::Format_RGB888);
  else{
    bg = QImage(16, 16, QImage::Format_RGB888);
    bg.fill(Qt::blue);
  }
  const uchar* p = bg.constBits();
#ifdef VR_RAW_TEX
  //glActiveTexture(GL_TEXTURE0);
  glGenTextures(1, &mTexId);  // Background
  glGenTextures(1, &mFrameId);  // Overlay
  //glEnable(GL_TEXTURE_CUBE_MAP); fragment shader handles it
  glGetIntegerv(GL_MAX_TEXTURE_SIZE, &mMaxTexSize);
  mMaxCubeSize = sqrt((mMaxTexSize*mMaxTexSize) / 6.);
  glBindTexture(GL_TEXTURE_CUBE_MAP, mTexId);
  if (mMaxCubeSize >= qMax(bg.height(), bg.width())){
    for (int i = 0; i < 6; i++){
      glTexImage2D(
        GL_TEXTURE_CUBE_MAP_POSITIVE_X + i,
        0, GL_RGB8,
        bg.width(),
        bg.height(),
        0,
        GL_RGB,
        GL_UNSIGNED_BYTE,
        (const void*)p);
    }
  }else
    qDebug("VrViewer::initializeGL Max texture resolution exceed");
  
  glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

  //glEnable(GL_TEXTURE_2D); not necessary with shaders
  glBindTexture(GL_TEXTURE_2D, mFrameId);
  if (mMaxTexSize >= qMax(bg.height(), bg.width() ) ){
    glTexImage2D(
      GL_TEXTURE_2D,
      0,
      GL_RGB8,
      bg.width(),
      bg.height(),
      0,
      GL_RGB,
      GL_UNSIGNED_BYTE,
      (const void*)p );
  }
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_NEAREST);
  
  // unbind
  glBindTexture(GL_TEXTURE_2D, 0);
  //glDisable(GL_TEXTURE_2D);
#else
  // cubemap texture settings
  mpTex = new QOpenGLTexture(QOpenGLTexture::TargetCubeMap);
  mpTex->setFormat(QOpenGLTexture::RGB8_UNorm);  // server-side (GPU)
  mpTex->setSize(bg.width(), bg.height());       // w:h must be 1 for cube
  mpTex->setAutoMipMapGenerationEnabled(false);
  
  // format+size pre-set! storage cant be resized after
  mpTex->allocateStorage(QOpenGLTexture::RGB, QOpenGLTexture::UInt8);  //client-side
  mTexId = -1;
  for (short i = 0; i < 6; i++)
    mpTex->setData(0, 0, (QOpenGLTexture::CubeMapFace)(QOpenGLTexture::CubeMapPositiveX + i), QOpenGLTexture::RGB, QOpenGLTexture::UInt8, p);

  mpTex->setMagnificationFilter(QOpenGLTexture::Linear);
  
  /* already default settings
  mpTex->setMinificationFilter(QOpenGLTexture::Nearest);
  mpTex->setWrapMode(QOpenGLTexture::DirectionS, QOpenGLTexture::ClampToEdge);
  mpTex->setWrapMode(QOpenGLTexture::DirectionT, QOpenGLTexture::ClampToEdge);
  mpTex->setWrapMode(QOpenGLTexture::DirectionR, QOpenGLTexture::ClampToEdge);*/
#endif //VR_TEX_RAW  

  // vertex shader
  QOpenGLShader *vshader = new QOpenGLShader(QOpenGLShader::Vertex, this);
  vshader->compileSourceFile(":/Resources/cubemap.vs");

  // texture shader
  QOpenGLShader *fshader = new QOpenGLShader(QOpenGLShader::Fragment, this);
  fshader->compileSourceFile(":/Resources/cubemap.frag");
  
  mpCubeShade = new QOpenGLShaderProgram;
  mpCubeShade->addShader(vshader);
  mpCubeShade->addShader(fshader);
  // index of 1. attribute
  mpCubeShade->bindAttributeLocation("position", 0);
  mpCubeShade->link();  // important: attributes bound before link
  mpCubeShade->bind();
  mpCubeShade->setUniformValue("cubemap", (GLuint)0);  //std texture index relates to 0
  mpCubeShade->release();

  // overlay
  vshader->compileSourceFile(":/Resources/frame.vs");
  fshader->compileSourceFile(":/Resources/frame.frag");

  mpFrameShade = new QOpenGLShaderProgram;
  mpFrameShade->addShader(vshader);
  mpFrameShade->addShader(fshader);
  mpFrameShade->bindAttributeLocation("position", 0);
  mpFrameShade->bindAttributeLocation("texCoord", 1);
  mpFrameShade->link();
  mpFrameShade->bind();
  mpFrameShade->setUniformValue("texture", (GLuint)0);
  mpFrameShade->release();

  mFrameserv = new QTimer(this);
  connect(this->mFrameserv, SIGNAL(timeout()), this, SLOT(checkTex()));
  mFrameserv->start(40);  //25Hz
  
  mChangedFace = 63;
  delete fshader;
  delete vshader;
}


void VrViewer::construct()
{
  const float scale = 1.f;
  VrVertex edgePts[] = {
    /*
    x:right, y:up, z:towards viewer (righthand system)
    front parts are by counter clockwise draw, back parts by clockwise (std cull).
    Skybox is made visible from the inside. use according cull side or winding.
    texture are expected as painted on the outside faces
    Vertex struct is: x,y,z, w,h, s.
    Index buffer is used in conjunction with vertex buffer to reuse vertex data
    Triangle_strip can be used to reduce effective vertexdraw (to14)
    cube from strip is build by wrapping onto a volume
    cubegeometry is used to retrieve texture coordinates (w,h, is ommitted)
    though cubesampler mirrows the texture (maps it on the outside face)
    */

    // front
    VrVertex(-1.f, -1.f, +1.f,  1.f, 0.f,  scale),  //0
    VrVertex(+1.f, -1.f, +1.f,  0.f, 0.f,  scale),  //1
    VrVertex(+1.f, +1.f, +1.f,  0.f, 1.f,  scale),  //2
    VrVertex(-1.f, +1.f, +1.f,  1.f, 1.f,  scale),  //3
    // back
    VrVertex(-1.f, -1.f, -1.f,  0.f, 0.f,  scale),  //4
    VrVertex(+1.f, -1.f, -1.f,  1.f, 0.f,  scale),  //5
    VrVertex(+1.f, +1.f, -1.f,  1.f, 1.f,  scale),  //6
    VrVertex(-1.f, +1.f, -1.f,  0.f, 1.f,  scale),  //7
  }; //size 8
  
  // index to triangles' pts, that build a cube (triangle_strip) 
  GLushort tri_ele[] = {
    3,2,0,1,5,2,6,7,5,4,0,7,3,2
    /*1,0,3, 3,2,1,  //+Z
    0,4,7, 7,3,0,  //-X
    4,5,6, 6,7,4,  //-Z
    5,1,2, 2,6,5,  //+X
    5,4,0, 0,1,5,  //-Y
    2,3,7, 7,6,2  //+Y
  }; mIdxCnt = 36;*/
  };//mIdxCnt = 14; Is now a precompiler define, dont forget to set

  // vertex data
  
  // std setting. must be set before allocate
  //mVbo.setUsagePattern(QOpenGLBuffer::StaticDraw);
  mVbo.bind();  //must be called before allocate (binds to context)
  
  // init data; cout of bytes
  mVbo.allocate(edgePts, 8 * sizeof(VrVertex));

  // index data
  mIbo.bind();
  mIbo.allocate(tri_ele, VR_IDX_CNT * sizeof(GLushort));
}


void VrViewer::paintGL()
{//not connected to a timer. must be triggered manually by update()
  // do not render to main rendertarget while Widget is composing
  if (mWait) return;

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // avoid increment (repeated )projection!
  QMatrix4x4 mvp = mP*mV;
  
  // start skybox (background)
  glDepthMask(GL_FALSE);
  mVbo.bind();
  mIbo.bind();
  mpCubeShade->bind();
  mpCubeShade->enableAttributeArray(0);
  // loads pts from buffer (vertices) [index,datatype,offset,count,attribute lenght]
  mpCubeShade->setAttributeBuffer(0, GL_FLOAT, 0, 3, sizeof(VrVertex));
  // sets uniform slot in shader to mvp
  mpCubeShade->setUniformValue("transform", mvp);
  
#ifdef VR_RAW_TEX
  //glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_CUBE_MAP, mTexId);
#else
  mpTex->bind();
#endif

  glDrawElements(GL_TRIANGLE_STRIP, VR_IDX_CNT, GL_UNSIGNED_SHORT, 0);
  //glDrawElements(GL_TRIANGLES, mIdxCnt, GL_UNSIGNED_SHORT, 0);

  mIbo.release();
  mVbo.release();
  mpCubeShade->release();
  glDepthMask(GL_TRUE);
  // end of skybox

#ifdef VR_RAW_TEX
  glBindTexture(GL_TEXTURE_CUBE_MAP, 0);
  
  // start frame 
  if (mDrawFrame){
    mvp = mP*(mV*mR);
    mpFrameShade->bind();
    mpFrameShade->enableAttributeArray(0);
    mpFrameShade->enableAttributeArray(1);
    mpFrameShade->setAttributeArray(0, mQuad, 3);
    mpFrameShade->setAttributeArray(1, VR_2D_CORNERS, 2);
    mpFrameShade->setUniformValue("transform", mvp);

    glBindTexture(GL_TEXTURE_2D, mFrameId);
    glDrawArrays(GL_QUADS, 0, 4);

    mpFrameShade->release();
    glBindTexture(GL_TEXTURE_2D, 0);
    //end of frame
  }
#else
  mpTex->release();
  //[TODO] Frame as QOpenGLTexture...
#endif
}


void VrViewer::resizeGL(int width, int height)
{
  glViewport(0,0, width, height);
  mP.setToIdentity();
  mP.perspective(mFov, (float)width / (float)height, mNear, mFar);
}


// Rotation of scene camera to observer viewing angle
void VrViewer::setView(float jaw, float pitch, float roll)
{
  // range angle
  jaw = jaw / 360.f;
  pitch = pitch / 360.f;
  roll = roll / 360.f;
  jaw = (jaw - (int)jaw)*360.f;
  pitch = (pitch - (int)pitch)*360.f;
  roll = (roll - (int)roll)*360.f;

  // scale x-axis
  if (pitch > 180.f)
    pitch -= 360.f;
  else if (pitch < -180.f)
    pitch += 360.f;
  // scale y-axis
  if (jaw > 180.f)
    jaw += -360.f;
  else if (jaw < -180.f)
    jaw += 360.f;
  // scale z-axis
  if (roll > 180.f)
    roll -= 360.f;
  else if (roll < -180.f)
    roll += 360.f;

  /* clip
  if (abs( jaw ) > 180){
    qDebug( "VrViewer::setView - pan clipped" );
    jaw >= 0 ? jaw = 180 : jaw = -180;
  }
  if (abs( pitch ) > 90){
    qDebug( "VrViewer::setView - tilt clipped" );
    pitch >= 0 ? pitch = 90 : pitch = -90;
  }
  if (abs( roll ) > 180){
    qDebug( "VrViewer::setView - roll clipped" );
    roll >= 0 ? roll = 180 : roll = -180;
  }
  */
  
  // up to here: angles scaled to +/-180
  /* mod axes
  if (pitch > 90.f && pitch <= 270){
    pitch = 180.f - pitch;
    jaw -= 180.f;
    roll -= 180;
  } else if (pitch < -90.f && pitch >= -270 ){
    pitch = -180.f - pitch;
    jaw -= 180.f;
    roll -= 180;
  }*/

  // rotate world inverse to camera
  mV.setToIdentity();
  
  // eyeposition,heading,y-axis
  //mV.lookAt(QVector3D(), QVector3D(0.f,0.f,-1.f), QVector3D(0.f, 1.f, 0.f));
  
  // rotation order important. Relative to world axes
  // seem to handle angles from -360..360, but utilizes sin,cos of angle
  mV.rotate(-roll, 0.f, 0.f, 1.f);
  mV.rotate(-pitch, 1.f, 0.f, 0.f);
  mV.rotate(-jaw, 0.f, 1.f, 0.f);
  
  // queue a update
  // 63_dez updates all (0x3F)
  // or set to the faces, changed by rotate)
  //mChangedFace = 63;
}


void VrViewer::setView( const QVector<float>* m )
{
  mV = QMatrix4x4(
    m->at( 0 ), m->at( 1 ), m->at( 2 ), 0.f,
    m->at( 4 ), m->at( 5 ), m->at( 6 ), 0.f,
    m->at( 8 ), m->at( 9 ), m->at( 10 ), 0.f,
    0.f, 0.f, 0.f, 1.f
    );
}


void VrViewer::setTex(const uchar* pImg, int w, int h, int channels, unsigned char face)
{//considered in correct byte-format

  if (face >= 6 || qMax(w, h) > mMaxCubeSize){
    qDebug("VrViewer::setTex Texture size exceed, upload skipped");
    return;
  }
  
  // not initialized?
  if (!(this->isValid())){
    return;
  }

  // direct update
  QImage in,mir;
    
  // properties check
  QOpenGLTexture::PixelFormat form;
  switch (channels)
  {
    // GL_RGB
    case 3:
      in = QImage(pImg, w, h, w * 3, QImage::Format_RGB888);
      form = QOpenGLTexture::RGB;
      break;
    
    // GL_RGBA
    case 4:
      in = QImage(pImg, w, h, w * 4, QImage::Format_RGBA8888);
      form = QOpenGLTexture::RGBA;
      break;
    
    // GL_LUMINANCE
    case 1:
      in = QImage(pImg, w, h, w, QImage::Format_Grayscale8);
      form = QOpenGLTexture::Luminance;
      break;
    
    // unsupported color format
    default: return;
  }
  this->makeCurrent();
  
#ifdef VR_RAW_TEX
  //static int curr_w, curr_h;
  if (mCurrSize[0] != w || mCurrSize[1] != h){
    // if cubemapsize changed, it becomes entirely invalid (black)
    QImage bg(w, h, QImage::Format_RGBA8888);
    bg.fill(Qt::white);
    const uchar* p0=bg.constBits();
    for (short i = 0; i < 6; i++)
      uploadCubeMap(p0, w, h, form, i);
    
    //curr_w = w; curr_h = h;
  }
  
#else
  //no texture object set?
  if (!mpTex) { this->doneCurrent(); return; }
  
  // note mpTex->depth() is NOT channels()
  if (mpTex->width() != w || mpTex->height() != h){
    // mismatch
    // QTs client storage is fixed after allocate (server format, dimensions),
    // we must destroy the old. This requires valid context.
    if (mpTex->isStorageAllocated())
      mpTex->destroy(); 
    
    // setters implicit call create() and therefore need context
    
    // server-side (GPU)
    mpTex->setFormat(QOpenGLTexture::RGBA8_UNorm);
    mpTex->setSize(w, h, 1);  //depth=1, only relevant for 3Dtextures
    mpTex->setAutoMipMapGenerationEnabled(false);
    mpTex->setMagnificationFilter(QOpenGLTexture::Linear);
    /*already std settings
    mpTex->setMinificationFilter(QOpenGLTexture::Nearest);
    mpTex->setWrapMode(QOpenGLTexture::DirectionS, QOpenGLTexture::ClampToEdge);
    mpTex->setWrapMode(QOpenGLTexture::DirectionT, QOpenGLTexture::ClampToEdge);
    
    //not supported for samplercube? It should!
    mpTex->setWrapMode(QOpenGLTexture::DirectionR, QOpenGLTexture::ClampToEdge);
    */
  }
  
  if (!(mpTex->isStorageAllocated())){
    mpTex->allocateStorage(form, QOpenGLTexture::UInt8);  //client side
    QImage bg(w, h, QImage::Format_RGBA8888);
    bg.fill(Qt::white);
    const uchar* p0=bg.constBits();
    for (short i = 0; i < 6; i++)
      uploadCubeMap(p0, w, h, form, i);
  }
    
#endif  //NOT VR_RAW_TEX
  //top or bottom
  if (face == 2 || face == 3)
    mir = in.mirrored(); //implicit deepcopy!
  
  else
    mir = in.mirrored(true, false);
  
  uploadCubeMap(mir.constBits(), w, h, form, face);
  this->doneCurrent();
  
  switch (face)
  {
    case 0: mChangedFace |= 1; break;
    case 1: mChangedFace |= 2; break;
    case 2: mChangedFace |= 4; break;
    case 3: mChangedFace |= 8; break;
    case 4: mChangedFace |= 16; break;
    case 5: mChangedFace |= 32; break;
  }
}


void VrViewer::uploadCubeMap(const void* src, int w, int h, QOpenGLTexture::PixelFormat pf, short face)
{// only 8bit pixeldata! need current context when called

  // existing context and ID?
  if (!(this->isValid() ) || mTexId==0) return;
  //QImage((const uchar*)src, w, h, 3 * w, QImage::Format_RGB888).save("Output/tex" + QString::number(face) + ".png");
  
#ifdef VR_RAW_TEX
  // not clear if this must be called. Qt redirects all to a FBO, which is unsupported <GL3.0
  //glActiveTexture(GL_TEXTURE0);
  
  glBindTexture(GL_TEXTURE_CUBE_MAP, mTexId);
  // [todo] utilize glTexSubImage2D
  glTexImage2D(
    GL_TEXTURE_CUBE_MAP_POSITIVE_X + face, 0, GL_RGBA8,
    w, h, 0, pf, GL_UNSIGNED_BYTE, src );
  mCurrSize[0] = w; mCurrSize[1] = h;
  glBindTexture(GL_TEXTURE_CUBE_MAP, 0);
#else
  if (mpTex){
    // needed when padding differs from gl memory aligment
    //QOpenGLPixelTransferOptions pto;
    //pto.setAlignment(1);
    mpTex->setData(0, 0, (QOpenGLTexture::CubeMapFace)(GL_TEXTURE_CUBE_MAP_POSITIVE_X + face), pf, QOpenGLTexture::UInt8, src);
  }
#endif //VR_TEX_RAW
}


int VrViewer::loadCubeMapFile( QString path, QString name_, QString dotExt )
{
  QImage src, bg;

  for (unsigned char i = 0; i < 6; i++){
    if (src.load( path+name_+QString::number(i)+dotExt )){
      bg = src.convertToFormat( QImage::Format_RGB888 );
      setTex( bg.constBits(), bg.width(), bg.height(), 3, i );
    }else
      qDebug() << "Failed loading map " << i;
  }
  return bg.width();
}


void VrViewer::setFov(int fov)
{// full width vertical vield of view
  mFov = fov;
  mP.setToIdentity();
  mP.perspective(mFov, (float)this->width() / (float)this->height(), mNear, mFar);
  mChangedFace = 63;
}

void VrViewer::setRange(int nearf, int farf)
{
  if (nearf < 1) nearf = 1;
  if (farf < 2) farf = 2;

  mNear = (float)nearf/100.f;
  mFar = (float)farf/100.f;

  mP.setToIdentity();
  mP.perspective(mFov, (float)this->width() / (float)this->height(), mNear, mFar);

  /*float cs = mFar - mNear;
  mQuad[2]= -cs;
  mQuad[5]= -cs;
  mQuad[8]= -cs;
  mQuad[11]=-cs;*/

  mChangedFace = 63;
}


void VrViewer::checkTex()
{//triggered by frameserv
  if (mChangedFace > 0){
    /*for (short i = 0; i < 6; i++){
      if (mChangedFace & (1 << i)){
        do something
      }
    }*/
    mChangedFace = 0;
    if (this->isVisible())
      this->update();
  }
}


void VrViewer::setRate(int hz)
{
  if (mFrameserv)
    mFrameserv->setInterval(1000/hz);
}


void VrViewer::setFramesize(int w, int h, double focalLen)
{
  double base = sqrt(2.)*focalLen;  //should be same as in cubegen
  float wfrac = (double)w / base;
  float hfrac = (double)h / base;

  mQuad[0] =  wfrac; mQuad[1] =  hfrac;  //UR
  mQuad[3] = -wfrac; mQuad[4] =  hfrac;  //UL
  mQuad[6] = -wfrac; mQuad[7] = -hfrac;  //DL
  mQuad[9] =  wfrac; mQuad[10] = -hfrac;  //DR
}

// rotation of frame to camera position
void VrViewer::setRotation(float pan, float tilt, float rotation)
{
  mR.setToIdentity();
  mR.rotate(pan, 0.f, 1.f, 0.f);
  mR.rotate(tilt, 1.f, 0.f, 0.f);
  mR.rotate(rotation, 0.f, 0.f, -1.f);

  //mFrameData.clear();
  QVector3D pt;
  for (short i = 0; i < 10; i+=3){
    //pt.setX(mQuad[i]);
    //pt.setY(mQuad[i + 1]);
    //pt.setZ(mQuad[i + 2]);
    pt = QVector3D( mQuad[i], mQuad[i + 1], mQuad[i + 2] );
    // gl uses float multiplication, so we do the same
    pt = mR*pt;
    mFrameData[i] = pt.x();
    mFrameData[i + 1] = pt.y();
    mFrameData[i + 2] = pt.z();
  }

  //No need to inform about change, as long mFrameData isn't reassigned or deleted
  //however, unsynchronized access may be crittical. Better not clear vector.
  //emit verticeChanged( &mFrameData );
}


void VrViewer::setFrame(const uchar* pImg, int w, int h, int channels)
{//considered in correct byte-format
  if (qMax(w, h) > mMaxTexSize){
    qDebug("VrViewer::setFrame Texture size exceed, upload skipped");
    return;
  }
  
  // properties check
  QOpenGLTexture::PixelFormat form;
  switch (channels)
  {
    case 3: form = QOpenGLTexture::RGB; break;
    case 4: form = QOpenGLTexture::RGBA; break;
    case 1: form = QOpenGLTexture::Luminance; break;
    default: return;
  }
  
  this->makeCurrent();
  uploadTex(pImg, w, h, form, mFrameId);
  this->doneCurrent();
  mChangedFace = 63;
}


void VrViewer::setFrame(const QImage &Img, int channels)
{//8 bit per channel! direct update
  int w = Img.width();
  int h = Img.height();
  if (qMax(w, h) > mMaxTexSize){
    qDebug("VrViewer::setFrame Texture size exceed, upload skipped");
    return;
  }
  
  // properties check
  QOpenGLTexture::PixelFormat form;
  switch (channels)
  {
    case 3: form = QOpenGLTexture::RGB; break;
    case 4: form = QOpenGLTexture::RGBA; break;
    case 1: form = QOpenGLTexture::Luminance; break;
    default: return;
  }
  
  this->makeCurrent();
  uploadTex(Img.constBits(), w, h, form, mFrameId);
  this->doneCurrent();
  
  mChangedFace = 63;
}


void VrViewer::uploadTex(const void* src, int w, int h, QOpenGLTexture::PixelFormat pf,GLuint id)
{//only 8bit pixeldata! needs current context when called

  //existing context and ID?
  if (!(this->isValid()) || id == 0) return;
  glBindTexture(GL_TEXTURE_2D, id);
  glTexImage2D(
    GL_TEXTURE_2D, 0, GL_RGBA8,
    w, h, 0, pf, GL_UNSIGNED_BYTE, src);

  glBindTexture(GL_TEXTURE_2D, 0);
}


void VrViewer::onMessageLogged( QOpenGLDebugMessage message )
{
  qDebug() << message;
}


void VrViewer::onCompose()
{
  mWait = true;
}


void VrViewer::onSwapped()
{
  mWait = false;
}
/*
QImage VrViewer::getView()
{
  return this->grabFramebuffer();
}
*/