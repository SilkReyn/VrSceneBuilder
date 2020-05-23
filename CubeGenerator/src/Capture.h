/**
  @file Capture.h
  @brief Provides functionality to handle plugin input data.
  
  Implementation: Capture.cpp
  Content:
  - #Capture
  + #Capture::InputData
  + #Capture::Angles
  
  @author Andre Halim
  @author Fraunhofer IOSB VID
  @date 2016, Apr.-Sept.
 */
 
//==================================================================================================

#ifndef CAPTURE_H_
#define CAPTURE_H_

#include <opencv2/opencv.hpp>

//==================================================================================================

/// Container class for image information.
/**
  Offers a struct #InputData, to store relevant image properties in one place and keep them linked
  to the pixel data. While the struct itself is public to manipulation, once used in a class
  instance and therefore linked to particular properties, the instance acts as proxy to the data to
  avoid inadvertent change. For image pixel data, cv::Mat of OpenCV http://docs.opencv.org is
  utilized.
  
  Example usage:
      InputData data();
      income.m = cv::imread("image.png");
      Capture input(data, true);
      input.setAngles(-45.f,45.f);
  
  The class also provides methods to convert pixel data directly from/to memory, arranged in plane
  seperated structure, namely DevEnviroViewerPluginInterface::Image
  
  @see InputData, Capture(InputData,bool,bool), setAngles
 */
class Capture
{
  public:
  //___Declarations_____________________________________________________________
  /// Struct to store various image properties
  /// field   | description
  /// --------|------------------------------------------------------
  /// #m      | A OpenCV Mat, which is itself a header to image data.
  /// #angles | array to store angular data as float.
  /// #zoom   | float val to hint at which zoom the image was taken.
  /// --------------------------------------------------------------
  /// @see setAngles, ::zoom(), pan(), tilt(), setMat
  struct InputData{
    cv::Mat m;
    float angles[3];
    float zoom;
    //float foc
    //QDateTime ts;
    InputData() : zoom(1.f) { angles[0] = 360.f; angles[1] = 360.f; angles[2] = 360.f; };
  };

  //____Fields__________________________________________________________________
  //cv::Mat map1;  ///< Holds lookuptables for repaint
  //cv::Mat map2;
  long mHyp;            ///< Hypotenuse of width * height, floor'ed.
  cv::Size mResolution; ///< Holds properties width and height of \
                             the stored image.
  
  //____Initialization__________________________________________________________
  /// Creates empty instance, initializes positions with 360.f and zoom 1.f
  Capture();
  
  /// Instanciation using #InputData as argument.
  /// Use this to create a instance from already existing data.
  /// @param src Struct holding the image and properties.
  /// @param isBGR bool needed for conversion purpose. TRUE if the images colors
  ///        are in blue;green;red order, which is standard for cv::Mat. This is
  ///        unrelevant for most operations on images.
  /// @param preserve Set this bool TRUE if src has limited lifetime or must not
  ///        be changed by this instance. Will force a deep-copy in this case.
  ///        If this argument is not provided, it will perform as if TRUE. 
  /// @see InputData, isBGR
  Capture(const InputData &src, bool isBGR, bool preserve = true);
  
  /// Instanciation with only basic image properties.
  /// Use this to create a Instance and prepare it for incomming data, for example
  /// when importing from memory:
  ///     Capture input(false);
  ///     input.readPlanar(pData,width, height, planes, true);
  ///
  /// @param isBGR bool needed for conversion purpose. TRUE if the images colors
  ///        are in blue;green;red order, which is standard for cv::Mat. This is
  ///        unrelevant for most operations on images.
  /// @param zoom #InputData::zoom
  /// @see Capture(), InputData, readPlanar
  Capture(bool isBGR, float zoom = 1.f);
  
  //____Output__________________________________________________________________
  char channels() const;  ///< Returns numerical count of color channels of \
                               stored image, usually 3 for RGB.
  float pan() const;      ///< Returns #InputData::angles 1st element
  float tilt() const;     ///< Returns #InputData::angles 2st element
  float zoom() const;     ///< Returns #InputData::zoom
  
  /// Returns a const pointer to first 8Bit pixel value of stored image.
  const uchar* dataStart() const; 
  
  /// Returns a detached copy of this instance.
  Capture clone() const;
  
  /// Copies stored image data to external memory.
  /// Internally the image consists of a line of B-G-R-B-G-R-... values. The
  /// produced output starting at \a data will have R-R-...-G-G-...-B-B-... format.
  /// The method assumes 8Bit values and \a data ready to store
  /// width*height*channels number of Bytes.
  /// @param Pointer to pre-allocated random-access-memory.
  void writePlanar(unsigned char* data) const;

  //____Methods_________________________________________________________________
  
  /// Returns the header of the stored image.
  const cv::Mat& getMat();  
  
  /// Refresh the stored image header with another. 
  /// No copy of the underlying data is done. The old header is lost for this
  /// instance and referenced data deallocated if that header was unique.
  /// Though this is not the preferred way of updating. Consider generation of
  /// a new Capture instance instead, because remaining properties might happen
  /// to be unrelated to the new image.
  /// @param src Source image header referencing new pixel data.
  /// @see setMat(const cv::Mat)
  void setMat(cv::Mat &&src);
  
  /// Overload method of #setMat(cv::Mat)
  /// Refreshes the stored image header with another, but performs a deep-copy
  /// of \a src to store a clone of it.
  /// @param src const source image header referencing new pixel data.
  /// @see setMat(cv::Mat)
  void setMat(const cv::Mat &src);
  
  /// Transfers external image data to own image storage and format.
  /// This method is used to import images saved in separate color planes format.
  /// #mIsBGR alters the way how incomming planes are ordered by the created
  /// image header. \a cpy controls if the content referenced by \a pData should
  /// be copied or only re-referenced. If `cpy==FALSE` the memory pointed by
  /// \a pData with size of w*h*pl must stay valid for whole lifetime of this
  /// instance. Input data is expected to be in 8Bit depth and continous in
  /// memory (no padding).
  /// @param pData pointer referencing random-access-memory to pixel data
  /// @param w Width of image referenced by \a pData
  /// @param h Height of image referenced by \a pData
  /// @param pl Count of planes, referenced by \a pData
  /// @param cpy bool to control whether data should be copied at the transfer
  /// @see writePlanar
  bool readPlanar(unsigned char* pData, int w, int h, int pl, bool cpy = true);  //datapointer, width, height, planes, allow copy
  
  /// Alters the angles of what directions the camera faced at cature time.
  /// Right-hand polar coordinate system is used to express directions in
  /// components of pan, tilt and rotation. Positive angles are counter-clockwise.
  /// @param pan y-axis rotation
  /// @param tilt x-axis rotation
  /// @param rot z-axis rotation. 0.f if omitted.
  /// @see InputData::angles
  void setAngles(float pan, float tilt, float rot = 0.f);

  bool clearMat();

  private:
  enum Angles{ PAN=0, TIL, ROT };  ///< Internally used for indexing and accessing \
                                        the angles array of #InputData

  //____Private fields__________________________________________________________
  InputData mIn;  ///< Holds the struct containing input data.
  bool mIsBGR;    ///< Hints whether pixel data is stored \
                       using blue,green,red or RGB ordering.

  //____Private methods_________________________________________________________
  /// Method to directly access angles
  /// Only a slight difference to the getters #pan, #tilt, #rot, by
  /// utilizing a single method. Private because of the limited values the method
  /// accepts.
  /// @param idx class defined index of requested angle
  /// @see Angles
  const float& angle(Angles idx) const;
};

//==================================================================================================

#endif /*Capture_H_*/