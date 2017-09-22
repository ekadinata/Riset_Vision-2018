/*
 * ColorFinder.h
 *
 *  Created on: 2011. 1. 10.
 *      Author: zerom
 */

#ifndef COLORFINDER_H_
#define COLORFINDER_H_

#include <string>
#include <vector>
#include <map>

#include "Point.h"
#include "Image.h"
#include "minIni.h"
#include "DebugDrawings.h"
#include "Camera.h"

#define COLOR_SECTION   "Find Color"
#define INVALID_VALUE   -1024.0

#define MAXDEGREE 360
#define MAXRADIUS 400

#define MAX_GOAL_SPOTS 8

namespace Robot
{
struct coordinate
{
  unsigned int x, y;
};

struct lineBlob
{
  unsigned int min, max;
  unsigned int blobId;
  bool attached;
};

struct blob
{
  coordinate min, max;
  coordinate center;
};

/** @brief Class ColorFinder
  * Di dalam kelas ColorFinder terdapat pendeteksian Field (lapangan) dan Ball (Ball)
  */
class ColorFinder
{
private:
  Point2D m_center_point;

  /** @brief Prosedur untuk melakukan filtering gambar dari kamera
    * I.S. Variabel img terdefinisi dan dapat diproses untuk filtering image
    * F.S. Hasil filtering image telah ditampung ke dalam sebuah variabel m_result
    * @param img sebuah variabel bertipe Image yang digunakan untuk menampung gambar yang diterima dari kamera
    */
  void Filtering(Image* img);

public:
  int m_hue;             /* 0 ~ 360 */
  int m_hue_tolerance;   /* 0 ~ 180 */
  int m_min_saturation;  /* 0 ~ 100 */
  int m_min_value;       /* 0 ~ 100 */
  double m_min_percent;  /* 0.0 ~ 100.0 */
  double m_max_percent;  /* 0.0 ~ 100.0 */

  std::string color_section;

  int Xmin, Xmax, Ymin, Ymax;

  Image * m_result;
  Image * edge_img;

  int * accumulator;
  // int accumulator[101376];


  /** @brief Constructor
    * Konstruktor default Color Finder
    */
  ColorFinder();

  /** @brief Constructor
    * Konstruktor Color Finder dengan parameter
    * @param hue
    * @param hue_tol
    * @param min_sat
    * @param min_val
    * @param min_per
    * @param max_per
    */
  ColorFinder(int hue, int hue_tol, int min_sat, int min_val, double min_per, double max_per);

  /** @brief Destructor
    * Destruktor Color Finder
    */
  virtual ~ColorFinder();

  /** @brief Prosedur untuk melakukan Load dan Save nilai konfigurasi kamera
    */
  void LoadINISettings(minIni* ini);
  void LoadINISettings(minIni* ini, const std::string &section);
  void SaveINISettings(minIni* ini);
  void SaveINISettings(minIni* ini, const std::string &section);

  Point2D m_center;

  /** @brief Fungsi yang digunakan untuk mencari center point dari gambar
    * @param yHorizon batas atas horizon (batas pandang) pendeteksian bola di kamera robot
    * @return m_center_point variabel point yang berada di tengah suatu gambar
    */
  Point2D& GetPositionMoment(int yHorizon);

  /** @brief Fungsi untuk menentukan batas ujung lapangan sepakbola
    * Fungsi digunakan agar robot tidak melihat bola yang berada di luar batas lapangan
    * @param img variabel Image dari gambar yang sedang ditangkap kamera
    * @param segmentChildThreshold
    * @param segmentLostThreshold
    * @param yHorizon batas atas horizon (batas pandang) di kamera robot
    * @return fieldBorderPoints sebuah array yang menampung titik (point) dari batas lapangan
    */
  std::vector<Point2D> getConvexFieldBorders(Image *img, int segmentChildThreshold, int segmentLostThreshold, int yHorizon);

  /** @brief
    *
    * @param fieldBorderPoints
    * @param segmentChildThreshold
    * @return
    */
  std::vector<Point2D> linePercept(std::vector<Point2D> fieldBorderPoints, int segmentChildThreshold);

  /** @brief Fungsi yang digunakan untuk menggambar rectangle(kotak) pada kandidat bola (blob) di lapangan yang tertangkap kamera
    * Fungsi getBlobLine akan menandai semua kandidat bola di lapangan dengan menggunakan gambar kotak di kamera
    * denga menggunakan metode filtering warna
    * @param img variabel Image yang digunakan untuk menyimpan data dari gambar yang ditangkap kamera
    * @param fieldBorderPoints sebuah array yang menampung nilai titik batas lapangan
    * @return listBlob(m_center_point) sebuah nilai titik pusat dari bola yang dideteksi oleh robot
    */
  std::vector<Point2D> getBlobLine(Image *img, std::vector<Point2D> fieldBorderPoints);

  /** @brief Fungsi yang digunakan untuk mencari nilai titik (point) dari bola di lapangan
    * Fungsi getBlobCenter melakukan penyeleksian luas blob dari blob yang terdeteksi di dalam gambar
    * dan mengambil nilai luas blob maksimum untuk nilai titik (point) bola yang dideteksi oleh robot
    * @param img variabel Image yang digunakan untuk menyimpan data dari gambar yang ditangkap kamera
    * @param fieldBorderPoints sebuah array yang menampung nilai titik batas lapangan
    * @return listBlob(m_center_point) sebuah nilai titik pusat dari bola yang dideteksi oleh robot
    */
  std::vector<Point2D> getBlobCenter(Image *img, std::vector<Point2D> fieldBorderPoints);

  /** @brief Fungsi yang digunakan untuk mendeteksi musuh atau rintangan
    *
    * @param img variabel gambar yang digunakan untuk menyimpan data dari citra yang dideteksi oleh kamera
    * @param fieldBorderPoints sebuah array yang menampung nilai titik batas lapangan
    * @return listBlob sebuah array berupa titik pusat koordinat dari rintangan atau musuh
    */
  std::vector<Point2D> getObstacle(Image *img, std::vector<Point2D> fieldBorderPoints);

  /** @brief Prosedur tidak diimplementasikan di ColorFinder.cpp
    *
    * @param xStart
    * @param yStart
    * @param yEnd
    */
  void scanDown(int xStart, int yStart, int& yEnd);

  /** @brief Prosedur tidak diimplementasikan di ColorFinder.cpp
    *
    * @param xStart
    * @param yStart
    * @param yEnd
    */
  void findDown(int xStart, int yStart, int& yEnd);

  void Reset(Image * img);

  void HoughTransf(Image * img);

  void EdgeDetect(Image * img);

  void FillHoughSpace(Image * img);

  void Process(Image * img);

  void HoughCircle(Image * img, int r_min, int r_max);

  void Detect(Image * img, int r_min, int r_max);

  void Accum_circle(int * accum, int i, int j, int rad);

  void Accum_pixel(int * accum, int x, int y);

  void drawCircle(Image * img, int i, int j, int rad);

  void drawPixel(Image * img, int x, int y);

  ColorClasses::Color imgColor(Image* img, int x, int y);

  /**
    *
    */
  inline ColorClasses::Color imageColor(Image* img, int x, int y);

  /** @brief Prosedur yang sama dengan FilteringImageErotionDilation
    * @param img variabel Image yang digunakan untuk menyimpan data dari gambar yang ditangkap kamera
    */
  void FilteringImage(Image *img);

  /** @brief Prosedur yang digunakan untuk transformasi gambar dengan metode erosi dan dilasi
    * @param img variabel Image yang digunakan untuk menyimpan data dari gambar yang ditangkap kamera
    */
  void FilteringImageErotionDilation(Image *img);

  /** @brief Prosedur untuk melakukan transformasi gambar dengan erosi dan dilasi
    *
    */
  void Erode_Dilate();
};

/** @brief Class GoalPercept
  * hanya berisi variabel untuk pendeteksian gawang
  */
class GoalPercept
{
private :
  static GoalPercept* m_UniqueInstance;

  GoalPercept();
public:
  enum
  {
    NONE,
    UNKNOWN_POST,
    RIGHT_POST,
    LEFT_POST,
    POSSIBLE_RIGHT_POST,
    POSSIBLE_LEFT_POST,
    BOTH_POST
  };

  enum
  {
    UNCLEAR,
    OWN_GOAL,
    OPPONENT_GOAL
  };

  static GoalPercept* GetInstance()
  {
    return m_UniqueInstance;
  }

  Point2D RightPart, LeftPart, Center, CenterFoot, CenterHead;
  Point2D RightFoot, LeftFoot; //edited by imre
  Point2D CenterUpperPost, RightUpperPost, LeftUpperPost;
  int Owner;
  std::vector<Point2D> foot, head;
  int Status;

  void CheckPostStatus();
};

/** @brief Class GoalPercept
  * berisi fungsi atau metode yang digunakan untuk pendeteksian gawang
  */
class GoalPerceptor
{
public:
  /** @brief Constructor
    * Constructor default GoalPerceptor
    */
  GoalPerceptor();

  //void Process(Image* img, int yHorizon, int GoalColour);

  /** @brief Prosedur Process melakukan pemrosesan gambar yang didapat untuk mendeteksi gawang
    * I.S. variabel img, yHorizon, OppGoal terdefinisi, gawang pada citra gambar dapat terlihat oleh robot atau tidak
    * F.S. Posisi gawang ditemukan dan robot bergerak (menendang bola) menuju ke arah gawang
    * @param img variabel gambar yang digunakan untuk menyimpan data dari citra yang dideteksi oleh kamera
    * @param yHorizon batas atas horizon (batas pandang) pendeteksian bola di kamera robot
    * @param OppGoal variabel boolean untuk menandakan apakah sedang melihat gawang musuh atau bukan
    */
  void Process(Image* img, int yHorizon, bool OppGoal);

  //Point2D findUpperPost(Image* img, int xCenter, int yHorizon, int GoalColour);

  /** @brief Fungsi yang digunakan untuk melakukan pendeteksian tiang atas gawang
    * Fungsi akan mengembalikan koordinat titik tengah dari tiang atas gawang
    * @param img variabel gambar yang digunakan untuk menyimpan data dari citra yang dideteksi oleh kamera
    * @param xCenter
    * @param yHorizon batas atas horizon (batas pandang) pendeteksian bola di kamera robot
    * @return CenterUpperPost Koordinat titik tengah dari tiang atas gawang
    */
  Point2D findUpperPost(Image* img, int xCenter, int yHorizon);

private:
  enum GoalColour
  {
    BLUE_POST = 0,
    YELLOW_POST
  };

  class Parameters
  {
  public:
    Parameters() :
      maxSkip(1),
      minSegmentLength(5),
      minYRunDiff(5),
      minPostPixelHeight(40),
      minHeadVisibleOffset(5),
      //widthStepSize(10),
      widthStepSize(2),	//EDITED
      maxGreenScan(30),//10
      minGreenBelow(10),//5
      maxWidthErrorRatio(1.5f),
      minWidthErrorRatio(0.4f),
      maxWrongSizeRatio(0.25f),
      maxFootDistance(721.0f)
    {}

    int minSegmentLength,
    widthStepSize,
    maxSkip,
    minYRunDiff,
    minPostPixelHeight,
    minHeadVisibleOffset,
    maxGreenScan,
    minGreenBelow;
    float maxWidthErrorRatio,
    minWidthErrorRatio,
    maxWrongSizeRatio,
    maxFootDistance;

  };

  class Spot
  {
  public:
    ColorClasses::Color color;
    Point2D head, foot;
    Point2D center;
    int avrgWidth;
    bool headVisible,
    footVisible,
    dead;
    Point2D onField;
    int footWidth;
    std::vector<Point2D> errorLeft, errorRight;
    std::vector<int> errorWidth;
  };

  Image* imgProcess;
  Parameters parameters;
  std::vector<Spot> spots;

  void findSpots(Image* img, int yHorizon);
  void checkSpotsWidth(Image* img);
  void checkGreenBelow(Image* img);
  void checkMaximumDistance();
  void resetPercept();
  //void createPercept(Image* img, int TargetColor);
  void createPercept(Image* img, bool OppGoal);

  int findBlueOrYellowRight(Image* img, int x, int y, int xEnd);
  int findGreenDown(int x, int y, int yEnd);
  int runRight(Image* img, int x, int y, ColorClasses::Color col, int xEnd, int maxSkip);
  int runLeft(Image* img, int x, int y, ColorClasses::Color col, int xEnd, int maxSkip);
  int runDown(Image* img, int x, int y, ColorClasses::Color col, int yEnd, int maxSkip);
  int runUp(Image* img, int x, int y, ColorClasses::Color col, int yEnd, int maxSkip);
  inline ColorClasses::Color imageColor(Image* img, int x, int y);
};
}

#endif /* COLORFINDER_H_ */
