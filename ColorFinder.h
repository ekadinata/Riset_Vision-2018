/*
 * ColorFinder.h
 *
 *  Created on: 2011. 1. 10.
 *      Author: zerom
 */

#ifndef COLORFINDER_H_
#define COLORFINDER_H_

#include <string>
#include <math.h>
#include <vector>
#include <map>

#include "Camera.h"
#include "Point.h"
#include "MathFunction.h"
#include "Image.h"
#include "minIni.h"
#include "DebugDrawings.h"

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

class ColorFinder
{
private:
  Point2D m_center_point;
  
  
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

  int accumulator[352][288];

  Image* edge_img;
  Image*  m_result;

  ColorFinder();
  ColorFinder(int hue, int hue_tol, int min_sat, int min_val, double min_per, double max_per);
  virtual ~ColorFinder();

  void LoadINISettings(minIni* ini);
  void LoadINISettings(minIni* ini, const std::string &section);
  void SaveINISettings(minIni* ini);
  void SaveINISettings(minIni* ini, const std::string &section);

  Point2D m_center;

  Point2D& GetPositionMoment(int yHorizon);
  std::vector<Point2D> getConvexFieldBorders(Image *img, int segmentChildThreshold, int segmentLostThreshold, int yHorizon);
  std::vector<Point2D> linePercept(std::vector<Point2D> fieldBorderPoints, int segmentChildThreshold);
  std::vector<Point2D> getBlobLine(Image *img, std::vector<Point2D> fieldBorderPoints);
  std::vector<Point2D> getBlobCenter(Image *img, std::vector<Point2D> fieldBorderPoints);
  std::vector<Point2D> getObstacle(Image *img, std::vector<Point2D> fieldBorderPoints);

  void scanDown(int xStart, int yStart, int& yEnd);
  void findDown(int xStart, int yStart, int& yEnd);
  inline ColorClasses::Color imageColor(Image* img, int x, int y);

  void FilteringImage(Image *img);
  void FilteringImageErotionDilation(Image *img);
  void Erode_Dilate();
  void EdgeDetection(Image *img);
  void HoughCircle(Image* img, int r_min, int r_max);
  void FillHoughSpace(Image* img);
  int GetAccumulator(int i, int j);
  Point2D Voting(Image* img);
  Point2D getPos(Image* img);
  void Reset();

};

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
    UNCLEAR, //belum jelas gawang siapa
    OWN_GOAL, //gawang sendiri
    OPPONENT_GOAL //gawang lawan
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

class GoalPerceptor
{
public:
  GoalPerceptor();

  //void Process(Image* img, int yHorizon, int GoalColour);
  void Process(Image* img, int yHorizon, bool OppGoal);

  //Point2D findUpperPost(Image* img, int xCenter, int yHorizon, int GoalColour);
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
