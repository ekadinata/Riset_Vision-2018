/*
 * ColorFinder.cpp
 *
 *  Created on: 2011. 1. 10.
 *      Author: zerom
 */

#include <assert.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "ColorFinder.h"
#include "ImgProcess.h"
#include "DebugDrawings.h"
#include "Camera.h"
#include "MotionStatus.h"
#include "Head.h"

using namespace std;
using namespace Robot;

ColorFinder::ColorFinder() :
  m_center_point(Point2D()),
  m_hue(356),
  m_hue_tolerance(15),
  m_min_saturation(50),
  m_min_value(10),
  m_min_percent(0.07),
  m_max_percent(5.0),
  color_section(""),
  m_result(0)
{

  Xmin = 1000;
  Xmax = -1.0;
  Ymin = 1000;
  Ymax = -1.0;
  m_center.X = -1.0;
  m_center.Y = -1.0;
}

ColorFinder::ColorFinder(int hue, int hue_tol, int min_sat, int min_val, double min_per, double max_per) :
  m_hue(hue),
  m_hue_tolerance(hue_tol),
  m_min_saturation(min_sat),
  m_min_value(min_val),
  m_min_percent(min_per),
  m_max_percent(max_per),
  color_section(""),
  m_result(0)
{
  Xmin = 1000;
  Xmax = -1.0;
  Ymin = 1000;
  Ymax = -1.0;
  m_center.X = -1.0;
  m_center.Y = -1.0;
}

ColorFinder::~ColorFinder()
{
  // TODO Auto-generated destructor stub
}

void ColorFinder::Filtering(Image *hsv_img)
{
  unsigned int h, s, v;
  int h_max, h_min;
    delete m_result;
//		printf("m_result
//  if(m_result == NULL)
    m_result = new Image(hsv_img->m_Width, hsv_img->m_Height, 1);

  h_max = m_hue + m_hue_tolerance;
  h_min = m_hue - m_hue_tolerance;
  if(h_max > 360)
    h_max -= 360;
  if(h_min < 0)
    h_min += 360;

  for(int i = 0; i < hsv_img->m_NumberOfPixels; i++)//luqman
    {
      h = (hsv_img->m_ImageData[i*hsv_img->m_PixelSize + 0] << 8) | hsv_img->m_ImageData[i*hsv_img->m_PixelSize + 1];
      s =  hsv_img->m_ImageData[i*hsv_img->m_PixelSize + 2];
      v =  hsv_img->m_ImageData[i*hsv_img->m_PixelSize + 3];

      if( h > 360 )
        h = h % 360;

      if( ((int)s > m_min_saturation) && ((int)v > m_min_value) )
        {
          if(h_min <= h_max)
            {
              if((h_min < (int)h) && ((int)h < h_max))
                m_result->m_ImageData[i]= 1;
              else
                m_result->m_ImageData[i]= 0;
            }
          else
            {
              if((h_min < (int)h) || ((int)h < h_max))
                m_result->m_ImageData[i]= 1;
              else
                m_result->m_ImageData[i]= 0;
            }
        }
      else
        {
          m_result->m_ImageData[i]= 0;
        }
    }
}

void ColorFinder::FilteringImage(Image *hsv_img)
{
  Filtering(hsv_img);

//  ImgProcess::Dilation(m_result);
//  ImgProcess::Erosion(m_result);
}
void ColorFinder::FilteringImageErotionDilation(Image *hsv_img)
{
  Filtering(hsv_img);

  ImgProcess::Erosion(m_result);
  ImgProcess::Dilation(m_result);
}

void ColorFinder::Erode_Dilate()
{
  ImgProcess::Dilation(m_result);
//  ImgProcess::Erosion(m_result);
}

Point2D& ColorFinder::GetPositionMoment(int yHorizon)
{
  int sum_x = 0, sum_y = 0, count = 0;

  for(int y = 0; y < m_result->m_Height; y++)
    {
      for(int x = 0; x < m_result->m_Width; x++)
        {
          if(m_result->m_ImageData[m_result->m_Width * y + x] > 0)
            {
              sum_x += x;
              sum_y += y;
              count++;
            }
        }
    }

  if(count <= (m_result->m_NumberOfPixels * m_min_percent / 100) || count > (m_result->m_NumberOfPixels * m_max_percent / 100))
    {
      m_center_point.X = -1.0;
      m_center_point.Y = -1.0;
    }
  else
    {
      m_center_point.X = (int)((double)sum_x / (double)count);
      m_center_point.Y = (int)((double)sum_y / (double)count);
    }

  return m_center_point;
}

#define LEFT_OF(x0, x1, x2) ((x1.X-x0.X)*(-x2.Y+x0.Y)-(x2.X-x0.X)*(-x1.Y+x0.Y) > 0)
#define FIND_Y(x1, y1, x2, y2, x) ((y2-y1)*(x-x1)/((x2-x1) == 0? 1 : (x2 - x1))+y1)

vector<Point2D> ColorFinder::getConvexFieldBorders(Image *img, int segmentChildThreshold, int segmentLostThreshold, int yHorizon)
{
  vector<Point2D> points;
	  Xmin = 1000;
	  Xmax = -1.0;
	  Ymin = 1000;
	  Ymax = -1.0;

  for(int column = 0; column < m_result->m_Width; ++column)//luqman
    {
      for(int row = 0; row < m_result->m_Height; ++row)//luqman
        {
          unsigned char byte = (unsigned char) m_result->m_ImageData[(row*m_result->m_Width) + column];

          if(byte == 1 && row > yHorizon)
            {
              int start = row;
              for(; byte == 1 && row <= m_result->m_Height; byte = (unsigned char) m_result->m_ImageData[(row*m_result->m_Width)+ column], ++row);//luqman
              int stop = row-1;

              if((stop - start) > segmentChildThreshold)//start ambil ijo pertama, trus jalan ke kiri, kalo nemuin ijo lagi, maka stop
		//kalo jarak satu ijo ke lainnya lebih dari segment, maka tidak dianggap sebagai borders...
                {
                  m_center_point.X = column;
                  m_center_point.Y = start;
                  points.push_back(m_center_point);
//		  printf("PINGGIR LAPANGAN GUYS.........\n");
                  break;
                }
            }
        }
    }

  //Andrew's Monotone Chain Algorithm to compute the upper hull
  typedef std::vector<Point2D >::const_iterator CI;
  std::vector<Point2D > hull;



  if(points.size() != 0)
    {
      const CI pmin = points.begin(), pmax = points.end()-1;
      hull.push_back(*pmin);
      for(CI pi = pmin + 1; pi != pmax+1; pi++)//luqman
        {
          if(!LEFT_OF((*pmin), (*pmax), (*pi)) && pi != pmax)
            continue;

          while((int)hull.size() > 1)
            {
              const CI p1 = hull.end() - 1, p2 = hull.end() - 2;
              if(LEFT_OF((*p1), (*p2), (*pi)))
                break;
              hull.pop_back();
            }
          hull.push_back(*pi);
        }

      vector<Point2D> fieldBorderPoints;
      Point2D temp;


      for(std::vector<Point2D >::iterator iter = hull.begin(); iter != hull.end(); iter++)
        {
          int X1 = int((*iter).X);
          int Y1 = int((*iter).Y);
          int X2 = int((*(iter+1)).X);
          int Y2 = int((*(iter+1)).Y);

          for(int x = X1; x < X2; x++)
            {
              temp.X = x;
              temp.Y = FIND_Y(X1, Y1, X2, Y2, x);

		if(Xmin > temp.X)
			Xmin = temp.X;
		if(Xmax < temp.X)
			Xmax = temp.X;
		if(Ymin > temp.Y)
			Ymin = temp.Y;
		if(Ymax < temp.Y)
			Ymax = temp.Y;
              fieldBorderPoints.push_back(temp);
            }
        }

      	for(std::vector<Point2D>::iterator iter = fieldBorderPoints.begin(); iter!=fieldBorderPoints.end(); iter++)
              {
              	Draw::Circle(img, Point2D((*iter).X,(*iter).Y), 1, ColorRGB(255,0,0));
              }
      
      return fieldBorderPoints;
    }

  return points;
}

vector<Point2D> ColorFinder::getBlobLine(Image *img, vector<Point2D> fieldBorderPoints)
{
  m_center.X = -1.0;
  m_center.Y = -1.0;

  int segmentCounter = 0;
  vector< vector<lineBlob> > lineSegment(m_result->m_Width);
  //Check existing lineBlob in every row
  for(int column = 0; column < (int)fieldBorderPoints.size(); ++column)
    {
      for(int row = fieldBorderPoints[column].Y; row < m_result->m_Height; ++row)
        {
          unsigned char byte = (unsigned char) m_result->m_ImageData[(row*m_result->m_Width)+ int(fieldBorderPoints[column].X)];

          if(byte == 1)
            {
              int start = row;
              for(; byte == 1 && row < m_result->m_Height; byte = (unsigned char) m_result->m_ImageData[(row*m_result->m_Width)+ int(fieldBorderPoints[column].X)], ++row);

              int stop = row-1;
              lineBlob lineBlobData = {start, stop, segmentCounter, false};

              lineSegment[fieldBorderPoints[column].X].push_back(lineBlobData);
              segmentCounter++;
            }
        }
    }
  vector<Point2D> listBlob;
  /* Check if any lineBlob exist */
  if (segmentCounter == 0)
    {
      m_center_point.X = -1.0;
      m_center_point.Y = -1.0;
      listBlob.push_back(m_center_point);
      return listBlob;
    }

  /* Check lineBlobs for a touching lineblob on the next column ; Assigning blobId and attached */
  for(int column = 0; column < (int)lineSegment.size()-1; ++column)
    {
      for(int entryLine1 = 0; entryLine1 < (int)lineSegment[column].size(); ++entryLine1)
        {
          for(int entryLine2 = 0; entryLine2 < (int)lineSegment[column+1].size(); ++entryLine2)
            {
              if(((lineSegment[column][entryLine1].max > lineSegment[column+1][entryLine2].min) || abs((int)lineSegment[column][entryLine1].max - (int)lineSegment[column+1][entryLine2].min) < 3)
                  && ((lineSegment[column][entryLine1].min < lineSegment[column+1][entryLine2].max) || abs((int)lineSegment[column][entryLine1].min -(int)lineSegment[column+1][entryLine2].max) < 3))
                {
                  if(lineSegment[column+1][entryLine2].attached == false)
                    {
                      lineSegment[column+1][entryLine2].blobId = lineSegment[column][entryLine1].blobId;
                      lineSegment[column+1][entryLine2].attached = true;
                    }
                  else
                    {
                      lineSegment[column][entryLine1].blobId = lineSegment[column+1][entryLine2].blobId;
                      lineSegment[column][entryLine1].attached = true;
                    }
                }
            }
        }
    }

  map<unsigned int, blob> blobs;
  // Sort and group blobs and account for min and max blob
  for(int column = 0; column < (int)lineSegment.size(); ++column)
    {
      for(int entry = 0; entry < (int)lineSegment[column].size(); ++entry)
        {
          if(blobs.find(lineSegment[column][entry].blobId) == blobs.end()) // Blob does not exist yet
            {
              blob blobData = {{column, lineSegment[column][entry].min}, {column, lineSegment[column][entry].max}, {0,0}};
              blobs[lineSegment[column][entry].blobId] = blobData;
            }
          else
            {
              if(lineSegment[column][entry].min < blobs[lineSegment[column][entry].blobId].min.y)
                blobs[lineSegment[column][entry].blobId].min.y = lineSegment[column][entry].min;
              else if(lineSegment[column][entry].max > blobs[lineSegment[column][entry].blobId].max.y)
                blobs[lineSegment[column][entry].blobId].max.y = lineSegment[column][entry].max;

              if(column > blobs[lineSegment[column][entry].blobId].max.x)
                blobs[lineSegment[column][entry].blobId].max.x = column;
            }
        }
    }

  // Calculate center
  int sizeball(0);
  for(map<unsigned int, blob>::iterator i = blobs.begin(); i != blobs.end(); ++i)
    {
      int size = ((*i).second.max.x - (*i).second.min.x) * ((*i).second.max.y - (*i).second.min.y);

      // Print coordinates on image, if it is large enough
      if(size <= ((m_result->m_Height*m_result->m_Width) * m_min_percent / 100) || size > ((m_result->m_Height*m_result->m_Width) * m_max_percent / 100))
        {
          m_center_point.X = -1.0;
          m_center_point.Y = -1.0;
        }
      else
        {
          m_center_point.X = (*i).second.min.x + ((*i).second.max.x - (*i).second.min.x) / 2;
          m_center_point.Y = (*i).second.min.y + ((*i).second.max.y - (*i).second.min.y) / 2;

          /*			unsigned int minX = (*i).second.min.x,
                                                  minY = (*i).second.min.y,
                                                  maxX = (*i).second.max.x,
                                                  maxY = (*i).second.max.y;

          			if(sizeball < size && minX > 5 && maxX < 315 && minY > 5 && maxY < 235)
          			{
          				if(maxX - minX > maxY - minY)
          				{
          					if((maxX - minX)/(maxY - minY) < 2)
          					{
          						m_center.X = m_center_point.X;
                                          		m_center.Y = m_center_point.Y;

          						sizeball = size;
          					}
          				}
          				else
          				{
          					if((maxY - minY)/(maxX - minX) < 2)
                                                  {
                                                          m_center.X = m_center_point.X;
                                                          m_center.Y = m_center_point.Y;

          						sizeball = size;
                                                  }
          				}
          			}
          */
          bool penalty = true;
          int x = (*i).second.min.x - 10;
          int y = (*i).second.min.y - 10;
          int xMin = (*i).second.min.x - 10;
          int xMax = (*i).second.max.x + 10;
          int yMin = (*i).second.min.y - 10;
          int yMax = (*i).second.max.y + 10;
          while( x < xMax )
            {
              const ColorClasses::Color c = imageColor(img, x, yMin);
              if(c != ColorClasses::green)
                {
                  penalty = false;
                  break;
                }

              const ColorClasses::Color c2 = imageColor(img, x, yMax);
              if(c2 != ColorClasses::green)
                {
                  penalty = false;
                  break;
                }

              x++;
            }
          while( y < yMax && penalty)
            {
              const ColorClasses::Color c = imageColor(img, xMin, y);
              if(c != ColorClasses::green)
                {
                  penalty = false;
                  break;
                }

              const ColorClasses::Color c2 = imageColor(img, xMax, y);
              if(c2 != ColorClasses::green)
                {
                  penalty = false;
                  break;
                }

              y++;
            }
          if(penalty)
            {

              m_center.X = m_center_point.X;
              m_center.Y = m_center_point.Y;
            }

//			Draw::Rectangle(img, Point2D((*i).second.min.x-5,(*i).second.min.y-5), Point2D((*i).second.max.x+5, (*i).second.max.y+5), ColorRGB(255,255,255));

        }
      listBlob.push_back(m_center_point);

      Point2D A, B;
      A.X = (*i).second.min.x;
      A.Y = (*i).second.min.y;
      B.X = (*i).second.max.x;
      B.Y = (*i).second.max.y;
      Draw::Rectangle(img, A, B, ColorRGB(0,0,0));
    }

  return listBlob;
}

ColorClasses::Color ColorFinder::imageColor(Image* img, int x, int y)
{
  if(img->m_ImageData[(y * Camera::WIDTH + x) * img->m_PixelSize + 0] == 255 &&
      img->m_ImageData[(y * Camera::WIDTH + x) * img->m_PixelSize + 1] == 255 &&
      img->m_ImageData[(y * Camera::WIDTH + x) * img->m_PixelSize + 2] == 0)
    return ColorClasses::yellow;
  else if(img->m_ImageData[(y * Camera::WIDTH + x) * img->m_PixelSize + 0] == 0 &&
          img->m_ImageData[(y * Camera::WIDTH + x) * img->m_PixelSize + 1] == 0 &&
          img->m_ImageData[(y * Camera::WIDTH + x) * img->m_PixelSize + 2] == 255)
    return ColorClasses::blue;
  else if(img->m_ImageData[(y * Camera::WIDTH + x) * img->m_PixelSize + 0] == 0 &&
          img->m_ImageData[(y * Camera::WIDTH + x) * img->m_PixelSize + 1] == 255 &&
          img->m_ImageData[(y * Camera::WIDTH + x) * img->m_PixelSize + 2] == 0)
    return ColorClasses::green;
  else
    return ColorClasses::none;
}

vector<Point2D> ColorFinder::getBlobCenter(Image *img, vector<Point2D> fieldBorderPoints)
{
  m_center.X = -1.0;
  m_center.Y = -1.0;

  int segmentCounter = 0;
  vector< vector<lineBlob> > lineSegment(m_result->m_Width);
  //Check existing lineBlob in every row
  for(int column = 0; column < (int)fieldBorderPoints.size(); ++column)//luqman
    {
      for(int row = fieldBorderPoints[column].Y; row < m_result->m_Height; ++row)//luqman
        {
          unsigned char byte = (unsigned char) m_result->m_ImageData[(row*m_result->m_Width)+ int(fieldBorderPoints[column].X)];

          if(byte == 1)
            {
              int start = row;
              for(; byte == 1 && row < m_result->m_Height; byte = (unsigned char) m_result->m_ImageData[(row*m_result->m_Width)+ int(fieldBorderPoints[column].X)], ++row);

              int stop = row-1;
              lineBlob lineBlobData = {start, stop, segmentCounter, false};

              lineSegment[fieldBorderPoints[column].X].push_back(lineBlobData);
              segmentCounter++;
            }
        }
    }
  vector<Point2D> listBlob;
  /* Check if any lineBlob exist */
  if (segmentCounter == 0)
    {
      m_center_point.X = -1.0;
      m_center_point.Y = -1.0;
      listBlob.push_back(m_center_point);
      return listBlob;
    }

  /* Check lineBlobs for a touching lineblob on the next column ; Assigning blobId and attached */
  for(int column = 0; column < (int)lineSegment.size()-1; ++column)//luqman
    {
      for(int entryLine1 = 0; entryLine1 < (int)lineSegment[column].size(); ++entryLine1)//luqman
        {
          for(int entryLine2 = 0; entryLine2 < (int)lineSegment[column+1].size(); ++entryLine2)//luqman
            {
              if(((lineSegment[column][entryLine1].max > lineSegment[column+1][entryLine2].min) || abs((int)lineSegment[column][entryLine1].max - (int)lineSegment[column+1][entryLine2].min) < 3)
                  && ((lineSegment[column][entryLine1].min < lineSegment[column+1][entryLine2].max) || abs((int)lineSegment[column][entryLine1].min - (int)lineSegment[column+1][entryLine2].max) < 3))
                {
                  if(lineSegment[column+1][entryLine2].attached == false)
                    {
                      lineSegment[column+1][entryLine2].blobId = lineSegment[column][entryLine1].blobId;
                      lineSegment[column+1][entryLine2].attached = true;
                    }
                  else
                    {
                      lineSegment[column][entryLine1].blobId = lineSegment[column+1][entryLine2].blobId;
                      lineSegment[column][entryLine1].attached = true;
                    }
                }
            }
        }
    }

  map<unsigned int, blob> blobs;
  // Sort and group blobs and account for min and max blob
  for(int column = 0; column < (int)lineSegment.size(); ++column)
    {
      for(int entry = 0; entry < (int)lineSegment[column].size(); ++entry)
        {
          if(blobs.find(lineSegment[column][entry].blobId) == blobs.end()) // Blob does not exist yet
            {
              blob blobData = {{column, lineSegment[column][entry].min}, {column, lineSegment[column][entry].max}, {0,0}};
              blobs[lineSegment[column][entry].blobId] = blobData;
            }
          else
            {
              if(lineSegment[column][entry].min < blobs[lineSegment[column][entry].blobId].min.y)
                blobs[lineSegment[column][entry].blobId].min.y = lineSegment[column][entry].min;
              else if(lineSegment[column][entry].max > blobs[lineSegment[column][entry].blobId].max.y)
                blobs[lineSegment[column][entry].blobId].max.y = lineSegment[column][entry].max;

              if(column > blobs[lineSegment[column][entry].blobId].max.x)
                blobs[lineSegment[column][entry].blobId].max.x = column;
            }
        }
    }

  // Calculate center
  int sizeball(0);
  for(map<unsigned int, blob>::iterator i = blobs.begin(); i != blobs.end(); ++i)
    {
      int size = ((*i).second.max.x - (*i).second.min.x) * ((*i).second.max.y - (*i).second.min.y);
			int width = ((*i).second.max.x - (*i).second.min.x);
			int height = ((*i).second.max.y - (*i).second.min.y);
			float widthperheight = width/height;
      // Print coordinates on image, if it is large enough
      if((size <= ((m_result->m_Height*m_result->m_Width) * m_min_percent / 100)) 
      || (size > ((m_result->m_Height*m_result->m_Width) * m_max_percent / 100)))
//      || (widthperheight < 0.8)
//      || (widthperheight > 1.25))
        {
          m_center_point.X = -1.0;
          m_center_point.Y = -1.0;
        }
      else
        {
          m_center_point.X = (*i).second.min.x + ((*i).second.max.x - (*i).second.min.x) / 2;
          m_center_point.Y = (*i).second.min.y + ((*i).second.max.y - (*i).second.min.y) / 2;

          if(sizeball < size)
            {
              m_center.X = m_center_point.X;
              m_center.Y = m_center_point.Y;
              sizeball = size;
            }
          //Draw::Circle(img, m_center_point, 3, ColorRGB(255,0,0));
        }
      //listBlob.push_back(m_center_point);

      Point2D A, B;
      A.X = (*i).second.min.x;
      A.Y = (*i).second.min.y;
      B.X = (*i).second.max.x;
      B.Y = (*i).second.max.y;
      Draw::Rectangle(img, A, B, ColorRGB(255,0,0));
    }
    Draw::Circle(img, m_center, 3, ColorRGB(255,0,0));
    listBlob.push_back(m_center);

  return listBlob;
}

vector<Point2D> ColorFinder::getObstacle(Image *img, vector<Point2D> fieldBorderPoints)
{
  m_center.X = -1.0;
  m_center.Y = -1.0;

  int segmentCounter = 0;
  vector< vector<lineBlob> > lineSegment(m_result->m_Width);
  //Check existing lineBlob in every row
  for(int column = 0; column < (int)fieldBorderPoints.size(); ++column)
    {
      for(int row = fieldBorderPoints[column].Y + 10; row < m_result->m_Height; ++row)
        {
          unsigned char byte = (unsigned char) m_result->m_ImageData[(row*m_result->m_Width)+ int(fieldBorderPoints[column].X)];

          if(byte == 0)
            {
              int start = row;
              for(; byte == 0 && row < m_result->m_Height; byte = (unsigned char) m_result->m_ImageData[(row*m_result->m_Width)+ int(fieldBorderPoints[column].X)], ++row);

              int stop = row-1;
              if(stop - start < 5) continue;
              lineBlob lineBlobData = {start, stop, segmentCounter, false};

              lineSegment[fieldBorderPoints[column].X].push_back(lineBlobData);
              segmentCounter++;
            }
        }
    }
  vector<Point2D> listBlob;
  /* Check if any lineBlob exist */
  if (segmentCounter == 0)
    {
      m_center_point.X = -1.0;
      m_center_point.Y = -1.0;
      listBlob.push_back(m_center_point);
      return listBlob;
    }

  /* Check lineBlobs for a touching lineblob on the next column ; Assigning blobId and attached */
  for(int column = 0; column < (int)lineSegment.size()-1; ++column)
    {
      for(int entryLine1 = 0; entryLine1 < (int)lineSegment[column].size(); ++entryLine1)
        {
          for(int entryLine2 = 0; entryLine2 < (int)lineSegment[column+1].size(); ++entryLine2)
            {
              if(((lineSegment[column][entryLine1].max > lineSegment[column+1][entryLine2].min) || abs((int)lineSegment[column][entryLine1].max - (int)lineSegment[column+1][entryLine2].min) < 3)
                  && ((lineSegment[column][entryLine1].min < lineSegment[column+1][entryLine2].max) || abs((int)lineSegment[column][entryLine1].min - (int)lineSegment[column+1][entryLine2].max) < 3))
                {
                  if(lineSegment[column+1][entryLine2].attached == false)
                    {
                      lineSegment[column+1][entryLine2].blobId = lineSegment[column][entryLine1].blobId;
                      lineSegment[column+1][entryLine2].attached = true;
                    }
                  else
                    {
                      lineSegment[column][entryLine1].blobId = lineSegment[column+1][entryLine2].blobId;
                      lineSegment[column][entryLine1].attached = true;
                    }
                }
            }
        }
    }

  map<unsigned int, blob> blobs;
  // Sort and group blobs and account for min and max blob
  for(int column = 0; column < (int)lineSegment.size(); ++column)
    {
      for(int entry = 0; entry < (int)lineSegment[column].size(); ++entry)
        {
          if(blobs.find(lineSegment[column][entry].blobId) == blobs.end()) // Blob does not exist yet
            {
              blob blobData = {{column, lineSegment[column][entry].min}, {column, lineSegment[column][entry].max}, {0,0}};
              blobs[lineSegment[column][entry].blobId] = blobData;
            }
          else
            {
              if(lineSegment[column][entry].min < blobs[lineSegment[column][entry].blobId].min.y)
                blobs[lineSegment[column][entry].blobId].min.y = lineSegment[column][entry].min;
              else if(lineSegment[column][entry].max > blobs[lineSegment[column][entry].blobId].max.y)
                blobs[lineSegment[column][entry].blobId].max.y = lineSegment[column][entry].max;

              if(column > blobs[lineSegment[column][entry].blobId].max.x)
                blobs[lineSegment[column][entry].blobId].max.x = column;
            }
        }
    }

  // Calculate center
  int sizeball(0);
  for(map<unsigned int, blob>::iterator i = blobs.begin(); i != blobs.end(); ++i)
    {
      int size = ((*i).second.max.x - (*i).second.min.x) * ((*i).second.max.y - (*i).second.min.y);

      // Print coordinates on image, if it is large enough
      if(size <= ((m_result->m_Height*m_result->m_Width) * m_min_percent / 100) || size > ((m_result->m_Height*m_result->m_Width) * m_max_percent / 100))
        {
          m_center_point.X = -1.0;
          m_center_point.Y = -1.0;
        }
      else
        {
          if(((*i).second.max.x - (*i).second.min.x) <= ((*i).second.max.y - (*i).second.min.y))
            {
              m_center_point.X = (*i).second.min.x + ((*i).second.max.x - (*i).second.min.x) / 2;
              m_center_point.Y = (*i).second.min.y + ((*i).second.max.y - (*i).second.min.y) / 2;

              if(sizeball < size)
                {
                  m_center.X = m_center_point.X;
                  m_center.Y = m_center_point.Y;
                  sizeball = size;
                }
              Draw::Cross(img, m_center_point, 10, ColorRGB(255,0,0));
            }
          else
            {
              m_center_point.X = -1.0;
              m_center_point.Y = -1.0;
            }
        }
      listBlob.push_back(m_center_point);

      Point2D A, B;
      A.X = (*i).second.min.x;
      A.Y = (*i).second.min.y;
      B.X = (*i).second.max.x;
      B.Y = (*i).second.max.y;
      Draw::Rectangle(img, A, B, ColorRGB(255,0,0));
    }

  return listBlob;
}

vector<Point2D> ColorFinder::linePercept(vector<Point2D> fieldBorderPoints, int segmentChildThreshold)
{
  int segmentCounter = 0;
  vector< vector<lineBlob> > lineSegment(m_result->m_Width);

  //Check existing lineBlob in every row
  for(int column = 0; column < (int)fieldBorderPoints.size(); ++column)
    {
      for(int row = fieldBorderPoints[column].Y; row < m_result->m_Height; ++row)
        {
          //unsigned char byte = (unsigned char) imgStream.get();
          unsigned char byte = (unsigned char) m_result->m_ImageData[(row*m_result->m_Width)+ int(fieldBorderPoints[column].X)];

          if(byte == 1)
            {
              int start = row;
              for(; byte == 1 && row < m_result->m_Height; byte = (unsigned char) m_result->m_ImageData[(row*m_result->m_Width)+ int(fieldBorderPoints[column].X)], ++row);
              int stop = row-1;

              if((stop - start) > segmentChildThreshold)
                {
                  lineBlob lineBlobData = {start, stop, segmentCounter, false};
                  lineSegment[fieldBorderPoints[column].X].push_back(lineBlobData);
                  segmentCounter++;
                }
            }
        }
    }

  vector<Point2D> listLine;

  // Check if any lineBlob exist
  if (segmentCounter == 0)
    {
      m_center_point.X = -1.0;
      m_center_point.Y = -1.0;
      listLine.push_back(m_center_point);
      return listLine;
    }

  /* Check lineBlobs for a touching lineblob on the next column ; Assigning blobId and attached */
  for(int column = 0; column < (int)lineSegment.size()-1; ++column)
    {
      for(int entryLine1 = 0; entryLine1 < (int)lineSegment[column].size(); ++entryLine1)
        {
          for(int entryLine2 = 0; entryLine2 < (int)lineSegment[column+1].size(); ++entryLine2)
            {
              if(((lineSegment[column][entryLine1].max > lineSegment[column+1][entryLine2].min) || abs((int)lineSegment[column][entryLine1].max - (int)lineSegment[column+1][entryLine2].min) < 30)
                  && ((lineSegment[column][entryLine1].min < lineSegment[column+1][entryLine2].max) || abs((int)lineSegment[column][entryLine1].min - (int)lineSegment[column+1][entryLine2].max) < 30))
                {
                  if(lineSegment[column+1][entryLine2].attached == false)
                    {
                      lineSegment[column+1][entryLine2].blobId = lineSegment[column][entryLine1].blobId;
                      lineSegment[column+1][entryLine2].attached = true;
                    }
                  else
                    {
                      lineSegment[column][entryLine1].blobId = lineSegment[column+1][entryLine2].blobId;
                      lineSegment[column][entryLine1].attached = true;
                    }
                }
            }
        }
    }

  vector < int > segmentId;
  map<unsigned int, blob> blobs;
  // Sort and group blobs and account for min and max blob
  for(int column = 0; column < (int)fieldBorderPoints.size(); column++)
    {
      for(int entry = 0; entry < (int)lineSegment[fieldBorderPoints[column].X].size(); ++entry)
        {
          if(blobs.find(lineSegment[fieldBorderPoints[column].X][entry].blobId) == blobs.end()) // Blob does not exist yet
            {
              blobs[lineSegment[fieldBorderPoints[column].X][entry].blobId] = {{0,0}, {0,0}, {0,0}};
              segmentId.push_back(lineSegment[fieldBorderPoints[column].X][entry].blobId);
            }
        }
    }


  vector< vector<Point2D> > linePoints(blobs.size());
  for(int column = 0; column < (int)fieldBorderPoints.size(); column++)
    {
      for(int entryLine1 = 0; entryLine1 < (int)lineSegment[fieldBorderPoints[column].X].size(); ++entryLine1)
        {
          for(int lineId = 0; lineId < (int)segmentId.size(); lineId++)
            {
              if(lineSegment[fieldBorderPoints[column].X][entryLine1].blobId == segmentId[lineId])
                {
                  Point2D pt;
                  pt.X = fieldBorderPoints[column].X;
                  pt.Y = (lineSegment[fieldBorderPoints[column].X][entryLine1].min + lineSegment[fieldBorderPoints[column].X][entryLine1].max)/2;
                  linePoints[lineId].push_back(pt);
                }
            }
        }
    }

  for(vector<vector<Point2D> >::iterator iter = linePoints.begin(); iter != linePoints.end(); iter++)
    {
      Point2D pt0 = (*iter).front();
      Point2D pt1 = (*iter).back();
      listLine.push_back(pt0);
      listLine.push_back(pt1);
    }

  return listLine;
}

void ColorFinder::EdgeDetection(Image* img)
{
	for (int i = 0; i < Camera::HEIGHT; i++)
    {
    	for(int j = 0; j < Camera::WIDTH; j++)
    	{
			if(imageColor(img,j,i) == ColorClasses::yellow)
			{
				if( (imageColor(img,j-1,i) != ColorClasses::orange) || 
					(imageColor(img,j,i-1) != ColorClasses::orange) ||
					(imageColor(img,j,i+1) != ColorClasses::orange) ||
					(imageColor(img,j+1,i) != ColorClasses::orange)
					)
				{
					img->m_ImageData[(Camera::WIDTH*(i) + j) * edge_img->m_PixelSize + 0] = 255;
					img->m_ImageData[(Camera::WIDTH*(i) + j) * edge_img->m_PixelSize + 1] = 255;
					img->m_ImageData[(Camera::WIDTH*(i) + j) * edge_img->m_PixelSize + 2] = 255;
				}
			}
			/*else
			{
				edge_img->m_ImageData[(Camera::WIDTH*(i) + j) * edge_img->m_PixelSize + 0] = 0;
				edge_img->m_ImageData[(Camera::WIDTH*(i) + j) * edge_img->m_PixelSize + 1] = 0;
				edge_img->m_ImageData[(Camera::WIDTH*(i) + j) * edge_img->m_PixelSize + 2] = 0;
			}*/
    	}  
    }
}

void ColorFinder::HoughCircle(Image* img, int r_min, int r_max)
{	
	int x0, y0;
    for(int i = 0; i < img->m_NumberOfPixels; i++)
    {
        if(img->m_ImageData[i*img->m_PixelSize + 0] == 255)
        {
            for(int r = r_min; r <= r_max; r++) 
            {
                for(int teta = 0; teta < MAXDEGREE; teta++ )
                {
                    x0 = i%Camera::WIDTH-r*cos((M_PI*teta)/180);
                    y0 = i/Camera::WIDTH-r*sin((M_PI*teta)/180);
                    if(x0 < Camera::HEIGHT && x0 > 0 && y0 < Camera::WIDTH && y0 > 0)
                    {
                        accumulator[x0][y0] += 1;
                    }
                }
            }
        }
    }
}

Point2D ColorFinder::Voting(Image* img)
{
  int local_maxima = 0;
  Point2D pos(0,0);
  for(int i = 0; i < img->m_NumberOfPixels; i++)
  {
    /*if(img->m_ImageData[i*img->m_PixelSize + 0] == 255)
    {*/
      for(int r = 10; r < 100; r++)
      {
        if(local_maxima > accumulator[i%Camera::WIDTH][i/Camera::WIDTH])
        {
          local_maxima = accumulator[i%Camera::WIDTH][i/Camera::WIDTH];
          pos = Point2D(i%Camera::WIDTH,i/Camera::WIDTH);
        }
      }
  }

  return pos;
}

void ColorFinder::FillHoughSpace(Image* img)
{
    for(int j = 0; j < MAXDEGREE; j++)
    {
        for(int i = 0; i < img->m_NumberOfPixels; i++)
        {
            /*img->m_ImageData[(MAXRADIUS*j + i)*img->m_PixelSize + 0] = accumulator[j*MAXRADIUS + i] + 3;
            img->m_ImageData[(MAXRADIUS*j + i)*img->m_PixelSize + 1] = accumulator[j*MAXRADIUS + i] + 3;
            img->m_ImageData[(MAXRADIUS*j + i)*img->m_PixelSize + 2] = accumulator[j*MAXRADIUS + i] + 3;*/
        }
    }
}

int ColorFinder::GetAccumulator(int i, int j)
{
	return 1;
}

Point2D ColorFinder::getPos(Image* rgb_img)
{
	EdgeDetection(rgb_img);
	HoughCircle(rgb_img,10,100);
	return(Voting(edge_img));
}

void ColorFinder::Reset()
{
	for (int i = 0; i < Camera::HEIGHT; i++)
    {
    	for(int j = 0; j < Camera::WIDTH; j++)
    	{
			for(int r=0; i<50; i++) 
    		{
        		accumulator[i][j] = 0;
        	}
        }
    }
}

void ColorFinder::LoadINISettings(minIni* ini)
{
  LoadINISettings(ini, COLOR_SECTION);
}

void ColorFinder::LoadINISettings(minIni* ini, const std::string &section)
{
  int value = -2;
  if((value = ini->geti(section, "hue", INVALID_VALUE)) != INVALID_VALUE)             m_hue = value;
  if((value = ini->geti(section, "hue_tolerance", INVALID_VALUE)) != INVALID_VALUE)   m_hue_tolerance = value;
  if((value = ini->geti(section, "min_saturation", INVALID_VALUE)) != INVALID_VALUE)  m_min_saturation = value;
  if((value = ini->geti(section, "min_value", INVALID_VALUE)) != INVALID_VALUE)       m_min_value = value;

  double dvalue = -2.0;
  if((dvalue = ini->getd(section, "min_percent", INVALID_VALUE)) != INVALID_VALUE)    m_min_percent = dvalue;
  if((dvalue = ini->getd(section, "max_percent", INVALID_VALUE)) != INVALID_VALUE)    m_max_percent = dvalue;

  color_section = section;
}

void ColorFinder::SaveINISettings(minIni* ini)
{
  SaveINISettings(ini, COLOR_SECTION);
}

void ColorFinder::SaveINISettings(minIni* ini, const std::string &section)
{
  ini->put(section,   "hue",              m_hue);
  ini->put(section,   "hue_tolerance",    m_hue_tolerance);
  ini->put(section,   "min_saturation",   m_min_saturation);
  ini->put(section,   "min_value",        m_min_value);
  ini->put(section,   "min_percent",      m_min_percent);
  ini->put(section,   "max_percent",      m_max_percent);

  color_section = section;
}

GoalPerceptor::GoalPerceptor()
{
  imgProcess = new Image(Camera::WIDTH, Camera::HEIGHT, Image::RGB_PIXEL_SIZE);
}

GoalPercept* GoalPercept::m_UniqueInstance = new GoalPercept();

GoalPercept::GoalPercept()
{
  LeftPart.X = -1;
  LeftPart.Y = -1;
  RightPart.X = -1;
  RightPart.Y = -1;
  RightFoot.X = -1;
  RightFoot.Y = -1;
  LeftFoot.X = -1;
  LeftFoot.Y = -1;
  Center.X = -1;
  Center.Y = -1;
  CenterHead.X = -1;
  CenterHead.Y = -1;
  CenterFoot.X = -1;
  CenterFoot.Y = -1;
  Status = NONE;
}

//################################################################################################################################

//Point2D GoalPerceptor::findUpperPost(Image* img, int xCenter, int yHorizon, int GoalColour)
Point2D GoalPerceptor::findUpperPost(Image* img, int xCenter, int yHorizon)
{
  ColorClasses::Color targetColor;
  targetColor = ColorClasses::yellow;

  int y = 0;
  int x = 0;
  int xStop = xCenter, xStart = xCenter;
  const int yEnd = yHorizon;

  int size = 0;
  Point2D CenterUpperPost(-1,-1);

  while( y < yEnd -1 )
    {
      const ColorClasses::Color c = imageColor(img, xCenter, y);
      if(c == targetColor)
        {
          //Draw::Cross(img, Point2D(xCenter,y), 5, ColorRGB(255,255,255));
          x = xCenter;
          x = runRight(img, xCenter, y, targetColor, Camera::WIDTH, 1);
          xStop = x;

          x = runLeft(img, xCenter, y, targetColor, 0, 1);
          xStart = x;

//			Draw::Cross(img, Point2D(xStart,y), 5, ColorRGB(255,255,255));
//                        Draw::Cross(img, Point2D(xStop,y), 5, ColorRGB(255,255,255));

          if(xStop - xStart > size)
            {
              size  = xStop - xStart;
              CenterUpperPost.X = (xStart+xStop)/2.0;
              CenterUpperPost.Y = (double)y;

//				Draw::Cross(img, Point2D(xStart,y), 5, ColorRGB(255,255,255));
//				Draw::Cross(img, Point2D(xStop,y), 5, ColorRGB(255,255,255));
            }
        }

      y++;
    }

  return CenterUpperPost;
}

//void GoalPerceptor::Process(Image* img, int yHorizon, int GoalColour)

void GoalPerceptor::Process(Image* img, int yHorizon, bool OppGoal)
{
  memcpy(imgProcess->m_ImageData, img->m_ImageData, img->m_ImageSize);
  findSpots(img, yHorizon);
  checkSpotsWidth(img);
  checkGreenBelow(img);
  //checkMaximumDistance();

  createPercept(img, OppGoal);

  /*int counter = 0;
  printf("SPOTS = %d   ",(int)spots.size());
  for(int i = 0; i < spots.size(); i++)
  {
  	if(!spots[i].dead)
  		counter++;
  }
  printf("ALIVE = %d\n",counter);
  */

  GoalPercept::GetInstance()->CenterUpperPost = Point2D(-1,-1);

  if(GoalPercept::GetInstance()->Status == GoalPercept::POSSIBLE_LEFT_POST || GoalPercept::GetInstance()->Status == GoalPercept::POSSIBLE_RIGHT_POST)
    {
      GoalPercept::GetInstance()->CenterUpperPost = findUpperPost(img, GoalPercept::GetInstance()->Center.X, yHorizon);
      //Draw::Cross(img, GoalPercept::GetInstance()->CenterUpperPost, 20, ColorRGB(255,255,255));
      //printf("Center(%lf,%lf)\n",GoalPercept::GetInstance()->CenterUpperPost.X,GoalPercept::GetInstance()->CenterUpperPost.Y);
      if(GoalPercept::GetInstance()->CenterUpperPost.Y < GoalPercept::GetInstance()->CenterHead.Y)
        {
          if(GoalPercept::GetInstance()->CenterUpperPost.X < GoalPercept::GetInstance()->CenterHead.X)
            {
              GoalPercept::GetInstance()->Status = GoalPercept::RIGHT_POST;
              GoalPercept::GetInstance()->CenterHead = GoalPercept::GetInstance()->CenterUpperPost;
            }
          else
            {
              GoalPercept::GetInstance()->Status = GoalPercept::LEFT_POST;
              GoalPercept::GetInstance()->CenterHead = GoalPercept::GetInstance()->CenterUpperPost;
            }
        }
//      else
//        {
//          GoalPercept::GetInstance()->Status = GoalPercept::UNKNOWN_POST;
//        }
    }
}

void GoalPerceptor::findSpots(Image* img, int yHorizon)
{
  spots.clear();

  if(yHorizon < 1)
    yHorizon = 1;

  if(yHorizon >= Camera::HEIGHT - 1)
    yHorizon = Camera::HEIGHT - 2;

  int x = 0;
  const int xEnd = Camera::WIDTH, yEnd = Camera::HEIGHT;
  //Draw::Line(img,  Point2D(0, yHorizon), Point2D(320, yHorizon), ColorRGB(ColorClasses::red));
  while( x < xEnd -1 )
    {
      x = findBlueOrYellowRight(imgProcess, x, yHorizon, xEnd);

      if( x == -1)
        break;
      ColorClasses::Color spotColor = imageColor(imgProcess, x, yHorizon);

      const int xStart = x;
      x++;

      if (x >= xEnd)
        break;

      x = runRight(imgProcess, x, yHorizon, spotColor, xEnd, parameters.maxSkip);
      const int xSegEnd = x;

      if(xSegEnd - xStart < parameters.minSegmentLength)
			{
				continue;
			}

      //Draw::Line(img, Point2D(xStart, yHorizon), Point2D(xSegEnd, yHorizon), ColorRGB(0, 0, 255));
      //Draw::Line(img, Point2D(0, yHorizon), Point2D(Camera::WIDTH, yHorizon), ColorRGB(255, 0, 0));

      //run down
      int lastYRunEnd;
      int yRunEnd;
      yRunEnd = yHorizon;
      int xMid = (xStart + xSegEnd) / 2;
      int footWidth;
      for(;;)
        {
          lastYRunEnd = yRunEnd;
          yRunEnd = runDown(imgProcess, xMid, yRunEnd, spotColor, yEnd, parameters.maxSkip);
          Draw::Line(img, Point2D(xMid, lastYRunEnd), Point2D(xMid, yRunEnd), ColorRGB(0, 255, 255));

          if(yRunEnd - lastYRunEnd < parameters.minYRunDiff)
            break;

          const int xS = runLeft(imgProcess, xMid, yRunEnd-1, spotColor, 0, parameters.maxSkip);
          const int xR = runRight(imgProcess, xMid, yRunEnd-1, spotColor, xEnd, parameters.maxSkip);
          //Draw::Line(img, Point2D(xMid, yRunEnd-1), Point2D(xS, yRunEnd-1), ColorRGB(0, 255, 255));
          //Draw::Line(img, Point2D(xMid, yRunEnd-1), Point2D(xR, yRunEnd-1), ColorRGB(0, 255, 255));
          xMid = (xS + xR) / 2;
          footWidth = xR - xS;
        }
      yRunEnd--;

      //run up
      int yRunEndUp = yHorizon;
      int xMidUp = (xStart + xSegEnd) / 2;
			int lastxMidUp = xMidUp;
			float Width = 1;
			float lastWidth = 1000;
      for(;;)
        {
          const int xS = runLeft(imgProcess, xMidUp, yRunEndUp+1, spotColor, 0, parameters.maxSkip);
          const int xR = runRight(imgProcess, xMidUp, yRunEndUp+1, spotColor, xEnd, parameters.maxSkip);
          //Draw::Line(img, Point2D(xMidUp, yRunEndUp+1), Point2D(xS, yRunEndUp+1), ColorRGB(ColorClasses::blue));
          //Draw::Line(img, Point2D(xMidUp, yRunEndUp+1), Point2D(xR, yRunEndUp+1), ColorRGB(ColorClasses::blue));
          xMidUp = (xS + xR) / 2;

					Width = xR - xS;
					
					if(Width/lastWidth > 1.5)
					{
						xMidUp = lastxMidUp;
						break;
					}
					lastxMidUp = xMidUp;
					lastWidth = Width;

          lastYRunEnd = yRunEndUp;
          yRunEndUp = runUp(imgProcess, xMidUp, yRunEndUp, spotColor, 0, parameters.maxSkip);
          //Draw::Line(img, Point2D(xMidUp, lastYRunEnd), Point2D(xMidUp, yRunEndUp), ColorRGB(ColorClasses::blue));

//          if(yRunEndUp - lastYRunEnd < parameters.minYRunDiff)
          if( lastYRunEnd - yRunEndUp < parameters.minYRunDiff)
				  break;

        }
      yRunEndUp++;

      if(yRunEnd - yRunEndUp < parameters.minPostPixelHeight)
        {
//        	printf("\nparameter.minPostPixel Hegiht \n"); 	
          //ARROW("module:GoalPerceptor:Image", xMidUp, yRunEndUp , (xMid + xMidUp)/2, (yRunEnd + yRunEndUp)/2, 2, Drawings::ps_dot, ColorRGBA(255,0, 0));
          //ARROW("module:GoalPerceptor:Image", xMid, yRunEnd , (xMid + xMidUp)/2, (yRunEnd + yRunEndUp)/2, 2, Drawings::ps_dot, ColorRGBA(255,0, 0));
          continue;
        }

      //Draw::Line(img, Point2D(0, parameters.minHeadVisibleOffset), Point2D(Camera::WIDTH, parameters.minHeadVisibleOffset), ColorRGB(ColorClasses::red));
      //Draw::Line(img, Point2D(0, Camera::HEIGHT - parameters.minHeadVisibleOffset), Point2D(Camera::WIDTH, Camera::HEIGHT - parameters.minHeadVisibleOffset), ColorRGB(ColorClasses::red));

      const bool headVisible = yRunEndUp > parameters.minHeadVisibleOffset;
      const bool        footVisible = Camera::HEIGHT - yRunEnd > parameters.minHeadVisibleOffset;
     if(!footVisible)
        {
          //ARROW("module:GoalPerceptor:Image", xMidUp, yRunEndUp, xMidUp + (xMid - xMidUp) * 1.2, yRunEndUp + (yRunEnd - yRunEndUp) * 1.2, 2, Drawings::ps_dot, ColorRGBA(255,0, 0));
          //continue;
        }

      //Draw::Line(img, Point2D(xMid, yRunEnd), Point2D(xMidUp, yRunEndUp), ColorRGB(255,0,255));

      Spot spotDetect;
      spotDetect.head.X = (float) xMidUp;
      spotDetect.head.Y = (float) yRunEndUp;
//	printf("spotDetecthead.X = %lf \n", spotDetect.head.X);
//	printf("spotDetecthead.Y = %lf \n", spotDetect.head.Y);
      spotDetect.foot.X = (float) xMid;
      spotDetect.foot.Y = (float) yRunEnd;
      spotDetect.headVisible = headVisible;
      spotDetect.footVisible = footVisible;
      spotDetect.color = spotColor;
      spotDetect.dead = false;
      spotDetect.center.X = 0;
      spotDetect.center.Y = 0;
      spotDetect.center.X = ( (float)xMidUp + (float)xMid ) / 2.0;
      spotDetect.center.Y = ( (float)yRunEndUp + (float)yRunEnd )/ 2.0;

      spotDetect.footWidth = footWidth;
      spotDetect.footWidth = xSegEnd - xStart;
      spotDetect.avrgWidth = footWidth;
      spots.push_back(spotDetect);
    }

  for(vector<Spot>::iterator iter = spots.begin(); iter !=  spots.end(); iter++)
    {
//      if((*iter).headVisible)
//        {
//	printf("crosssss on head \n");     
     Draw::Cross(img, Point2D((*iter).head.X, (*iter).head.Y), 5, ColorRGB(255,0,0));
//        }
//      if((*iter).footVisible)
//        {
//	printf("crosssss on foot \n");
          Draw::Cross(img, Point2D((*iter).foot.X, (*iter).foot.Y), 5, ColorRGB(0,0,255));
//        }

      //Draw::Line(img, Point2D(Point2D((*iter).head.X, (*iter).head.Y)), Point2D(Point2D((*iter).foot.X, (*iter).foot.Y)), ColorRGB(0, 255, 255));
    }
}

int GoalPerceptor::findBlueOrYellowRight(Image* img, int x, int yHorizon, int xEnd)
{
  while( x < xEnd )
    {
      const ColorClasses::Color c = imageColor(img, x,yHorizon);
      if(c == ColorClasses::blue || c == ColorClasses::yellow)
        return x;
      x++;
    }
  return -1;
}

ColorClasses::Color GoalPerceptor::imageColor(Image* img, int x, int y)
{
  if(img->m_ImageData[(y * Camera::WIDTH + x) * img->m_PixelSize + 0] == 255 &&
      img->m_ImageData[(y * Camera::WIDTH + x) * img->m_PixelSize + 1] == 255 &&
      img->m_ImageData[(y * Camera::WIDTH + x) * img->m_PixelSize + 2] == 0)
    return ColorClasses::yellow;
  else if(img->m_ImageData[(y * Camera::WIDTH + x) * img->m_PixelSize + 0] == 0 &&
          img->m_ImageData[(y * Camera::WIDTH + x) * img->m_PixelSize + 1] == 0 &&
          img->m_ImageData[(y * Camera::WIDTH + x) * img->m_PixelSize + 2] == 255)
    return ColorClasses::blue;
  else if(img->m_ImageData[(y * Camera::WIDTH + x) * img->m_PixelSize + 0] == 0 &&
          img->m_ImageData[(y * Camera::WIDTH + x) * img->m_PixelSize + 1] == 255 &&
          img->m_ImageData[(y * Camera::WIDTH + x) * img->m_PixelSize + 2] == 0)
    return ColorClasses::green;
  else
    return ColorClasses::none;
}

int GoalPerceptor::runRight(Image* img, int x, int y, ColorClasses::Color col, int xEnd, int maxSkip)
{
  int lastIndex;
  for (x+=maxSkip; x < xEnd; x+=maxSkip)
    {
      if (imageColor(imgProcess, x,y) != col)
        {
          lastIndex = std::max(x-maxSkip,0);
          for (--x; x > lastIndex; --x)
            if (imageColor(img, x,y) == col)
              break;

          if (x == lastIndex)
            {
              ++x;
              break;
            }
        }
    }

  if (x >= xEnd)
    x = xEnd-1;

  return x;
}

int GoalPerceptor::runLeft(Image* img, int x, int y, ColorClasses::Color col, int xEnd, int maxSkip)
{
  int lastIndex;
  for (x-=maxSkip; x >= xEnd; x-=maxSkip)
    {
      if (imageColor(img, x,y) != col)
        {
          lastIndex = x+maxSkip;
          for (++x; x < lastIndex; ++x)
            if (imageColor(img, x,y) == col)
              break;

          if (x == lastIndex)
            {
              --x;
              break;
            }
        }
    }

  if (x < xEnd)
    x = xEnd;

  return x;
}

int GoalPerceptor::runDown(Image* img, int x, int y, ColorClasses::Color col, int yEnd, int maxSkip)
{
  int lastIndex;
  for (y+=maxSkip; y < yEnd; y+=maxSkip)
    {
      if (imageColor(img, x,y) != col)
        {
          lastIndex = std::max(y-maxSkip,0);
          for (--y; y > lastIndex; --y)
            if (imageColor(img, x,y) == col)
              break;

          if (y == lastIndex)
            {
              ++y;
              break;
            }
        }
    }

  if (y >= yEnd)
    y = yEnd-1;

  return y;
}

int GoalPerceptor::runUp(Image* img, int x, int y, ColorClasses::Color col, int yEnd, int maxSkip)
{
  int lastIndex;
  for (y-=maxSkip; y >= yEnd; y-=maxSkip)
    {
      if (imageColor(img, x,y) != col)
        {
          lastIndex = y+maxSkip;
          for (++y; y < lastIndex; ++y)
            if (imageColor(img, x,y) == col)
              break;

          if (y == lastIndex)
            {
              --y;
              break;
            }
        }
    }

  if (y < yEnd)
    y = yEnd;

  return y;
}

void GoalPerceptor::checkSpotsWidth(Image* img)
{
  for(std::vector<Spot>::iterator iter = spots.begin(); iter != spots.end(); iter++)
    {
      (*iter).avrgWidth = (*iter).footWidth;
      const int expectedWidth = (*iter).footWidth;

      if( expectedWidth <= 0 )
        {
          (*iter).dead = true;
	  (*iter).foot.X = -1;
 	  (*iter).foot.Y = -1;
 	  (*iter).head.X = -1;
 	  (*iter).head.Y = -1;
 	  (*iter).center.X = -1; 
          (*iter).center.Y = -1;
          continue;
        }

      int sizeErrorCounter = 0;
      int widthCounter = 0;
      int validCounter = 0;
      Point2D avrgDir(0,0);
      Point2D lastMid(-1,-1);
      for(int y = int((*iter).foot.Y - parameters.widthStepSize/2); y >= (*iter).head.Y; y-=parameters.widthStepSize)
        {
          Point2D mid((*iter).foot.X, y);

          const int left = runLeft(imgProcess, (int)mid.X, (int)mid.Y, (*iter).color, 0, parameters.maxSkip)+1;
          const int right = runRight(imgProcess, (int)mid.X, (int)mid.Y, (*iter).color, Camera::WIDTH, parameters.maxSkip)-1;
          const int width = right - left;
          const float widthErrorRatio = width / (float)expectedWidth;



          if(widthErrorRatio > parameters.maxWidthErrorRatio || widthErrorRatio < parameters.minWidthErrorRatio)
            {
              if(widthErrorRatio > parameters.maxWidthErrorRatio)
                {
                  (*iter).errorLeft.push_back(Point2D(left, mid.Y));
                  (*iter).errorRight.push_back(Point2D(right, mid.Y));
                  (*iter).errorWidth.push_back(width);
                }
							sizeErrorCounter++;
              Draw::Line(img, Point2D(left, mid.Y), Point2D(right, mid.Y), ColorRGB(255, 0, 0));
            }
          else
            {
//              (*iter).center.X += (left + right)/2;
//              (*iter).center.Y += y;
//              (*iter).avrgWidth += width;
              validCounter++;

              Point2D midNew(double((right + left)/2), mid.Y);
              if(lastMid.X != -1)
                {
                  avrgDir += (midNew - lastMid);
                }
	              lastMid = midNew;
              Draw::Line(img, Point2D(left, mid.Y), Point2D(right, mid.Y), ColorRGB(0, 0, 255));//INI YANG DGANTI
            }
          widthCounter++;
        }
						//														foot.Y^2
		double postheight =  sqrt((pow((((*iter).foot.Y)-((*iter).head.Y)),2)) +   (pow((((*iter).foot.X)-((*iter).head.X)),2))   )	;

			if(postheight/(double)expectedWidth > 15)
			{
          (*iter).dead = true;
         // Draw::Line(img, Point2D((*iter).foot.X, (*iter).foot.Y), Point2D((*iter).head.X, (*iter).head.Y), ColorRGB(255, 0, 0));
         // Draw::Line(img, Point2D((*iter).foot.X-10, (*iter).foot.Y), Point2D((*iter).foot.X+10, (*iter).foot.Y), ColorRGB(255, 0, 0));
         // Draw::Line(img, Point2D((*iter).head.X-10, (*iter).head.Y), Point2D((*iter).head.X+10, (*iter).head.Y), ColorRGB(255, 0, 0));
          (*iter).foot.X = -1;
 	  (*iter).foot.Y = -1;
 	  (*iter).head.X = -1;
 	  (*iter).head.Y = -1;
 	  (*iter).center.X = -1; 
          (*iter).center.Y = -1;
       	  continue;
			
			}

      if(sizeErrorCounter / (float)widthCounter > parameters.maxWrongSizeRatio*3) //jika ga bener widthnya, maka spotny ga dianggep gawang
        {
          (*iter).dead = true;
          //Draw::Line(img, Point2D((*iter).foot.X, (*iter).foot.Y), Point2D((*iter).head.X, (*iter).head.Y), ColorRGB(255, 0, 0));
          //Draw::Line(img, Point2D((*iter).foot.X-10, (*iter).foot.Y), Point2D((*iter).foot.X+10, (*iter).foot.Y), ColorRGB(255, 0, 0));
          //Draw::Line(img, Point2D((*iter).head.X-10, (*iter).head.Y), Point2D((*iter).head.X+10, (*iter).head.Y), ColorRGB(255, 0, 0));
          (*iter).foot.X = -1;
 	  (*iter).foot.Y = -1;
 	  (*iter).head.X = -1;
 	  (*iter).head.Y = -1;
 	  (*iter).center.X = -1; 
          (*iter).center.Y = -1;
       	  continue;
        }
    }
}

void GoalPerceptor::checkGreenBelow(Image* img)
{
  for(vector<Spot>::iterator iter = spots.begin(); iter != spots.end(); iter++)
    {
      if( (*iter).dead )
        continue;

      int y = (int) (*iter).foot.Y;
      const int yEnd = std::min((int)((*iter).foot.Y + parameters.maxGreenScan), Camera::HEIGHT);
      //Draw::Line(img, Point2D(0, yEnd), Point2D(Camera::WIDTH, yEnd), ColorRGB(255,255, 255));

      int greenCounter = 0;
      while(y < yEnd)
        {
          if(imageColor(imgProcess, (int) (*iter).foot.X, y) == ColorClasses::green)
            greenCounter++;
          y++;
        }
      if(greenCounter < parameters.minGreenBelow && y < Camera::HEIGHT - parameters.minHeadVisibleOffset/*EDITAN TAMBAHAN*/)
        {
          //check whether there is some green on the left and right side
          const int leftStartPoint = std::max(0,int((*iter).foot.X) - (*iter).avrgWidth);
          const int rightStartPoint = std::min(Camera::WIDTH,int((*iter).foot.X) + (*iter).avrgWidth);
          //Draw::Line(img, Point2D(leftStartPoint,(*iter).foot.Y), Point2D(rightStartPoint,(*iter).foot.Y), ColorRGB(128,255,128));
          int xEnd = std::max(0, (int)(leftStartPoint - parameters.maxGreenScan));
          int x = leftStartPoint;

          const int y = std::min((int)((*iter).foot.Y + 2), Camera::HEIGHT-1);

          int leftGreenCounter = 0;
          while(x > xEnd)
            {
              if(imageColor(imgProcess, x, y) == ColorClasses::green)
                leftGreenCounter++;
              x--;
            }
          xEnd = std::min((int)(rightStartPoint + parameters.maxGreenScan), Camera::WIDTH);
          x = rightStartPoint;
          int rightGreenCounter = 0;
          while(x < xEnd)
            {
              if(imageColor(imgProcess, x, y) == ColorClasses::green)
                rightGreenCounter++;
              x++;
            }
          if(leftGreenCounter < parameters.minGreenBelow || rightGreenCounter < parameters.minGreenBelow)
            {
              (*iter).dead = true;
	      (*iter).foot.X = -1;
 	      (*iter).foot.Y = -1;
 	      (*iter).head.X = -1;
 	      (*iter).head.Y = -1;
 	      (*iter).center.X = -1; 
              (*iter).center.Y = -1;	
              //CROSS("module:GoalPerceptor:Image", s.foot.x, s.foot.y, 5, 3, Drawings::ps_solid, ColorRGBA(128,255,128));
            }
        }
    }
}
//CATATAN:
/* Jadi kalo iter.dead =true, maka spot tersebut tidak dianggap sebagai sebuah gawang, shingga semua parameter goalPercept = 0. 
iter.dead = true jika widthnya ga sama, trus jika bawahnya bkan ijo.*/


void GoalPerceptor::checkMaximumDistance()
{
  for(vector<Spot>::iterator iter = spots.begin(); iter != spots.end(); iter++)
    {
      Spot &s = (*iter);

      if(s.dead)
        continue;

      /*double distance = 35 * tan((50.0+Head::GetInstance()->GetTiltAngle())/180*M_PI);



      if(distance > parameters.maxFootDistance)
      {
      	s.dead = true;
      	continue;
      }
      double tilt = MotionStatus::m_CurrentJoints.GetAngle(JointData::ID_HEAD_TILT);
      printf("DISTANCE = %lf   TILT = %lf  ", distance, tilt);*/
    }
}

//bagian create percept ini ada bagian penentuan foot dari tiang.
//void GoalPerceptor::createPercept(Image* img, int TargetColor)

void GoalPerceptor::createPercept(Image* img, bool OppGoal)
{
  int yellowPosts = 0;
  Point2D upperPost(-1,-1);
  std::vector<Point2D> center, head, foot;

  GoalPercept::GetInstance()->LeftPart = Point2D(-1,-1);
  GoalPercept::GetInstance()->RightPart = Point2D(-1,-1);
  GoalPercept::GetInstance()->RightFoot = Point2D(-1,-1);
  GoalPercept::GetInstance()->LeftFoot = Point2D(-1,-1);
  GoalPercept::GetInstance()->Center = Point2D(-1,-1);
  GoalPercept::GetInstance()->CenterHead = Point2D(-1, -1);
  GoalPercept::GetInstance()->CenterFoot = Point2D(-1, -1);
  GoalPercept::GetInstance()->Status = GoalPercept::NONE;
  GoalPercept::GetInstance()->head.clear();
  GoalPercept::GetInstance()->foot.clear();
  GoalPercept::GetInstance()->CenterUpperPost = Point2D(-1,-1);
  GoalPercept::GetInstance()->Owner = GoalPercept::UNCLEAR;

  for(vector<Spot>::iterator iter = spots.begin(); iter != spots.end(); iter++)
    {
      Spot &s = (*iter);
      if(s.dead)
        continue;

      if(s.color == ColorClasses::yellow)
        {
          yellowPosts++;
        }
    }

  if (yellowPosts == 0)
    return;

  if(yellowPosts > 2)
    {
      //printf("UNFOUND2\n");
      return;
    }


  if (OppGoal){
    GoalPercept::GetInstance()->Owner = GoalPercept::OPPONENT_GOAL;
  }  
  else{
    GoalPercept::GetInstance()->Owner = GoalPercept::OWN_GOAL;
  }
    
  if(yellowPosts == 1) //kalau ada satu tiang kuning
    {
      for(vector<Spot>::iterator iter = spots.begin(); iter != spots.end(); iter++)
        {
          Spot &s = (*iter);

          if(s.dead)
	{
        	printf("deaddeaddeadededeadadadadaeded\n"); 
	   continue;
	}
          if(s.color == ColorClasses::yellow)
            {
              if(s.errorWidth.size() == 0)
                {
                  if(s.head.X > s.foot.X)
                    {
                      //printf("POSSIBLE RIGHT POST YELLOW\n");
                      GoalPercept::GetInstance()->LeftPart = s.foot;
                      GoalPercept::GetInstance()->RightPart = s.head;
                      //edited imre
                      GoalPercept::GetInstance()->RightFoot = s.foot;
                      //GoalPercept::GetInstance()->LeftFoot = s.foot;
                      // border
                      GoalPercept::GetInstance()->CenterHead = s.head;
                      GoalPercept::GetInstance()->CenterFoot = s.foot;
                      GoalPercept::GetInstance()->Center = (s.foot+s.head)/2.0;
                      GoalPercept::GetInstance()->Status = GoalPercept::POSSIBLE_RIGHT_POST;
                      GoalPercept::GetInstance()->head.push_back(s.head);
                      GoalPercept::GetInstance()->foot.push_back(s.foot);
                      //Draw::Cross(img,GoalPercept::GetInstance()->Center,5,ColorRGB(255,0,0));
                    }
                  else
                    {
                      //printf("POSSIBLE LEFT POST YELLOW\n");
                      GoalPercept::GetInstance()->LeftPart = s.head;
                      GoalPercept::GetInstance()->RightPart = s.foot;
                      //edited imre
                      //GoalPercept::GetInstance()->RightFoot = s.foot;
                      GoalPercept::GetInstance()->LeftFoot = s.foot;
                      // border
                      GoalPercept::GetInstance()->CenterHead = s.head;
                      GoalPercept::GetInstance()->CenterFoot = s.foot;
                      GoalPercept::GetInstance()->Center = (s.foot+s.head)/2.0;
                      GoalPercept::GetInstance()->Status = GoalPercept::POSSIBLE_LEFT_POST;
                      GoalPercept::GetInstance()->head.push_back(s.head);
                      GoalPercept::GetInstance()->foot.push_back(s.foot);
                      //Draw::Cross(img,GoalPercept::GetInstance()->Center,5,ColorRGB(255,0,0));
                    }
                }
              else
                {
                  double size = 0;
                  Point2D leftpointerror, rightpointerror;
                  for(int i = 0; i < (int)s.errorLeft.size(); i++)
                    {
                      if(s.errorRight[i].X - s.errorLeft[i].X > size)
                        {
                          size = s.errorRight[i].X - s.errorLeft[i].X;
                          leftpointerror = s.errorLeft[i];
                          rightpointerror = s.errorRight[i];
                        }
                    }

                  upperPost = (leftpointerror + rightpointerror)/2.0;

                  if( upperPost.X < s.center.X )
                    {
                      GoalPercept::GetInstance()->LeftPart = leftpointerror;
                      GoalPercept::GetInstance()->RightPart = s.center;
                      //edited imre
                      GoalPercept::GetInstance()->RightFoot = s.foot;
                      //GoalPercept::GetInstance()->LeftFoot = s.foot;
                      // border
                      //GoalPercept::GetInstance()->CenterHead = s.head;
                      GoalPercept::GetInstance()->CenterHead = upperPost;
                      GoalPercept::GetInstance()->CenterFoot = s.foot;
                      GoalPercept::GetInstance()->Center = (leftpointerror + s.center)/2.0;
                      GoalPercept::GetInstance()->Status = GoalPercept::RIGHT_POST;
                      GoalPercept::GetInstance()->head.push_back(s.head);
                      GoalPercept::GetInstance()->foot.push_back(s.foot);
//						printf("RIGHT POST YELLOW\n");
                    }
                  else
                    {
                      GoalPercept::GetInstance()->LeftPart = s.center;
                      GoalPercept::GetInstance()->RightPart = rightpointerror;//s.errorRight[0];
                      //edited imre
                      //GoalPercept::GetInstance()->RightFoot = s.foot;
                      GoalPercept::GetInstance()->LeftFoot = s.foot;
                      // border
                      //GoalPercept::GetInstance()->CenterHead = s.head;
                      GoalPercept::GetInstance()->CenterHead = upperPost;
                      GoalPercept::GetInstance()->CenterFoot = s.foot;
                      GoalPercept::GetInstance()->Center = (s.center + rightpointerror)/2.0;
                      GoalPercept::GetInstance()->Status = GoalPercept::LEFT_POST;
                      GoalPercept::GetInstance()->head.push_back(s.head);
                      GoalPercept::GetInstance()->foot.push_back(s.foot);
//						printf("LEFT POST YELLOW\n");
                    }
                }
            }
        }
    }
  else if (yellowPosts==2) //kalau ada dua tiang kuning
    {
      for(vector<Spot>::iterator iter = spots.begin(); iter != spots.end(); iter++) //just loop for every spots you have
        {
          Spot &s = (*iter);

          if(s.dead)
            continue;

          if(s.color == ColorClasses::yellow)
            {
              center.push_back(s.center);
              head.push_back(s.head);
              foot.push_back(s.foot);
            }
        }
      if(center.size() == 2)  //ada dua center tiang gawang
        {
          if(center[0].X > center[1].X)  //tiang pertama[0] ada di posisi lebih kanan di citra
            {
              GoalPercept::GetInstance()->LeftPart = center[1];
              GoalPercept::GetInstance()->RightPart = center[0];
            }
          else
            {
              GoalPercept::GetInstance()->LeftPart = center[0];
              GoalPercept::GetInstance()->RightPart = center[1];
            }

          if(foot[0].X > foot[1].X)
            {
              GoalPercept::GetInstance()->LeftFoot = foot[1];
              GoalPercept::GetInstance()->RightFoot = foot[0];
            }
          else
            {
              GoalPercept::GetInstance()->LeftFoot = foot[0];
              GoalPercept::GetInstance()->RightFoot = foot[1];
            }

          GoalPercept::GetInstance()->CenterHead = (head[0]+head[1])/2.0;
          GoalPercept::GetInstance()->CenterFoot = (foot[0]+foot[1])/2.0;
          GoalPercept::GetInstance()->head = head;
          GoalPercept::GetInstance()->foot = foot;
        }

      GoalPercept::GetInstance()->Center = (GoalPercept::GetInstance()->LeftPart + GoalPercept::GetInstance()->RightPart)/2.0;
      GoalPercept::GetInstance()->Status = GoalPercept::BOTH_POST;
//		printf("BOTH POST YELLOW\n");
    }

  //Draw::Cross(img, GoalPercept::GetInstance()->LeftPart, 10, ColorRGB(0,0,255));
  //Draw::Cross(img, GoalPercept::GetInstance()->RightPart, 10, ColorRGB(0,0,255));


  //hati2 kalau ada yang -1,-1
  //Draw::Cross(img, GoalPercept::GetInstance()->LeftFoot, 10, ColorRGB(255,0,0));
  //Draw::Cross(img, GoalPercept::GetInstance()->RightFoot, 10, ColorRGB(255,0,0));
  //Draw::Cross(img, GoalPercept::GetInstance()->CenterFoot, 10, ColorRGB(255,0,0));

//  GoalPercept::GetInstance()->CheckPostStatus();
}

void GoalPercept::CheckPostStatus()
{
  if(Owner == UNCLEAR)
    printf("UNCLEAR -- ");
  else if (Owner == OWN_GOAL)
    printf("OWN_GOAL -- ");
  else if (Owner == OPPONENT_GOAL)
    printf("OPPONENT_GOAL -- ");
  if(Status == NONE)
    printf("NONE\n");
  else if(Status == UNKNOWN_POST)
    printf("UNKNOWN POST\n");
  else if(Status == RIGHT_POST)
    printf("RIGHT POST\n");
  else if(Status == LEFT_POST)
    printf("LEFT POST\n");
  else if(Status == POSSIBLE_RIGHT_POST)
    printf("POSSIBLE RIGHT POST\n");
  else if(Status == POSSIBLE_LEFT_POST)
    printf("POSSIBLE LEFT POST\n");
  else if(Status == BOTH_POST)
    printf("BOTH POST\n");
}

