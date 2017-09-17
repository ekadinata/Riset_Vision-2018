/*
 * main.cpp
 *
 *  Created on: 2011. 1. 4.
 *      Author: robotis
 */

#include <unistd.h>
#include <string.h>
#include <libgen.h>

#include "Camera.h"
#include "Point.h"
#include "mjpg_streamer.h"
#include "minIni.h"
#include "LinuxCamera.h"
#include "ColorFinder.h"
#include "Head.h"

#define INI_FILE_PATH       "../../../../Data/config.ini"

void change_current_dir()
{
    char exepath[1024] = {0};
    if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
        chdir(dirname(exepath));
}

int main(void)
{
    printf( "\n===== Color filtering Tutorial for DARwIn =====\n\n");

    change_current_dir();

    minIni* ini = new minIni(INI_FILE_PATH);
    Image* rgb_ball = new Image(Camera::WIDTH, Camera::HEIGHT, Image::RGB_PIXEL_SIZE);

    LinuxCamera::GetInstance()->Initialize(1);
    LinuxCamera::GetInstance()->LoadINISettings(ini);

    mjpg_streamer* streamer = new mjpg_streamer(Camera::WIDTH, Camera::HEIGHT);

    ColorFinder* ball_finder = new ColorFinder();
    ball_finder->LoadINISettings(ini, "Orange");
    httpd::ball_finder = ball_finder;

    ColorFinder* red_finder = new ColorFinder();
    red_finder->LoadINISettings(ini, "Green");
    httpd::red_finder = red_finder;

    ColorFinder* yellow_finder = new ColorFinder();
    yellow_finder->LoadINISettings(ini, "Yellow");
    httpd::yellow_finder = yellow_finder;

    ColorFinder* blue_finder = new ColorFinder();
    blue_finder->LoadINISettings(ini, "Blue");
    httpd::blue_finder = blue_finder;

    httpd::ini = ini;
    
    while(1)
    {
        Point2D pos;
        LinuxCamera::GetInstance()->CaptureFrame();
        memcpy(rgb_ball->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageData, LinuxCamera::GetInstance()->fbuffer->m_RGBFrame->m_ImageSize);

        ball_finder->FilteringImage(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
        blue_finder->FilteringImage(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
        yellow_finder->FilteringImage(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);
        red_finder->FilteringImage(LinuxCamera::GetInstance()->fbuffer->m_HSVFrame);


        //fprintf(stderr, "posx: %f, posy: %f \r", pos.X, pos.Y);
        //printf("a: %lf", ball_finder->getPos(rgb_ball).X);

        for(int i = 0; i < rgb_ball->m_NumberOfPixels; i++)
        {
            if(ball_finder->m_result->m_ImageData[i] == 1)
            {
                rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 0] = 255;
                rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 1] = 128;
                rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 2] = 0;
            }
            else if(blue_finder->m_result->m_ImageData[i] == 1)
            {
                rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 0] = 0;
                rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 1] = 0;
                rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 2] = 255;
            }
            else if(yellow_finder->m_result->m_ImageData[i] == 1)
            {
                rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 0] = 255;
                rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 1] = 255;
                rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 2] = 0;
            }
            else if(red_finder->m_result->m_ImageData[i] == 1)
            {
                rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 0] = 0;
                rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 1] = 255;
                rgb_ball->m_ImageData[i*rgb_ball->m_PixelSize + 2] = 0;
            }
        }

        //double yHorizon = 120;
        //std::vector<Point2D> border = red_finder->getConvexFieldBorders(rgb_ball, 10, 10, yHorizon);
        //std::vector<Point2D> ball = ball_finder->getBlobCenter(rgb_ball, border);

        //ball_finder->EdgeDetection(rgb_ball);

        streamer->send_image(rgb_ball);
    }

    return 0;
}
