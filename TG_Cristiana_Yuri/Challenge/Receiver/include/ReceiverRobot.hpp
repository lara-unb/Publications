#pragma once
#include <string>
#include <CommRobot.hpp>
#include <unBoard.hpp>
#include <ButtonData.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <vector>

// Aldebaran includes.
#include <alproxies/alvideodeviceproxy.h>
#include <alvision/alimage.h>
#include <alvision/alvisiondefinitions.h>
#include <alerror/alerror.h>

#define NUM_OF_IMAGES 6
#define IMG_WIDTH 640
#define ROI_WIDTH 33
#define ROI_HEIGHT 38

//10
#define MAX_YELLOW 60
#define MIN_YELLOW 30
//2
#define MAX_RED 190
#define MIN_RED 150
//0
#define MAX_CYAN 110
#define MIN_CYAN 80

namespace State
{
    enum
    {
        PENALIZED = 0,
        START,
        CALIBRATE,
        SYNC,
        WAIT,
        RECEIVE,
        SEND
    };
}

namespace LEDIndex
{
    enum
    {
        RIGHT = 0,
        LEFT,
        CENTER
    };
}

namespace LEDColor
{
    enum
    {
        YELLOW = 0,
        CYAN,
        RED,
        UNCLASSIFIED
    };
}

class ReceiverRobot
{
public:
    ReceiverRobot(std::string remoteIP, int remotePort);
    virtual ~ReceiverRobot();

    //STATES
    int Penalized();
    int Start();
    int Calibrate();
    int Sync();
    int Wait();
    int Receive();
    int Send();

private:
    bool ButtonPressed();
    //void DetectMessage();

    cv::Mat GetFrame();
    cv::Mat GenerateMaskedImg(cv::Mat originalFrame);
    cv::Mat GenerateInverseMask(cv::Mat originalFrame);
    std::vector<cv::Rect> DetectBlobs(cv::Mat binaryFrame);
    void SquareRects();
    void ParseHSVFile();
    void FindROI();
    std::vector<cv::Mat> GetLedMat();
    bool isTransition(std::vector<cv::Mat> leds);
    std::vector<int> GetIndex();
    int GetColor(cv::Mat img);
    void DetectRects();

    bool WaitTransition();
    void MainLoop();

    //return integer number from 3 color vector: MSC -> Chest -> REye -> LEye -> LSC
    int transmitValue(std::vector<int> colors);

    std::vector<int> getPredominantColor (int c_led, int r_led, int l_led);

    CommRobot* toComputer;
    unBoard<ButtonData> buttonBoard;
    std::string remoteIP;
    int remotePort;
    int h1, s1, v1; //1 = Lower boundary, 2 = upper boundary
    int h2, s2, v2;
    cv::Rect roi;
    AL::ALVideoDeviceProxy *camProxy;
    std::string clientName;
    std::vector<cv::Rect> rects;
    std::vector<int> indexes;
    bool mode;           //Mode: true - Field / false - Data

    std::vector<int> c_buffer, l_buffer, r_buffer; //for getPredominantColor

    int x_final, y_final;

    std::vector<int> data_pack;
    int data_offset;

    std::vector<int> field_pack;
};
