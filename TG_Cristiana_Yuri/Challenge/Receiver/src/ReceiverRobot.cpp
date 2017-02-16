#include <ReceiverRobot.hpp>
#include <Conversions.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <ledAuxiliarFunctions.hpp>
#include <motionNAOqiFunction.hpp>
#include <algorithm>

using namespace cv;

ReceiverRobot::ReceiverRobot(std::string remoteIP, int remotePort) : remoteIP(remoteIP), remotePort(remotePort), x_final(0), y_final(0)
{    
    camProxy = new AL::ALVideoDeviceProxy("127.0.0.1");
    AL::ALValue cameras, camerasColor, camerasRes;
    cameras.arraySetSize(2);
    camerasColor.arraySetSize(2);
    camerasRes.arraySetSize(2);
    cameras[0] = 0;
    cameras[1] = 1;
    camerasColor[0] = camerasColor[1] = AL::kBGRColorSpace;
    camerasRes[0] = camerasRes[1] = AL::kVGA;
    //clientName = camProxy->subscribeCameras("test123", cameras, camerasRes, camerasColor, 30);
    clientName = camProxy->subscribe("clientNamewqsxsdafac", AL::kVGA, AL::kBGRColorSpace, 30);
    camProxy->setParam(AL::kCameraSelectID,0);
    Mat frame = GetFrame();
    imwrite("frameInicial.jpg", frame);
    roi = Rect(300, 200, 170 ,100);
    toComputer = new CommRobot(this->remoteIP, this->remotePort);
}

ReceiverRobot::~ReceiverRobot()
{
    camProxy->unsubscribe(clientName);
}

int ReceiverRobot::Penalized()
{
    std::cout << "STATE PENALIZED" << std::endl;
    Command command;
    MotionNAOqi::Stand();
    command.leds = LEDAux::changeColor(Conversions::RED, Conversions::OFF, Conversions::OFF);

    MotionAux::SendCommand(command);
    toComputer->Disconnect();
    if(ButtonPressed())
    {
        return State::START;
    }
    return State::PENALIZED;
}

int ReceiverRobot::Start()
{
    std::cout << "STATE START" << std::endl;
    if(ButtonPressed())
    {
        return State::PENALIZED;
    }
    //Reinitialize Data
    data_offset = 0;


    Command command;

    //CHAMAR FUNCAO DA NAOQI
    MotionNAOqi::Stand();

    command.leds = LEDAux::changeColor(Conversions::OFF, Conversions::OFF, Conversions::OFF);

    MotionAux::SendCommand(command);

    toComputer->Connect();
    return State::CALIBRATE;
    /*int x, y, quadrant = 0;
    std::cin >> x;
    std::cin >> y;

    //Converting to transmission
    std::cout << "Starting Conversion" << std::endl;
    Conversions::MoveCenterToCorner(&x, &y);
    Conversions::ConvertToQuadrant(&x, &y, &quadrant);
    x = Conversions::decimal_octal(x);
    y = Conversions::decimal_octal(y);
    std::cout << "X: " << x << " Y: " << y << " Q: " << quadrant << std::endl;
    //Converting back
    x = Conversions::octal_decimal(x);
    y = Conversions::octal_decimal(y);
    Conversions::ConvertFromQuadrant(&x, &y, quadrant);
    Conversions::MoveCornerToCenter(&x, &y);
    toComputer->SendLocationToCommTester(x,y);*/
}

int ReceiverRobot::Calibrate()
{
    std::cout << "STATE CALIBRATE" << std::endl;
    if(ButtonPressed())
    {
        return State::PENALIZED;
    }
    DetectRects();
    while(rects.size() != 3)
    {
        if(ButtonPressed())
        {
            return State::PENALIZED;
        }
        DetectRects();
    }
    indexes = GetIndex();
    bool flagStart = true;
    while(flagStart)
    {
        if(ButtonPressed())
        {
            return State::PENALIZED;
        }
        std::vector<Mat> ledMat = GetLedMat();
        if(isTransition(ledMat))
        {
            std::cout << "Ended Calibration" << std::endl;
            return State::SYNC;
        }
    }
    /*
    //Wait for the First Message
    WaitTransition();
    //Get Mode
    MainLoop();
    //If field
        //goto reception
    //else if data
        //goto reception data
*/
}

int ReceiverRobot::Sync()
{
    std::cout << "STATE SYNC" << std::endl;
    if(ButtonPressed())
    {
        return State::PENALIZED;
    }
    if(WaitTransition() == false)
    {
        return State::PENALIZED;
    }
    std::vector<Mat> ledMat = GetLedMat();
    int c_led, r_led, l_led;
    c_led = GetColor(ledMat[LEDIndex::CENTER]);
    r_led = GetColor(ledMat[LEDIndex::RIGHT]);
    l_led = GetColor(ledMat[LEDIndex::LEFT]);
    std::vector<int> ret = getPredominantColor(c_led, r_led, l_led);
    if(ret[0] == -1)
    {
        return State::SYNC;
    }
    else
    {
        if(ret[0] == LEDColor::YELLOW)
        {
            //Is Field
            mode = true;
        }
        else
        {
            //Is Data
            mode = false;
        }
        return State::WAIT;
    }
}

int ReceiverRobot::Wait()
{
    std::cout << "STATE WAIT" << std::endl;
    if(ButtonPressed())
    {
        return State::PENALIZED;
    }
    if(WaitTransition() == false)
    {
        return State::PENALIZED;
    }
    return State::RECEIVE;
}

int ReceiverRobot::Receive()
{
    std::cout << "STATE RECEIVE" << std::endl;
    if(ButtonPressed())
    {
        return State::PENALIZED;
    }
    if(mode)
    {
        //FAZ COISAS DO FIELD
        std::vector<Mat> ledMat = GetLedMat();
        int c_led, r_led, l_led;
        c_led = GetColor(ledMat[LEDIndex::CENTER]);
        r_led = GetColor(ledMat[LEDIndex::RIGHT]);
        l_led = GetColor(ledMat[LEDIndex::LEFT]);
        std::vector<int> ret = getPredominantColor(c_led, r_led, l_led);
        if(ret[0] == -1)
        {
            return State::RECEIVE;
        }
        else
        {
            int number = transmitValue(ret);
            field_pack.push_back(number);
            if((int)(field_pack.size()) == 9)
            {
                return State::SEND;
            }
            return State::WAIT;
        }
    }
    else
    {
        //FAZ COISAS DO DATA
        std::vector<Mat> ledMat = GetLedMat();
        int c_led, r_led, l_led;
        c_led = GetColor(ledMat[LEDIndex::CENTER]);
        r_led = GetColor(ledMat[LEDIndex::RIGHT]);
        l_led = GetColor(ledMat[LEDIndex::LEFT]);
        std::vector<int> ret = getPredominantColor(c_led, r_led, l_led);
        if(ret[0] == -1)
        {
            return State::RECEIVE;
        }
        else
        {
            data_pack.push_back(ret[2]); //Chest
            data_pack.push_back(ret[1]); //REye
            data_pack.push_back(ret[0]); //LEye

            if((int)(data_pack.size()) >= 8)
            {
                return State::SEND;
            }
            return State::WAIT;
        }
    }
}

int ReceiverRobot::Send()
{
    std::cout << "STATE SEND" << std::endl;
    if(ButtonPressed())
    {
        return State::PENALIZED;
    }
    if(mode)
    {
        int x;
        int y;
        int quadrant;
        Conversions::vec2num(&x,&y,&quadrant,field_pack);
        std::cout << "X octal: " << x << " Y octal: " << y << "Qua: " << quadrant << std::endl;
        x = Conversions::octal_decimal(x);
        y = Conversions::octal_decimal(y);
        Conversions::ConvertFromQuadrant(&x, &y, quadrant);
        Conversions::MoveCornerToCenter(&x, &y);
        toComputer->SendLocationToCommTester(x, y);
        MotionNAOqi::moveArm(x,y);
        std::cout << "X final: " << x << " Y final: " << y << std::endl;
        field_pack.clear();
        return State::WAIT;
    }
    else
    {
        //TODO
        std::vector<int> byte;
        toComputer->SendDataToCommTester(data_pack, data_offset);
        data_offset++;
        data_pack.clear();
        return State::WAIT;
    }
}

void ReceiverRobot::DetectRects()
{

    //ParseHSVFile();
    std::cout << "Detect Rects" << std::endl;
    FindROI();

    bool first = true;
    int i = 0;
    Mat originalFrame1, originalFrame2;
    Mat thresh1, thresh2;
    Mat addedDiff;
    std::vector<Mat> im(NUM_OF_IMAGES);

    while(i < NUM_OF_IMAGES)
    {
        //Get image 1
        originalFrame1 = GetFrame();
        std::cout << "got frame 1 subtraction" << std::endl;
        thresh1 = Mat(originalFrame1, roi);
        boost::this_thread::sleep(boost::posix_time::milliseconds(100));

        //Get image 2
        originalFrame2 = GetFrame();
        std::cout << "got frame 2 subtraction" << std::endl;
        thresh2 = Mat(originalFrame2, roi);

        //Subtract images
        if(first)
        {
            absdiff(thresh1, thresh2, addedDiff);
            first = false;
            addedDiff = GenerateInverseMask(addedDiff);
            addedDiff.copyTo(im[0]);
        }
        else
        {
            Mat tmp;
            absdiff(thresh1, thresh2, tmp);
            tmp = GenerateInverseMask(tmp);
            add(addedDiff, tmp, addedDiff);
            addedDiff.copyTo(im[i-1]);
        }
    }

    //Invert mask

    bitwise_not(addedDiff, addedDiff);
    int erosion_size = 1;
    Mat element_erosion = getStructuringElement( MORPH_ELLIPSE, Size( 2*erosion_size + 1, 2*erosion_size+1 ), cv::Point( erosion_size, erosion_size ) );
    erode( addedDiff, addedDiff, element_erosion );

    int dilation_size = 1;
    Mat element_dilate = getStructuringElement( MORPH_ELLIPSE, Size( 2*dilation_size + 1, 2*dilation_size+1 ), cv::Point( dilation_size, dilation_size ) );

    dilate( addedDiff, addedDiff, element_dilate );

    rects = DetectBlobs(addedDiff);
    SquareRects();
}

Mat ReceiverRobot::GetFrame()
{
    std::cout << "get frame" << std::endl;
    Mat img(cv::Size(640,480), CV_8UC3);
    AL::ALValue image = camProxy->getImageRemote(clientName);
    if(image.getSize() < 7)
    {
        std::cout << "deu merda na imagem" << std::endl;
    }
    std::cout << "got image remote" << std::endl;
    img.data = (uchar*) image[6].GetBinary();
    std::cout << "got image from binary" << std::endl;
    if(img.empty())
    {
        std::cout << "Get Frame: Image Empty" << std::endl;
    }
    Mat imgRet;
    img.copyTo(imgRet);
    camProxy->releaseImage(clientName);
    return imgRet;
}

//DEPRECATED NOT NEEDED ANYMORE
Mat ReceiverRobot::GenerateMaskedImg(Mat originalFrame)
{
    Mat hsvFrame;
    Mat thresh;

    std::cout << "Roi" << std::endl;

    Mat roiFrame(originalFrame, roi);

    std::cout << "HSV in range" << std::endl;

    cvtColor(roiFrame, hsvFrame,CV_BGR2HSV);
    inRange(hsvFrame, Scalar(h1,s1,v1), Scalar(h2,s2,v2), thresh);

    //int erosion_size = 1;
    //Mat element_erosion = getStructuringElement( MORPH_ELLIPSE, Size( 2*erosion_size + 1, 2*erosion_size+1 ), Point( erosion_size, erosion_size ) );

    std::cout << "Dilation" << std::endl;
    int dilation_size = 4;
    Mat element_dilate = getStructuringElement( MORPH_ELLIPSE, Size( 2*dilation_size + 1, 2*dilation_size+1 ), cv::Point( dilation_size, dilation_size ) );

    //erode( thresh, thresh, element_erosion );
    dilate( thresh, thresh, element_dilate );

    std::cout << "Masking" << std::endl;

    Mat maskedImg;

    roiFrame.copyTo(maskedImg, thresh);

    //imwrite("maskedImg.jpg", maskedImg);

    return maskedImg;
}

Mat ReceiverRobot::GenerateInverseMask(Mat originalFrame)
{
    std::cout << "generate inverse mask" << std::endl;
    Mat imgGray;
    cvtColor(originalFrame, imgGray, CV_BGR2GRAY);
    Mat thresh;
    threshold(imgGray, thresh, 10, 255, THRESH_BINARY);
    Mat maskInv;
    bitwise_not(thresh, maskInv);
    return maskInv;
}

std::vector<Rect> ReceiverRobot::DetectBlobs(Mat binaryFrame)
{
    std::cout << "Detecting Blobs" << std::endl;
    std::vector<std::vector<cv::Point> > contours;
    std::vector<Vec4i> hierarchy;
    //Achou contornos dos blobs
    findContours(binaryFrame, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0,0));

    if(contours.size() <= 0)
    {
        std::cout << "No contour" << std::endl;
        //exit(1);
    }
    //Pega rects externos aos contornos
    std::vector<std::vector<cv::Point> > contours_poly(contours.size());
    std::vector<Rect> rects(contours.size());

    std::cout << "Getting Rects" << std::endl;
    for(int i = 0; i < contours.size(); i++)
    {
        approxPolyDP(Mat(contours[i]), contours_poly[i], 2, true);
        rects[i] = boundingRect( Mat(contours_poly[i]));
    }
    return rects;
}

void ReceiverRobot::SquareRects()
{
    std::cout << "Square rects" << std::endl;
    for(int i = 0; i < rects.size(); i++)
    {
        int centerX = rects[i].x + rects[i].width/2;
        int centerY = rects[i].y + rects[i].height/2;
        if(rects[i].width > rects[i].height)
        {
            rects[i].width = rects[i].height;
            rects[i].x = centerX - rects[i].width/2;
            rects[i].y = centerY - rects[i].height/2;
        }
        else if(rects[i].width < rects[i].height)
        {
            rects[i].height = rects[i].width;
            rects[i].x = centerX - rects[i].width/2;
            rects[i].y = centerY - rects[i].height/2;
        }
    }
}

void ReceiverRobot::ParseHSVFile()
{
    std::ifstream file("hsv_values.txt");
    std::string line;
    if(file.is_open())
    {
        int i = 0;
        while(std::getline(file, line))
        {
            if(line.at(0) == '#')
            {
                continue;
            }
            std::stringstream ss(line);
            ss >> h1 >> s1 >> v1 >> h2 >> s2 >> v2;
            i++;
        }
    }
    else
    {
        std::cout << "Could Not open HSV values file" << std::endl;
        exit(1);
    }
}

void ReceiverRobot::FindROI()
{
    std::cout << "Find Roi" << std::endl;
    int roi_x, roi_y, roi_width, roi_height;
    roi_x = 1;
    roi_y = 240;
    roi_width = IMG_WIDTH-1;
    roi_height = 100;

    Mat templ;
    templ = imread("/home/nao/naoqi/template_1bin.jpg", 1);
    if(templ.empty())
    {
        std::cout << "roi empty" << std::endl;
    }
    std::cout << "templ size "  << templ.rows << ", " << templ.cols  << "channels: " << templ.channels() << std::endl;
    int result_cols =  roi_width - templ.cols + 1;
    int result_rows = roi_height - templ.rows + 1;

    Mat result;
    result.create( result_rows, result_cols, CV_32FC1 );

    //(235,248,192,43)
    cv::Rect region_of_interest = cv::Rect(roi_x,
                                           roi_y,
                                           roi_width,
                                           roi_height);
    for(int i =0; i < 10; i++)
    {
        GetFrame();
    }
    Mat frame = GetFrame();
    std::cout << "frame size " << frame.rows << ", " << frame.cols << " channels: " << frame.channels() << std::endl;
    std::cout << "got frame inside findroi" << std::endl;
    Mat image_roi = frame(region_of_interest);
    std::cout << "get image roi" << std::endl;
    imwrite("frame.jpg", frame);
    matchTemplate( image_roi, templ, result, CV_TM_SQDIFF );
    std::cout << "match template" << std::endl;
    if(result.empty())
    {
        std::cout << "result match template empty" << std::endl;
        return;
    }
    normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );

    /// Localizing the best match with minMaxLoc
    double minVal; double maxVal; cv::Point minLoc; cv::Point maxLoc; cv::Point final_rect;
    cv::Point matchLoc;

    minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
    matchLoc = minLoc;

    matchLoc.x += roi_x; matchLoc.y += roi_y;
    final_rect.x = matchLoc.x - 4;
    final_rect.y = matchLoc.y - 50;

    roi = Rect(final_rect.x, final_rect.y, ROI_WIDTH, ROI_HEIGHT);
}

std::vector<Mat> ReceiverRobot::GetLedMat()
{
    std::cout << "get led mat" << std::endl;
    Mat frame = GetFrame();
    Mat roiFrame(frame, roi);
    std::vector<Mat> leds(3);
    leds[LEDIndex::RIGHT] = Mat(roiFrame, rects[indexes[LEDIndex::RIGHT]]);
    leds[LEDIndex::LEFT] = Mat(roiFrame, rects[indexes[LEDIndex::LEFT]]);
    leds[LEDIndex::CENTER] = Mat(roiFrame, rects[indexes[LEDIndex::CENTER]]);

    return leds;
}

bool ReceiverRobot::isTransition(std::vector<Mat> leds)
{
    std::cout << "is transition" << std::endl;
    int rightEye = GetColor(leds[LEDIndex::RIGHT]);
    int leftEye = GetColor(leds[LEDIndex::LEFT]);
    int chest = GetColor(leds[LEDIndex::CENTER]);
    if(rightEye == LEDColor::RED && leftEye == LEDColor::RED && chest == LEDColor::RED)
    {
        return true;
    }
}

std::vector<int> ReceiverRobot::GetIndex()
{
    std::cout << "get index" << std::endl;
    int chest_i,  r_eye_i, l_eye_i;
    std::vector<int> indexes;

    int chest_y = 0;
    for (size_t i = 0; i < rects.size(); i++) {
        if (rects[i].y > chest_y) {
            chest_y = rects[i].y;
            chest_i = i;
        }
    }

    int r_eye_x = 255;
    for (size_t i =0; i < rects.size(); i++) {
        if (i!=chest_i) {
            if (rects[i].x < r_eye_x) {
                r_eye_x = rects[i].x;
                r_eye_i = i;
            }
        }
    }

    for (size_t i =0; i < rects.size(); i++) {
        if ( (i!=chest_i) && (i!=r_eye_i) )
                l_eye_i = i;
    }

    indexes.push_back(r_eye_i);
    indexes.push_back(l_eye_i);
    indexes.push_back(chest_i);

    return indexes;
}

int ReceiverRobot::GetColor(cv::Mat img)
{
    std::cout << "get color" << std::endl;
    assert(img.channels() == 3);

    int color;
    cv::Mat hsv;
    cv::cvtColor(img,hsv,CV_BGR2HSV);
    cv::Scalar meanValue_hsv = cv::mean(hsv);
    float h = meanValue_hsv[0];

    if (h <= MAX_CYAN && h >= MIN_CYAN)
        color = LEDColor::CYAN;
    else if (h <= MAX_RED && h >= MIN_RED)
        color = LEDColor::RED;
    else if (h <= MAX_YELLOW && h >= MIN_YELLOW)
        color = LEDColor::YELLOW;
    else
        color = LEDColor::UNCLASSIFIED;

    return color;
}

bool ReceiverRobot::WaitTransition()
{
    std::cout << "wait transition" << std::endl;
    while(true)
    {
        if(ButtonPressed())
        {
            return false; //Penalized
        }
        std::vector<Mat> ledMat = GetLedMat();
        if(!isTransition(ledMat))
        {
            std::cout << "Ended Transition" << std::endl;
            break;
        }
    }
    return true;
}

void ReceiverRobot::MainLoop()
{
    bool penalized = false;
    while(true)
    {
        if(ButtonPressed())
        {

        }
    }
}

int ReceiverRobot::transmitValue(std::vector<int> colors)
{
    std::cout << "wait transition" << std::endl;
    int value = 0;

    for (size_t i = 0; i < colors.size(); i++)
        value+= colors[i]*pow(2,i);

    return value;
}

std::vector<int> ReceiverRobot::getPredominantColor (int c_led, int r_led, int l_led) {
    std::cout << "get predominant color" << std::endl;

    std::vector<int> color;
    bool flag=true;
    int c_color = LEDColor::UNCLASSIFIED;
    int l_color = LEDColor::UNCLASSIFIED;
    int r_color = LEDColor::UNCLASSIFIED;
    if (c_led != LEDColor::RED) {
        if (c_led == LEDColor::YELLOW)
            c_buffer.push_back(LEDColor::YELLOW);
        else if (c_led == LEDColor::CYAN)
            c_buffer.push_back(LEDColor::CYAN);
    }

    if (l_led != LEDColor::RED) {
        if (l_led == LEDColor::YELLOW)
            l_buffer.push_back(LEDColor::YELLOW);
        else if (l_led == LEDColor::CYAN)
            l_buffer.push_back(LEDColor::CYAN);
    }

    if (r_led != LEDColor::RED) {
        if (r_led == LEDColor::YELLOW)
            r_buffer.push_back(LEDColor::YELLOW);
        else if (r_led == LEDColor::CYAN)
            r_buffer.push_back(LEDColor::CYAN);
    }


    if ((c_led == LEDColor::RED) && (l_led == LEDColor::RED)
            && (r_led == LEDColor::RED)) {

        int c_yellow = std::count(c_buffer.begin(),c_buffer.end(),(int)LEDColor::YELLOW);
        int c_cyan = std::count(c_buffer.begin(),c_buffer.end(),(int)LEDColor::CYAN);
        int l_yellow = std::count(l_buffer.begin(),l_buffer.end(),(int)LEDColor::YELLOW);
        int l_cyan = std::count(l_buffer.begin(),l_buffer.end(),(int)LEDColor::CYAN);
        int r_yellow = std::count(r_buffer.begin(),r_buffer.end(),(int)LEDColor::YELLOW);
        int r_cyan = std::count(r_buffer.begin(),r_buffer.end(),(int)LEDColor::CYAN);

        c_color = c_yellow > c_cyan ? LEDColor::YELLOW : LEDColor::CYAN;
        l_color = l_yellow > l_cyan ? LEDColor::YELLOW : LEDColor::CYAN;
        r_color = r_yellow > r_cyan ? LEDColor::YELLOW : LEDColor::CYAN;

        color.push_back(l_color);
        color.push_back(r_color);
        color.push_back(c_color);

        flag = false;

        c_buffer.clear();
        l_buffer.clear();
        r_buffer.clear();

        return color;

        }
    else {
        color.push_back(-1);
        return color;
    }

}

/*bool ReceiverRobot::isField(std::vector<Mat> leds)
{
    int rightEye = GetColor(leds[LEDIndex::RIGHT]);
    int leftEye = GetColor(leds[LEDIndex::LEFT]);
    int chest = GetColor(leds[LEDIndex::CENTER]);
    if(rightEye == LEDColor::CYAN && leftEye == LEDColor::CYAN && chest == LEDColor::CYAN)
    {
        return true;
    }
    else
        return false;
}*/

bool ReceiverRobot::ButtonPressed()
{
    std::cout << "check button pressed" << std::endl;
    ButtonData buttons;
    buttons = buttonBoard.load();
    if(buttons.button.pop(1))
    {
        //Chest Button pressed once
        buttonBoard.save(buttons);
        return true;
    }
    buttonBoard.save(buttons);
    return false;
}
