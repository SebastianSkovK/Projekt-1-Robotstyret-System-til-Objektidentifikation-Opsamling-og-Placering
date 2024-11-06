#include "opencv2/core.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include <iostream>
#include <vector>
#include "pylon/PylonIncludes.h"
#include "includes/calibrationPhotos.h"
#include "includes/cameraCalibration.h"
#include "includes/undistortion.h"
#include "includes/readVar.h"
#include "includes/objectDetection.h"
#include "includes/cornerPoints.h"
#include "includes/homograf.h"
#include "includes/drawCoordinates.h"
#include "includes/robot.h"
#include "includes/gripper.h"
#include "includes/data.h"
#include "includes/createFile.h"
#include <cmath>
#include "includes/RobotThrow.h"
#include "includes/jointVelocity.h"
#include "includes/angle.h"
#include "includes/throwVector.h"

#include "cmath"

Gripper *gripperObj;

void exitHandler()
{
    gripperObj->~Gripper();
}

double degToRad(int deg){
    return deg * (M_PI / 180);
}

struct MouseParams
{
    std::vector<cv::Point2i>* points;
    cv::Point2i currentPos;
};

void calibrationMouseHandler (int evt, int x, int y, int flags, void* p)
{
    MouseParams* params = (MouseParams*)p;

    params->currentPos = cv::Point2i (x, y) * 2;

    if (evt == cv::EVENT_LBUTTONDOWN)
    {
        params->points->push_back (params->currentPos);
        std::cout << "Point added: " << x << ", " << y << std::endl;
    }

    
}

int main(int argc, char *argv[])
{

    int myExposure = 30000;

    // The exit code of the sample application.
    int exitCode = 0;

    // Automagically call PylonInitialize and PylonTerminate to ensure the pylon runtime system
    // is initialized during the lifetime of this object.
    Pylon::PylonAutoInitTerm autoInitTerm;

    try
    {
        // Create an instant camera object with the camera device found first.
        Pylon::CInstantCamera camera(Pylon::CTlFactory::GetInstance().CreateFirstDevice());

        // Get a camera nodemap in order to access camera parameters.
        GenApi::INodeMap &nodemap = camera.GetNodeMap();

        // Open the camera before accessing any parameters.
        camera.Open();
        // Create pointers to access the camera Width and Height parameters.

        GenApi::CIntegerPtr width = nodemap.GetNode("Width");
        GenApi::CIntegerPtr height = nodemap.GetNode("Height");

        // The parameter MaxNumBuffer can be used to control the count of buffers
        // allocated for grabbing. The default value of this parameter is 10.
        // camera.MaxNumBuffer = 5;
        if (width.IsValid() && height.IsValid())
        {
            width->SetValue(1080);
            height->SetValue(1080);
        }
        GenApi::CIntegerPtr offsetX = nodemap.GetNode("OffsetX");
        GenApi::CIntegerPtr offsetY = nodemap.GetNode("OffsetY");
        if (offsetX.IsValid())
        {
            offsetX->SetValue(188);
        }
        if (offsetY.IsValid())
        {
            offsetY->SetValue(4);
        }
        // Genstart kameraet for at anvende ændringerne.
        camera.Close();
        camera.Open(); // Genstart kameraet
        // Create a pylon ImageFormatConverter object.
        Pylon::CImageFormatConverter formatConverter;
        // Specify the output pixel format.
        formatConverter.OutputPixelFormat = Pylon::PixelType_BGR8packed;
        // Create a PylonImage that will be used to create OpenCV images later.
        Pylon::CPylonImage pylonImage;

        // Create an OpenCV image.
        cv::Mat openCvImage;

        // Set exposure to manual
        GenApi::CEnumerationPtr exposureAuto(nodemap.GetNode("ExposureAuto"));
        if (GenApi::IsWritable(exposureAuto))
        {
            exposureAuto->FromString("Off");
            std::cout << "Exposure auto disabled." << std::endl;
        }

        // Set custom exposure
        GenApi::CFloatPtr exposureTime = nodemap.GetNode("ExposureTime");
        std::cout << "Old exposure: " << exposureTime->GetValue() << std::endl;
        if (exposureTime.IsValid())
        {
            if (myExposure >= exposureTime->GetMin() && myExposure <= exposureTime->GetMax())
            {
                exposureTime->SetValue(myExposure);
            }
            else
            {
                exposureTime->SetValue(exposureTime->GetMin());
                std::cout << ">> Exposure has been set with the minimum available value." << std::endl;
                std::cout << ">> The available exposure range is [" << exposureTime->GetMin() << " - " << exposureTime->GetMax() << "] (us)" << std::endl;
            }
        }
        else
        {

            std::cout << ">> Failed to set exposure value." << std::endl;
            return false;
        }
        std::cout << "New exposure: " << exposureTime->GetValue() << std::endl;

        // Start the grabbing of c_countOfImagesToGrab images.
        // The camera device is parameterized with a default configuration which
        // sets up free-running continuous acquisition.
        camera.StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);

        //samling af hjørne punkter
        std::vector<int> pixelsBall, pixelsCup;
        // lokale includes med metoder

        // This smart pointer will receive the grab result data.
        Pylon::CGrabResultPtr ptrGrabResult;

        std::cout << "Would you like to perfom calibration? (Y/n)" << std::endl;

        char calibrateChoice;
        std::cin >> calibrateChoice;

        if (calibrateChoice == 'y' || calibrateChoice == 'Y')
        {
            std::cout << "Would you like to calibrate camera? (Y/n)" << std::endl;
            char cameraCalibrationChoice;
            std::cin >> cameraCalibrationChoice;

            if (cameraCalibrationChoice == 'y' || cameraCalibrationChoice == 'Y')
            {
                int counter = 1;

                while (camera.IsGrabbing ())
                {
                    // Wait for an image and then retrieve it. A timeout of 500 ms is used.
                    camera.RetrieveResult(500, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);

                    // Image grabbed successfully?
                    if (ptrGrabResult->GrabSucceeded())
                    {
                        formatConverter.Convert(pylonImage, ptrGrabResult);
                        cv::Mat frame(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t *)pylonImage.GetBuffer());
                        
                        if (!calibrationPhotos(frame, counter))
                        {
                            // Kalibrere vi kameraet med billederne taget med calibrationPhotos
                            calibrateCamera();
                            // Her anvender vi den fundne perspectivering til at funde homografi for kamera til bord og kamera til kop
                            cv::destroyAllWindows();
                        }
                    }
                }
            }

            std::cout << "Would you like to calibrate ball and cup pixel offsets? (Y/n)" << std::endl;
            char cornerPointsChoice;
            std::cin >> cornerPointsChoice;

            if (cornerPointsChoice == 'y' || cornerPointsChoice == 'Y')
            {
                cv::namedWindow ("Select Corner Points");

                MouseParams params;
                
                params.currentPos = cv::Point2i (0, 0);

                cv::setMouseCallback ("Select Corner Points", calibrationMouseHandler, &params);

                for (int i = 0; i < 2; i++)
                {
                    std::cout << "Mark corners with " << (i == 0 ? "balls" : "cups") << " and press any key..." << std::endl;
                        
                    char c;
                    std::cin >> c;

                    std::vector<cv::Point2i> cornerPoints;

                    params.points = &cornerPoints;

                    while (true)
                    {
                        camera.RetrieveResult(500, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);

                        if (!ptrGrabResult->GrabSucceeded())
                            throw std::logic_error ("Failed to grab image");

                        formatConverter.Convert(pylonImage, ptrGrabResult);
                        cv::Mat frame(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t *)pylonImage.GetBuffer());

                        undistort(frame);

                        for (const auto& p : cornerPoints)
                        {
                            cv::circle (frame, p, 5, cv::Scalar (0, 0, 255), 2);
                        }

                        cv::circle (frame, params.currentPos, 10, cv::Scalar (255, 255, 255), 2);

                        cv::resize (frame, frame, frame.size () / 2);
                        cv::imshow ("Select Corner Points", frame);

                        cv::pollKey ();

                        if (cornerPoints.size () == 2)
                            break;
                    }

                    std::string path = (i == 0 ? "../ballVar.yaml" : "../cupVar.yaml");

                    cv::FileStorage fs(path, cv::FileStorage::WRITE);
                    fs << "xMin" << cornerPoints[0].x;
                    fs << "yMin" << cornerPoints[0].y;
                    fs << "xMax" << cornerPoints[1].x;
                    fs << "yMax" << cornerPoints[1].y;
                    fs.release();

                    std::vector<int>& pixels = (i == 0 ? pixelsBall : pixelsCup);
                    pixels.resize (4);

                    pixels[0] = cornerPoints[0].x;
                    pixels[1] = cornerPoints[0].y;
                    pixels[2] = cornerPoints[1].x;
                    pixels[3] = cornerPoints[1].y;
                }

                cv::destroyWindow ("Select Corner Points");
            }
        }
        

        // image grabbing loop
        int frame = 1;
        gripperObj = new Gripper("192.168.1.20", 1000);
        Gripper &gripper = *gripperObj;
        atexit(exitHandler);
        int score = 0;
        Robot robot("192.168.1.10");
        robot.loadPointsAndCalculateMatrix();
        bool ballGrib = false;
        if(pixelsBall.empty())
        {
            readVar(false, pixelsBall);
        }
        if(pixelsCup.empty())
        {
            readVar(true, pixelsCup);
        }

        if(pixelsCup.empty() || pixelsBall.empty())
            throw std::logic_error ("Failed to load corner points from .yaml files, check for errors");

        std::cout << "Would you like to gather data on ball and cup positions? (Y/N)" << std::endl;
        char dataChoice;
        std::cin >> dataChoice;
        if(dataChoice == 'y' || dataChoice == 'Y')
        {
                bool cupOrBall = false;
                std::vector<std::vector<double > > container;
                while(camera.IsGrabbing ())
                {
                    std::cout << "Place objects and press any key..." << std::endl;
                    char c;
                    std::cin >> c;
                    
                    camera.RetrieveResult(500, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);

                    if (!ptrGrabResult->GrabSucceeded())
                        continue;

                    formatConverter.Convert(pylonImage, ptrGrabResult);
                    cv::Mat frame(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t *)pylonImage.GetBuffer());

                    undistort(frame);
                    
                    if(readPoints(frame,pixelsBall,false,container) && !cupOrBall)
                    {   
                            std::cout << "Would you like to calculate more ball points? (Y/N)" << std::endl;
                            char choice;
                            std::cin >> choice;
                            if(choice == 'y' || choice == 'Y')
                                continue;
                            else
                            {
                            createFile(container,false);
                            cupOrBall = true;
                            container.clear();
                            }

                    }
                    else if(readPoints(frame,pixelsCup,true,container) && cupOrBall)
                    {
                            std::cout << "Would you like to calculate more cup points? (Y/N)" << std::endl;
                            char choice;
                            std::cin >> choice;
                            if(choice == 'y' || choice == 'Y')
                                continue;
                            {
                            createFile(container,true);
                            break;
                            }
                    }
                    else
                        std::cout << "Failed to read points, try again." << std::endl;
                }
        }

        std::vector<double> homeQ = { degToRad(-67), degToRad(-55), degToRad(-137), degToRad(-75), degToRad(90), degToRad(0) };
        robot.getControl().moveJ(homeQ);

        while (camera.IsGrabbing())
        {
            // Wait for an image and then retrieve it. A timeout of 5000 ms is used.
            camera.RetrieveResult(5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
            std::vector<double> q_start = {degToRad(-67), degToRad(-7), degToRad(-123), degToRad(-41), degToRad(90), degToRad(0)}; 
            std::vector<double> q_end = {degToRad(-67), degToRad(-70), degToRad(-70), degToRad(-20), degToRad(90), degToRad(0)}; // Bruges ikke

            cv::Matx31d robotForward = cv::Matx31d(-1,0,0);

            // Image grabbed successfully?
            if (ptrGrabResult->GrabSucceeded())
            {
                // Access the image data.
                // cout << "SizeX: " << ptrGrabResult->GetWidth() << endl;
                // cout << "SizeY: " << ptrGrabResult->GetHeight() << endl;

                // Convert the grabbed buffer to a pylon image.
                formatConverter.Convert(pylonImage, ptrGrabResult);
                // Tager et frame og undistorter det
                cv::Mat stream = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t *)pylonImage.GetBuffer());
                undistort(stream);
                // Finder bolde og deres centrum
                std::vector<cv::Vec3f> ballCenter;
                if (detectObject(stream, ballCenter, false,pixelsBall))
                {
                    for (int i = 0; i < ballCenter.size(); i++)
                        drawCoordinate(cv::Point3f(pixelsBall[0] + ballCenter[i][0], pixelsBall[1] + ballCenter[i][1], ballCenter[i][2]), stream, pixelsBall);
                }
                // Finder kopper og deres centrum
                std::vector<cv::Vec3f> cupCenter;
                if (detectObject(stream, cupCenter, true,pixelsCup))
                {
                    for (int i = 0; i < cupCenter.size(); i++)
                        drawCoordinate(cv::Point3f(pixelsCup[0] + cupCenter[i][0], pixelsCup[1] + cupCenter[i][1], cupCenter[i][2]), stream, pixelsCup);
                }
                // Viser framet med det koordinater for kop og bold hvis de kan lokaliseres.
                /*cv::Mat resizedStream;
                cv::Size res(1920, 1080);
                cv::resize(stream, resizedStream, res);
                cv::namedWindow("stream", cv::WINDOW_NORMAL);
                cv::setWindowProperty("stream", cv::WND_PROP_FULLSCREEN, cv::WINDOW_FULLSCREEN);*/
                cv::imshow("stream", stream);
                // Gemmer koordinaterne fra koppen og bolden og sender det over til robotten.


                
                if (ballCenter.size() > 0 && !ballGrib)
                {
                    // Omregner via homografi fra pixel koordinater til world coordinater (CM)
                    cv::Point2f ball = hom(cv::Point2f(pixelsBall[0] + ballCenter[0][0], pixelsBall[1] + ballCenter[0][1]), pixelsBall);
                    cv::Point3f moveRobotBall(ball.x / 100, ball.y / 100, 0);

                    robot.move(cv::Matx31d(moveRobotBall.y, moveRobotBall.x, 0.2), cv::Matx31d(0, -3.1415, 0));
                    robot.move(cv::Matx31d(moveRobotBall.y, moveRobotBall.x, 0.0175), cv::Matx31d(0, -3.1415, 0));
                    gripper.grip();
                    robot.move(cv::Matx31d(moveRobotBall.y, moveRobotBall.x, 0.2), cv::Matx31d(0, -3.1415, 0));
                    ballGrib = true;

                    //robot.getControl().moveJ(q_start);

                    //RobotThrow kast(robot.getControl(), gripper, q_start, q_end); // moves to start pos and performs throw.

                }
                
                if (cupCenter.size() > 0 && ballGrib)
                {
                    // Omregner via homografi fra pixel koordinater til world coordinater (CM)
                    score = cupCenter.size();
                    cv::Point2f cupCoordinate = hom(cv::Point2f(pixelsCup[0] + cupCenter[0][0], pixelsCup[1] + cupCenter[0][1]), pixelsCup);
                    cv::Matx31d moveRobotCup(cupCoordinate.y / 100, cupCoordinate.x / 100, 0.12);

                    cv::Matx31d throwPointBallPos((cupCoordinate.y / 100)+ 0.1, (cupCoordinate.x / 100), 0.3);
                    
                    
                    cv::Matx31d cupVector = vectormaker(cv::Matx31d(moveRobotCup(0), moveRobotCup(1), 0) );
                    double cupAngle = angle(robotForward, cupVector);
                    // std::cout << "angle = " << (cupAngle/M_PI)*180.0 << std::endl;
                    // q_start[0] += cupAngle;
                    
                    double distToThrowLink = 0.109;
                    double lenghtBaseToCup = cv::norm(cupVector);
                    // std::cout << "lenght to cup: " << lenghtBaseToCup << std::endl;
                    double baseAngle = asin(distToThrowLink/lenghtBaseToCup) + cupAngle;
                    q_start[0] += baseAngle;
                    q_end[0] = q_start[0];
                    robot.getControl().moveJ(q_start);

                    cv::Matx61d qStartMat = vecToMat (q_start);
                    cv::Matx31d throwStartRobotSpace = getTcpPosToGivenQ (qStartMat);

                    cv::Matx31d throwStartWorldSpace = d4Tod3 (robot.getRobotToWorld () * d3Tod4 (throwStartRobotSpace));

                    double throwCircleRadius = 0.25;

                    cv::Matx21d start2D(throwStartWorldSpace(0), throwStartWorldSpace(1));
                    cv::Matx21d cup2D (moveRobotCup(0), moveRobotCup(1));

                    cv::Matx21d startToCup2D = cup2D - start2D;
                    startToCup2D /= cv::norm (startToCup2D);

                    cup2D += startToCup2D * 0; // Kast eventuelt "for langt"

                    cv::Matx21d throwEnd2D = start2D + startToCup2D * throwCircleRadius;
                    cv::Matx31d throwEndWorldSpace(throwEnd2D(0), throwEnd2D(1), throwStartWorldSpace(2) + throwCircleRadius);

                    double height = throwEndWorldSpace(2);
                    double length = cv::norm (throwEnd2D - cup2D);
                    
                    // Throw
                    cv::Matx31d cupRobotSpace = d4Tod3 (robot.getWorldToRobot () * d3Tod4 (moveRobotCup));
                    cv::Matx31d throwDirRobotSpace = cupRobotSpace - throwStartRobotSpace;
                    throwDirRobotSpace(2) = 0;
                    throwDirRobotSpace /= cv::norm (throwDirRobotSpace);

                    int freq = 125;
                    double dt = 1.0 / freq;
                    double gain = 1000;
                    double lookahead = 0.05;

                    double throwSpeed = sqrt(2) * length * sqrt(9.82 / height) * 0.5;

                    // Iterate over distance of on a circle cirfumerence, where the end effector velocity is slowly increased over time

                    double throwCircleAngle = M_PI / 2;
                    double throwTravelLength = throwCircleRadius * throwCircleAngle;
                    double stopTravelLength = 0.1;
                    double travelLength = throwTravelLength + stopTravelLength;

                    cv::Matx61d q = qStartMat;

                    bool thrown = false;

                    std::cout << "Throwing... " << "length = " << length << " height = " << height << " speed = " << throwSpeed << std::endl << "travelLength = " << travelLength << std::endl;

                    for (double p = 0; p <= travelLength; )
                    {
                        std::chrono::steady_clock::time_point t_start = robot.getControl().initPeriod();

                        cv::Matx31d velDir = cv::Matx31d::zeros ();
                        double speedScale;

                        if (p < throwTravelLength)
                        {
                            double arcAngle = p / throwCircleRadius;
                            cv::Matx21d circleTangent2D(sin(arcAngle), cos(arcAngle));
                            velDir(0) = circleTangent2D(0) * throwDirRobotSpace(0);
                            velDir(1) = circleTangent2D(0) * throwDirRobotSpace(1);
                            velDir(2) = circleTangent2D(1);

                            speedScale = arcAngle <= throwCircleAngle ? (p / throwTravelLength) : 1;

                            if (speedScale < 0.05)
                                speedScale = 0.05;
                        } else
                        {
                            velDir = throwDirRobotSpace;
                            speedScale = 1;
                        }

                        std::cout << "p = " << p << " velDir = " << velDir << " speedScale = " << speedScale << std::endl;

                        double currentSpeed = throwSpeed * speedScale;
                        cv::Matx31d v = velDir * currentSpeed;

                        cv::Matx61d endEffectorVelocity (v(0), v(1), v(2), 0, 0, 0);

                        /*cv::Matx61d dq = getJointVelocities (q, endEffectorVelocity);
                        q += dq * dt;*/

                        cv::Matx31d dq = getJointVelocitiesSEW (q, endEffectorVelocity);
                        
                        for (int i = 0; i < 3; i++)
                        {
                            q(i + 1) += dq(i) * dt;
                        }

                        p += currentSpeed * dt;

                        if (p > throwTravelLength * 1 && !thrown)
                        {
                            thrown = true;
                            gripper.open (false);
                        }

                        robot.getControl().servoJ (matToVec (q), 0, 0, dt, lookahead, gain);
                        robot.getControl().waitPeriod(t_start);
                    }

                    robot.getControl().servoStop();

                    gripper.flushResponse ();

                    gripper.home();

                    ballGrib = false;
                    robot.getControl().moveJ(homeQ);
                }
                
                // Detect key press and quit if 'q' is pressed

                int keyPressed = cv::waitKey(10);
                if (keyPressed == 'q' || keyPressed == 'Q')
                { // quit
                    std::cout << "Shutting down camera..." << std::endl;
                    camera.Close();
                    std::cout << "Camera successfully closed." << std::endl;
                    break;
                }
                // Tager billede af frame med koordinater til dokumentation

                else if (keyPressed == 'p' || keyPressed == 'P')
                {
                    std::string fileDest;
                    std::cin >> fileDest;
                    cv::imwrite("../docPhotos/" + fileDest + ".png", stream);
                    std::cout << "File created at destination: " << fileDest << std::endl;
                }
            }
            else
            {
                std::cout << "Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription() << std::endl;
            }
            frame++;
        }
    }
    catch (GenICam::GenericException &e)
    {
        // Error handling.
        std::cerr << "An exception occurred." << std::endl
                  << e.GetDescription() << std::endl;
        exitCode = 1;
        std::cerr << "An exception occured ";
    }

    return exitCode;
}
