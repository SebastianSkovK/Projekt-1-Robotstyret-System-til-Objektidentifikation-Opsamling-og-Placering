#include <vector>
#include "opencv2/core.hpp"
#include <iostream>
#include <string>
#include "opencv2/opencv.hpp"
#include "cornerPoints.h"

void calibrateCamera()
{
    // Gennemgår filer og samler i en vector af strings
    std::vector<cv::String> fileNames;
    cv::glob("../caliPic/frame*.png", fileNames, false);
    // Størrelsen på kalibrationsboardet, hvor algoritmen kigger på de indre hjørner, dermed trækkes en fra i x og y
    cv::Size patternSize(10 - 1, 7 - 1);
    // billed container til billeder hvor man kigger after hjørner til calibrering
    std::vector<std::vector<cv::Point2f>> objectContainer(fileNames.size());
    // Container til fundne hjørne og deres pixel position
    std::vector<std::vector<cv::Point3f>> imageCorners;
    // Danner et fiktivt board med samme størrelse som det rigtige
    int checkerBoard[2] = {10, 7};
    int fieldSize = 5;
    // Definer world koordinater i 3D
    std::vector<cv::Point3f> objp;
    for (int i = 1; i < checkerBoard[1]; i++)
    {
        for (int j = 1; j < checkerBoard[0]; j++)
            objp.push_back(cv::Point3f(j * fieldSize, i * fieldSize, 0));
    }
    std::size_t i = 0;
    for (auto const &f : fileNames)
    {
        std::cout << std::string(f) << std::endl;
        cv::Mat img = cv::imread(fileNames[i]);
        cv::Mat gray;

        cv::cvtColor(img, gray, cv::COLOR_RGB2GRAY);
        bool patternFound = cv::findChessboardCorners(gray, patternSize, objectContainer[i], cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);
        // indsætter vores fiktive board i imagecorners hvis vores algoritme finder et hjørne
        if (patternFound)
        {
            cv::cornerSubPix(gray, objectContainer[i], cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.01));
            imageCorners.push_back(objp);
        }
        
        // Tegner hjørnene og de tilhørende strenge for visuelt feedback
        cv::drawChessboardCorners(img, patternSize, objectContainer[i], patternFound);
        //cv::imshow(fileNames[i], img);
        //cv::waitKey(0);
        cv::destroyAllWindows();
        i++;
    }
    // Kamera kalibreringsdelen

    cv::Matx33f instrinsicM(cv::Matx33f::eye()); // Kameraets egenskaber (intrinsic matrix)
    cv::Vec<float, 5> koeff(0, 0, 0, 0, 0);      // distortion coeff
    std::vector<cv::Mat> rMatrix, tMatrix;       // Rotation og translation matrix
    std::vector<double> intrinsics, extrinsics, perViewErrors;
    //int flags =
        //cv::CALIB_FIX_ASPECT_RATIO + cv::CALIB_FIX_K3 +
        //cv::CALIB_ZERO_TANGENT_DIST + cv::CALIB_FIX_PRINCIPAL_POINT;

    std::cout << "Calibrating.." << std::endl;
    cv::Size frameSize(1080, 1080);
    // Tjekker den numeriske størrelse af usikkerheden på kalibreringen
    float error = cv::calibrateCamera(imageCorners, objectContainer, frameSize, instrinsicM, koeff, rMatrix,tMatrix,intrinsics,extrinsics,perViewErrors);
    // Gemmer koeff og intrinsic matrix i en fil
    cv::FileStorage fs("../calibration.yaml", cv::FileStorage::WRITE);
    fs << "ins" << instrinsicM;
    fs << "Dis" << koeff;
    fs << "error" << error;
    fs.release();
    
}

bool calibrateCorners (cv::Mat frame, bool cups, std::vector<int>& points)
{
    if (cornerPoints(frame, cups, points))
    {
        std::cout << "Bold koordinater gemt" << std::endl;
        for(int i = 0; i < points.size(); i++)
        {
            char type = 'x';
            if(i % 2 != 0)
            {
                type = 'y';
            }
            std::cout << points[i] << type << std::endl;
        }

        return true;
    }

    return false;
}