#include "mine_detector/mine_detector.hpp"
#include <iostream>
#include <vector>
#include <string>

cv::Point2f MineDetector::computeCentroid(const std::vector<candidateContour>& candidates) {
    cv::Point2f center;
    for(auto& candidate : candidates) {
        center.x += candidate.corner.x;
        center.y += candidate.corner.y;
    }
    center.x /= candidates.size();
    center.y /= candidates.size();
    return center;
}

bool MineDetector::compareByRelativeAngle(const candidateContour& a, const candidateContour& b, const cv::Point2f& center) {
    float angleA = std::atan2(a.corner.y - center.y, a.corner.x - center.x);
    float angleB = std::atan2(b.corner.y - center.y, b.corner.x - center.x);
    return angleA < angleB;
}

void MineDetector::getImage(cv::Mat& image) {
    this->source = image;
    cv::Mat grayImage;
    cv::cvtColor(this->source, grayImage, cv::COLOR_BGR2GRAY);
    cv::Mat blurredImage;
    cv::GaussianBlur(grayImage, blurredImage, cv::Size(5,5), 0);
    cv::Mat thresholdImage;
    cv::adaptiveThreshold(blurredImage, thresholdImage, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 5, 2);
    this->binaryImage = thresholdImage;
}

void MineDetector::selectContours() {
    double minArea = 100,maxArea = 400;
    double maxRatio = 4.5;
    double minAreaRatio = 0.1,maxAreaRatio = 10;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<candidateContour> candidateContours;
    cv::findContours(this->binaryImage, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for(auto& contour : contours) {
        double area = cv::contourArea(contour);
        std::vector<std::vector<cv::Point>> contour_ = {contour};
        cv::drawContours(source, contour_, 0, cv::Scalar(0,255,0), 2);
        cv::Moments moments = cv::moments(contour);
        int centerX = static_cast<int>(moments.m10/moments.m00);
        int centerY = static_cast<int>(moments.m01/moments.m00);
        std::string areaText = "Area: " + std::to_string(area);
        cv::putText(source,areaText,cv::Point(centerX,centerY),cv::FONT_HERSHEY_SIMPLEX,0.4,cv::Scalar(0,0,255),1);
        if(area < minArea || area > maxArea) 
            continue;
        cv::RotatedRect boundary = cv::minAreaRect(contour);
        double boundArea = boundary.size.height * boundary.size.width;
        std::vector<cv::Point> approx;
        double epsilon = 0.02 * cv::arcLength(contour,true);
        cv::approxPolyDP(contour, approx, epsilon, true);

        if(approx.size() <= 5 || approx.size() >= 13) 
            continue;

        double widthHeightRatio = static_cast<double>(boundary.size.width) / boundary.size.height;
        double heightWidthRatio = static_cast<double>(boundary.size.height) / boundary.size.width;
        if(widthHeightRatio > maxRatio || heightWidthRatio > maxRatio) 
            continue;
        cv::Point2f center;
        float radius;
        cv::minEnclosingCircle(contour,center,radius);
        candidateContour currentContour(contour);
        currentContour.area = area;
        currentContour.corner = center;
        candidateContours.push_back(currentContour);
    }

    if(candidateContours.size() < 4)
        return;
    cv::Point2f center = computeCentroid(candidateContours);
    this->center = center;
    std::sort(candidateContours.begin(),candidateContours.end(),[&](const candidateContour &a, const candidateContour &b) {
        return compareByRelativeAngle(a,b,center);
    });
    int contourNum = candidateContours.size();
    UnionFind UF(contourNum);
    for(int i = 0;i < contourNum;i += 1) {
        for(int j = i + 1;j < contourNum;j += 1) {
            double areaRatio = candidateContours[i].area / candidateContours[j].area;
            if(areaRatio > minAreaRatio && areaRatio < maxAreaRatio)
                UF.unite(i,j);
        }
    }
    std::vector<int> setSizes(contourNum,0);
    for(int i = 0;i < contourNum;i += 1) {
        setSizes[UF.find(i)] += 1;
    }
    int root = -1;
    for(int i = 0;i < contourNum;i += 1) {
        if(setSizes[i] >= 4) {
            root = i;
            break;
        }
    }
    if(root == -1) {
        std::cerr << "No suitable Area(MINE) found!" << std::endl;
        return;
    }
    found = true;
    for(int i = 0;i < contourNum;i += 1) {
        if(UF.same(i,root)) {
            corners.push_back(candidateContours[i]);
        }
    }
}

void MineDetector::getCorners() {
    
}

void MineDetector::show() {
    cv::imshow("Mine Detector", this->binaryImage);
    cv::waitKey(1);
}