#include "exchangestation_detector/exchangestation_detector.hpp"
//use yaml-cpp to read yaml files
#include <yaml-cpp/yaml.h>
using namespace cv;
// helper method
cv::Point2f ExchangeStationDetector::computeCentroid(const std::vector<candidateContour>& candidates) {
    cv::Point2f center(0, 0);
    for (const candidateContour& candidate : candidates) {
        center.x += candidate.corner.x;
        center.y += candidate.corner.y;
    }
    center.x /= candidates.size();
    center.y /= candidates.size();
    return center;
}

//helper method
bool ExchangeStationDetector::compareByRelativeAngle(const candidateContour& a, const candidateContour& b, const cv::Point2f& center) {
    float angleA = std::atan2(a.corner.y - center.y, a.corner.x - center.x);
    float angleB = std::atan2(b.corner.y - center.y, b.corner.x - center.x);
    return angleA < angleB;
}


ExchangeStationDetector::ExchangeStationDetector() {
    DistortionCoeff = (cv::Mat_<double>(1, 5) << 0.06055295456945162, -4.14124115366316, -0.00243068226202482, -0.004122633276866059, 37.04189535760245);
    CameraMatrix = (cv::Mat_<double>(3, 3) << 2385.27508341373, 0, 706.3992740188233, 0, 2388.628439590787, 548.9891287751039, 0, 0, 1);
    lastFrameCorners.push_back(cv::Point(100, 100));
    lastFrameCorners.emplace_back(cv::Point(100, -100));
    lastFrameCorners.emplace_back(cv::Point(-100, -100));
    lastFrameCorners.emplace_back(cv::Point(-100, 100));
    currentFrameCorners = lastFrameCorners;
}

ExchangeStationDetector::ExchangeStationDetector(int blueThreshold,int redThreshold,int detectColor) {
  /*
    try {
        YAML::Node config = YAML::LoadFile("/home/jlurobovision/eng_ws/src/exchangestation_detector/config/camera.yaml");

        if (config["distortion_coeff"]) {
            const YAML::Node& distortionCoeffNode = config["distortion_coeff"];
            std::vector<double> distortionCoeffData;
            for (const auto& val : distortionCoeffNode) {
                distortionCoeffData.push_back(val.as<double>());
            }
            for(auto& val : distortionCoeffData)
                std::cout << val << std::endl;
            DistortionCoeff = cv::Mat(distortionCoeffData);
        } else {
            std::cout << "distortion_coeff not found in the YAML file." << std::endl;
        }

        if (config["camera_matrix"]["data"]) {
            const YAML::Node& cameraMatrixNode = config["camera_matrix"]["data"];
            std::vector<double> cameraMatrixData;
            for (const auto& val : cameraMatrixNode) {
                cameraMatrixData.push_back(val.as<double>());
            }
            CameraMatrix = cv::Mat(cameraMatrixData).reshape(1, 3); // Assuming camera matrix is 3x3
        } else {
            std::cout << "camera_matrix data not found in the YAML file." << std::endl;
        }
    } catch (const std::exception& e) {
        std::cout << "Error: " << e.what() << std::endl;
    }
        //print CameraMatrix for debug
    std::cout << "CameraMatrix:" << CameraMatrix << std::endl;
    //print DistortionCoeff for debug
    std::cout << "DistortionCoeff:" << DistortionCoeff << std::endl;
    */
    CameraMatrix = (cv::Mat_<double>(3, 3) << 1850.906255, 0.0000, 496.601643,
         0.0000, 1846.847240, 212.542900,
         0.0000, 0.0000, 1.0000);
    DistortionCoeff = (cv::Mat_<double>(1, 5) << -0.20730, -0.1343, 0.00000, 0.01260, 0.05390);
    this->redThreshold = redThreshold;
    this->blueThreshold = blueThreshold;
    this->detectColor = detectColor;
    lastFrameCorners.push_back(cv::Point(100, 100));
    lastFrameCorners.emplace_back(cv::Point(100, -100));
    lastFrameCorners.emplace_back(cv::Point(-100, -100));
    lastFrameCorners.emplace_back(cv::Point(-100, 100));
    currentFrameCorners = lastFrameCorners;
}
ExchangeStationDetector::ExchangeStationDetector(cv::Mat& source) {
    this->source = source;
    cv::FileStorage fs("config/camera.yaml", cv::FileStorage::READ);
    fs["DistortionCoeff"] >> DistortionCoeff;
    fs["CameraMatrix"] >> CameraMatrix;
    fs.release();

    cv::Mat binaryImg;
    std::vector<cv::Mat> channels;
    cv::split(source, channels);
    cv::Mat redChannel = channels[2];
    cv::Mat blueChannel = channels[0];
    cv::Mat mergedChannels = redChannel + blueChannel;
    cv::Mat binaryResult;
    cv::threshold(mergedChannels, binaryResult, 50, 255, cv::THRESH_BINARY);
    this->binaryImg = binaryResult;
}

void ExchangeStationDetector::getImage(cv::Mat& source) {
    lastFrameCorners = currentFrameCorners;
    this->source = source;
    cv::Mat grayImg;
    cv::Mat binaryResult;
    cv::cvtColor(source,grayImg,cv::COLOR_BGR2GRAY);
    //cv::threshold(grayImg, binaryResult, 50, 255, cv::THRESH_BINARY);
    //std::vector<cv::Mat> channels;
    //cv::split(source, channels);
    //cv::Mat redChannel = channels[2];
    //cv::Mat blueChannel = channels[0];
    //cv::Mat mergedChannels = redChannel + blueChannel;
    //cv::imshow("pro", mergedChannels);
    //cv::waitKey(0);
    //cv::Mat binaryResult;
    cv::threshold(grayImg, binaryResult, redThreshold, 255, cv::THRESH_BINARY);

    int windowWidth = 800;
    int windowHeight = 600;
    //cv::namedWindow("binaryResult", cv::WINDOW_NORMAL);
    //cv::resizeWindow("binaryResult", windowWidth, windowHeight);
    //cv::imshow("binaryResult",source);
    //cv::waitKey(1);
    this->binaryImg = binaryResult;
}




Packet ExchangeStationDetector::solveAngle()
{
    //print CameraMatrix for debug
    //std::cout << "CameraMatrix:" << CameraMatrix << std::endl;
    //print DistortionCoeff for debug
    //std::cout << "DistortionCoeff:" << DistortionCoeff << std::endl;
    if(found == false) {
        return Packet();
    }
    double size = 288;
    std::vector<cv::Point3f> ExStationPos = { cv::Point3f(size / 2, -size / 2,0),
                                              cv::Point3f(size / 2, size / 2,0),
                                              cv::Point3f(-size / 2, size / 2,0),
                                              cv::Point3f(-size / 2, -size / 2,0) };
    std::vector<cv::Point2f> realPos = { this->corners[0].corner,
                                         this->corners[1].corner, 
                                         this->corners[2].corner, 
                                         this->corners[3].corner };
    cv::Mat tVec, rVec;
    cv::solvePnP(ExStationPos, realPos, CameraMatrix, DistortionCoeff, rVec, tVec, false,cv::SOLVEPNP_IPPE_SQUARE);
    cv::Mat rotationVector;
    cv::Rodrigues(rVec, rotationVector);
    cv::Mat mtxR, mtxQ;
    cv::Vec3d eulerAngles = cv::RQDecomp3x3(rotationVector, mtxR, mtxQ, cv::noArray(), cv::noArray());
    std::cout << "EulerAngle(pitch,yaw,roll)" << eulerAngles << std::endl;
    Packet packet;
    packet.found = true;
    packet.pitch = eulerAngles[0];
    packet.yaw = eulerAngles[1];
    packet.roll = eulerAngles[2];
    packet.x = tVec.at<double>(0, 0);
    packet.y = tVec.at<double>(1, 0);
    packet.z = tVec.at<double>(2, 0);
    return packet;
    //print eularangle on the screen through imshow
    cv::putText(source, "pitch:" + std::to_string(eulerAngles[0]), cv::Point(20, 20), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255, 0, 0), 3);
    cv::putText(source, "yaw:" + std::to_string(eulerAngles[1]), cv::Point(20, 60), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255, 0, 0), 3);
    cv::putText(source, "roll:" + std::to_string(eulerAngles[2]), cv::Point(20, 160), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(255, 0, 0), 3);
    //solve quarternion and coordinates from rVec and tVec, alread have rVec and tVec
    //cv::Mat rotationMatrix;
    //cv::Rodrigues(rVec, rotationMatrix);
    //double Q[4];
    //getQuaternion(rotationMatrix, Q);
    //packet.quaternion1 = Q[0];
    //packet.quaternion2 = Q[1];
    //packet.quaternion3 = Q[2];
    //packet.quaternion4 = Q[3];
    //packet.x = tVec.at<double>(0, 0);
    //packet.y = tVec.at<double>(1, 0);
    //packet.z = tVec.at<double>(2, 0);    
    //return packet;
}

void getQuaternion(cv::Mat rotationMatrix, double Q[]) {
    double trace = rotationMatrix.at<double>(0, 0) + rotationMatrix.at<double>(1, 1) + rotationMatrix.at<double>(2, 2);
    if (trace > 0) {
        double s = sqrt(trace + 1.0);
        Q[3] = s * 0.5;
        s = 0.5 / s;
        Q[0] = (rotationMatrix.at<double>(2, 1) - rotationMatrix.at<double>(1, 2)) * s;
        Q[1] = (rotationMatrix.at<double>(0, 2) - rotationMatrix.at<double>(2, 0)) * s;
        Q[2] = (rotationMatrix.at<double>(1, 0) - rotationMatrix.at<double>(0, 1)) * s;
    }
    else {
      int i = rotationMatrix.at<double>(0, 0) < rotationMatrix.at<double>(1, 1) ?
        (rotationMatrix.at<double>(1, 1) < rotationMatrix.at<double>(2, 2) ? 2 : 1) :
        (rotationMatrix.at<double>(0, 0) < rotationMatrix.at<double>(2, 2) ? 2 : 0);
        int j = (i + 1) % 3;
        int k = (i + 2) % 3;
        double s = sqrt(rotationMatrix.at<double>(i, i) - rotationMatrix.at<double>(j, j) - rotationMatrix.at<double>(k, k) + 1.0);
        Q[i] = s * 0.5;
        s = 0.5 / s;
        Q[3] = (rotationMatrix.at<double>(k, j) - rotationMatrix.at<double>(j, k)) * s;
        Q[j] = (rotationMatrix.at<double>(j, i) + rotationMatrix.at<double>(i, j)) * s;
        Q[k] = (rotationMatrix.at<double>(k, i) + rotationMatrix.at<double>(i, k)) * s;
    }
}



void ExchangeStationDetector::selectContours() {
    found = false; //Reset state
    currentFrameSmallSquares.clear();
	  double minArea = 300, maxArea = 1000; // For a single contour
	  double maxRatio = 4.5; // For a single contour : width / height
    double minDis = 0, maxDis = 400; // Compare between contours
    double minAreaRatio = 0.1, maxAreaRatio = 10; // Compare between contours
	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(binaryImg, contours, cv::RETR_EXTERNAL,cv::CHAIN_APPROX_SIMPLE);// should be in counter clock-wise order according to the video
    std::vector<candidateContour> candidateContours; // Potential corner contours
    /*
    for (auto& candidate : contours) {
        std::vector<std::vector<cv::Point>> contour = { candidate };
        cv::drawContours(source, contour, 0, cv::Scalar(255, 0, 0), 2);
        cv::imshow("Polygons", source);
        cv::waitKey(500);
    }
    */
	for (const auto& contour : contours) {
		double area = cv::contourArea(contour);
        //std::cout << std::endl;

        //std::cout << "area:" << area << std::endl;

        
        std::vector<std::vector<cv::Point>> contour_ = { contour };
        cv::drawContours(source, contour_, 0, cv::Scalar(0, 255, 0), 2);

        cv::Moments moments = cv::moments(contour);
        int centerX = static_cast<int>(moments.m10 / moments.m00);
        int centerY = static_cast<int>(moments.m01 / moments.m00);

        std::string areaText = "Area: " + std::to_string(area);
        cv::putText(source, areaText, cv::Point(centerX, centerY), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
        

        cv::RotatedRect boundary = cv::minAreaRect(contour);
        //select two small Points
        double boundArea = boundary.size.height * boundary.size.width;
        if (area > 20 && area < 150 && boundArea < 2 * area && boundary.size.width / boundary.size.height < 2 && boundary.size.height / boundary.size.width < 2) {
            cv::Point2f center = boundary.center;
            if (currentFrameSmallSquares.size() < 2)
                currentFrameSmallSquares.push_back(boundary.center);
            continue;
        }
		//Area Judge
		if (area < minArea || area > maxArea)
			continue;
		//approxPolygon Judge
        std::vector<cv::Point> approx;
        double epsilon = 0.02 * cv::arcLength(contour, true);
        cv::approxPolyDP(contour, approx, epsilon, true);
        
        //cv::polylines(source, approx, true, cv::Scalar(255, 0, 0), 10);

        std::cout <<"approxSize:" << approx.size() << std::endl;
        
        if (approx.size() <= 5 || approx.size() >= 13) {
            continue;
        }
        //Ratio Judge
        
		double widthHeightRatio = static_cast<double>(boundary.size.width) / boundary.size.height;
        double heightWidthRatio = static_cast<double>(boundary.size.height) / boundary.size.width;
        
        //std::cout << "widthHeightRatio" << widthHeightRatio << std::endl;
        //std::cout << "heightWidthRatio" << heightWidthRatio << std::endl;

		if (widthHeightRatio > maxRatio || heightWidthRatio > maxRatio)
			continue; 

        candidateContour chosenContour(contour);
        cv::Point2f center;
        float radius;
        cv::minEnclosingCircle(contour, center, radius);
        chosenContour.corner = center;
        chosenContour.area = cv::contourArea(contour);
		candidateContours.push_back(chosenContour);
	}

    if (!currentFrameSmallSquares.empty())
        lastFrameSmallSquares = currentFrameSmallSquares;

    std::cout << "currentFrameSmallSquares: " << currentFrameSmallSquares.size() << std::endl;

    if (candidateContours.size() < 4) {
        std::cout << "Not enough corners" << std::endl; // Haven't found yet
        return;
    }

    cv::Point2f center = computeCentroid(candidateContours);
    this->center = center;
    std::sort(candidateContours.begin(), candidateContours.end(), [&](const candidateContour& a, const candidateContour& b) {
        return compareByRelativeAngle(a, b, center);
        });
    /*
    cv::Mat Source = source.clone();
    for (auto& candidate : candidateContours) {
        std::vector<std::vector<cv::Point>> contour = { candidate.contour };
        cv::drawContours(Source, contour, -1, cv::Scalar(255, 0, 0), 2);
        cv::imshow("Polygons", Source);
        cv::waitKey(500);
    }
    */
    


    int contourNum = candidateContours.size();
    

    //Find corner set
    UnionFind UF(contourNum);
    for (int i = 0; i < contourNum; i += 1) {
        for (int j = i + 1; j < contourNum; j += 1) {
            //double distance = cv::norm(candidateContours[i].corner - candidateContours[j].corner);
            double areaRatio = candidateContours[i].area / candidateContours[j].area;
            if (areaRatio > minAreaRatio && areaRatio < maxAreaRatio)
                UF.unite(i, j);
        }
    }
    // Find a set with 4 contours
    std::vector<int> setSizes(contourNum, 0);
    for (int i = 0; i < contourNum; i++) {
        setSizes[UF.find(i)]++;
    }
    int root = -1;
    for (int i = 0; i < contourNum; i++) {
        if (setSizes[i] == 4) {
            root = i;
            break;
        }
    }
    if (root == -1) {
        std::cerr << "No suitable Area Found!" << std::endl;
        return;
    }
    found = true;
    std::vector<candidateContour> resultCorners;
    for (int i = 0; i < contourNum; i++) {
        if (UF.same(i, root)) {
            resultCorners.push_back(candidateContours[i]);
        }     
    }
    this->corners = resultCorners;
}


void ExchangeStationDetector::getCorners() {
    if (found == false)
        return;
    std::cout << "Found suitable area" << std::endl;
    std::cout << this->corners.size() << std::endl;
    cv::circle(source, center, 10, cv::Scalar(0, 255, 0), -1);
    for (auto& corner : this->corners) {
        std::vector<cv::Point> approxTriangle;
        cv::minEnclosingTriangle(corner.contour, approxTriangle);
        
        //cv::polylines(source, approxTriangle, true, cv::Scalar(255, 0, 0), 10);
        
        /*
        std::vector<std::vector<cv::Point>> contours = { approxTriangle };
        cv::Mat Source = source.clone();
        cv::drawContours(Source, contours, 0, cv::Scalar(255, 0, 0), 2);
        cv::imshow("approxResult", Source);
        cv::waitKey(500);
        */
        
        double maxAngle = 0;
        cv::Point maxAnglePoint;
        for (int i = 0; i < 3; i += 1) {
            // iterate through 3 points and calculate the angles
            cv::Point a = approxTriangle[i];
            cv::Point b = approxTriangle[(i + 1) % 3];
            cv::Point c = approxTriangle[(i + 2) % 3];

            cv::Point ab = { a.x - b.x, a.y - b.y };
            cv::Point cb = { c.x - b.x, c.y - b.y };
            // tan�� = crossProduct / dotProduct
            double dotProduct = ab.x * cb.x + ab.y * cb.y;
            double crossProduct = ab.x * cb.y - ab.y * cb.x;
            double angle = std::atan(std::abs(crossProduct) / std::abs(dotProduct));
            if (angle > maxAngle) {
                maxAngle = angle;
                maxAnglePoint = b;
            }
        }
        corner.corner = maxAnglePoint;
        
        
        
        if (maxAngle * (180 / M_PI) > 130)
            corner.corner = maxAnglePoint;
        else {
            double maxDistance = 0;
            int maxDisIndex;
            for (int i = 0; i < 3; i++) {
                double distance = cv::norm(static_cast<cv::Point2f>(approxTriangle[i]) - this->center);
                if (distance > maxDistance) {
                    maxDistance = distance;
                    maxDisIndex = i;
                }
            }
            corner.corner = approxTriangle[maxDisIndex];
        }
        //cv::circle(source, corner.corner, 10, cv::Scalar(0, 255, 0), -1);
        
    }
    for (const auto& point : currentFrameSmallSquares) {
        cv::circle(source, point, 10, cv::Scalar(255, 0, 0), -1);
    }
    //cv::imshow("cornerPoints", source);
    
    // Use area to judge which one is the up right corner
    if (currentFrameSmallSquares.empty())
        currentFrameSmallSquares = lastFrameSmallSquares;
    double minDis = 1000000;
    auto minAreaContour = this->corners.begin();
        for (auto iterator = corners.begin(); iterator < corners.end(); iterator++) {
            double distanceFromSquares = 0;
            for (auto& point : currentFrameSmallSquares) {
                distanceFromSquares += cv::norm(point - iterator->corner);
            }
            if (distanceFromSquares < minDis) {
                minDis = distanceFromSquares;
                minAreaContour = iterator;
            }
        }
    
    cv::circle(source, minAreaContour->corner, 10, cv::Scalar(0, 255, 0), -1);
    std::rotate(this->corners.begin(), minAreaContour, this->corners.end());

    for (int i = 0; i < 4; i += 1) {
        currentFrameCorners[i] = 0.1 * corners[i].corner + 0.9 * static_cast<cv::Point2f>(lastFrameCorners[i]);
        std::cout << cv::contourArea(corners[i].contour) << std::endl;
        /*
        std::vector<std::vector<cv::Point>> contours = { iterator.contour };
        cv::drawContours(source, contours, -1, cv::Scalar(255, 0, 0), 2);
        cv::imshow("finalChoice", source);
        cv::waitKey(500);
        */
    }
    
    for (int i = 0; i < 4; i += 1) {
        cv::circle(source, corners[i].corner, 5, cv::Scalar(255, 0, 0), -1);
    }
    
}



void ExchangeStationDetector::show() {
    cv::polylines(source, currentFrameCorners, true, cv::Scalar(255, 0, 0), 10);
    cv::imshow("source",source);
    cv::waitKey(1);
}