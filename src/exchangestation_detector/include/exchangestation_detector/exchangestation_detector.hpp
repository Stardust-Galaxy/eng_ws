
/*
	Find corners of the exchange station box
*/

#ifndef EXSTATIONRECOG_H
#define EXSTATIONRECOG_H
#define RED 0
#define BLUE 1
#include <cmath>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <filesystem>

#include "../../serial/include/serial/packet.hpp"

class ExchangeStationDetector {
public:
	//Two ways of initialization. Choose/Delete one way if you see fit.
	ExchangeStationDetector();
	
	ExchangeStationDetector(int blueThreshold, int redThreshold,int detectColor = 0);

	void getImage(cv::Mat& binaryImg);
	
	//ExchangeStationDetector(cv::Mat& source);

	void selectContours();

	void getCorners();

	Packet solveAngle(float,float);

  void getQuaternion(cv::Mat rotationMatrix,double Q[]);
	
	void show();
private:
	struct candidateContour
	{
		std::vector<cv::Point> contour;
		double area = 0;
		cv::Point2f corner;
		candidateContour() = default;
		candidateContour(std::vector<cv::Point> contour) {
			this->contour = contour;
		}
	};

	int redThreshold = 160;
	int blueThreshold = 80;
	int detectColor = 0;

	int foundCount = 0;

	cv::Mat source;
	
	cv::Mat binaryImg;
	//State 
	bool found = false;
	//Four cornerPoints
	std::vector<candidateContour> corners;
	//Points from last frame
	std::vector<cv::Point> lastFrameCorners;
	//Points of current frame
	std::vector<cv::Point> currentFrameCorners;
	//Center of the four corner points
	cv::Point2f center;
	// Two small points
	std::vector<cv::Point2f> currentFrameSmallSquares;
	//Two small points in last frame
	std::vector<cv::Point2f> lastFrameSmallSquares;
	//Return the suitable set of contours
	double values[5] = {0.06055295456945162, -4.14124115366316, -0.00243068226202482, -0.004122633276866059, 37.04189535760245};

	cv::Mat DistortionCoeff;
	
	cv::Mat CameraMatrix;
	
	bool compareByRelativeAngle(const candidateContour& a, const candidateContour& b, const cv::Point2f& center);

	cv::Point2f computeCentroid(const std::vector<candidateContour>& candidates);

	class UnionFind {
	public:
		UnionFind(int size) {
			parent.resize(size);
			rank.resize(size, 0);
			for (int i = 0; i < size; ++i) {
				parent[i] = i;
			}
		}

		int find(int x) {
			if (parent[x] != x) {
				parent[x] = find(parent[x]);
			}
			return parent[x];
		}

		void unite(int x, int y) {
			int rootX = find(x);
			int rootY = find(y);
			if (rootX != rootY) {
				if (rank[rootX] < rank[rootY]) {
					parent[rootX] = rootY;
				}
				else if (rank[rootX] > rank[rootY]) {
					parent[rootY] = rootX;
				}
				else {
					parent[rootY] = rootX;
					rank[rootX]++;
				}
			}
		}

		bool same(int x, int y) {
			return find(x) == find(y);
		}
	private:
		std::vector<int> parent;
		std::vector<int> rank;
	};

};

#endif

