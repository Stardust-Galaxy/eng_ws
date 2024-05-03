#ifndef MINE_DETECTOR_
#define MINE_DETECTOR_

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

class MineDetector {
public:
    MineDetector(){}
    void getImage(cv::Mat& img);
    void selectContours();
    void getCorners();
    void show();
    ~MineDetector();
private:
    int threshold = 150;
    bool found = false;
    cv::Mat source;
    cv::Mat binaryImage;
    std::vector<std::vector<cv::Point>> contours;  
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
    bool compareByRelativeAngle(const candidateContour& a, const candidateContour& b, const cv::Point2f& center); 
    cv::Point2f computeCentroid(const std::vector<candidateContour>& candidates);
    std::vector<candidateContour> corners;
    cv::Point2f center;
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