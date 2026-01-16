#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core.hpp>
#include <vector>
#include <string>
#include <limits>

struct CPSFrameResult { //샤시 2대 일 떄 동시 판단을 위해서 2개까지 ..
    std::vector<int>         clusterCounts;   // up to 2 clusters (point counts) 
    std::vector<cv::Point3f> tailPoints;      // up to 2 tailpoints (nearest first)
    long long                processing_ms = 0;
};

//파이프라인 구축 부분 ...
// One-shot pipeline: ROI / denoise / cluster / tail detect -> (optional) save
CPSFrameResult processCPSFrame(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& rawCloud,
    const std::string& baseDir,     // session folder 
    const std::string& frameName,   // frame stem 
    const std::string& csvPath,     // CSV append path 
    bool save = true);

// Exposed building blocks (kept for compatibility / unit tests)
pcl::PointCloud<pcl::PointXYZ>::Ptr removeNoise(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& inputCloud,
    float x_min, float x_max,
    float y_min, float y_max,
    float z_min, float z_max,
    const std::string& debug_filename = "");

std::vector<cv::Point3f> detectChassisTailCenters(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    float cluster_tolerance_mm,
    int min_points,
    float z_slice_percent, // 0.0~1.0 =>하위 5프로로 설정
    const std::string& debug_base = "",
    std::vector<int>* cluster_counts = nullptr);

// Save utilities
void saveTailPointsToPLY(const std::string& filename, const std::vector<cv::Point3f>& points);
void saveTailPointsToPLY(const std::string& filename, const pcl::PointCloud<pcl::PointXYZ>& cloud);
void saveRawCloudToPLY(const std::string& filename, const pcl::PointCloud<pcl::PointXYZ>& cloud);

// CSV append (frame, clusters0, clusters1, tail_x, tail_y, tail_z, processing_ms)
void appendCPSResultToCSV(
    const std::string& csvPath,
    const std::string& frameName,
    const std::vector<int>& clusterCounts,
    const std::vector<cv::Point3f>& tailPoints,
    long long processing_ms);
