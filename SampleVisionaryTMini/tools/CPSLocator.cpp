//25.08.28 CPSLocator.cpp 수정ver...
#include "CPSLocator.h"

#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/io/ply_io.h>

#include <algorithm>
#include <numeric>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <chrono>
#include <cmath>
#include <limits>

namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;

// ---- Tunables (relaxed for robustness) ----
static constexpr float ROI_X_Min = -2000.f, ROI_X_Max = 2000.f;
static constexpr float ROI_Y_Min = -1900.f, ROI_Y_Max = 2000.f;
static constexpr float ROI_Z_Min = 1500.f, ROI_Z_Max = 5000.f;

static constexpr float VOX_MM = 5.0f;
static constexpr int   SOR_K = 30;
static constexpr float SOR_STD = 1.0f;        // previously 1.5

static constexpr float CLUSTER_TOL = 120.0f;  // previously 60
static constexpr int   MIN_PTS = 30;      // previously 50

static inline bool file_exists(const std::string& p) { return fs::exists(p); }

// ==================== Save utils ====================
void saveTailPointsToPLY(const std::string& filename, const std::vector<cv::Point3f>& points) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.reserve(points.size());
    for (const auto& pt : points) cloud.push_back(pcl::PointXYZ(pt.x, pt.y, pt.z));
    pcl::io::savePLYFileBinary(filename, cloud);
}
void saveTailPointsToPLY(const std::string& filename, const pcl::PointCloud<pcl::PointXYZ>& cloud) {
    pcl::io::savePLYFileBinary(filename, cloud);
}
void saveRawCloudToPLY(const std::string& filename, const pcl::PointCloud<pcl::PointXYZ>& cloud) {
    pcl::io::savePLYFileBinary(filename, cloud);
}

// ==================== CSV append ====================
void appendCPSResultToCSV(const std::string& csvPath,
    const std::string& frameName,
    const std::vector<int>& clusterCounts,
    const std::vector<cv::Point3f>& tailPoints,
    long long processing_ms)
{
    bool needHeader = !file_exists(csvPath);
    std::ofstream ofs(csvPath, std::ios::app);
    if (!ofs.is_open()) return;

    if (needHeader) ofs << "frame,clusters0,clusters1,tail_x,tail_y,tail_z,processing_ms\n";

    float tx = NAN, ty = NAN, tz = NAN;
    if (!tailPoints.empty()) {
        auto it = std::min_element(tailPoints.begin(), tailPoints.end(),
            [](const auto& a, const auto& b) { return a.z < b.z; });
        tx = it->x; ty = it->y; tz = it->z;
    }
    int c0 = (clusterCounts.size() >= 1) ? clusterCounts[0] : 0;
    int c1 = (clusterCounts.size() >= 2) ? clusterCounts[1] : 0;

    ofs << frameName << ',' << c0 << ',' << c1 << ','
        << tx << ',' << ty << ',' << tz << ','
        << processing_ms << "\n";
}

// ==================== Filtering / Denoising ====================
pcl::PointCloud<pcl::PointXYZ>::Ptr removeNoise(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& inputCloud,
    float x_min, float x_max,
    float y_min, float y_max,
    float z_min, float z_max,
    const std::string& debug_filename)
{
    if (!inputCloud || inputCloud->empty())
        return std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(
        new pcl::PointCloud<pcl::PointXYZ>(*inputCloud));

    pcl::PassThrough<pcl::PointXYZ> pass;

    pass.setInputCloud(cloud); pass.setFilterFieldName("x");
    pass.setFilterLimits(x_min, x_max); pass.filter(*cloud);

    pass.setInputCloud(cloud); pass.setFilterFieldName("y");
    pass.setFilterLimits(y_min, y_max); pass.filter(*cloud);

    pass.setInputCloud(cloud); pass.setFilterFieldName("z");
    pass.setFilterLimits(z_min, z_max); pass.filter(*cloud);




    // Voxel downsample
    {
        pcl::VoxelGrid<pcl::PointXYZ> vox; vox.setInputCloud(cloud);
        vox.setLeafSize(VOX_MM, VOX_MM, VOX_MM); vox.filter(*cloud);
    }
    // Statistical outlier removal
    {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud);
        sor.setMeanK(SOR_K);
        sor.setStddevMulThresh(SOR_STD);
        sor.filter(*cloud);
    }

    if (!debug_filename.empty())
        pcl::io::savePLYFileBinary(debug_filename, *cloud);

    return cloud;
}

// ==================== Tailpoint from clusters ====================
static cv::Point3f tailFromCluster_ZPercentile_YMeanNearest(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster,
    float percentile = 0.05f,
    const std::string& debug_slice_filename = "")
{
    if (!cluster || cluster->empty()) return { -9999, -9999, -9999 };

    std::vector<float> zs; zs.reserve(cluster->size());
    for (const auto& p : cluster->points) zs.push_back(p.z);

    size_t k = (size_t)std::clamp((int)std::floor(zs.size() * percentile), 0, (int)zs.size() - 1);
    std::nth_element(zs.begin(), zs.begin() + k, zs.end());
    float z_thresh = zs[k];

    auto sliced = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    sliced->reserve(cluster->size());
    for (const auto& p : cluster->points) if (p.z <= z_thresh) sliced->push_back(p);

    if (sliced->empty()) return { -9999, -9999, -9999 };
    if (!debug_slice_filename.empty())
        pcl::io::savePLYFileBinary(debug_slice_filename, *sliced);

    float my = 0.f; for (const auto& p : sliced->points) my += p.y; my /= sliced->size();

    float best = 1e9f; pcl::PointXYZ bp;
    for (const auto& p : sliced->points) {
        float d = std::fabs(p.y - my);
        if (d < best) { best = d; bp = p; }
    }
    return { bp.x, bp.y, bp.z };
}

std::vector<cv::Point3f> detectChassisTailCenters(
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
    float cluster_tolerance_mm,
    int min_points,
    float z_slice_percent,
    const std::string& debug_base,
    std::vector<int>* cluster_counts)
{
    std::vector<cv::Point3f> results;
    if (cluster_counts) cluster_counts->clear();
    if (!cloud || cloud->empty()) return results;

    // Euclidean clustering
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusters_idx;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance_mm);
    ec.setMinClusterSize(min_points);
    ec.setMaxClusterSize(250000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusters_idx);

    if (clusters_idx.empty()) return results;

    struct CI { int n; pcl::PointIndices idx; };
    std::vector<CI> cs; cs.reserve(clusters_idx.size());
    for (auto& id : clusters_idx) cs.push_back({ (int)id.indices.size(), id });
    std::sort(cs.begin(), cs.end(), [](const CI& a, const CI& b) { return a.n > b.n; });
    if (cs.size() > 2) cs.resize(2);

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
    clusters.reserve(cs.size());
    for (size_t i = 0; i < cs.size(); ++i) {
        auto cl = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        cl->reserve(cs[i].n);
        for (int id : cs[i].idx.indices) cl->push_back(cloud->points[(size_t)id]);
        if (cluster_counts) cluster_counts->push_back((int)cl->size());
        clusters.push_back(cl);
        if (!debug_base.empty())
            pcl::io::savePLYFileBinary(debug_base + "_cluster" + std::to_string(i) + ".ply", *cl);
    }

    for (size_t i = 0; i < clusters.size(); ++i) {
        auto tail = tailFromCluster_ZPercentile_YMeanNearest(
            clusters[i], std::clamp(z_slice_percent, 0.01f, 0.20f),
            debug_base.empty() ? "" : (debug_base + "_cluster" + std::to_string(i) + "_zsliced.ply"));
        if (tail.z > -9000.f) results.push_back(tail);
    }

    // Nearer first
    std::sort(results.begin(), results.end(), [](const auto& a, const auto& b) { return a.z < b.z; });
    if (results.size() > 2) results.resize(2);
    return results;
}

// ==================== Main pipeline ====================
CPSFrameResult processCPSFrame(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& rawCloud,
    const std::string& baseDir,
    const std::string& frameName,
    const std::string& csvPath,
    bool save)
{
    auto t0 = Clock::now();
    CPSFrameResult result;
    result.clusterCounts = { 0,0 };

    if (!rawCloud || rawCloud->empty()) {
        result.processing_ms = 0;
        return result;
    }

    // --- 0) Auto unit scaling
    pcl::PointCloud<pcl::PointXYZ>::Ptr src(new pcl::PointCloud<pcl::PointXYZ>());
    src->reserve(rawCloud->size());
    float zmin = 1e9f, zmax = -1e9f;
    for (const auto& p : rawCloud->points) { zmin = std::min(zmin, p.z); zmax = std::max(zmax, p.z); }
    const float scale = (zmax < 50.f ? 1000.f : 1.f); // threshold 50 is safe (in mm it's always >>50)
    for (const auto& p : rawCloud->points) src->push_back(pcl::PointXYZ(p.x * scale, p.y * scale, p.z * scale));

    // --- 1) ROI + denoise
    std::string debugFiltered = (save && !baseDir.empty() && !frameName.empty())
        ? (baseDir + "/" + frameName + "_filtered.ply") : "";
    auto filtered = removeNoise(src, ROI_X_Min, ROI_X_Max, ROI_Y_Min, ROI_Y_Max, ROI_Z_Min, ROI_Z_Max, debugFiltered);

    // --- 2) Cluster & tail detection
    std::string debugBase = (save && !baseDir.empty() && !frameName.empty())
        ? (baseDir + "/" + frameName) : "";
    auto tails = detectChassisTailCenters(filtered, CLUSTER_TOL, MIN_PTS, 0.05f, debugBase, &result.clusterCounts);

    // --- 3) Fallback...  if detection fails, return at least min-Z point from filtered cloud (avoid Z=-1)
    if (tails.empty() && filtered && !filtered->empty()) {
        float bestZ = 1e9f; pcl::PointXYZ bestP;
        for (const auto& q : filtered->points) { if (q.z < bestZ) { bestZ = q.z; bestP = q; } }
        tails.push_back(cv::Point3f(bestP.x, bestP.y, bestP.z));
    }

    // Nearer first, cap to 2
    std::sort(tails.begin(), tails.end(), [](const auto& a, const auto& b) { return a.z < b.z; });
    if (tails.size() > 2) tails.resize(2);
    result.tailPoints = tails;

    result.processing_ms = (long long)std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - t0).count();

    // --- 4) Optional saves
    if (save && !baseDir.empty() && !frameName.empty()) {
        fs::create_directories(baseDir);
        saveRawCloudToPLY(baseDir + "/" + frameName + "_raw.ply", *src); // save autoscaled cloud
        if (!result.tailPoints.empty())
            saveTailPointsToPLY(baseDir + "/" + frameName + "_tail.ply", result.tailPoints);
        appendCPSResultToCSV(csvPath, frameName, result.clusterCounts, result.tailPoints, result.processing_ms);
    }

    return result;
}
