#pragma once

#include <pcl/pcl_base.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>

#include <vector>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <mutex>
#include <atomic>

/**
 * @brief Chassis Position Sensing (CPS) class for detecting truck chassis position
 *
 * This class provides functionality to detect the distance from a SICK Visionary T-Mini
 * ToF sensor to a truck chassis tail end. It handles different chassis shapes and is
 * robust against noise from reflective panels.
 *
 * Features efficient single-pass filtering on X, Y, and Z axes.
 */
class CPS {
public:
    /**
     * @brief Constructor with configuration parameters
     *
     * @param minX Minimum X value to consider (mm)
     * @param maxX Maximum X value to consider (mm)
     * @param minY Minimum Y value to consider (mm)
     * @param maxY Maximum Y value to consider (mm)
     * @param minZ Minimum Z value to consider (mm)
     * @param maxZ Maximum Z value to consider (mm)
     * @param historySize Size of history buffer for temporal filtering
     * @param maxJump Maximum allowed jump between consecutive measurements (mm)
     */
    CPS(
        float minX = -std::numeric_limits<float>::max(), // No limit by default
        float maxX = std::numeric_limits<float>::max(),  // No limit by default
        float minY = 300.0f,                            // Min chassis height from ground
        float maxY = std::numeric_limits<float>::max(),  // No upper limit by default
        float minZ = 500.0f,                            // Min distance (0.5 meters)
        float maxZ = 10000.0f,                          // Max distance (10 meters)
        float voxelSize = 30.0f,
        int historySize = 5,                            // Number of frames for temporal filtering
        float maxJump = 200.0f                          // Max allowed jump between measurements
    );

    /**
     * @brief Reset the detection state
     *
     * Clears history and resets internal state
     */
    void reset();

    /**
     * @brief Detects the distance from sensor to truck chassis tail end
     *
     * Uses efficient single-pass filtering on all three axes
     *
     * @param cloud Input point cloud (in mm) from SICK Visionary T-Mini
     * @return Distance in mm from sensor to chassis tail end, -1 if no chassis detected
     */
    float detectChassisDistance(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    /**
     * @brief Get the last valid distance detected
     *
     * @return Last valid distance in mm, -1 if no valid distance has been detected
     */
    float getLastValidDistance() const;

    /**
     * @brief Set the X axis filtering limits
     *
     * @param minX Minimum X value to consider (mm)
     * @param maxX Maximum X value to consider (mm)
     */
    void setXLimits(float minX, float maxX);

    /**
     * @brief Set the Y axis filtering limits
     *
     * @param minY Minimum Y value to consider (mm)
     * @param maxY Maximum Y value to consider (mm)
     */
    void setYLimits(float minY, float maxY);

    /**
     * @brief Set the Z axis filtering limits
     *
     * @param minZ Minimum Z value to consider (mm)
     * @param maxZ Maximum Z value to consider (mm)
     */
    void setZLimits(float minZ, float maxZ);

    /**
     * @brief Set the history size for temporal filtering
     *
     * @param historySize Number of frames to keep in history
     */
    void setHistorySize(int historySize);

    /**
     * @brief Set the maximum allowed jump between consecutive measurements
     *
     * @param maxJump Maximum jump in mm
     */
    void setMaxJump(float maxJump);

    /**
     * @brief Setting all necessary parameters
     *
     * @param all params.
     */
    void CPS::setParams(
        float minX,
        float maxX,
        float minY,
        float maxY,
        float minZ,
        float maxZ,
        float voxelSize,
        int historySize,
        float maxJump
    );

    /**
     * @brief Get the filtered point cloud from the last detection
     *
     * @return Filtered point cloud
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr getFilteredCloud() const;

    pcl::PointCloud<pcl::PointXYZ>::Ptr getFilteredCloud2() const;

    /**
     * @brief Get the voxelized point cloud from the last detection
     *
     * @return Voxelized point cloud
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr getVoxelizedCloud() const;

private:
    // Configuration parameters for filtering
    float m_minX;
    float m_maxX;
    float m_minY;
    float m_maxY;
    float m_minZ;
    float m_maxZ;
    float m_voxelSize;

    // Parameters for temporal filtering
    int m_historySize;
    float m_maxJump;

    // State variables
    std::vector<float> m_distanceHistory;
    float m_lastValidDistance;
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_filteredCloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_filteredCloud2; //Second-stage filter.
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_voxelizedCloud;

    // Constants for chassis detection
    const float m_sliceWidth = 100.0f; // Width of Z slices in mm
    const int m_minPointsPerSlice = 3; // Minimum points needed for a valid slice
    const float m_minSpatialExtent = 50.0f; // Minimum height or width for a valid chassis surface
    const float m_minDensity = 0.00005f; // Minimum point density for a valid surface

    int get_valid_min_z_binary_search(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input);
    pcl::PointCloud<pcl::PointXYZ>::Ptr singlePassFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr input, float minX, float maxX, float minY, float maxY, float minZ, float maxZ);

    int get_valid_min_z_fixed_buckets(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
        int minZ = 300,
        int slice_step = 50,
        int min_points_per_slice = 30
    );

    int get_valid_min_z_comprehensive(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
        int minZ = 300,
        int slice_step = 50,
        int min_points_per_slice = 30,
        float min_x_spread = 200.0f,
        float min_y_spread = 100.0f,
        float min_score_threshold = 50.0f
    );

};