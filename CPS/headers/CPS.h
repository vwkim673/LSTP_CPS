#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
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
        int historySize,
        float maxJump
    );

    /**
     * @brief Get the filtered point cloud from the last detection
     *
     * @return Filtered point cloud
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr getFilteredCloud() const;

private:
    // Configuration parameters for filtering
    float m_minX;
    float m_maxX;
    float m_minY;
    float m_maxY;
    float m_minZ;
    float m_maxZ;

    // Parameters for temporal filtering
    int m_historySize;
    float m_maxJump;

    // State variables
    std::vector<float> m_distanceHistory;
    float m_lastValidDistance;
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_filteredCloud;

    // Constants for chassis detection
    const int m_xBins = 10;  // Horizontal bins
    const int m_yBins = 8;   // Vertical bins
    const float m_consensusThreshold = 100.0f; // 100mm
    const int m_minInliers = 3;
    const int m_minPointsPerBin = 5;
};