#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <vector>
#include <algorithm>
#include <numeric>
#include <cmath>

/**
 * @brief Chassis Position Sensing (CPS) class for detecting truck chassis position
 *
 * This class provides functionality to detect the distance from a SICK Visionary T-Mini
 * ToF sensor to a truck chassis tail end. It handles different chassis shapes and is
 * robust against noise from reflective panels.
 */
class CPS {
public:
    /**
     * @brief Constructor with configuration parameters
     *
     * @param maxDistance Maximum expected distance to consider (mm)
     * @param minDistance Minimum expected distance to consider (mm)
     * @param groundHeight Expected height of ground plane (mm)
     * @param minChassisHeight Minimum expected chassis height from ground (mm)
     * @param historySize Size of history buffer for temporal filtering
     * @param maxJump Maximum allowed jump between consecutive measurements (mm)
     */
    CPS(
        float maxDistance = 10000.0f,    // 10 meters
        float minDistance = 500.0f,      // 0.5 meters
        float groundHeight = -400.0f,    // Assuming sensor is mounted above ground
        float minChassisHeight = 300.0f, // Min height of chassis from ground
        int historySize = 5,             // Number of frames for temporal filtering
        float maxJump = 200.0f           // Max allowed jump between measurements
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
     * @brief Set the maximum distance to consider
     *
     * @param maxDistance Maximum distance in mm
     */
    void setMaxDistance(float maxDistance);

    /**
     * @brief Set the minimum distance to consider
     *
     * @param minDistance Minimum distance in mm
     */
    void setMinDistance(float minDistance);

    /**
     * @brief Set the ground height
     *
     * @param groundHeight Ground height in mm
     */
    void setGroundHeight(float groundHeight);

    /**
     * @brief Set the minimum chassis height
     *
     * @param minChassisHeight Minimum chassis height in mm
     */
    void setMinChassisHeight(float minChassisHeight);

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
     * @brief Set class parameters in total.
     *
     * @param max, min distance, ground height, minChassisHeight, history size, max jum.
     */
    void setParams(float maxDistance,
        float minDistance,
        float groundHeight,
        float minChassisHeight,
        int historySize,
        float maxJump);

    /**
     * @brief Get the filtered point cloud from the last detection
     *
     * @return Filtered point cloud
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr getFilteredCloud() const;

private:
    // Configuration parameters
    float m_maxDistance;
    float m_minDistance;
    float m_groundHeight;
    float m_minChassisHeight;
    int m_historySize;
    float m_maxJump;

    // State variables
    std::vector<float> m_distanceHistory;
    float m_lastValidDistance;
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_filteredCloud;

    // Constants
    const int m_xBins = 10;  // Horizontal bins
    const int m_yBins = 8;   // Vertical bins
    const float m_consensusThreshold = 100.0f; // 100mm
    const int m_minInliers = 3;
    const int m_minPointsPerBin = 5;
};