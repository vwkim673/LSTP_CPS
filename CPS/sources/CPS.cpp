#include "CPS.h"
#include <iostream>

CPS::CPS(
    float minX,
    float maxX,
    float minY,
    float maxY,
    float minZ,
    float maxZ,
    int historySize,
    float maxJump
) :
    m_minX(minX),
    m_maxX(maxX),
    m_minY(minY),
    m_maxY(maxY),
    m_minZ(minZ),
    m_maxZ(maxZ),
    m_historySize(historySize),
    m_maxJump(maxJump),
    m_lastValidDistance(-1.0f)
{
    m_filteredCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
}

void CPS::reset() {
    m_distanceHistory.clear();
    m_lastValidDistance = -1.0f;
    m_filteredCloud->clear();
}

float CPS::detectChassisDistance(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    if (cloud->empty()) {
        return m_lastValidDistance;
    }

    // Step 1: Single-pass filtering on X, Y, and Z axes
    m_filteredCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& point : cloud->points) {
        // Efficient single-pass filtering on all three axes
        if (point.x >= m_minX && point.x <= m_maxX &&
            point.y >= m_minY && point.y <= m_maxY &&
            point.z >= m_minZ && point.z <= m_maxZ) {
            m_filteredCloud->points.push_back(point);
        }
    }

    if (m_filteredCloud->empty()) {
        return m_lastValidDistance;
    }

    m_filteredCloud->width = m_filteredCloud->points.size();
    m_filteredCloud->height = 1;
    m_filteredCloud->is_dense = false;

    // Step 2: Create a 2D histogram in the X-Y plane to identify the chassis
    // Get cloud dimensions
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(*m_filteredCloud, minPt, maxPt);

    float xRange = maxPt.x - minPt.x;
    float yRange = maxPt.y - minPt.y;

    // Avoid division by zero
    if (xRange < 1.0f) xRange = 1.0f;
    if (yRange < 1.0f) yRange = 1.0f;

    // Create histogram bins for Z values
    std::vector<std::vector<std::vector<float>>> histogram(m_xBins,
        std::vector<std::vector<float>>(m_yBins));

    // Fill histogram with z-values
    for (const auto& point : m_filteredCloud->points) {
        int xBin = std::min(m_xBins - 1,
            static_cast<int>((point.x - minPt.x) / xRange * m_xBins));
        int yBin = std::min(m_yBins - 1,
            static_cast<int>((point.y - minPt.y) / yRange * m_yBins));

        if (xBin >= 0 && xBin < m_xBins && yBin >= 0 && yBin < m_yBins) {
            histogram[xBin][yBin].push_back(point.z);
        }
    }

    // Step 3: Process each bin to find potential chassis points
    std::vector<float> binMedians;
    std::vector<int> binCounts;

    for (int x = 0; x < m_xBins; x++) {
        for (int y = 0; y < m_yBins; y++) {
            auto& bin = histogram[x][y];

            // Only consider bins with enough points (avoid noise)
            if (bin.size() >= m_minPointsPerBin) {
                std::sort(bin.begin(), bin.end());
                float median = bin[bin.size() / 2];
                binMedians.push_back(median);
                binCounts.push_back(bin.size());
            }
        }
    }

    if (binMedians.empty()) {
        return m_lastValidDistance;
    }

    // Step 4: Find the most consistent distance using RANSAC-like approach
    int bestInliers = 0;
    float bestDistance = -1.0f;

    // Sort by bin count (prioritize bins with more points)
    std::vector<size_t> indices(binMedians.size());
    std::iota(indices.begin(), indices.end(), 0);
    std::sort(indices.begin(), indices.end(),
        [&binCounts](size_t i1, size_t i2) { return binCounts[i1] > binCounts[i2]; });

    // Try each bin median as a candidate distance
    for (size_t i = 0; i < indices.size(); i++) {
        float candidateDistance = binMedians[indices[i]];
        int inliers = 0;
        float weightedSum = 0.0f;
        int weightedCount = 0;

        for (size_t j = 0; j < binMedians.size(); j++) {
            if (std::abs(binMedians[j] - candidateDistance) <= m_consensusThreshold) {
                inliers++;
                weightedSum += binMedians[j] * binCounts[j];
                weightedCount += binCounts[j];
            }
        }

        if (inliers > bestInliers) {
            bestInliers = inliers;
            bestDistance = (weightedCount > 0) ? (weightedSum / weightedCount) : candidateDistance;
        }
    }

    // Step 5: Apply temporal filtering
    float currentDistance = -1.0f;

    if (bestInliers >= m_minInliers) {
        currentDistance = bestDistance;

        // Apply temporal filtering if we have a previous valid distance
        if (m_lastValidDistance > 0 && std::abs(currentDistance - m_lastValidDistance) > m_maxJump) {
            // Large jump detected, use weighted average to smooth
            currentDistance = 0.7f * m_lastValidDistance + 0.3f * currentDistance;
        }

        // Update history
        m_distanceHistory.push_back(currentDistance);
        while (static_cast<int>(m_distanceHistory.size()) > m_historySize) {
            m_distanceHistory.erase(m_distanceHistory.begin());
        }

        // Calculate median of history for stable output
        if (!m_distanceHistory.empty()) {
            std::vector<float> sortedHistory = m_distanceHistory;
            std::sort(sortedHistory.begin(), sortedHistory.end());
            currentDistance = sortedHistory[sortedHistory.size() / 2];
        }

        m_lastValidDistance = currentDistance;
    }

    return currentDistance;
}

float CPS::getLastValidDistance() const {
    return m_lastValidDistance;
}

void CPS::setXLimits(float minX, float maxX) {
    m_minX = minX;
    m_maxX = maxX;
}

void CPS::setYLimits(float minY, float maxY) {
    m_minY = minY;
    m_maxY = maxY;
}

void CPS::setZLimits(float minZ, float maxZ) {
    m_minZ = minZ;
    m_maxZ = maxZ;
}

void CPS::setHistorySize(int historySize) {
    m_historySize = historySize;

    // Trim history if needed
    while (static_cast<int>(m_distanceHistory.size()) > m_historySize) {
        m_distanceHistory.erase(m_distanceHistory.begin());
    }
}

void CPS::setMaxJump(float maxJump) {
    m_maxJump = maxJump;
}

void CPS::setParams(
    float minX,
    float maxX,
    float minY,
    float maxY,
    float minZ,
    float maxZ,
    int historySize,
    float maxJump
)
{
    m_minX = minX;
    m_maxX = maxX;
    m_minY = minY;
    m_maxY = maxY;
    m_minZ = minZ;
    m_maxZ = maxZ;
    m_historySize = historySize;
    m_maxJump = maxJump;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CPS::getFilteredCloud() const {
    // Return a copy to ensure safety
    pcl::PointCloud<pcl::PointXYZ>::Ptr copy(new pcl::PointCloud<pcl::PointXYZ>(*m_filteredCloud));
    return copy;
}