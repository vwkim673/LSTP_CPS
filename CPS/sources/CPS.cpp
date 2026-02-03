#include "CPS.h"
#include <iostream>

CPS::CPS(
    float minX,
    float maxX,
    float minY,
    float maxY,
    float minZ,
    float maxZ,
    float voxelSize,
    int historySize,
    float maxJump
) :
    m_minX(minX),
    m_maxX(maxX),
    m_minY(minY),
    m_maxY(maxY),
    m_minZ(minZ),
    m_maxZ(maxZ),
    m_voxelSize(voxelSize),
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
    m_voxelizedCloud->clear();
}


void getCloudXYRange(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    float& minX, float& maxX, float& minY, float& maxY
) {
    // Initialize with extreme values
    minX = std::numeric_limits<float>::max();
    maxX = std::numeric_limits<float>::lowest();
    minY = std::numeric_limits<float>::max();
    maxY = std::numeric_limits<float>::lowest();

    // Early check for empty cloud
    if (cloud->empty()) {
        return;
    }

    // Single pass through the point cloud
    for (const auto& point : cloud->points) {
        // Update X range
        if (point.x < minX) minX = point.x;
        if (point.x > maxX) maxX = point.x;

        // Update Y range
        if (point.y < minY) minY = point.y;
        if (point.y > maxY) maxY = point.y;
    }
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

    if (m_filteredCloud->empty() || m_filteredCloud->points.size() < 200) {
        // Not enough points for reliable detection
        return m_lastValidDistance;
    }

    m_filteredCloud->width = m_filteredCloud->points.size();
    m_filteredCloud->height = 1;
    m_filteredCloud->is_dense = false;

    // Step 2: Apply voxel grid filtering to create uniform point density
    pcl::VoxelGrid<pcl::PointXYZ> voxelFilter;
    voxelFilter.setInputCloud(m_filteredCloud);
    voxelFilter.setLeafSize(m_voxelSize, m_voxelSize, m_voxelSize);
    m_voxelizedCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    voxelFilter.filter(*m_voxelizedCloud);

    if (m_voxelizedCloud->empty() || m_voxelizedCloud->points.size() < 5) {
        // Not enough points after voxelization
        return m_lastValidDistance;
    }

    //Step3. Get "valid" minimum Z.
    int valid_min_z = get_valid_min_z_binary_search(m_voxelizedCloud);

    //Another filter that does Z range filter.
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& point : m_voxelizedCloud->points) {
        // Efficient single-pass filtering on all three axes
        if (point.z > valid_min_z && point.z <= valid_min_z + 300) {
            tempCloud->points.push_back(point);
        }
    }

    //Step4. narrow down X,Y range by 10% from each end.
    float f2minX, f2maxX, f2minY, f2maxY;
    getCloudXYRange(tempCloud, f2minX, f2maxX, f2minY, f2maxY);
    int f2width = f2maxX - f2minX;
    int f2height = f2maxY - f2minY;

    m_filteredCloud2.reset(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& point : tempCloud->points) {
        // Efficient single-pass filtering on all three axes
        if (point.x >= f2minX + f2width * 0.1 && point.x <= f2maxX - f2width * 0.1 &&
            point.y >= f2minY + f2height * 0.1 && point.y <= f2maxY - f2height * 0.1)
        {
            m_filteredCloud2->points.push_back(point);
        }
    }

    //Step4. Get "Buckets" Z slice 
    int chassisDistance = get_valid_min_z_comprehensive(m_filteredCloud2, valid_min_z, 50, 30, 200.f, 100.f);

    return chassisDistance;
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
    float voxelSize,
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
    m_voxelSize = voxelSize;
    m_historySize = historySize;
    m_maxJump = maxJump;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CPS::getFilteredCloud() const {
    // Return a copy to ensure safety
    pcl::PointCloud<pcl::PointXYZ>::Ptr copy(new pcl::PointCloud<pcl::PointXYZ>(*m_filteredCloud));
    return copy;
}
pcl::PointCloud<pcl::PointXYZ>::Ptr CPS::getFilteredCloud2() const {
    // Return a copy to ensure safety
    pcl::PointCloud<pcl::PointXYZ>::Ptr copy(new pcl::PointCloud<pcl::PointXYZ>(*m_filteredCloud2));
    return copy;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CPS::getVoxelizedCloud() const {
    // Return a copy to ensure safety
    pcl::PointCloud<pcl::PointXYZ>::Ptr copy(new pcl::PointCloud<pcl::PointXYZ>(*m_voxelizedCloud));
    return copy;
}



pcl::PointCloud<pcl::PointXYZ>::Ptr CPS::singlePassFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr input, float minX, float maxX, float minY, float maxY, float minZ, float maxZ)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& point : input->points) {
        // Efficient single-pass filtering on all three axes
        if (point.x >= minX && point.x <= maxX &&
            point.y >= minY && point.y <= maxY &&
            point.z >= minZ && point.z <= maxZ) {
            filteredCloud->points.push_back(point);
        }
    }
    return filteredCloud;
}

int CPS::get_valid_min_z_binary_search(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input) {
    // Early check for empty cloud
    if (input->empty()) {
        return 0;
    }

    // Use direct access to find min/max Z values and collect Z values in a single pass
    int local_minZ = 10000;
    int local_maxZ = -10000;

    // Pre-allocate vectors for better memory efficiency
    std::vector<float> zValues;
    zValues.reserve(input->size());

    // Single pass through the point cloud
    for (const auto& point : input->points) {
        float z = point.z;

        // Filter out extreme values
        if (z > -10000 && z < 10000) {
            zValues.push_back(z);

            // Update min/max
            if (z < local_minZ) local_minZ = static_cast<int>(z);
            if (z > local_maxZ) local_maxZ = static_cast<int>(z);
        }
    }

    // Sort Z values for binary search
    std::sort(zValues.begin(), zValues.end());

    // Use fixed slice parameters
    const int sliceDepth = 40;
    const int minPointsPerSlice = 50;

    // Slice through Z range with 10mm steps
    for (int cz = local_minZ; cz < local_maxZ; cz += 10) {
        // Use binary search to find the slice boundaries
        auto lowerBound = std::lower_bound(zValues.begin(), zValues.end(), static_cast<float>(cz));
        auto upperBound = std::upper_bound(zValues.begin(), zValues.end(), static_cast<float>(cz + sliceDepth));

        // Calculate number of points in this slice
        int pointsInSlice = std::distance(lowerBound, upperBound);

        // Check if this slice has enough points
        if (pointsInSlice >= minPointsPerSlice) {
            // Calculate the average Z value of this slice
            float sumZ = std::accumulate(lowerBound, upperBound, 0.0f);
            return static_cast<int>(sumZ / pointsInSlice);
        }
    }

    // No valid slice found, return the minimum Z
    return local_minZ;
}

/**
 * @brief Ultra-optimized get_valid_min_z function using compile-time fixed buckets
 *
 * This version uses fixed-size arrays for maximum performance with small, known slice counts.
 * Ideal when max_slices is small and known at compile time.
 *
 * @param input Input point cloud
 * @param minZ Starting minimum Z value (default 300mm)
 * @param slice_step Slice width (default 50mm)
 * @param min_points_per_slice Minimum points needed for a valid slice (default 30)
 * @return Valid minimum Z value (int)
 */
int CPS::get_valid_min_z_fixed_buckets(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
    int minZ,
    int slice_step,
    int min_points_per_slice
) {
    // Early check for empty cloud
    if (input->empty()) {
        return minZ;
    }

    // Fixed number of buckets for maximum performance
    constexpr int NUM_BUCKETS = 10;

    // Calculate the maximum Z value to consider
    int maxZ = minZ + (NUM_BUCKETS * slice_step / 2);

    // Create fixed-size arrays for buckets
    // Using arrays instead of vectors for better cache performance
    std::array<int, NUM_BUCKETS> counts = {};  // Initialize all to 0
    std::array<float, NUM_BUCKETS> sums = {};  // Initialize all to 0.0

    // Single pass through the point cloud
    for (const auto& point : input->points) {
        // Apply all filters in a single check
        if (point.y >= 0 && point.y <= 2000 && point.z >= minZ && point.z <= maxZ) {
            // Find which buckets this point belongs to
            for (int i = 0; i < NUM_BUCKETS; i++) {
                int sliceMinZ = minZ + i * (slice_step / 2);
                int sliceMaxZ = sliceMinZ + slice_step;

                if (point.z >= sliceMinZ && point.z <= sliceMaxZ) {
                    // Add point to this bucket
                    counts[i]++;
                    sums[i] += point.z;
                }
            }
        }
    }

    // Find the first valid bucket (slice)
    for (int i = 0; i < NUM_BUCKETS; i++) {
        if (counts[i] >= min_points_per_slice) {
            // Calculate average Z for this slice
            float avgZ = sums[i] / counts[i];
            return static_cast<int>(avgZ);
        }
    }

    // If no valid slice found, return the original minZ
    return minZ;
}

int CPS::get_valid_min_z_comprehensive(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input,
    int minZ,
    int slice_step,
    int min_points_per_slice,
    float min_x_spread,
    float min_y_spread,
    float min_score_threshold
) {
    // Early check for empty cloud
    if (input->empty()) {
        return minZ;
    }

    // Fixed number of buckets for maximum performance
    constexpr int NUM_BUCKETS = 10;

    // Calculate the maximum Z value to consider
    int maxZ = minZ + (NUM_BUCKETS * slice_step / 2);

    // Structure to store comprehensive bucket information
    struct BucketInfo {
        int count = 0;                          // Point count
        float sum_z = 0.0f;                     // Sum of Z values
        float min_z = std::numeric_limits<float>::max();
        float min_x = std::numeric_limits<float>::max();
        float max_x = std::numeric_limits<float>::lowest();
        float min_y = std::numeric_limits<float>::max();
        float max_y = std::numeric_limits<float>::lowest();
        // For calculating variance
        float sum_z_squared = 0.0f;             // Sum of Z^2 values
    };

    // Create array of bucket information
    std::array<BucketInfo, NUM_BUCKETS> buckets = {};

    // Single pass through the point cloud
    for (const auto& point : input->points) {
        // Apply all filters in a single check
        if (point.y >= m_minY && point.y <= m_maxY && point.z >= minZ && point.z <= maxZ) {
            // Find which buckets this point belongs to
            for (int i = 0; i < NUM_BUCKETS; i++) {
                int sliceMinZ = minZ + i * (slice_step / 2);
                int sliceMaxZ = sliceMinZ + slice_step;

                if (point.z >= sliceMinZ && point.z <= sliceMaxZ) {
                    // Update bucket information
                    BucketInfo& bucket = buckets[i];
                    bucket.count++;
                    bucket.sum_z += point.z;
                    bucket.sum_z_squared += point.z * point.z;

                    // Update X and Y ranges
                    bucket.min_x = std::min(bucket.min_x, point.x);
                    bucket.max_x = std::max(bucket.max_x, point.x);
                    bucket.min_y = std::min(bucket.min_y, point.y);
                    bucket.max_y = std::max(bucket.max_y, point.y);
                    bucket.min_z = std::min(bucket.min_z, point.z);

                }
            }
        }
    }

    // Calculate scores for each bucket based on multiple criteria
    std::array<float, NUM_BUCKETS> scores = {};
    std::array<bool, NUM_BUCKETS> is_valid_bucket = {};

    for (int i = 0; i < NUM_BUCKETS; i++) {
        const BucketInfo& bucket = buckets[i];

        // Skip buckets with too few points
        if (bucket.count < min_points_per_slice) {
            continue;
        }

        // Calculate spatial properties
        float x_spread = bucket.max_x - bucket.min_x;
        float y_spread = bucket.max_y - bucket.min_y;
        float area = x_spread * y_spread;
        float density = (area > 0) ? bucket.count / area : 0;

        // Calculate Z variance (lower is better - indicates flat surface)
        float mean_z = bucket.sum_z / bucket.count;
        float variance_z = (bucket.sum_z_squared / bucket.count) - (mean_z * mean_z);

        // Check if bucket has sufficient spread
        if (true) {
            // Calculate score based on multiple factors
            scores[i] = bucket.count * 0.7f +         // Point count
                x_spread * 0.5f +             // X spread
                y_spread * 0.25f +             // Y spread
                density * 1000.0f; //-           // Point density
                //variance_z * 10.0f;           // Z variance (negative factor)

            // Mark bucket as valid if score is above threshold
            is_valid_bucket[i] = (scores[i] >= min_score_threshold);
        }
    }

    // Find the closest valid bucket
    for (int i = 0; i < NUM_BUCKETS; i++) {
        if (is_valid_bucket[i]) {
            // Found the closest valid bucket
            float avgZ = buckets[i].sum_z / buckets[i].count;
            return static_cast<int>(avgZ);
        }
    }

    // If no valid bucket found, return the original minZ
    return minZ;
}