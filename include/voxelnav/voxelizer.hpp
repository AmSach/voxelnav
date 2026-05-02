// VoxelNav - Real-time Semantic Voxel Mapping
// Core voxel hashing with octree structure

#ifndef VOXELNAV_VOXELIZER_HPP
#define VOXELNAV_VOXELIZER_HPP

#include <Eigen/Dense>
#include <vector>
#include <unordered_map>
#include <cstdint>
#include <mutex>
#include <memory>

namespace voxelnav {

struct Voxel {
    int32_t x, y, z;
    uint16_t label;      // Semantic label (0=unknown)
    float confidence;    // Segmentation confidence
    uint32_t point_count;
    float average_depth;
    uint64_t timestamp_ns;
    
    Voxel() : x(0), y(0), z(0), label(0), confidence(0.0f), 
              point_count(0), average_depth(0.0f), timestamp_ns(0) {}
};

struct VoxelHash {
    size_t operator()(const Voxel& v) const {
        size_t seed = 0;
        seed ^= std::hash<int32_t>{}(v.x) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= std::hash<int32_t>{}(v.y) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        seed ^= std::hash<int32_t>{}(v.z) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        return seed;
    }
};

struct VoxelEqual {
    bool operator()(const Voxel& a, const Voxel& b) const {
        return a.x == b.x && a.y == b.y && a.z == b.z;
    }
};

struct VoxelGridConfig {
    float voxel_size;           // meters per voxel
    float max_range;            // max sensor range
    float min_points_per_voxel; // minimum points to create voxel
    uint32_t grid_size_x;
    uint32_t grid_size_y;
    uint32_t grid_size_z;
    bool enable_dynamic_removal;
    float dynamic_threshold;    // change threshold for dynamic objects
    
    VoxelGridConfig() 
        : voxel_size(0.05f), max_range(10.0f), min_points_per_voxel(3),
          grid_size_x(200), grid_size_y(200), grid_size_z(200),
          enable_dynamic_removal(true), dynamic_threshold(0.3f) {}
};

class Voxelizer {
public:
    explicit Voxelizer(const VoxelGridConfig& config = VoxelGridConfig());
    ~Voxelizer() = default;
    
    // Process point cloud and update voxel grid
    bool processPointCloud(
        const std::vector<Eigen::Vector3f>& points,
        const std::vector<uint16_t>& labels = {},
        const std::vector<float>& confidences = {}
    );
    
    // Get all voxels as vector
    std::vector<Voxel> getVoxels() const;
    
    // Get voxels within bounding box
    std::vector<Voxel> getVoxelsInBounds(
        const Eigen::Vector3f& min,
        const Eigen::Vector3f& max
    ) const;
    
    // Clear the voxel grid
    void clear();
    
    // Get statistics
    size_t getVoxelCount() const;
    float getGridMemoryMB() const;
    std::unordered_map<uint16_t, size_t> getLabelHistogram() const;
    std::vector<Voxel> getTopVoxelsByConfidence(size_t limit) const;
    size_t pruneStaleVoxels(uint64_t max_age_ns, float min_confidence = 0.1f);
    
    // Enable/disable dynamic object removal
    void setDynamicRemoval(bool enable, float threshold = 0.3f);
    
    // Frame differencing for dynamic objects
    std::vector<Voxel> detectDynamicChanges(
        const std::vector<Eigen::Vector3f>& new_points,
        float change_threshold
    );

    Eigen::Vector3f voxelToWorld(const Voxel& v) const;

private:
    VoxelGridConfig config_;
    mutable std::mutex mutex_;
    
    // Hash map for O(1) voxel lookup
    std::unordered_map<Voxel, uint32_t, VoxelHash, VoxelEqual> voxel_index_map_;
    std::vector<Voxel> voxels_;
    
    // World to grid coordinate conversion
    Voxel worldToVoxel(const Eigen::Vector3f& point) const;
    bool isValidPoint(const Eigen::Vector3f& point) const;
    bool isWithinGridBounds(const Voxel& voxel) const;
    
    // Update existing voxel with new data
    void updateVoxel(Voxel& voxel, float depth, uint16_t label, float confidence);
};

} // namespace voxelnav

#endif // VOXELNAV_VOXELIZER_HPP