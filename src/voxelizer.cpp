// VoxelNav - Voxelizer Implementation
// Core voxel hashing with octree structure

#include "voxelnav/voxelizer.hpp"
#include <chrono>
#include <algorithm>
#include <cmath>

namespace voxelnav {

Voxelizer::Voxelizer(const VoxelGridConfig& config)
    : config_(config)
{
    // Pre-allocate space for typical use case
    voxels_.reserve(10000);
    voxel_index_map_.reserve(10000);
}

Voxel Voxelizer::worldToVoxel(const Eigen::Vector3f& point) const {
    Voxel v;
    v.x = static_cast<uint32_t>(std::floor(point.x() / config_.voxel_size));
    v.y = static_cast<uint32_t>(std::floor(point.y() / config_.voxel_size));
    v.z = static_cast<uint32_t>(std::floor(point.z() / config_.voxel_size));
    return v;
}

Eigen::Vector3f Voxelizer::voxelToWorld(const Voxel& v) const {
    return Eigen::Vector3f(
        (static_cast<float>(v.x) + 0.5f) * config_.voxel_size,
        (static_cast<float>(v.y) + 0.5f) * config_.voxel_size,
        (static_cast<float>(v.z) + 0.5f) * config_.voxel_size
    );
}

bool Voxelizer::isValidPoint(const Eigen::Vector3f& point) const {
    float dist = point.norm();
    if (dist > config_.max_range || dist < 0.1f) {
        return false;
    }
    
    // Check grid bounds
    Voxel v = worldToVoxel(point);
    return v.x < config_.grid_size_x &&
           v.y < config_.grid_size_y &&
           v.z < config_.grid_size_z;
}

void Voxelizer::updateVoxel(Voxel& voxel, float depth, uint16_t label, float confidence) {
    // Update average depth
    float total_depth = voxel.average_depth * voxel.point_count + depth;
    voxel.point_count++;
    voxel.average_depth = total_depth / voxel.point_count;
    
    // Update label if confidence is higher
    if (confidence > voxel.confidence) {
        voxel.label = label;
        voxel.confidence = confidence;
    }
    
    // Update timestamp
    auto now = std::chrono::high_resolution_clock::now();
    voxel.timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        now.time_since_epoch()
    ).count();
}

bool Voxelizer::processPointCloud(
    const std::vector<Eigen::Vector3f>& points,
    const std::vector<uint16_t>& labels,
    const std::vector<float>& confidences
) {
    if (points.empty()) {
        return false;
    }
    
    std::lock_guard<std::mutex> lock(mutex_);
    
    bool has_labels = !labels.empty() && labels.size() == points.size();
    bool has_confidences = !confidences.empty() && confidences.size() == points.size();
    
    // Temporary storage for point counts per voxel
    std::unordered_map<Voxel, std::vector<size_t>, VoxelHash, VoxelEqual> voxel_points;
    
    for (size_t i = 0; i < points.size(); ++i) {
        if (!isValidPoint(points[i])) {
            continue;
        }
        
        Voxel v = worldToVoxel(points[i]);
        voxel_points[v].push_back(i);
    }
    
    // Create/update voxels that meet minimum point threshold
    for (const auto& [voxel_key, point_indices] : voxel_points) {
        if (point_indices.size() < config_.min_points_per_voxel) {
            continue;
        }
        
        // Check if voxel already exists
        auto it = voxel_index_map_.find(voxel_key);
        
        if (it != voxel_index_map_.end()) {
            // Update existing voxel
            Voxel& existing = voxels_[it->second];
            for (size_t idx : point_indices) {
                float depth = points[idx].norm();
                uint16_t label = has_labels ? labels[idx] : 0;
                float conf = has_confidences ? confidences[idx] : 0.0f;
                updateVoxel(existing, depth, label, conf);
            }
        } else {
            // Create new voxel
            Voxel new_voxel = voxel_key;
            new_voxel.point_count = 0;
            new_voxel.average_depth = 0.0f;
            
            for (size_t idx : point_indices) {
                float depth = points[idx].norm();
                uint16_t label = has_labels ? labels[idx] : 0;
                float conf = has_confidences ? confidences[idx] : 0.0f;
                updateVoxel(new_voxel, depth, label, conf);
            }
            
            // Add to storage
            uint32_t new_index = static_cast<uint32_t>(voxels_.size());
            voxels_.push_back(new_voxel);
            voxel_index_map_[new_voxel] = new_index;
        }
    }
    
    return true;
}

std::vector<Voxel> Voxelizer::getVoxels() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return voxels_;
}

std::vector<Voxel> Voxelizer::getVoxelsInBounds(
    const Eigen::Vector3f& min,
    const Eigen::Vector3f& max
) const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    std::vector<Voxel> result;
    result.reserve(voxels_.size() / 10);  // Estimate 10% in bounds
    
    Voxel v_min = worldToVoxel(min);
    Voxel v_max = worldToVoxel(max);
    
    for (const Voxel& v : voxels_) {
        if (v.x >= v_min.x && v.x <= v_max.x &&
            v.y >= v_min.y && v.y <= v_max.y &&
            v.z >= v_min.z && v.z <= v_max.z) {
            result.push_back(v);
        }
    }
    
    return result;
}

void Voxelizer::clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    voxels_.clear();
    voxel_index_map_.clear();
}

size_t Voxelizer::getVoxelCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return voxels_.size();
}

float Voxelizer::getGridMemoryMB() const {
    std::lock_guard<std::mutex> lock(mutex_);
    
    size_t bytes = voxels_.capacity() * sizeof(Voxel) +
                   voxel_index_map_.size() * (sizeof(Voxel) + sizeof(uint32_t));
    
    return static_cast<float>(bytes) / (1024.0f * 1024.0f);
}

void Voxelizer::setDynamicRemoval(bool enable, float threshold) {
    config_.enable_dynamic_removal = enable;
    config_.dynamic_threshold = threshold;
}

std::vector<Voxel> Voxelizer::detectDynamicChanges(
    const std::vector<Eigen::Vector3f>& new_points,
    float change_threshold
) {
    std::vector<Voxel> dynamic_voxels;
    
    if (!config_.enable_dynamic_removal) {
        return dynamic_voxels;
    }
    
    // Count points in each voxel for new frame
    std::unordered_map<Voxel, int, VoxelHash, VoxelEqual> new_counts;
    
    for (const auto& point : new_points) {
        if (!isValidPoint(point)) continue;
        Voxel v = worldToVoxel(point);
        new_counts[v]++;
    }
    
    // Compare with existing voxels
    std::lock_guard<std::mutex> lock(mutex_);
    
    for (const Voxel& existing : voxels_) {
        auto it = new_counts.find(existing);
        
        if (it == new_counts.end()) {
            // Voxel disappeared - potentially dynamic object moved
            dynamic_voxels.push_back(existing);
        } else {
            // Check for significant change in point count
            float ratio = static_cast<float>(it->second) / 
                          static_cast<float>(existing.point_count);
            
            if (ratio < change_threshold || ratio > (1.0f / change_threshold)) {
                dynamic_voxels.push_back(existing);
            }
        }
    }
    
    return dynamic_voxels;
}

} // namespace voxelnav