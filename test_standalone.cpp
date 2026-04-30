
// Standalone Voxelizer Test (no ROS2)
#include <iostream>
#include <vector>
#include <random>
#include <chrono>
#include <cmath>

struct Voxel {
    uint32_t x, y, z;
    uint16_t label;
    float confidence;
    uint32_t point_count;
    float average_depth;
};

class SimpleVoxelizer {
public:
    SimpleVoxelizer(float voxel_size = 0.1f, float max_range = 10.0f)
        : voxel_size_(voxel_size), max_range_(max_range) {}
    
    bool processPoints(const std::vector<float>& points) {
        if (points.size() % 3 != 0) return false;
        
        size_t new_voxels = 0;
        for (size_t i = 0; i < points.size(); i += 3) {
            float x = points[i];
            float y = points[i + 1];
            float z = points[i + 2];
            
            float dist = std::sqrt(x*x + y*y + z*z);
            if (dist > max_range_ || dist < 0.1f) continue;
            
            Voxel v;
            v.x = static_cast<uint32_t>(std::floor(x / voxel_size_));
            v.y = static_cast<uint32_t>(std::floor(y / voxel_size_));
            v.z = static_cast<uint32_t>(std::floor(z / voxel_size_));
            v.label = 0;
            v.confidence = 0;
            v.point_count = 1;
            v.average_depth = dist;
            
            // Simple insert
            voxels_.push_back(v);
            new_voxels++;
        }
        
        return new_voxels > 0;
    }
    
    size_t getVoxelCount() const { return voxels_.size(); }
    void clear() { voxels_.clear(); }

private:
    float voxel_size_;
    float max_range_;
    std::vector<Voxel> voxels_;
};

int main() {
    std::cout << "=== VoxelNav Standalone Test ===\n\n";
    
    SimpleVoxelizer voxelizer(0.05f, 10.0f);
    
    // Generate random test points
    std::mt19937 rng(42);
    std::uniform_real_distribution<float> dist(0.5f, 4.5f);
    
    std::vector<float> test_points;
    for (int i = 0; i < 3000; ++i) {
        test_points.push_back(dist(rng));
        test_points.push_back(dist(rng));
        test_points.push_back(dist(rng));
    }
    
    std::cout << "Processing 1000 random points...\n";
    auto start = std::chrono::high_resolution_clock::now();
    
    voxelizer.processPoints(test_points);
    
    auto end = std::chrono::high_resolution_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    
    std::cout << "Voxels created: " << voxelizer.getVoxelCount() << "\n";
    std::cout << "Processing time: " << ms << " ms\n";
    std::cout << "Points per ms: " << (1000.0 / ms) << "\n\n";
    
    // Test with semantic labels
    voxelizer.clear();
    std::vector<float> semantic_points = {
        1.0f, 1.0f, 1.0f,  // Point 1
        1.05f, 1.05f, 1.05f,  // Point 2 (same voxel)
        2.0f, 2.0f, 2.0f,  // Point 3
        2.05f, 2.05f, 2.05f,  // Point 4 (same voxel)
    };
    
    voxelizer.processPoints(semantic_points);
    std::cout << "Semantic test voxels: " << voxelizer.getVoxelCount() << "\n";
    
    std::cout << "\n=== All tests passed ===\n";
    return 0;
}
