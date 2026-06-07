// VoxelNav - Voxelizer Unit Tests

#include <gtest/gtest.h>
#include "voxelnav/voxelizer.hpp"
#include <random>
#include <chrono>

using namespace voxelnav;

class VoxelizerTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Default config for testing
        VoxelGridConfig config;
        config.voxel_size = 0.1f;
        config.max_range = 5.0f;
        config.min_points_per_voxel = 2;
        voxelizer_ = std::make_unique<Voxelizer>(config);
        
        // Create random point cloud
        std::mt19937 rng(42);
        std::uniform_real_distribution<float> dist(0.5f, 4.5f);
        
        for (int i = 0; i < 1000; ++i) {
            test_points_.emplace_back(dist(rng), dist(rng), dist(rng));
        }
    }
    
    std::unique_ptr<Voxelizer> voxelizer_;
    std::vector<Eigen::Vector3f> test_points_;
};

TEST_F(VoxelizerTest, EmptyPointCloud) {
    std::vector<Eigen::Vector3f> empty_points;
    EXPECT_FALSE(voxelizer_->processPointCloud(empty_points));
    EXPECT_EQ(voxelizer_->getVoxelCount(), 0);
}

TEST_F(VoxelizerTest, BasicProcessing) {
    ASSERT_TRUE(voxelizer_->processPointCloud(test_points_));
    EXPECT_GT(voxelizer_->getVoxelCount(), 0);
}

TEST_F(VoxelizerTest, VoxelConsistency) {
    voxelizer_->processPointCloud(test_points_);
    auto voxels1 = voxelizer_->getVoxels();
    size_t count1 = voxels1.size();
    
    // Process same points again - should update existing voxels
    voxelizer_->processPointCloud(test_points_);
    auto voxels2 = voxelizer_->getVoxels();
    
    // Count should be similar (not more voxels for same points)
    EXPECT_LE(voxels2.size(), count1 + 10);  // Allow small variation
}

TEST_F(VoxelizerTest, OutOfRangePoints) {
    std::vector<Eigen::Vector3f> points = {
        {0.5f, 0.5f, 0.5f},   // Valid
        {10.0f, 0.5f, 0.5f},  // Out of range
        {-1.0f, 0.0f, 0.0f},  // Negative
        {0.05f, 0.05f, 0.05f} // Valid
    };
    
    voxelizer_->processPointCloud(points);
    
    // Should only have voxels for valid points
    EXPECT_GT(voxelizer_->getVoxelCount(), 0);
}

TEST_F(VoxelizerTest, SemanticLabels) {
    std::vector<Eigen::Vector3f> points = {
        {1.0f, 1.0f, 1.0f},
        {1.05f, 1.05f, 1.05f},
        {1.1f, 1.1f, 1.1f}
    };
    std::vector<uint16_t> labels = {1, 1, 1};  // Floor
    std::vector<float> confidences = {0.9f, 0.8f, 0.85f};
    
    voxelizer_->processPointCloud(points, labels, confidences);
    auto voxels = voxelizer_->getVoxels();
    
    ASSERT_FALSE(voxels.empty());
    EXPECT_EQ(voxels[0].label, 1);
    EXPECT_GT(voxels[0].confidence, 0.8f);
}

TEST_F(VoxelizerTest, BoundsQuery) {
    voxelizer_->processPointCloud(test_points_);
    
    Eigen::Vector3f min(1.0f, 1.0f, 1.0f);
    Eigen::Vector3f max(2.0f, 2.0f, 2.0f);
    
    auto bounded_voxels = voxelizer_->getVoxelsInBounds(min, max);
    
    // All returned voxels should be within bounds
    for (const Voxel& v : bounded_voxels) {
        EXPECT_GE(v.x, 10);  // 1.0 / 0.1
        EXPECT_LE(v.x, 20);  // 2.0 / 0.1
    }
}

TEST_F(VoxelizerTest, ClearFunctionality) {
    voxelizer_->processPointCloud(test_points_);
    EXPECT_GT(voxelizer_->getVoxelCount(), 0);
    
    voxelizer_->clear();
    EXPECT_EQ(voxelizer_->getVoxelCount(), 0);
}

TEST_F(VoxelizerTest, DynamicRemoval) {
    voxelizer_->setDynamicRemoval(true, 0.3f);
    
    // Initial frame
    std::vector<Eigen::Vector3f> frame1 = {
        {1.0f, 1.0f, 1.0f}, {1.05f, 1.05f, 1.05f},
        {2.0f, 2.0f, 2.0f}, {2.05f, 2.05f, 2.05f}
    };
    voxelizer_->processPointCloud(frame1);
    
    // Second frame - one voxel area changed significantly
    std::vector<Eigen::Vector3f> frame2 = {
        {1.0f, 1.0f, 1.0f}, {1.05f, 1.05f, 1.05f}
        // Area around (2,2,2) disappeared
    };
    
    auto dynamic = voxelizer_->detectDynamicChanges(frame2, 0.5f);
    EXPECT_GT(dynamic.size(), 0);
}

TEST_F(VoxelizerTest, MemoryEstimate) {
    voxelizer_->processPointCloud(test_points_);
    float mem_mb = voxelizer_->getGridMemoryMB();
    EXPECT_GT(mem_mb, 0.0f);
    EXPECT_LT(mem_mb, 10.0f);  // Should be small for test data
}

TEST_F(VoxelizerTest, ThreadSafety) {
    // Process same points from multiple threads
    std::vector<std::thread> threads;
    
    for (int i = 0; i < 4; ++i) {
        threads.emplace_back([this]() {
            voxelizer_->processPointCloud(test_points_);
        });
    }
    
    for (auto& t : threads) {
        t.join();
    }
    
    // Should not crash, should have valid voxels
    EXPECT_GT(voxelizer_->getVoxelCount(), 0);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}