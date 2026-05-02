#include "voxelnav/segmenter.hpp"
#include "voxelnav/voxelizer.hpp"

#include <opencv2/opencv.hpp>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

using namespace voxelnav;

static void require_true(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "[FAIL] " << message << std::endl;
        std::exit(1);
    }
}

int main() {
    std::cout << "VoxelNav core smoke test\n";

    VoxelGridConfig voxel_config;
    voxel_config.voxel_size = 0.1f;
    voxel_config.max_range = 5.0f;
    voxel_config.min_points_per_voxel = 2;
    voxel_config.grid_size_x = 200;
    voxel_config.grid_size_y = 200;
    voxel_config.grid_size_z = 200;

    Voxelizer voxelizer(voxel_config);

    std::vector<Eigen::Vector3f> points = {
        {0.11f, 0.11f, 0.11f},
        {0.12f, 0.12f, 0.12f},
        {-0.11f, -0.11f, -0.11f},
        {-0.12f, -0.12f, -0.12f},
        {1.00f, 1.00f, 1.00f},
        {1.05f, 1.05f, 1.05f},
    };
    std::vector<uint16_t> labels = {2, 2, 1, 1, 1, 1};
    std::vector<float> confidences = {0.9f, 0.8f, 0.7f, 0.95f, 0.88f, 0.92f};

    require_true(voxelizer.processPointCloud(points, labels, confidences), "voxelizer accepted point cloud");
    require_true(voxelizer.getVoxelCount() >= 3, "voxelizer created at least three voxels");
    require_true(voxelizer.getGridMemoryMB() > 0.0f, "voxelizer reports memory usage");
    require_true(voxelizer.getLabelHistogram().size() >= 2, "label histogram is populated");
    require_true(!voxelizer.getTopVoxelsByConfidence(2).empty(), "top voxel ranking works");

    auto voxels = voxelizer.getVoxels();
    const size_t initial_voxel_count = voxels.size();
    bool saw_label_one = false;
    bool saw_negative_coord = false;
    for (const auto& voxel : voxels) {
        if (voxel.label == 1 && voxel.confidence >= 0.8f) {
            saw_label_one = true;
        }
        if (voxel.x < 0 || voxel.y < 0 || voxel.z < 0) {
            saw_negative_coord = true;
        }
    }
    require_true(saw_label_one, "semantic labels were preserved on at least one voxel");
    require_true(saw_negative_coord, "negative world coordinates map into signed voxel space");

    auto bounded = voxelizer.getVoxelsInBounds(Eigen::Vector3f(-0.5f, -0.5f, -0.5f), Eigen::Vector3f(1.2f, 1.2f, 1.2f));
    require_true(!bounded.empty(), "bounded query returned voxels");

    auto dynamic = voxelizer.detectDynamicChanges({{0.11f, 0.11f, 0.11f}, {1.00f, 1.00f, 1.00f}}, 0.5f);
    require_true(!dynamic.empty(), "dynamic change detection returned something");

    require_true(voxelizer.pruneStaleVoxels(1, 1.1f) > 0, "stale pruning can remove weak voxels");
    const size_t pruned_voxel_count = voxelizer.getVoxelCount();

    SegmenterConfig segmenter_config;
    segmenter_config.model_path = "/tmp/voxelnav_missing_model.onnx";
    segmenter_config.input_width = 320;
    segmenter_config.input_height = 240;

    Segmenter segmenter(segmenter_config);
    require_true(segmenter.initialize(), "segmenter initializes even without a real ONNX model");

    cv::Mat rgb(240, 320, CV_8UC3, cv::Scalar(20, 30, 40));
    auto seg = segmenter.segment(rgb);
    require_true(seg.labels.rows == 240 && seg.labels.cols == 320, "segmentation output matches input size");
    require_true(seg.confidences.rows == 240 && seg.confidences.cols == 320, "confidence output matches input size");
    require_true(!seg.unique_labels.empty(), "segmentation produces at least one label");
    require_true(seg.inference_time_ms >= 0.0, "segmentation reports an inference time");

    cv::Mat depth_float(240, 320, CV_32FC1, cv::Scalar(1.0f));
    cv::Mat depth_u16(240, 320, CV_16UC1, cv::Scalar(1000));

    auto seg_depth_float = segmenter.segmentWithDepth(rgb, depth_float);
    auto seg_depth_u16 = segmenter.segmentWithDepth(rgb, depth_u16);
    require_true(seg_depth_float.labels.rows == 240 && seg_depth_float.labels.cols == 320, "float depth path works");
    require_true(seg_depth_u16.labels.rows == 240 && seg_depth_u16.labels.cols == 320, "uint16 depth path works");

    require_true(Segmenter::getClassName(1) == "floor", "class name lookup works");
    require_true(Segmenter::getClassColor(1)[1] == 0, "class color lookup works");

    std::cout << "Initial voxel count: " << initial_voxel_count << '\n';
    std::cout << "Pruned voxel count: " << pruned_voxel_count << '\n';
    std::cout << "Unique semantic labels: " << seg.unique_labels.size() << '\n';
    std::cout << "Inference time: " << seg.inference_time_ms << " ms\n";
    std::cout << "All core smoke checks passed.\n";
    return 0;
}
