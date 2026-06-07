// VoxelNav - Semantic Segmentation Module
// MobileNetV3 ONNX inference wrapper

#ifndef VOXELNAV_SEGMENTER_HPP
#define VOXELNAV_SEGMENTER_HPP

#include <string>
#include <vector>
#include <memory>
#include <opencv2/opencv.hpp>

namespace voxelnav {

struct SegmentationResult {
    cv::Mat labels;        // Per-pixel class labels
    cv::Mat confidences;   // Per-pixel confidence scores
    std::vector<uint16_t> unique_labels;
    double inference_time_ms;
    
    SegmentationResult() : inference_time_ms(0.0) {}
};

enum class SemanticClass : uint16_t {
    UNKNOWN = 0,
    FLOOR = 1,
    WALL = 2,
    CEILING = 3,
    DOOR = 4,
    WINDOW = 5,
    FURNITURE = 6,
    PERSON = 7,
    VEHICLE = 8,
    PLANT = 9,
    STAIRS = 10,
    TABLE = 11,
    CHAIR = 12,
    BOX = 13,
    NUM_CLASSES = 14
};

struct SegmenterConfig {
    std::string model_path;
    int input_width;
    int input_height;
    int num_classes;
    float confidence_threshold;
    bool use_gpu;
    int gpu_device_id;
    
    SegmenterConfig()
        : model_path("models/mobilenet_v3_small.onnx"),
          input_width(320),
          input_height(240),
          num_classes(static_cast<int>(SemanticClass::NUM_CLASSES)),
          confidence_threshold(0.5f),
          use_gpu(false),
          gpu_device_id(0) {}
};

class Segmenter {
public:
    explicit Segmenter(const SegmenterConfig& config = SegmenterConfig());
    ~Segmenter();
    
    // Initialize the model (load ONNX)
    bool initialize();
    
    // Run semantic segmentation on RGB image
    SegmentationResult segment(const cv::Mat& rgb_image);
    
    // Run segmentation with depth alignment
    SegmentationResult segmentWithDepth(
        const cv::Mat& rgb_image,
        const cv::Mat& depth_image
    );
    
    // Get class name from label
    static std::string getClassName(uint16_t label);
    
    // Get class color for visualization
    static cv::Scalar getClassColor(uint16_t label);
    
    // Check if model is loaded
    bool isInitialized() const { return initialized_; }
    
    // Get config
    const SegmenterConfig& getConfig() const { return config_; }

private:
    SegmenterConfig config_;
    bool initialized_;
    
    // ONNX Runtime handles (forward decl for header)
    struct Impl;
    std::unique_ptr<Impl> impl_;
    
    // Preprocess image for model input
    cv::Mat preprocess(const cv::Mat& image);
    
    // Postprocess model output to labels
    SegmentationResult postprocess(
        const std::vector<float>& output,
        int orig_width,
        int orig_height
    );
};

} // namespace voxelnav

#endif // VOXELNAV_SEGMENTER_HPP