// VoxelNav - Semantic Segmentation Module
// SegFormer ONNX Runtime wrapper with deterministic fallback

#ifndef VOXELNAV_SEGMENTER_HPP
#define VOXELNAV_SEGMENTER_HPP

#include <memory>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

namespace voxelnav {

struct SegmentationResult {
    cv::Mat labels;
    cv::Mat confidences;
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
    bool allow_fallback;

    SegmenterConfig()
        : model_path("models/segformer-b0-finetuned-ade-512-512/onnx/model_static.onnx"),
          input_width(512),
          input_height(512),
          num_classes(static_cast<int>(SemanticClass::NUM_CLASSES)),
          confidence_threshold(0.5f),
          use_gpu(false),
          gpu_device_id(0),
          allow_fallback(true) {}
};

class Segmenter {
public:
    explicit Segmenter(const SegmenterConfig& config = SegmenterConfig());
    ~Segmenter();

    bool initialize();
    SegmentationResult segment(const cv::Mat& rgb_image);
    SegmentationResult segmentWithDepth(const cv::Mat& rgb_image, const cv::Mat& depth_image);

    static std::string getClassName(uint16_t label);
    static cv::Scalar getClassColor(uint16_t label);

    bool isInitialized() const { return initialized_; }
    bool hasRealModel() const;
    bool isUsingFallback() const { return !hasRealModel(); }
    const SegmenterConfig& getConfig() const { return config_; }

private:
    struct Impl;

    SegmenterConfig config_;
    bool initialized_;
    std::unique_ptr<Impl> impl_;

    cv::Mat preprocess(const cv::Mat& image) const;
    SegmentationResult fallbackSegment(const cv::Mat& rgb_image) const;
    SegmentationResult decodeTensor(const std::vector<float>& logits, const std::vector<int64_t>& shape, int orig_width, int orig_height) const;
    uint16_t remapRawLabel(int raw_label) const;
    bool tryLoadRealModel();
};

} // namespace voxelnav

#endif // VOXELNAV_SEGMENTER_HPP
