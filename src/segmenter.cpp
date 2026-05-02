// VoxelNav - Semantic Segmentation Module
// MobileNetV3 ONNX inference wrapper

#include "voxelnav/segmenter.hpp"
#include <fstream>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <chrono>

namespace voxelnav {

// Class colors for visualization
static const cv::Scalar CLASS_COLORS[] = {
    cv::Scalar(128, 128, 128),  // UNKNOWN - gray
    cv::Scalar(128, 0, 128),    // FLOOR - purple
    cv::Scalar(255, 0, 0),      // WALL - blue
    cv::Scalar(0, 255, 255),    // CEILING - yellow
    cv::Scalar(0, 255, 0),      // DOOR - green
    cv::Scalar(255, 255, 0),    // WINDOW - cyan
    cv::Scalar(0, 128, 255),    // FURNITURE - orange
    cv::Scalar(0, 0, 255),      // PERSON - red
    cv::Scalar(255, 0, 255),    // VEHICLE - magenta
    cv::Scalar(0, 255, 128),    // PLANT - spring green
    cv::Scalar(128, 64, 0),     // STAIRS - brown
    cv::Scalar(255, 128, 128),  // TABLE - light blue
    cv::Scalar(128, 255, 128),  // CHAIR - light green
    cv::Scalar(128, 128, 255),  // BOX - light red
};

// Class names
static const char* CLASS_NAMES[] = {
    "unknown", "floor", "wall", "ceiling", "door", "window",
    "furniture", "person", "vehicle", "plant", "stairs",
    "table", "chair", "box"
};

// ONNX Runtime implementation details
struct Segmenter::Impl {
    void* session = nullptr;
    void* env = nullptr;
    void* session_options = nullptr;
    std::vector<const char*> input_names;
    std::vector<const char*> output_names;
    std::vector<std::vector<int64_t>> input_shapes;
    std::vector<std::vector<int64_t>> output_shapes;
};

Segmenter::Segmenter(const SegmenterConfig& config)
    : config_(config)
    , initialized_(false)
    , impl_(std::make_unique<Impl>())
{
}

Segmenter::~Segmenter() {
    // Cleanup ONNX resources
    if (impl_->session) {
        // Would call OrtReleaseSession here
    }
    if (impl_->env) {
        // Would call OrtReleaseEnv here
    }
}

bool Segmenter::initialize() {
    if (initialized_) {
        return true;
    }

    std::ifstream model_file(config_.model_path, std::ios::binary);
    if (!model_file.good()) {
        std::cerr << "[VoxelNav] Model file not found: " << config_.model_path << std::endl;
        std::cerr << "[VoxelNav] Using deterministic mock segmenter instead of ONNX runtime." << std::endl;
    }

    initialized_ = true;
    std::cout << "[VoxelNav] Segmenter initialized with model: " << config_.model_path << std::endl;

    return true;
}

cv::Mat Segmenter::preprocess(const cv::Mat& image) {
    cv::Mat resized;
    cv::resize(image, resized, cv::Size(config_.input_width, config_.input_height));

    // Convert to float and normalize (ImageNet mean/std)
    cv::Mat float_img;
    resized.convertTo(float_img, CV_32FC3);

    // Normalize: subtract mean, divide by std
    float_img = (float_img - cv::Scalar(123.675, 116.28, 103.53)) /
                cv::Scalar(58.395, 57.12, 57.375);

    return float_img;
}

SegmentationResult Segmenter::postprocess(
    const std::vector<float>& output,
    int orig_width,
    int orig_height
) {
    SegmentationResult result;

    int output_height = config_.input_height;
    int output_width = config_.input_width;

    // Output shape: [1, num_classes, H, W]
    int num_classes = config_.num_classes;

    // Create label and confidence maps
    cv::Mat labels(output_height, output_width, CV_16UC1);
    cv::Mat confidences(output_height, output_width, CV_32FC1);

    for (int y = 0; y < output_height; ++y) {
        for (int x = 0; x < output_width; ++x) {
            // Find class with highest probability
            float max_prob = -1.0f;
            int max_class = 0;

            for (int c = 0; c < num_classes; ++c) {
                int idx = c * output_height * output_width + y * output_width + x;
                float prob = output[idx];

                if (prob > max_prob) {
                    max_prob = prob;
                    max_class = c;
                }
            }

            labels.at<uint16_t>(y, x) = static_cast<uint16_t>(max_class);
            confidences.at<float>(y, x) = max_prob;
        }
    }

    // Resize back to original size
    if (labels.cols != orig_width || labels.rows != orig_height) {
        cv::resize(labels, result.labels, cv::Size(orig_width, orig_height),
                   0, 0, cv::INTER_NEAREST);
        cv::resize(confidences, result.confidences, cv::Size(orig_width, orig_height),
                   0, 0, cv::INTER_LINEAR);
    } else {
        result.labels = labels;
        result.confidences = confidences;
    }

    // Find unique labels
    for (int y = 0; y < result.labels.rows; ++y) {
        for (int x = 0; x < result.labels.cols; ++x) {
            uint16_t label = result.labels.at<uint16_t>(y, x);
            if (result.confidences.at<float>(y, x) >= config_.confidence_threshold) {
                if (std::find(result.unique_labels.begin(),
                              result.unique_labels.end(), label) == result.unique_labels.end()) {
                    result.unique_labels.push_back(label);
                }
            }
        }
    }

    return result;
}

SegmentationResult Segmenter::segment(const cv::Mat& rgb_image) {
    if (!initialized_) {
        std::cerr << "[VoxelNav] Segmenter not initialized!" << std::endl;
        return SegmentationResult();
    }

    if (rgb_image.empty()) {
        std::cerr << "[VoxelNav] Empty input image!" << std::endl;
        return SegmentationResult();
    }

    auto start = std::chrono::high_resolution_clock::now();

    // Preprocess
    cv::Mat preprocessed = preprocess(rgb_image);

    // Run ONNX inference
    // Note: This is a simplified mock implementation
    // In production, would use actual ONNX Runtime API

    // Mock output: random segmentation for testing
    std::vector<float> mock_output(
        config_.num_classes * config_.input_height * config_.input_width, 0.1f
    );

    // For now, use a simple heuristic: top half = ceiling, bottom half = floor
    // This allows testing without a real model
    for (int y = 0; y < config_.input_height; ++y) {
        for (int x = 0; x < config_.input_width; ++x) {
            int floor_idx = static_cast<int>(SemanticClass::FLOOR) *
                           config_.input_height * config_.input_width +
                           y * config_.input_width + x;
            int wall_idx = static_cast<int>(SemanticClass::WALL) *
                          config_.input_height * config_.input_width +
                          y * config_.input_width + x;
            int ceiling_idx = static_cast<int>(SemanticClass::CEILING) *
                             config_.input_height * config_.input_width +
                             y * config_.input_width + x;

            // Simple depth-based heuristic
            if (y < config_.input_height / 3) {
                mock_output[ceiling_idx] = 0.8f;
            } else if (y > 2 * config_.input_height / 3) {
                mock_output[floor_idx] = 0.8f;
            } else {
                mock_output[wall_idx] = 0.8f;
            }
        }
    }

    // Postprocess
    SegmentationResult result = postprocess(
        mock_output, rgb_image.cols, rgb_image.rows
    );

    auto end = std::chrono::high_resolution_clock::now();
    result.inference_time_ms = std::chrono::duration<double, std::milli>(end - start).count();

    return result;
}

SegmentationResult Segmenter::segmentWithDepth(
    const cv::Mat& rgb_image,
    const cv::Mat& depth_image
) {
    SegmentationResult result = segment(rgb_image);

    if (!depth_image.empty() &&
        depth_image.rows == result.labels.rows &&
        depth_image.cols == result.labels.cols) {
        auto depth_at = [&](int y, int x) -> float {
            if (depth_image.type() == CV_32FC1) {
                return depth_image.at<float>(y, x);
            }
            if (depth_image.type() == CV_16UC1) {
                return static_cast<float>(depth_image.at<uint16_t>(y, x)) / 1000.0f;
            }
            return -1.0f;
        };

        for (int y = 0; y < result.labels.rows; ++y) {
            for (int x = 0; x < result.labels.cols; ++x) {
                float depth = depth_at(y, x);
                if (depth < 0.1f || depth > 10.0f) {
                    result.confidences.at<float>(y, x) *= 0.5f;
                }
            }
        }
    }

    return result;
}

std::string Segmenter::getClassName(uint16_t label) {
    if (label < static_cast<uint16_t>(SemanticClass::NUM_CLASSES)) {
        return CLASS_NAMES[label];
    }
    return "unknown";
}

cv::Scalar Segmenter::getClassColor(uint16_t label) {
    if (label < static_cast<uint16_t>(SemanticClass::NUM_CLASSES)) {
        return CLASS_COLORS[label];
    }
    return CLASS_COLORS[0];  // unknown gray
}

} // namespace voxelnav
