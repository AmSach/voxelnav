// VoxelNav - Semantic Segmentation Module
// SegFormer ONNX Runtime wrapper with deterministic fallback

#include "voxelnav/segmenter.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>

#include "onnxruntime_cxx_api.h"

namespace voxelnav {

static const char* INTERNAL_CLASS_NAMES[] = {
    "unknown", "floor", "wall", "ceiling", "door", "window", "furniture", "person", "vehicle", "plant", "stairs", "table", "chair", "box"
};

static const cv::Scalar INTERNAL_CLASS_COLORS[] = {
    cv::Scalar(128, 128, 128),
    cv::Scalar(128, 0, 128),
    cv::Scalar(255, 0, 0),
    cv::Scalar(0, 255, 255),
    cv::Scalar(0, 255, 0),
    cv::Scalar(255, 255, 0),
    cv::Scalar(0, 128, 255),
    cv::Scalar(0, 0, 255),
    cv::Scalar(255, 0, 255),
    cv::Scalar(0, 255, 128),
    cv::Scalar(128, 64, 0),
    cv::Scalar(255, 128, 128),
    cv::Scalar(128, 255, 128),
    cv::Scalar(128, 128, 255)
};

struct Segmenter::Impl {
    Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "voxelnav"};
    std::unique_ptr<Ort::Session> session;
    Ort::SessionOptions session_options;
    bool loaded = false;
    bool using_fallback = true;
    std::vector<const char*> input_names;
    std::vector<const char*> output_names;
    std::string input_name_storage;
    std::string output_name_storage;
    std::vector<int64_t> input_shape;
    std::vector<int64_t> output_shape;
};

Segmenter::Segmenter(const SegmenterConfig& config)
    : config_(config), initialized_(false), impl_(std::make_unique<Impl>()) {}

Segmenter::~Segmenter() = default;

bool Segmenter::hasRealModel() const {
    return impl_ && impl_->loaded && !impl_->using_fallback && impl_->session != nullptr;
}

uint16_t Segmenter::remapRawLabel(int raw_label) const {
    switch (raw_label) {
        case 0: return 2;   // wall
        case 1: return 2;   // building -> wall-ish
        case 2: return 0;   // sky
        case 3: return 1;   // floor
        case 4: return 9;   // tree -> plant-ish
        case 5: return 3;   // ceiling
        case 6: return 1;   // road -> floor/path-ish
        case 7: return 6;   // bed -> furniture
        case 8: return 5;   // windowpane
        case 9: return 9;   // grass -> plant-ish
        case 10: return 6;  // cabinet
        case 11: return 1;  // sidewalk -> floor/path-ish
        case 12: return 7;  // person
        case 14: return 4;  // door
        case 15: return 11; // table
        case 17: return 9;  // plant
        case 19: return 12; // chair
        case 20: return 8;  // car
        case 23: return 6;  // sofa
        case 24: return 6;  // shelf
        case 32: return 0;  // fence
        case 33: return 6;  // desk
        case 35: return 6;  // wardrobe
        case 36: return 6;  // lamp
        case 37: return 6;  // bathtub
        case 41: return 13; // box
        case 44: return 6;  // chest of drawers
        case 47: return 6;  // sink
        case 49: return 6;  // fireplace
        case 50: return 6;  // refrigerator
        case 53: return 10; // stairs
        case 56: return 11; // pool table
        case 57: return 6;  // pillow
        case 58: return 4;  // screen door
        case 59: return 10; // stairway
        case 61: return 0;  // bridge
        case 62: return 6;  // bookcase
        case 64: return 11; // coffee table
        case 65: return 6;  // toilet
        case 66: return 9;  // flower
        case 69: return 6;  // bench
        case 71: return 6;  // stove
        case 72: return 9;  // palm
        case 73: return 6;  // kitchen island
        case 74: return 6;  // computer
        case 75: return 12; // swivel chair
        case 80: return 8;  // bus
        case 83: return 8;  // truck
        case 84: return 0;  // tower
        case 89: return 6;  // television receiver
        case 90: return 8;  // airplane
        case 96: return 10; // escalator
        case 97: return 6;  // ottoman
        case 98: return 6;  // bottle
        case 99: return 6;  // buffet
        case 102: return 8; // van
        case 103: return 8; // ship
        case 107: return 6; // washer
        case 108: return 6; // plaything
        case 110: return 12; // stool
        case 111: return 6; // barrel
        case 112: return 6; // basket
        case 114: return 0; // tent
        case 115: return 6; // bag
        case 116: return 8; // minibike
        case 117: return 6; // cradle
        case 118: return 6; // oven
        case 120: return 6; // food
        case 121: return 10; // step
        case 122: return 8; // tank
        case 124: return 6; // microwave
        case 125: return 6; // pot
        case 127: return 8; // bicycle
        case 129: return 6; // dishwasher
        case 130: return 6; // screen
        case 131: return 6; // blanket
        case 132: return 6; // sculpture
        case 135: return 6; // vase
        case 143: return 6; // monitor
        case 145: return 6; // shower
        case 146: return 6; // radiator
        case 148: return 6; // clock
        default: return 0;
    }
}

bool Segmenter::tryLoadRealModel() {
    std::ifstream model_file(config_.model_path, std::ios::binary);
    if (!model_file.good()) {
        return false;
    }
    model_file.close();

    try {
        impl_->session_options.SetIntraOpNumThreads(1);
        impl_->session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        impl_->session = std::make_unique<Ort::Session>(impl_->env, config_.model_path.c_str(), impl_->session_options);

        Ort::AllocatorWithDefaultOptions allocator;
        auto input_name = impl_->session->GetInputNameAllocated(0, allocator);
        auto output_name = impl_->session->GetOutputNameAllocated(0, allocator);
        impl_->input_name_storage = input_name.get();
        impl_->output_name_storage = output_name.get();
        impl_->input_names = {impl_->input_name_storage.c_str()};
        impl_->output_names = {impl_->output_name_storage.c_str()};

        auto input_type = impl_->session->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo();
        auto output_type = impl_->session->GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo();
        impl_->input_shape = input_type.GetShape();
        impl_->output_shape = output_type.GetShape();
        impl_->loaded = true;
        impl_->using_fallback = false;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[VoxelNav] Real model load failed: " << e.what() << std::endl;
        impl_->session.reset();
        impl_->loaded = false;
        impl_->using_fallback = true;
        return false;
    }
}

bool Segmenter::initialize() {
    if (initialized_) {
        return true;
    }

    const bool loaded = tryLoadRealModel();
    if (loaded) {
        std::cout << "[VoxelNav] Real segmentation model loaded: " << config_.model_path << std::endl;
    } else {
        if (!config_.allow_fallback) {
            std::cerr << "[VoxelNav] No valid segmentation model available." << std::endl;
            return false;
        }
        std::cerr << "[VoxelNav] Using deterministic fallback segmenter instead of a real model." << std::endl;
    }

    initialized_ = true;
    return true;
}

cv::Mat Segmenter::preprocess(const cv::Mat& image) const {
    cv::Mat resized;
    cv::resize(image, resized, cv::Size(config_.input_width, config_.input_height));
    if (resized.channels() == 1) {
        cv::cvtColor(resized, resized, cv::COLOR_GRAY2BGR);
    }
    cv::Mat float_img;
    resized.convertTo(float_img, CV_32FC3, 1.0 / 255.0);
    return (float_img - cv::Scalar(0.485f, 0.456f, 0.406f)) / cv::Scalar(0.229f, 0.224f, 0.225f);
}

SegmentationResult Segmenter::fallbackSegment(const cv::Mat& rgb_image) const {
    auto start = std::chrono::high_resolution_clock::now();
    const int h = config_.input_height;
    const int w = config_.input_width;
    cv::Mat labels(h, w, CV_16UC1);
    cv::Mat confidences(h, w, CV_32FC1);

    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            uint16_t label = 0;
            float confidence = 0.82f;
            if (y < h / 3) {
                label = 3;
                confidence = 0.90f;
            } else if (y > (2 * h) / 3) {
                label = 1;
                confidence = 0.93f;
            } else if ((x / 40) % 3 == 0) {
                label = 4;
                confidence = 0.78f;
            } else if ((x / 40) % 3 == 1) {
                label = 11;
                confidence = 0.75f;
            }
            labels.at<uint16_t>(y, x) = label;
            confidences.at<float>(y, x) = confidence;
        }
    }

    SegmentationResult result;
    cv::resize(labels, result.labels, cv::Size(rgb_image.cols, rgb_image.rows), 0, 0, cv::INTER_NEAREST);
    cv::resize(confidences, result.confidences, cv::Size(rgb_image.cols, rgb_image.rows), 0, 0, cv::INTER_LINEAR);
    for (uint16_t i = 0; i < 14; ++i) {
        result.unique_labels.push_back(i);
    }
    auto end = std::chrono::high_resolution_clock::now();
    result.inference_time_ms = std::chrono::duration<double, std::milli>(end - start).count();
    return result;
}

SegmentationResult Segmenter::decodeTensor(const std::vector<float>& logits, const std::vector<int64_t>& shape, int orig_width, int orig_height) const {
    SegmentationResult result;
    if (shape.size() < 4) {
        result.labels = cv::Mat(orig_height, orig_width, CV_16UC1, cv::Scalar(0));
        result.confidences = cv::Mat(orig_height, orig_width, CV_32FC1, cv::Scalar(0.1f));
        return result;
    }

    const int classes = static_cast<int>(shape[1]);
    const int h = static_cast<int>(shape[2]);
    const int w = static_cast<int>(shape[3]);
    cv::Mat label_map(h, w, CV_16UC1);
    cv::Mat conf_map(h, w, CV_32FC1);

    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            float best = -std::numeric_limits<float>::infinity();
            int best_class = 0;
            for (int c = 0; c < classes; ++c) {
                const float value = logits[static_cast<size_t>(c * h * w + y * w + x)];
                if (value > best) {
                    best = value;
                    best_class = c;
                }
            }
            label_map.at<uint16_t>(y, x) = remapRawLabel(best_class);
            conf_map.at<float>(y, x) = best;
        }
    }

    cv::resize(label_map, result.labels, cv::Size(orig_width, orig_height), 0, 0, cv::INTER_NEAREST);
    cv::resize(conf_map, result.confidences, cv::Size(orig_width, orig_height), 0, 0, cv::INTER_LINEAR);

    result.unique_labels = {0};
    for (int y = 0; y < result.labels.rows; ++y) {
        for (int x = 0; x < result.labels.cols; ++x) {
            const uint16_t label = result.labels.at<uint16_t>(y, x);
            if (result.confidences.at<float>(y, x) >= config_.confidence_threshold) {
                if (std::find(result.unique_labels.begin(), result.unique_labels.end(), label) == result.unique_labels.end()) {
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
    cv::Mat preprocessed = preprocess(rgb_image);

    if (hasRealModel()) {
        try {
            std::vector<float> input_tensor;
            input_tensor.reserve(static_cast<size_t>(config_.input_width * config_.input_height * 3));
            for (int y = 0; y < preprocessed.rows; ++y) {
                const cv::Vec3f* row = preprocessed.ptr<cv::Vec3f>(y);
                for (int x = 0; x < preprocessed.cols; ++x) {
                    input_tensor.push_back(row[x][2]);
                    input_tensor.push_back(row[x][1]);
                    input_tensor.push_back(row[x][0]);
                }
            }

            std::vector<int64_t> input_shape{1, 3, config_.input_height, config_.input_width};
            Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
            Ort::Value input = Ort::Value::CreateTensor<float>(memory_info, input_tensor.data(), input_tensor.size(), input_shape.data(), input_shape.size());
            auto outputs = impl_->session->Run(Ort::RunOptions{nullptr}, impl_->input_names.data(), &input, 1, impl_->output_names.data(), 1);
            auto& out = outputs[0];
            float* out_data = out.GetTensorMutableData<float>();
            auto type_info = out.GetTensorTypeAndShapeInfo();
            std::vector<int64_t> out_shape = type_info.GetShape();
            const size_t total = type_info.GetElementCount();
            std::vector<float> logits(out_data, out_data + total);
            SegmentationResult result = decodeTensor(logits, out_shape, rgb_image.cols, rgb_image.rows);
            auto end = std::chrono::high_resolution_clock::now();
            result.inference_time_ms = std::chrono::duration<double, std::milli>(end - start).count();
            return result;
        } catch (const std::exception& e) {
            std::cerr << "[VoxelNav] Real inference failed, falling back: " << e.what() << std::endl;
        }
    }

    SegmentationResult result = fallbackSegment(rgb_image);
    auto end = std::chrono::high_resolution_clock::now();
    result.inference_time_ms = std::chrono::duration<double, std::milli>(end - start).count();
    return result;
}

SegmentationResult Segmenter::segmentWithDepth(const cv::Mat& rgb_image, const cv::Mat& depth_image) {
    SegmentationResult result = segment(rgb_image);
    if (!depth_image.empty() && depth_image.rows == result.labels.rows && depth_image.cols == result.labels.cols) {
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
                const float depth = depth_at(y, x);
                if (depth < 0.1f || depth > 10.0f) {
                    result.confidences.at<float>(y, x) *= 0.5f;
                }
            }
        }
    }
    return result;
}

std::string Segmenter::getClassName(uint16_t label) {
    if (label < 14) {
        return INTERNAL_CLASS_NAMES[label];
    }
    return "unknown";
}

cv::Scalar Segmenter::getClassColor(uint16_t label) {
    if (label < 14) {
        return INTERNAL_CLASS_COLORS[label % 14];
    }
    return INTERNAL_CLASS_COLORS[0];
}

} // namespace voxelnav
