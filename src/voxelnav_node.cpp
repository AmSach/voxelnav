// VoxelNav - ROS2 Main Node
// Real-time semantic voxel mapping

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <cmath>
#include <chrono>

#include "voxelnav/voxelizer.hpp"
#include "voxelnav/segmenter.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace voxelnav {

class VoxelNavNode : public rclcpp::Node {
public:
    explicit VoxelNavNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
        : Node("voxelnav_node", options)
    {
        declare_parameter("voxel_size", 0.05);
        declare_parameter("max_range", 10.0);
        declare_parameter("enable_semantic", true);
        declare_parameter("publish_rate", 10.0);
        declare_parameter("frame_id", "map");
        declare_parameter("model_path", "models/segformer-b0-finetuned-ade-512-512/onnx/model_fp16.onnx");
        declare_parameter("cloud_topic", "/camera/camera/depth/color/points");
        declare_parameter("rgb_topic", "/camera/camera/color/image_raw");
        declare_parameter("depth_topic", "/camera/camera/depth/image_rect_raw");
        declare_parameter("camera_info_topic", "/camera/camera/color/camera_info");

        voxel_size_ = get_parameter("voxel_size").as_double();
        max_range_ = get_parameter("max_range").as_double();
        enable_semantic_ = get_parameter("enable_semantic").as_bool();
        double publish_rate = get_parameter("publish_rate").as_double();
        frame_id_ = get_parameter("frame_id").as_string();
        model_path_ = get_parameter("model_path").as_string();
        cloud_topic_ = get_parameter("cloud_topic").as_string();
        rgb_topic_ = get_parameter("rgb_topic").as_string();
        depth_topic_ = get_parameter("depth_topic").as_string();
        camera_info_topic_ = get_parameter("camera_info_topic").as_string();

        if (cloud_topic_.empty()) cloud_topic_ = "/camera/camera/depth/color/points";
        if (rgb_topic_.empty()) rgb_topic_ = "/camera/camera/color/image_raw";
        if (depth_topic_.empty()) depth_topic_ = "/camera/camera/depth/image_rect_raw";
        if (camera_info_topic_.empty()) camera_info_topic_ = "/camera/camera/color/camera_info";

        voxelizer_ = std::make_unique<Voxelizer>(createVoxelizerConfig());
        segmenter_ = std::make_unique<Segmenter>(createSegmenterConfig());

        if (enable_semantic_) {
            if (!segmenter_->initialize()) {
                RCLCPP_WARN(get_logger(),
                    "Failed to initialize segmenter. Running in geometry-only mode.");
                enable_semantic_ = false;
            }
        }

        RCLCPP_INFO(get_logger(),
            "Segmenter mode: %s",
            segmenter_->hasRealModel() ? "real ONNX model" : "deterministic fallback");
        RCLCPP_INFO(get_logger(),
            "SegFormer input: %dx%d, classes: %d",
            segmenter_->getConfig().input_width, segmenter_->getConfig().input_height, segmenter_->getConfig().num_classes);

        cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic_, rclcpp::SensorDataQoS(),
            std::bind(&VoxelNavNode::cloudCallback, this, std::placeholders::_1));

        rgb_sub_.subscribe(this, rgb_topic_);
        depth_sub_.subscribe(this, depth_topic_);
        camera_info_sub_.subscribe(this, camera_info_topic_);

        sync_ = std::make_shared<message_filters::TimeSynchronizer<
            sensor_msgs::msg::Image,
            sensor_msgs::msg::Image,
            sensor_msgs::msg::CameraInfo>>(rgb_sub_, depth_sub_, camera_info_sub_, 10);

        sync_->registerCallback(std::bind(
            &VoxelNavNode::rgbdCallback, this,
            std::placeholders::_1,
            std::placeholders::_2,
            std::placeholders::_3));

        voxel_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "/voxel_map", rclcpp::QoS(10).transient_local());

        semantic_voxel_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "/semantic_voxel_map", rclcpp::QoS(10).transient_local());

        publish_timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / publish_rate),
            std::bind(&VoxelNavNode::publishVoxels, this));

        RCLCPP_INFO(get_logger(),
            "VoxelNav node initialized. Voxel size: %.3fm, Max range: %.1fm, Semantic: %s",
            voxel_size_, max_range_, enable_semantic_ ? "ON" : "OFF");
    }

private:
    VoxelGridConfig createVoxelizerConfig() {
        VoxelGridConfig config;
        config.voxel_size = static_cast<float>(voxel_size_);
        config.max_range = static_cast<float>(max_range_);
        return config;
    }

    SegmenterConfig createSegmenterConfig() {
        SegmenterConfig config;
        config.model_path = model_path_;
        config.confidence_threshold = 0.5f;
        config.input_width = 512;
        config.input_height = 512;
        return config;
    }

    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        RCLCPP_DEBUG(get_logger(), "Received point cloud with %d points",
            msg->width * msg->height);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        std::vector<Eigen::Vector3f> points;
        points.reserve(cloud->size());

        for (const auto& point : cloud->points) {
            if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z)) {
                points.emplace_back(point.x, point.y, point.z);
            }
        }

        if (voxelizer_->processPointCloud(points)) {
            stats_clouds_processed_++;
            stats_total_points_ += points.size();
        }
    }

    void rgbdCallback(
        const sensor_msgs::msg::Image::SharedPtr rgb_msg,
        const sensor_msgs::msg::Image::SharedPtr depth_msg,
        const sensor_msgs::msg::CameraInfo::SharedPtr info_msg
    ) {
        if (!enable_semantic_) {
            return;
        }

        RCLCPP_DEBUG(get_logger(), "Received RGB-D frame");

        cv_bridge::CvImageConstPtr rgb_ptr;
        cv_bridge::CvImageConstPtr depth_ptr;

        try {
            rgb_ptr = cv_bridge::toCvShare(rgb_msg);
            depth_ptr = cv_bridge::toCvShare(depth_msg);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(get_logger(), "CV bridge error: %s", e.what());
            return;
        }

        SegmentationResult seg_result = segmenter_->segmentWithDepth(
            rgb_ptr->image, depth_ptr->image);

        std::vector<Eigen::Vector3f> points;
        std::vector<uint16_t> labels;
        std::vector<float> confidences;

        float fx = info_msg->k[0];
        float fy = info_msg->k[4];
        float cx = info_msg->k[2];
        float cy = info_msg->k[5];

        const cv::Mat& depth = depth_ptr->image;
        auto depth_at = [&](int y, int x) -> float {
            if (depth.type() == CV_32FC1) {
                return depth.at<float>(y, x);
            }
            if (depth.type() == CV_16UC1) {
                return static_cast<float>(depth.at<uint16_t>(y, x)) / 1000.0f;
            }
            return -1.0f;
        };

        for (int y = 0; y < depth.rows; ++y) {
            for (int x = 0; x < depth.cols; ++x) {
                float d = depth_at(y, x);

                if (d < 0.1f || d > max_range_) continue;

                float z = d;
                float x_3d = (x - cx) * z / fx;
                float y_3d = (y - cy) * z / fy;

                points.emplace_back(x_3d, y_3d, z);
                labels.push_back(seg_result.labels.at<uint16_t>(y, x));
                confidences.push_back(seg_result.confidences.at<float>(y, x));
            }
        }

        voxelizer_->processPointCloud(points, labels, confidences);
        stats_frames_processed_++;
    }

    void publishVoxels() {
        auto voxels = voxelizer_->getVoxels();

        if (voxels.empty()) {
            return;
        }

        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        cloud.reserve(voxels.size());

        for (const Voxel& v : voxels) {
            Eigen::Vector3f world_pos = voxelizer_->voxelToWorld(v);

            pcl::PointXYZRGB point;
            point.x = world_pos.x();
            point.y = world_pos.y();
            point.z = world_pos.z();

            cv::Scalar color = Segmenter::getClassColor(v.label);
            point.r = static_cast<uint8_t>(color[2]);
            point.g = static_cast<uint8_t>(color[1]);
            point.b = static_cast<uint8_t>(color[0]);

            cloud.push_back(point);
        }

        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(cloud, output);
        output.header.frame_id = frame_id_;
        output.header.stamp = now();

        voxel_pub_->publish(output);
        if (enable_semantic_ && semantic_voxel_pub_) {
            semantic_voxel_pub_->publish(output);
        }

        if (stats_clouds_processed_ % 100 == 0) {
            RCLCPP_INFO(get_logger(),
                "Stats: %lu clouds, %lu frames, %lu voxels, %.1f MB",
                stats_clouds_processed_, stats_frames_processed_,
                voxels.size(), voxelizer_->getGridMemoryMB());
        }
    }

    float voxel_size_ = 0.05f;
    float max_range_ = 10.0f;
    bool enable_semantic_ = true;
    std::string frame_id_ = "map";
    std::string model_path_;
    std::string cloud_topic_;
    std::string rgb_topic_;
    std::string depth_topic_;
    std::string camera_info_topic_;

    std::unique_ptr<Voxelizer> voxelizer_;
    std::unique_ptr<Segmenter> segmenter_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> camera_info_sub_;
    std::shared_ptr<message_filters::TimeSynchronizer<
        sensor_msgs::msg::Image,
        sensor_msgs::msg::Image,
        sensor_msgs::msg::CameraInfo>> sync_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr voxel_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr semantic_voxel_pub_;

    rclcpp::TimerBase::SharedPtr publish_timer_;

    size_t stats_clouds_processed_ = 0;
    size_t stats_frames_processed_ = 0;
    size_t stats_total_points_ = 0;
};

} // namespace voxelnav

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<voxelnav::VoxelNavNode>());
    rclcpp::shutdown();
    return 0;
}
