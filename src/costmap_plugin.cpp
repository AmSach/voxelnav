// VoxelNav - Nav2 Costmap Plugin
// Provides voxel-based obstacle layer for Nav2

#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include "voxelnav/voxelizer.hpp"

namespace voxelnav {

class VoxelCostmapPlugin : public nav2_costmap_2d::Layer {
public:
    VoxelCostmapPlugin()
        : nav2_costmap_2d::Layer()
        , enabled_(true)
    {
    }
    
    virtual ~VoxelCostmapPlugin() = default;
    
    void onInitialize() override {
        auto node = node_.lock();
        if (!node) {
            return;
        }
        
        declareParameter("enabled", true);
        declareParameter("voxel_size", 0.05);
        declareParameter("obstacle_threshold", 0.5);
        declareParameter("filter_labels", std::vector<int64_t>{});
        
        getParameters();
        
        RCLCPP_INFO(node->get_logger(),
            "VoxelCostmapPlugin initialized. Voxel size: %.3f", voxel_size_);
    }
    
    void updateBounds(
        double robot_x, double robot_y, double robot_yaw,
        double* min_x, double* min_y, 
        double* max_x, double* max_y) override 
    {
        if (!enabled_) return;
        
        // Update bounds based on voxel grid extent
        *min_x = std::min(*min_x, -grid_extent_);
        *min_y = std::min(*min_y, -grid_extent_);
        *max_x = std::max(*max_x, grid_extent_);
        *max_y = std::max(*max_y, grid_extent_);
    }
    
    void updateCosts(
        nav2_costmap_2d::Costmap2D& master_grid,
        int min_i, int min_j, 
        int max_i, int max_j) override 
    {
        if (!enabled_) return;
        
        auto node = node_.lock();
        if (!node) return;
        
        std::lock_guard<std::recursive_mutex> lock(*getMutex());
        
        // Get voxels from the voxelizer
        auto voxels = voxelizer_->getVoxels();
        
        for (const Voxel& v : voxels) {
            // Skip filtered labels
            if (!filter_labels_.empty()) {
                if (std::find(filter_labels_.begin(), filter_labels_.end(), 
                              static_cast<int64_t>(v.label)) != filter_labels_.end()) {
                    continue;
                }
            }
            
            // Convert voxel to costmap coordinates
            unsigned int mx, my;
            double wx = v.x * voxel_size_;
            double wy = v.y * voxel_size_;
            
            if (!master_grid.worldToMap(wx, wy, mx, my)) {
                continue;
            }
            
            // Set cost based on voxel properties
            unsigned char cost;
            if (v.confidence >= obstacle_threshold_) {
                cost = nav2_costmap_2d::LETHAL_OBSTACLE;
            } else if (v.confidence >= obstacle_threshold_ * 0.5) {
                cost = nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
            } else {
                cost = nav2_costmap_2d::FREE_SPACE;
            }
            
            // Only update if higher cost
            unsigned char current_cost = master_grid.getCost(mx, my);
            if (cost > current_cost) {
                master_grid.setCost(mx, my, cost);
            }
        }
    }
    
    void reset() override {
        if (voxelizer_) {
            voxelizer_->clear();
        }
    }
    
    bool isClearable() override {
        return true;
    }
    
    void activate() override {
        enabled_ = true;
    }
    
    void deactivate() override {
        enabled_ = false;
    }
    
    // Set the voxelizer (called from main node)
    void setVoxelizer(std::shared_ptr<Voxelizer> voxelizer) {
        voxelizer_ = voxelizer;
    }

private:
    void getParameters() {
        auto node = node_.lock();
        if (!node) return;
        
        node->get_parameter(name_ + ".enabled", enabled_);
        node->get_parameter(name_ + ".voxel_size", voxel_size_);
        node->get_parameter(name_ + ".obstacle_threshold", obstacle_threshold_);
        
        std::vector<int64_t> filter_labels_int;
        node->get_parameter(name_ + ".filter_labels", filter_labels_int);
        filter_labels_ = filter_labels_int;
    }
    
    bool enabled_;
    double voxel_size_ = 0.05;
    double obstacle_threshold_ = 0.5;
    double grid_extent_ = 10.0;
    std::vector<int64_t> filter_labels_;
    std::shared_ptr<Voxelizer> voxelizer_;
};

} // namespace voxelnav

PLUGINLIB_EXPORT_CLASS(voxelnav::VoxelCostmapPlugin, nav2_costmap_2d::Layer)