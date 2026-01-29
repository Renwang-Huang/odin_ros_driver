#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <sys/stat.h>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
#include <string>
#include "depth_image.hpp"
#include <rcpputils/filesystem_helper.hpp>

bool fileExists(const std::string& filename) {
    struct stat buffer;
    return (stat(filename.c_str(), &buffer) == 0);
}

bool loadCalibParameters(std::shared_ptr<rclcpp::Node> node, const std::string& calib_file_path) {
    try {
        YAML::Node config = YAML::LoadFile(calib_file_path);

        if (config["cam_num"]) {
            int cam_num = config["cam_num"].as<int>();
            node->declare_parameter("cam_num", cam_num);
        }
        if (config["img_topic_0"]) {
            std::string img_topic = config["img_topic_0"].as<std::string>();
            node->declare_parameter("img_topic_0", img_topic);
        }
        if (config["Tcl_0"]) {
            std::vector<double> tcl_matrix = config["Tcl_0"].as<std::vector<double>>();
            node->declare_parameter("Tcl_0", tcl_matrix);
        }
        if (config["cam_0"]) {
            YAML::Node cam_0 = config["cam_0"];
            if (cam_0["cam_model"]) {
                node->declare_parameter("cam_0.cam_model", cam_0["cam_model"].as<std::string>());
            }
            if (cam_0["image_width"]) {
                node->declare_parameter("cam_0.image_width", cam_0["image_width"].as<int>());
            }
            if (cam_0["image_height"]) {
                node->declare_parameter("cam_0.image_height", cam_0["image_height"].as<int>());
            }
            for (const auto& param : {"k2", "k3", "k4", "k5", "k6", "k7", "p1", "p2"}) {
                if (cam_0[param]) {
                    node->declare_parameter(std::string("cam_0.") + param, cam_0[param].as<double>());
                }
            }
            for (const auto& param : {"A11", "A12", "A22", "u0", "v0"}) {
                if (cam_0[param]) {
                    node->declare_parameter(std::string("cam_0.") + param, cam_0[param].as<double>());
                }
            }
            if (cam_0["isFast"]) {
                node->declare_parameter("cam_0.isFast", cam_0["isFast"].as<int>());
            }
            if (cam_0["numDiff"]) {
                node->declare_parameter("cam_0.numDiff", cam_0["numDiff"].as<int>());
            }
            if (cam_0["maxIncidentAngle"]) {
                node->declare_parameter("cam_0.maxIncidentAngle", cam_0["maxIncidentAngle"].as<int>());
            }
        }

        RCLCPP_INFO(node->get_logger(), "Calib loaded: %s", calib_file_path.c_str());
        return true;
        
    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(node->get_logger(), "YAML parsing error: %s", e.what());
        return false;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Error loading calib parameters: %s", e.what());
        return false;
    }
}
std::string get_package_source_directory() {
    // 使用 rcpputils::fs::path 替代 std::filesystem::path
    rcpputils::fs::path current_file(__FILE__);
    
    // 回溯到包根目录
    auto path = current_file.parent_path();
    
    // 使用 rcpputils::fs::exists 替代 std::filesystem::exists
    while (!path.empty() && !rcpputils::fs::exists(path / "package.xml")) {
        path = path.parent_path();
    }
    
    if (path.empty()) {
        throw std::runtime_error("Failed to locate package root directory");
    }
    
    return path.string();
}
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    

    auto node = std::make_shared<rclcpp::Node>("pcd2depth_node");
    std::string package_path = get_package_source_directory();
    std::string config_file = package_path + "/config/control_command.yaml";

    YAML::Node config = YAML::LoadFile(config_file);
    if (!config["register_keys"]) {
        throw std::runtime_error("Missing 'register_keys' section");
    }
    if (!config["register_keys"]["senddepth"]) {
        throw std::runtime_error("Missing 'senddepth' parameter");
    }
    int senddepth = config["register_keys"]["senddepth"].as<int>();
    if (senddepth == 0) {
        RCLCPP_INFO(node->get_logger(), "senddepth=0, depth publishing disabled");
        return 0;
    }

    std::string calib_file_path = package_path + "/config/calib.yaml";
    if (!fileExists(calib_file_path)) {
        RCLCPP_INFO(node->get_logger(), "Waiting for calib.yaml at %s ...", calib_file_path.c_str());
    }
    while (rclcpp::ok() && !fileExists(calib_file_path)) {
        RCLCPP_INFO_THROTTLE(node->get_logger(), *node->get_clock(), 5000, "Waiting for calib.yaml ...");
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        rclcpp::spin_some(node);
    }
    if (!rclcpp::ok()) {
        return 0;
    }

    if (!loadCalibParameters(node, calib_file_path)) {
        RCLCPP_ERROR(node->get_logger(), "Failed to load calib: %s", calib_file_path.c_str());
        return 1;
    }

    rclcpp::NodeOptions depth_node_options;
    std::vector<rclcpp::Parameter> params_override;
    try {
        params_override.push_back(rclcpp::Parameter("cam_0.image_width", node->get_parameter("cam_0.image_width").as_int()));
        params_override.push_back(rclcpp::Parameter("cam_0.image_height", node->get_parameter("cam_0.image_height").as_int()));
        params_override.push_back(rclcpp::Parameter("cam_0.A11", node->get_parameter("cam_0.A11").as_double()));
        params_override.push_back(rclcpp::Parameter("cam_0.A12", node->get_parameter("cam_0.A12").as_double()));
        params_override.push_back(rclcpp::Parameter("cam_0.A22", node->get_parameter("cam_0.A22").as_double()));
        params_override.push_back(rclcpp::Parameter("cam_0.u0", node->get_parameter("cam_0.u0").as_double()));
        params_override.push_back(rclcpp::Parameter("cam_0.v0", node->get_parameter("cam_0.v0").as_double()));
        params_override.push_back(rclcpp::Parameter("cam_0.k2", node->get_parameter("cam_0.k2").as_double()));
        params_override.push_back(rclcpp::Parameter("cam_0.k3", node->get_parameter("cam_0.k3").as_double()));
        params_override.push_back(rclcpp::Parameter("cam_0.k4", node->get_parameter("cam_0.k4").as_double()));
        params_override.push_back(rclcpp::Parameter("cam_0.k5", node->get_parameter("cam_0.k5").as_double()));
        params_override.push_back(rclcpp::Parameter("cam_0.k6", node->get_parameter("cam_0.k6").as_double()));
        params_override.push_back(rclcpp::Parameter("cam_0.k7", node->get_parameter("cam_0.k7").as_double()));
        params_override.push_back(rclcpp::Parameter("Tcl_0", node->get_parameter("Tcl_0").as_double_array()));
        depth_node_options.parameter_overrides(params_override);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Parameter prepare failed: %s", e.what());
        return 1;
    }

    try {
        auto depth_node = std::make_shared<DepthImageRos2Node>(depth_node_options);
        depth_node->initialize();
        RCLCPP_INFO(node->get_logger(), "pcd2depth node running");
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(depth_node);
        executor.spin();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Depth node error: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}
