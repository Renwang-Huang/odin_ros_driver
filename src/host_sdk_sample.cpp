#include "host_sdk_sample.h"
#include "yaml_parser.h"
#include "rawCloudRender.h"
#include <filesystem> 
#include <thread>
#include <string>
#include <stdexcept>
#include <atomic>
#include <mutex>
#include <memory>
#include <opencv2/opencv.hpp>
#include <deque> 
#include <unistd.h> 
#include <cstdlib>
#include <cstring>
#include <sys/types.h>
#include <sys/wait.h>
#include <signal.h>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <vector>
#include <cstdio>
#include <array>
#include <system_error>
// #include <yaml-cpp/yaml.h>
#include <iomanip>
#include <sstream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#define ros_driver_version "0.8.0"
#define required_firmware_version_major 0
#define required_firmware_version_minor 9
#define required_firmware_version_patch 0

// Unified logger and terminal output
static const rclcpp::Logger& odin_logger() {
    static const rclcpp::Logger log = rclcpp::get_logger("odin");
    return log;
}
#define ODIN_SEP    "-------------------------------------------"
#define ODIN_BANNER "=============== Odin 驱动 ==============="

// Global variable declarations
static device_handle odinDevice = nullptr;
static std::atomic<bool> deviceConnected(false);
static std::atomic<bool> deviceDisconnected(false);  // Device disconnection flag
static std::mutex device_mutex;                      // Device operation mutex lock
static std::atomic<bool> g_connection_timeout(false);
static std::atomic<bool> g_usb_version_error(false);
static std::atomic<bool> g_shutdown_requested(false);  // Signal handler flag
std::shared_ptr<MultiSensorPublisher> g_ros_object = nullptr;

int g_log_level = LOG_LEVEL_INFO;
int g_show_fps = 0;  // FPS display toggle control

// Custom parameter monitoring
static std::atomic<bool> g_param_monitor_running(false);
static std::thread g_param_monitor_thread;

// Command file monitoring
static std::string g_command_file_path = "";

static std::mutex g_rgb_mutex;
static std::shared_ptr<cv::Mat> g_latest_bgr;
static uint64_t g_latest_rgb_timestamp = 0;
static bool g_has_rgb = false;
static capture_Image_List_t g_latest_rgb;
static bool g_renderer_initialized = false;
static std::shared_ptr<rawCloudRender> g_renderer = nullptr;
std::string calib_file_ = "";
static std::shared_ptr<odin_ros_driver::YamlParser> g_parser = nullptr;

 // usb device
static std::string TARGET_VENDOR = "2207";
static std::string TARGET_PRODUCT = "0019";
// Global configuration variables
int g_sendrgb = 1;
int g_sendimu = 1;
int g_senddtof = 1;
int g_sendodom = 1;
int g_send_odom_baselink_tf = 0;
int g_sendcloudslam = 0;
int g_sendcloudrender = 0;
int g_sendrgb_compressed = 0;
int g_sendrgb_undistort = 0;
int g_record_data = 0;
int g_devstatus_log = 0;
int g_pub_intensity_gray = 0;
int g_show_path = 0;
int g_show_camerapose = 0;
int g_strict_usb3_0_check = 0;
int g_use_host_ros_time = 0;
int g_save_log = 0;

std::filesystem::path log_root_dir_;
int g_custom_map_mode = 0;
bool g_relocalization_success_msg_printed = false;

std::string g_relocalization_map_abs_path = "";
std::string g_mapping_result_dest_dir = "";
std::string g_mapping_result_file_name = "";

const char* DEV_STATUS_CSV_FILE = "dev_status.csv";
FILE* dev_status_csv_file = nullptr;

std::filesystem::path map_root_dir_;

char driver_start_time[32];

typedef struct  {
    struct timespec start = {0, 0};
    struct timespec last = {0, 0};
    int count = 0;
    std::mutex fps_mutex;
} fpsHandle;

void update_count(fpsHandle* handle) {
    struct timespec now;
    std::lock_guard<std::mutex> lock(handle->fps_mutex);
    clock_gettime(CLOCK_MONOTONIC, &now);
    if (handle->count == 0) {
        handle->start = now;
    } else {
        handle->last = now;
    }
    handle->count++;
}

double cal_fps(fpsHandle* handle, const char* name, bool print = false) 
{
    std::lock_guard<std::mutex> lock(handle->fps_mutex);
    if (handle->count < 2) {
        return 0.0;
    }
    double elapsed = (handle->last.tv_sec - handle->start.tv_sec)
                   + (handle->last.tv_nsec - handle->start.tv_nsec) / 1e9;
    double fps = (handle->count - 1) / elapsed;
    if (print) {
        RCLCPP_INFO(odin_logger(), "%s FPS: %.1f（帧数: %d，耗时: %.2fs）", name, fps, handle->count, elapsed);
    }
    handle->start = handle->last;
    handle->count = 1;
    return fps;
}

static fpsHandle rgb_rx_fps;
static fpsHandle dtof_rx_fps;
static fpsHandle imu_rx_fps;
static fpsHandle slam_cloud_rx_fps;
static fpsHandle slam_odom_rx_fps;
static fpsHandle slam_odom_highfreq_rx_fps;

class RosNodeControlImpl : public RosNodeControlInterface {
    public:
        void setDtofSubframeODR(int interval) override {
            dtof_subframe_interval_time = interval;
        }
        
        int getDtofSubframeODR() const override {
            return dtof_subframe_interval_time;
        }

        void setUseHostRosTime(bool use_host_ros_time) override {
            pub_use_host_ros_time = use_host_ros_time;
        }

        bool useHostRosTime() const override {
            return pub_use_host_ros_time;
        }
    
        void setSendOdomBaseLinkTF(bool send_odom_baselink_tf) override {
            pub_odom_baselink_tf = send_odom_baselink_tf;
        }

        bool sendOdomBaseLinkTF() const override {
            return pub_odom_baselink_tf;
        }

    private:
        int dtof_subframe_interval_time = 0;
        bool pub_use_host_ros_time = false;
        bool pub_odom_baselink_tf = false;
    };
    
static RosNodeControlImpl g_rosNodeControlImpl;

RosNodeControlInterface* getRosNodeControl() {
    return &g_rosNodeControlImpl;
}

// Return resident memory (RSS) in **megabytes** for a given PID
double read_rss_mb(pid_t pid) {
    std::string path = "/proc/" + std::to_string(pid) + "/status";
    std::ifstream in(path);
    if (!in) return 0.0;
    std::string key;
    long kb = 0;
    while (in >> key) {
        if (key == "VmRSS:") {        // VmRSS is reported in kB
            in >> kb;
            break;
        }
        in.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
    return kb / 1024.0;               // convert to MB
}

double read_pss_mb(pid_t pid) {
    std::string path = "/proc/" + std::to_string(pid) + "/smaps_rollup";
    std::ifstream in(path);
    if (!in) return 0.0;
    std::string key;
    long kb = 0;
    while (in >> key) {
        if (key == "Pss:") {     // Proportional Set Size in kB
            in >> kb;
            break;
        }
        in.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
    return kb / 1024.0;          // convert to MB
}

// Recursively collect child PIDs of the given pid
void collect_children(pid_t pid, std::vector<pid_t>& all) {
    std::string task_path = "/proc/" + std::to_string(pid) +
                            "/task/" + std::to_string(pid) + "/children";
    std::ifstream in(task_path);
    pid_t child;
    while (in >> child) {
        all.push_back(child);
        collect_children(child, all);
    }
}

void clear_all_queues();

static bool convert_calib_to_cam_in_ex(const std::string& calib_path, const std::filesystem::path& out_path);

// Signal handler for Ctrl+C
static void signal_handler(int signum) {
        if (signum == SIGINT || signum == SIGTERM) {
        RCLCPP_INFO(odin_logger(), "%s\n  [关机] 收到信号 %d，正在退出...", ODIN_SEP, signum);

        g_shutdown_requested = true;

        // Stop custom parameter monitoring thread
        g_param_monitor_running = false;
        if (g_param_monitor_thread.joinable()) {
            g_param_monitor_thread.join();
        }

        // Close device
        if (odinDevice) {
            // Convert calib.yaml to cam_in_ex.txt at program end
            if (g_ros_object) {
                const std::filesystem::path out_path = g_ros_object->get_root_dir() / "image" / "cam_in_ex.txt";
                (void)convert_calib_to_cam_in_ex(calib_file_, out_path);

                RCLCPP_INFO(odin_logger(), "  统计: pose=%d cloud=%d image=%d",
                    g_ros_object->get_pose_index(), g_ros_object->get_cloud_index(), g_ros_object->get_image_index());
            }

            RCLCPP_INFO(odin_logger(), "  [关机] 正在关闭设备...");

            if (lidar_stop_stream(odinDevice, LIDAR_MODE_SLAM))
            {
                RCLCPP_WARN(odin_logger(), "  停止数据流失败");
            }
            odinDevice = nullptr;
        }

        // Deinitialize lidar system
        RCLCPP_INFO(odin_logger(), "  [关机] 正在反初始化雷达系统...");
        lidar_system_deinit();

        // Close CSV file
        if (dev_status_csv_file) {
            std::fflush(dev_status_csv_file);
            fclose(dev_status_csv_file);
            dev_status_csv_file = nullptr;
        }

        // Shutdown ROS
        rclcpp::shutdown();

        exit(0);
    }
}

// Custom parameter monitoring function
static void custom_parameter_monitor() {
    int last_save_map_val = -1;
    while (g_param_monitor_running && deviceConnected) {
        if (odinDevice) {
            if (g_custom_map_mode == 1) {
                int value = 0;
                int result = lidar_get_custom_parameter(odinDevice, "save_map", &value);

                if (result == 0) {
                    if (last_save_map_val == 1 && value == 0) {
                        auto now = std::chrono::system_clock::now();
                        std::time_t t = std::chrono::system_clock::to_time_t(now);
                        std::tm tm{};
                        #ifdef _WIN32
                            localtime_s(&tm, &t);
                        #else
                            localtime_r(&t, &tm);
                        #endif
                        char map_save_time[32];
                        std::strftime(map_save_time, sizeof(map_save_time), "%Y%m%d_%H%M%S", &tm);

                        std::string map_dir = g_mapping_result_dest_dir != "" ? g_mapping_result_dest_dir : map_root_dir_.string();
                        std::string map_name = g_mapping_result_file_name != "" ? g_mapping_result_file_name : "map_" + std::string(map_save_time) + ".bin";
                        RCLCPP_INFO(odin_logger(), "[建图] 地图已保存，正在传输到 %s/%s", map_dir.c_str(), map_name.c_str());
                        int ret = lidar_get_mapping_result(odinDevice, map_dir.c_str(), map_name.c_str());
                        if (ret < 0 ) {
                            RCLCPP_WARN(odin_logger(), "[建图] 获取地图失败");
                        } else if (ret == 0) {
                            RCLCPP_INFO(odin_logger(), "[建图] 传输已开始");
                        } else {
                            RCLCPP_WARN(odin_logger(), "[建图] 地图错误码: %d", ret);
                        }
                    }
                    last_save_map_val = value;

                } else if (result == -2) {
                    RCLCPP_INFO(odin_logger(), "[建图] 传输进行中，请稍后重试");
                } else {
                    RCLCPP_WARN(odin_logger(), "[建图] save_map 错误: %d", result);
                }
            }
        }

        // Sleep for 1 second (1Hz)
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

// Process command from file
static void process_command_file() {
    if (!std::filesystem::exists(g_command_file_path)) {
        return;
    }
    
    std::ifstream file(g_command_file_path);
    if (!file.is_open()) {
        return;
    }
    
    std::string line;
    if (std::getline(file, line)) {
        file.close();
        
        // Delete the command file after reading
        std::filesystem::remove(g_command_file_path);
        
        if (line.empty()) return;
        
        std::istringstream iss(line);
        std::string command, param_name, value_str;
        
        if (!(iss >> command >> param_name >> value_str)) {
            RCLCPP_WARN(odin_logger(), "[命令] 格式错误，用法: set <参数名> <值>");
            return;
        }
        
        if (command == "set") {
            if (!deviceConnected || !odinDevice) {
                RCLCPP_WARN(odin_logger(), "[命令] 设备未连接");
                return;
            }
            
            try {
                int value = std::stoi(value_str);
                int result = lidar_set_custom_parameter(odinDevice, param_name.c_str(), &value, sizeof(int));

                if (result == 0) {
                    RCLCPP_INFO(odin_logger(), "[命令] 已设置 %s = %d", param_name.c_str(), value);
                } else {
                    RCLCPP_ERROR(odin_logger(), "[命令] 设置 %s 失败: %d", param_name.c_str(), result);
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(odin_logger(), "[命令] 无效取值: %s", value_str.c_str());
            }
        } else {
            RCLCPP_WARN(odin_logger(), "[命令] 未知命令: %s", command.c_str());
        }
    } else {
        file.close();
    }
}

// detect USB3.0
bool isUsb3OrHigher(const std::string& vendorId, const std::string& productId) {
    std::string command = "lsusb -d " + vendorId + ":" + productId + " -v | grep 'bcdUSB'";
    
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(command.c_str(), "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    
    if (result.empty()) {
        RCLCPP_ERROR(odin_logger(), "[USB] 无法获取版本");
        return false;
    }
    
    // find bcdUSB
    size_t pos = result.find("bcdUSB");
    if (pos == std::string::npos) {
        RCLCPP_ERROR(odin_logger(), "[USB] 未找到 bcdUSB");
        return false;
    }
    
    std::string versionStr = result.substr(pos + 7); // "bcdUSB" + space
    float version = std::stof(versionStr);
    
    RCLCPP_INFO(odin_logger(), "[USB] 检测到版本 %.1f", version);
    if (!g_strict_usb3_0_check) {
        RCLCPP_INFO(odin_logger(), "[USB] 已关闭严格 USB3.0 检查");
        return true;
    }
    return version >= 3.0;
}

bool isUsbDevicePresent(const std::string& vendorId, const std::string& productId) {
    std::ifstream devicesList("/sys/bus/usb/devices");
    if (devicesList.is_open()) {
        std::string line;
        while (std::getline(devicesList, line)) {
            if (line.find('.') != std::string::npos) continue;
            if (line.empty()) continue;
            
            std::string vendorPath = "/sys/bus/usb/devices/" + line + "/idVendor";
            std::ifstream vendorFile(vendorPath);
            if (vendorFile.is_open()) {
                std::string vendorContent;
                if (std::getline(vendorFile, vendorContent)) {
                    vendorContent.erase(vendorContent.find_last_not_of(" \n\r\t") + 1);
                    
                    std::string productPath = "/sys/bus/usb/devices/" + line + "/idProduct";
                    std::ifstream productFile(productPath);
                    if (productFile.is_open()) {
                        std::string productContent;
                        if (std::getline(productFile, productContent)) {
                            productContent.erase(productContent.find_last_not_of(" \n\r\t") + 1);
                            
                            if (vendorContent == vendorId && productContent == productId) {
                                return true;
                            }
                        }
                        productFile.close();
                    }
                }
                vendorFile.close();
            }
        }
        devicesList.close();
    }
    return false;
}
// Convert calib.yaml to cam_in_ex.txt
static bool convert_calib_to_cam_in_ex(const std::string& calib_path, const std::filesystem::path& out_path) {
    try {
        if (calib_path.empty()) {
            RCLCPP_WARN(odin_logger(), "[设备] 标定文件为空，跳过 cam_in_ex.txt");
            return false;
        }

        YAML::Node root = YAML::LoadFile(calib_path);

        // Read Tcl_0 matrix (16 values)
        std::array<double, 16> Tcl{};
        YAML::Node tcl = root["Tcl_0"];
        for (size_t i = 0; i < 16; ++i) {
            if (tcl && tcl.IsSequence() && i < tcl.size()) {
                Tcl[i] = tcl[i].as<double>();
            } else {
                // Default last row to [0,0,0,1] if missing, others 0
                Tcl[i] = (i == 15) ? 1.0 : 0.0;
            }
        }

        // Read cam_0 parameters (with defaults)
        YAML::Node cam0 = root["cam_0"];
        auto get_i = [&](const char* key, int def) -> int {
            return (cam0 && cam0[key]) ? cam0[key].as<int>() : def;
        };
        auto get_d = [&](const char* key, double def) -> double {
            return (cam0 && cam0[key]) ? cam0[key].as<double>() : def;
        };

        int image_width = get_i("image_width", 0);
        int image_height = get_i("image_height", 0);
        double k2 = get_d("k2", 0.0);
        double k3 = get_d("k3", 0.0);
        double k4 = get_d("k4", 0.0);
        double k5 = get_d("k5", 0.0);
        double k6 = get_d("k6", 0.0);
        double k7 = get_d("k7", 0.0);
        double p1 = get_d("p1", 0.0);
        double p2 = get_d("p2", 0.0);
        double A11 = get_d("A11", 0.0);
        double A12 = get_d("A12", 0.0);
        double A22 = get_d("A22", 0.0);
        double u0 = get_d("u0", 0.0);
        double v0 = get_d("v0", 0.0);

        // Ensure parent directory exists
        std::error_code ec;
        std::filesystem::create_directories(out_path.parent_path(), ec);

        // Truncate file then write content
        std::ofstream ofs(out_path, std::ios::out | std::ios::trunc);
        if (!ofs.is_open()) {
            RCLCPP_ERROR(odin_logger(), "[设备] 无法写入 cam_in_ex.txt: %s", out_path.string().c_str());
            return false;
        }

        auto fmt = [](double v) {
            std::ostringstream ss; ss.setf(std::ios::fixed); ss << std::setprecision(6) << v; return ss.str();
        };

        // Write Tcl_0 with line breaks every 4 elements
        ofs << "Tcl_0: [";
        for (int i = 0; i < 16; ++i) {
            if (i > 0) {
                ofs << ", ";
                if (i % 4 == 0) ofs << "\n        ";
            }
            ofs << fmt(Tcl[i]);
        }
        ofs << "]\n";

        // Write cam_0 block
        ofs << "cam_0: \n";
        ofs << "   image_width: " << image_width << "\n";
        ofs << "   image_height: " << image_height << "\n";
        ofs << "   k2: " << fmt(k2) << "\n";
        ofs << "   k3: " << fmt(k3) << "\n";
        ofs << "   k4: " << fmt(k4) << "\n";
        ofs << "   k5: " << fmt(k5) << "\n";
        ofs << "   k6: " << fmt(k6) << "\n";
        ofs << "   k7: " << fmt(k7) << "\n";
        ofs << "   p1: " << fmt(p1) << "\n";
        ofs << "   p2: " << fmt(p2) << "\n";
        ofs << "   A11: " << fmt(A11) << "\n";
        ofs << "   A12: " << fmt(A12) << "\n";
        ofs << "   A22: " << fmt(A22) << "\n";
        ofs << "   u0: " << fmt(u0) << "\n";
        ofs << "   v0: " << fmt(v0) << "\n";

        ofs.flush();

        RCLCPP_INFO(odin_logger(), "[设备] 已写入 cam_in_ex.txt");
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(odin_logger(), "[设备] 转换 calib.yaml 失败: %s", e.what());
        return false;
    }
}

// Get package share path
std::string get_package_share_path(const std::string& package_name) {
    try {
        return ament_index_cpp::get_package_share_directory(package_name);
    } catch (const std::exception& e) {
        throw std::runtime_error("Package not found: " + std::string(e.what()));
    }
}

std::string get_package_source_directory() {
    // 获取当前源文件的绝对路径
    std::filesystem::path current_file(__FILE__);
    
    // 回溯到包根目录（包含package.xml的目录）
    auto path = current_file.parent_path();
    while (!path.empty() && !std::filesystem::exists(path / "package.xml")) {
        path = path.parent_path();
    }
    
    if (path.empty()) {
        throw std::runtime_error("无法定位到包根目录，请检查目录结构");
    }
    
    return path.string();
}

std::string get_package_path(const std::string& package_name) {
    return ament_index_cpp::get_package_share_directory(package_name);
}

// Clear all queues
void clear_all_queues() {
    // Reset state variables
    g_latest_bgr.reset();
    g_latest_rgb_timestamp = 0;
    g_has_rgb = false;
}

// Lidar data callback
static void lidar_data_callback(const lidar_data_t *data, void *user_data)
{
    // If device is not connected, ignore all data
    if (!deviceConnected) {
        return;
    }
    
    device_handle *dev_handle = static_cast<device_handle *>(user_data);
    if(!dev_handle || !data) {
        RCLCPP_WARN(odin_logger(), "[设备] 无效句柄或数据");
        return;
    }
    imu_convert_data_t *imudata = nullptr;
    lidar_device_status_t *dev_info_data;
    
    pid_t self = getpid();
    std::vector<pid_t> pids;

    double total_mb = 0.0;

    switch(data->type) {
        case LIDAR_DT_NONE:
            RCLCPP_WARN(odin_logger(), "[设备] 空雷达数据类型: 0x%x", data->type);
            break;
        case LIDAR_DT_RAW_RGB:
            if (g_sendrgb) {
                g_ros_object->publishRgb((capture_Image_List_t *)&data->stream);
            }
            update_count(&rgb_rx_fps);
            break;
        case LIDAR_DT_RAW_IMU:
            if (g_sendimu) {
                imudata = (imu_convert_data_t *)data->stream.imageList[0].pAddr;
                g_ros_object->publishImu(imudata);
            }
            update_count(&imu_rx_fps);
            break;
        case LIDAR_DT_RAW_DTOF:
            if (g_senddtof ) {
                g_ros_object->publishIntensityCloud((capture_Image_List_t *)&data->stream, 1);
            }
            if (g_pub_intensity_gray) {
                g_ros_object->publishGrayUInt8((capture_Image_List_t *)&data->stream, 2);
            }
            update_count(&dtof_rx_fps);
            break;
        case LIDAR_DT_SLAM_CLOUD:
            if (g_sendcloudslam) {
                g_ros_object->publishPC2XYZRGBA((capture_Image_List_t *)&data->stream, 0);
            }
            update_count(&slam_cloud_rx_fps);
            break;
        case LIDAR_DT_SLAM_ODOMETRY:
            if (g_sendodom) {
                g_ros_object->publishOdometry((capture_Image_List_t *)&data->stream, OdometryType::STANDARD, g_show_path, g_show_camerapose);
            }
            update_count(&slam_odom_rx_fps);
            break;
        case LIDAR_DT_DEV_STATUS:
            dev_info_data = (lidar_device_status_t *)data->stream.imageList[0].pAddr;

            pids.push_back(self);
            collect_children(self, pids);
            for (pid_t p : pids) {
                total_mb += read_pss_mb(p); // read_rss_mb(p);
            }

            if (g_devstatus_log) {
                if (dev_status_csv_file) {
                    // append the data row
                    int rc = 0;
                    rc = std::fprintf(dev_status_csv_file, "%.2f,%d,%d,%d,%d,%d,", 
                                            dev_info_data->uptime_seconds,
                                            dev_info_data->soc_thermal.package_temp,
                                            dev_info_data->soc_thermal.cpu_temp,
                                            dev_info_data->soc_thermal.center_temp,
                                            dev_info_data->soc_thermal.gpu_temp,
                                            dev_info_data->soc_thermal.npu_temp);
                    if (rc < 0) { 
                        RCLCPP_WARN(odin_logger(), "[设备] 状态 CSV 写入失败");
                    }

                    rc = std::fprintf(dev_status_csv_file, "%d,%d,", 
                        dev_info_data->dtof_sensor.tx_temp,
                        dev_info_data->dtof_sensor.rx_temp);
                    if (rc < 0) { 
                        RCLCPP_WARN(odin_logger(), "[设备] 状态 CSV 写入失败");
                    }

                    for (int i = 0; i < 8; i++) {
                        rc = std::fprintf(dev_status_csv_file, "%d,", dev_info_data->cpu_use_rate[i]);
                    }
                    rc = std::fprintf(dev_status_csv_file, "%d,", dev_info_data->ram_use_rate);

                    rc = std::fprintf(dev_status_csv_file, "%.2f,%.2f,%.2f,", 
                        ((float)dev_info_data->rgb_sensor.configured_odr)/1000,
                        ((float)dev_info_data->rgb_sensor.tx_odr)/1000,
                        cal_fps(&rgb_rx_fps, "rgb_rx")
                    );
                    if (rc < 0) { 
                        RCLCPP_WARN(odin_logger(), "[设备] 状态 CSV 写入失败");
                    }

                    rc = std::fprintf(dev_status_csv_file, "%.2f,%.2f,%.2f,",
                        ((float)dev_info_data->dtof_sensor.configured_odr)/1000,
                        ((float)dev_info_data->dtof_sensor.tx_odr)/1000,
                        cal_fps(&dtof_rx_fps, "dtof_rx")
                    );
                    if (rc < 0) { 
                        RCLCPP_WARN(odin_logger(), "[设备] 状态 CSV 写入失败");
                    }

                    rc = std::fprintf(dev_status_csv_file, "%.2f,%.2f,%.2f,",
                        ((float)dev_info_data->imu_sensor.configured_odr)/1000,
                        ((float)dev_info_data->imu_sensor.tx_odr)/1000,
                        cal_fps(&imu_rx_fps, "imu_rx")
                    );
                    if (rc < 0) { 
                        RCLCPP_WARN(odin_logger(), "[设备] 状态 CSV 写入失败");
                    }

                    rc = std::fprintf(dev_status_csv_file, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,", 
                        ((float)dev_info_data->slam_cloud_tx_odr)/1000,
                        cal_fps(&slam_cloud_rx_fps, "slam_cloud_rx"),
                        ((float)dev_info_data->slam_odom_tx_odr)/1000,
                        cal_fps(&slam_odom_rx_fps, "slam_odom_rx"),
                        ((float)dev_info_data->slam_odom_highfreq_tx_odr)/1000,
                        cal_fps(&slam_odom_highfreq_rx_fps, "slam_odom_highfreq_rx"));
                    if (rc < 0) { 
                        RCLCPP_WARN(odin_logger(), "[设备] 状态 CSV 写入失败");
                    }
                                   
                    rc = std::fprintf(dev_status_csv_file, "%.2f\n", total_mb);
                    if (rc < 0) { 
                        RCLCPP_WARN(odin_logger(), "[设备] 状态 CSV 写入失败");
                    }

                    std::fflush(dev_status_csv_file);
                }
            }
            if (g_show_fps) {
                double rgb_rx = cal_fps(&rgb_rx_fps, "rgb_rx");
                double dtof_rx = cal_fps(&dtof_rx_fps, "dtof_rx");
                double imu_rx = cal_fps(&imu_rx_fps, "imu_rx");
                double slam_cloud_rx = cal_fps(&slam_cloud_rx_fps, "slam_cloud_rx");
                double slam_odom_rx = cal_fps(&slam_odom_rx_fps, "slam_odom_rx");
                double slam_hf_rx = cal_fps(&slam_odom_highfreq_rx_fps, "slam_odom_highfreq_rx");
                std::ostringstream ss;
                ss << "\n" << ODIN_SEP << "\n  [Status] Thermal(C): pkg=" << dev_info_data->soc_thermal.package_temp
                   << " cpu=" << dev_info_data->soc_thermal.cpu_temp << " center=" << dev_info_data->soc_thermal.center_temp
                   << " gpu=" << dev_info_data->soc_thermal.gpu_temp << " npu=" << dev_info_data->soc_thermal.npu_temp
                   << "\n  CPU(%): ";
                for (int i = 0; i < 8; i++) ss << (i ? " " : "") << dev_info_data->cpu_use_rate[i];
                ss << " | RAM=" << dev_info_data->ram_use_rate << "%"
                   << "\n  RGB(Hz): cfg=" << ((float)dev_info_data->rgb_sensor.configured_odr)/1000
                   << " tx=" << ((float)dev_info_data->rgb_sensor.tx_odr)/1000 << " rx=" << std::fixed << std::setprecision(1) << rgb_rx
                   << "\n  DTOF(Hz): cfg=" << ((float)dev_info_data->dtof_sensor.configured_odr)/1000
                   << " tx=" << ((float)dev_info_data->dtof_sensor.tx_odr)/1000 << " rx=" << dtof_rx
                   << " sub=" << ((float)dev_info_data->dtof_sensor.subframe_odr)/1000
                   << " | T(tx=" << dev_info_data->dtof_sensor.tx_temp << "C rx=" << dev_info_data->dtof_sensor.rx_temp << "C)"
                   << "\n  IMU(Hz): cfg=" << ((float)dev_info_data->imu_sensor.configured_odr)/1000
                   << " tx=" << ((float)dev_info_data->imu_sensor.tx_odr)/1000 << " rx=" << imu_rx
                   << "\n  SLAM(Hz): cloud tx=" << ((float)dev_info_data->slam_cloud_tx_odr)/1000 << " rx=" << slam_cloud_rx
                   << " | odom tx=" << ((float)dev_info_data->slam_odom_tx_odr)/1000 << " rx=" << slam_odom_rx
                   << " | hf tx=" << ((float)dev_info_data->slam_odom_highfreq_tx_odr)/1000 << " rx=" << slam_hf_rx
                   << "\n  RAM: " << std::setprecision(2) << total_mb << " MB\n" << ODIN_SEP;
                RCLCPP_INFO(odin_logger(), "%s", ss.str().c_str());
            }
            break;
            case LIDAR_DT_SLAM_ODOMETRY_HIGHFREQ:
            {
                if (g_sendodom) {
                    g_ros_object->publishOdometry((capture_Image_List_t *)&data->stream, OdometryType::HIGHFREQ, false, false);
                }
                update_count(&slam_odom_highfreq_rx_fps);
            }
            break;
            case LIDAR_DT_SLAM_ODOMETRY_TF:
            {
                if (g_custom_map_mode == 2) {
                    g_ros_object->publishOdometry((capture_Image_List_t *)&data->stream, OdometryType::TRANSFORM, false, false);
                    if (!g_relocalization_success_msg_printed) {
                        RCLCPP_INFO(odin_logger(), "[Odom] Relocalization success");
                        g_relocalization_success_msg_printed = true;
                    }
                }
            }
            break;
            case LIDAR_DT_SLAM_WIWC:
            {
                if(g_record_data ) {
                    g_ros_object->recordrotate((capture_Image_List_t *)&data->stream);
                }
            }
            break;
        default:
            RCLCPP_WARN(odin_logger(), "[设备] 未知数据类型: 0x%x", data->type);
            return;
    }
}

static void lidar_device_callback(const lidar_device_info_t* device, bool attach)
{
    int type = LIDAR_MODE_SLAM;
    // int type = LIDAR_MODE_RAW;
    static std::chrono::steady_clock::time_point software_connect_start; 
    static bool software_connect_timing = false; 
    
    if(attach == true) {
        RCLCPP_INFO(odin_logger(), "%s\n  [阶段] 硬件已连接，正在建立软件连接...\n%s", ODIN_BANNER, ODIN_SEP);
        if (!isUsb3OrHigher(TARGET_VENDOR, TARGET_PRODUCT)) {
            RCLCPP_FATAL(odin_logger(), "[USB] 设备接在 USB 2.0 上，需 USB 3.0 及以上。程序退出。");

            g_usb_version_error = true;
            system("pkill -f rviz");
            exit(1);
            return;
        }

        software_connect_start = std::chrono::steady_clock::now();
        software_connect_timing = true;
        
        if (odinDevice) {
            odinDevice = nullptr;
        }
        
        if (lidar_create_device(const_cast<lidar_device_info_t*>(device), &odinDevice)) {
            RCLCPP_ERROR(odin_logger(), "[设备] 创建设备失败");
            return;
        }
	const std::string package_name = "odin_ros_driver";
	std::string config_dir = "";
	char* ros_workspace = std::getenv("COLCON_PREFIX_PATH");
	if (ros_workspace) {
	    std::string workspace_path(ros_workspace);
	    size_t pos = workspace_path.find("/install");
	    if (pos != std::string::npos) {
		config_dir = workspace_path.substr(0, pos) + "/src/odin_ros_driver/config";
	    } else {
		config_dir = ament_index_cpp::get_package_share_directory(package_name) + "/config";
	    }
	} else {
	    config_dir = ament_index_cpp::get_package_share_directory(package_name) + "/config";
	}
        RCLCPP_INFO(odin_logger(), "[Device] Calibration dir: %s", config_dir.c_str());

        std::filesystem::path per_con_log_root_dir;
        {
            auto connection_time = std::chrono::system_clock::now();
            std::time_t t = std::chrono::system_clock::to_time_t(connection_time);
            std::tm tm{};
            #ifdef _WIN32
                localtime_s(&tm, &t);
            #else
                localtime_r(&t, &tm);
            #endif
            char buf[32];
            std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm);
            std::string folder_name = std::string("Conn_") + std::string(buf);
            std::filesystem::path base_log_dir = log_root_dir_.empty()
                ? std::filesystem::path(config_dir)
                : log_root_dir_;
            per_con_log_root_dir = base_log_dir / folder_name;

            std::error_code per_con_dir_err;
            std::filesystem::create_directories(per_con_log_root_dir, per_con_dir_err);
            if (per_con_dir_err) {
                RCLCPP_WARN(odin_logger(), "[设备] 创建日志目录失败 %s: %s",
                    per_con_log_root_dir.string().c_str(), per_con_dir_err.message().c_str());
            }
        }

        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - software_connect_start);
        if (elapsed.count() >= 60) {
            RCLCPP_FATAL(odin_logger(), "[Device] Connection timed out (60s). Exiting.");
            
            if (odinDevice) {
                lidar_close_device(odinDevice);
                lidar_destory_device(odinDevice);
                odinDevice = nullptr;
            }
            
            g_connection_timeout = true;
            return;
        }
        
        lidar_fireware_version_t version;
        if(lidar_get_version(odinDevice,&version)) {
            RCLCPP_ERROR(odin_logger(), "[设备] 获取固件版本失败，请升级设备固件");
            system("pkill -f rviz");
            system("pkill -f host_sdk_sample");
            exit(1);
        }
        else {
            RCLCPP_INFO(odin_logger(), "%s\n  [版本] 驱动 %s，要求固件 >= %d.%d.%d | 内核 V%d.%d.%d  MCU V%d.%d.%d  SOC V%d.%d.%d\n%s",
                ODIN_SEP, ros_driver_version, required_firmware_version_major, required_firmware_version_minor, required_firmware_version_patch,
                version.kernel_version.major, version.kernel_version.minor, version.kernel_version.patch,
                version.mcu_version.major, version.mcu_version.minor, version.mcu_version.patch,
                version.soc_version.major, version.soc_version.minor, version.soc_version.patch,
                ODIN_SEP);

            if (version.soc_version.major < required_firmware_version_major || (version.soc_version.minor < required_firmware_version_minor) || (version.soc_version.patch < required_firmware_version_patch)) {
                RCLCPP_ERROR(odin_logger(), "[版本] SOC 版本过低，请升级固件至 >= %d.%d.%d", required_firmware_version_major, required_firmware_version_minor, required_firmware_version_patch);
                system("pkill -f rviz");
                system("pkill -f host_sdk_sample");
                exit(1);
            }
        }

        if (g_save_log) {
            if (lidar_enable_encrypted_device_log(const_cast<lidar_device_info_t*>(device), per_con_log_root_dir.c_str())) {
                RCLCPP_ERROR(odin_logger(), "[设备] 开启加密日志失败");
                lidar_close_device(odinDevice);
                lidar_destory_device(odinDevice);
                odinDevice = nullptr;
                return;
            }

            RCLCPP_INFO(odin_logger(), "[设备] 加密日志已开启: %s", per_con_log_root_dir.c_str());
        } else {
            RCLCPP_INFO(odin_logger(), "[设备] 加密日志已关闭");
        }

        bool need_open_device = true;
        bool get_calib_file = true;
        switch (device->initial_state) {
            case LIDAR_DEVICE_NOT_INITIALIZED:
                RCLCPP_INFO(odin_logger(), "[阶段] 设备未初始化，执行完整初始化");
                break;
            case LIDAR_DEVICE_INITIALIZED:
                need_open_device = false;
                RCLCPP_INFO(odin_logger(), "[阶段] 设备已初始化，跳过打开");
                break;
            case LIDAR_DEVICE_STREAMING:
                RCLCPP_WARN(odin_logger(), "[设备] 异常：已在推流，退出");
                system("pkill -f rviz");
                exit(1);
                break;
            case LIDAR_DEVICE_STREAM_STOPPED:
                need_open_device = false;
                get_calib_file = false;
                RCLCPP_INFO(odin_logger(), "[Device] State: stream stopped, resuming");
                break;
            default:
                RCLCPP_WARN(odin_logger(), "[Device] Unknown state: %d", device->initial_state);
                break;
        }

        if (need_open_device) {
            if (lidar_open_device(odinDevice)) {
                RCLCPP_ERROR(odin_logger(), "[设备] 打开设备失败");
                lidar_destory_device(odinDevice);
                odinDevice = nullptr;
                return;
            }
        }
        
        std::string calib_config = config_dir + "/calib.yaml";
        calib_file_ = calib_config;
        if (get_calib_file) {
            if (lidar_get_calib_file(odinDevice, config_dir.c_str())) {
                RCLCPP_ERROR(odin_logger(), "[设备] 获取标定文件失败");
                lidar_close_device(odinDevice);
                lidar_destory_device(odinDevice);
                odinDevice = nullptr;
                return;
            }
            
            RCLCPP_INFO(odin_logger(), "[Device] Calibration files retrieved");
        } else {
            RCLCPP_INFO(odin_logger(), "[Device] Skip calibration (current state)");
        }
        
        if (std::filesystem::exists(calib_config)) {
            g_renderer = std::make_shared<rawCloudRender>();
            if (g_renderer->init(calib_config)) {
                RCLCPP_INFO(odin_logger(), "[阶段] 点云着色已就绪");
            } else {
                RCLCPP_ERROR(odin_logger(), "[设备] 点云着色初始化失败");
            }
        } else {
            RCLCPP_WARN(odin_logger(), "[设备] 未找到着色配置: %s", calib_config.c_str());
        }
        
        if (lidar_set_mode(odinDevice, type)) {
            RCLCPP_ERROR(odin_logger(), "[设备] 设置模式失败");
            lidar_close_device(odinDevice);
            lidar_destory_device(odinDevice);
            odinDevice = nullptr;
            return;
        }

        // Apply custom parameters after setting mode
        if (g_parser && !g_parser->applyCustomParameters(odinDevice)) {
            RCLCPP_WARN(odin_logger(), "[设备] 部分自定义参数应用失败");
        }

        RCLCPP_INFO(odin_logger(), "[阶段] 建图模式: %d", g_custom_map_mode);

        if (g_custom_map_mode == 1) {
            int save_map_init_value = 0;
            int result = lidar_set_custom_parameter(odinDevice, "save_map", &save_map_init_value, sizeof(int));

            if (result == 0) {
                RCLCPP_INFO(odin_logger(), "[命令] save_map 已初始化为 %d", save_map_init_value);
            } else {
                RCLCPP_ERROR(odin_logger(), "[命令] save_map 初始化失败: %d", result);
            } 
        } else if (g_custom_map_mode == 2) {
            if (g_relocalization_map_abs_path != "" && std::filesystem::exists(g_relocalization_map_abs_path) && 
                lidar_set_relocalization_map(odinDevice, g_relocalization_map_abs_path.c_str()) == 0) {
                RCLCPP_INFO(odin_logger(), "[Device] Relocalization map set OK");
            } else {
                RCLCPP_ERROR(odin_logger(), "[Device] Relocalization map set failed");
                lidar_close_device(odinDevice);
                lidar_destory_device(odinDevice);
                odinDevice = nullptr;
                return;
            }
        }
 
        lidar_data_callback_info_t data_callback_info;
        data_callback_info.data_callback = lidar_data_callback;
        data_callback_info.user_data = &odinDevice;

        if (lidar_register_stream_callback(odinDevice, data_callback_info)) {
            RCLCPP_ERROR(odin_logger(), "[设备] 注册回调失败");
            lidar_close_device(odinDevice);
            lidar_destory_device(odinDevice);
            odinDevice = nullptr;
            return;
        }
        
        std::string dev_status_csv_file_path_ = per_con_log_root_dir / "dev_status.csv";

        if (dev_status_csv_file) {
            std::fflush(dev_status_csv_file);
            fclose(dev_status_csv_file);
            dev_status_csv_file = nullptr;
        }

        // Open the file in append mode
        dev_status_csv_file = fopen(dev_status_csv_file_path_.c_str(), "a");
        if (!dev_status_csv_file) {
            RCLCPP_ERROR(odin_logger(), "[初始化] 打开设备状态 CSV 失败");
        } else {
            const char* header =
            "uptime_seconds,package_temp,cpu_temp,center_temp,gpu_temp,npu_temp,dtof_tx_temp,dtof_rx_temp,"
            "cpu0,cpu1,cpu2,cpu3,cpu4,cpu5,cpu6,cpu7,ram_use(%),"
            "rgb_configured_odr,rgb_tx_odr,rgb_rx_odr,dtof_configured_odr,dtof_tx_odr,dtof_rx_odr,imu_configured_odr,imu_tx_odr,imu_rx_odr,"
            "slam_cloud_tx_odr,slam_cloud_rx_odr,slam_odom_tx_odr,slam_odom_rx_odr,slam_odom_highfreq_tx_odr,slam_odom_highfreq_rx_odr,"
            "host_ram_use(mb)\n";
            fprintf(dev_status_csv_file, "%s", header);
            std::fflush(dev_status_csv_file);
        }

        uint32_t dtof_subframe_odr = 0;
        if (lidar_start_stream(odinDevice, type, dtof_subframe_odr)) {
            RCLCPP_ERROR(rclcpp::get_logger("device_cb"), "Start stream failed");
            lidar_close_device(odinDevice);
            lidar_destory_device(odinDevice);
            odinDevice = nullptr;
            return;
        }
        
        if (dtof_subframe_odr > 0) {
            g_rosNodeControlImpl.setDtofSubframeODR(dtof_subframe_odr);
        }
        
        if (g_sendrgb) {
            lidar_activate_stream_type(odinDevice, LIDAR_DT_RAW_RGB);
        }
        if (g_sendimu) {
            lidar_activate_stream_type(odinDevice, LIDAR_DT_RAW_IMU);
        }
        if (g_sendodom) {
            lidar_activate_stream_type(odinDevice, LIDAR_DT_SLAM_ODOMETRY);
        }
        if (g_senddtof) {
            lidar_activate_stream_type(odinDevice, LIDAR_DT_RAW_DTOF);
        }
        if (g_sendcloudslam) {
            lidar_activate_stream_type(odinDevice, LIDAR_DT_SLAM_CLOUD);
        }
        
        software_connect_timing = false;
        deviceConnected = true;
        deviceDisconnected = false;
        
        // Start custom parameter monitoring thread
        g_param_monitor_running = true;
        g_param_monitor_thread = std::thread(custom_parameter_monitor);
        
        RCLCPP_INFO(odin_logger(), "[阶段] 命令文件已就绪: echo 'set save_map 1' > %s", g_command_file_path.c_str());
        
        bool load_status = g_ros_object->loadCameraParams(calib_config);
        if (g_sendrgb_undistort &&  load_status == 0) {
            g_ros_object->buildUndistortMap();
        }

        RCLCPP_INFO(odin_logger(), "%s\n  [Device] Ready in %ld s, streams active\n%s", ODIN_SEP,
                   std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - software_connect_start).count(), ODIN_SEP);
    } else {
        RCLCPP_INFO(odin_logger(), "[Device] Detaching...");

        deviceConnected = false;
        deviceDisconnected = true;
        
        // Stop custom parameter monitoring thread
        g_param_monitor_running = false;
        if (g_param_monitor_thread.joinable()) {
            g_param_monitor_thread.join();
        }
        

        clear_all_queues();

        if (dev_status_csv_file) {
            std::fflush(dev_status_csv_file);
            fclose(dev_status_csv_file);
            dev_status_csv_file = nullptr;
        }

        RCLCPP_INFO(odin_logger(), "[阶段] 等待设备重新连接...");
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("lydros_node");
    g_ros_object = std::make_shared<MultiSensorPublisher>(node);

    // Register signal handlers for Ctrl+C
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    try {
        std::string package_path = get_package_source_directory();
        std::string config_dir = package_path + "/config";
        RCLCPP_INFO(odin_logger(), "%s\n  [Odin] package: %s  config: %s\n%s", ODIN_BANNER, package_path.c_str(), config_dir.c_str(), ODIN_SEP);
        std::string config_file = config_dir + "/control_command.yaml";

        g_command_file_path = "/tmp/odin_command.txt";

        g_parser = std::make_shared<odin_ros_driver::YamlParser>(config_file);
        if (!g_parser->loadConfig()) {
            RCLCPP_ERROR(odin_logger(), "[初始化] 配置加载失败: %s", config_file.c_str());
            return -1;
        }

        auto keys = g_parser->getRegisterKeys();
        auto keys_w_str_val = g_parser->getRegisterKeysStrVal();
        g_parser->printConfig();

        auto get_key_value = [&](const std::string& key, int default_value) -> int {
            auto it = keys.find(key);
            return it != keys.end() ? it->second : default_value;
        };

        g_sendrgb       = get_key_value("sendrgb", 1);
        g_sendimu       = get_key_value("sendimu", 1);
        g_senddtof      = get_key_value("senddtof", 1);
        g_sendodom      = get_key_value("sendodom", 1);
        g_send_odom_baselink_tf = get_key_value("send_odom_baselink_tf", 0);
        g_sendcloudslam = get_key_value("sendcloudslam", 0);
        g_sendcloudrender = get_key_value("sendcloudrender", 1);
        g_sendrgb_compressed = get_key_value("sendrgbcompressed", 1);
        g_sendrgb_undistort = get_key_value("sendrgbundistort", 0);
        g_record_data = get_key_value("recorddata", 0);
        g_show_fps = get_key_value("showfps", 0);
        g_devstatus_log = get_key_value("devstatuslog", 0);
        g_pub_intensity_gray = get_key_value("pubintensitygray", 0);
        g_show_path = get_key_value("showpath", 0);
        g_show_camerapose = get_key_value("showcamerapose", 0);
        g_log_level = get_key_value("log_devel", LOG_LEVEL_INFO);
        g_strict_usb3_0_check = get_key_value("strict_usb3.0_check", 1);
        g_use_host_ros_time = get_key_value("use_host_ros_time", 0);
        g_save_log = get_key_value("save_log", 0);

        if (g_use_host_ros_time) {
            g_rosNodeControlImpl.setUseHostRosTime(true);
        }

        if (g_send_odom_baselink_tf) {
            g_rosNodeControlImpl.setSendOdomBaseLinkTF(true);
        }

        auto get_key_str_value = [&](const std::string& key, const std::string& default_value) -> std::string {
            auto it = keys_w_str_val.find(key);
            return it != keys_w_str_val.end() ? it->second : default_value;
        };

        g_relocalization_map_abs_path = get_key_str_value("relocalization_map_abs_path", "");
        g_mapping_result_dest_dir = get_key_str_value("mapping_result_dest_dir", "");
        g_mapping_result_file_name = get_key_str_value("mapping_result_file_name", "");

        g_custom_map_mode = g_parser->getCustomMapMode(2);

        lidar_log_set_level(LIDAR_LOG_INFO);

        const std::string package_name = "odin_ros_driver";
        std::string data_dir = "";
        std::string log_dir = "";
        std::string map_dir = "";
        char* ros_workspace = std::getenv("COLCON_PREFIX_PATH");
        if (ros_workspace) {
            std::string workspace_path(ros_workspace);
            size_t pos = workspace_path.find("/install");
            if (pos != std::string::npos) {
                data_dir = workspace_path.substr(0, pos) + "/src/odin_ros_driver/recorddata";
                log_dir = workspace_path.substr(0, pos) + "/src/odin_ros_driver/log";
                map_dir = workspace_path.substr(0, pos) + "/src/odin_ros_driver/map";
            } else {
                data_dir = ament_index_cpp::get_package_share_directory(package_name) + "/recorddata";
                log_dir = ament_index_cpp::get_package_share_directory(package_name) + "/log";
                map_dir = ament_index_cpp::get_package_share_directory(package_name) + "/map";
            }
        } else {
            data_dir = ament_index_cpp::get_package_share_directory(package_name) + "/recorddata";
            log_dir = ament_index_cpp::get_package_share_directory(package_name) + "/log";
            map_dir = ament_index_cpp::get_package_share_directory(package_name) + "/map";
        }

        if (g_record_data) {
            g_ros_object->initialize_data_logger(data_dir);
        }

        auto now = std::chrono::system_clock::now();
        std::time_t t = std::chrono::system_clock::to_time_t(now);
        std::tm tm{};
        #ifdef _WIN32
            localtime_s(&tm, &t);
        #else
            localtime_r(&t, &tm);
        #endif
        std::strftime(driver_start_time, sizeof(driver_start_time), "%Y%m%d_%H%M%S", &tm);

        if (g_devstatus_log) {
            std::string folder_name = std::string("Driver_") + std::string(driver_start_time);
            log_root_dir_ = std::filesystem::path(log_dir) / folder_name;
            std::filesystem::create_directories(log_root_dir_);
        }

        if (g_custom_map_mode == 1 && g_mapping_result_dest_dir == "") {
            map_root_dir_ = std::filesystem::path(map_dir) / driver_start_time;
            std::filesystem::create_directories(map_root_dir_);
        }

        if (lidar_system_init(lidar_device_callback)) {
            RCLCPP_ERROR(odin_logger(), "[Init] Lidar system init failed");
            return -1;
        }
        

        bool usbPresent = false;
        bool usbVersionChecked = false; 
        while (!deviceConnected) {
            if (!rclcpp::ok()) {
                break;
            }

            usbPresent = isUsbDevicePresent(TARGET_VENDOR, TARGET_PRODUCT); 
            if (usbPresent) { 
                if (!usbVersionChecked) {
                    usbVersionChecked = true;
                    
                    if (!isUsb3OrHigher(TARGET_VENDOR, TARGET_PRODUCT)) {
                        RCLCPP_FATAL(odin_logger(), "[USB] 设备接在 USB 2.0 上，需 USB 3.0。程序退出。");
                        lidar_system_deinit();
                        return 1;
                    }
                }
            }
            
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(odin_logger(), "[初始化] 异常: %s", e.what());
        lidar_system_deinit();
        return -1;
    }

    if (!deviceConnected) {
        if (g_ros_object) {
            g_ros_object.reset();   // destroys all publishers/subscribers
        }
        node.reset();              // destroy the node first
        rclcpp::shutdown();
        return 1;
    }

    bool disconnect_msg_printed = false;
    // Create 10Hz Rate object
    rclcpp::Rate rate(10);
    
    while (rclcpp::ok()) {
        rclcpp::spin_some(node);
        // Check device disconnection status
        if (deviceDisconnected.load()) {
            if (!disconnect_msg_printed) {
                RCLCPP_INFO(odin_logger(), "[Device] Disconnected, waiting for reconnection...");
                disconnect_msg_printed = true;
            }
            
            // Wait 0.1 seconds
            rate.sleep();
            continue;  // Skip rest of this loop iteration
        }
        
        // Data processing when device is connected
        if (g_sendcloudrender) {
            g_ros_object->try_process_pair();  
        }
        
        // Check for command file
        if (deviceConnected) {
            process_command_file();
        }
        
        disconnect_msg_printed = false;

        // Wait 0.1 seconds
        rate.sleep();
    }
    rclcpp::shutdown();

    // Cleanup on normal program exit
    if (odinDevice) {
        // Convert calib.yaml to cam_in_ex.txt at program end
        if (g_ros_object) {
            const std::filesystem::path out_path = g_ros_object->get_root_dir() / "image" / "cam_in_ex.txt";
            (void)convert_calib_to_cam_in_ex(calib_file_, out_path);
        }
        RCLCPP_INFO(odin_logger(), "[退出] 统计: pose=%d cloud=%d image=%d",
            g_ros_object->get_pose_index(), g_ros_object->get_cloud_index(), g_ros_object->get_image_index());
        if(lidar_unregister_stream_callback(odinDevice)) {
            RCLCPP_WARN(odin_logger(), "[退出] 注销数据流回调失败");
        }

        if (dev_status_csv_file) {
            std::fflush(dev_status_csv_file);
            fclose(dev_status_csv_file);
            dev_status_csv_file = nullptr;
        }
    }
    
    // Stop custom parameter monitoring thread on exit
    g_param_monitor_running = false;
    if (g_param_monitor_thread.joinable()) {
        g_param_monitor_thread.join();
    }
      
    // lidar_system_deinit();

    return 0;
}
