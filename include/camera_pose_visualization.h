#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

class camera_pose_visualization {
  public:
    std::string m_marker_ns;

    camera_pose_visualization(float r, float g, float b, float a);

    void setImageBoundaryColor(float r, float g, float b, float a = 1.0);
    void setOpticalCenterConnectorColor(float r, float g, float b, float a = 1.0);
    void setScale(double s);
    void setLineWidth(double width);

    void add_pose(const Eigen::Vector3d& p, const Eigen::Quaterniond& q);
    void reset();

    using ColorRGBA = std_msgs::msg::ColorRGBA;
    using Marker = visualization_msgs::msg::Marker;
    using MarkerArray = visualization_msgs::msg::MarkerArray;
    using Header = std_msgs::msg::Header;
    using Publisher = rclcpp::Publisher<MarkerArray>::SharedPtr;

    void publish_by(Publisher& pub, const Header& header);
    void add_edge(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1);
    void add_loopedge(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1);
  private:
    std::vector<Marker> m_markers;
    ColorRGBA m_image_boundary_color;
    ColorRGBA m_optical_center_connector_color;
    double m_scale;
    double m_line_width;

    static const Eigen::Vector3d imlt;
    static const Eigen::Vector3d imlb;
    static const Eigen::Vector3d imrt;
    static const Eigen::Vector3d imrb;
    static const Eigen::Vector3d oc  ;
    static const Eigen::Vector3d lt0 ;
    static const Eigen::Vector3d lt1 ;
    static const Eigen::Vector3d lt2 ;
};
