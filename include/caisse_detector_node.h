#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#if __has_include(<cv_bridge/cv_bridge.hpp>)
#include <cv_bridge/cv_bridge.hpp> // ROS2 Humble+
#else
#include <cv_bridge/cv_bridge.h> // older distributions
#endif
#include "krabi_msgs/msg/caisses_sides.hpp"
#include <opencv2/opencv.hpp>

/**
 * CaisseDetectorNode
 *
 * Subscribes to an image topic and detects 4 colored tiles (blue/yellow)
 * arranged left-to-right. Publishes a CaissesSides message indicating
 * whether each tile is "our side up" based on the `is_blue` parameter.
 *
 * Parameters:
 *   is_blue       (bool, default: true)  — if true, blue = our side up
 *   debug_image   (bool, default: false) — if true, publish annotated debug image
 *
 * Subscribes:  /krabi_ns/camera_node/image_raw  (sensor_msgs/Image)
 * Publishes:   ~/caisses_sides                  (krabi_msgs/CaissesSides)
 *              ~/debug_image                    (sensor_msgs/Image)  [optional]
 */

// ── HSV colour ranges ─────────────────────────────────────────────────────────
// Tune these if lighting conditions change.
const cv::Scalar YELLOW_LOW(15, 80, 80);
const cv::Scalar YELLOW_HIGH(40, 255, 255);

// Blue sits around 100-130 in OpenCV HSV
const cv::Scalar BLUE_LOW(90, 80, 50);
const cv::Scalar BLUE_HIGH(140, 255, 255);

// Minimum contour area to be considered a valid tile (px²)
constexpr double MIN_TILE_AREA = 2000.0;

// Expected number of tiles
constexpr int N_TILES = 4;

// ── Tile descriptor ───────────────────────────────────────────────────────────
struct Tile
{
    cv::Rect bbox;
    bool is_blue; // true = blue, false = yellow
    int cx;       // centroid x (for left-to-right sorting)
};

// ── Helper: build a morphologically-cleaned binary mask ───────────────────────
cv::Mat colourMask(const cv::Mat& hsv, const cv::Scalar& lo, const cv::Scalar& hi);

// ── Node class ────────────────────────────────────────────────────────────────
class CaisseDetectorNode : public rclcpp::Node
{
public:
    CaisseDetectorNode();

private:
    // Classify a tile region as blue or yellow (uses top 60 % to avoid QR symbol)
    bool regionIsBlue(const cv::Mat& hsv, const cv::Rect& bbox);

    // Map tile colour → "our side" using the is_blue parameter
    bool tileIsOurSide(const Tile& t) const;

    // Main image callback
    void imageCb(const sensor_msgs::msg::Image::SharedPtr msg);

    // Publish annotated debug image
    void publishDebug(const cv::Mat& img,
                      const std::vector<Tile>& tiles,
                      const std_msgs::msg::Header& header);

    // ── Members ──────────────────────────────────────────────────────────────────
    bool is_blue_{ true };
    bool debug_image_{ false };

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<krabi_msgs::msg::CaissesSides>::SharedPtr sides_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};