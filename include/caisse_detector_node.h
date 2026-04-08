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

#include <chrono>
#include <deque>

/**
 * CaisseDetectorNode
 *
 * Subscribes to an image topic and detects 4 colored tiles (blue/yellow)
 * arranged left-to-right. Publishes a CaissesSides message indicating
 * whether each tile is "our side up" based on the `is_blue` parameter.
 *
 * Parameters:
 *   is_blue                (bool,   default: true)   — blue = our side up
 *   debug_image            (bool,   default: false)  — publish annotated debug image
 *   history_duration_ms    (int,    default: 1000)   — sliding vote window (ms)
 *   roi_x                  (int,    default: 0)      — ROI top-left x (px)
 *   roi_y                  (int,    default: 0)      — ROI top-left y (px)
 *   roi_width              (int,    default: 0)      — ROI width  (0 = full image)
 *   roi_height             (int,    default: 0)      — ROI height (0 = full image)
 *   min_tile_area_fraction (double, default: 0.005)  — min blob area as fraction of ROI area
 *
 * Subscribes:  /krabi_ns/camera_node/image_raw  (sensor_msgs/Image)
 * Publishes:   ~/caisses_sides                  (krabi_msgs/CaissesSides)
 *              ~/debug_image                    (sensor_msgs/Image)  [optional]
 */

// ── HSV colour ranges ─────────────────────────────────────────────────────────
// Tune these if lighting conditions change.
const cv::Scalar YELLOW_LOW(15, 103, 162);
const cv::Scalar YELLOW_HIGH(40, 255, 255);

const cv::Scalar BLUE_LOW(90, 80, 50);
const cv::Scalar BLUE_HIGH(140, 255, 255);

// Expected number of tiles
constexpr int N_TILES = 4;

// A blob wider than this multiple of the smallest blob is considered merged
constexpr double MERGE_RATIO_THRESHOLD = 1.6;

// ── Tile descriptor ───────────────────────────────────────────────────────────
struct Tile
{
    cv::Rect bbox; // coordinates in ROI space
    bool is_blue;
    int cx;       // centroid x in ROI space (for left-to-right sorting)
    double score; // colour_purity × sqrt(area) — higher = more confident
};

// ── Detection result snapshot (one valid frame) ───────────────────────────────
struct DetectionSnapshot
{
    std::array<bool, N_TILES> is_our_side;
    std::chrono::steady_clock::time_point timestamp;
};

// ── Free helpers ──────────────────────────────────────────────────────────────
cv::Mat colourMask(const cv::Mat& hsv, const cv::Scalar& lo, const cv::Scalar& hi);

std::vector<cv::Rect> splitBlobVertically(const cv::Rect& bbox, int n);

int estimateTileWidth(const std::vector<cv::Rect>& blobs);

// ── Node class ────────────────────────────────────────────────────────────────
class CaisseDetectorNode : public rclcpp::Node
{
public:
    CaisseDetectorNode();

private:
    // ── Detection helpers ─────────────────────────────────────────────────────
    // Returns colour purity [0,1]: blue or yellow
    double colourPurity(const cv::Mat& hsv, const cv::Rect& bbox);

    // Classify bbox as blue or yellow
    bool regionIsBlue(const cv::Mat& hsv, const cv::Rect& bbox);

    bool tileIsOurSide(const Tile& t) const;

    // Clamp and return the effective ROI given current image size
    cv::Rect effectiveRoi(int img_w, int img_h) const;

    // Compute the minimum tile area threshold from the current ROI size
    double minTileArea(const cv::Rect& roi) const;

    // ── Callbacks ─────────────────────────────────────────────────────────────
    void imageCb(const sensor_msgs::msg::CompressedImage::SharedPtr msg);

    // ── Publishing ────────────────────────────────────────────────────────────
    void publishVotedResult(const std_msgs::msg::Header& header);

    void publishDebug(const cv::Mat& full_img,
                      const std::vector<Tile>& tiles, // ROI-space coords
                      const cv::Rect& roi,
                      const std_msgs::msg::Header& header);

    // ── Members ───────────────────────────────────────────────────────────────
    bool is_blue_{ true };
    bool debug_image_{ false };
    int history_duration_ms_{ 1000 };
    double min_tile_area_fraction_{ 0.1 };

    // ROI parameters (0 width/height = full image)
    int roi_x_{ 0 };
    int roi_y_{ 0 };
    int roi_width_{ 0 };
    int roi_height_{ 0 };

    // Sliding window of valid detections
    std::deque<DetectionSnapshot> history_;

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;
    rclcpp::Publisher<krabi_msgs::msg::CaissesSides>::SharedPtr sides_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};