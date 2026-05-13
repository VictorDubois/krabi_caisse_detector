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

#include <array>
#include <chrono>
#include <deque>

/**
 * CaisseDetectorNode
 *
 * Subscribes to an image topic. Each of the 4 caisses has a dedicated RoI;
 * the dominant colour (blue vs yellow) inside that RoI determines whether
 * the caisse is "our side up". A per-RoI sliding vote window smooths results.
 *
 * Parameters:
 *   is_blue                    (bool,   default: true)   — blue = our side up
 *   debug_image                (bool,   default: false)  — publish annotated debug image
 *   history_duration_ms        (int,    default: 1000)   — sliding vote window (ms)
 *   min_color_coverage         (double, default: 0.05)   — min fraction of RoI covered by
 *                                                          blue or yellow to count as valid
 *   roi_{leftmost,leftcenter,rightcenter,rightmost}_{x,y,width,height}
 *                              (int,    default: 0)      — RoI corners (0 width/height = full image)
 *
 * Subscribes:  krabi_cam/image_raw/compressed  (sensor_msgs/CompressedImage)
 * Publishes:   ~/caisses_sides                 (krabi_msgs/CaissesSides)
 *              ~/debug_image                   (sensor_msgs/Image)  [optional]
 */

// ── HSV colour ranges ─────────────────────────────────────────────────────────
// Tune these if lighting conditions change.
const cv::Scalar YELLOW_LOW(15, 103, 162);
const cv::Scalar YELLOW_HIGH(40, 255, 255);

const cv::Scalar BLUE_LOW(90, 80, 50);
const cv::Scalar BLUE_HIGH(140, 255, 255);

constexpr int N_TILES = 4;

// Position labels — index order matches CaissesSides message fields
static constexpr std::array<const char*, N_TILES> ROI_NAMES
  = { "leftmost", "leftcenter", "rightcenter", "rightmost" };

// ── Detection result snapshot (one valid frame, one RoI) ─────────────────────
struct DetectionSnapshot
{
    bool is_our_side;
    std::chrono::steady_clock::time_point timestamp;
};

// ── Free helper ───────────────────────────────────────────────────────────────
cv::Mat colourMask(const cv::Mat& hsv, const cv::Scalar& lo, const cv::Scalar& hi);

// ── Node class ────────────────────────────────────────────────────────────────
class CaisseDetectorNode : public rclcpp::Node
{
public:
    CaisseDetectorNode();

private:
    bool tileIsOurSide(bool is_blue) const;

    // Clamp RoI[idx] to image bounds
    cv::Rect effectiveRoi(int idx, int img_w, int img_h) const;

    // ── Callbacks ─────────────────────────────────────────────────────────────
    void imageCb(const sensor_msgs::msg::CompressedImage::SharedPtr msg);

    // ── Publishing ────────────────────────────────────────────────────────────
    void publishVotedResult(const std_msgs::msg::Header& header);

    void publishDebug(const cv::Mat& full_img,
                      const std::array<cv::Rect, N_TILES>& rois,
                      const std::array<bool, N_TILES>& is_our_side,
                      const std::array<bool, N_TILES>& valid,
                      const std_msgs::msg::Header& header);

    // ── Members ───────────────────────────────────────────────────────────────
    bool is_blue_{ true };
    bool debug_image_{ false };
    int history_duration_ms_{ 1000 };
    double min_color_coverage_{ 0.05 };

    struct RoiParams
    {
        int x{ 0 }, y{ 0 }, w{ 0 }, h{ 0 };
    };
    std::array<RoiParams, N_TILES> rois_{};

    // One independent vote history per RoI
    std::array<std::deque<DetectionSnapshot>, N_TILES> history_;

    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;
    rclcpp::Publisher<krabi_msgs::msg::CaissesSides>::SharedPtr sides_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};
