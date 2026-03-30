#include "caisse_detector_node.h"

// ── colourMask ────────────────────────────────────────────────────────────────
cv::Mat colourMask(const cv::Mat& hsv, const cv::Scalar& lo, const cv::Scalar& hi)
{
    cv::Mat mask;
    cv::inRange(hsv, lo, hi, mask);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, { 5, 5 });
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    return mask;
}

// ── Constructor ───────────────────────────────────────────────────────────────
CaisseDetectorNode::CaisseDetectorNode()
  : Node("caisse_detector")
{
    // ── Parameters ──────────────────────────────────────────────────────────────
    this->declare_parameter<bool>("is_blue", true);
    this->declare_parameter<bool>("debug_image", false);

    is_blue_ = this->get_parameter("is_blue").as_bool();
    debug_image_ = this->get_parameter("debug_image").as_bool();

    RCLCPP_INFO(get_logger(),
                "CaisseDetector started — is_blue=%s  debug=%s",
                is_blue_ ? "true" : "false",
                debug_image_ ? "true" : "false");

    // ── Publishers ───────────────────────────────────────────────────────────────
    sides_pub_ = create_publisher<krabi_msgs::msg::CaissesSides>("~/caisses_sides", 10);

    if (debug_image_)
    {
        debug_pub_ = create_publisher<sensor_msgs::msg::Image>("~/debug_image", 10);
    }

    // ── Subscriber ───────────────────────────────────────────────────────────────
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "krabi_cam/image_raw",
      10,
      std::bind(&CaisseDetectorNode::imageCb, this, std::placeholders::_1));

    // ── Live parameter updates ───────────────────────────────────────────────────
    param_cb_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& params)
      {
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = true;
          for (auto& p : params)
          {
              if (p.get_name() == "is_blue")
                  is_blue_ = p.as_bool();
              else if (p.get_name() == "debug_image")
                  debug_image_ = p.as_bool();
          }
          return result;
      });
}

// ── regionIsBlue ─────────────────────────────────────────────────────────────
bool CaisseDetectorNode::regionIsBlue(const cv::Mat& hsv, const cv::Rect& bbox)
{
    // Sample only the top 60 % — avoids the dark QR-code symbol at the bottom
    cv::Rect roi = bbox;
    roi.height = static_cast<int>(bbox.height * 0.6);
    if (roi.height < 1)
        roi.height = 1;

    cv::Mat region = hsv(roi);

    cv::Mat blue_mask, yellow_mask;
    cv::inRange(region, BLUE_LOW, BLUE_HIGH, blue_mask);
    cv::inRange(region, YELLOW_LOW, YELLOW_HIGH, yellow_mask);

    int blue_px = cv::countNonZero(blue_mask);
    int yellow_px = cv::countNonZero(yellow_mask);

    return blue_px >= yellow_px; // tie → treat as blue
}

// ── tileIsOurSide ─────────────────────────────────────────────────────────────
bool CaisseDetectorNode::tileIsOurSide(const Tile& t) const
{
    return is_blue_ ? t.is_blue : !t.is_blue;
}

// ── imageCb ───────────────────────────────────────────────────────────────────
void CaisseDetectorNode::imageCb(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // Convert to OpenCV BGR
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
    }
    catch (const cv_bridge::Exception& e)
    {
        RCLCPP_ERROR(get_logger(), "cv_bridge: %s", e.what());
        return;
    }
    cv::Mat img = cv_ptr->image;

    // BGR → HSV
    cv::Mat hsv;
    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);

    // Combined tile mask (blue OR yellow)
    cv::Mat mask_tiles;
    cv::bitwise_or(
      colourMask(hsv, BLUE_LOW, BLUE_HIGH), colourMask(hsv, YELLOW_LOW, YELLOW_HIGH), mask_tiles);

    // Find and filter contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask_tiles, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<Tile> tiles;
    for (auto& cnt : contours)
    {
        if (cv::contourArea(cnt) < MIN_TILE_AREA)
            continue;
        cv::Rect bbox = cv::boundingRect(cnt);
        int cx = bbox.x + bbox.width / 2;
        tiles.push_back({ bbox, regionIsBlue(hsv, bbox), cx });
    }

    // Sort left-to-right
    std::sort(tiles.begin(), tiles.end(), [](const Tile& a, const Tile& b) { return a.cx < b.cx; });

    // Guard: need exactly N_TILES
    if (static_cast<int>(tiles.size()) != N_TILES)
    {
        RCLCPP_WARN_THROTTLE(get_logger(),
                             *get_clock(),
                             2000,
                             "Expected %d tiles, found %zu — skipping frame",
                             N_TILES,
                             tiles.size());
        if (debug_image_ && debug_pub_)
            publishDebug(img, tiles, msg->header);
        return;
    }

    // Publish CaissesSides
    auto out = krabi_msgs::msg::CaissesSides();
    out.leftmost_is_our_side_up = tileIsOurSide(tiles[0]);
    out.leftcenter_is_our_side_up = tileIsOurSide(tiles[1]);
    out.rightcenter_is_our_side_up = tileIsOurSide(tiles[2]);
    out.rightmost_is_our_side_up = tileIsOurSide(tiles[3]);
    sides_pub_->publish(out);

    RCLCPP_DEBUG(get_logger(),
                 "Sides: [%d %d %d %d]",
                 out.leftmost_is_our_side_up,
                 out.leftcenter_is_our_side_up,
                 out.rightcenter_is_our_side_up,
                 out.rightmost_is_our_side_up);

    if (debug_image_ && debug_pub_)
        publishDebug(img, tiles, msg->header);
}

// ── publishDebug ──────────────────────────────────────────────────────────────
void CaisseDetectorNode::publishDebug(const cv::Mat& img,
                                      const std::vector<Tile>& tiles,
                                      const std_msgs::msg::Header& header)
{
    cv::Mat dbg = img.clone();

    static const std::vector<std::string> pos_labels
      = { "LEFTMOST", "LEFT-CTR", "RIGHT-CTR", "RIGHTMOST" };

    for (size_t i = 0; i < tiles.size(); ++i)
    {
        const Tile& t = tiles[i];
        bool ours = tileIsOurSide(t);
        cv::Scalar colour = ours ? cv::Scalar(0, 220, 0) : cv::Scalar(0, 0, 220);

        cv::rectangle(dbg, t.bbox, colour, 3);

        std::string label = (i < pos_labels.size() ? pos_labels[i] : std::to_string(i))
                            + (t.is_blue ? " [BLUE]" : " [YEL]") + (ours ? " OUR" : "");

        cv::putText(
          dbg, label, { t.bbox.x, t.bbox.y - 8 }, cv::FONT_HERSHEY_SIMPLEX, 0.55, colour, 2);
    }

    debug_pub_->publish(*cv_bridge::CvImage(header, "bgr8", dbg).toImageMsg());
}

// ── main ──────────────────────────────────────────────────────────────────────
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CaisseDetectorNode>());
    rclcpp::shutdown();
    return 0;
}