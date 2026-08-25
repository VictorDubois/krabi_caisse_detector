#include "caisse_detector_node.h"

#include <algorithm>

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
    // ── Declare parameters ────────────────────────────────────────────────────
    this->declare_parameter<bool>("is_blue", true);
    this->declare_parameter<bool>("debug_image", false);
    this->declare_parameter<int>("history_duration_ms", 1000);
    this->declare_parameter<double>("min_color_coverage", 0.05);

    for (int i = 0; i < N_TILES; i++)
    {
        std::string pfx = std::string("roi_") + ROI_NAMES[i] + "_";
        this->declare_parameter<int>(pfx + "x", 0);
        this->declare_parameter<int>(pfx + "y", 0);
        this->declare_parameter<int>(pfx + "width", 0);
        this->declare_parameter<int>(pfx + "height", 0);
    }

    // ── Read initial values ───────────────────────────────────────────────────
    is_blue_ = this->get_parameter("is_blue").as_bool();
    debug_image_ = this->get_parameter("debug_image").as_bool();
    history_duration_ms_ = this->get_parameter("history_duration_ms").as_int();
    min_color_coverage_ = this->get_parameter("min_color_coverage").as_double();

    for (int i = 0; i < N_TILES; i++)
    {
        std::string pfx = std::string("roi_") + ROI_NAMES[i] + "_";
        rois_[i].x = this->get_parameter(pfx + "x").as_int();
        rois_[i].y = this->get_parameter(pfx + "y").as_int();
        rois_[i].w = this->get_parameter(pfx + "width").as_int();
        rois_[i].h = this->get_parameter(pfx + "height").as_int();
    }

    RCLCPP_INFO(get_logger(),
                "CaisseDetector started — is_blue=%s  debug=%s  history=%dms  "
                "min_color_coverage=%.3f",
                is_blue_ ? "true" : "false",
                debug_image_ ? "true" : "false",
                history_duration_ms_,
                min_color_coverage_);
    for (int i = 0; i < N_TILES; i++)
        RCLCPP_INFO(get_logger(),
                    "  RoI[%s]: (%d, %d  %dx%d)",
                    ROI_NAMES[i],
                    rois_[i].x,
                    rois_[i].y,
                    rois_[i].w,
                    rois_[i].h);

    // ── Publishers ────────────────────────────────────────────────────────────
    sides_pub_ = create_publisher<krabi_msgs::msg::CaissesSides>("caisses_sides", 10);

    if (debug_image_)
        debug_pub_ = create_publisher<sensor_msgs::msg::CompressedImage>("debug_image", 10);

    // ── Subscriber ────────────────────────────────────────────────────────────
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "krabi_cam/image_raw", // /compressed",
      10,
      std::bind(&CaisseDetectorNode::imageCb, this, std::placeholders::_1));

    // ── Live parameter updates ────────────────────────────────────────────────
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
              else if (p.get_name() == "history_duration_ms")
                  history_duration_ms_ = p.as_int();
              else if (p.get_name() == "min_color_coverage")
                  min_color_coverage_ = p.as_double();
              else
              {
                  for (int i = 0; i < N_TILES; i++)
                  {
                      std::string pfx = std::string("roi_") + ROI_NAMES[i] + "_";
                      if (p.get_name() == pfx + "x")
                          rois_[i].x = p.as_int();
                      else if (p.get_name() == pfx + "y")
                          rois_[i].y = p.as_int();
                      else if (p.get_name() == pfx + "width")
                          rois_[i].w = p.as_int();
                      else if (p.get_name() == pfx + "height")
                          rois_[i].h = p.as_int();
                  }
              }
          }
          return result;
      });
}

// ── effectiveRoi ──────────────────────────────────────────────────────────────
cv::Rect CaisseDetectorNode::effectiveRoi(int idx, int img_w, int img_h) const
{
    const RoiParams& r = rois_[idx];
    int x = std::clamp(r.x, 0, img_w - 1);
    int y = std::clamp(r.y, 0, img_h - 1);
    int w = (r.w > 0) ? std::clamp(r.w, 1, img_w - x) : (img_w - x);
    int h = (r.h > 0) ? std::clamp(r.h, 1, img_h - y) : (img_h - y);
    return { x, y, w, h };
}

// ── tileIsOurSide ─────────────────────────────────────────────────────────────
bool CaisseDetectorNode::tileIsOurSide(bool is_blue) const
{
    return is_blue_ ? is_blue : !is_blue;
}

// ── imageCb ───────────────────────────────────────────────────────────────────
void CaisseDetectorNode::imageCb(const sensor_msgs::msg::Image::SharedPtr msg)
{
    if ((nb_images_received++) % 6 != 0)
    {
        // Only process 1/3 of the images to reduce CPU load
        return;
    }

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
    const cv::Mat& full_img = cv_ptr->image;

    auto now = std::chrono::steady_clock::now();
    auto cutoff = now - std::chrono::milliseconds(history_duration_ms_);

    std::array<cv::Rect, N_TILES> rects{};
    std::array<bool, N_TILES> results{};
    std::array<bool, N_TILES> valid{};

    for (int i = 0; i < N_TILES; i++)
    {
        cv::Rect roi = effectiveRoi(i, full_img.cols, full_img.rows);
        rects[i] = roi;
        cv::Mat roi_view = full_img(roi); // zero-copy view

        cv::Mat hsv;
        cv::cvtColor(roi_view, hsv, cv::COLOR_BGR2HSV);

        cv::Mat blue_mask = colourMask(hsv, BLUE_LOW, BLUE_HIGH);
        cv::Mat yellow_mask = colourMask(hsv, YELLOW_LOW, YELLOW_HIGH);

        int blue_px = cv::countNonZero(blue_mask);
        int yellow_px = cv::countNonZero(yellow_mask);
        int total_px = roi.width * roi.height;

        double coverage = static_cast<double>(blue_px + yellow_px) / total_px;

        if (coverage < min_color_coverage_)
        {
            RCLCPP_DEBUG(get_logger(),
                         "RoI[%s] low colour coverage (%.3f < %.3f) — skipping",
                         ROI_NAMES[i],
                         coverage,
                         min_color_coverage_);
            valid[i] = false;
            continue;
        }
        valid[i] = true;

        bool is_blue = (blue_px >= yellow_px);
        results[i] = tileIsOurSide(is_blue);

        history_[i].push_back({ results[i], now });

        // Evict expired snapshots
        while (!history_[i].empty() && history_[i].front().timestamp < cutoff)
            history_[i].pop_front();
    }

    if (debug_image_ && debug_pub_)
        publishDebug(full_img, rects, results, valid, msg->header);

    publishVotedResult(msg->header);
}

// ── publishVotedResult ────────────────────────────────────────────────────────
void CaisseDetectorNode::publishVotedResult(const std_msgs::msg::Header& /*header*/)
{
    std::array<int, N_TILES> votes{};
    std::array<int, N_TILES> totals{};

    for (int i = 0; i < N_TILES; i++)
    {
        totals[i] = static_cast<int>(history_[i].size());
        for (auto& snap : history_[i])
            if (snap.is_our_side)
                ++votes[i];
    }

    auto out = krabi_msgs::msg::CaissesSides();
    out.leftmost_is_our_side_up = (totals[0] > 0) && (votes[0] * 2 >= totals[0]);
    out.leftcenter_is_our_side_up = (totals[1] > 0) && (votes[1] * 2 >= totals[1]);
    out.rightcenter_is_our_side_up = (totals[2] > 0) && (votes[2] * 2 >= totals[2]);
    out.rightmost_is_our_side_up = (totals[3] > 0) && (votes[3] * 2 >= totals[3]);

    sides_pub_->publish(out);

    RCLCPP_INFO(get_logger(),
                "Voted sides: [%d %d %d %d]  votes/total: [%d/%d %d/%d %d/%d %d/%d]",
                out.leftmost_is_our_side_up,
                out.leftcenter_is_our_side_up,
                out.rightcenter_is_our_side_up,
                out.rightmost_is_our_side_up,
                votes[0],
                totals[0],
                votes[1],
                totals[1],
                votes[2],
                totals[2],
                votes[3],
                totals[3]);
}

// ── publishDebug ──────────────────────────────────────────────────────────────
void CaisseDetectorNode::publishDebug(const cv::Mat& full_img,
                                      const std::array<cv::Rect, N_TILES>& rois,
                                      const std::array<bool, N_TILES>& is_our_side,
                                      const std::array<bool, N_TILES>& valid,
                                      const std_msgs::msg::Header& header)
{
    cv::Mat dbg = full_img.clone();

    for (int i = 0; i < N_TILES; i++)
    {
        const cv::Rect& roi = rois[i];

        // Compute voted result for this RoI
        int hist_size = static_cast<int>(history_[i].size());
        int v = 0;
        for (auto& s : history_[i])
            if (s.is_our_side)
                ++v;
        bool voted = (hist_size > 0) && (v * 2 >= hist_size);

        // Grey = no coverage, green = our side, red = their side
        cv::Scalar colour = !valid[i] ? cv::Scalar(128, 128, 128)
                            : voted   ? cv::Scalar(0, 220, 0)
                                      : cv::Scalar(0, 0, 220);

        cv::rectangle(dbg, roi, colour, 2);

        std::string label = ROI_NAMES[i];
        if (!valid[i])
            label += " LOW";
        else
            label += is_our_side[i] ? " OUR" : " THEIR";
        label += " v" + std::to_string(v) + "/" + std::to_string(hist_size);

        cv::putText(
          dbg, label, { roi.x + 4, roi.y + 20 }, cv::FONT_HERSHEY_SIMPLEX, 0.5, colour, 2);
    }

    debug_pub_->publish(*cv_bridge::CvImage(header, "bgr8", dbg).toCompressedImageMsg());
}

// ── main ──────────────────────────────────────────────────────────────────────
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CaisseDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
