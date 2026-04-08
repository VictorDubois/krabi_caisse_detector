#include "caisse_detector_node.h"

#include <algorithm>
#include <cmath>

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

// ── splitBlobVertically ───────────────────────────────────────────────────────
std::vector<cv::Rect> splitBlobVertically(const cv::Rect& bbox, int n)
{
    std::vector<cv::Rect> slices;
    int slice_w = bbox.width / n;
    for (int i = 0; i < n; ++i)
    {
        int x = bbox.x + i * slice_w;
        int w = (i == n - 1) ? (bbox.x + bbox.width - x) : slice_w;
        slices.push_back({ x, bbox.y, w, bbox.height });
    }
    return slices;
}

// ── estimateTileWidth ─────────────────────────────────────────────────────────
int estimateTileWidth(const std::vector<cv::Rect>& blobs)
{
    if (blobs.empty())
        return 0;
    int min_w = blobs[0].width;
    for (auto& b : blobs)
        min_w = std::min(min_w, b.width);
    return min_w;
}

// ── Constructor ───────────────────────────────────────────────────────────────
CaisseDetectorNode::CaisseDetectorNode()
  : Node("caisse_detector")
{
    // ── Declare parameters ────────────────────────────────────────────────────
    this->declare_parameter<bool>("is_blue", true);
    this->declare_parameter<bool>("debug_image", false);
    this->declare_parameter<int>("history_duration_ms", 1000);
    this->declare_parameter<double>("min_tile_area_fraction", 0.005);
    this->declare_parameter<int>("roi_x", 0);
    this->declare_parameter<int>("roi_y", 0);
    this->declare_parameter<int>("roi_width", 0);
    this->declare_parameter<int>("roi_height", 0);

    // ── Read initial values ───────────────────────────────────────────────────
    is_blue_ = this->get_parameter("is_blue").as_bool();
    debug_image_ = this->get_parameter("debug_image").as_bool();
    history_duration_ms_ = this->get_parameter("history_duration_ms").as_int();
    min_tile_area_fraction_ = this->get_parameter("min_tile_area_fraction").as_double();
    roi_x_ = this->get_parameter("roi_x").as_int();
    roi_y_ = this->get_parameter("roi_y").as_int();
    roi_width_ = this->get_parameter("roi_width").as_int();
    roi_height_ = this->get_parameter("roi_height").as_int();

    RCLCPP_INFO(get_logger(),
                "CaisseDetector started — is_blue=%s  debug=%s  history=%dms  "
                "min_area_frac=%.4f  ROI=(%d,%d %dx%d)",
                is_blue_ ? "true" : "false",
                debug_image_ ? "true" : "false",
                history_duration_ms_,
                min_tile_area_fraction_,
                roi_x_,
                roi_y_,
                roi_width_,
                roi_height_);

    // ── Publishers ────────────────────────────────────────────────────────────
    sides_pub_ = create_publisher<krabi_msgs::msg::CaissesSides>("caisses_sides", 10);

    if (debug_image_)
    {
        debug_pub_ = create_publisher<sensor_msgs::msg::Image>("debug_image", 10);
    }

    // ── Subscriber ────────────────────────────────────────────────────────────
    image_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
      "krabi_cam/image_raw/compressed",
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
              else if (p.get_name() == "min_tile_area_fraction")
                  min_tile_area_fraction_ = p.as_double();
              else if (p.get_name() == "roi_x")
                  roi_x_ = p.as_int();
              else if (p.get_name() == "roi_y")
                  roi_y_ = p.as_int();
              else if (p.get_name() == "roi_width")
                  roi_width_ = p.as_int();
              else if (p.get_name() == "roi_height")
                  roi_height_ = p.as_int();
          }
          return result;
      });
}

// ── effectiveRoi ──────────────────────────────────────────────────────────────
cv::Rect CaisseDetectorNode::effectiveRoi(int img_w, int img_h) const
{
    int x = roi_x_;
    int y = roi_y_;
    int w = (roi_width_ > 0) ? roi_width_ : img_w;
    int h = (roi_height_ > 0) ? roi_height_ : img_h;

    x = std::clamp(x, 0, img_w - 1);
    y = std::clamp(y, 0, img_h - 1);
    w = std::clamp(w, 1, img_w - x);
    h = std::clamp(h, 1, img_h - y);

    return { x, y, w, h };
}

// ── minTileArea ───────────────────────────────────────────────────────────────
double CaisseDetectorNode::minTileArea(const cv::Rect& roi) const
{
    return min_tile_area_fraction_ * roi.width * roi.height;
}

// ── colourPurity ──────────────────────────────────────────────────────────────
// Returns [0, 1]: fraction of the ROI that is either blue or yellow.
double CaisseDetectorNode::colourPurity(const cv::Mat& hsv, const cv::Rect& bbox)
{
    cv::Rect roi = bbox;
    roi.height = static_cast<int>(bbox.height);
    if (roi.height < 1)
        roi.height = 1;
    roi &= cv::Rect(0, 0, hsv.cols, hsv.rows);
    if (roi.empty())
        return 0.0;

    cv::Mat region = hsv(roi);

    cv::Mat blue_mask, yellow_mask, combined;
    cv::inRange(region, BLUE_LOW, BLUE_HIGH, blue_mask);
    cv::inRange(region, YELLOW_LOW, YELLOW_HIGH, yellow_mask);
    cv::bitwise_or(blue_mask, yellow_mask, combined);

    int total_px = roi.width * roi.height;
    int colour_px = cv::countNonZero(combined);

    return (total_px > 0) ? static_cast<double>(colour_px) / total_px : 0.0;
}

// ── regionIsBlue ─────────────────────────────────────────────────────────────
bool CaisseDetectorNode::regionIsBlue(const cv::Mat& hsv, const cv::Rect& bbox)
{
    cv::Rect roi = bbox;
    roi.height = static_cast<int>(bbox.height * 0.6);
    if (roi.height < 1)
        roi.height = 1;
    roi &= cv::Rect(0, 0, hsv.cols, hsv.rows);
    if (roi.empty())
        return false;

    cv::Mat region = hsv(roi);
    cv::Mat blue_mask, yellow_mask;
    cv::inRange(region, BLUE_LOW, BLUE_HIGH, blue_mask);
    cv::inRange(region, YELLOW_LOW, YELLOW_HIGH, yellow_mask);

    return cv::countNonZero(blue_mask) >= cv::countNonZero(yellow_mask);
}

// ── tileIsOurSide ─────────────────────────────────────────────────────────────
bool CaisseDetectorNode::tileIsOurSide(const Tile& t) const
{
    return is_blue_ ? t.is_blue : !t.is_blue;
}

// ── imageCb ───────────────────────────────────────────────────────────────────
void CaisseDetectorNode::imageCb(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
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
    cv::Mat full_img = cv_ptr->image;

    // ── Crop to ROI ───────────────────────────────────────────────────────────
    cv::Rect roi = effectiveRoi(full_img.cols, full_img.rows);
    cv::Mat img = full_img(roi); // zero-copy view

    // ── BGR → HSV ─────────────────────────────────────────────────────────────
    cv::Mat hsv;
    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);

    // ── Combined tile mask ────────────────────────────────────────────────────
    cv::Mat mask_tiles;
    cv::bitwise_or(
      colourMask(hsv, BLUE_LOW, BLUE_HIGH), colourMask(hsv, YELLOW_LOW, YELLOW_HIGH), mask_tiles);

    // ── Find raw blobs, filter by resolution-relative area ───────────────────
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask_tiles, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double min_area = minTileArea(roi);
    std::vector<cv::Rect> raw_blobs;
    for (auto& cnt : contours)
    {
        if (cv::contourArea(cnt) < min_area)
            continue;
        raw_blobs.push_back(cv::boundingRect(cnt));
    }

    // ── Split merged blobs (always into exactly 2) ────────────────────────────
    int tile_w = estimateTileWidth(raw_blobs);

    std::vector<Tile> candidates;
    for (auto& blob : raw_blobs)
    {
        bool is_merged
          = (tile_w > 0) && (static_cast<double>(blob.width) / tile_w > MERGE_RATIO_THRESHOLD);

        auto add_tile = [&](const cv::Rect& r)
        {
            double purity = colourPurity(hsv, r);
            double area = static_cast<double>(r.width * r.height);
            double score = purity * std::sqrt(area);
            int cx = r.x + r.width / 2;
            candidates.push_back({ r, regionIsBlue(hsv, r), cx, score });
        };

        if (is_merged)
        {
            RCLCPP_DEBUG(get_logger(),
                         "Splitting merged blob (w=%d) into 2 tiles (tile_w≈%d)",
                         blob.width,
                         tile_w);
            for (auto& slice : splitBlobVertically(blob, 2))
                add_tile(slice);
        }
        else
        {
            add_tile(blob);
        }
    }

    // ── Keep the N_TILES best-scoring candidates ──────────────────────────────
    if (static_cast<int>(candidates.size()) > N_TILES)
    {
        std::partial_sort(candidates.begin(),
                          candidates.begin() + N_TILES,
                          candidates.end(),
                          [](const Tile& a, const Tile& b)
                          {
                              return a.score > b.score; // descending
                          });
        candidates.resize(N_TILES);
    }

    // ── Sort surviving candidates left-to-right ───────────────────────────────
    std::sort(candidates.begin(),
              candidates.end(),
              [](const Tile& a, const Tile& b) { return a.cx < b.cx; });

    // ── Debug image (drawn before the count guard, always visible) ───────────
    if (debug_image_ && debug_pub_)
        publishDebug(full_img, candidates, roi, msg->header);

    // ── Still need exactly N_TILES after selection ────────────────────────────
    // (can only be < N_TILES if the scene genuinely has fewer blobs)
    if (static_cast<int>(candidates.size()) != N_TILES)
    {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Only %zu blobs found after filtering (min_area=%.0f px²) — skipping frame",
          candidates.size(),
          min_area);
        return;
    }

    // RCLCPP_INFO_STREAM(get_logger(), "Valid detection!");

    // ── Store valid detection in history ─────────────────────────────────────
    DetectionSnapshot snap;
    snap.timestamp = std::chrono::steady_clock::now();
    for (int i = 0; i < N_TILES; ++i)
        snap.is_our_side[i] = tileIsOurSide(candidates[i]);
    history_.push_back(snap);

    // ── Evict old snapshots ───────────────────────────────────────────────────
    auto cutoff
      = std::chrono::steady_clock::now() - std::chrono::milliseconds(history_duration_ms_);
    while (!history_.empty() && history_.front().timestamp < cutoff)
        history_.pop_front();

    // ── Publish majority-voted result ─────────────────────────────────────────
    publishVotedResult(msg->header);
}

// ── publishVotedResult ────────────────────────────────────────────────────────
void CaisseDetectorNode::publishVotedResult(const std_msgs::msg::Header& /*header*/)
{
    if (history_.empty())
    {
        RCLCPP_INFO_STREAM(get_logger(), "Empty history!");

        return;
    }

    std::array<int, N_TILES> votes = { 0, 0, 0, 0 };
    for (auto& snap : history_)
        for (int i = 0; i < N_TILES; ++i)
            if (snap.is_our_side[i])
                ++votes[i];

    int total = static_cast<int>(history_.size());

    auto out = krabi_msgs::msg::CaissesSides();
    out.leftmost_is_our_side_up = (votes[0] * 2 >= total);
    out.leftcenter_is_our_side_up = (votes[1] * 2 >= total);
    out.rightcenter_is_our_side_up = (votes[2] * 2 >= total);
    out.rightmost_is_our_side_up = (votes[3] * 2 >= total);

    sides_pub_->publish(out);

    RCLCPP_INFO(get_logger(),
                "Voted sides (n=%d): [%d %d %d %d]  votes: [%d %d %d %d]",
                total,
                out.leftmost_is_our_side_up,
                out.leftcenter_is_our_side_up,
                out.rightcenter_is_our_side_up,
                out.rightmost_is_our_side_up,
                votes[0],
                votes[1],
                votes[2],
                votes[3]);

    // RCLCPP_INFO_STREAM(get_logger(), "Published detection!");
}

// ── publishDebug ──────────────────────────────────────────────────────────────
void CaisseDetectorNode::publishDebug(const cv::Mat& full_img,
                                      const std::vector<Tile>& tiles,
                                      const cv::Rect& roi,
                                      const std_msgs::msg::Header& header)
{
    cv::Mat dbg = full_img.clone();

    // ── ROI rectangle (white) ─────────────────────────────────────────────────
    cv::rectangle(dbg, roi, cv::Scalar(255, 255, 255), 2);
    cv::putText(dbg,
                "ROI",
                { roi.x + 4, roi.y + 18 },
                cv::FONT_HERSHEY_SIMPLEX,
                0.55,
                cv::Scalar(255, 255, 255),
                2);

    // ── Tile bounding boxes ───────────────────────────────────────────────────
    static const std::vector<std::string> pos_labels
      = { "LEFTMOST", "LEFT-CTR", "RIGHT-CTR", "RIGHTMOST" };

    for (size_t i = 0; i < tiles.size(); ++i)
    {
        const Tile& t = tiles[i];
        bool ours = tileIsOurSide(t);
        cv::Scalar colour = ours ? cv::Scalar(0, 220, 0) : cv::Scalar(0, 0, 220);

        // Translate from ROI space → full image space
        cv::Rect full_bbox = t.bbox + cv::Point(roi.x, roi.y);
        cv::rectangle(dbg, full_bbox, colour, 3);

        std::string label = (i < pos_labels.size() ? pos_labels[i] : std::to_string(i))
                            + (t.is_blue ? " [BLUE]" : " [YEL]") + (ours ? " OUR" : "")
                            + "  s=" + std::to_string(static_cast<int>(t.score));

        cv::putText(
          dbg, label, { full_bbox.x, full_bbox.y + 24 }, cv::FONT_HERSHEY_SIMPLEX, 0.5, colour, 2);
    }

    // ── History buffer size overlay ───────────────────────────────────────────
    std::string hist_label = "history: " + std::to_string(history_.size()) + " frames";
    cv::putText(dbg,
                hist_label,
                { roi.x + 4, roi.y + roi.height - 8 },
                cv::FONT_HERSHEY_SIMPLEX,
                0.5,
                cv::Scalar(200, 200, 0),
                2);

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