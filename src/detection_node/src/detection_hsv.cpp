#include<iostream>
#include<ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include<opencv2/opencv.hpp>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/image_encodings.h>
#include<dynamic_reconfigure/server.h>
#include<detection_node/detectionConfig.h>
#include<cmath>
#include<algorithm>
#include<mutex>
#include<string>

using namespace std; 

struct LightBar {
    cv::Point2f top;      // 上端两点中点
    cv::Point2f bottom;   // 下端两点中点
    cv::Point2f center;   // 灯条中心
    float angle;    // 相对竖直方向的夹角（度），右倾为正、左倾为负
    float length;   // 灯条长度 |top - bottom|
};

struct ArmorPair {
    int i;   // 左灯条下标
    int j;   // 右灯条下标
};

struct ArmorLight{
    float width;   // 装甲板宽度
    float height;  // 装甲板高度
    cv::Point2f center;  // 装甲板中心
    float angle_longSize;  // 装甲板长边相对竖直方向的夹角（度），右倾为正、左倾为负
    float angle_shortSize; // 装甲板短边相对竖直方向的夹角（度），右倾为正、左倾为负
    float angle_mid;       // 两灯条**中点连线**相对竖直方向的夹角（度），两灯条齐平时 ≈ 90°
    cv::Point2f vertices[4];  // 装甲板四个顶点，顺时针排列，顶点 0 为左上角
    float max_gap_ratio_debug; // 间距上限 = N × 灯条长度
    float max_angle_diff_debug;// 倾角差上限（度）
    float min_len_ratio_debug;   // 长度比下限（短/长）
    float min_long_angle_debug; // 长边与竖直夹角的下限（度）
    float max_edge_dy_debug;    // 上下边两端 y 差上限（像素）
    cv::Point2f mid;  // 两灯条中点连线的中点

};

ros::Publisher pub_img_mask;
ros::Publisher pub_img_mask_debug;
ros::Publisher pub_img_mask_armor;

cv_bridge::CvImage out_msg_mask;
cv_bridge::CvImage out_msg_mask_debug;
cv_bridge::CvImage out_msg_mask_armor;

static void ImageCallback(const sensor_msgs::CompressedImage::ConstPtr& msg);
static double fps_statistics(double t);
static cv::Mat Range(const cv::Mat& img, const detection_node::detectionConfig& cfg);
static void cfgCallback(detection_node::detectionConfig &config, uint32_t);
static void FindArmor(const cv::Mat& img_mask, cv::Mat& img_debug, cv::Mat& img_armor,
                      const detection_node::detectionConfig& cfg);
static double DistanceMeasurement();

int main(int argc, char** argv){
    
    ros::init(argc, argv,"delection_hsv");
    ros::NodeHandle nh;
    ros::Subscriber sub_img=nh.subscribe("/hk_camera/image_raw/compressed",1,ImageCallback);
    //ros::Subscriber sub_camera_info=nh.subscribe("/hk_camera/camera_info",1,DistanceMeasurement);
    pub_img_mask = nh.advertise<sensor_msgs::Image>("/hk_camera/image_mask",1);
    pub_img_mask_debug = nh.advertise<sensor_msgs::Image>("/hk_camera/image_mask_debug",1);
    pub_img_mask_armor = nh.advertise<sensor_msgs::Image>("/hk_camera/image_mask_armor",1);
    //ros::Subscriber sub_camera_info=nh.subscribe("/hk_camera/camera_info",1,imageCallback);
    //ROS_INFO(sub_camera_info.getTopic().c_str());
    dynamic_reconfigure::Server<detection_node::detectionConfig> server;
    server.setCallback([](detection_node::detectionConfig &config, uint32_t level) {cfgCallback(config, level);});

    ros::spin();
    return 0;
}

static std::mutex g_cfg_mutex;
static detection_node::detectionConfig g_cfg;
static double g_min_fps = 0.0;   
static double g_max_fps = 0.0;
static int    g_warmup  = 10;  
static double fps_statistics(double t){
    const double fps = (t > 1e-9) ? (1.0 / t) : 0.0;
    if (g_warmup > 0) {
        --g_warmup;                                 
    } else if (fps > 0.0) {
        g_min_fps = (g_min_fps <= 0.0 || fps < g_min_fps) ? fps : g_min_fps;
        g_max_fps = std::max(g_max_fps, fps);
    }
    return fps;
}

static void ImageCallback(const sensor_msgs::CompressedImage::ConstPtr& msg){
    
    if (msg->data.empty()) return;

    detection_node::detectionConfig cfg;{
        std::lock_guard<std::mutex> lock(g_cfg_mutex);
        cfg = g_cfg;
    }

    double t = (double)cv::getTickCount();

    cv::Mat img_mask,img_bf,img_debug,img_armor;
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
    cv::Mat img = cv_ptr->image;
    cv::bilateralFilter(img, img_bf, 5, 75.0, 75.0);
    //cv::GaussianBlur(img_bf, img_bf, cv::Size(3, 3), 0);

    img_debug = img_bf.clone();
    
    img_mask = Range(img_bf, cfg);
    
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3), cv::Point(-1, -1));
    cv::morphologyEx(img_mask, img_mask, 
                    cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_RECT, 
                    cv::Size(3,3), cv::Point(-1, -1)));
    cv::morphologyEx(img_mask, img_mask, 
                    cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, 
                    cv::Size(5,5), cv::Point(-1, -1)));
    cv::erode(img_mask, img_mask, kernel, cv::Point(-1, -1));
    
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    double fps = fps_statistics(t);
    cv::putText(img_debug,
                cv::format("FPS: %.1f | max: %.1f | min: %.1f", fps, g_max_fps, g_min_fps),
                cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8,
                cv::Scalar(0, 255, 0), 2, cv::LINE_AA);

    FindArmor(img_mask, img_debug, img_armor, cfg);

    out_msg_mask.header = msg->header;
    out_msg_mask.encoding = sensor_msgs::image_encodings::MONO8;
    out_msg_mask.image = img_mask;
    pub_img_mask.publish(out_msg_mask.toImageMsg());
    
    out_msg_mask_debug.header = msg->header;
    out_msg_mask_debug.encoding = sensor_msgs::image_encodings::BGR8;
    out_msg_mask_debug.image = img_debug;
    pub_img_mask_debug.publish(out_msg_mask_debug.toImageMsg());

    if (!img_armor.empty()) {
        out_msg_mask_armor.header = msg->header;
        out_msg_mask_armor.encoding = sensor_msgs::image_encodings::BGR8;
        out_msg_mask_armor.image = img_armor;
        pub_img_mask_armor.publish(out_msg_mask_armor.toImageMsg());
    }
}

static void cfgCallback(detection_node::detectionConfig &config, uint32_t){
    std::lock_guard<std::mutex> lock(g_cfg_mutex);
    g_cfg = config;
}

static cv::Mat Range(const cv::Mat& img, const detection_node::detectionConfig& cfg){

    cv::Mat img_hsv;
    cv::cvtColor(img, img_hsv, cv::COLOR_BGR2HSV);

    const cv::Scalar hsv_min = cfg.use_blue
        ? cv::Scalar(cfg.blue_h_min, cfg.blue_s_min, cfg.blue_v_min)
        : cv::Scalar(cfg.red_h_min,  cfg.red_s_min,  cfg.red_v_min);
    const cv::Scalar hsv_max = cfg.use_blue
        ? cv::Scalar(cfg.blue_h_max, cfg.blue_s_max, cfg.blue_v_max)
        : cv::Scalar(cfg.red_h_max,  cfg.red_s_max,  cfg.red_v_max);

    cv::Mat img_mask;
    cv::inRange(img_hsv, hsv_min, hsv_max, img_mask);
    return img_mask;
}

// 前向声明：放在结构体定义之后（放回文件开头会报 ArmorPair was not declared）
static vector<ArmorPair> Pairing(const vector<LightBar>& bars,
                                 const detection_node::detectionConfig& cfg);
static vector<ArmorPair> Pairing_debug(const vector<LightBar>& bars,
                                       const detection_node::detectionConfig& cfg);
static void DrawPairHud(cv::Mat& img, const ArmorLight& a, const LightBar& L, const LightBar& R,
                        const bool pass[7], const cv::Point& org);


// 两条直线 p1p2 与 p3p4 的交点（装甲板的两条对角线）
static cv::Point2f LineIntersect(const cv::Point2f& p1, const cv::Point2f& p2,
                                 const cv::Point2f& p3, const cv::Point2f& p4)
{
    const float d = (p1.x - p2.x) * (p3.y - p4.y) - (p1.y - p2.y) * (p3.x - p4.x);
    if (std::fabs(d) < 1e-6f)                        // 退化：两条对角线几乎平行
        return (p1 + p2 + p3 + p4) * 0.25f;
    const float a = p1.x * p2.y - p1.y * p2.x;
    const float b = p3.x * p4.y - p3.y * p4.x;
    return cv::Point2f((a * (p3.x - p4.x) - (p1.x - p2.x) * b) / d,
                       (a * (p3.y - p4.y) - (p1.y - p2.y) * b) / d);
}

// 方向向量 → 相对竖直方向的夹角（度），右倾为正、左倾为负，结果落在 (-90, 90]
static float LineAngleFromVertical(const cv::Point2f& d)
{
    float a = static_cast<float>(std::atan2(d.x, -d.y) * 180.0 / CV_PI);
    if (a >  90.f) a -= 180.f;
    if (a <= -90.f) a += 180.f;
    return a;
}

// 让 0 号点落到 x+y 最小的角上（只循环移位，保持原来的绕行方向）
static void RotateToTopLeft(cv::Point2f v[4])
{
    int k = 0;
    for (int i = 1; i < 4; ++i)
        if (v[i].x + v[i].y < v[k].x + v[k].y) k = i;
    if (k == 0) return;
    cv::Point2f tmp[4];
    for (int i = 0; i < 4; ++i) tmp[i] = v[(k + i) % 4];
    for (int i = 0; i < 4; ++i) v[i] = tmp[i];
}

// 两根灯条 → 装甲板
static ArmorLight MakeArmorLight(const LightBar& L, const LightBar& R)
{
    ArmorLight a;

    // 左上 = 左灯条上端 → 右上 → 右下 → 左下（图像上的顺时针）
    a.vertices[0] = L.top;
    a.vertices[1] = R.top;
    a.vertices[2] = R.bottom;
    a.vertices[3] = L.bottom;
    RotateToTopLeft(a.vertices);

    // 中心 = 两条对角线的交点
    a.center = LineIntersect(a.vertices[0], a.vertices[2], a.vertices[1], a.vertices[3]);

    // 宽 = 上下两条边的平均；高 = 左右两条边（= 灯条长度）的平均
    a.width  = 0.5f * (static_cast<float>(cv::norm(a.vertices[1] - a.vertices[0])) +
                       static_cast<float>(cv::norm(a.vertices[2] - a.vertices[3])));
    a.height = 0.5f * (static_cast<float>(cv::norm(a.vertices[3] - a.vertices[0])) +
                       static_cast<float>(cv::norm(a.vertices[2] - a.vertices[1])));

    // 长边 / 短边方向：按实际边长判长短，再统一换算成"相对竖直方向"的角度
    const cv::Point2f e_top   = a.vertices[1] - a.vertices[0];
    const cv::Point2f e_right = a.vertices[2] - a.vertices[1];
    const bool long_is_top = (a.width >= a.height);
    a.angle_longSize  = LineAngleFromVertical(long_is_top ? e_top   : e_right);
    a.angle_shortSize = LineAngleFromVertical(long_is_top ? e_right : e_top);

    // 调试用：把这一对灯条的**实际测量值**一并带出去，方便和 rqt 里的阈值对照
    const float gap     = std::fabs(L.center.x - R.center.x);
    const float len_avg = 0.5f * (L.length + R.length);
    const float len_max = std::max(L.length, R.length);
    a.max_gap_ratio_debug  = (len_avg > 1e-6f) ? gap / len_avg : 0.f;
    a.max_angle_diff_debug = std::fabs(L.angle - R.angle);
    a.min_len_ratio_debug  = (len_max > 1e-6f) ? std::min(L.length, R.length) / len_max : 0.f;
    a.min_long_angle_debug = std::fabs(a.angle_longSize);
    a.max_edge_dy_debug    = std::max(std::fabs(a.vertices[0].y - a.vertices[1].y),
                                      std::fabs(a.vertices[2].y - a.vertices[3].y));
    // 两灯条中点连线的中点（灯条中心就是灯条上/下端的中点）
    a.mid = (L.center + R.center) * 0.5f;
    // 两灯条中点连线相对竖直方向的夹角：由左灯条中心指向右灯条中心，
    // 两根灯条齐平（连线水平）时 ≈ 90°，一根高一跟低时变小
    a.angle_mid = std::fabs(LineAngleFromVertical(R.center - L.center));
    return a;
}

// 4 个角点 → 上端两点中点 / 下端两点中点
static void TopBottomMid(const cv::Point2f v[4], cv::Point2f& top_mid, cv::Point2f& bot_mid)
{
    std::vector<cv::Point2f> p(v, v + 4);
    std::sort(p.begin(), p.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
        if (a.y != b.y) return a.y < b.y;   // y 小的在上面
        return a.x < b.x;                   // 同高时靠左的在前
    });
    top_mid = (p[0] + p[1]) * 0.5f;
    bot_mid = (p[2] + p[3]) * 0.5f;
}


// 7 条判据是否通过，顺序与 Pairing() 里的 1)~7) 一一对应
static void CheckPair(const ArmorLight& a, const LightBar& L, const LightBar& R,
                      const detection_node::detectionConfig& cfg, bool pass[7])
{
    const float gap = std::fabs(L.center.x - R.center.x);
    const float ratio = static_cast<float>(cfg.pair_max_gap_ratio);
    const bool tilt_applicable = std::fabs(L.angle - R.angle) < 1.0f;

    pass[0] = a.min_len_ratio_debug  >= static_cast<float>(cfg.pair_min_len_ratio);    // 1) 长度比
    pass[1] = a.max_angle_diff_debug <= static_cast<float>(cfg.pair_max_angle_diff);   // 2) 倾角差
    pass[2] = !(tilt_applicable && cfg.pair_same_tilt && L.angle * R.angle < 0.0f);    // 3) 同向倾斜
    pass[3] = !(gap > ratio * L.length && gap > ratio * R.length);                     // 4) 间距
    pass[4] = a.min_long_angle_debug >= static_cast<float>(cfg.pair_min_long_angle);   // 5) 长边接近水平
    pass[5] = a.max_edge_dy_debug    <= static_cast<float>(cfg.pair_max_edge_dy);      // 6) 上下边 y 差
    pass[6] = a.angle_mid >= static_cast<float>(cfg.pair_min_mid_angle);  // 7) 两灯条中点连线接近水平
}

static vector<ArmorPair> Pairing(const vector<LightBar>& bars,
                                 const detection_node::detectionConfig& cfg)
{
    // 每根灯条最多参与几次配对（1 = 一根灯条只属于一块装甲）
    const int max_use = std::max(1, static_cast<int>(cfg.pair_max_use));

    // ---- 1) 把所有满足 7 条判据的候选对收起来，并打分 ----
    struct ScoredPair { int i; int j; float score; };
    vector<ScoredPair> cand;

    for (int i = 0; i < static_cast<int>(bars.size()); ++i) {
        for (int j = i + 1; j < static_cast<int>(bars.size()); ++j) {
            const LightBar& L = bars[i];
            const LightBar& R = bars[j];

            const ArmorLight armor = MakeArmorLight(L, R);

            // 七条判据全部通过才算成框（顺序：长度比 / 倾角差 / 同向 / 间距 / 长边水平 / 上下边水平 / 中点连线水平）
            bool pass[7];
            CheckPair(armor, L, R, cfg, pass);
            bool ok = true;
            for (int t = 0; t < 7; ++t) ok = ok && pass[t];
            if (!ok) continue;

            // 打分（越小越好）：倾角差越小、两根灯条越等长，越像同一块装甲
            const float ang_cost = armor.max_angle_diff_debug /
                                   std::max(1e-3f, static_cast<float>(cfg.pair_max_angle_diff));
            const float len_cost = 1.0f - armor.min_len_ratio_debug;

            ScoredPair sp;
            sp.i = i;
            sp.j = j;
            sp.score = ang_cost + len_cost;
            cand.push_back(sp);
        }
    }

    // ---- 2) 好配的先配：按分数排序后贪心占用 ----
    std::sort(cand.begin(), cand.end(),
              [](const ScoredPair& a, const ScoredPair& b) { return a.score < b.score; });

    vector<int> used(bars.size(), 0);
    vector<ArmorPair> pairs;
    for (size_t k = 0; k < cand.size(); ++k) {
        const ScoredPair& sp = cand[k];
        if (used[sp.i] >= max_use || used[sp.j] >= max_use) continue;   // 有一根已经用满

        ++used[sp.i];
        ++used[sp.j];

        ArmorPair pr;
        pr.i = sp.i;
        pr.j = sp.j;
        pairs.push_back(pr);
    }

    // ---- 3) 输出按左灯条下标排序，P0/P1 的编号才稳定 ----
    std::sort(pairs.begin(), pairs.end(),
              [](const ArmorPair& a, const ArmorPair& b) { return a.i < b.i; });
    return pairs;
}

// 宽松版配对：只做"间距"这一道粗筛，其余条件都不判。
// 它给出的候选对减去 Pairing() 的结果，就是"没成框"的那些，FindArmor 用白色画出来。
static vector<ArmorPair> Pairing_debug(const vector<LightBar>& bars,
                                       const detection_node::detectionConfig& cfg)
{
    const float max_gap_ratio = static_cast<float>(cfg.pair_max_gap_ratio);

    vector<ArmorPair> pairs_debug;
    for (int i = 0; i < static_cast<int>(bars.size()); ++i) {
        for (int j = i + 1; j < static_cast<int>(bars.size()); ++j) {
            const LightBar& L = bars[i];
            const LightBar& R = bars[j];

            const float gap = std::fabs(L.center.x - R.center.x);
            if (gap > max_gap_ratio * L.length && gap > max_gap_ratio * R.length) continue;

            ArmorPair pr;
            pr.i = i;
            pr.j = j;
            pairs_debug.push_back(pr);
        }
    }
    return pairs_debug;
}

static void FindArmor(const cv::Mat& img_mask, cv::Mat& img_debug, cv::Mat& img_armor,
                      const detection_node::detectionConfig& cfg){
    
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(img_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty()) return;
    
    std::vector<LightBar> bars;
    for (const auto& c : contours) {
        const cv::RotatedRect rect = cv::minAreaRect(c);

        cv::Point2f vertices[4];
        rect.points(vertices);

        cv::Point2f top_mid, bot_mid;
        TopBottomMid(vertices, top_mid, bot_mid);

        const cv::Point2f d = top_mid - bot_mid;
        const float length = std::sqrt(d.x * d.x + d.y * d.y);

        // 上/下端中点挨得太近 → 不是细长灯条，丢掉
        const float long_side = std::max(rect.size.width, rect.size.height);
        if (length < 0.95f * long_side) continue;

        // 相对竖直方向的夹角：d 由下指向上，右倾为正、左倾为负
        const float angle = static_cast<float>(std::atan2(d.x, -d.y) * 180.0 / CV_PI);
        // 可选：只想留接近竖直的灯条（顶部那根横条会算成 ±88° 左右）
        // if (std::fabs(angle) > 60.f) continue;

        LightBar bar;
        bar.top    = top_mid;
        bar.bottom = bot_mid;
        bar.center = (top_mid + bot_mid) * 0.5f;
        bar.angle  = angle;
        bar.length = length;
        bars.push_back(bar);
    }

    // 从左到右排一遍，配对时 i 恒在 j 左边
    std::sort(bars.begin(), bars.end(),
              [](const LightBar& a, const LightBar& b) { return a.center.x < b.center.x; });

    // 画出灯条和它的倾角
    for (const auto& b : bars) {
        cv::line(img_debug, b.top, b.bottom, cv::Scalar(0, 255, 0), 2);
        cv::circle(img_debug, b.top, 3, cv::Scalar(0, 0, 255), -1);
        cv::circle(img_debug, b.bottom, 3, cv::Scalar(0, 0, 255), -1);
        cv::putText(img_debug, cv::format("%.1f", b.angle),
                    cv::Point(cvRound(b.top.x) + 6, cvRound(b.top.y) - 6),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
    }

    const std::vector<ArmorPair> pairs = Pairing(bars, cfg);
    //Pairing_debug
    const std::vector<ArmorPair> pairs_debug = Pairing_debug(bars, cfg);

        for (size_t k = 0; k < pairs.size(); ++k) {
        const ArmorLight armor = MakeArmorLight(bars[pairs[k].i], bars[pairs[k].j]);
        const ArmorLight armor_debug = MakeArmorLight(bars[pairs[k].i], bars[pairs[k].j]);

        // 装甲板外框：0(左上) → 1(右上) → 2(右下) → 3(左下)
        for (int j = 0; j < 4; ++j) {
            cv::line(img_debug, armor.vertices[j], armor.vertices[(j + 1) % 4],
                     cv::Scalar(255, 0, 255), 2);
            cv::putText(img_debug, cv::format("%d", j + 1),
                        cv::Point(cvRound(armor.vertices[j].x) - 15,
                                  cvRound(armor.vertices[j].y) - 5),
                        cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
        }

        cv::line(img_debug, armor_debug.vertices[0], armor_debug.vertices[2], cv::Scalar(255, 255, 255), 1);
        cv::line(img_debug, armor_debug.vertices[1], armor_debug.vertices[3], cv::Scalar(255, 255, 255), 1);

        // 两条对角线（中心就是它的交点）
        cv::line(img_debug, armor.vertices[0], armor.vertices[2], cv::Scalar(128, 0, 128), 1);
        cv::line(img_debug, armor.vertices[1], armor.vertices[3], cv::Scalar(128, 0, 128), 1);

        // 两灯条中点连线：第 7 条判据量的就是这条线（两端是两根灯条的中心）
        cv::line(img_debug, bars[pairs[k].i].center, bars[pairs[k].j].center,
                 cv::Scalar(0, 255, 255), 1);
        cv::circle(img_debug, armor.mid, 2, cv::Scalar(0, 255, 255), -1);

        // 中心：红色实心点
        cv::circle(img_debug, armor.center, 4, cv::Scalar(0, 0, 255), -1);

        cv::putText(img_debug, cv::format("P%d %.0fx%.0f", static_cast<int>(k),
                                          armor.width, armor.height),
                    cv::Point(cvRound(armor.center.x) - 24, cvRound(armor.center.y) - 8),
                    cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(255, 0, 255), 1, cv::LINE_AA);

        // 调试：这一对的实际测量值（全都通过，所以 7 行都是白色）
        bool pass[7];
        CheckPair(armor, bars[pairs[k].i], bars[pairs[k].j], cfg, pass);
        DrawPairHud(img_debug, armor, bars[pairs[k].i], bars[pairs[k].j], pass,
                    cv::Point(cvRound(armor.center.x) + 20,
                              cvRound(armor.center.y) + 40));
    }

    img_armor = img_debug.clone();

    // "没成框"的：pairs_debug 里不在 pairs 中的那些，白色 1px
    int bad = 0;
    for (size_t k = 0; k < pairs_debug.size(); ++k) {
        bool accepted = false;
        for (size_t t = 0; t < pairs.size(); ++t) {
            if (pairs[t].i == pairs_debug[k].i && pairs[t].j == pairs_debug[k].j) {
                accepted = true;
                break;
            }
        }
        if (accepted) continue;

        const ArmorLight armor_debug = MakeArmorLight(bars[pairs_debug[k].i], bars[pairs_debug[k].j]);

        // 同一个 HUD，没通过的那几行标红
        bool pass[7];
        CheckPair(armor_debug, bars[pairs_debug[k].i], bars[pairs_debug[k].j], cfg, pass);

        // 7 条全过却没入选 = 灯条已被别的装甲占用，不算"没成框"，不画
        bool all_ok = true;
        for (int t = 0; t < 7; ++t) all_ok = all_ok && pass[t];
        if (all_ok) continue;

        for (int j = 0; j < 4; ++j)
            cv::line(img_debug, armor_debug.vertices[j], armor_debug.vertices[(j + 1) % 4],
                     cv::Scalar(255, 255, 255), 1);

        // 没成框的也画出中点连线，方便对照第 7 条判据
        cv::line(img_debug, bars[pairs_debug[k].i].center, bars[pairs_debug[k].j].center,
                 cv::Scalar(0, 255, 255), 1);
        cv::circle(img_debug, armor_debug.mid, 2, cv::Scalar(0, 255, 255), -1);

        DrawPairHud(img_debug, armor_debug,
                    bars[pairs_debug[k].i], bars[pairs_debug[k].j], pass,
                    cv::Point(cvRound(armor_debug.center.x) + 20,
                              cvRound(armor_debug.center.y) - 110));
        ++bad;
    }

    cv::putText(img_debug, cv::format("bars=%d pairs=%d bad=%d",
                                      static_cast<int>(bars.size()),
                                      static_cast<int>(pairs.size()),
                                      bad),
                cv::Point(10, 62), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
}

static void DrawPairHud(cv::Mat& img, const ArmorLight& a, const LightBar& L, const LightBar& R,
                        const bool pass[7], const cv::Point& org)
{
    const std::string txt[7] = {
        cv::format("1 len=%.2f",  a.min_len_ratio_debug),
        cv::format("2 ang=%.1f",  a.max_angle_diff_debug),
        cv::format("3 tilt=%s",   (std::fabs(L.angle - R.angle) < 1.0f)
                                   ? ((L.angle * R.angle < 0.0f) ? "opp" : "ok")
                                   : "n/a"),
        cv::format("4 gap=%.2f",  a.max_gap_ratio_debug),
        cv::format("5 long=%.1f", a.min_long_angle_debug),
        cv::format("6 edge=%.1f", a.max_edge_dy_debug),
        cv::format("7 mid=%.1f",  a.angle_mid)
    };
    for (int t = 0; t < 7; ++t)
        cv::putText(img, txt[t], org + cv::Point(0, t * 15),
                    cv::FONT_HERSHEY_SIMPLEX, 0.42,
                    pass[t] ? cv::Scalar(255, 255, 255) : cv::Scalar(0, 0, 255),
                    1, cv::LINE_AA);
}