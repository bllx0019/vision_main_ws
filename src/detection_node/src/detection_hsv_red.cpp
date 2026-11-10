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

using namespace std;

// ===== rqt_reconfigure 动态参数 =====
// cfg/detection.cfg 里的 use_blue 就是红/蓝切换开关，
// Red / Blue / Filter 三组参数改完即时生效，不用重启节点
static std::mutex g_cfg_mutex;
static detection_node::detectionConfig g_cfg;

void cfgCallback(detection_node::detectionConfig &config, uint32_t /*level*/)
{
    std::lock_guard<std::mutex> lock(g_cfg_mutex);
    g_cfg = config;
}

// ===== 竖直段：匹配成“门”用 =====
struct VSeg {
    cv::Point2f top;      // 上端点（u 已统一朝下，所以 mid1 在上）
    cv::Point2f bot;      // 下端点
    float tilt_deg;       // 相对竖直的倾角
    float len;            // 长边长度
    float wid;            // 短边长度
    cv::Point2f quad[4];  // minAreaRect 的四个角，用来单独重画这根线
};

// 两条线段求交（要求交点落在两条线段内，凸四边形才成立）
static bool segIntersect(const cv::Point2f& p1, const cv::Point2f& p2,
                         const cv::Point2f& q1, const cv::Point2f& q2,
                         cv::Point2f& out)
{
    const cv::Point2f r = p2 - p1;
    const cv::Point2f s = q2 - q1;
    const float den = r.x * s.y - r.y * s.x;
    if (std::fabs(den) < 1e-6f) return false;   // 平行/重合
    const cv::Point2f qp = q1 - p1;
    const float t = (qp.x * s.y - qp.y * s.x) / den;
    const float u = (qp.x * r.y - qp.y * r.x) / den;
    if (t < 0.0f || t > 1.0f || u < 0.0f || u > 1.0f) return false;
    out = p1 + r * t;
    return true;
}

// 两个向量的夹角，度数，范围 0~180；任一向量退化时返回 -1
static float angleBetweenDeg(const cv::Point2f& a, const cv::Point2f& b)
{
    const float na = static_cast<float>(cv::norm(a));
    const float nb = static_cast<float>(cv::norm(b));
    if (na < 1e-3f || nb < 1e-3f) return -1.0f;
    float c = (a.x * b.x + a.y * b.y) / (na * nb);
    c = std::min(1.0f, std::max(-1.0f, c));
    return std::acos(c) * 57.29578f;
}

// 求灯条的最小外接矩形和四个角点；轮廓太小/退化返回 false
static bool findBarRect(const std::vector<cv::Point>& cnt,
                        cv::RotatedRect& r, cv::Point2f pts[4],
                        float min_side = 2.0f)
{
    if (cnt.size() < 3) return false;
    r = cv::minAreaRect(cnt);
    if (std::min(r.size.width, r.size.height) < min_side) return false;
    r.points(pts);
    return true;
}

// 画灯条外轮廓：把 minAreaRect 的四个角依次连起来
static void drawBarOutline(cv::Mat& img, const cv::Point2f pts[4],
                           const cv::Scalar& color = cv::Scalar(0, 0, 255),
                           int thickness = 2)
{
    for (int i = 0; i < 4; ++i)
        cv::line(img, pts[i], pts[(i + 1) % 4], color, thickness);
}

// 单独画一根候选线（框 + 两端点 + 中线 + 角度），配对成功时才用到
static void drawSeg(cv::Mat& img, const VSeg& s)
{
    drawBarOutline(img, s.quad);
    cv::circle(img, s.top, 4, cv::Scalar(0, 255, 0), -1);
    cv::circle(img, s.bot, 4, cv::Scalar(0, 255, 0), -1);
    cv::line(img, s.top, s.bot, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
    cv::putText(img, cv::format("%.1f", s.tilt_deg), s.bot + cv::Point2f(6, 12),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
}

// 画“门”：1=左上 2=左下 3=右下 4=右上（逆时针），对角线交点标 0
// max_corner_dev：横线(1-4 / 2-3)与白色竖线的夹角相对 90° 的最大偏差
// max_horiz_deg ：横线相对图像水平线的最大倾角（地平线水平判断）
static bool drawGate(cv::Mat& img, VSeg A, VSeg B, float max_corner_dev, float max_horiz_deg)
{
    // 左/右两条线的判定：用两段 x 的中点比较（都是近竖直线，够用）
    if (A.top.x + A.bot.x > B.top.x + B.bot.x) std::swap(A, B);
    const cv::Point2f p1 = A.top;   // 1 左上
    const cv::Point2f p2 = A.bot;   // 2 左下
    const cv::Point2f p3 = B.bot;   // 3 右下
    const cv::Point2f p4 = B.top;   // 4 右上

    // (a) 地平线水平判断：横线相对图像水平线不能太斜
    const float ang_top = std::atan2(std::fabs(p4.y - p1.y), std::fabs(p4.x - p1.x)) * 57.29578f;
    const float ang_bot = std::atan2(std::fabs(p3.y - p2.y), std::fabs(p3.x - p2.x)) * 57.29578f;
    if (ang_top > max_horiz_deg || ang_bot > max_horiz_deg) return false;

    // (b) 两条横线 1-4 / 2-3 与两条白色竖线 1-2 / 4-3 的夹角，都应当接近 90°
    const cv::Point2f e_top = p4 - p1;   // 上横线
    const cv::Point2f e_bot = p3 - p2;   // 下横线
    const cv::Point2f v_left  = p2 - p1; // 左侧白线（1->2）
    const cv::Point2f v_right = p3 - p4; // 右侧白线（4->3）
    float corner_dev = 0.0f;
    for (const cv::Point2f& e : {e_top, e_bot}) {
        for (const cv::Point2f& v : {v_left, v_right}) {
            corner_dev = std::max(corner_dev, std::fabs(angleBetweenDeg(e, v) - 90.0f));
        }
    }
    if (corner_dev > max_corner_dev) return false;

    cv::Point2f o;
    if (!segIntersect(p1, p3, p2, p4, o)) return false;   // 对角线不相交就不算门

    const cv::Scalar green(0, 255, 0), cyan(255, 255, 0), red(0, 0, 255), yellow(0, 255, 255);
    cv::line(img, p1, p2, green, 2, cv::LINE_AA);   // 1-2
    cv::line(img, p2, p3, green, 2, cv::LINE_AA);   // 2-3
    cv::line(img, p3, p4, green, 2, cv::LINE_AA);   // 3-4
    cv::line(img, p4, p1, green, 2, cv::LINE_AA);   // 4-1
    cv::line(img, p1, p3, cyan, 1, cv::LINE_AA);    // 对角线 1-3
    cv::line(img, p2, p4, cyan, 1, cv::LINE_AA);    // 对角线 2-4

    const cv::Point2f pts[4] = {p1, p2, p3, p4};
    const char* names[4] = {"1", "2", "3", "4"};
    for (int i = 0; i < 4; ++i) {
        cv::circle(img, pts[i], 3, yellow, -1, cv::LINE_AA);
        cv::putText(img, names[i], pts[i] + cv::Point2f(5, -5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, yellow, 1, cv::LINE_AA);
    }

    cv::circle(img, o, 5, red, -1, cv::LINE_AA);
    cv::putText(img, "0", o + cv::Point2f(7, -7),
                cv::FONT_HERSHEY_SIMPLEX, 0.55, red, 1, cv::LINE_AA);
    return true;
}

cv::Mat img;
cv::Mat img_hsv;
cv::Mat img_hsv_mask;
cv_bridge::CvImage out_msg_mask;
cv_bridge::CvImage out_msg_debug;
cv_bridge::CvImage out_msg_debug_with_arm;   // 画好框和门的调试图

ros::Publisher pub_img;
ros::Publisher pub_img_debug;
ros::Publisher pub_img_debug_arm;

void imageCallback(const sensor_msgs::CompressedImage::ConstPtr& img_msg){
    
    if (img_msg->data.empty()) return;
    // 每帧取一份参数快照，避免调参时半帧切换
    detection_node::detectionConfig cfg;{
        std::lock_guard<std::mutex> lock(g_cfg_mutex);
        cfg = g_cfg;
    }
    cv_bridge::CvImagePtr cv_ptr =cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
    img = cv_ptr->image;
    
    cv::cvtColor(img, img_hsv, cv::COLOR_BGR2HSV);
    cv::GaussianBlur(img_hsv, img_hsv, cv::Size(5, 5), 0);
    // 红色/蓝色两套 HSV 范围，由 rqt_reconfigure 里的 use_blue 开关决定用哪套
    const cv::Scalar hsv_min = cfg.use_blue
        ? cv::Scalar(cfg.blue_h_min, cfg.blue_s_min, cfg.blue_v_min)
        : cv::Scalar(cfg.red_h_min,  cfg.red_s_min,  cfg.red_v_min);
    const cv::Scalar hsv_max = cfg.use_blue
        ? cv::Scalar(cfg.blue_h_max, cfg.blue_s_max, cfg.blue_v_max)
        : cv::Scalar(cfg.red_h_max,  cfg.red_s_max,  cfg.red_v_max);
    cv::inRange(img_hsv, hsv_min, hsv_max, img_hsv_mask);

    //cv::inRange(img_hsv, cv::Scalar(170, 100, 100), cv::Scalar(180, 255, 255), mask2);
    //灯光偏橙，不考虑合并
    //cv::bitwise_or(mask1, mask2, img_hsv_mask);     
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3), cv::Point(-1, -1));
    cv::Mat kernel2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5), cv::Point(-1, -1));
    //cv::erode(img_hsv_mask, img_hsv_mask, kernel, cv::Point(-1, -1),2);
    cv::morphologyEx(img_hsv_mask, img_hsv_mask, cv::MORPH_OPEN, kernel,cv::Point(-1, -1));
    cv::morphologyEx(img_hsv_mask, img_hsv_mask, cv::MORPH_CLOSE, kernel2,cv::Point(-1, -1),2);
    //cv::dilate(img_hsv_mask, img_hsv_mask, kernel);
    //cv::erode(img_hsv_mask, img_hsv_mask, kernel2, cv::Point(-1, -1),2);

    vector<vector<cv::Point>> contours;
    cv::findContours(img_hsv_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    cv::Mat debug_img = img.clone();
    cv::Mat debug_img_mask = img_hsv_mask.clone();
    std::vector<VSeg> vsegs;   // 收集所有竖直段，循环结束后两两匹配
    for (const auto& cnt : contours) {
        
        // CHAIN_APPROX_SIMPLE 会把轴对齐矩形压成 4 个点，阈值不能写 5
        if (cnt.size() < 3) continue;
        double area = cv::contourArea(cnt);
        if (area < cfg.min_area || area > 0.5 * img_hsv_mask.total()) continue;
        
        cv::RotatedRect r;
        cv::Point2f pts[4];
        if (!findBarRect(cnt, r, pts)) continue;   // 外接矩形 + 四个角点

        const float len01 = static_cast<float>(cv::norm(pts[1] - pts[0]));  // = size.height
        const float len12 = static_cast<float>(cv::norm(pts[2] - pts[1]));  // = size.width
        // 长轴方向 = 较长的那条边，统一朝下，消除 180° 翻转
        cv::Point2f u = (len01 >= len12) ? (pts[1] - pts[0]) : (pts[2] - pts[1]);
        const float ulen = static_cast<float>(cv::norm(u));
        if (ulen < 1e-3f) continue;
        u /= ulen;
        if (u.y < 0.0f || (std::fabs(u.y) < 1e-6f && u.x < 0.0f)) u = -u;

        // u 是单位向量 => u.y = cos(倾角)。直接用分量判“竖不竖”，不用 atan2
        if (std::fabs(u.y) < cfg.min_cos_vert) continue;   // 长轴太靠水平，当横向线段丢掉

        const float len    = std::max(len01, len12);                 // 长边(px)
        const float wid    = std::min(len01, len12);                 // 短边(px)
        const float aspect = (wid > 1e-3f) ? len / wid : 0.0f;

        drawBarOutline(debug_img, pts);   // 灯条外轮廓

        // 长轴两端 = 两条短边的中点
        cv::Point2f mid1 = r.center - u * (len * 0.5f);
        cv::Point2f mid2 = r.center + u * (len * 0.5f);
        // 只有显示才把 cos 反解回角度：±acos(u.y)，和 atan2 等价
        const float deg = std::acos(std::min(1.0f, std::max(0.0f, u.y))) * 57.29578f
                          * (u.x < 0.0f ? -1.0f : 1.0f);

        cv::circle(debug_img, mid1, 4, cv::Scalar(0, 255, 0), -1);
        cv::circle(debug_img, mid2, 4, cv::Scalar(0, 255, 0), -1);
        cv::line(debug_img, mid1, mid2, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

        if (aspect >= 1.05f && len >= 8.0f) {                        // 太方/太短
            // 角度写在线的下端，避免压住门的角点编号 1/4
            cv::putText(debug_img, cv::format("%.1f", deg), mid2 + cv::Point2f(6, 12),cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
        }

        // u 朝下 => mid1 在上、mid2 在下
        vsegs.push_back({mid1, mid2, deg, len, wid, {pts[0], pts[1], pts[2], pts[3]}});
    }

    // image_hsv_debug 显示所有候选框；image_debug_with_arm 从干净原图重画，
    // 只有配对成门的那两根线才会出现
    cv::Mat debug_img_arm = img.clone();

    // ===== 两两匹配：长度接近 + 基本平行 + 间距不超过 2 倍平均长度 =====
    struct Cand { int i, j; float cost, gap; };
    std::vector<Cand> cands;
    for (int i = 0; i < static_cast<int>(vsegs.size()); ++i) {
        for (int j = i + 1; j < static_cast<int>(vsegs.size()); ++j) {
            const VSeg& A = vsegs[i];
            const VSeg& B = vsegs[j];
            const float lenAvg = 0.5f * (A.len + B.len);
            if (lenAvg < 1e-3f) continue;

            // 1) 长度接近
            const float dLen = std::fabs(A.len - B.len);
            if (dLen > cfg.match_len_tol * lenAvg) continue;

            // 2) 平行：倾角差在容差内
            const float dAng = std::fabs(A.tilt_deg - B.tilt_deg);
            if (dAng > cfg.match_angle_tol) continue;

            // 2.1) 倾角异号（一左一右）直接禁止配对；正好 0° 算中性，不参与符号判断
            if (A.tilt_deg * B.tilt_deg < 0.0f) continue;

            // 3) 间距：两条线中点沿 B 法向的距离，不得超过X倍平均长度
            cv::Point2f dirB = B.bot - B.top;
            const float nB = static_cast<float>(cv::norm(dirB));
            if (nB < 1e-3f) continue;
            dirB /= nB;
            const cv::Point2f nrmB(-dirB.y, dirB.x);
            const cv::Point2f dMid = 0.5f * ((A.top + A.bot) - (B.top + B.bot));
            const float gap = std::fabs(dMid.dot(nrmB));
            if (gap > cfg.match_max_gap * lenAvg) continue;
            if (gap < 3.0f) continue;   // 太近：同一根线或上下堆叠

            const float cost = dLen / lenAvg + dAng / (cfg.match_angle_tol + 1e-3f);
            cands.push_back({i, j, cost, gap});
        }
    }

    // 贪心配对：代价小的先配，每个线段只用一次
    std::sort(cands.begin(), cands.end(),
              [](const Cand& a, const Cand& b) { return a.cost < b.cost; });
    std::vector<char> used(vsegs.size(), 0);
    for (const Cand& c : cands) {
        if (used[c.i] || used[c.j]) continue;
        if (!drawGate(debug_img_arm, vsegs[c.i], vsegs[c.j],
                      static_cast<float>(cfg.gate_corner_tol),
                      static_cast<float>(cfg.gate_horiz_tol))) continue;
        drawSeg(debug_img_arm, vsegs[c.i]);
        drawSeg(debug_img_arm, vsegs[c.j]);
        used[c.i] = 1;
        used[c.j] = 1;
    }

    out_msg_mask.header = img_msg->header;
    out_msg_mask.encoding = sensor_msgs::image_encodings::MONO8;
    out_msg_mask.image = img_hsv_mask;
    pub_img.publish(out_msg_mask.toImageMsg());
    out_msg_debug.header = img_msg->header;
    out_msg_debug.encoding = sensor_msgs::image_encodings::BGR8;
    out_msg_debug.image = debug_img;
    pub_img_debug.publish(out_msg_debug.toImageMsg());
    // 画好框和门的调试图
    out_msg_debug_with_arm.header = img_msg->header;
    out_msg_debug_with_arm.encoding = sensor_msgs::image_encodings::BGR8;
    out_msg_debug_with_arm.image = debug_img_arm;
    pub_img_debug_arm.publish(out_msg_debug_with_arm.toImageMsg());
}


int main(int argc, char** argv){
    ros::init(argc, argv,"delection_hsv");
    ros::NodeHandle nh;
    pub_img = nh.advertise<sensor_msgs::Image>("/hk_camera/image_hsv_mask",1);
    pub_img_debug = nh.advertise<sensor_msgs::Image>("/hk_camera/image_hsv_debug",1);
    pub_img_debug_arm = nh.advertise<sensor_msgs::Image>("/hk_camera/image_debug_with_arm",1);
    ros::Subscriber sub_img=nh.subscribe("/hk_camera/image_raw/compressed",1,imageCallback);
    //ros::Subscriber sub_camera_info=nh.subscribe("/hk_camera/camera_info",1,imageCallback);

    // rqt_reconfigure 服务：rosrun rqt_reconfigure rqt_reconfigure
    dynamic_reconfigure::Server<detection_node::detectionConfig> server;
    server.setCallback([](detection_node::detectionConfig &config, uint32_t level) {
        cfgCallback(config, level);
    });

    ros::spin();
    return 0;
}


    