#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <errno.h>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <optional>
#include <regex>
#include <sstream>
#include <string>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <vector>

#if defined(LVB_HAVE_OPENCV)
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#endif

namespace {

constexpr uint16_t kGuidanceHeader = 0xAA55;
constexpr uint16_t kStatusHeader = 0x55AA;
constexpr uint8_t kGuidanceType = 0x01;
constexpr uint8_t kStatusType = 0x81;

enum class RuntimeMode { Disabled, AprilTag };
enum class MissionState { Search = 0, Track = 1, Approach = 2, Hold = 3 };
enum class LogLevel { Error = 0, Info = 1, Debug = 2 };

struct GuidanceFrame {
    uint32_t timestamp_ms{};
    uint8_t target_type{};
    float dx_norm{};
    float dy_norm{};
    float z_est_m{};
    float yaw_err_deg{};
    float confidence{};
    bool target_valid{};
};

struct StatusFrame {
    uint8_t mode{};
    uint8_t ack_code{};
};

struct TargetSample {
    int64_t timestamp_ms{};
    float dx_norm{};
    float dy_norm{};
    float z_est_m{};
    float yaw_err_deg{};
    float confidence{};
};

struct Config {
    RuntimeMode mode{RuntimeMode::AprilTag};
    bool debug_enabled{false};
    int debug_status_every_n{30};
    int cam_device{0};
    int width{640};
    int height{480};
    int fps{30};
    float fx{520.f};
    float fy{520.f};
    float cx{320.f};
    float cy{240.f};
    float tag_size_m{0.14f};
    float min_confidence{0.65f};
    int lost_timeout_ms{300};
    std::string uart_port{"/dev/ttyS1"};
    int uart_baud{921600};
    int send_rate_hz{30};
    int status_timeout_ms{500};
    int target_ttl_ms{300};
    float approach_z_m{1.5f};
    float hold_z_m{0.8f};
    float max_abs_offset_for_hold{0.12f};
    std::string csv_path{"/tmp/vision_bridge_log.csv"};
};

LogLevel g_log_level = LogLevel::Info;

void log_msg(LogLevel level, const std::string& msg) {
    if (static_cast<int>(level) > static_cast<int>(g_log_level)) {
        return;
    }
    const char* tag = (level == LogLevel::Error) ? "ERROR" : (level == LogLevel::Info) ? "INFO" : "DEBUG";
    std::cerr << "[" << tag << "] " << msg << "\n";
}

int64_t now_ms() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

RuntimeMode parse_mode(const std::string& mode) {
    if (mode == "apriltag") return RuntimeMode::AprilTag;
    return RuntimeMode::Disabled;
}

bool regex_str(const std::string& s, const std::string& p, std::string* out) {
    std::smatch m;
    if (!std::regex_search(s, m, std::regex(p, std::regex::ECMAScript))) return false;
    *out = m[1].str();
    return true;
}

bool regex_num(const std::string& s, const std::string& p, double* out) {
    std::string raw;
    if (!regex_str(s, p, &raw)) return false;
    *out = std::stod(raw);
    return true;
}

bool regex_bool(const std::string& s, const std::string& p, bool* out) {
    std::string raw;
    if (!regex_str(s, p, &raw)) return false;
    *out = (raw == "true");
    return true;
}

bool load_config(const std::string& path, Config* cfg) {
    std::ifstream in(path);
    if (!in.is_open()) return false;
    std::stringstream ss;
    ss << in.rdbuf();
    const std::string raw = ss.str();

    std::string mode;
    if (regex_str(raw, R"re("runtime"\s*:\s*\{[\s\S]*?"mode"\s*:\s*"([^"]+)")re", &mode)) cfg->mode = parse_mode(mode);

    bool dbg = false;
    if (regex_bool(raw, R"re("debug"\s*:\s*\{[\s\S]*?"enabled"\s*:\s*(true|false))re", &dbg)) cfg->debug_enabled = dbg;
    double n = 0;
    if (regex_num(raw, R"re("debug"\s*:\s*\{[\s\S]*?"status_every_n"\s*:\s*([-0-9.]+))re", &n)) cfg->debug_status_every_n = static_cast<int>(n);

    if (regex_num(raw, R"re("device"\s*:\s*([-0-9.]+))re", &n)) cfg->cam_device = static_cast<int>(n);
    if (regex_num(raw, R"re("width"\s*:\s*([-0-9.]+))re", &n)) cfg->width = static_cast<int>(n);
    if (regex_num(raw, R"re("height"\s*:\s*([-0-9.]+))re", &n)) cfg->height = static_cast<int>(n);
    if (regex_num(raw, R"re("fps"\s*:\s*([-0-9.]+))re", &n)) cfg->fps = static_cast<int>(n);
    if (regex_num(raw, R"re("fx"\s*:\s*([-0-9.]+))re", &n)) cfg->fx = static_cast<float>(n);
    if (regex_num(raw, R"re("fy"\s*:\s*([-0-9.]+))re", &n)) cfg->fy = static_cast<float>(n);
    if (regex_num(raw, R"re("cx"\s*:\s*([-0-9.]+))re", &n)) cfg->cx = static_cast<float>(n);
    if (regex_num(raw, R"re("cy"\s*:\s*([-0-9.]+))re", &n)) cfg->cy = static_cast<float>(n);
    if (regex_num(raw, R"re("tag_size_m"\s*:\s*([-0-9.]+))re", &n)) cfg->tag_size_m = static_cast<float>(n);
    if (regex_num(raw, R"re("apriltag"\s*:\s*\{[\s\S]*?"min_confidence"\s*:\s*([-0-9.]+))re", &n)) cfg->min_confidence = static_cast<float>(n);
    if (regex_num(raw, R"re("apriltag"\s*:\s*\{[\s\S]*?"lost_timeout_ms"\s*:\s*([-0-9.]+))re", &n)) cfg->lost_timeout_ms = static_cast<int>(n);
    if (regex_num(raw, R"re("baud"\s*:\s*([-0-9.]+))re", &n)) cfg->uart_baud = static_cast<int>(n);
    if (regex_num(raw, R"re("send_rate_hz"\s*:\s*([-0-9.]+))re", &n)) cfg->send_rate_hz = static_cast<int>(n);
    if (regex_num(raw, R"re("status_timeout_ms"\s*:\s*([-0-9.]+))re", &n)) cfg->status_timeout_ms = static_cast<int>(n);
    if (regex_num(raw, R"re("target_ttl_ms"\s*:\s*([-0-9.]+))re", &n)) cfg->target_ttl_ms = static_cast<int>(n);
    if (regex_num(raw, R"re("approach_z_m"\s*:\s*([-0-9.]+))re", &n)) cfg->approach_z_m = static_cast<float>(n);
    if (regex_num(raw, R"re("hold_z_m"\s*:\s*([-0-9.]+))re", &n)) cfg->hold_z_m = static_cast<float>(n);
    if (regex_num(raw, R"re("max_abs_offset_for_hold"\s*:\s*([-0-9.]+))re", &n)) cfg->max_abs_offset_for_hold = static_cast<float>(n);

    std::string s;
    if (regex_str(raw, R"re("port"\s*:\s*"([^"]+)")re", &s)) cfg->uart_port = s;
    if (regex_str(raw, R"re("csv_path"\s*:\s*"([^"]+)")re", &s)) cfg->csv_path = s;
    return true;
}

uint16_t crc16_ccitt_false(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021) : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

std::vector<uint8_t> pack_guidance(uint16_t seq, const GuidanceFrame& frame, bool emergency_hover) {
    uint8_t flags = 0;
    if (frame.target_valid) flags |= 0x01;
    if (emergency_hover) flags |= 0x02;

    std::vector<uint8_t> out(33, 0);
    size_t p = 0;
    auto put_u8 = [&](uint8_t v) { out[p++] = v; };
    auto put_u16 = [&](uint16_t v) { out[p++] = static_cast<uint8_t>(v); out[p++] = static_cast<uint8_t>(v >> 8); };
    auto put_u32 = [&](uint32_t v) {
        out[p++] = static_cast<uint8_t>(v); out[p++] = static_cast<uint8_t>(v >> 8);
        out[p++] = static_cast<uint8_t>(v >> 16); out[p++] = static_cast<uint8_t>(v >> 24);
    };
    auto put_f32 = [&](float v) { uint32_t u = 0; std::memcpy(&u, &v, sizeof(float)); put_u32(u); };

    put_u16(kGuidanceHeader);
    put_u8(kGuidanceType);
    put_u16(seq);
    put_u32(frame.timestamp_ms);
    put_u8(frame.target_type);
    put_f32(frame.dx_norm);
    put_f32(frame.dy_norm);
    put_f32(frame.z_est_m);
    put_f32(frame.yaw_err_deg);
    put_f32(frame.confidence);
    put_u8(flags);
    put_u16(crc16_ccitt_false(out.data(), out.size() - 2));
    return out;
}

bool parse_status(const uint8_t* data, size_t len, StatusFrame* out) {
    if (len != 14) return false;
    const uint16_t header = static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8);
    if (header != kStatusHeader || data[2] != kStatusType) return false;
    const uint16_t crc = static_cast<uint16_t>(data[12]) | (static_cast<uint16_t>(data[13]) << 8);
    if (crc != crc16_ccitt_false(data, 12)) return false;
    out->mode = data[5];
    out->ack_code = data[11];
    return true;
}

speed_t map_baud(int baud) {
    switch (baud) {
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        default: return B921600;
    }
}

class Uart {
public:
    bool open_port(const std::string& port, int baud, std::string* err) {
        close_port();
        fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) {
            *err = "open(" + port + ") failed: errno=" + std::to_string(errno) + " " + std::strerror(errno);
            return false;
        }

        termios tty{};
        if (tcgetattr(fd_, &tty) != 0) {
            *err = "tcgetattr failed: errno=" + std::to_string(errno) + " " + std::strerror(errno);
            return false;
        }
        cfsetospeed(&tty, map_baud(baud));
        cfsetispeed(&tty, map_baud(baud));
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_iflag = 0;
        tty.c_oflag = 0;
        tty.c_lflag = 0;
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 0;
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
        if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
            *err = "tcsetattr failed: errno=" + std::to_string(errno) + " " + std::strerror(errno);
            return false;
        }
        return true;
    }

    void close_port() { if (fd_ >= 0) { ::close(fd_); fd_ = -1; } }
    ~Uart() { close_port(); }

    bool write_all(const std::vector<uint8_t>& data, std::string* err) {
        size_t sent = 0;
        while (sent < data.size()) {
            const ssize_t n = ::write(fd_, data.data() + sent, data.size() - sent);
            if (n <= 0) {
                *err = "uart write failed: errno=" + std::to_string(errno) + " " + std::strerror(errno);
                return false;
            }
            sent += static_cast<size_t>(n);
        }
        return true;
    }

    void read_bytes(std::vector<uint8_t>* out) {
        uint8_t tmp[128];
        for (;;) {
            const ssize_t n = ::read(fd_, tmp, sizeof(tmp));
            if (n <= 0) break;
            out->insert(out->end(), tmp, tmp + n);
        }
    }

private:
    int fd_{-1};
};

MissionState update_mission(const Config& cfg, const std::optional<TargetSample>& target, MissionState prev, bool* emergency_hover) {
    *emergency_hover = false;
    if (!target.has_value() || (now_ms() - target->timestamp_ms) > cfg.target_ttl_ms) {
        *emergency_hover = true;
        return MissionState::Search;
    }
    if (prev == MissionState::Search) prev = MissionState::Track;

    if (target->z_est_m <= cfg.hold_z_m &&
        std::abs(target->dx_norm) <= cfg.max_abs_offset_for_hold &&
        std::abs(target->dy_norm) <= cfg.max_abs_offset_for_hold) return MissionState::Hold;
    if (target->z_est_m <= cfg.approach_z_m) return MissionState::Approach;
    return MissionState::Track;
}

#if defined(LVB_HAVE_OPENCV)
std::optional<TargetSample> detect_apriltag(const Config& cfg, cv::VideoCapture* cap, int64_t* last_seen_ms) {
    cv::Mat frame;
    if (!cap->read(frame) || frame.empty()) return std::nullopt;

    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    auto dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(gray, dict, corners, ids);
    if (ids.empty()) return std::nullopt;

    size_t best = 0;
    float best_area = 0.f;
    for (size_t i = 0; i < corners.size(); ++i) {
        const float area = std::abs(static_cast<float>(cv::contourArea(corners[i])));
        if (area > best_area) { best_area = area; best = i; }
    }

    std::vector<std::vector<cv::Point2f>> chosen{corners[best]};
    cv::Mat K = (cv::Mat_<double>(3, 3) << cfg.fx, 0.0, cfg.cx, 0.0, cfg.fy, cfg.cy, 0.0, 0.0, 1.0);
    cv::Mat D = cv::Mat::zeros(1, 5, CV_64F);
    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::aruco::estimatePoseSingleMarkers(chosen, cfg.tag_size_m, K, D, rvecs, tvecs);

    cv::Point2f center(0.f, 0.f);
    for (const auto& p : chosen[0]) center += p;
    center *= 0.25f;
    const float dx = (center.x - cfg.width * 0.5f) / (cfg.width * 0.5f);
    const float dy = (center.y - cfg.height * 0.5f) / (cfg.height * 0.5f);

    float z = 9.99f;
    if (!tvecs.empty()) z = static_cast<float>(tvecs[0][2]);
    float yaw = 0.0f;
    if (!rvecs.empty()) {
        cv::Mat R;
        cv::Rodrigues(rvecs[0], R);
        yaw = static_cast<float>(std::atan2(R.at<double>(1, 0), R.at<double>(0, 0)) * 180.0 / CV_PI);
    }

    const float confidence = std::min(1.0f, best_area / static_cast<float>(cfg.width * cfg.height * 0.10f));
    if (confidence < cfg.min_confidence) return std::nullopt;
    *last_seen_ms = now_ms();
    return TargetSample{now_ms(), dx, dy, z, yaw, confidence};
}
#endif

void usage() {
    std::cout << "Usage: luckfox_bridge --config <path> [--mode apriltag|disabled] [--debug] [--quiet] [--no-uart]\n";
}

}  // namespace

int main(int argc, char** argv) {
    std::string config_path = "luckfox/config/default.json";
    std::optional<RuntimeMode> mode_override;
    bool no_uart = false;
    bool force_debug = false;
    bool force_quiet = false;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--config" && i + 1 < argc) config_path = argv[++i];
        else if (arg == "--mode" && i + 1 < argc) mode_override = parse_mode(argv[++i]);
        else if (arg == "--no-uart") no_uart = true;
        else if (arg == "--debug") force_debug = true;
        else if (arg == "--quiet") force_quiet = true;
        else if (arg == "--help") { usage(); return 0; }
    }

    Config cfg;
    if (!load_config(config_path, &cfg)) {
        log_msg(LogLevel::Error, "Failed to load config: " + config_path);
        return 1;
    }
    if (mode_override.has_value()) cfg.mode = mode_override.value();
    if (force_debug) cfg.debug_enabled = true;
    if (force_quiet) cfg.debug_enabled = false;
    g_log_level = cfg.debug_enabled ? LogLevel::Debug : LogLevel::Info;

    log_msg(LogLevel::Info, "Starting luckfox_bridge mode=" + std::string(cfg.mode == RuntimeMode::AprilTag ? "apriltag" : "disabled"));

#if !defined(LVB_HAVE_OPENCV)
    if (cfg.mode == RuntimeMode::AprilTag) {
        log_msg(LogLevel::Error, "Binary built without OpenCV. AprilTag mode is unavailable. Rebuild with OpenCV or use mode=disabled.");
        return 2;
    }
#endif

#if defined(LVB_HAVE_OPENCV)
    cv::VideoCapture cap(cfg.cam_device);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, cfg.width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, cfg.height);
    cap.set(cv::CAP_PROP_FPS, cfg.fps);
    if (cfg.mode == RuntimeMode::AprilTag && !cap.isOpened()) {
        log_msg(LogLevel::Error, "Failed to open camera device " + std::to_string(cfg.cam_device));
        return 3;
    }
#endif

    Uart uart;
    if (!no_uart) {
        std::string err;
        if (!uart.open_port(cfg.uart_port, cfg.uart_baud, &err)) {
            log_msg(LogLevel::Error, err);
            return 4;
        }
    }

    std::ofstream log(cfg.csv_path, std::ios::app);
    if (!log.is_open()) {
        log_msg(LogLevel::Error, "Failed to open csv log: " + cfg.csv_path);
        return 5;
    }
    log << "timestamp_ms,seq,state,target,confidence,dx,dy,z,status_mode,status_ack,error\n";

    MissionState state = MissionState::Search;
    uint16_t seq = 0;
#if defined(LVB_HAVE_OPENCV)
    int64_t last_seen_ms = 0;
#endif
    int64_t last_status_ms = now_ms();
    std::vector<uint8_t> rx;
    const int period_ms = std::max(1, 1000 / std::max(1, cfg.send_rate_hz));
    int frame_idx = 0;

    while (true) {
        const int64_t t0 = now_ms();
        std::optional<TargetSample> target = std::nullopt;
        std::string frame_error;

        if (cfg.mode == RuntimeMode::AprilTag) {
#if defined(LVB_HAVE_OPENCV)
            target = detect_apriltag(cfg, &cap, &last_seen_ms);
#else
            frame_error = "apriltag_unavailable_no_opencv";
#endif
        }

        bool emergency_hover = false;
        state = update_mission(cfg, target, state, &emergency_hover);

        GuidanceFrame frame{};
        frame.timestamp_ms = static_cast<uint32_t>(now_ms() & 0xFFFFFFFF);
        frame.target_type = target.has_value() ? 1 : 0;
        frame.dx_norm = target.has_value() ? target->dx_norm : 0.0f;
        frame.dy_norm = target.has_value() ? target->dy_norm : 0.0f;
        frame.z_est_m = target.has_value() ? target->z_est_m : 99.0f;
        frame.yaw_err_deg = target.has_value() ? target->yaw_err_deg : 0.0f;
        frame.confidence = target.has_value() ? target->confidence : 0.0f;
        frame.target_valid = target.has_value();

        auto pkt = pack_guidance(seq, frame, emergency_hover);
        int status_mode = -1;
        int status_ack = -1;

        if (!no_uart) {
            std::string err;
            if (!uart.write_all(pkt, &err)) {
                frame_error = err;
                log_msg(LogLevel::Error, err);
            } else {
                uart.read_bytes(&rx);
                while (rx.size() >= 14) {
                    if (rx[0] != 0xAA || rx[1] != 0x55) { rx.erase(rx.begin()); continue; }
                    StatusFrame st{};
                    if (!parse_status(rx.data(), 14, &st)) { rx.erase(rx.begin()); continue; }
                    rx.erase(rx.begin(), rx.begin() + 14);
                    last_status_ms = now_ms();
                    status_mode = static_cast<int>(st.mode);
                    status_ack = static_cast<int>(st.ack_code);
                }
                if ((now_ms() - last_status_ms) > cfg.status_timeout_ms) {
                    frame_error = "status_timeout_reopen_uart";
                    log_msg(LogLevel::Error, frame_error);
                    uart.close_port();
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    if (!uart.open_port(cfg.uart_port, cfg.uart_baud, &err)) {
                        log_msg(LogLevel::Error, err);
                    }
                    last_status_ms = now_ms();
                }
            }
        }

        log << now_ms() << "," << seq << "," << static_cast<int>(state) << ","
            << static_cast<int>(frame.target_type) << "," << frame.confidence << ","
            << frame.dx_norm << "," << frame.dy_norm << "," << frame.z_est_m << ","
            << status_mode << "," << status_ack << "," << (frame_error.empty() ? "none" : frame_error) << "\n";

        if (cfg.debug_enabled && (++frame_idx % std::max(1, cfg.debug_status_every_n) == 0)) {
            std::ostringstream oss;
            oss << "seq=" << seq << " state=" << static_cast<int>(state)
                << " target=" << static_cast<int>(frame.target_type)
                << " conf=" << frame.confidence
                << " dx=" << frame.dx_norm
                << " dy=" << frame.dy_norm
                << " z=" << frame.z_est_m
                << " status_ack=" << status_ack;
            log_msg(LogLevel::Debug, oss.str());
        }

        seq = static_cast<uint16_t>(seq + 1);
        const int64_t elapsed = now_ms() - t0;
        if (elapsed < period_ms) std::this_thread::sleep_for(std::chrono::milliseconds(period_ms - elapsed));
    }
}
