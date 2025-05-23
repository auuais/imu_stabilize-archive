#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/logger.hpp>
#include <librealsense2/rs.hpp>
#include <iostream>
#include <cmath>
#include <mutex>
#include <thread>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// === Original Intrinsics ===
const int width = 1920;
const int height = 1080;
const float fx_orig = 1204.505857f;
const float fy_orig = 1204.505857f;
const float cx_orig = width / 2.0f;
const float cy_orig = height / 2.0f;
const float k1 = -0.343698f;
const float k2 = 0.105503f;
const float k3 = 0.0f;
const float p1 = -0.001558f;
const float p2 = -0.002416f;

float fx = fx_orig, fy = fy_orig, cx = cx_orig, cy = cy_orig;

// === Global IMU Variables ===
std::mutex orientation_mutex;
double roll = 0.0, pitch = 0.0, yaw = 0.0;

// === Helper Functions ===
double normalize_angle(double angle) {
    return fmod((angle + 180.0), 360.0) - 180.0;
}

template<typename T>
std::pair<double, double> compute_roll_pitch(T ax, T ay, T az) {
    if (az == 0.0) az = 1e-6;
    double roll = atan2(ay, az);
    double pitch = atan2(-ax, sqrt(ay * ay + az * az));
    return { roll * 180.0 / M_PI, pitch * 180.0 / M_PI };
}

void imu_orientation_updater(rs2::pipeline& pipeline) {
    const double alpha = 0.98;
    const double beta = 1.0;
    const double GYRO_DRIFT_X = -0.002843;
    const double GYRO_DRIFT_Y = 0.000743;
    const double GYRO_DRIFT_Z = -0.001002;

    double yaw_reset = 0.0;
    double prev_ts = 0.0;
    bool initialized = false;

    while (true) {
        try {
            rs2::frameset frames = pipeline.wait_for_frames(1000);
            rs2::motion_frame accel = frames.first_or_default(RS2_STREAM_ACCEL);
            rs2::motion_frame gyro = frames.first_or_default(RS2_STREAM_GYRO);

            if (!accel || !gyro) continue;

            auto a = accel.get_motion_data();
            auto g = gyro.get_motion_data();

            double ts = gyro.get_timestamp();
            if (!initialized) {
                prev_ts = ts;
                initialized = true;
                continue;
            }

            double dt = (ts - prev_ts) / 1000.0;
            prev_ts = ts;
            if (dt <= 0 || dt > 0.01) continue;

            auto acc_angles = compute_roll_pitch(-a.y, a.x, -a.z);
            double roll_acc = acc_angles.first;
            double pitch_acc = acc_angles.second;

            std::lock_guard<std::mutex> lock(orientation_mutex);
            double gx_deg = (g.x - GYRO_DRIFT_X) * 180.0 / M_PI;
            double gy_deg = (g.y - GYRO_DRIFT_Y) * 180.0 / M_PI;
            double gz_deg = (g.z - GYRO_DRIFT_Z) * 180.0 / M_PI;
            roll = normalize_angle(alpha * (roll + gx_deg * dt) + (1 - alpha) * roll_acc);
            pitch = normalize_angle(alpha * (pitch + gy_deg * dt) + (1 - alpha) * pitch_acc);
            yaw = normalize_angle(beta * (yaw + gz_deg * dt) + (1 - beta) * yaw_reset);
        }
        catch (...) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }
    }
}

cv::Vec3b interpolation(const cv::Mat& img, float x, float y) {
    int x0 = static_cast<int>(std::floor(x));
    int y0 = static_cast<int>(std::floor(y));
    float dx = x - x0;
    float dy = y - y0;

    if (x0 < 0 || x0 >= img.cols - 1 || y0 < 0 || y0 >= img.rows - 1)
        return cv::Vec3b(0, 0, 0);

    cv::Vec3b p00 = img.at<cv::Vec3b>(y0, x0);
    cv::Vec3b p10 = img.at<cv::Vec3b>(y0, x0 + 1);
    cv::Vec3b p01 = img.at<cv::Vec3b>(y0 + 1, x0);
    cv::Vec3b p11 = img.at<cv::Vec3b>(y0 + 1, x0 + 1);

    cv::Vec3b result;
    for (int c = 0; c < 3; ++c) {
        float val = (1 - dx) * (1 - dy) * p00[c]
            + dx * (1 - dy) * p10[c]
            + (1 - dx) * dy * p01[c]
            + dx * dy * p11[c];
        result[c] = cv::saturate_cast<uchar>(val);
    }
    return result;
}

cv::Mat manualUndistort(const cv::Mat& img) {
    cv::Mat output(height, width, img.type(), cv::Scalar(0, 0, 0));
    for (int v = 0; v < height; ++v) {
        for (int u = 0; u < width; ++u) {
            float x = (u - cx) / fx;
            float y = (v - cy) / fy;
            float r2 = x * x + y * y;
            float radial = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
            float dx = 2 * p1 * x * y + p2 * (r2 + 2 * x * x);
            float dy = p1 * (r2 + 2 * y * y) + 2 * p2 * x * y;

            float x_dist = x * radial + dx;
            float y_dist = y * radial + dy;

            float u_d = x_dist * fx_orig + cx_orig;
            float v_d = y_dist * fy_orig + cy_orig;

            output.at<cv::Vec3b>(v, u) = interpolation(img, u_d, v_d);
        }
    }
    return output;
}

cv::Mat applyManualAffine(const cv::Mat& img, float angle_deg, float translate_y) {
    int w = img.cols, h = img.rows;
    float angle_rad = angle_deg * M_PI / 180.0f;
    float cos_a = std::cos(angle_rad);
    float sin_a = std::sin(angle_rad);
    float cx_center = w / 2.0f;
    float cy_center = h / 2.0f;

    cv::Mat output(h, w, img.type(), cv::Scalar(0, 0, 0));

    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            float dx = x - cx_center;
            float dy = y - cy_center - translate_y;

            float src_x = cos_a * dx + sin_a * dy + cx_center;
            float src_y = -sin_a * dx + cos_a * dy + cy_center;

            output.at<cv::Vec3b>(y, x) = interpolation(img, src_x, src_y);
        }
    }
    return output;
}

void generateCylindricalRemap(cv::Mat& map_x, cv::Mat& map_y, int width, int height, float fov_deg = 60.0f, float angle_deg = 0.0f) {
    float fov_rad = fov_deg * M_PI / 180.0f;
    float angle_rad = angle_deg * M_PI / 180.0f;

    float fx_proj = width / (2.0f * std::tan(fov_rad / 2.0f));
    float fy_proj = fx_proj;
    float cx_proj = width / 2.0f;
    float cy_proj = height / 2.0f;

    map_x = cv::Mat(height, width, CV_32FC1);
    map_y = cv::Mat(height, width, CV_32FC1);

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            float theta = (x - cx_proj) / fx_proj + angle_rad;
            float h_ = (y - cy_proj) / fy_proj;

            float X = std::sin(theta);
            float Y = h_;
            float Z = std::cos(theta);

            float x_ = fx_proj * X / Z + cx_proj;
            float y_ = fy_proj * Y / Z + cy_proj;

            map_x.at<float>(y, x) = x_;
            map_y.at<float>(y, x) = y_;
        }
    }
}

cv::Mat cylindricalRemap(const cv::Mat& img, const cv::Mat& map_x, const cv::Mat& map_y) {
    cv::Mat output(img.rows, img.cols, img.type(), cv::Scalar(0, 0, 0));
    for (int y = 0; y < img.rows; ++y) {
        for (int x = 0; x < img.cols; ++x) {
            float src_x = map_x.at<float>(y, x);
            float src_y = map_y.at<float>(y, x);
            output.at<cv::Vec3b>(y, x) = interpolation(img, src_x, src_y);
        }
    }
    return output;
}

// === Convert RPY from Sys1 to SysX with Yaw/Pitch Offset
cv::Vec3f convert_rpy_sys1_to_sys(double roll_sys1, double pitch_sys1, double yaw_offset, double pitch_offset = 0.0, double roll_offset = 0.0) {
    // Convert degrees to radians
    double roll_rad = roll_sys1 * M_PI / 180.0;
    double pitch_rad = pitch_sys1 * M_PI / 180.0;
    double roll_offset_rad = roll_offset * M_PI / 180.0;
    double pitch_offset_rad = pitch_offset * M_PI / 180.0;
    double yaw_offset_rad = yaw_offset * M_PI / 180.0;

    // Construct R_sys1 using 'xyz' = Rx * Ry
    cv::Matx33d Rx = {
        1, 0, 0,
        0, cos(roll_rad), -sin(roll_rad),
        0, sin(roll_rad), cos(roll_rad)
    };
    cv::Matx33d Ry = {
        cos(pitch_rad), 0, sin(pitch_rad),
        0, 1, 0,
        -sin(pitch_rad), 0, cos(pitch_rad)
    };
    cv::Matx33d R_sys1 = Rx * Ry; // FIXED: match Python R.from_euler('xyz', ...)

    // Construct offset rotation: roll * pitch * yaw (roll offset first)
    cv::Matx33d R_roll_offset = {
        1, 0, 0,
        0, cos(roll_offset_rad), -sin(roll_offset_rad),
        0, sin(roll_offset_rad), cos(roll_offset_rad)
    };
    cv::Matx33d R_pitch_offset = {
        cos(pitch_offset_rad), 0, sin(pitch_offset_rad),
        0, 1, 0,
        -sin(pitch_offset_rad), 0, cos(pitch_offset_rad)
    };
    cv::Matx33d R_yaw_offset = {
        cos(yaw_offset_rad), -sin(yaw_offset_rad), 0,
        sin(yaw_offset_rad),  cos(yaw_offset_rad), 0,
        0, 0, 1
    };
    cv::Matx33d R_offset = R_yaw_offset * R_pitch_offset * R_roll_offset;

    // Apply inverse offset to transform
    cv::Matx33d R_sysX = R_sys1 * R_offset;

    // Convert back to Euler 'xyz'
    double pitchX = asin(-R_sysX(2, 0));
    double cy = cos(pitchX);
    double rollX, yawX;
    if (std::abs(cy) > 1e-6) {
        rollX = atan2(R_sysX(2, 1), R_sysX(2, 2));
        yawX = atan2(R_sysX(1, 0), R_sysX(0, 0));
    }
    else {
        rollX = 0;
        yawX = atan2(-R_sysX(0, 1), R_sysX(1, 1));
    }

    return cv::Vec3f(rollX * 180.0 / M_PI, pitchX * 180.0 / M_PI, yawX * 180.0 / M_PI);
}



// === Predict Translation from IMU Pitch/Yaw ===
int predict_y_translation(double pitch_deg) {
    return static_cast<int>(-17.44153917818162 * pitch_deg);
}

int predict_x_translation(double yaw_deg) {
    return static_cast<int>(17.84 * yaw_deg);
}



// ... [previous code remains unchanged] ...

cv::Mat centerCropOnly(const cv::Mat& frame, float crop_scale_h, float crop_scale_w) {
    int h = frame.rows;
    int w = frame.cols;
    int crop_h = static_cast<int>(h * crop_scale_h);
    int crop_w = static_cast<int>(w * crop_scale_w);
    int start_y = (h - crop_h) / 2;
    int start_x = (w - crop_w) / 2;
    return frame(cv::Rect(start_x, start_y, crop_w, crop_h)).clone();
}

int main() {
    cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_ERROR);

    // IMU setup
    rs2::pipeline pipeline;
    rs2::config config;
    config.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F, 250);
    config.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F, 400);
    bool imu_running = false;
    std::thread imu_thread;
    try {
        pipeline.start(config);
        imu_thread = std::thread(imu_orientation_updater, std::ref(pipeline));
        imu_running = true;
    }
    catch (const rs2::error& e) {
        std::cerr << "Failed to start RealSense pipeline: " << e.what() << std::endl;
    }

    // Adjusted camera matrix
    cv::Mat K = (cv::Mat_<double>(3, 3) << fx_orig, 0, cx_orig, 0, fy_orig, cy_orig, 0, 0, 1);
    cv::Mat dist = (cv::Mat_<double>(1, 5) << k1, k2, p1, p2, k3);
    cv::Mat newK = cv::getOptimalNewCameraMatrix(K, dist, cv::Size(width, height), 1);
    fx = static_cast<float>(newK.at<double>(0, 0));
    fy = static_cast<float>(newK.at<double>(1, 1));
    cx = static_cast<float>(newK.at<double>(0, 2));
    cy = static_cast<float>(newK.at<double>(1, 2));

    // Generate cylindrical remap maps
    std::vector<cv::Mat> map_x_list(2), map_y_list(2);
    generateCylindricalRemap(map_x_list[0], map_y_list[0], width, height);
    generateCylindricalRemap(map_x_list[1], map_y_list[1], width, height);

    // Open multiple cameras
    std::vector<cv::VideoCapture> cameras(2);
    cameras[0].open(3);
    cameras[1].open(0);
    for (auto& cam : cameras) {
        cam.set(cv::CAP_PROP_FRAME_WIDTH, width);
        cam.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    }

    cv::namedWindow("Panorama", cv::WINDOW_NORMAL);
    cv::resizeWindow("Panorama", width * 2, height); // Resize to expected panorama size
    cv::namedWindow("Original", cv::WINDOW_NORMAL);
    cv::resizeWindow("Original", width * 2, height);

    while (true) {
        std::vector<cv::Mat> frames(2);
        for (int i = 0; i < 2; ++i) {
            cameras[i] >> frames[i];
            if (frames[i].empty()) {
                std::cerr << "Camera " << i << " failed to capture." << std::endl;
                return -1;
            }
        }

        double roll_copy, pitch_copy, yaw_copy;
        {
            std::lock_guard<std::mutex> lock(orientation_mutex);
            roll_copy = roll;
            pitch_copy = pitch;
            yaw_copy = yaw;
        }

        std::vector<cv::Mat> stabilized_frames, original_frames;
        std::vector<std::string> annotations;

        for (int cam_idx = 0; cam_idx < 2; ++cam_idx) {
            auto rpy_corrected = convert_rpy_sys1_to_sys(roll_copy, pitch_copy, cam_idx * 60.0, cam_idx * 0.5);
            double roll_corr = rpy_corrected[0];
            double pitch_corr = rpy_corrected[1];
            int y_shift = predict_y_translation(pitch_corr);

            cv::Mat undistorted = manualUndistort(frames[cam_idx]);
            cv::Mat f_orig = centerCropOnly(undistorted.clone(), 0.54f, 0.615f);

            cv::Mat affine = applyManualAffine(undistorted, roll_corr, static_cast<float>(y_shift));
            cv::Mat projected = cylindricalRemap(affine, map_x_list[cam_idx], map_y_list[cam_idx]);
            cv::Mat cropped = centerCropOnly(projected, 0.5f, 0.615f);

            stabilized_frames.push_back(cropped);
            original_frames.push_back(f_orig);

            std::ostringstream oss;
            oss << "Cam" << cam_idx + 1 << " - R:" << roll_corr << " P:" << pitch_corr << " Y:" << yaw_copy;
            annotations.push_back(oss.str());
        }

        cv::Mat panorama, original;
        cv::hconcat(stabilized_frames, panorama);
        cv::hconcat(original_frames, original);

        for (size_t i = 0; i < annotations.size(); ++i) {
            cv::putText(panorama, annotations[i], cv::Point(30, 40 + static_cast<int>(i) * 40), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 0), 2);
        }

        cv::imshow("Panorama", panorama);
        cv::imshow("Original", original);
        if (cv::waitKey(1) == 'q') break;
    }

    for (auto& cam : cameras) cam.release();
    if (imu_thread.joinable()) imu_thread.detach();
    cv::destroyAllWindows();
    return 0;
}


