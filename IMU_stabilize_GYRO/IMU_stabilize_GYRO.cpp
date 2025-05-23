// SVN_block

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

// === Camera & Calibration Constants ===
const int camera_width = 1920;
const int camera_height = 1080;
const float fx_original = 1204.505857f;
const float fy_original = 1204.505857f;
const float cx_original = camera_width / 2.0f;
const float cy_original = camera_height / 2.0f;

const float distortion_k1 = -0.343698f;
const float distortion_k2 = 0.105503f;
const float distortion_k3 = 0.0f;
const float distortion_p1 = -0.001558f;
const float distortion_p2 = -0.002416f;

// === Cylindrical Remap & Offset Settings ===
const float cylindrical_fov_deg = 60.0f;
const float yaw_offset_cam[] = { 0.0f, 60.0f };
const float pitch_offset_cam[] = { 0.0f, 0.5f };
const float panorama_crop_height_scale = 0.5f;
const float panorama_crop_width_scale = 0.615f;
const float original_crop_height_scale = 0.54f;
const float original_crop_width_scale = 0.615f;

// === Translation Factors ===
const float pitch_to_translation_factor = -17.44153917818162f;
const float yaw_to_translation_factor = 17.84f;

float fx = fx_original, fy = fy_original, cx = cx_original, cy = cy_original;

std::mutex orientation_mutex;
double roll = 0.0, pitch = 0.0, yaw = 0.0;

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

// === IMU Orientation Updater ===
// 
// 
// This can be ignored as in the final application this orientation calculation will be done by the MCU
// For Test scenario, fixed dummy values can be used instead of this function. 
// 
// 
// This function runs in a separate thread, continuously collecting gyroscope and accelerometer data
// from the RealSense pipeline. It computes the current orientation (roll, pitch, yaw) using a complementary filter.
// The angles are stabilized with correction for gyroscope drift.
void imu_orientation_updater(rs2::pipeline& pipeline) {
    const double alpha = 0.98, beta = 1.0; // Complementary filter weights
    const double GYRO_DRIFT_X = -0.002843, GYRO_DRIFT_Y = 0.000743, GYRO_DRIFT_Z = -0.001002; // Empirical gyro drift

    double yaw_reset = 0.0, prev_ts = 0.0;
    bool initialized = false;

    while (true) {
        try {
            // Wait for new IMU frames
            rs2::frameset frames = pipeline.wait_for_frames(1000);
            rs2::motion_frame accel = frames.first_or_default(RS2_STREAM_ACCEL);
            rs2::motion_frame gyro = frames.first_or_default(RS2_STREAM_GYRO);
            if (!accel || !gyro) continue;

            auto a = accel.get_motion_data();
            auto g = gyro.get_motion_data();
            double ts = gyro.get_timestamp();

            // First frame: initialize timestamp
            if (!initialized) {
                prev_ts = ts;
                initialized = true;
                continue;
            }

            double dt = (ts - prev_ts) / 1000.0;
            prev_ts = ts;
            if (dt <= 0 || dt > 0.01) continue; // Ignore unrealistic timestamps

            // Calculate roll and pitch from accelerometer
            auto acc_angles = compute_roll_pitch(-a.y, a.x, -a.z);
            double roll_acc = acc_angles.first;
            double pitch_acc = acc_angles.second;

            std::lock_guard<std::mutex> lock(orientation_mutex);
            double gx_deg = (g.x - GYRO_DRIFT_X) * 180.0 / M_PI;
            double gy_deg = (g.y - GYRO_DRIFT_Y) * 180.0 / M_PI;
            double gz_deg = (g.z - GYRO_DRIFT_Z) * 180.0 / M_PI;

            // Apply complementary filter
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

// === Interpolation Function ===
// Bilinear interpolation used during remapping to get smooth pixel values between known pixel coordinates.
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
        float val = (1 - dx) * (1 - dy) * p00[c] + dx * (1 - dy) * p10[c] + (1 - dx) * dy * p01[c] + dx * dy * p11[c];
        result[c] = cv::saturate_cast<uchar>(val);
    }
    return result;
}

// === Manual Undistortion ===
// Removes lens distortion from the input image using the camera intrinsics and distortion coefficients.
cv::Mat manualUndistort(const cv::Mat& img) {
    cv::Mat output(camera_height, camera_width, img.type(), cv::Scalar(0, 0, 0));
    for (int v = 0; v < camera_height; ++v) {
        for (int u = 0; u < camera_width; ++u) {
            float x = (u - cx) / fx;
            float y = (v - cy) / fy;
            float r2 = x * x + y * y;
            float radial = 1 + distortion_k1 * r2 + distortion_k2 * r2 * r2 + distortion_k3 * r2 * r2 * r2;
            float dx = 2 * distortion_p1 * x * y + distortion_p2 * (r2 + 2 * x * x);
            float dy = distortion_p1 * (r2 + 2 * y * y) + 2 * distortion_p2 * x * y;
            float x_dist = x * radial + dx;
            float y_dist = y * radial + dy;
            float u_d = x_dist * fx_original + cx_original;
            float v_d = y_dist * fy_original + cy_original;
            output.at<cv::Vec3b>(v, u) = interpolation(img, u_d, v_d);
        }
    }
    return output;
}

// === Affine Transformation ===
// Applies rotation and vertical translation to simulate stabilization.
cv::Mat applyManualAffine(const cv::Mat& img, float angle_deg, float translate_y) {
    int w = img.cols, h = img.rows;
    float angle_rad = angle_deg * M_PI / 180.0f;
    float cos_a = std::cos(angle_rad);
    float sin_a = std::sin(angle_rad);
    float cx_center = w / 2.0f, cy_center = h / 2.0f;
    cv::Mat output(h, w, img.type(), cv::Scalar(0, 0, 0));
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            float dx = x - cx_center, dy = y - cy_center - translate_y;
            float src_x = cos_a * dx + sin_a * dy + cx_center;
            float src_y = -sin_a * dx + cos_a * dy + cy_center;
            output.at<cv::Vec3b>(y, x) = interpolation(img, src_x, src_y);
        }
    }
    return output;
}

// === Cylindrical Projection Map Generator ===
// Precomputes maps for remapping a rectilinear image to cylindrical coordinates based on field of view.
void generateCylindricalRemap(cv::Mat& map_x, cv::Mat& map_y, int width, int height, float fov_deg = 60.0f, float angle_deg = 0.0f) {
    float fov_rad = fov_deg * M_PI / 180.0f, angle_rad = angle_deg * M_PI / 180.0f;
    float fx_proj = width / (2.0f * std::tan(fov_rad / 2.0f));
    float fy_proj = fx_proj;
    float cx_proj = width / 2.0f, cy_proj = height / 2.0f;
    map_x = cv::Mat(height, width, CV_32FC1);
    map_y = cv::Mat(height, width, CV_32FC1);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            float theta = (x - cx_proj) / fx_proj + angle_rad;
            float h_ = (y - cy_proj) / fy_proj;
            float X = std::sin(theta), Y = h_, Z = std::cos(theta);
            float x_ = fx_proj * X / Z + cx_proj;
            float y_ = fy_proj * Y / Z + cy_proj;
            map_x.at<float>(y, x) = x_;
            map_y.at<float>(y, x) = y_;
        }
    }
}

// === Cylindrical Remapping ===
// Uses precomputed cylindrical maps (map_x, map_y) to remap the input image
// to a cylindrical projection. This method enhances visual consistency in panorama stitching.
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

// === Convert Roll-Pitch-Yaw from One System to Another ===
// Converts RPY angles from one system (e.g., Cam1) to another (e.g., Cam2) using offset correction.
// It builds rotation matrices from Euler angles and combines them to derive new RPY values.
cv::Vec3f convert_rpy_sys1_to_sys(double roll_sys1, double pitch_sys1, double yaw_offset, double pitch_offset = 0.0, double roll_offset = 0.0) {
    double roll_rad = roll_sys1 * M_PI / 180.0, pitch_rad = pitch_sys1 * M_PI / 180.0;
    double roll_offset_rad = roll_offset * M_PI / 180.0, pitch_offset_rad = pitch_offset * M_PI / 180.0;
    double yaw_offset_rad = yaw_offset * M_PI / 180.0;

    // Build rotation from system 1 angles (Roll * Pitch)
    cv::Matx33d Rx = { 1, 0, 0, 0, cos(roll_rad), -sin(roll_rad), 0, sin(roll_rad), cos(roll_rad) };
    cv::Matx33d Ry = { cos(pitch_rad), 0, sin(pitch_rad), 0, 1, 0, -sin(pitch_rad), 0, cos(pitch_rad) };
    cv::Matx33d R_sys1 = Rx * Ry;

    // Build rotation from offsets (Yaw * Pitch * Roll)
    cv::Matx33d R_roll_offset = { 1, 0, 0, 0, cos(roll_offset_rad), -sin(roll_offset_rad), 0, sin(roll_offset_rad), cos(roll_offset_rad) };
    cv::Matx33d R_pitch_offset = { cos(pitch_offset_rad), 0, sin(pitch_offset_rad), 0, 1, 0, -sin(pitch_offset_rad), 0, cos(pitch_offset_rad) };
    cv::Matx33d R_yaw_offset = { cos(yaw_offset_rad), -sin(yaw_offset_rad), 0, sin(yaw_offset_rad), cos(yaw_offset_rad), 0, 0, 0, 1 };
    cv::Matx33d R_offset = R_yaw_offset * R_pitch_offset * R_roll_offset;

    // Final transformed rotation matrix
    cv::Matx33d R_sysX = R_sys1 * R_offset;

    // Convert back to Euler angles (Roll-Pitch-Yaw) using XYZ convention
    double pitchX = asin(-R_sysX(2, 0));
    double cy = cos(pitchX), rollX, yawX;
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

// === Center Crop Function ===
// Extracts a centered sub-region from the input image based on height and width scaling factors.
cv::Mat centerCropOnly(const cv::Mat& frame, float crop_scale_h, float crop_scale_w) {
    int h = frame.rows, w = frame.cols;
    int crop_h = static_cast<int>(h * crop_scale_h);
    int crop_w = static_cast<int>(w * crop_scale_w);
    int start_y = (h - crop_h) / 2;
    int start_x = (w - crop_w) / 2;
    return frame(cv::Rect(start_x, start_y, crop_w, crop_h)).clone();
}



// === MAIN FUNCTION ===
int main() {
    // Set OpenCV logging level to suppress verbose logs
    cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_ERROR);

    // Configure and start RealSense pipeline for IMU (gyro + accel)
    rs2::pipeline pipeline;
    rs2::config config;
    config.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F, 250);
    config.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F, 400);
    std::thread imu_thread;

    try {
        pipeline.start(config);
        imu_thread = std::thread(imu_orientation_updater, std::ref(pipeline)); // Start IMU update thread
    }
    catch (const rs2::error& e) {
        std::cerr << "Failed to start RealSense pipeline: " << e.what() << std::endl;
    }

    // Setup camera intrinsic and distortion parameters for undistortion
    cv::Mat K = (cv::Mat_<double>(3, 3) << fx_original, 0, cx_original, 0, fy_original, cy_original, 0, 0, 1);
    cv::Mat dist = (cv::Mat_<double>(1, 5) << distortion_k1, distortion_k2, distortion_p1, distortion_p2, distortion_k3);
    cv::Mat newK = cv::getOptimalNewCameraMatrix(K, dist, cv::Size(camera_width, camera_height), 1);
    fx = static_cast<float>(newK.at<double>(0, 0));
    fy = static_cast<float>(newK.at<double>(1, 1));
    cx = static_cast<float>(newK.at<double>(0, 2));
    cy = static_cast<float>(newK.at<double>(1, 2));

    // Generate remapping matrices for cylindrical projection
    std::vector<cv::Mat> map_x_list(2), map_y_list(2);
    for (int i = 0; i < 2; ++i)
        generateCylindricalRemap(map_x_list[i], map_y_list[i], camera_width, camera_height, cylindrical_fov_deg);

    // Open camera streams
    std::vector<cv::VideoCapture> cameras(2);
    int camera_indices[] = { 3, 0 }; // Replace with your actual camera IDs
    for (int i = 0; i < 2; ++i) {
        cameras[i].open(camera_indices[i]);
        cameras[i].set(cv::CAP_PROP_FRAME_WIDTH, camera_width);
        cameras[i].set(cv::CAP_PROP_FRAME_HEIGHT, camera_height);
    }

    // Create display windows
    cv::namedWindow("Panorama", cv::WINDOW_NORMAL);
    cv::resizeWindow("Panorama", camera_width * 2, camera_height);
    cv::namedWindow("Original", cv::WINDOW_NORMAL);
    cv::resizeWindow("Original", camera_width * 2, camera_height);

    while (true) {
        // === STEP 1: Capture frames from both cameras ===
        std::vector<cv::Mat> frames(2);
        for (int i = 0; i < 2; ++i) {
            cameras[i] >> frames[i];
            if (frames[i].empty()) {
                std::cerr << "Camera " << i << " failed to capture." << std::endl;
                return -1;
            }
        }

        // === STEP 2: Copy latest IMU orientation values safely ===
        double roll_copy, pitch_copy, yaw_copy;
        {
            std::lock_guard<std::mutex> lock(orientation_mutex);
            roll_copy = roll;
            pitch_copy = pitch;
            yaw_copy = yaw;
        }

        std::vector<cv::Mat> stabilized_frames, original_frames;
        std::vector<std::string> annotations;

        // === STEP 3: Process each camera frame ===
        for (int cam_idx = 0; cam_idx < 2; ++cam_idx) {
            // Convert IMU RPY to current camera orientation with offsets
            auto rpy_corrected = convert_rpy_sys1_to_sys(roll_copy, pitch_copy, yaw_offset_cam[cam_idx], pitch_offset_cam[cam_idx]);
            double roll_corr = rpy_corrected[0];
            double pitch_corr = rpy_corrected[1];
            int y_shift = static_cast<int>(pitch_to_translation_factor * pitch_corr);

            // === STEP 4: Undistort and stabilize image ===
            cv::Mat undistorted = manualUndistort(frames[cam_idx]);
            cv::Mat f_orig = centerCropOnly(undistorted.clone(), original_crop_height_scale, original_crop_width_scale);

            // Apply affine stabilization + cylindrical remap + cropping
            cv::Mat affine = applyManualAffine(undistorted, roll_corr, static_cast<float>(y_shift));
            cv::Mat projected = cylindricalRemap(affine, map_x_list[cam_idx], map_y_list[cam_idx]);
            cv::Mat cropped = centerCropOnly(projected, panorama_crop_height_scale, panorama_crop_width_scale);

            stabilized_frames.push_back(cropped);
            original_frames.push_back(f_orig);

            std::ostringstream oss;
            oss << "Cam" << cam_idx + 1 << " - R:" << roll_corr << " P:" << pitch_corr << " Y:" << yaw_copy;
            annotations.push_back(oss.str());
        }

        // === STEP 5: Concatenate and show result ===
        cv::Mat panorama, original;
        cv::hconcat(stabilized_frames, panorama);
        cv::hconcat(original_frames, original);

        for (size_t i = 0; i < annotations.size(); ++i) {
            cv::putText(panorama, annotations[i], cv::Point(30, 40 + static_cast<int>(i) * 40), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 0), 2);
        }

        cv::imshow("Panorama", panorama);
        cv::imshow("Original", original);
        if (cv::waitKey(1) == 'q') break; // Exit loop on 'q'
    }

    // === STEP 6: Cleanup ===
    for (auto& cam : cameras) cam.release();
    if (imu_thread.joinable()) imu_thread.detach();
    cv::destroyAllWindows();
    return 0;
}

