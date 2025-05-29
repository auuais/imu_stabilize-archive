// SVN_block

#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/logger.hpp>
//#include <librealsense2/rs.hpp>
#include <iostream>
#include <cmath>
#include <mutex>
#include <thread>
#include <Windows.h>
#include <Shlwapi.h>  // PathRemoveFileSpecW
#include <string>
#include <filesystem>
#include "ini_manager.h"
#include <string>
#include <vector>
#include <locale>
#include <codecvt>  // C++17 이전에만 사용 가능

#pragma comment(lib, "Shlwapi.lib")
#pragma comment(lib, "User32.lib")

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
const float cylindrical_fov_deg = 77.0f;
const float yaw_offset_cam[] = { 0.0f, 60.0f };
const float pitch_offset_cam[] = { 0.0f, 0.5f };
const float panorama_crop_height_scale = 0.615f;
const float panorama_crop_width_scale = 0.54f;
const float original_crop_height_scale = 0.7f;
const float original_crop_width_scale = 0.54f;

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
cv::Vec3f convert_rpy_Sensor_to_cam(double roll_sys1, double pitch_sys1, double yaw_offset, double pitch_offset = 0.0, double roll_offset = 0.0) {
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

cv::Mat manualCrop(const cv::Mat& frame, int start_x, int start_y, int crop_w, int crop_h) {
    cv::Mat cropped(crop_h, crop_w, frame.type(), cv::Scalar(0, 0, 0));

    for (int y = 0; y < crop_h; ++y) {
        for (int x = 0; x < crop_w; ++x) {
            int src_x = start_x + x;
            int src_y = start_y + y;

            // Safety check for bounds
            if (src_x >= 0 && src_x < frame.cols && src_y >= 0 && src_y < frame.rows) {
                cropped.at<cv::Vec3b>(y, x) = frame.at<cv::Vec3b>(src_y, src_x);
            }
        }
    }

    return cropped;
}

// === Center Crop Function ===
// Extracts a centered sub-region from the input image based on height and width scaling factors.
cv::Mat centerCropOnly(const cv::Mat& frame, float crop_scale_h, float crop_scale_w) {
    int h = frame.rows, w = frame.cols;
    int crop_h = static_cast<int>(h * crop_scale_h);
    int crop_w = static_cast<int>(w * crop_scale_w);
    int start_y = (h - crop_h) / 2;
    int start_x = (w - crop_w) / 2;
    return manualCrop(frame, start_x, start_y, crop_w, crop_h);

}

std::wstring StringToWString(const std::string& str)
{
    if (str.empty()) return {};
    int size_needed = MultiByteToWideChar(CP_UTF8, 0, str.data(), (int)str.size(), NULL, 0);
    std::wstring result(size_needed, 0);
    MultiByteToWideChar(CP_UTF8, 0, str.data(), (int)str.size(), &result[0], size_needed);
    return result;
}

std::string WStringToUtf8(const std::wstring& wstr)
{
    if (wstr.empty()) return {};
    int size = WideCharToMultiByte(CP_UTF8, 0, wstr.data(), (int)wstr.length(), nullptr, 0, nullptr, nullptr);
    std::string result(size, 0);
    WideCharToMultiByte(CP_UTF8, 0, wstr.data(), (int)wstr.length(), &result[0], size, nullptr, nullptr);
    return result;
}

std::string SaveImageFilePath(const std::wstring& captureDir, const std::wstring&prefix)
{
    SYSTEMTIME st;
    GetLocalTime(&st);

    wchar_t buffer[512];
    swprintf_s(buffer, 512, L"%s\\%04d%02d%02d_%02d%02d%02d_%s.bmp",
        captureDir.c_str(),
        st.wYear, st.wMonth, st.wDay,
        st.wHour, st.wMinute, st.wSecond, prefix.c_str());

    std::wstring widePath(buffer);
    return WStringToUtf8(widePath);
}





int main() {
    wchar_t szPath[MAX_PATH] = L"";
    GetModuleFileNameW(NULL, szPath, MAX_PATH);

    // 2. filesystem으로 경로 파싱
    std::filesystem::path exePath(szPath);

    // 3. 확장자를 .ini로 변경
    std::filesystem::path iniPath = exePath;
    iniPath.replace_extension(L".ini");
	INIManager iniManager(iniPath);

    PathRemoveFileSpecW(szPath);
    std::wstring dirCapture = std::wstring(szPath) + L"\\Capture";
    CreateDirectoryW(dirCapture.c_str(), NULL);  // 이미 존재하면 무시

	// 4. ini 파일에서 설정 읽기
    const int num_cams = 6;
    std::vector<std::string> filenames;
    for (int i = 0; i < num_cams; ++i) {
        std::wstring key = L"file_" + std::to_wstring(i);
        std::wstring val = iniManager.ReadString(L"images", key, L"");
        filenames.push_back(WStringToUtf8(val));
    }

    std::vector<float> yaw_offset_cam = iniManager.ReadFloatArray(L"offset", L"yaw");
    std::vector<float> pitch_offset_cam = iniManager.ReadFloatArray(L"offset", L"pitch");
    std::vector<float> roll_offset_cam = iniManager.ReadFloatArray(L"offset", L"roll");

    // 파일 경로 출력
    std::cout << "[image list]" << std::endl;
    for (size_t i = 0; i < filenames.size(); ++i) {
        std::cout << "  file_" << i << ": " << filenames[i] << std::endl;
    }

    // 오프셋 출력 함수 (재사용 가능)
    auto printOffsets = [](const std::string& label, const std::vector<float>& values) {
        std::cout << "\n[" << label << "]" << std::endl;
        for (size_t i = 0; i < values.size(); ++i) {
            std::cout << "  " << label << "_" << i << ": " << values[i] << std::endl;
        }
        };

    printOffsets("yaw_offset_cam", yaw_offset_cam);
    printOffsets("pitch_offset_cam", pitch_offset_cam);
    printOffsets("roll_offset_cam", roll_offset_cam);

    //std::cout << "\n........Processing........" << std::endl;
    // Setup camera intrinsic and distortion parameters
    cv::Mat K = (cv::Mat_<double>(3, 3) << fx_original, 0, cx_original, 0, fy_original, cy_original, 0, 0, 1);
    cv::Mat dist = (cv::Mat_<double>(1, 5) << distortion_k1, distortion_k2, distortion_p1, distortion_p2, distortion_k3);
    cv::Mat newK = cv::getOptimalNewCameraMatrix(K, dist, cv::Size(camera_width, camera_height), 1);
    fx = static_cast<float>(newK.at<double>(0, 0));
    fy = static_cast<float>(newK.at<double>(1, 1));
    cx = static_cast<float>(newK.at<double>(0, 2));
    cy = static_cast<float>(newK.at<double>(1, 2));    

    // Create cylindrical remaps for each cam
    std::vector<cv::Mat> map_x_list(num_cams), map_y_list(num_cams);
    for (int i = 0; i < num_cams; ++i)
        generateCylindricalRemap(map_x_list[i], map_y_list[i], camera_width, camera_height, cylindrical_fov_deg, 0.0f);

    // Read images
    std::vector<cv::Mat> frames(num_cams);
    for (int i = 0; i < num_cams; ++i) {
        frames[i] = cv::imread(filenames[i]);
        if (frames[i].empty()) {
            std::cerr << "Failed to load " << filenames[i] << std::endl;
            return -1;
        }
    }

    float roll_Sensor = iniManager.ReadFloat(L"Sensor_Orientation", L"roll", 5.25f);
    float pitch_Sensor = iniManager.ReadFloat(L"Sensor_Orientation", L"pitch", -0.75f);
    float yaw_Sensor = iniManager.ReadFloat(L"Sensor_Orientation", L"yaw", 0.0f);

    std::cout << "\n[Sensor_Orientation]" << std::endl;
    std::cout << "  roll_Sensor: " << roll_Sensor << std::endl;
    std::cout << "  pitch_Sensor: " << pitch_Sensor << std::endl;
    std::cout << "  yaw_Sensor: " << yaw_Sensor << std::endl;


    std::cout << "\n........Processing........" << std::endl;


    std::vector<cv::Mat> stabilized_frames, original_frames;
    std::vector<std::string> annotations;

    for (int cam_idx = 0; cam_idx < num_cams; ++cam_idx) {
        auto rpy_cam_transformed = convert_rpy_Sensor_to_cam(
            roll_Sensor, pitch_Sensor,
            yaw_offset_cam[cam_idx],
            pitch_offset_cam[cam_idx],
            roll_offset_cam[cam_idx]
        );

        double roll_cam = rpy_cam_transformed[0];
        double pitch_cam = rpy_cam_transformed[1];
        int y_shift = static_cast<int>(pitch_to_translation_factor * pitch_cam);

        cv::Mat undistorted = manualUndistort(frames[cam_idx]);
        cv::Mat f_orig = centerCropOnly(undistorted.clone(), original_crop_height_scale, original_crop_width_scale);

        cv::Mat affine = applyManualAffine(undistorted, roll_cam, static_cast<float>(y_shift));
        cv::Mat projected = cylindricalRemap(affine, map_x_list[cam_idx], map_y_list[cam_idx]);
        cv::Mat cropped = centerCropOnly(projected, panorama_crop_height_scale, panorama_crop_width_scale);

        stabilized_frames.push_back(cropped);
        original_frames.push_back(f_orig);

        std::ostringstream oss;
        oss << "Cam" << cam_idx << " - R:" << roll_cam << " P:" << pitch_cam << " Y:" << yaw_Sensor;
        annotations.push_back(oss.str());
    }

    cv::Mat panorama, original;
    cv::hconcat(stabilized_frames, panorama);
    cv::hconcat(original_frames, original);

    for (size_t i = 0; i < annotations.size(); ++i) {
        cv::putText(
            panorama,
            annotations[i],
            cv::Point(30, 50 + static_cast<int>(i) * 40), // stacked vertically
            cv::FONT_HERSHEY_SIMPLEX,
            0.8,
            cv::Scalar(255, 255, 0),
            2
        );
    }

    std::string original_path = SaveImageFilePath(dirCapture, L"original");
    std::string result_path = SaveImageFilePath(dirCapture, L"panorama");

    cv::imwrite(original_path, original);
    cv::imwrite(result_path, panorama);

	std::string msg = "\n[Save files]\n" + original_path + "\n" + result_path ;
    std::cout << msg << std::endl;

    return 0;
}



