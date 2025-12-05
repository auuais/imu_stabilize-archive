#define _CRT_SECURE_NO_WARNINGS
#define NOMINMAX

// IMU_stabilize_GYRO_MT_V9_streaming.cpp
// - 6-camera panorama with IMU-based stabilization
// - Cylindrical + undistort maps in Q12.4
// - Horizontally-resampled composed maps
// - Streaming, no-bucket line-buffer renderer per camera
// - Multi-threaded across 6 cameras per frame (std::thread)
// - Frames processed sequentially; profiling per stage retained.

#include <Windows.h>
#include <Shlwapi.h>
#pragma comment(lib, "Shlwapi.lib")

#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cmath>
#include <cstring>
#include <string>
#include <vector>
#include <thread>
#include <array>
#include <filesystem>
#include <chrono>
#include <stdexcept>
#include <atomic>

#include "ini_manager.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ===================== Global switches =====================
#ifndef NUM_FRAMES
#define NUM_FRAMES          960
#endif

#ifndef SAVE_EVERY_K
#define SAVE_EVERY_K        0   // 0 = save none; 1 = save all; 30 = every 30th
#endif

#ifndef STRIP_H
#define STRIP_H             2   // line-buffer height (must be >= 2)
#endif

// ===================== Fixed-point constants =====================
static const int WFIX_SHIFT = 16;   // Q16.16
static const int MAP_SHIFT = 4;     // Q12.4
static const int MAP_SCALE = (1 << MAP_SHIFT);

// ===================== Logging =====================
static inline void logln(const char* s) { printf("%s\r\n", s); fflush(stdout); }

// ===================== Profiling =====================
enum Stage : int { ST_IO = 0, ST_MAPS, ST_UNDISTMAP, ST_LUT, ST_RENDER, ST_BLEND, ST_MISC, ST_COUNT };
static const char* STAGE_NAME[ST_COUNT] = { "I/O","Maps","UndistMap","LUT","Render","Blend","Misc" };

struct StageStats {
    std::atomic<uint64_t>   calls{ 0 };
    std::atomic<long double> ms_total{ 0.0L };
    std::atomic<uint64_t>   bytes_read{ 0 };
    std::atomic<uint64_t>   bytes_written{ 0 };
};

static StageStats G_STATS[ST_COUNT];
static thread_local Stage TL_CUR_STAGE = ST_MISC;

// Light profiling: ignore byte counts
#define MEM_READ(x)  do{}while(0)
#define MEM_WRITE(x) do{}while(0)

struct StageTimer {
    Stage s;
    std::chrono::high_resolution_clock::time_point t0;
    StageTimer(Stage st) : s(st), t0(std::chrono::high_resolution_clock::now()) {
        TL_CUR_STAGE = s;
        G_STATS[s].calls.fetch_add(1, std::memory_order_relaxed);
    }
    ~StageTimer() {
        auto t1 = std::chrono::high_resolution_clock::now();
        long double ms = std::chrono::duration<long double, std::milli>(t1 - t0).count();
        long double old = G_STATS[s].ms_total.load(std::memory_order_relaxed);
        G_STATS[s].ms_total.store(old + ms, std::memory_order_relaxed);
        TL_CUR_STAGE = ST_MISC;
    }
};

static inline void reset_stats() {
    for (int s = 0; s < ST_COUNT; ++s) {
        G_STATS[s].calls.store(0);
        G_STATS[s].ms_total.store(0.0L);
        G_STATS[s].bytes_read.store(0);
        G_STATS[s].bytes_written.store(0);
    }
}

static inline void print_stats_summary(const char* header) {
    printf("\r\n===== %s =====\r\n", header);
    for (int s = 0; s < ST_COUNT; ++s) {
        long double ms = G_STATS[s].ms_total.load();
        double sec = (double)ms / 1000.0;
        double rd_MB = (double)G_STATS[s].bytes_read.load() / (1024.0 * 1024.0);
        double wr_MB = (double)G_STATS[s].bytes_written.load() / (1024.0 * 1024.0);
        double rd_MBps = sec > 0.0 ? rd_MB / sec : 0.0;
        double wr_MBps = sec > 0.0 ? wr_MB / sec : 0.0;
        printf("Stage %-10s | calls:%6llu | time:%9.3Lf ms | R:%7.2f MB | W:%7.2f MB | R:%8.2f MB/s | W:%8.2f MB/s\r\n",
            STAGE_NAME[s],
            (unsigned long long)G_STATS[s].calls.load(),
            ms, rd_MB, wr_MB, rd_MBps, wr_MBps);
    }
    fflush(stdout);
}

// ===================== Helpers =====================
static inline int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }
static unsigned char sat_u8(int v) { return (v < 0) ? 0 : (v > 255 ? 255 : (unsigned char)v); }

// ===================== Image struct (RGB) =====================
typedef struct { int width, height, channels; unsigned char* data; } Image;
#define PIXEL(img,y,x,c) ((img)->data[((y)*(img)->width+(x))*(img)->channels+(c)])

static void freeImage(Image* img) {
    if (!img) return;
    free(img->data);
    free(img);
}

// ===================== BMP I/O =====================
#pragma pack(push,1)
typedef struct {
    unsigned short bfType;
    unsigned int   bfSize;
    unsigned short bfReserved1;
    unsigned short bfReserved2;
    unsigned int   bfOffBits;
} CustomBitmapFileHeader;

typedef struct {
    unsigned int biSize;
    int          biWidth;
    int          biHeight;
    unsigned short biPlanes;
    unsigned short biBitCount;
    unsigned int   biCompression;
    unsigned int   biSizeImage;
    int          biXPelsPerMeter;
    int          biYPelsPerMeter;
    unsigned int   biClrUsed;
    unsigned int   biClrImportant;
} CustomBitmapInfoHeader;
#pragma pack(pop)

static Image* readBMP(const char* filename) {
    StageTimer _t(ST_IO);
    FILE* fp = fopen(filename, "rb");
    if (!fp) return nullptr;

    BITMAPFILEHEADER bfh{};
    BITMAPINFOHEADER bih{};
    fread(&bfh, sizeof(bfh), 1, fp); MEM_READ(sizeof(bfh));
    fread(&bih, sizeof(bih), 1, fp); MEM_READ(sizeof(bih));

    Image* img = (Image*)malloc(sizeof(Image));
    img->width = bih.biWidth;
    img->height = abs(bih.biHeight);
    img->channels = 3;

    int row_padded = ((img->width * 3 + 3) / 4) * 4;
    img->data = (unsigned char*)malloc((size_t)img->width * img->height * 3);

    fseek(fp, bfh.bfOffBits, SEEK_SET);
    for (int i = 0; i < img->height; ++i) {
        size_t got = fread(img->data + (size_t)i * img->width * 3, 1, (size_t)img->width * 3, fp);
        MEM_READ(got);
        fseek(fp, row_padded - img->width * 3, SEEK_CUR);
    }
    fclose(fp);
    return img;
}

static int writeBMP(const char* filename, const Image* img) {
    StageTimer _t(ST_IO);
    FILE* fp = fopen(filename, "wb");
    if (!fp) return 0;

    int w = img->width;
    int h = img->height;
    int c = img->channels;
    if (c != 3) c = 3;

    int row_padded = ((w * 3 + 3) / 4) * 4;
    int fileSize = 54 + row_padded * h;
    unsigned char bmpfileheader[14] = {
        'B','M',
        0,0,0,0,
        0,0,0,0,
        54,0,0,0
    };
    unsigned char bmpinfoheader[40] = { 0 };
    unsigned char pad[3] = { 0,0,0 };

    bmpfileheader[2] = (unsigned char)(fileSize);

    bmpinfoheader[0] = 40;
    bmpinfoheader[4] = (unsigned char)(w);
    bmpinfoheader[5] = (unsigned char)(w >> 8);
    bmpinfoheader[6] = (unsigned char)(w >> 16);
    bmpinfoheader[7] = (unsigned char)(w >> 24);
    bmpinfoheader[8] = (unsigned char)(h);
    bmpinfoheader[9] = (unsigned char)(h >> 8);
    bmpinfoheader[10] = (unsigned char)(h >> 16);
    bmpinfoheader[11] = (unsigned char)(h >> 24);
    bmpinfoheader[12] = 1;
    bmpinfoheader[14] = 24;

    fwrite(bmpfileheader, 1, 14, fp); MEM_WRITE(14);
    fwrite(bmpinfoheader, 1, 40, fp); MEM_WRITE(40);

    for (int i = 0; i < h; ++i) {
        const unsigned char* s = &img->data[(size_t)i * w * c];
        fwrite(s, 1, (size_t)w * 3, fp); MEM_WRITE(w * 3);
        fwrite(pad, 1, (size_t)(row_padded - w * 3), fp); MEM_WRITE(row_padded - w * 3);
    }

    fclose(fp);
    return 1;
}

// ===================== Path helpers =====================
static std::wstring StringToWString(const std::string& s) {
    if (s.empty()) return {};
    int n = MultiByteToWideChar(CP_UTF8, 0, s.data(), (int)s.size(), NULL, 0);
    std::wstring r(n, 0);
    MultiByteToWideChar(CP_UTF8, 0, s.data(), (int)s.size(), &r[0], n);
    return r;
}

static std::string WStringToUtf8(const std::wstring& w) {
    if (w.empty()) return {};
    int n = WideCharToMultiByte(CP_UTF8, 0, w.data(), (int)w.size(), NULL, 0, NULL, NULL);
    std::string r(n, 0);
    WideCharToMultiByte(CP_UTF8, 0, w.data(), (int)w.size(), &r[0], n, NULL, NULL);
    return r;
}

#define CAPTURE_DIR_W L"C:\\SVNProjects\\IMU_Stabilize_V40\\x64\\Debug\\Capture" ///Change to your path please

static std::string SaveImageFilePath(const std::wstring& captureDir, const std::wstring& prefix, int frameIdx = -1) {
    StageTimer _t(ST_IO);
    SYSTEMTIME st; GetLocalTime(&st);
    wchar_t fname[256];
    if (frameIdx < 0)
        swprintf_s(fname, 256, L"%04d%02d%02d_%02d%02d%02d_%s.bmp",
            st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond, prefix.c_str());
    else
        swprintf_s(fname, 256, L"%04d%02d%02d_%02d%02d%02d_%s_f%04d.bmp",
            st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond, prefix.c_str(), frameIdx);
    std::filesystem::path out = std::filesystem::path(captureDir) / fname;
    return WStringToUtf8(out.wstring());
}

// ===================== Global params (INI-driven) =====================
static int   WIDTH = 1920, HEIGHT = 1080, NUM_CAMS = 6, overlap_px = 100;
static float fx = 1204.505857f, fy = 1204.505857f, cx = 960.0f, cy = 540.0f;
static float K1 = -0.343698f, K2 = 0.105503f, K3 = 0.0f, P1 = -0.001558f, P2 = -0.002416f;
static float fx_new = 1204.505857f, fy_new = 1204.505857f, cx_new = 947.559f, cy_new = 534.965f;
static float cylindrical_H_fov_deg = 60.0f, cylindrical_V_fov_deg = 51.3f;
static float panorama_crop_height_scale = 0.80f, panorama_crop_width_scale = 0.80f;
static float resize_factor = 0.5f;
static float PITCH_TO_TRANSLATION_FACTOR = -17.44153917818162f, YAW_TO_TRANSLATION_FACTOR = 17.84f;
static float roll_Sensor = 0.0f, pitch_Sensor = 0.0f, yaw_Sensor = 0.0f;
static int   TARGET_W = 3840, TARGET_H = 480;

// ===================== Angle LUT (±5°, 0.25°) Q1.15 =====================
struct AngleLUTEntry { int16_t c; int16_t s; };
static std::vector<AngleLUTEntry> ANG_LUT;

static inline void build_angle_lut() {
    if (!ANG_LUT.empty()) return;
    const float minDeg = -5.0f, maxDeg = 5.0f, step = 0.25f;
    int N = (int)std::lround((maxDeg - minDeg) / step) + 1;
    ANG_LUT.resize(N);
    for (int i = 0; i < N; ++i) {
        float deg = minDeg + i * step;
        float rad = deg * (float)M_PI / 180.0f;
        int16_t c = (int16_t)std::lroundf(cosf(rad) * 32767.0f);
        int16_t s = (int16_t)std::lroundf(sinf(rad) * 32767.0f);
        ANG_LUT[i] = { c, s };
    }
}

static inline void get_cos_sin_q15(float deg, int16_t* c, int16_t* s) {
    float idxf = (deg + 5.0f) / 0.25f;
    int idx = (int)std::lround(idxf);
    if (idx < 0) idx = 0;
    if (idx >= (int)ANG_LUT.size()) idx = (int)ANG_LUT.size() - 1;
    *c = ANG_LUT[idx].c;
    *s = ANG_LUT[idx].s;
}

// ===================== Cylindrical + undistort maps (Q12.4) =====================
static void loadCylMapsQ(std::vector<int16_t>& mapx, std::vector<int16_t>& mapy, int w, int h, const std::filesystem::path& exePath) {
    StageTimer _t(ST_MAPS);
    mapx.resize((size_t)w * h);
    mapy.resize((size_t)w * h);
    auto px = exePath.parent_path() / "map_x_q.bin";
    auto py = exePath.parent_path() / "map_y_q.bin";
    FILE* fxp = fopen(px.string().c_str(), "rb");
    FILE* fyp = fopen(py.string().c_str(), "rb");
    if (!fxp || !fyp) {
        if (fxp) fclose(fxp);
        if (fyp) fclose(fyp);
        throw std::runtime_error("Cyl maps not found");
    }
    fread(mapx.data(), sizeof(int16_t), (size_t)w * h, fxp); fclose(fxp);
    fread(mapy.data(), sizeof(int16_t), (size_t)w * h, fyp); fclose(fyp);
    printf("[INFO] Cylindrical maps (Q12.4) loaded.\r\n");
}

static void saveCylMapsQ(const std::vector<int16_t>& mapx, const std::vector<int16_t>& mapy, const std::filesystem::path& exePath) {
    StageTimer _t(ST_MAPS);
    auto px = exePath.parent_path() / "map_x_q.bin";
    auto py = exePath.parent_path() / "map_y_q.bin";
    FILE* fxp = fopen(px.string().c_str(), "wb");
    FILE* fyp = fopen(py.string().c_str(), "wb");
    fwrite(mapx.data(), sizeof(int16_t), mapx.size(), fxp); fclose(fxp);
    fwrite(mapy.data(), sizeof(int16_t), mapy.size(), fyp); fclose(fyp);
    printf("[INFO] Cylindrical maps (Q12.4) saved.\r\n");
}

static void generateCylMapsQ(std::vector<int16_t>& mapx, std::vector<int16_t>& mapy,
    int w, int h, float fov_h_deg, float fov_v_deg,
    float x_shift, const std::filesystem::path& exePath) {
    StageTimer _t(ST_MAPS);
    mapx.resize((size_t)w * h);
    mapy.resize((size_t)w * h);

    float fov_h_rad = fov_h_deg * (float)M_PI / 180.0f;
    float fov_v_rad = fov_v_deg * (float)M_PI / 180.0f;
    float fx_cyl = (float)w / (2.0f * tanf(fov_h_rad / 2.0f));
    float fy_cyl = (float)h / (2.0f * tanf(fov_v_rad / 2.0f));
    float cx_cyl = (float)w / 2.0f;
    float cy_cyl = (float)h / 2.0f;

    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            float theta = (x - cx_cyl + x_shift) / fx_cyl;
            float hh = (y - cy_cyl) / fy_cyl;

            float xs = sinf(theta);
            float ys = hh;
            float zs = cosf(theta);

            float src_x = fx * resize_factor * (xs / zs) + cx * resize_factor;
            float src_y = fy * resize_factor * (ys / zs) + cy * resize_factor;

            int32_t qx = (int32_t)lroundf(src_x * MAP_SCALE);
            int32_t qy = (int32_t)lroundf(src_y * MAP_SCALE);
            qx = clampi(qx, INT16_MIN, INT16_MAX);
            qy = clampi(qy, INT16_MIN, INT16_MAX);

            mapx[(size_t)y * w + x] = (int16_t)qx; MEM_WRITE(sizeof(int16_t));
            mapy[(size_t)y * w + x] = (int16_t)qy; MEM_WRITE(sizeof(int16_t));
        }
    }
    saveCylMapsQ(mapx, mapy, exePath);
}

static std::filesystem::path undXPath(const std::filesystem::path& exe) { return exe.parent_path() / L"und_x_q.bin"; }
static std::filesystem::path undYPath(const std::filesystem::path& exe) { return exe.parent_path() / L"und_y_q.bin"; }

static bool loadUndistForwardMapsQ(std::vector<int16_t>& udx_q, std::vector<int16_t>& udy_q,
    int w, int h, const std::filesystem::path& exe) {
    StageTimer _t(ST_UNDISTMAP);
    auto px = undXPath(exe);
    auto py = undYPath(exe);
    FILE* fxp = fopen(px.string().c_str(), "rb");
    FILE* fyp = fopen(py.string().c_str(), "rb");
    if (!fxp || !fyp) {
        if (fxp) fclose(fxp);
        if (fyp) fclose(fyp);
        return false;
    }
    udx_q.resize((size_t)w * h);
    udy_q.resize((size_t)w * h);
    fread(udx_q.data(), sizeof(int16_t), (size_t)w * h, fxp); fclose(fxp);
    fread(udy_q.data(), sizeof(int16_t), (size_t)w * h, fyp); fclose(fyp);
    printf("[INFO] Forward undistort maps (Q12.4) loaded.\r\n");
    return true;
}

static void saveUndistForwardMapsQ(const std::vector<int16_t>& udx_q, const std::vector<int16_t>& udy_q,
    int w, int h, const std::filesystem::path& exe) {
    StageTimer _t(ST_UNDISTMAP);
    auto px = undXPath(exe);
    auto py = undYPath(exe);
    FILE* fxp = fopen(px.string().c_str(), "wb");
    FILE* fyp = fopen(py.string().c_str(), "wb");
    fwrite(udx_q.data(), sizeof(int16_t), udx_q.size(), fxp); fclose(fxp);
    fwrite(udy_q.data(), sizeof(int16_t), udy_q.size(), fyp); fclose(fyp);
    printf("[INFO] Forward undistort maps saved.\r\n");
}

static void generateUndistForwardMapsQ(
    std::vector<int16_t>& udx_q, std::vector<int16_t>& udy_q,
    int w, int h,
    float fx_src, float fy_src, float cx_src, float cy_src,
    float fx_und, float fy_und, float cx_und, float cy_und,
    float k1, float k2, float k3, float p1, float p2,
    float resize_factor_local)
{
    StageTimer _t(ST_UNDISTMAP);
    udx_q.resize((size_t)w * h);
    udy_q.resize((size_t)w * h);

    const float fx_r = fx_src * resize_factor_local;
    const float fy_r = fy_src * resize_factor_local;
    const float cx_r = cx_src * resize_factor_local;
    const float cy_r = cy_src * resize_factor_local;
    const float fxu_r = fx_und * resize_factor_local;
    const float fyu_r = fy_und * resize_factor_local;
    const float cxu_r = cx_und * resize_factor_local;
    const float cyu_r = cy_und * resize_factor_local;

    for (int v = 0; v < h; ++v) {
        for (int u = 0; u < w; ++u) {
            float x = (u - cxu_r) / fxu_r;
            float y = (v - cyu_r) / fyu_r;
            float r2 = x * x + y * y;
            float r4 = r2 * r2;
            float r6 = r4 * r2;

            float radial = 1.0f + k1 * r2 + k2 * r4 + k3 * r6;
            float dx = 2.0f * p1 * x * y + p2 * (r2 + 2.0f * x * x);
            float dy = p1 * (r2 + 2.0f * y * y) + 2.0f * p2 * x * y;

            float xd = x * radial + dx;
            float yd = y * radial + dy;
            float u_d = xd * fx_r + cx_r;
            float v_d = yd * fy_r + cy_r;

            int32_t qx = (int32_t)lroundf(u_d * MAP_SCALE);
            int32_t qy = (int32_t)lroundf(v_d * MAP_SCALE);
            qx = clampi(qx, INT16_MIN, INT16_MAX);
            qy = clampi(qy, INT16_MIN, INT16_MAX);

            size_t idx = (size_t)v * w + u;
            udx_q[idx] = (int16_t)qx;
            udy_q[idx] = (int16_t)qy;
            MEM_WRITE(2 * sizeof(int16_t));
        }
    }
    printf("[INFO] Forward undistort maps (Q12.4) built.\r\n");
}

// ===================== Compose cyl + undist (Q12.4) =====================
static std::filesystem::path compXPath(const std::filesystem::path& exe) { return exe.parent_path() / L"comp_x_q.bin"; }
static std::filesystem::path compYPath(const std::filesystem::path& exe) { return exe.parent_path() / L"comp_y_q.bin"; }

static bool loadComposedMapsQ(std::vector<int16_t>& ox, std::vector<int16_t>& oy,
    int w, int h, const std::filesystem::path& exe) {
    StageTimer _t(ST_UNDISTMAP);
    auto px = compXPath(exe);
    auto py = compYPath(exe);
    FILE* fxp = fopen(px.string().c_str(), "rb");
    FILE* fyp = fopen(py.string().c_str(), "rb");
    if (!fxp || !fyp) {
        if (fxp) fclose(fxp);
        if (fyp) fclose(fyp);
        return false;
    }
    ox.resize((size_t)w * h);
    oy.resize((size_t)w * h);
    fread(ox.data(), sizeof(int16_t), (size_t)w * h, fxp); fclose(fxp);
    fread(oy.data(), sizeof(int16_t), (size_t)w * h, fyp); fclose(fyp);
    printf("[INFO] Composed maps loaded.\r\n");
    return true;
}

static void saveComposedMapsQ(const std::vector<int16_t>& ox, const std::vector<int16_t>& oy,
    int w, int h, const std::filesystem::path& exe) {
    StageTimer _t(ST_UNDISTMAP);
    auto px = compXPath(exe);
    auto py = compYPath(exe);
    FILE* fxp = fopen(px.string().c_str(), "wb");
    FILE* fyp = fopen(py.string().c_str(), "wb");
    fwrite(ox.data(), sizeof(int16_t), ox.size(), fxp); fclose(fxp);
    fwrite(oy.data(), sizeof(int16_t), oy.size(), fyp); fclose(fyp);
    printf("[INFO] Composed maps saved.\r\n");
}

static void composeCylWithUndistQ(
    const std::vector<int16_t>& cyl_x_q,
    const std::vector<int16_t>& cyl_y_q,
    const std::vector<int16_t>& und_x_q,
    const std::vector<int16_t>& und_y_q,
    int w, int h,
    std::vector<int16_t>& out_x_q,
    std::vector<int16_t>& out_y_q)
{
    StageTimer _t(ST_UNDISTMAP);
    out_x_q.resize((size_t)w * h);
    out_y_q.resize((size_t)w * h);

    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            int32_t fu_q = (int32_t)cyl_x_q[(size_t)y * w + x];
            int32_t fv_q = (int32_t)cyl_y_q[(size_t)y * w + x];

            int32_t fu_q16 = fu_q << (WFIX_SHIFT - MAP_SHIFT);
            int32_t fv_q16 = fv_q << (WFIX_SHIFT - MAP_SHIFT);

            int iu = clampi((fu_q16 >> WFIX_SHIFT), 0, w - 2);
            int iv = clampi((fv_q16 >> WFIX_SHIFT), 0, h - 2);

            uint8_t wx8 = (uint8_t)((fu_q16 >> 8) & 0xFF);
            uint8_t wy8 = (uint8_t)((fv_q16 >> 8) & 0xFF);

            int W_x1 = (int)wx8;
            int W_y1 = (int)wy8;
            int W_x0 = 256 - W_x1;
            int W_y0 = 256 - W_y1;

            int32_t v00x = und_x_q[(size_t)iv * w + iu];
            int32_t v10x = und_x_q[(size_t)iv * w + iu + 1];
            int32_t v01x = und_x_q[(size_t)(iv + 1) * w + iu];
            int32_t v11x = und_x_q[(size_t)(iv + 1) * w + iu + 1];

            int32_t v00y = und_y_q[(size_t)iv * w + iu];
            int32_t v10y = und_y_q[(size_t)iv * w + iu + 1];
            int32_t v01y = und_y_q[(size_t)(iv + 1) * w + iu];
            int32_t v11y = und_y_q[(size_t)(iv + 1) * w + iu + 1];

            auto bilerp_q = [&](int32_t A00, int32_t A10, int32_t A01, int32_t A11)->int32_t {
                int32_t h0 = A00 + (((A10 - A00) * W_x1) >> 8);
                int32_t h1 = A01 + (((A11 - A01) * W_x1) >> 8);
                return h0 + (((h1 - h0) * W_y1) >> 8);
                };

            int32_t ux_q = bilerp_q(v00x, v10x, v01x, v11x);
            int32_t uy_q = bilerp_q(v00y, v10y, v01y, v11y);

            out_x_q[(size_t)y * w + x] = (int16_t)clampi(ux_q, INT16_MIN, INT16_MAX);
            out_y_q[(size_t)y * w + x] = (int16_t)clampi(uy_q, INT16_MIN, INT16_MAX);
            MEM_WRITE(2 * sizeof(int16_t));
        }
    }
    printf("[INFO] Composed Cyl×Undist maps built (Q12.4).\r\n");
}

// ===================== IMU sensor -> camera =====================
static void convert_rpy_sensor_to_cam(double roll_sys1, double pitch_sys1,
    double yaw_offset, double pitch_offset, double roll_offset,
    float* roll_deg_out, float* pitch_deg_out, float* yaw_deg_out) {
    StageTimer _t(ST_MISC);
    double rr = roll_sys1 * M_PI / 180.0;
    double pp = pitch_sys1 * M_PI / 180.0;
    double ror = roll_offset * M_PI / 180.0;
    double por = pitch_offset * M_PI / 180.0;
    double yor = yaw_offset * M_PI / 180.0;

    double Rx[3][3] = { {1,0,0},{0,cos(rr),-sin(rr)},{0,sin(rr),cos(rr)} };
    double Ry[3][3] = { {cos(pp),0,sin(pp)},{0,1,0},{-sin(pp),0,cos(pp)} };
    double R_sys1[3][3];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            R_sys1[i][j] = Rx[i][0] * Ry[0][j] + Rx[i][1] * Ry[1][j] + Rx[i][2] * Ry[2][j];

    double Rr[3][3] = { {1,0,0},{0,cos(ror),-sin(ror)},{0,sin(ror),cos(ror)} };
    double Rp[3][3] = { {cos(por),0,sin(por)},{0,1,0},{-sin(por),0,cos(por)} };
    double Rz[3][3] = { {cos(yor),-sin(yor),0},{sin(yor),cos(yor),0},{0,0,1} };

    double Rt[3][3];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            Rt[i][j] = Rz[i][0] * Rp[0][j] + Rz[i][1] * Rp[1][j] + Rz[i][2] * Rp[2][j];

    double Roff[3][3];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            Roff[i][j] = Rt[i][0] * Rr[0][j] + Rt[i][1] * Rr[1][j] + Rt[i][2] * Rr[2][j];

    double R[3][3];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            R[i][j] = R_sys1[i][0] * Roff[0][j] + R_sys1[i][1] * Roff[1][j] + R_sys1[i][2] * Roff[2][j];

    double pitchX = asin(-R[2][0]);
    double cy_ = cos(pitchX);
    double rollX, yawX;
    if (fabs(cy_) > 1e-6) {
        rollX = atan2(R[2][1], R[2][2]);
        yawX = atan2(R[1][0], R[0][0]);
    }
    else {
        rollX = 0;
        yawX = atan2(-R[0][1], R[1][1]);
    }
    *roll_deg_out = (float)(rollX * 180.0 / M_PI);
    *pitch_deg_out = (float)(pitchX * 180.0 / M_PI);
    *yaw_deg_out = (float)(yawX * 180.0 / M_PI);
}

// ===================== Streaming, no-bucket line-buffer renderer =====================
// - For each camera, walks all per-camera output pixels once.
// - Computes mapping via composed maps + IMU roll/pitch.
// - Maintains a sliding vertical window [baseY, baseY+STRIP_H) in a small line buffer.
// - Reloads the buffer only when y0 leaves the current window.
static void renderStreamingNoBuckets_FromResampled(
    int camIdx,
    const std::vector<int16_t>& comp_x_s,
    const std::vector<int16_t>& comp_y_s,
    int per_cam_w,
    int per_cam_h,
    int src_stride_for_comp,   // per_cam_w_max
    const Image* src,
    int resized_w,
    int resized_h,
    float yaw_off_deg,
    float pit_off_deg,
    float rol_off_deg,
    Image* out)
{
    (void)camIdx; (void)resized_w; (void)resized_h; // not used directly here

    StageTimer _t(ST_RENDER);

    // IMU -> cam orientation
    float roll_cam = 0, pitch_cam = 0, yaw_cam = 0;
    convert_rpy_sensor_to_cam(roll_Sensor, pitch_Sensor,
        yaw_off_deg, pit_off_deg, rol_off_deg,
        &roll_cam, &pitch_cam, &yaw_cam);

    // Vertical translation from pitch
    int y_shift_px = (int)lroundf(PITCH_TO_TRANSLATION_FACTOR * pitch_cam);

    // Roll rotation (Q1.15)
    int16_t c15 = 0, s15 = 0;
    get_cos_sin_q15(-roll_cam, &c15, &s15);

    const int32_t yshift_q = ((int32_t)(-y_shift_px)) << WFIX_SHIFT;
    const int32_t inv_resize_q = (int32_t)llroundf((1.0f / resize_factor) * (1 << WFIX_SHIFT));

    const int resized_w_r = resized_w;
    const int resized_h_r = resized_h;

    const int32_t cxr_q = (resized_w_r << 15);
    const int32_t cyr_q = (resized_h_r << 15);

    const int in_w = src->width;
    const int in_h = src->height;
    const int32_t max_x_q = ((in_w - 2) << WFIX_SHIFT);
    const int32_t max_y_q = ((in_h - 2) << WFIX_SHIFT);

    // Output image allocation
    out->width = per_cam_w;
    out->height = per_cam_h;
    out->channels = 3;
    out->data = (unsigned char*)malloc((size_t)per_cam_w * per_cam_h * 3);
    if (!out->data) return;

    const int sstride = in_w * 3;

    // Sliding line buffer window [baseY, baseY + validH)
    int baseY = -1000;
    int validH = 0;
    std::vector<unsigned char> linebuf((size_t)STRIP_H * sstride);

    auto loadWindow = [&](int y0) {
        // Ensure we have a window that covers y0..y0+1
        baseY = y0;
        if (baseY < 0) baseY = 0;
        if (baseY > in_h - STRIP_H) baseY = in_h - STRIP_H;
        validH = STRIP_H;
        if (baseY < 0) { baseY = 0; }
        if (baseY + validH > in_h) validH = in_h - baseY;
        if (validH < 2) validH = 2; // at least 2 rows for bilinear

        for (int sy = 0; sy < validH; ++sy) {
            const unsigned char* srcRow = &src->data[(size_t)(baseY + sy) * sstride];
            unsigned char* dstRow = &linebuf[(size_t)sy * sstride];
            memcpy(dstRow, srcRow, (size_t)sstride);
        }
        };

    for (int oy = 0; oy < per_cam_h; ++oy) {
        const int16_t* mx = &comp_x_s[(size_t)oy * src_stride_for_comp];
        const int16_t* my = &comp_y_s[(size_t)oy * src_stride_for_comp];

        for (int ox = 0; ox < per_cam_w; ++ox) {
            // --- Mapping (same math as previous bucket LUT, but per-pixel streaming) ---
            int32_t srcx_q16 = ((int32_t)mx[ox]) << (WFIX_SHIFT - MAP_SHIFT);
            int32_t srcy_q16 = ((int32_t)my[ox]) << (WFIX_SHIFT - MAP_SHIFT);

            int32_t dx_q = srcx_q16 - cxr_q;
            int32_t dy_q = (srcy_q16 - cyr_q) - yshift_q;

            int32_t ax_q = (int32_t)(((int64_t)c15 * dx_q + (int64_t)s15 * dy_q) >> 15) + cxr_q;
            int32_t ay_q = (int32_t)((-(int64_t)s15 * dx_q + (int64_t)c15 * dy_q) >> 15) + cyr_q;

            int32_t ox_q = (int32_t)(((int64_t)ax_q * inv_resize_q) >> WFIX_SHIFT);
            int32_t oy_q = (int32_t)(((int64_t)ay_q * inv_resize_q) >> WFIX_SHIFT);

            ox_q = clampi(ox_q, 0, max_x_q);
            oy_q = clampi(oy_q, 0, max_y_q);

            int16_t x0 = (int16_t)(ox_q >> WFIX_SHIFT);
            int16_t y0 = (int16_t)(oy_q >> WFIX_SHIFT);
            uint16_t wx = (uint16_t)(ox_q & 0xFFFF);
            uint16_t wy = (uint16_t)(oy_q & 0xFFFF);

            if (y0 < 0 || y0 >= in_h - 1) continue;
            if (x0 < 0 || x0 >= in_w - 1) continue;

            // --- Ensure line-buffer window covers this y0 ---
            if (!(y0 >= baseY && y0 < baseY + validH - 1)) {
                loadWindow(y0);
            }

            int sy_local = y0 - baseY;
            if (sy_local < 0) sy_local = 0;
            if (sy_local > validH - 2) sy_local = validH - 2;

            unsigned W_x0 = 0x10000u - wx;
            unsigned W_x1 = wx;
            unsigned W_y0 = 0x10000u - wy;
            unsigned W_y1 = wy;

            unsigned W00 = (unsigned)(((uint64_t)W_x0 * W_y0) >> 16);
            unsigned W10 = (unsigned)(((uint64_t)W_x1 * W_y0) >> 16);
            unsigned W01 = (unsigned)(((uint64_t)W_x0 * W_y1) >> 16);
            unsigned W11 = (unsigned)(((uint64_t)W_x1 * W_y1) >> 16);

            const unsigned char* p0 = &linebuf[(size_t)sy_local * sstride + x0 * 3];
            const unsigned char* p1 = p0 + 3;
            const unsigned char* p2 = p0 + sstride;
            const unsigned char* p3 = p2 + 3;

            unsigned char* dst = &out->data[((size_t)oy * per_cam_w + ox) * 3];

            dst[0] = (unsigned char)((p0[0] * W00 + p1[0] * W10 + p2[0] * W01 + p3[0] * W11) >> 16);
            dst[1] = (unsigned char)((p0[1] * W00 + p1[1] * W10 + p2[1] * W01 + p3[1] * W11) >> 16);
            dst[2] = (unsigned char)((p0[2] * W00 + p1[2] * W10 + p2[2] * W01 + p3[2] * W11) >> 16);
        }
    }
}

// ===================== Feather blend (vertical pad) =====================
static Image* featherBlendWithPad(
    const std::vector<Image*>& frames,
    int overlap,
    int final_w,
    int final_h,
    float feather_strength = 1.0f)
{
    StageTimer _t(ST_BLEND);
    if (frames.empty()) return nullptr;

    const int c = 3;
    Image* blended = (Image*)malloc(sizeof(Image));
    blended->width = final_w;
    blended->height = final_h;
    blended->channels = c;
    blended->data = (unsigned char*)calloc((size_t)final_w * final_h * c, 1);

    const int src_h = frames[0]->height;
    const int ypad = (final_h - src_h) / 2;

    std::vector<uint16_t> alpha(std::max(overlap, 1));
    if (overlap > 0) {
        const double inv_overlap = 1.0 / overlap;
        const double gamma = 1.0 / std::max<float>(0.1f, feather_strength);
        for (int x = 0; x < overlap; ++x) {
            double t = (x + 0.5) * inv_overlap;
            double s = std::pow(t, gamma);
            alpha[x] = (uint16_t)std::lround(s * 65535.0);
        }
    }

    for (int y = 0; y < final_h; ++y) {
        int sy = y - ypad;
        if (sy < 0 || sy >= src_h) continue;

        unsigned char* dstRow = &blended->data[(size_t)y * final_w * c];

        const Image* img0 = frames[0];
        const unsigned char* src0 = &img0->data[(size_t)sy * img0->width * c];
        memcpy(dstRow, src0, (size_t)img0->width * c);
        int curx = img0->width;

        for (size_t i = 1; i < frames.size(); ++i) {
            const Image* img = frames[i];
            const unsigned char* srcRow = &img->data[(size_t)sy * img->width * c];
            int ov = overlap;
            if (ov > img->width) ov = img->width;
            if (ov > curx) ov = curx;

            for (int x = 0; x < ov; ++x) {
                unsigned a = alpha[x];
                size_t dp = (size_t)(curx - ov + x) * c;
                size_t sp = (size_t)x * c;
                for (int ch = 0; ch < 3; ++ch) {
                    unsigned prev = dstRow[dp + ch];
                    unsigned curr = srcRow[sp + ch];
                    dstRow[dp + ch] = (unsigned char)((prev * (0x10000 - a) + curr * a) >> 16);
                }
            }

            int tail = img->width - ov;
            if (tail > 0) {
                memcpy(&dstRow[(size_t)curx * c],
                    &srcRow[(size_t)ov * c],
                    (size_t)tail * c);
            }
            curx += tail;
        }
    }
    return blended;
}

// ===================== Main =====================
int main() {
    build_angle_lut();

    wchar_t wpath[MAX_PATH] = L"";
    GetModuleFileNameW(NULL, wpath, MAX_PATH);
    std::filesystem::path exePath(wpath);
    std::filesystem::path imuIniPath = exePath; imuIniPath.replace_extension(L".ini");
    std::filesystem::path paramsIniPath = exePath; paramsIniPath.replace_filename(L"parameters.ini");

    // Read parameters
    INIManager paramIni(paramsIniPath);
    WIDTH = paramIni.ReadInteger(L"image", L"width", WIDTH);
    HEIGHT = paramIni.ReadInteger(L"image", L"height", HEIGHT);
    NUM_CAMS = paramIni.ReadInteger(L"image", L"num_cams", NUM_CAMS);

    fx = paramIni.ReadFloat(L"calibration", L"fx", fx);
    fy = paramIni.ReadFloat(L"calibration", L"fy", fy);
    float cx_ratio = paramIni.ReadFloat(L"calibration", L"cx_ratio", 0.5f);
    float cy_ratio = paramIni.ReadFloat(L"calibration", L"cy_ratio", 0.5f);
    cx = WIDTH * cx_ratio;
    cy = HEIGHT * cy_ratio;

    fx_new = paramIni.ReadFloat(L"calibration", L"fx_dist", fx_new);
    fy_new = paramIni.ReadFloat(L"calibration", L"fy_dist", fy_new);
    cx_new = paramIni.ReadFloat(L"calibration", L"cx_dist", cx_new);
    cy_new = paramIni.ReadFloat(L"calibration", L"cy_dist", cy_new);

    K1 = paramIni.ReadFloat(L"calibration", L"k1", K1);
    K2 = paramIni.ReadFloat(L"calibration", L"k2", K2);
    K3 = paramIni.ReadFloat(L"calibration", L"k3", K3);
    P1 = paramIni.ReadFloat(L"calibration", L"p1", P1);
    P2 = paramIni.ReadFloat(L"calibration", L"p2", P2);

    cylindrical_H_fov_deg = paramIni.ReadFloat(L"projection", L"cylindrical_H_fov_deg", cylindrical_H_fov_deg);
    cylindrical_V_fov_deg = paramIni.ReadFloat(L"projection", L"cylindrical_V_fov_deg", cylindrical_V_fov_deg);

    panorama_crop_height_scale = paramIni.ReadFloat(L"cropping", L"panorama_crop_height_scale", panorama_crop_height_scale);
    panorama_crop_width_scale = paramIni.ReadFloat(L"cropping", L"panorama_crop_width_scale", panorama_crop_width_scale);

    resize_factor = paramIni.ReadFloat(L"resize", L"resize_factor ", resize_factor);
    PITCH_TO_TRANSLATION_FACTOR =
        paramIni.ReadFloat(L"transform", L"pitch_to_translation_factor", PITCH_TO_TRANSLATION_FACTOR) * resize_factor;
    YAW_TO_TRANSLATION_FACTOR =
        paramIni.ReadFloat(L"transform", L"yaw_to_translation_factor", YAW_TO_TRANSLATION_FACTOR);

    overlap_px = paramIni.ReadInteger(L"blend", L"overlap_px ", overlap_px);
    float feather_strength = paramIni.ReadFloat(L"blend", L"feather_strength", 1.0f);

    TARGET_W = paramIni.ReadInteger(L"output", L"target_w", TARGET_W);
    TARGET_H = paramIni.ReadInteger(L"output", L"target_h", TARGET_H);

    // IMU ini
    INIManager imuIni(imuIniPath);

    // Input files
    std::vector<std::string> filenames; filenames.reserve(NUM_CAMS);
    for (int i = 0; i < NUM_CAMS; ++i) {
        std::wstring key = L"file_" + std::to_wstring(i);
        filenames.push_back(WStringToUtf8(imuIni.ReadString(L"images", key, L"")));
    }

    std::vector<float> yaw_off = imuIni.ReadFloatArray(L"offset", L"yaw");
    std::vector<float> pit_off = imuIni.ReadFloatArray(L"offset", L"pitch");
    std::vector<float> rol_off = imuIni.ReadFloatArray(L"offset", L"roll");

    auto clamp5 = [&](float v) { return v > 5.0f ? 5.0f : (v < -5.0f ? -5.0f : v); };
    bool StabilizationIni = imuIni.ReadBoolean(L"Sensor_Orientation", L"stabilization", true);
    if (StabilizationIni) {
        roll_Sensor = clamp5(imuIni.ReadFloat(L"Sensor_Orientation", L"roll", 0.0f));
        pitch_Sensor = clamp5(imuIni.ReadFloat(L"Sensor_Orientation", L"pitch", 0.0f));
        yaw_Sensor = clamp5(imuIni.ReadFloat(L"Sensor_Orientation", L"yaw", 0.0f));
    }
    else {
        roll_Sensor = pitch_Sensor = yaw_Sensor = 0.0f;
    }

    logln("........Processing (MT-V9: 6 threads per frame, streaming no-bucket + line buffer)........");

    // Geometry in resized space
    const int resized_w = (int)lroundf(WIDTH * resize_factor);
    const int resized_h = (int)lroundf(HEIGHT * resize_factor);
    const int crop_w = (int)lroundf(resized_w * panorama_crop_width_scale);
    const int crop_h = (int)lroundf(resized_h * panorama_crop_height_scale);
    const int crop_x0 = (resized_w - crop_w) / 2;
    const int crop_y0 = (resized_h - crop_h) / 2;

    // Maps
    std::vector<int16_t> cyl_x_q, cyl_y_q;
    try {
        loadCylMapsQ(cyl_x_q, cyl_y_q, resized_w, resized_h, exePath);
    }
    catch (...) {
        generateCylMapsQ(cyl_x_q, cyl_y_q, resized_w, resized_h, cylindrical_H_fov_deg, cylindrical_V_fov_deg, 0.0f, exePath);
    }

    std::vector<int16_t> und_x_q, und_y_q;
    if (!loadUndistForwardMapsQ(und_x_q, und_y_q, resized_w, resized_h, exePath)) {
        generateUndistForwardMapsQ(und_x_q, und_y_q, resized_w, resized_h,
            fx, fy, cx, cy,
            fx_new, fy_new, cx_new, cy_new,
            K1, K2, K3, P1, P2, resize_factor);
        saveUndistForwardMapsQ(und_x_q, und_y_q, resized_w, resized_h, exePath);
    }

    std::vector<int16_t> comp_x_q, comp_y_q;
    if (!loadComposedMapsQ(comp_x_q, comp_y_q, resized_w, resized_h, exePath)) {
        composeCylWithUndistQ(cyl_x_q, cyl_y_q, und_x_q, und_y_q,
            resized_w, resized_h, comp_x_q, comp_y_q);
        saveComposedMapsQ(comp_x_q, comp_y_q, resized_w, resized_h, exePath);
    }

    // Load all camera frames
    std::vector<Image*> frames(NUM_CAMS, nullptr);
    for (int i = 0; i < NUM_CAMS; ++i) {
        frames[i] = readBMP(filenames[i].c_str());
        if (!frames[i]) {
            printf("[ERROR] Failed to load camera %d\n", i);
            return 1;
        }
        printf("[INFO] Camera %d: %dx%d\n", i, frames[i]->width, frames[i]->height);
    }

    // Per-camera target widths to fill TARGET_W with overlap
    const int overlap_resized = (int)lroundf(overlap_px * resize_factor);
    const int pano_w_crop = NUM_CAMS * crop_w - (NUM_CAMS - 1) * overlap_resized;
    const double scale_x = (double)TARGET_W / (double)pano_w_crop;

    int overlap_target = (int)llround(overlap_resized * scale_x);
    int per_cam_w_base = (int)llround(crop_w * scale_x);

    std::vector<int> per_cam_w(NUM_CAMS, per_cam_w_base);
    {
        int total_now = NUM_CAMS * per_cam_w_base - (NUM_CAMS - 1) * overlap_target;
        int delta = TARGET_W - total_now;
        for (int i = 0; delta > 0 && i < NUM_CAMS; ++i, --delta) per_cam_w[i]++;
        for (int i = 0; delta < 0 && i < NUM_CAMS; ++i, ++delta)
            if (per_cam_w[i] > 1) per_cam_w[i]--;
    }
    const int per_cam_h = crop_h;
    int per_cam_w_max = 0;
    for (int w : per_cam_w) per_cam_w_max = std::max(per_cam_w_max, w);

    // One-time horizontal resample of composed maps to per_cam_w_max x per_cam_h
    std::vector<int16_t> comp_x_s((size_t)per_cam_w_max * per_cam_h);
    std::vector<int16_t> comp_y_s((size_t)per_cam_w_max * per_cam_h);

    std::vector<int> iu(per_cam_w_max);
    std::vector<uint8_t> wx8(per_cam_w_max);

    {
        const double sx = (double)crop_w / (double)per_cam_w_max;
        for (int x = 0; x < per_cam_w_max; ++x) {
            double u = crop_x0 + (x + 0.5) * sx - 0.5;
            int iu0 = clampi((int)floor(u), 0, resized_w - 2);
            iu[x] = iu0;
            int w8 = (int)std::lround((u - iu0) * 256.0);
            if (w8 < 0) w8 = 0; if (w8 > 255) w8 = 255;
            wx8[x] = (uint8_t)w8;
        }
    }

    {
        for (int y = 0; y < per_cam_h; ++y) {
            int src_y = clampi(crop_y0 + y, 0, resized_h - 2);
            const int16_t* r0x = &comp_x_q[(size_t)src_y * resized_w];
            const int16_t* r0y = &comp_y_q[(size_t)src_y * resized_w];
            const int16_t* r1x = &comp_x_q[(size_t)(src_y + 1) * resized_w];
            const int16_t* r1y = &comp_y_q[(size_t)(src_y + 1) * resized_w];

            int16_t* dxx = &comp_x_s[(size_t)y * per_cam_w_max];
            int16_t* dyy = &comp_y_s[(size_t)y * per_cam_w_max];

            for (int x = 0; x < per_cam_w_max; ++x) {
                int u0 = iu[x];
                int Wx1 = wx8[x];

                int32_t a00x = r0x[u0], a10x = r0x[u0 + 1];
                int32_t a00y = r0y[u0], a10y = r0y[u0 + 1];

                int32_t hx0x = a00x + (((a10x - a00x) * Wx1) >> 8);
                int32_t hx0y = a00y + (((a10y - a00y) * Wx1) >> 8);

                dxx[x] = (int16_t)clampi(hx0x, INT16_MIN, INT16_MAX);
                dyy[x] = (int16_t)clampi(hx0y, INT16_MIN, INT16_MAX);
            }
        }
    }

    // Prepare capture dir
    std::wstring captureDirW = CAPTURE_DIR_W;
    CreateDirectoryW(captureDirW.c_str(), NULL);

    std::vector<double> frame_ms(NUM_FRAMES, 0.0);
    auto total_t0 = std::chrono::high_resolution_clock::now();

    for (int f = 0; f < NUM_FRAMES; ++f) {
        reset_stats();
        auto frame_t0 = std::chrono::high_resolution_clock::now();

        printf("\n[Frame %d] start\n", f);

        std::array<Image*, 6> stabilized{};
        std::array<std::thread, 6> threads;

        for (int cam = 0; cam < NUM_CAMS; ++cam) {
            stabilized[cam] = (Image*)malloc(sizeof(Image));
            threads[cam] = std::thread([&, cam] {
                renderStreamingNoBuckets_FromResampled(
                    cam,
                    comp_x_s,
                    comp_y_s,
                    per_cam_w[cam],
                    per_cam_h,
                    per_cam_w_max,
                    frames[cam],
                    resized_w,
                    resized_h,
                    yaw_off[cam],
                    pit_off[cam],
                    rol_off[cam],
                    stabilized[cam]);
                });
        }

        for (int cam = 0; cam < NUM_CAMS; ++cam) {
            if (threads[cam].joinable()) threads[cam].join();
        }

        std::vector<Image*> views;
        views.reserve(NUM_CAMS);
        for (int i = 0; i < NUM_CAMS; ++i) views.push_back(stabilized[i]);

        Image* pano = featherBlendWithPad(
            views,
            overlap_target,
            TARGET_W,
            TARGET_H,
            feather_strength);

        if (SAVE_EVERY_K > 0 && ((f + 1) % SAVE_EVERY_K == 0)) {
            std::string path = SaveImageFilePath(captureDirW, L"panorama_v9_streaming", f);
            writeBMP(path.c_str(), pano);
            printf("[Saved] %s\n", path.c_str());
        }

        auto frame_t1 = std::chrono::high_resolution_clock::now();
        double ms = std::chrono::duration<double, std::milli>(frame_t1 - frame_t0).count();
        frame_ms[f] = ms;
        printf("[Frame %d] time %.3f ms (%.2f FPS)\n", f, ms, 1000.0 / ms);

        print_stats_summary("PROFILING SUMMARY (MT-V9 streaming, single frame)");

        for (int i = 0; i < NUM_CAMS; ++i) {
            if (stabilized[i]) { free(stabilized[i]->data); free(stabilized[i]); }
        }
        if (pano) { free(pano->data); free(pano); }
    }

    auto total_t1 = std::chrono::high_resolution_clock::now();
    double total_ms = std::chrono::duration<double, std::milli>(total_t1 - total_t0).count();
    double throughput_fps = 1000.0 * NUM_FRAMES / total_ms;

    double sum_ms = 0.0;
    for (int i = 0; i < NUM_FRAMES; ++i) sum_ms += frame_ms[i];
    double avg_ms = (NUM_FRAMES > 0) ? sum_ms / NUM_FRAMES : 0.0;
    double latency_fps = avg_ms > 0.0 ? 1000.0 / avg_ms : 0.0;

    printf("\n===== MULTI-FRAME SUMMARY (MT-V9, streaming no-buckets) =====\n");
    printf("Frames: %d\n", NUM_FRAMES);
    printf("Avg latency per frame: %.3f ms (%.2f FPS)\n", avg_ms, latency_fps);
    printf("Total time: %.3f ms (Throughput %.2f FPS)\n", total_ms, throughput_fps);

    for (auto* fimg : frames) if (fimg) { free(fimg->data); free(fimg); }

    return 0;
}
