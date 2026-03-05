// IMU_stabilize_GYRO_MT_V19.cpp
//
// V19 (PC-time optimizations + Release path robustness):


#define _CRT_SECURE_NO_WARNINGS
#define NOMINMAX

#include <Windows.h>
#include <Shlwapi.h>
#include <Commdlg.h>
#pragma comment(lib, "Shlwapi.lib")
#pragma comment(lib, "Comdlg32.lib")

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
#include <algorithm>
#include <mutex>
#include <condition_variable>
#include <memory>

#include "ini_manager.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ===================== Global switches =====================
int NUM_FRAMES = 1;
int SAVE_EVERY_K = 1;

#ifndef SCANLINE_BUFFER
#define SCANLINE_BUFFER     2   // hard constraint
#endif

#ifndef ACTIVE_BUFFER
#define ACTIVE_BUFFER       180 // active destination-row slots (fixed for FPGA - sized for worst case: IR 640x480)
#endif

#ifndef PING_PONG_PUSH_BUFFER
#define PING_PONG_PUSH_BUFFER 32  // pano slice ping-pong height
#endif

static constexpr int FIXED_INPUT_W = 1920;
static constexpr int FIXED_OUTPUT_W = 3840;

// ===================== Fixed-point constants =====================
static const int WFIX_SHIFT = 16;   // Q16.16
static const int MAP_SHIFT = 4;    // Q12.4
static const int MAP_SCALE = (1 << MAP_SHIFT);

// RowRun delta stored as Q12.4 (int16), promoted to Q16.16 by << 12
static const int DRUN_SHIFT = 4;                        // Q12.4
static const int DRUN_TO_Q16 = (WFIX_SHIFT - DRUN_SHIFT);// 12

#ifndef RUN_TOL_Q16
#define RUN_TOL_Q16         (256) // 1/256 pixel in Q16.16
#endif

// ===================== Logging =====================
static inline void logln(const char* s) { printf("%s\r\n", s); fflush(stdout); }

// ===================== Profiling (thread-safe) =====================
enum Stage : int {
    ST_IO = 0,
    ST_MAPS,
    ST_UNDISTMAP,
    ST_LUT,
    ST_SCHED,
    ST_RENDER,
    ST_BLEND,
    ST_DDR,
    ST_MISC,
    ST_COUNT
};

static const char* STAGE_NAME[ST_COUNT] = {
    "I/O","Maps","UndistMap","LUT","Sched","Render","Blend","DDR","Misc"
};

struct StageStats {
    std::atomic<uint64_t> calls{ 0 };
    std::atomic<uint64_t> ns_total{ 0 };
    std::atomic<uint64_t> bytes_read{ 0 };
    std::atomic<uint64_t> bytes_written{ 0 };
};

static StageStats G_STATS[ST_COUNT];
static thread_local Stage TL_CUR_STAGE = ST_MISC;

#define MEM_READ(n)  do{ G_STATS[TL_CUR_STAGE].bytes_read.fetch_add((uint64_t)(n), std::memory_order_relaxed); }while(0)
#define MEM_WRITE(n) do{ G_STATS[TL_CUR_STAGE].bytes_written.fetch_add((uint64_t)(n), std::memory_order_relaxed); }while(0)

struct StageTimer {
    Stage s;
    std::chrono::high_resolution_clock::time_point t0;
    StageTimer(Stage st) : s(st), t0(std::chrono::high_resolution_clock::now()) {
        TL_CUR_STAGE = s;
        G_STATS[s].calls.fetch_add(1, std::memory_order_relaxed);
    }
    ~StageTimer() {
        auto t1 = std::chrono::high_resolution_clock::now();
        uint64_t ns = (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count();
        G_STATS[s].ns_total.fetch_add(ns, std::memory_order_relaxed);
        TL_CUR_STAGE = ST_MISC;
    }
};

static inline void reset_stats() {
    for (int s = 0; s < ST_COUNT; ++s) {
        G_STATS[s].calls.store(0, std::memory_order_relaxed);
        G_STATS[s].ns_total.store(0, std::memory_order_relaxed);
        G_STATS[s].bytes_read.store(0, std::memory_order_relaxed);
        G_STATS[s].bytes_written.store(0, std::memory_order_relaxed);
    }
}

static inline void print_stats_summary(const char* header) {
    printf("\r\n===== %s =====\r\n", header);
    for (int s = 0; s < ST_COUNT; ++s) {
        uint64_t ns = G_STATS[s].ns_total.load(std::memory_order_relaxed);
        long double ms = (long double)ns / 1.0e6L;
        double sec = (double)ns / 1.0e9;

        double rd_MB = (double)G_STATS[s].bytes_read.load(std::memory_order_relaxed) / (1024.0 * 1024.0);
        double wr_MB = (double)G_STATS[s].bytes_written.load(std::memory_order_relaxed) / (1024.0 * 1024.0);
        double rd_MBps = sec > 0.0 ? rd_MB / sec : 0.0;
        double wr_MBps = sec > 0.0 ? wr_MB / sec : 0.0;

        printf("Stage %-10s | calls:%6llu | time:%9.3Lf ms | R:%7.2f MB | W:%7.2f MB | R:%8.2f MB/s | W:%8.2f MB/s\r\n",
            STAGE_NAME[s],
            (unsigned long long)G_STATS[s].calls.load(std::memory_order_relaxed),
            ms, rd_MB, wr_MB, rd_MBps, wr_MBps);
    }
    fflush(stdout);
}

// ===================== Helpers =====================
static inline int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }
static inline unsigned char sat_u8(int v) { return (v < 0) ? 0 : (v > 255 ? 255 : (unsigned char)v); }
static inline uint64_t ceil_div_u64(uint64_t a, uint64_t b) { return (a + b - 1) / b; }
static inline int32_t deg_to_centi(float deg) { return (int32_t)std::lround((double)deg * 100.0); }
static inline float centi_to_deg(int32_t centi) { return (float)centi * 0.01f; }
static inline int32_t float_to_q16(float v) { return (int32_t)std::lround((double)v * 65536.0); }
static inline float q16_to_float(int32_t q16) { return (float)q16 / 65536.0f; }

static inline int32_t div_round_i64(int64_t num, int64_t den)
{
    if (den == 0) return 0;
    if (num >= 0) return (int32_t)((num + den / 2) / den);
    return (int32_t)((num - den / 2) / den);
}

static inline uint64_t yuv422_bytes(int w, int h) {
    return (uint64_t)w * (uint64_t)h * 2ull;
}

// ===================== Image struct (RGB for I/O only) =====================
typedef struct { int width, height, channels; unsigned char* data; } Image;

static void freeImage(Image* img) {
    if (!img) return;
    free(img->data);
    free(img);
}

// ===================== YUV 4:2:2 planar struct =====================
struct YUV422Image {
    int width = 0;
    int height = 0;
    unsigned char* Y = nullptr;
    unsigned char* U = nullptr;
    unsigned char* V = nullptr;
};

static void freeYUV422ImageHeap(YUV422Image* img) {
    if (!img) return;
    free(img->Y);
    free(img->U);
    free(img->V);
    free(img);
}

// ===================== RGB -> YUV422 planar =====================
static YUV422Image* rgbToYUV422(const Image* src) {
    StageTimer _t(ST_MISC);
    if (!src || src->channels != 3) return nullptr;

    const int w = src->width;
    const int h = src->height;
    const int cw = (w + 1) / 2;

    YUV422Image* yuv = (YUV422Image*)malloc(sizeof(YUV422Image));
    if (!yuv) return nullptr;
    yuv->width = w;
    yuv->height = h;
    yuv->Y = (unsigned char*)malloc((size_t)w * h);
    yuv->U = (unsigned char*)malloc((size_t)cw * h);
    yuv->V = (unsigned char*)malloc((size_t)cw * h);

    if (!yuv->Y || !yuv->U || !yuv->V) {
        freeYUV422ImageHeap(yuv);
        return nullptr;
    }

    const unsigned char* srcData = src->data;
    for (int y = 0; y < h; ++y) {
        const unsigned char* row = srcData + (size_t)y * w * 3;
        unsigned char* rowY = yuv->Y + (size_t)y * w;
        unsigned char* rowU = yuv->U + (size_t)y * cw;
        unsigned char* rowV = yuv->V + (size_t)y * cw;

        for (int x = 0; x < w; x += 2) {
            int x0 = x;
            int x1 = (x + 1 < w) ? (x + 1) : (w - 1);

            const unsigned char* p0 = row + x0 * 3;
            const unsigned char* p1 = row + x1 * 3;

            int B0 = p0[0], G0 = p0[1], R0 = p0[2];
            int B1 = p1[0], G1 = p1[1], R1 = p1[2];

            int Y0 = (int)std::lround(0.299f * R0 + 0.587f * G0 + 0.114f * B0);
            int Y1 = (int)std::lround(0.299f * R1 + 0.587f * G1 + 0.114f * B1);
            rowY[x0] = sat_u8(Y0);
            if (x1 < w) rowY[x1] = sat_u8(Y1);

            float Uf0 = -0.168736f * R0 - 0.331264f * G0 + 0.5f * B0 + 128.0f;
            float Vf0 = 0.5f * R0 - 0.418688f * G0 - 0.081312f * B0 + 128.0f;
            float Uf1 = -0.168736f * R1 - 0.331264f * G1 + 0.5f * B1 + 128.0f;
            float Vf1 = 0.5f * R1 - 0.418688f * G1 - 0.081312f * B1 + 128.0f;

            int Uc = (int)std::lround((Uf0 + Uf1) * 0.5f);
            int Vc = (int)std::lround((Vf0 + Vf1) * 0.5f);

            int cidx = x / 2;
            rowU[cidx] = sat_u8(Uc);
            rowV[cidx] = sat_u8(Vc);
        }
    }
    return yuv;
}

// ===================== YUV422 -> BGR (debug/save) =====================
static Image* yuv422ToBGR(const YUV422Image* src) {
    if (!src) return nullptr;

    const int w = src->width;
    const int h = src->height;
    const int cw = (w + 1) / 2;

    Image* img = (Image*)malloc(sizeof(Image));
    if (!img) return nullptr;

    img->width = w;
    img->height = h;
    img->channels = 3;
    img->data = (unsigned char*)malloc((size_t)w * h * 3);
    if (!img->data) { free(img); return nullptr; }

    for (int y = 0; y < h; ++y) {
        const unsigned char* rowY = src->Y + (size_t)y * w;
        const unsigned char* rowU = src->U + (size_t)y * cw;
        const unsigned char* rowV = src->V + (size_t)y * cw;
        unsigned char* dst = img->data + (size_t)y * w * 3;

        for (int x = 0; x < w; x += 2) {
            int cidx = x / 2;

            int Y0 = rowY[x];
            int Y1 = (x + 1 < w) ? rowY[x + 1] : Y0;

            int Uc = rowU[cidx];
            int Vc = rowV[cidx];
            int U = Uc - 128;
            int V = Vc - 128;

            auto write_pixel = [&](int Yval) {
                float Yf = (float)Yval;
                float Uf = (float)U;
                float Vf = (float)V;

                float Rf = Yf + 1.402f * Vf;
                float Gf = Yf - 0.344136f * Uf - 0.714136f * Vf;
                float Bf = Yf + 1.772f * Uf;

                int R = clampi((int)std::lround(Rf), 0, 255);
                int G = clampi((int)std::lround(Gf), 0, 255);
                int B = clampi((int)std::lround(Bf), 0, 255);

                dst[0] = (unsigned char)B;
                dst[1] = (unsigned char)G;
                dst[2] = (unsigned char)R;
                dst += 3;
                };

            write_pixel(Y0);
            if (x + 1 < w) write_pixel(Y1);
        }
    }
    return img;
}

// ===================== BMP I/O =====================
static Image* readBMP(const char* filename) {
    StageTimer _t(ST_IO);
    FILE* fp = fopen(filename, "rb");
    if (!fp) return nullptr;

    BITMAPFILEHEADER bfh{};
    BITMAPINFOHEADER bih{};
    if (fread(&bfh, sizeof(bfh), 1, fp) != 1) { fclose(fp); return nullptr; } MEM_READ(sizeof(bfh));
    if (fread(&bih, sizeof(bih), 1, fp) != 1) { fclose(fp); return nullptr; } MEM_READ(sizeof(bih));

    if (bfh.bfType != 0x4D42) { fclose(fp); return nullptr; } // "BM"

    Image* img = (Image*)malloc(sizeof(Image));
    if (!img) { fclose(fp); return nullptr; }

    img->width = bih.biWidth;
    img->height = abs(bih.biHeight);
    img->channels = 3;

    int row_padded = ((img->width * 3 + 3) / 4) * 4;
    img->data = (unsigned char*)malloc((size_t)img->width * img->height * 3);
    if (!img->data) { free(img); fclose(fp); return nullptr; }

    fseek(fp, bfh.bfOffBits, SEEK_SET);

    // NOTE: This reads rows as stored (BMP can be bottom-up). For your pipeline, orientation is usually irrelevant.
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

    unsigned char bmpfileheader[14] = { 'B','M',0,0,0,0,0,0,0,0,54,0,0,0 };
    unsigned char bmpinfoheader[40] = { 0 };
    unsigned char pad[3] = { 0,0,0 };

    bmpfileheader[2] = (unsigned char)(fileSize);
    bmpfileheader[3] = (unsigned char)(fileSize >> 8);
    bmpfileheader[4] = (unsigned char)(fileSize >> 16);
    bmpfileheader[5] = (unsigned char)(fileSize >> 24);

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
        fwrite(s, 1, (size_t)w * 3, fp); MEM_WRITE((size_t)w * 3);
        fwrite(pad, 1, (size_t)(row_padded - w * 3), fp); MEM_WRITE((size_t)(row_padded - w * 3));
    }
    fclose(fp);
    return 1;
}

static int writeYUV422(const char* filename, const YUV422Image* img) {
StageTimer _t(ST_IO);
FILE* fp = fopen(filename, "wb");
if (!fp) return 0;
const int w = img->width;
const int h = img->height;
const int cw = (w + 1) / 2;
    fwrite(img->Y, 1, (size_t)w * h, fp); MEM_WRITE((size_t)w * h);
    fwrite(img->U, 1, (size_t)cw * h, fp); MEM_WRITE((size_t)cw * h);
    fwrite(img->V, 1, (size_t)cw * h, fp); MEM_WRITE((size_t)cw * h);
    fclose(fp);
    return 1;
}

// ===================== Prompt helpers =====================
static std::string PromptUserForFile(const char* title, const char* filter) {
    char filename[MAX_PATH] = "";
    OPENFILENAMEA ofn;
    ZeroMemory(&ofn, sizeof(ofn));
    ofn.lStructSize = sizeof(ofn);
    ofn.hwndOwner = NULL;
    ofn.lpstrFilter = filter;
    ofn.lpstrFile = filename;
    ofn.nMaxFile = MAX_PATH;
    ofn.lpstrTitle = title;
    ofn.Flags = OFN_DONTADDTORECENT | OFN_FILEMUSTEXIST | OFN_NOCHANGEDIR;

    if (GetOpenFileNameA(&ofn)) {
        return std::string(filename);
    }
    return "";
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

#include <shobjidl.h>

static std::vector<std::string> PromptUserForMultipleFiles(const char* title, const char* filter) {
    std::vector<std::string> files;
    char filename[MAX_PATH * 50] = {0};
    OPENFILENAMEA ofn;
    ZeroMemory(&ofn, sizeof(ofn));
    ofn.lStructSize = sizeof(ofn);
    ofn.hwndOwner = NULL;
    ofn.lpstrFilter = filter;
    ofn.lpstrFile = filename;
    ofn.nMaxFile = sizeof(filename);
    ofn.lpstrTitle = title;
    ofn.Flags = OFN_DONTADDTORECENT | OFN_FILEMUSTEXIST | OFN_NOCHANGEDIR | OFN_ALLOWMULTISELECT | OFN_EXPLORER;

    if (GetOpenFileNameA(&ofn)) {
        char* p = filename;
        std::string dir = p;
        p += dir.length() + 1;
        if (*p == '\0') {
            files.push_back(dir);
        } else {
            while (*p) {
                std::string filepath = dir + "\\" + p;
                files.push_back(filepath);
                p += strlen(p) + 1;
            }
        }
    }
    return files;
}

static std::string PromptUserForFolder(const char* title) {
    std::string selectedFolder = "";
    IFileDialog *pfd;
    if (SUCCEEDED(CoCreateInstance(CLSID_FileOpenDialog, NULL, CLSCTX_INPROC_SERVER, IID_PPV_ARGS(&pfd)))) {
        DWORD dwOptions;
        if (SUCCEEDED(pfd->GetOptions(&dwOptions))) {
            pfd->SetOptions(dwOptions | FOS_PICKFOLDERS | FOS_FORCEFILESYSTEM);
        }
        std::wstring wtitle = StringToWString(title);
        pfd->SetTitle(wtitle.c_str());
        if (SUCCEEDED(pfd->Show(NULL))) {
            IShellItem *psi;
            if (SUCCEEDED(pfd->GetResult(&psi))) {
                PWSTR pszPath;
                if (SUCCEEDED(psi->GetDisplayName(SIGDN_FILESYSPATH, &pszPath))) {
                    selectedFolder = WStringToUtf8(pszPath);
                    CoTaskMemFree(pszPath);
                }
                psi->Release();
            }
        }
        pfd->Release();
    }
    return selectedFolder;
}

#define CAPTURE_DIR_W L"C:\\SVNProjects\\IMU_Stabilize_V40\\x64\\Debug\\Capture"

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

static std::string SaveYUVFilePath(const std::wstring& captureDir, const std::wstring& prefix, int frameIdx = -1) {
    StageTimer _t(ST_IO);
    SYSTEMTIME st; GetLocalTime(&st);
    wchar_t fname[256];
    if (frameIdx < 0)
        swprintf_s(fname, 256, L"%04d%02d%02d_%02d%02d%02d_%s.yuv",
            st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond, prefix.c_str());
    else
        swprintf_s(fname, 256, L"%04d%02d%02d_%02d%02d%02d_%s_f%04d.yuv",
            st.wYear, st.wMonth, st.wDay, st.wHour, st.wMinute, st.wSecond, prefix.c_str(), frameIdx);
    std::filesystem::path out = std::filesystem::path(captureDir) / fname;
    return WStringToUtf8(out.wstring());
}

static inline bool file_exists(const std::filesystem::path& p) {
    std::error_code ec;
    return std::filesystem::exists(p, ec);
}

static std::filesystem::path pick_imu_ini(const std::filesystem::path& exePath, int argc, char** argv) {
    if (argc >= 2 && argv[1] && argv[1][0]) {
        return std::filesystem::path(StringToWString(argv[1]));
    }
    std::filesystem::path exeIni = exePath; exeIni.replace_extension(L".ini");
    if (file_exists(exeIni)) return exeIni;

    std::filesystem::path imuIni = exePath.parent_path() / L"imu.ini";
    if (file_exists(imuIni)) return imuIni;

    // fallback: try IMU.ini
    std::filesystem::path imuIni2 = exePath.parent_path() / L"IMU.ini";
    if (file_exists(imuIni2)) return imuIni2;

    return exeIni; // will fail later with clear error
}

static std::filesystem::path pick_params_ini(const std::filesystem::path& exePath, int argc, char** argv) {
    if (argc >= 3 && argv[2] && argv[2][0]) {
        return std::filesystem::path(StringToWString(argv[2]));
    }
    auto try_paths = [&](const std::vector<std::filesystem::path>& candidates) -> std::filesystem::path {
        for (const auto& p : candidates) {
            if (file_exists(p)) return p;
        }
        return candidates.empty() ? exePath : candidates.back();
    };

    std::filesystem::path exeDir = exePath.parent_path();
    std::filesystem::path parentDir = exeDir.parent_path();

    // Prefer unified; then legacy; search exe dir then parent dir (project root).
    return try_paths({
        exeDir / L"parameters_unified.ini",
        exeDir / L"parameters.ini",
        parentDir / L"parameters_unified.ini",
        parentDir / L"parameters.ini"
    });
}

static std::string make_abs_if_relative(const std::string& in, const std::filesystem::path& baseDir) {
    if (in.empty()) return in;
    std::filesystem::path p = std::filesystem::path(StringToWString(in));
    if (p.is_relative()) {
        p = baseDir / p;
    }
    return WStringToUtf8(p.wstring());
}

// ===================== Global params (INI-driven) =====================
static int   WIDTH = 1920, HEIGHT = 1080, NUM_CAMS = 6, overlap_px = 100;
static float fx = 1204.505857f, fy = 1204.505857f, cx = 960.0f, cy = 540.0f;
static float K1 = -0.343698f, K2 = 0.105503f, K3 = 0.0f, P1 = -0.001558f, P2 = -0.002416f;
static float fx_new = 1204.505857f, fy_new = 1204.505857f, cx_new = 947.559f, cy_new = 534.965f;
static float cylindrical_H_fov_deg = 77.0f, cylindrical_V_fov_deg = 51.3f;
static float panorama_crop_height_scale = 0.70f, panorama_crop_width_scale = 0.70f;
static float resize_factor = 0.5f;
static float PITCH_TO_TRANSLATION_FACTOR = -17.44153917818162f, YAW_TO_TRANSLATION_FACTOR = 17.84f;
static int32_t panorama_crop_height_scale_q16 = (int32_t)(0.70f * 65536.0f + 0.5f);
static int32_t panorama_crop_width_scale_q16 = (int32_t)(0.70f * 65536.0f + 0.5f);
static int32_t resize_factor_q16 = (int32_t)(0.5f * 65536.0f + 0.5f);
static int32_t PITCH_TO_TRANSLATION_FACTOR_q16 = (int32_t)(-17.44153917818162f * 65536.0f);
static int32_t YAW_TO_TRANSLATION_FACTOR_q16 = (int32_t)(17.84f * 65536.0f + 0.5f);
static int32_t roll_Sensor_centi = 0, pitch_Sensor_centi = 0, yaw_Sensor_centi = 0;
static int   TARGET_W = 3840, TARGET_H = 480;
static float feather_strength = 1.0f;
static int32_t feather_strength_q16 = (1 << 16);

// ===================== Mode / Config =====================
enum OutputMode : int {
    MODE_BLACKOUT                 = 0x00,
    MODE_EO_PANO_STAB             = 0x01,
    MODE_IR_PANO_STAB             = 0x02,
    MODE_EO_PANO_NO_STAB          = 0x03,
    MODE_IR_PANO_NO_STAB          = 0x06,
    MODE_EO_UNDIST_CAM1           = 0x07,
    MODE_EO_UNDIST_CAM6           = 0x0C,
    MODE_IR_UNDIST_CAM1           = 0x0D,
    MODE_IR_UNDIST_CAM6           = 0x12,
    MODE_STATUS                   = 0x13
};

struct CameraConfig {
    int   width = 1920;
    int   height = 1080;
    int   num_cams = 6;

    float fx = 1204.505857f;
    float fy = 1204.505857f;
    float cx_ratio = 0.5f;
    float cy_ratio = 0.5f;

    float fx_dist = 1204.505857f;
    float fy_dist = 1204.505857f;
    float cx_dist = 947.559f;
    float cy_dist = 534.965f;

    float k1 = -0.343698f;
    float k2 = 0.105503f;
    float k3 = 0.0f;
    float p1 = -0.001558f;
    float p2 = -0.002416f;

    float resize_factor = 0.5f;
    int32_t resize_factor_q16 = (int32_t)(0.5f * 65536.0f + 0.5f);
    float cyl_h_fov = 77.0f;
    float cyl_v_fov = 51.3f;

    float pano_crop_h = 0.70f;
    float pano_crop_w = 0.70f;
    int32_t pano_crop_h_q16 = (int32_t)(0.70f * 65536.0f + 0.5f);
    int32_t pano_crop_w_q16 = (int32_t)(0.70f * 65536.0f + 0.5f);

    bool use_nearest_neighbor = false;

    float pitch_to_translation_factor = -17.44f;
    float yaw_to_translation_factor   = 17.84f;
    int32_t pitch_to_translation_factor_q16 = (int32_t)(-17.44f * 65536.0f);
    int32_t yaw_to_translation_factor_q16 = (int32_t)(17.84f * 65536.0f + 0.5f);

    int   overlap_px = 100;
    float feather_strength = 1.0f;
    int32_t feather_strength_q16 = (1 << 16);

    int   pano_width = 3840;
    int   pano_height = 480;

    // offsets
    std::vector<int32_t> yaw_off;
    std::vector<int32_t> pitch_off;
    std::vector<int32_t> roll_off;

    // image files
    std::vector<std::string> filenames;
};

static int32_t readAngleCenti(INIManager& ini, const wchar_t* section, const wchar_t* keyBase, int32_t defaultCenti)
{
    // Prefer human-readable floating degree keys; quantize immediately.
    if (ini.HasKey(section, keyBase)) {
        const float defaultDeg = centi_to_deg(defaultCenti);
        return deg_to_centi(ini.ReadFloat(section, keyBase, defaultDeg));
    }

    const std::wstring keyCenti = std::wstring(keyBase) + L"_centi";
    if (ini.HasKey(section, keyCenti)) {
        return (int32_t)ini.ReadInteger(section, keyCenti, (int)defaultCenti);
    }

    const std::wstring keyCdeg = std::wstring(keyBase) + L"_cdeg";
    if (ini.HasKey(section, keyCdeg)) {
        return (int32_t)ini.ReadInteger(section, keyCdeg, (int)defaultCenti);
    }

    return defaultCenti;
}

static int32_t readAngleCentiWithFallback(INIManager& ini, const wchar_t* primarySection,
    const wchar_t* fallbackSection, const wchar_t* keyBase, int32_t defaultCenti)
{
    int32_t fb = readAngleCenti(ini, fallbackSection, keyBase, defaultCenti);
    return readAngleCenti(ini, primarySection, keyBase, fb);
}

static std::vector<int32_t> readAngleArrayCenti(INIManager& ini, const wchar_t* section, const wchar_t* keyBase)
{
    // Prefer human-readable floating degree arrays; quantize immediately.
    if (ini.HasKey(section, keyBase)) {
        std::vector<float> valsDeg = ini.ReadFloatArray(section, keyBase);
        std::vector<int32_t> valsCenti;
        valsCenti.reserve(valsDeg.size());
        for (float v : valsDeg) valsCenti.push_back(deg_to_centi(v));
        return valsCenti;
    }

    const std::wstring keyCenti = std::wstring(keyBase) + L"_centi";
    if (ini.HasKey(section, keyCenti)) {
        const std::vector<int> vals = ini.ReadIntegerArray(section, keyCenti);
        return std::vector<int32_t>(vals.begin(), vals.end());
    }

    const std::wstring keyCdeg = std::wstring(keyBase) + L"_cdeg";
    if (ini.HasKey(section, keyCdeg)) {
        const std::vector<int> vals = ini.ReadIntegerArray(section, keyCdeg);
        return std::vector<int32_t>(vals.begin(), vals.end());
    }

    return {};
}

static CameraConfig loadCameraConfig(INIManager& ini, bool isEO, const std::filesystem::path& baseDir) {
    CameraConfig cfg;
    const wchar_t* camSec = isEO ? L"eo_camera" : L"ir_camera";
    const wchar_t* imgSec = isEO ? L"eo_images" : L"ir_images";
    const wchar_t* offSec = isEO ? L"eo_offset" : L"ir_offset";

    cfg.width = ini.ReadInteger(camSec, L"width", cfg.width);
    cfg.height = ini.ReadInteger(camSec, L"height", cfg.height);
    cfg.num_cams = ini.ReadInteger(camSec, L"num_cams", cfg.num_cams);

    cfg.fx = ini.ReadFloat(camSec, L"fx", cfg.fx);
    cfg.fy = ini.ReadFloat(camSec, L"fy", cfg.fy);
    cfg.cx_ratio = ini.ReadFloat(camSec, L"cx_ratio", cfg.cx_ratio);
    cfg.cy_ratio = ini.ReadFloat(camSec, L"cy_ratio", cfg.cy_ratio);

    cfg.fx_dist = ini.ReadFloat(camSec, L"fx_dist", cfg.fx_dist);
    cfg.fy_dist = ini.ReadFloat(camSec, L"fy_dist", cfg.fy_dist);
    cfg.cx_dist = ini.ReadFloat(camSec, L"cx_dist", cfg.cx_dist);
    cfg.cy_dist = ini.ReadFloat(camSec, L"cy_dist", cfg.cy_dist);

    cfg.k1 = ini.ReadFloat(camSec, L"k1", cfg.k1);
    cfg.k2 = ini.ReadFloat(camSec, L"k2", cfg.k2);
    cfg.k3 = ini.ReadFloat(camSec, L"k3", cfg.k3);
    cfg.p1 = ini.ReadFloat(camSec, L"p1", cfg.p1);
    cfg.p2 = ini.ReadFloat(camSec, L"p2", cfg.p2);

    cfg.resize_factor = ini.ReadFloat(camSec, L"resize_factor", cfg.resize_factor);
    cfg.resize_factor_q16 = float_to_q16(cfg.resize_factor);
    cfg.cyl_h_fov = ini.ReadFloat(camSec, L"cylindrical_H_fov_deg", cfg.cyl_h_fov);
    cfg.cyl_v_fov = ini.ReadFloat(camSec, L"cylindrical_V_fov_deg", cfg.cyl_v_fov);

    cfg.pano_crop_h = ini.ReadFloat(camSec, L"panorama_crop_height_scale", cfg.pano_crop_h);
    cfg.pano_crop_w = ini.ReadFloat(camSec, L"panorama_crop_width_scale", cfg.pano_crop_w);
    cfg.pano_crop_h_q16 = float_to_q16(cfg.pano_crop_h);
    cfg.pano_crop_w_q16 = float_to_q16(cfg.pano_crop_w);

    cfg.pitch_to_translation_factor = ini.ReadFloat(camSec, L"pitch_to_translation_factor", cfg.pitch_to_translation_factor);
    cfg.yaw_to_translation_factor   = ini.ReadFloat(camSec, L"yaw_to_translation_factor", cfg.yaw_to_translation_factor);
    cfg.pitch_to_translation_factor_q16 = float_to_q16(cfg.pitch_to_translation_factor);
    cfg.yaw_to_translation_factor_q16 = float_to_q16(cfg.yaw_to_translation_factor);

    cfg.overlap_px = ini.ReadInteger(camSec, L"overlap_px", cfg.overlap_px);
    cfg.feather_strength = ini.ReadFloat(camSec, L"feather_strength", cfg.feather_strength);
    cfg.feather_strength_q16 = float_to_q16(cfg.feather_strength);

    cfg.pano_width = ini.ReadInteger(camSec, L"pano_width", cfg.pano_width);
    cfg.pano_height = ini.ReadInteger(camSec, L"pano_height", cfg.pano_height);

    cfg.use_nearest_neighbor = ini.ReadBoolean(camSec, L"use_nearest_neighbor", cfg.use_nearest_neighbor);

    cfg.yaw_off = readAngleArrayCenti(ini, offSec, L"yaw");
    cfg.pitch_off = readAngleArrayCenti(ini, offSec, L"pitch");
    cfg.roll_off = readAngleArrayCenti(ini, offSec, L"roll");

    cfg.filenames.reserve(cfg.num_cams);
    for (int i = 0; i < cfg.num_cams; ++i) {
        std::wstring key = L"file_" + std::to_wstring(i);
        std::string p = WStringToUtf8(ini.ReadString(imgSec, key, L""));
        p = make_abs_if_relative(p, baseDir);
        cfg.filenames.push_back(p);
    }
    return cfg;
}

static void applyCameraConfig(const CameraConfig& cfg) {
    WIDTH = cfg.width;
    HEIGHT = cfg.height;
    NUM_CAMS = cfg.num_cams;

    fx = cfg.fx;
    fy = cfg.fy;
    cx = WIDTH * cfg.cx_ratio;
    cy = HEIGHT * cfg.cy_ratio;

    fx_new = cfg.fx_dist;
    fy_new = cfg.fy_dist;
    cx_new = cfg.cx_dist;
    cy_new = cfg.cy_dist;

    K1 = cfg.k1; K2 = cfg.k2; K3 = cfg.k3; P1 = cfg.p1; P2 = cfg.p2;

    cylindrical_H_fov_deg = cfg.cyl_h_fov;
    cylindrical_V_fov_deg = cfg.cyl_v_fov;

    panorama_crop_height_scale = cfg.pano_crop_h;
    panorama_crop_width_scale = cfg.pano_crop_w;
    panorama_crop_height_scale_q16 = cfg.pano_crop_h_q16;
    panorama_crop_width_scale_q16 = cfg.pano_crop_w_q16;

    resize_factor = cfg.resize_factor;
    resize_factor_q16 = cfg.resize_factor_q16;
    if (resize_factor_q16 == 0) resize_factor_q16 = (1 << 16);

    PITCH_TO_TRANSLATION_FACTOR_q16 = div_round_i64((int64_t)cfg.pitch_to_translation_factor_q16 << 16, resize_factor_q16);
    YAW_TO_TRANSLATION_FACTOR_q16 = cfg.yaw_to_translation_factor_q16;

    const float inv_resize_for_pitch = (std::fabs(resize_factor) > 1.0e-6f) ? (1.0f / resize_factor) : 1.0f;
    PITCH_TO_TRANSLATION_FACTOR = cfg.pitch_to_translation_factor * inv_resize_for_pitch;
    YAW_TO_TRANSLATION_FACTOR   = cfg.yaw_to_translation_factor;

    overlap_px = cfg.overlap_px;
    feather_strength = cfg.feather_strength;
    feather_strength_q16 = cfg.feather_strength_q16;

    TARGET_W = cfg.pano_width;
    TARGET_H = cfg.pano_height;
}

// ===================== Angle LUTs =====================
struct AngleLUTEntry { int16_t c; int16_t s; };
static std::vector<AngleLUTEntry> ANG_LUT;               // [-5,5] step 0.25 deg (legacy roll LUT)
static std::vector<AngleLUTEntry> ANG_CENTI_LUT;         // [0,35999] step 0.01 deg
static std::vector<int16_t> ATAN_RATIO_CENTI_LUT;        // atan(r), r in [0,1], centi-deg
static const int ATAN_RATIO_LUT_SIZE = 4096;

static inline void build_angle_lut() {
    if (!ANG_LUT.empty() && !ANG_CENTI_LUT.empty() && !ATAN_RATIO_CENTI_LUT.empty()) return;
    StageTimer _t(ST_LUT);
    if (ANG_LUT.empty()) {
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

    if (ANG_CENTI_LUT.empty()) {
        ANG_CENTI_LUT.resize(36000);
        for (int i = 0; i < 36000; ++i) {
            double deg = (double)i * 0.01;
            double rad = deg * M_PI / 180.0;
            int16_t c = (int16_t)std::lround(std::cos(rad) * 32767.0);
            int16_t s = (int16_t)std::lround(std::sin(rad) * 32767.0);
            ANG_CENTI_LUT[(size_t)i] = { c, s };
        }
    }

    if (ATAN_RATIO_CENTI_LUT.empty()) {
        ATAN_RATIO_CENTI_LUT.resize((size_t)ATAN_RATIO_LUT_SIZE + 1);
        for (int i = 0; i <= ATAN_RATIO_LUT_SIZE; ++i) {
            double ratio = (double)i / (double)ATAN_RATIO_LUT_SIZE;
            int16_t centi = (int16_t)std::lround(std::atan(ratio) * 18000.0 / M_PI);
            ATAN_RATIO_CENTI_LUT[(size_t)i] = centi;
        }
    }
}

static inline void get_cos_sin_q15(float deg, int16_t* c, int16_t* s) {
    if (ANG_LUT.empty()) build_angle_lut();
    float idxf = (deg + 5.0f) / 0.25f;
    int idx = (int)std::lround(idxf);
    if (idx < 0) idx = 0;
    if (idx >= (int)ANG_LUT.size()) idx = (int)ANG_LUT.size() - 1;
    *c = ANG_LUT[idx].c;
    *s = ANG_LUT[idx].s;
}

static inline int32_t wrap_centi_360(int32_t centi)
{
    int32_t v = centi % 36000;
    return (v < 0) ? (v + 36000) : v;
}

static inline void get_cos_sin_q15_centi(int32_t centi, int16_t* c, int16_t* s)
{
    if (ANG_CENTI_LUT.empty()) build_angle_lut();
    int32_t idx = wrap_centi_360(centi);
    const AngleLUTEntry& e = ANG_CENTI_LUT[(size_t)idx];
    *c = e.c;
    *s = e.s;
}

// ===================== Maps (Q12.4) =====================
static std::wstring sensor_prefix(bool isEO) { return isEO ? L"eo" : L"ir"; }

static void loadCylMapsQ(std::vector<int16_t>& mapx, std::vector<int16_t>& mapy,
    int w, int h, const std::filesystem::path& exePath, const std::wstring& prefix)
{
    StageTimer _t(ST_MAPS);
    mapx.resize((size_t)w * h);
    mapy.resize((size_t)w * h);
    auto px = exePath.parent_path() / (prefix + L"_map_x_q.bin");
    auto py = exePath.parent_path() / (prefix + L"_map_y_q.bin");
    FILE* fxp = fopen(px.string().c_str(), "rb");
    FILE* fyp = fopen(py.string().c_str(), "rb");
    if (!fxp || !fyp) {
        if (fxp) fclose(fxp);
        if (fyp) fclose(fyp);
        throw std::runtime_error("Cyl maps not found");
    }
    fread(mapx.data(), sizeof(int16_t), (size_t)w * h, fxp); MEM_READ(sizeof(int16_t) * (size_t)w * h); fclose(fxp);
    fread(mapy.data(), sizeof(int16_t), (size_t)w * h, fyp); MEM_READ(sizeof(int16_t) * (size_t)w * h); fclose(fyp);
    printf("[INFO] Cylindrical maps (Q12.4) loaded.\r\n");
}

static void saveCylMapsQ(const std::vector<int16_t>& mapx, const std::vector<int16_t>& mapy,
    const std::filesystem::path& exePath, const std::wstring& prefix)
{
    StageTimer _t(ST_MAPS);
    auto px = exePath.parent_path() / (prefix + L"_map_x_q.bin");
    auto py = exePath.parent_path() / (prefix + L"_map_y_q.bin");
    FILE* fxp = fopen(px.string().c_str(), "wb");
    FILE* fyp = fopen(py.string().c_str(), "wb");
    fwrite(mapx.data(), sizeof(int16_t), mapx.size(), fxp); MEM_WRITE(sizeof(int16_t) * mapx.size()); fclose(fxp);
    fwrite(mapy.data(), sizeof(int16_t), mapy.size(), fyp); MEM_WRITE(sizeof(int16_t) * mapy.size()); fclose(fyp);
    printf("[INFO] Cylindrical maps (Q12.4) saved.\r\n");
}

static void generateCylMapsQ(std::vector<int16_t>& mapx, std::vector<int16_t>& mapy,
    int w, int h, float fov_h_deg, float fov_v_deg, float x_shift,
    const std::filesystem::path& exePath, const std::wstring& prefix)
{
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

            mapx[(size_t)y * w + x] = (int16_t)qx;
            mapy[(size_t)y * w + x] = (int16_t)qy;
        }
    }
    saveCylMapsQ(mapx, mapy, exePath, prefix);
}

static std::filesystem::path undXPath(const std::filesystem::path& exe, const std::wstring& prefix) { return exe.parent_path() / (prefix + L"_und_x_q.bin"); }
static std::filesystem::path undYPath(const std::filesystem::path& exe, const std::wstring& prefix) { return exe.parent_path() / (prefix + L"_und_y_q.bin"); }

static bool loadUndistForwardMapsQ(std::vector<int16_t>& udx_q, std::vector<int16_t>& udy_q,
    int w, int h, const std::filesystem::path& exe, const std::wstring& prefix)
{
    StageTimer _t(ST_UNDISTMAP);
    auto px = undXPath(exe, prefix);
    auto py = undYPath(exe, prefix);
    FILE* fxp = fopen(px.string().c_str(), "rb");
    FILE* fyp = fopen(py.string().c_str(), "rb");
    if (!fxp || !fyp) {
        if (fxp) fclose(fxp);
        if (fyp) fclose(fyp);
        return false;
    }
    udx_q.resize((size_t)w * h);
    udy_q.resize((size_t)w * h);
    fread(udx_q.data(), sizeof(int16_t), (size_t)w * h, fxp); MEM_READ(sizeof(int16_t) * (size_t)w * h); fclose(fxp);
    fread(udy_q.data(), sizeof(int16_t), (size_t)w * h, fyp); MEM_READ(sizeof(int16_t) * (size_t)w * h); fclose(fyp);
    printf("[INFO] Forward undistort maps (Q12.4) loaded.\r\n");
    return true;
}

static void saveUndistForwardMapsQ(const std::vector<int16_t>& udx_q, const std::vector<int16_t>& udy_q,
    const std::filesystem::path& exe, const std::wstring& prefix)
{
    StageTimer _t(ST_UNDISTMAP);
    auto px = undXPath(exe, prefix);
    auto py = undYPath(exe, prefix);
    FILE* fxp = fopen(px.string().c_str(), "wb");
    FILE* fyp = fopen(py.string().c_str(), "wb");
    fwrite(udx_q.data(), sizeof(int16_t), udx_q.size(), fxp); MEM_WRITE(sizeof(int16_t) * udx_q.size()); fclose(fxp);
    fwrite(udy_q.data(), sizeof(int16_t), udy_q.size(), fyp); MEM_WRITE(sizeof(int16_t) * udy_q.size()); fclose(fyp);
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
        }
    }
    printf("[INFO] Forward undistort maps (Q12.4) built.\r\n");
}

static std::filesystem::path compXPath(const std::filesystem::path& exe, const std::wstring& prefix) { return exe.parent_path() / (prefix + L"_comp_x_q.bin"); }
static std::filesystem::path compYPath(const std::filesystem::path& exe, const std::wstring& prefix) { return exe.parent_path() / (prefix + L"_comp_y_q.bin"); }

static bool loadComposedMapsQ(std::vector<int16_t>& ox, std::vector<int16_t>& oy,
    int w, int h, const std::filesystem::path& exe, const std::wstring& prefix)
{
    StageTimer _t(ST_UNDISTMAP);
    auto px = compXPath(exe, prefix);
    auto py = compYPath(exe, prefix);
    FILE* fxp = fopen(px.string().c_str(), "rb");
    FILE* fyp = fopen(py.string().c_str(), "rb");
    if (!fxp || !fyp) {
        if (fxp) fclose(fxp);
        if (fyp) fclose(fyp);
        return false;
    }
    ox.resize((size_t)w * h);
    oy.resize((size_t)w * h);
    fread(ox.data(), sizeof(int16_t), (size_t)w * h, fxp); MEM_READ(sizeof(int16_t) * (size_t)w * h); fclose(fxp);
    fread(oy.data(), sizeof(int16_t), (size_t)w * h, fyp); MEM_READ(sizeof(int16_t) * (size_t)w * h); fclose(fyp);
    printf("[INFO] Composed maps loaded.\r\n");
    return true;
}

static void saveComposedMapsQ(const std::vector<int16_t>& ox, const std::vector<int16_t>& oy,
    const std::filesystem::path& exe, const std::wstring& prefix)
{
    StageTimer _t(ST_UNDISTMAP);
    auto px = compXPath(exe, prefix);
    auto py = compYPath(exe, prefix);
    FILE* fxp = fopen(px.string().c_str(), "wb");
    FILE* fyp = fopen(py.string().c_str(), "wb");
    fwrite(ox.data(), sizeof(int16_t), ox.size(), fxp); MEM_WRITE(sizeof(int16_t) * ox.size()); fclose(fxp);
    fwrite(oy.data(), sizeof(int16_t), oy.size(), fyp); MEM_WRITE(sizeof(int16_t) * oy.size()); fclose(fyp);
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
        }
    }
    printf("[INFO] Composed Cyl×Undist maps built (Q12.4).\r\n");
}

// ===================== IMU sensor -> camera =====================
static inline int32_t clamp_q15_from_q30(int64_t v_q30)
{
    int64_t q15 = (v_q30 + (1ll << 14)) >> 15;
    if (q15 > 32767) q15 = 32767;
    if (q15 < -32768) q15 = -32768;
    return (int32_t)q15;
}

static inline void matmul_q15(const int32_t A[3][3], const int32_t B[3][3], int32_t O[3][3])
{
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            int64_t acc_q30 =
                (int64_t)A[i][0] * B[0][j] +
                (int64_t)A[i][1] * B[1][j] +
                (int64_t)A[i][2] * B[2][j];
            O[i][j] = clamp_q15_from_q30(acc_q30);
        }
    }
}

static uint32_t isqrt_u64(uint64_t x)
{
    uint64_t res = 0;
    uint64_t bit = 1ull << 62;
    while (bit > x) bit >>= 2;
    while (bit != 0) {
        if (x >= res + bit) {
            x -= res + bit;
            res = (res >> 1) + bit;
        } else {
            res >>= 1;
        }
        bit >>= 2;
    }
    return (uint32_t)res;
}

static int32_t atan2_centi_q15(int32_t y_q15, int32_t x_q15)
{
    if (ATAN_RATIO_CENTI_LUT.empty()) build_angle_lut();
    if (y_q15 == 0 && x_q15 == 0) return 0;

    int64_t ay = std::llabs((long long)y_q15);
    int64_t ax = std::llabs((long long)x_q15);
    int32_t base = 0;

    if (ax == 0) {
        base = 9000;
    } else if (ay <= ax) {
        int idx = (int)((ay * ATAN_RATIO_LUT_SIZE + (ax / 2)) / ax);
        idx = clampi(idx, 0, ATAN_RATIO_LUT_SIZE);
        base = ATAN_RATIO_CENTI_LUT[(size_t)idx];
    } else {
        int idx = (int)((ax * ATAN_RATIO_LUT_SIZE + (ay / 2)) / ay);
        idx = clampi(idx, 0, ATAN_RATIO_LUT_SIZE);
        base = 9000 - ATAN_RATIO_CENTI_LUT[(size_t)idx];
    }

    if (x_q15 >= 0) return (y_q15 >= 0) ? base : -base;
    return (y_q15 >= 0) ? (18000 - base) : (base - 18000);
}

static int32_t asin_centi_q15(int32_t v_q15)
{
    int32_t v = clampi(v_q15, -32767, 32767);
    int64_t vv_q30 = (int64_t)v * (int64_t)v;
    int64_t one_q30 = (1ll << 30);
    int64_t rem_q30 = one_q30 - vv_q30;
    if (rem_q30 < 0) rem_q30 = 0;
    int32_t cos_q15 = (int32_t)isqrt_u64((uint64_t)rem_q30);
    return atan2_centi_q15(v, cos_q15);
}

static void convert_rpy_sensor_to_cam_centi(int32_t roll_sys1_centi, int32_t pitch_sys1_centi,
    int32_t yaw_offset_centi, int32_t pitch_offset_centi, int32_t roll_offset_centi,
    int32_t* roll_centi_out, int32_t* pitch_centi_out, int32_t* yaw_centi_out)
{
    int16_t c_rr, s_rr, c_pp, s_pp;
    int16_t c_ror, s_ror, c_por, s_por, c_yor, s_yor;

    get_cos_sin_q15_centi(roll_sys1_centi, &c_rr, &s_rr);
    get_cos_sin_q15_centi(pitch_sys1_centi, &c_pp, &s_pp);
    get_cos_sin_q15_centi(roll_offset_centi, &c_ror, &s_ror);
    get_cos_sin_q15_centi(pitch_offset_centi, &c_por, &s_por);
    get_cos_sin_q15_centi(yaw_offset_centi, &c_yor, &s_yor);

    const int32_t ONE_Q15 = 32767;
    int32_t Rx[3][3] = {
        { ONE_Q15, 0, 0 },
        { 0, c_rr, -s_rr },
        { 0, s_rr, c_rr }
    };
    int32_t Ry[3][3] = {
        { c_pp, 0, s_pp },
        { 0, ONE_Q15, 0 },
        { -s_pp, 0, c_pp }
    };

    int32_t Rr[3][3] = {
        { ONE_Q15, 0, 0 },
        { 0, c_ror, -s_ror },
        { 0, s_ror, c_ror }
    };
    int32_t Rp[3][3] = {
        { c_por, 0, s_por },
        { 0, ONE_Q15, 0 },
        { -s_por, 0, c_por }
    };
    int32_t Rz[3][3] = {
        { c_yor, -s_yor, 0 },
        { s_yor, c_yor, 0 },
        { 0, 0, ONE_Q15 }
    };

    int32_t R_sys1[3][3], Rt[3][3], Roff[3][3], R[3][3];
    matmul_q15(Rx, Ry, R_sys1);
    matmul_q15(Rz, Rp, Rt);
    matmul_q15(Rt, Rr, Roff);
    matmul_q15(R_sys1, Roff, R);

    int32_t pitch_centi = asin_centi_q15(-R[2][0]);

    int16_t c_pitch = 0, s_pitch = 0;
    get_cos_sin_q15_centi(pitch_centi, &c_pitch, &s_pitch);
    (void)s_pitch;

    int32_t roll_centi = 0;
    int32_t yaw_centi = 0;
    if (std::abs((int)c_pitch) > 8) {
        roll_centi = atan2_centi_q15(R[2][1], R[2][2]);
        yaw_centi = atan2_centi_q15(R[1][0], R[0][0]);
    } else {
        roll_centi = 0;
        yaw_centi = atan2_centi_q15(-R[0][1], R[1][1]);
    }

    if (roll_centi_out) *roll_centi_out = roll_centi;
    if (pitch_centi_out) *pitch_centi_out = pitch_centi;
    if (yaw_centi_out) *yaw_centi_out = yaw_centi;
}

// ===================== Build per-row base maps in ORIGINAL space =====================
static void buildBaseMapsOriginal(
    const std::vector<int16_t>& comp_x_q,
    const std::vector<int16_t>& comp_y_q,
    int resized_w, int resized_h,
    int crop_x0, int crop_y0,
    int crop_w, int crop_h,
    int per_cam_w_max,
    float resize_factor_local,
    std::vector<int32_t>& base_x_q16,
    std::vector<int32_t>& base_y_q16)
{
    StageTimer _t(ST_MAPS);

    base_x_q16.resize((size_t)per_cam_w_max * crop_h);
    base_y_q16.resize((size_t)per_cam_w_max * crop_h);

    const double sx = (double)crop_w / (double)per_cam_w_max;
    const float  inv_resize = 1.0f / resize_factor_local;
    const float  q16_scale = (float)(1 << WFIX_SHIFT);

    for (int y = 0; y < crop_h; ++y) {
        int src_y = clampi(crop_y0 + y, 0, resized_h - 1);
        const int16_t* rowX = &comp_x_q[(size_t)src_y * resized_w];
        const int16_t* rowY = &comp_y_q[(size_t)src_y * resized_w];

        for (int x = 0; x < per_cam_w_max; ++x) {
            double u = crop_x0 + (x + 0.5) * sx - 0.5;
            int iu0 = clampi((int)std::floor(u), 0, resized_w - 2);
            double tx = u - iu0;
            if (tx < 0.0) tx = 0.0;
            if (tx > 1.0) tx = 1.0;

            float v00x = rowX[iu0] / (float)MAP_SCALE;
            float v10x = rowX[iu0 + 1] / (float)MAP_SCALE;
            float src_x_r = v00x + (v10x - v00x) * (float)tx;

            float v00y = rowY[iu0] / (float)MAP_SCALE;
            float v10y = rowY[iu0 + 1] / (float)MAP_SCALE;
            float src_y_r = v00y + (v10y - v00y) * (float)tx;

            float src_x = src_x_r * inv_resize;
            float src_y = src_y_r * inv_resize;

            src_x = std::max(0.0f, std::min(src_x, (float)(WIDTH - 1)));
            src_y = std::max(0.0f, std::min(src_y, (float)(HEIGHT - 1)));

            int32_t qx16 = (int32_t)std::lround(src_x * q16_scale);
            int32_t qy16 = (int32_t)std::lround(src_y * q16_scale);

            base_x_q16[(size_t)y * per_cam_w_max + x] = qx16;
            base_y_q16[(size_t)y * per_cam_w_max + x] = qy16;
        }
    }

    printf("[INFO] Base maps in original space built (per_cam_w_max x crop_h, Q16.16).\r\n");
}

static std::filesystem::path baseXPath(const std::filesystem::path& exe, const std::wstring& prefix) { return exe.parent_path() / (prefix + L"_base_x_q16.bin"); }
static std::filesystem::path baseYPath(const std::filesystem::path& exe, const std::wstring& prefix) { return exe.parent_path() / (prefix + L"_base_y_q16.bin"); }

static bool loadBaseMapsQ16(std::vector<int32_t>& bx, std::vector<int32_t>& by,
    int per_cam_w_max, int crop_h,
    const std::filesystem::path& exe, const std::wstring& prefix)
{
    StageTimer _t(ST_MAPS);
    auto px = baseXPath(exe, prefix);
    auto py = baseYPath(exe, prefix);
    FILE* fxp = fopen(px.string().c_str(), "rb");
    FILE* fyp = fopen(py.string().c_str(), "rb");
    if (!fxp || !fyp) {
        if (fxp) fclose(fxp);
        if (fyp) fclose(fyp);
        return false;
    }
    size_t n = (size_t)per_cam_w_max * crop_h;
    bx.resize(n);
    by.resize(n);
    fread(bx.data(), sizeof(int32_t), n, fxp); MEM_READ(sizeof(int32_t) * n); fclose(fxp);
    fread(by.data(), sizeof(int32_t), n, fyp); MEM_READ(sizeof(int32_t) * n); fclose(fyp);
    printf("[INFO] Base maps (Q16.16) loaded from disk.\r\n");
    return true;
}

static void saveBaseMapsQ16(const std::vector<int32_t>& bx, const std::vector<int32_t>& by,
    const std::filesystem::path& exe, const std::wstring& prefix)
{
    StageTimer _t(ST_MAPS);
    auto px = baseXPath(exe, prefix);
    auto py = baseYPath(exe, prefix);
    FILE* fxp = fopen(px.string().c_str(), "wb");
    FILE* fyp = fopen(py.string().c_str(), "wb");
    fwrite(bx.data(), sizeof(int32_t), bx.size(), fxp); MEM_WRITE(sizeof(int32_t) * bx.size()); fclose(fxp);
    fwrite(by.data(), sizeof(int32_t), by.size(), fyp); MEM_WRITE(sizeof(int32_t) * by.size()); fclose(fyp);
    printf("[INFO] Base maps (Q16.16) saved to disk.\r\n");
}

// ===================== RowRun schedule =====================
#pragma pack(push, 1)
struct RowRun {
    uint16_t sy;
    uint16_t ox0;
    uint16_t len;
    int32_t  ax0_q16;
    int32_t  ay0_q16;
    int16_t  dax_q12_4;
    int16_t  day_q12_4;
};
#pragma pack(pop)

// ===================== 2-line BRAM cache =====================
struct LineCache {
    std::array<std::vector<unsigned char>, SCANLINE_BUFFER> Y;
    std::array<std::vector<unsigned char>, SCANLINE_BUFFER> U;
    std::array<std::vector<unsigned char>, SCANLINE_BUFFER> V;
    std::array<int, SCANLINE_BUFFER> rowId;
    int lineW = 0;
    int chromaW = 0;

    LineCache() {
        rowId.fill(-1);
        lineW = WIDTH;
        chromaW = WIDTH / 2;
        for (auto& buf : Y) buf.resize(FIXED_INPUT_W);
        for (auto& buf : U) buf.resize(FIXED_INPUT_W / 2);
        for (auto& buf : V) buf.resize(FIXED_INPUT_W / 2);
    }

    void loadRowPtr(int parity,
        const unsigned char* scanY,
        const unsigned char* scanU,
        const unsigned char* scanV,
        int row)
    {
        memcpy(Y[parity].data(), scanY, (size_t)lineW);
        memcpy(U[parity].data(), scanU, (size_t)chromaW);
        memcpy(V[parity].data(), scanV, (size_t)chromaW);
        rowId[parity] = row;
        MEM_WRITE((uint64_t)lineW + (uint64_t)chromaW + (uint64_t)chromaW);
    }
};

// ===================== Simple cyclic barrier =====================
struct SimpleBarrier {
    std::mutex m;
    std::condition_variable cv;
    int count = 0;
    int generation = 0;
    const int total;
    explicit SimpleBarrier(int total_) : total(total_) {}

    void wait() {
        std::unique_lock<std::mutex> lock(m);
        int gen = generation;
        if (++count == total) {
            generation++;
            count = 0;
            cv.notify_all();
        }
        else {
            cv.wait(lock, [&] { return gen != generation; });
        }
    }
};

// ===================== Row window buffers (O(1) slot allocation via freelist) =====================
struct RowWindow {
    int numCams = 0;
    int per_cam_h = 0;
    std::vector<int> per_cam_w;
    int fixed_w = FIXED_OUTPUT_W;
    int fixed_cw = FIXED_OUTPUT_W / 2;

    std::vector<int> sy_to_slot;     // [per_cam_h] -> slot or -1
    std::vector<int> slot_sy;        // [ACTIVE_BUFFER] -> sy or -1
    std::vector<int> free_slots;     // stack
    std::unique_ptr<std::atomic<int>[]> sy_next_cam;
    int sy_next_cam_size = 0;
    std::mutex mtx;

    std::vector<unsigned char> Y;
    std::vector<unsigned char> U;
    std::vector<unsigned char> V;

    void init(int nCams, int pch, const std::vector<int>& pcw) {
        numCams = nCams;
        per_cam_h = pch;
        per_cam_w = pcw;
        sy_to_slot.assign(per_cam_h, -1);
        slot_sy.assign(ACTIVE_BUFFER, -1);
        sy_next_cam_size = per_cam_h;
        sy_next_cam = std::make_unique<std::atomic<int>[]>(per_cam_h);
        for (int i = 0; i < sy_next_cam_size; ++i) {
            sy_next_cam[i].store(0, std::memory_order_relaxed);
        }

        free_slots.clear();
        free_slots.reserve(ACTIVE_BUFFER);
        for (int i = ACTIVE_BUFFER - 1; i >= 0; --i) free_slots.push_back(i);

        Y.assign((size_t)ACTIVE_BUFFER * fixed_w, 0);
        U.assign((size_t)ACTIVE_BUFFER * fixed_cw, 128);
        V.assign((size_t)ACTIVE_BUFFER * fixed_cw, 128);
    }

    void reset_mappings() {
        std::fill(sy_to_slot.begin(), sy_to_slot.end(), -1);
        std::fill(slot_sy.begin(), slot_sy.end(), -1);
        for (int i = 0; i < sy_next_cam_size; ++i) {
            sy_next_cam[i].store(0, std::memory_order_relaxed);
        }
        free_slots.clear();
        free_slots.reserve(ACTIVE_BUFFER);
        for (int i = ACTIVE_BUFFER - 1; i >= 0; --i) free_slots.push_back(i);
    }

    int allocate_slot_for_sy(int sy) {
        std::lock_guard<std::mutex> lock(mtx);
        if ((unsigned)sy >= (unsigned)per_cam_h) return -1;
        int& slot = sy_to_slot[sy];
        if (slot >= 0) return slot;

        if (free_slots.empty()) return -1;
        int freeSlot = free_slots.back();
        free_slots.pop_back();

        slot_sy[freeSlot] = sy;
        slot = freeSlot;

        unsigned char* y = &Y[(size_t)freeSlot * (size_t)fixed_w];
        unsigned char* u = &U[(size_t)freeSlot * (size_t)fixed_cw];
        unsigned char* v = &V[(size_t)freeSlot * (size_t)fixed_cw];
        memset(y, 0, (size_t)fixed_w);
        memset(u, 128, (size_t)fixed_cw);
        memset(v, 128, (size_t)fixed_cw);
        sy_next_cam[sy].store(0, std::memory_order_release);
        MEM_WRITE((uint64_t)fixed_w + 2ull * (uint64_t)fixed_cw);
        return freeSlot;
    }

    void free_sy(int sy) {
        std::lock_guard<std::mutex> lock(mtx);
        if ((unsigned)sy >= (unsigned)per_cam_h) return;
        int slot = sy_to_slot[sy];
        if (slot < 0) return;
        sy_to_slot[sy] = -1;
        slot_sy[slot] = -1;
        free_slots.push_back(slot);
    }

    void wait_for_cam_turn(int sy, int camId) {
        while (sy_next_cam[sy].load(std::memory_order_acquire) != camId) {
            std::this_thread::yield();
        }
    }

    void finish_cam(int sy) {
        sy_next_cam[sy].fetch_add(1, std::memory_order_release);
    }

    inline unsigned char* rowY(int slot) { return &Y[(size_t)slot * (size_t)fixed_w]; }
    inline unsigned char* rowU(int slot) { return &U[(size_t)slot * (size_t)fixed_cw]; }
    inline unsigned char* rowV(int slot) { return &V[(size_t)slot * (size_t)fixed_cw]; }
};

// ===================== On-the-fly mapping ax/ay (row-pointer based, less overhead) =====================
static inline bool compute_axay_for_dest_rowptr(
    int ox,
    int pcw,
    const int32_t* baseX_row, // points to base_x_q16[sy*per_cam_w_max]
    const int32_t* baseY_row,
    int32_t cx_q, int32_t cy_q,
    int32_t max_x_q, int32_t max_y_q,
    int32_t y_shift_q,
    int16_t c15, int16_t s15,
    int32_t& ax_q, int32_t& ay_q,
    int& y0_out,
    uint64_t& local_reads_bytes
) {
    if ((unsigned)ox >= (unsigned)pcw) return false;

    int32_t bx_q = baseX_row[ox];
    int32_t by_q = baseY_row[ox];
    local_reads_bytes += 8;

    by_q += y_shift_q;

    int32_t dx_q = bx_q - cx_q;
    int32_t dy_q = by_q - cy_q;

    int64_t tmpx = (int64_t)c15 * dx_q + (int64_t)s15 * dy_q;
    int64_t tmpy = -(int64_t)s15 * dx_q + (int64_t)c15 * dy_q;

    ax_q = (int32_t)(tmpx >> 15) + cx_q;
    ay_q = (int32_t)(tmpy >> 15) + cy_q;

    if (ax_q < 0 || ax_q > max_x_q || ay_q < 0 || ay_q > max_y_q) return false;

    int y0 = (ay_q >> WFIX_SHIFT);
    if (y0 < 0 || y0 >= HEIGHT - 1) return false;

    y0_out = y0;
    return true;
}

struct Y0SyIndex {
    int globalMin = 0;
    int rangeSize = 0;
    std::vector<int32_t> offsets;
    std::vector<uint16_t> sy;
};

struct RoiRect {
    int x0 = 0;
    int y0 = 0;
    int w = 0;
    int h = 0;
};

static RoiRect compute_undistort_roi(
    int w, int h,
    float fx_src, float fy_src, float cx_src, float cy_src,
    float fx_und, float fy_und, float cx_und, float cy_und,
    float k1, float k2, float k3, float p1, float p2)
{
    int minX = w;
    int maxX = -1;
    int minY = h;
    int maxY = -1;

    const float inv_fx_und = 1.0f / fx_und;
    const float inv_fy_und = 1.0f / fy_und;
    const float max_x = (float)(w - 1);
    const float max_y = (float)(h - 1);

    for (int v = 0; v < h; ++v) {
        const float yn = ((float)v - cy_und) * inv_fy_und;
        const float yn2 = yn * yn;
        const float p1_2yn = 2.0f * p1 * yn;
        const float p2_2 = 2.0f * p2;

        int rowMin = w;
        int rowMax = -1;
        for (int u = 0; u < w; ++u) {
            const float xn = ((float)u - cx_und) * inv_fx_und;
            const float xn2 = xn * xn;
            const float r2 = xn2 + yn2;
            const float r4 = r2 * r2;
            const float r6 = r4 * r2;

            const float radial = 1.0f + k1 * r2 + k2 * r4 + k3 * r6;
            const float dx = p1_2yn * xn + p2 * (r2 + 2.0f * xn2);
            const float dy = p1 * (r2 + 2.0f * yn2) + p2_2 * xn * yn;

            const float xd = xn * radial + dx;
            const float yd = yn * radial + dy;

            const float src_x = xd * fx_src + cx_src;
            const float src_y = yd * fy_src + cy_src;

            if (src_x < 0.0f || src_x > max_x || src_y < 0.0f || src_y > max_y) {
                continue;
            }

            rowMin = std::min(rowMin, u);
            rowMax = std::max(rowMax, u);
        }

        if (rowMax >= 0) {
            minY = std::min(minY, v);
            maxY = std::max(maxY, v);
            minX = std::min(minX, rowMin);
            maxX = std::max(maxX, rowMax);
        }
    }

    if (maxX < 0 || maxY < 0) {
        return { 0, 0, w, h };
    }

    int x0 = clampi(minX, 0, w - 1);
    int x1 = clampi(maxX, 0, w - 1);
    int y0 = clampi(minY, 0, h - 1);
    int y1 = clampi(maxY, 0, h - 1);

    if (x0 % 2 != 0) x0 = std::max(0, x0 - 1);
    if (((x1 - x0 + 1) % 2) != 0) {
        if (x1 + 1 < w) {
            x1++;
        } else if (x0 > 0) {
            x0--;
        } else {
            x1--;
        }
    }

    return { x0, y0, x1 - x0 + 1, y1 - y0 + 1 };
}

static void build_runs_for_y0_on_the_fly_rowptr(
    int y0_target,
    int pcw,
    int per_cam_h,
    int per_cam_w_max,
    const std::vector<int32_t>& base_x_q16,
    const std::vector<int32_t>& base_y_q16,
    int32_t cx_q, int32_t cy_q,
    int32_t max_x_q, int32_t max_y_q,
    int32_t y_shift_q,
    int16_t c15, int16_t s15,
    const std::vector<int16_t>& spanMin_sy,
    const std::vector<int16_t>& spanMax_sy,
    const Y0SyIndex& y0_to_sy_flat,
    std::vector<RowRun>& outRuns,
    size_t& outPeakBytes // updated by caller if desired
) {
    StageTimer _t(ST_SCHED);

    outRuns.clear();
    if (outRuns.capacity() < 4096) outRuns.reserve(4096);

    auto q_delta = [](int32_t dq16)->int16_t {
        int32_t v = (dq16 >= 0) ? (dq16 + (1 << (DRUN_TO_Q16 - 1))) : (dq16 - (1 << (DRUN_TO_Q16 - 1)));
        v >>= DRUN_TO_Q16;
        v = clampi(v, -32768, 32767);
        return (int16_t)v;
        };

    uint64_t local_reads = 0;

    // Use reverse index to find only sy rows that map to y0_target
    const uint16_t* syList = nullptr;
    int syCount = 0;
    if (!y0_to_sy_flat.offsets.empty()) {
        int localIdx = y0_target - y0_to_sy_flat.globalMin;
        if (localIdx >= 0 && localIdx < y0_to_sy_flat.rangeSize) {
            int off0 = y0_to_sy_flat.offsets[(size_t)localIdx];
            int off1 = y0_to_sy_flat.offsets[(size_t)localIdx + 1];
            syCount = off1 - off0;
            syList = y0_to_sy_flat.sy.data() + off0;
        }
    }

    for (int si = 0; si < syCount; ++si) {
        int sy = (int)syList[si];
        if (sy < 0 || sy >= per_cam_h) continue;

        const int32_t* rowBX = &base_x_q16[(size_t)sy * (size_t)per_cam_w_max];
        const int32_t* rowBY = &base_y_q16[(size_t)sy * (size_t)per_cam_w_max];

        int ox = 0;
        while (ox < pcw) {
            int32_t ax0, ay0;
            int y0c = -1;
            if (!compute_axay_for_dest_rowptr(
                ox, pcw, rowBX, rowBY,
                cx_q, cy_q, max_x_q, max_y_q,
                y_shift_q, c15, s15,
                ax0, ay0, y0c, local_reads))
            {
                ox++;
                continue;
            }
            if (y0c != y0_target) { ox++; continue; }

            int32_t ax1 = ax0, ay1 = ay0;
            int y0n = y0c;
            bool has1 = false;
            if (ox + 1 < pcw) {
                has1 = compute_axay_for_dest_rowptr(
                    ox + 1, pcw, rowBX, rowBY,
                    cx_q, cy_q, max_x_q, max_y_q,
                    y_shift_q, c15, s15,
                    ax1, ay1, y0n, local_reads);
                if (!has1 || y0n != y0_target) has1 = false;
            }

            int32_t dax_q16_exact = has1 ? (ax1 - ax0) : (1 << WFIX_SHIFT);
            int32_t day_q16_exact = has1 ? (ay1 - ay0) : 0;

            int16_t dax_q12_4 = q_delta(dax_q16_exact);
            int16_t day_q12_4 = q_delta(day_q16_exact);

            int start_ox = ox;
            int len = 1;

            int32_t dax_q16 = ((int32_t)dax_q12_4) << DRUN_TO_Q16;
            int32_t day_q16 = ((int32_t)day_q12_4) << DRUN_TO_Q16;

            for (int ox2 = ox + 1; ox2 < pcw; ++ox2) {
                int32_t axE, ayE;
                int y0E = -1;
                if (!compute_axay_for_dest_rowptr(
                    ox2, pcw, rowBX, rowBY,
                    cx_q, cy_q, max_x_q, max_y_q,
                    y_shift_q, c15, s15,
                    axE, ayE, y0E, local_reads))
                {
                    break;
                }
                if (y0E != y0_target) break;

                int i = ox2 - start_ox;
                int32_t axP = ax0 + (int32_t)((int64_t)i * dax_q16);
                int32_t ayP = ay0 + (int32_t)((int64_t)i * day_q16);

                if ((ayP >> WFIX_SHIFT) != y0_target) break;

                int32_t ex = axP - axE; if (ex < 0) ex = -ex;
                int32_t ey = ayP - ayE; if (ey < 0) ey = -ey;
                if (ex > RUN_TOL_Q16 || ey > RUN_TOL_Q16) break;

                len++;
            }

            RowRun rr{};
            rr.sy = (uint16_t)sy;
            rr.ox0 = (uint16_t)start_ox;
            rr.len = (uint16_t)len;
            rr.ax0_q16 = ax0;
            rr.ay0_q16 = ay0;
            rr.dax_q12_4 = dax_q12_4;
            rr.day_q12_4 = day_q12_4;
            outRuns.push_back(rr);

            ox += len;
        }
    }

    MEM_READ(local_reads);

    size_t bytes = outRuns.size() * sizeof(RowRun);
    if (bytes > outPeakBytes) outPeakBytes = bytes;
}

// ===================== Precompute y0 spans per (cam,sy) and a global maxY0Bound[sy] =====================
static void precompute_spans_and_bounds(
    int per_cam_h,
    int per_cam_w_max,
    const std::vector<int32_t>& base_x_q16,
    const std::vector<int32_t>& base_y_q16,
    const std::vector<int>& per_cam_w,
    const std::vector<int32_t>& y_shift_q_cam,
    const std::vector<int16_t>& c15_cam,
    const std::vector<int16_t>& s15_cam,
    int32_t cx_q, int32_t cy_q,
    int32_t max_x_q, int32_t max_y_q,
    std::vector<std::vector<int16_t>>& spanMin_cam_sy,
    std::vector<std::vector<int16_t>>& spanMax_cam_sy,
    std::vector<int>& maxY0Bound_sy,
    std::vector<Y0SyIndex>& y0_to_sy_cam
) {
    StageTimer _t(ST_SCHED);

    spanMin_cam_sy.assign((size_t)per_cam_w.size(), std::vector<int16_t>(per_cam_h, (int16_t)32767));
    spanMax_cam_sy.assign((size_t)per_cam_w.size(), std::vector<int16_t>(per_cam_h, (int16_t)-1));
    maxY0Bound_sy.assign(per_cam_h, -1);

    uint64_t local_reads = 0;

    for (int cam = 0; cam < (int)per_cam_w.size(); ++cam) {
        int pcw = per_cam_w[cam];
        int32_t y_shift_q = y_shift_q_cam[cam];
        int16_t c15 = c15_cam[cam];
        int16_t s15 = s15_cam[cam];

        for (int sy = 0; sy < per_cam_h; ++sy) {
            const int32_t* rowBX = &base_x_q16[(size_t)sy * (size_t)per_cam_w_max];
            const int32_t* rowBY = &base_y_q16[(size_t)sy * (size_t)per_cam_w_max];

            int mn = 1000000;
            int mx = -1000000;
            bool any = false;
            for (int ox = 0; ox < pcw; ++ox) {
                int32_t bx_q = rowBX[ox];
                int32_t by_q = rowBY[ox];
                local_reads += 8;

                by_q += y_shift_q;

                int32_t dx_q = bx_q - cx_q;
                int32_t dy_q = by_q - cy_q;

                int64_t tmpx = (int64_t)c15 * dx_q + (int64_t)s15 * dy_q;
                int64_t tmpy = -(int64_t)s15 * dx_q + (int64_t)c15 * dy_q;

                int32_t ax_q = (int32_t)(tmpx >> 15) + cx_q;
                int32_t ay_q = (int32_t)(tmpy >> 15) + cy_q;

                if (ax_q < 0 || ax_q > max_x_q || ay_q < 0 || ay_q > max_y_q) continue;

                int y0 = (ay_q >> WFIX_SHIFT);
                if (y0 < 0 || y0 >= HEIGHT - 1) continue;

                any = true;
                mn = std::min(mn, y0);
                mx = std::max(mx, y0);
            }

            if (!any) {
                spanMin_cam_sy[cam][sy] = (int16_t)32767;
                spanMax_cam_sy[cam][sy] = (int16_t)-1;
                continue;
            }

            spanMin_cam_sy[cam][sy] = (int16_t)clampi(mn, -32768, 32767);
            spanMax_cam_sy[cam][sy] = (int16_t)clampi(mx, -32768, 32767);

            if (mx > maxY0Bound_sy[sy]) maxY0Bound_sy[sy] = mx;
        }
    }

    MEM_READ(local_reads);

    // Build reverse index: for each cam, y0 -> list of sy rows whose span includes y0
    int numCams = (int)per_cam_w.size();
    y0_to_sy_cam.resize((size_t)numCams);
    for (int cam = 0; cam < numCams; ++cam) {
        // Find global y0 range for this cam
        int globalMin = INT32_MAX, globalMax = -1;
        for (int sy = 0; sy < per_cam_h; ++sy) {
            int16_t mn = spanMin_cam_sy[cam][sy];
            int16_t mx = spanMax_cam_sy[cam][sy];
            if (mx < 0) continue;
            if (mn < globalMin) globalMin = mn;
            if (mx > globalMax) globalMax = mx;
        }
        if (globalMax < 0) {
            y0_to_sy_cam[cam] = {};
            continue;
        }
        int rangeSize = globalMax - globalMin + 1;
        // Flat encoding: [rangeSize offset entries] [sy data...]
        // For simplicity, use a vector<vector<int16_t>> per cam keyed by y0
        // But that's too many vectors. Instead, store as a flat list with a start/count index.
        // Actually, use simple approach: vector<int16_t> with sentinel-separated lists per y0.
        // For efficiency, build count per y0, then flatten.
        std::vector<int> count((size_t)rangeSize, 0);
        for (int sy = 0; sy < per_cam_h; ++sy) {
            int16_t mn = spanMin_cam_sy[cam][sy];
            int16_t mx = spanMax_cam_sy[cam][sy];
            if (mx < 0) continue;
            for (int y0 = mn; y0 <= mx; ++y0) {
                if (y0 >= globalMin && y0 <= globalMax)
                    count[y0 - globalMin]++;
            }
        }
        // Build offset table: first rangeSize+1 entries are offsets, then data
        std::vector<int> offset((size_t)rangeSize + 1, 0);
        for (int i = 0; i < rangeSize; ++i)
            offset[i + 1] = offset[i] + count[i];
        int totalEntries = offset[rangeSize];

        // Flat storage: [globalMin(as int16), globalMax(as int16), rangeSize+1 offsets(as int16), totalEntries sy values(as int16)]
        // But offsets can exceed int16. Use int16 for sy values, pack offsets differently.
        // Simpler: store as flat int16_t vector: header + per-y0 blocks separated by -1 sentinel
        // Most efficient for the use case: just store (globalMin, rangeSize, offsets..., sy_data...)
        // Since this is PC simulation, use a straightforward flat vector.
        auto& flat = y0_to_sy_cam[cam];
        flat.globalMin = globalMin;
        flat.rangeSize = rangeSize;
        flat.offsets.assign(offset.begin(), offset.end());
        flat.sy.assign((size_t)totalEntries, 0);

        // Fill sy data
        std::vector<int> pos = offset; // write cursors
        for (int sy = 0; sy < per_cam_h; ++sy) {
            int16_t mn = spanMin_cam_sy[cam][sy];
            int16_t mx = spanMax_cam_sy[cam][sy];
            if (mx < 0) continue;
            for (int y0 = mn; y0 <= mx; ++y0) {
                if (y0 >= globalMin && y0 <= globalMax) {
                    int idx = y0 - globalMin;
                    flat.sy[(size_t)pos[idx]] = (uint16_t)sy;
                    pos[idx]++;
                }
            }
        }
    }
    printf("[INFO] Reverse index y0->sy built for %d cameras.\n", numCams);
}

// ===================== V10-like blending from row window into a destination row buffer =====================
static void blend_row_sy_into_dst(
    int sy_blend,
    int TARGET_W,
    int overlap_target,
    const std::vector<uint16_t>& alphaY,
    const std::vector<uint16_t>& alphaC,
    const std::vector<int>& per_cam_w,
    RowWindow& rw,
    unsigned char* dstYRow,
    unsigned char* dstURow,
    unsigned char* dstVRow
) {
    const int cw_out = TARGET_W / 2;

    std::fill(dstYRow, dstYRow + TARGET_W, (unsigned char)0);
    std::fill(dstURow, dstURow + cw_out, (unsigned char)128);
    std::fill(dstVRow, dstVRow + cw_out, (unsigned char)128);

    int slot = rw.sy_to_slot[sy_blend];
    if (slot < 0) return;

    StageTimer _b(ST_BLEND);
    const unsigned char* srcY = rw.rowY(slot);
    const unsigned char* srcU = rw.rowU(slot);
    const unsigned char* srcV = rw.rowV(slot);
    memcpy(dstYRow, srcY, (size_t)TARGET_W);
    memcpy(dstURow, srcU, (size_t)cw_out);
    memcpy(dstVRow, srcV, (size_t)cw_out);
    MEM_READ((uint64_t)TARGET_W + 2ull * (uint64_t)cw_out);
    MEM_WRITE((uint64_t)TARGET_W + 2ull * (uint64_t)cw_out);
}

// ===================== Internal memory report (FPGA model) =====================
static void print_internal_memory_report_v17(
    int numCams,
    int width,
    int targetW,
    int targetH,
    const std::vector<int>& per_cam_w,
    size_t peakScheduleBytes,
    int rowWindowH,
    int sliceH)
{
    const uint64_t BRAM36_BITS = 36ull * 1024ull;
    const uint64_t URAM_BITS = 288ull * 1024ull;

    uint64_t lineCacheBytesPerCam =
        (uint64_t)SCANLINE_BUFFER * ((uint64_t)FIXED_INPUT_W + (uint64_t)(FIXED_INPUT_W / 2) + (uint64_t)(FIXED_INPUT_W / 2));
    uint64_t lineCacheBytesTotal = lineCacheBytesPerCam * (uint64_t)numCams;

    uint64_t rowWindowBytes = (uint64_t)rowWindowH * (2ull * (uint64_t)FIXED_OUTPUT_W);

    uint64_t sliceBytesOne = 2ull * (uint64_t)FIXED_OUTPUT_W * (uint64_t)sliceH;
    uint64_t sliceBytesPingPong = 2ull * sliceBytesOne;

    uint64_t scheduleBytes = (uint64_t)peakScheduleBytes;
    uint64_t ddrPanoBytes = 2ull * (uint64_t)targetW * (uint64_t)targetH;

    uint64_t onChipTotalBytes = lineCacheBytesTotal + rowWindowBytes + sliceBytesPingPong + scheduleBytes;
    uint64_t onChipTotalBits = onChipTotalBytes * 8ull;

    uint64_t bram36Blocks = ceil_div_u64(onChipTotalBits, BRAM36_BITS);
    uint64_t uramBlocks = ceil_div_u64(onChipTotalBits, URAM_BITS);

    printf("\n===== INTERNAL MEMORY ESTIMATE (V19 MODEL) =====\n");
    printf("LineCache total: %llu bytes (%.2f KB) [per cam=%.2f KB]\n",
        (unsigned long long)lineCacheBytesTotal, lineCacheBytesTotal / 1024.0,
        lineCacheBytesPerCam / 1024.0);

    printf("RowWindow (ACTIVE_BUFFER=%d): %llu bytes (%.2f MB)\n",
        rowWindowH, (unsigned long long)rowWindowBytes, rowWindowBytes / (1024.0 * 1024.0));

    printf("Schedule (peak per-y0 RowRuns): %llu bytes (%.2f KB)\n",
        (unsigned long long)scheduleBytes, scheduleBytes / 1024.0);

    printf("Pano slice ping-pong (2x, sliceH=%d): %llu bytes (%.2f MB)\n",
        sliceH, (unsigned long long)sliceBytesPingPong, sliceBytesPingPong / (1024.0 * 1024.0));

    printf("ON-CHIP TOTAL: %llu bytes (%.2f MB)\n",
        (unsigned long long)onChipTotalBytes, onChipTotalBytes / (1024.0 * 1024.0));

    printf("Equivalent on-chip blocks:\n");
    printf("  BRAM36 ~ %llu blocks\n", (unsigned long long)bram36Blocks);
    printf("  URAM   ~ %llu blocks\n", (unsigned long long)uramBlocks);

    printf("\nExternal DDR (full pano frame): %llu bytes (%.2f MB)\n",
        (unsigned long long)ddrPanoBytes, ddrPanoBytes / (1024.0 * 1024.0));

    printf("==============================================\n");
}

// ===================== For DDR write slice ping-pong =====================
struct PanoSlicePingPong {
    int storageW = 0;
    int targetW = 0;
    int targetH = 0;
    int sliceH = 0;
    int storageCw = 0;
    int targetCw = 0;

    struct SliceBuf {
        std::vector<unsigned char> Y;
        std::vector<unsigned char> U;
        std::vector<unsigned char> V;
    };
    std::array<SliceBuf, 2> slice;

    int active = 0;
    int base_py = -1;
    int fill = 0;
    bool started = false;

    std::vector<unsigned char> ddrY;
    std::vector<unsigned char> ddrU;
    std::vector<unsigned char> ddrV;

    void init(int W, int H, int sliceH_) {
        targetW = W;
        targetH = H;
        sliceH = sliceH_;
        storageW = FIXED_OUTPUT_W;
        storageCw = storageW / 2;
        targetCw = targetW / 2;

        for (int i = 0; i < 2; ++i) {
            slice[i].Y.assign((size_t)storageW * sliceH, 0);
            slice[i].U.assign((size_t)storageCw * sliceH, 128);
            slice[i].V.assign((size_t)storageCw * sliceH, 128);
        }

        ddrY.assign((size_t)targetW * targetH, 0);
        ddrU.assign((size_t)targetCw * targetH, 128);
        ddrV.assign((size_t)targetCw * targetH, 128);

        active = 0;
        base_py = -1;
        fill = 0;
        started = false;
    }

    void clear_active() {
        std::fill(slice[active].Y.begin(), slice[active].Y.end(), (unsigned char)0);
        std::fill(slice[active].U.begin(), slice[active].U.end(), (unsigned char)128);
        std::fill(slice[active].V.begin(), slice[active].V.end(), (unsigned char)128);
        MEM_WRITE((uint64_t)slice[active].Y.size() + (uint64_t)slice[active].U.size() + (uint64_t)slice[active].V.size());
    }

    void flush_active_to_ddr() {
        if (!started || fill <= 0 || base_py < 0) return;

        StageTimer _t(ST_DDR);

        int rows = fill;
        if (base_py + rows > targetH) rows = targetH - base_py;
        if (rows <= 0) return;

        size_t yOff = (size_t)base_py * (size_t)targetW;
        size_t cOff = (size_t)base_py * (size_t)targetCw;
        const unsigned char* srcY = slice[active].Y.data();
        const unsigned char* srcU = slice[active].U.data();
        const unsigned char* srcV = slice[active].V.data();

        for (int r = 0; r < rows; ++r) {
            memcpy(ddrY.data() + yOff + (size_t)r * targetW, srcY + (size_t)r * storageW, (size_t)targetW);
            memcpy(ddrU.data() + cOff + (size_t)r * targetCw, srcU + (size_t)r * storageCw, (size_t)targetCw);
            memcpy(ddrV.data() + cOff + (size_t)r * targetCw, srcV + (size_t)r * storageCw, (size_t)targetCw);
        }

        MEM_WRITE((uint64_t)targetW * rows + 2ull * (uint64_t)targetCw * rows);

        active ^= 1;
        clear_active();
        started = false;
        base_py = -1;
        fill = 0;
    }

    bool begin_row(int py, unsigned char*& outY, unsigned char*& outU, unsigned char*& outV) {
        if (py < 0 || py >= targetH) return false;

        if (!started) {
            started = true;
            base_py = py;
            fill = 0;
        }

        if (py != base_py + fill) {
            flush_active_to_ddr();
            started = true;
            base_py = py;
            fill = 0;
        }

        if (fill >= sliceH) {
            flush_active_to_ddr();
            started = true;
            base_py = py;
            fill = 0;
        }

        outY = slice[active].Y.data() + (size_t)fill * (size_t)storageW;
        outU = slice[active].U.data() + (size_t)fill * (size_t)storageCw;
        outV = slice[active].V.data() + (size_t)fill * (size_t)storageCw;
        return true;
    }

    void end_row() {
        fill++;
        if (fill >= sliceH) flush_active_to_ddr();
    }

    void finalize() { flush_active_to_ddr(); }
};

// ===================== Build undistort base maps in ORIGINAL space (identity crop, 1:1) =====================
// For undistortion, the "composed map" is the undistortion forward map alone (no cylindrical).
// We build base_x_q16 / base_y_q16 of size (WIDTH x HEIGHT) in Q16.16, mapping each destination
// pixel (u,v) to its source coordinate.  Border replicate is applied.
// This feeds directly into the same RowRun / LineCache / RowWindow pipeline as panorama.
static void buildUndistBaseMapsOriginal(
    int w, int h,
    float fx_src, float fy_src, float cx_src, float cy_src,
    float fx_und, float fy_und, float cx_und, float cy_und,
    float k1, float k2, float k3, float p1, float p2,
    std::vector<int32_t>& base_x_q16,
    std::vector<int32_t>& base_y_q16)
{
    StageTimer _t(ST_UNDISTMAP);

    base_x_q16.resize((size_t)w * h);
    base_y_q16.resize((size_t)w * h);

    const float inv_fx_und = 1.0f / fx_und;
    const float inv_fy_und = 1.0f / fy_und;
    const float q16_scale = (float)(1 << WFIX_SHIFT);
    const float max_x = (float)(w - 1);
    const float max_y = (float)(h - 1);

    for (int v = 0; v < h; ++v) {
        const float yn = ((float)v - cy_und) * inv_fy_und;
        const float yn2 = yn * yn;
        const float p1_2yn = 2.0f * p1 * yn;
        const float p2_2 = 2.0f * p2;

        for (int u = 0; u < w; ++u) {
            const float xn = ((float)u - cx_und) * inv_fx_und;
            const float xn2 = xn * xn;
            const float r2 = xn2 + yn2;
            const float r4 = r2 * r2;
            const float r6 = r4 * r2;

            const float radial = 1.0f + k1 * r2 + k2 * r4 + k3 * r6;
            const float dx = p1_2yn * xn + p2 * (r2 + 2.0f * xn2);
            const float dy = p1 * (r2 + 2.0f * yn2) + p2_2 * xn * yn;

            const float xd = xn * radial + dx;
            const float yd = yn * radial + dy;

            float src_x = xd * fx_src + cx_src;
            float src_y = yd * fy_src + cy_src;

            // Border replicate
            if (src_x < 0.0f) src_x = 0.0f;
            if (src_y < 0.0f) src_y = 0.0f;
            if (src_x > max_x) src_x = max_x;
            if (src_y > max_y) src_y = max_y;

            size_t idx = (size_t)v * w + u;
            base_x_q16[idx] = (int32_t)std::lround(src_x * q16_scale);
            base_y_q16[idx] = (int32_t)std::lround(src_y * q16_scale);
        }
    }
    printf("[INFO] Undistort base maps built (Q16.16, %dx%d, border-replicate).\r\n", w, h);
}

// ===================== Mode Selection GUI =====================
static int g_selectedModeVal = 0x01;
static HWND g_hComboMode = NULL;
static bool g_modeDlgDone = false;

static LRESULT CALLBACK ModeDialogProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    switch (msg) {
    case WM_CREATE: {
        HFONT hFont = (HFONT)GetStockObject(DEFAULT_GUI_FONT);
        HWND hStat = CreateWindowW(L"STATIC", L"Select Output Mode:", WS_CHILD | WS_VISIBLE, 15, 15, 300, 20, hwnd, NULL, NULL, NULL);
        SendMessageW(hStat, WM_SETFONT, (WPARAM)hFont, FALSE);

        g_hComboMode = CreateWindowW(L"COMBOBOX", NULL, CBS_DROPDOWNLIST | WS_CHILD | WS_VISIBLE | WS_VSCROLL, 15, 40, 300, 300, hwnd, (HMENU)100, NULL, NULL);
        SendMessageW(g_hComboMode, WM_SETFONT, (WPARAM)hFont, FALSE);

        struct ModeOpt { int v; const wchar_t* n; };
        ModeOpt opts[] = {
            {0x00, L"0x00 : Blackout"},
            {0x01, L"0x01 : EO panorama + stabilization"},
            {0x02, L"0x02 : IR panorama + stabilization"},
            {0x03, L"0x03 : EO panorama (no stabilization)"},
            {0x06, L"0x06 : IR panorama (no stabilization)"},
            {0x07, L"0x07 : EO undistorted cam 0"},
            {0x08, L"0x08 : EO undistorted cam 1"},
            {0x09, L"0x09 : EO undistorted cam 2"},
            {0x0A, L"0x0A : EO undistorted cam 3"},
            {0x0B, L"0x0B : EO undistorted cam 4"},
            {0x0C, L"0x0C : EO undistorted cam 5"},
            {0x0D, L"0x0D : IR undistorted cam 0"},
            {0x0E, L"0x0E : IR undistorted cam 1"},
            {0x0F, L"0x0F : IR undistorted cam 2"},
            {0x10, L"0x10 : IR undistorted cam 3"},
            {0x11, L"0x11 : IR undistorted cam 4"},
            {0x12, L"0x12 : IR undistorted cam 5"},
            {0x13, L"0x13 : Status indication"}
        };

        for (auto& op : opts) {
            int idx = (int)SendMessageW(g_hComboMode, CB_ADDSTRING, 0, (LPARAM)op.n);
            SendMessageW(g_hComboMode, CB_SETITEMDATA, idx, op.v);
        }

        HWND hBtn = CreateWindowW(L"BUTTON", L"OK", WS_CHILD | WS_VISIBLE | BS_DEFPUSHBUTTON, 115, 80, 100, 30, hwnd, (HMENU)IDOK, NULL, NULL);
        SendMessageW(hBtn, WM_SETFONT, (WPARAM)hFont, FALSE);
        break;
    }
    case WM_COMMAND:
        if (LOWORD(wParam) == IDOK || LOWORD(wParam) == IDCANCEL) {
            if (LOWORD(wParam) == IDOK) {
                int idx = (int)SendMessageW(g_hComboMode, CB_GETCURSEL, 0, 0);
                if (idx != CB_ERR) {
                    g_selectedModeVal = (int)SendMessageW(g_hComboMode, CB_GETITEMDATA, idx, 0);
                }
            }
            g_modeDlgDone = true;
            DestroyWindow(hwnd);
        }
        break;
    case WM_DESTROY:
        g_modeDlgDone = true;
        PostThreadMessage(GetCurrentThreadId(), WM_NULL, 0, 0);
        break;
    case WM_CLOSE:
        DestroyWindow(hwnd);
        break;
    default:
        return DefWindowProc(hwnd, msg, wParam, lParam);
    }
    return 0;
}

static int PromptUserForModeDialog(int defaultMode) {
    WNDCLASSW wc = { 0 };
    wc.lpfnWndProc = ModeDialogProc;
    wc.hInstance = GetModuleHandle(NULL);
    wc.lpszClassName = L"ModeSelectDlgClass";
    wc.hbrBackground = (HBRUSH)(COLOR_BTNFACE + 1);
    RegisterClassW(&wc);

    g_modeDlgDone = false;
    g_selectedModeVal = defaultMode;

    int width = 350;
    int height = 160;
    int screenW = GetSystemMetrics(SM_CXSCREEN);
    int screenH = GetSystemMetrics(SM_CYSCREEN);

    HWND hwnd = CreateWindowExW(WS_EX_DLGMODALFRAME | WS_EX_TOPMOST, wc.lpszClassName, L"Select Output Mode",
        WS_POPUP | WS_CAPTION | WS_SYSMENU | WS_VISIBLE,
        (screenW - width) / 2, (screenH - height) / 2, width, height,
        NULL, NULL, wc.hInstance, NULL);

    if (hwnd) {
        int count = (int)SendMessageW(g_hComboMode, CB_GETCOUNT, 0, 0);
        bool found = false;
        for (int i = 0; i < count; i++) {
            if ((int)SendMessageW(g_hComboMode, CB_GETITEMDATA, i, 0) == defaultMode) {
                SendMessageW(g_hComboMode, CB_SETCURSEL, i, 0);
                found = true;
                break;
            }
        }
        if (!found && count > 0) SendMessageW(g_hComboMode, CB_SETCURSEL, 1, 0); // fallback
        SetFocus(g_hComboMode);
    }

    MSG msg;
    while (!g_modeDlgDone && GetMessage(&msg, NULL, 0, 0)) {
        if (!IsDialogMessage(hwnd, &msg)) {
            TranslateMessage(&msg);
            DispatchMessage(&msg);
        }
    }
    UnregisterClassW(wc.lpszClassName, wc.hInstance);
    return g_selectedModeVal;
}

// ===================== Main =====================
int main(int argc, char** argv) {
    HRESULT hrCoInit = CoInitializeEx(NULL, COINIT_APARTMENTTHREADED | COINIT_DISABLE_OLE1DDE);
    try {
        build_angle_lut();

        wchar_t wpath[MAX_PATH] = L"";
        GetModuleFileNameW(NULL, wpath, MAX_PATH);
        std::filesystem::path exePath(wpath);

        printf("[PATH] exe        : %s\n", exePath.string().c_str());

        std::string userIniStr = PromptUserForFile("Select Parameters INI File", "INI Files (*.ini)\0*.ini\0All Files (*.*)\0*.*\0");
        if (userIniStr.empty()) {
            printf("[FATAL] No INI file selected. Exiting.\n");
            return 1;
        }
        std::filesystem::path paramsIniPath(StringToWString(userIniStr));

        if (!file_exists(paramsIniPath)) {
            printf("[FATAL] Selected parameters INI file could not be found.\n");
            return 1;
        }

        printf("[PATH] params ini : %s\n", paramsIniPath.string().c_str());

        // Keep standard fallback paths for imu configs if needed by the code
        std::filesystem::path imuIniPath = paramsIniPath.parent_path() / L"imu.ini";
        bool imuIniAvailable = file_exists(imuIniPath);

        // Read parameters (unified file)
        INIManager paramIni(paramsIniPath);

        NUM_FRAMES = paramIni.ReadInteger(L"output", L"num_frames", NUM_FRAMES);
        SAVE_EVERY_K = paramIni.ReadInteger(L"output", L"save_every_k", SAVE_EVERY_K);

        int modeVal = paramIni.ReadInteger(L"output", L"mode", MODE_EO_PANO_STAB);

        modeVal = PromptUserForModeDialog(modeVal);
        printf("[INFO] Selected Option Mode from GUI: 0x%02X\n", modeVal);

        OutputMode mode = (OutputMode)modeVal;

        bool isEO = true;
        bool isPanorama = true;
        int camIdx = -1;
        switch (mode) {
        case MODE_EO_PANO_STAB:
        case MODE_EO_PANO_NO_STAB:
            isEO = true; isPanorama = true; break;
        case MODE_IR_PANO_STAB:
        case MODE_IR_PANO_NO_STAB:
            isEO = false; isPanorama = true; break;
        default:
            if (mode >= MODE_EO_UNDIST_CAM1 && mode <= MODE_EO_UNDIST_CAM6) { isEO = true; isPanorama = false; camIdx = mode - MODE_EO_UNDIST_CAM1; break; }
            if (mode >= MODE_IR_UNDIST_CAM1 && mode <= MODE_IR_UNDIST_CAM6) { isEO = false; isPanorama = false; camIdx = mode - MODE_IR_UNDIST_CAM1; break; }
            if (mode == MODE_BLACKOUT || mode == MODE_STATUS) { isPanorama = true; isEO = true; break; }
            break;
        }

        CameraConfig camCfg = loadCameraConfig(paramIni, isEO, exePath.parent_path());
        applyCameraConfig(camCfg);

        if (WIDTH > FIXED_INPUT_W) {
            printf("[FATAL] WIDTH=%d exceeds FIXED_INPUT_W=%d\n", WIDTH, FIXED_INPUT_W);
            return 1;
        }
        if (TARGET_W > FIXED_OUTPUT_W) {
            printf("[FATAL] TARGET_W=%d exceeds FIXED_OUTPUT_W=%d\n", TARGET_W, FIXED_OUTPUT_W);
            return 1;
        }

        // IMU / stabilization - read from sensor-specific section for test data
        // (In real-time operation, there's one IMU for both sensors, but test images were captured at different orientations)
        const wchar_t* orientSec = isEO ? L"eo_orientation" : L"ir_orientation";
        
        // Mode dictates stabilization entirely now
        bool isStabMode = (mode == MODE_EO_PANO_STAB || mode == MODE_IR_PANO_STAB);
        
        if (isStabMode) {
            roll_Sensor_centi = readAngleCentiWithFallback(paramIni, orientSec, L"Sensor_Orientation", L"roll", 0);
            pitch_Sensor_centi = readAngleCentiWithFallback(paramIni, orientSec, L"Sensor_Orientation", L"pitch", 0);
            yaw_Sensor_centi = readAngleCentiWithFallback(paramIni, orientSec, L"Sensor_Orientation", L"yaw", 0);
        }
        else {
            roll_Sensor_centi = pitch_Sensor_centi = yaw_Sensor_centi = 0;
            printf("[ORIENTATION] Stabilization disabled by mode\n");
        }
        
        bool StabilizationIni = isStabMode; // assigned for informational print log alone

        // IMU ini (for compatibility: images/offset fallback)
        std::unique_ptr<INIManager> imuIniPtr;
        if (imuIniAvailable) {
            imuIniPtr = std::make_unique<INIManager>(imuIniPath);
        }

        // Input files - Prompt selection
        std::vector<std::string> filenames;
        while (true) {
            filenames.clear();
            if (isPanorama) {
                // Request images required based on INI configs (usually stitched together per frame)
                int required_images = NUM_CAMS;
                char titleBuf[128];
                snprintf(titleBuf, sizeof(titleBuf), "Select exactly %d Images for Panorama", required_images);
                filenames = PromptUserForMultipleFiles(titleBuf, "Image Files (*.bmp;*.png;*.jpg)\0*.bmp;*.png;*.jpg\0All Files (*.*)\0*.*\0");
                
                if (filenames.empty()) {
                    printf("[FATAL] Image selection cancelled.\n");
                    return 1;
                }
                
                if (filenames.size() < (size_t)required_images) {
                    MessageBoxA(NULL, "Fewer images were selected than required for the panorama stitch. Please select all required images, or modify your INI config.", "Insufficient Images", MB_ICONERROR | MB_OK);
                    continue;
                }

                // If more selected manually by mistake, resize to exact required count
                filenames.resize(required_images);
            } else {
                std::string p = PromptUserForFile("Select Image for Selected Camera Test", "Image Files (*.bmp;*.png;*.jpg)\0*.bmp;*.png;*.jpg\0All Files (*.*)\0*.*\0");
                if (p.empty()) {
                    printf("[FATAL] Image selection cancelled.\n");
                    return 1;
                }
                filenames.push_back(p);
            }
            break;
        }

        for (size_t i = 0; i < filenames.size(); ++i) {
            printf("[Selected] Camera %zu: %s\n", i + 1, filenames[i].c_str());
        }

        std::vector<int32_t> yaw_off = camCfg.yaw_off;
        std::vector<int32_t> pit_off = camCfg.pitch_off;
        std::vector<int32_t> rol_off = camCfg.roll_off;
        if (yaw_off.empty() && imuIniPtr) yaw_off = readAngleArrayCenti(*imuIniPtr, L"offset", L"yaw");
        if (pit_off.empty() && imuIniPtr) pit_off = readAngleArrayCenti(*imuIniPtr, L"offset", L"pitch");
        if (rol_off.empty() && imuIniPtr) rol_off = readAngleArrayCenti(*imuIniPtr, L"offset", L"roll");

        // Clamp logic: compute maximum per-camera angle, scaling IMU sensor data if > 5.0
        if (roll_Sensor_centi != 0 || pitch_Sensor_centi != 0 || yaw_Sensor_centi != 0) {
            int32_t max_cam_angle_centi = 0;
            for (int cam = 0; cam < NUM_CAMS; ++cam) {
                int32_t r_cam_c = 0, p_cam_c = 0, y_cam_c = 0;
                convert_rpy_sensor_to_cam_centi(
                    roll_Sensor_centi, pitch_Sensor_centi,
                    (cam < (int)yaw_off.size() ? yaw_off[cam] : 0),
                    (cam < (int)pit_off.size() ? pit_off[cam] : 0),
                    (cam < (int)rol_off.size() ? rol_off[cam] : 0),
                    &r_cam_c, &p_cam_c, &y_cam_c
                );
                (void)y_cam_c;
                max_cam_angle_centi = std::max({ max_cam_angle_centi, std::abs(r_cam_c), std::abs(p_cam_c) });
            }

            if (max_cam_angle_centi > 500) {
                roll_Sensor_centi = div_round_i64((int64_t)roll_Sensor_centi * 500, max_cam_angle_centi);
                pitch_Sensor_centi = div_round_i64((int64_t)pitch_Sensor_centi * 500, max_cam_angle_centi);
                yaw_Sensor_centi = div_round_i64((int64_t)yaw_Sensor_centi * 500, max_cam_angle_centi);
                printf("[ORIENTATION] Max camera angle %d.%02d > 5.0. Scaled global sensor by %d/%d\n",
                    max_cam_angle_centi / 100, std::abs(max_cam_angle_centi % 100), 500, max_cam_angle_centi);
            }
        }

        printf("[ORIENTATION] Final Sensor: roll=%.2f pitch=%.2f yaw=%.2f stabilization=%s\n",
               centi_to_deg(roll_Sensor_centi), centi_to_deg(pitch_Sensor_centi), centi_to_deg(yaw_Sensor_centi),
               StabilizationIni ? "true" : "false");

        logln("........Processing (MT-V19 FPGA Model: ON-FLY y0 buckets (parallel) + RowWindow + 2-Line Cache + DDR Slice PingPong)........");

        std::wstring captureDirW = CAPTURE_DIR_W;
        if (SAVE_EVERY_K == 1) {
            std::string userDir = PromptUserForFolder("Select Output Save Directory");
            if (!userDir.empty()) {
                captureDirW = StringToWString(userDir);
            }
        }
        CreateDirectoryW(captureDirW.c_str(), NULL);

        // Geometry in resized space
        const int resized_w = div_round_i64((int64_t)WIDTH * resize_factor_q16, 1ll << 16);
        const int resized_h = div_round_i64((int64_t)HEIGHT * resize_factor_q16, 1ll << 16);
        const int crop_w = div_round_i64((int64_t)resized_w * panorama_crop_width_scale_q16, 1ll << 16);
        const int crop_h = div_round_i64((int64_t)resized_h * panorama_crop_height_scale_q16, 1ll << 16);
        const int crop_x0 = (resized_w - crop_w) / 2;
        const int crop_y0 = (resized_h - crop_h) / 2;

        std::wstring mapPrefix = sensor_prefix(isEO);

        // Maps (ONCE)
        std::vector<int16_t> cyl_x_q, cyl_y_q;
        try { loadCylMapsQ(cyl_x_q, cyl_y_q, resized_w, resized_h, exePath, mapPrefix); }
        catch (...) { generateCylMapsQ(cyl_x_q, cyl_y_q, resized_w, resized_h, cylindrical_H_fov_deg, cylindrical_V_fov_deg, 0.0f, exePath, mapPrefix); }

        std::vector<int16_t> und_x_q, und_y_q;
        if (!loadUndistForwardMapsQ(und_x_q, und_y_q, resized_w, resized_h, exePath, mapPrefix)) {
            generateUndistForwardMapsQ(und_x_q, und_y_q, resized_w, resized_h,
                fx, fy, cx, cy, fx_new, fy_new, cx_new, cy_new,
                K1, K2, K3, P1, P2, resize_factor);
            saveUndistForwardMapsQ(und_x_q, und_y_q, exePath, mapPrefix);
        }

        std::vector<int16_t> comp_x_q, comp_y_q;
        if (!loadComposedMapsQ(comp_x_q, comp_y_q, resized_w, resized_h, exePath, mapPrefix)) {
            composeCylWithUndistQ(cyl_x_q, cyl_y_q, und_x_q, und_y_q, resized_w, resized_h, comp_x_q, comp_y_q);
            saveComposedMapsQ(comp_x_q, comp_y_q, exePath, mapPrefix);
        }

        // Load all camera frames and convert to YUV422 once (KEEP contiguous buffers; no cam_scan vectors)
        std::vector<YUV422Image*> framesYUV(NUM_CAMS, nullptr);
        if (isPanorama) {
            for (int i = 0; i < NUM_CAMS; ++i) {
                Image* tmp = readBMP(filenames[i].c_str());
                if (!tmp) { printf("[ERROR] Failed to load camera %d: %s\n", i, filenames[i].c_str()); return 1; }
                printf("[INFO] Camera %d (RGB): %dx%d\n", i, tmp->width, tmp->height);

                framesYUV[i] = rgbToYUV422(tmp);
                freeImage(tmp);
                if (!framesYUV[i]) { printf("[ERROR] Failed to convert camera %d to YUV422\n", i); return 1; }
                printf("[INFO] Camera %d converted to YUV422: %dx%d\n", i, framesYUV[i]->width, framesYUV[i]->height);

                if (framesYUV[i]->width != WIDTH || framesYUV[i]->height != HEIGHT) {
                    printf("[FATAL] Camera %d dimensions mismatch. Expected %dx%d, got %dx%d\n",
                        i, WIDTH, HEIGHT, framesYUV[i]->width, framesYUV[i]->height);
                    return 1;
                }
            }
        } else {
            if (camIdx >= 0 && camIdx < NUM_CAMS && !filenames.empty()) {
                Image* tmp = readBMP(filenames[0].c_str());
                if (!tmp) { printf("[ERROR] Failed to load camera: %s\n", filenames[0].c_str()); return 1; }
                printf("[INFO] Camera %d (RGB): %dx%d\n", camIdx, tmp->width, tmp->height);

                framesYUV[camIdx] = rgbToYUV422(tmp);
                freeImage(tmp);
                if (!framesYUV[camIdx]) { printf("[ERROR] Failed to convert camera to YUV422\n"); return 1; }
                printf("[INFO] Camera %d converted to YUV422: %dx%d\n", camIdx, framesYUV[camIdx]->width, framesYUV[camIdx]->height);

                if (framesYUV[camIdx]->width != WIDTH || framesYUV[camIdx]->height != HEIGHT) {
                    printf("[FATAL] Camera %d dimensions mismatch. Expected %dx%d, got %dx%d\n",
                        camIdx, WIDTH, HEIGHT, framesYUV[camIdx]->width, framesYUV[camIdx]->height);
                    return 1;
                }
            } else {
                printf("[FATAL] Invalid camera index %d or missing files.\n", camIdx);
                return 1;
            }
        }

        // Individual (undistorted) modes - uses the SAME panorama HW pipeline:
        //   LineCache (2-line BRAM) → RowRun schedule → RowWindow → PanoSlicePingPong → DDR
        // Differences from panorama: numCams=1, no cylindrical, no feather blend, output=WIDTHxHEIGHT
        if (!isPanorama) {
            bool camEO = isEO;

            if (camIdx < 0 || camIdx >= NUM_CAMS) {
                printf("[FATAL] Unsupported individual mode 0x%02X\n", modeVal);
                for (auto* fimg : framesYUV) freeYUV422ImageHeap(fimg);
                return 1;
            }

            // ---- Build undistort base maps (same format as panorama base_x_q16/base_y_q16) ----
            // For undistortion: per_cam_w = {WIDTH}, per_cam_h = HEIGHT, numCams = 1
            // No crop, no resize, no cylindrical — just the undistortion forward map.
            const int und_num_cams = 1;
            const int und_per_cam_w = WIDTH;
            const int und_per_cam_h = HEIGHT;
            const int und_per_cam_w_max = WIDTH;

            std::vector<int32_t> und_base_x_q16, und_base_y_q16;
            buildUndistBaseMapsOriginal(
                WIDTH, HEIGHT,
                fx, fy, cx, cy,
                fx_new, fy_new, cx_new, cy_new,
                K1, K2, K3, P1, P2,
                und_base_x_q16, und_base_y_q16);

            // ---- Precompute spans and reverse index (same as panorama) ----
            const int32_t und_cx_q = (int32_t)((int64_t)WIDTH << (WFIX_SHIFT - 1));
            const int32_t und_cy_q = (int32_t)((int64_t)HEIGHT << (WFIX_SHIFT - 1));
            const int32_t und_max_x_q = (int32_t)((WIDTH - 2) << WFIX_SHIFT);
            const int32_t und_max_y_q = (int32_t)((HEIGHT - 2) << WFIX_SHIFT);

            // No stabilization for undistort: identity rotation, no y-shift
            std::vector<int32_t> und_y_shift_q_cam(und_num_cams, 0);
            std::vector<int16_t> und_c15_cam(und_num_cams, 32767); // cos(0) = 1.0 in Q1.15
            std::vector<int16_t> und_s15_cam(und_num_cams, 0);     // sin(0) = 0
            std::vector<int> und_per_cam_w_vec = { und_per_cam_w };

            std::vector<std::vector<int16_t>> und_spanMin_cam_sy, und_spanMax_cam_sy;
            std::vector<int> und_maxY0Bound_sy;
            std::vector<Y0SyIndex> und_y0_to_sy_cam;

            precompute_spans_and_bounds(
                und_per_cam_h, und_per_cam_w_max,
                und_base_x_q16, und_base_y_q16,
                und_per_cam_w_vec,
                und_y_shift_q_cam, und_c15_cam, und_s15_cam,
                und_cx_q, und_cy_q, und_max_x_q, und_max_y_q,
                und_spanMin_cam_sy, und_spanMax_cam_sy,
                und_maxY0Bound_sy, und_y0_to_sy_cam);

            RoiRect und_roi = compute_undistort_roi(
                WIDTH, HEIGHT,
                fx, fy, cx, cy,
                fx_new, fy_new, cx_new, cy_new,
                K1, K2, K3, P1, P2);

            printf("[UNDIST ROI] x0=%d y0=%d w=%d h=%d\n", und_roi.x0, und_roi.y0, und_roi.w, und_roi.h);

            // ---- Allocate panorama HW structures ----
            // RowWindow: 1 camera, per_cam_h = HEIGHT destination rows
            RowWindow und_rowWindow;
            und_rowWindow.init(und_num_cams, und_per_cam_h, und_per_cam_w_vec);

            const int und_out_w = und_roi.w;
            const int und_out_h = und_roi.h;

            // PanoSlicePingPong: output is cropped to valid ROI
            PanoSlicePingPong und_panoOut;
            und_panoOut.init(und_out_w, und_out_h, PING_PONG_PUSH_BUFFER);
            und_panoOut.clear_active();

            // Worker thread (1 camera) + barrier
            SimpleBarrier und_barrier(und_num_cams + 1); // 1 worker + 1 main
            std::atomic<bool> und_shutdownFlag(false);
            std::atomic<int> und_current_y0(0);
            std::atomic<size_t> und_peakScheduleBytes(0);
            std::atomic<uint64_t> und_cache_ok(0), und_cache_violation(0);

            const bool und_use_nn = camCfg.use_nearest_neighbor;

            std::thread und_worker([&]() {
                LineCache cache;
                std::vector<RowRun> localRuns;
                size_t localPeakBytes = 0;

                while (true) {
                    und_barrier.wait(); // start y0

                    if (und_shutdownFlag.load(std::memory_order_acquire)) {
                        und_barrier.wait();
                        break;
                    }

                    int y0 = und_current_y0.load(std::memory_order_relaxed);
                    if (y0 < 0 || y0 >= HEIGHT - 1) {
                        und_barrier.wait();
                        continue;
                    }

                    // 1) Build schedule (same as panorama worker)
                    build_runs_for_y0_on_the_fly_rowptr(
                        y0,
                        und_per_cam_w,
                        und_per_cam_h,
                        und_per_cam_w_max,
                        und_base_x_q16, und_base_y_q16,
                        und_cx_q, und_cy_q, und_max_x_q, und_max_y_q,
                        und_y_shift_q_cam[0],
                        und_c15_cam[0], und_s15_cam[0],
                        und_spanMin_cam_sy[0], und_spanMax_cam_sy[0],
                        und_y0_to_sy_cam[0],
                        localRuns,
                        localPeakBytes);

                    {
                        size_t cur = und_peakScheduleBytes.load(std::memory_order_relaxed);
                        while (localPeakBytes > cur &&
                            !und_peakScheduleBytes.compare_exchange_weak(cur, localPeakBytes, std::memory_order_relaxed)) {}
                    }

                    // 2) Render: load 2-line strip into LineCache, scatter into RowWindow
                    StageTimer _r(ST_RENDER);

                    const unsigned char* Y0 = framesYUV[camIdx]->Y + (size_t)y0 * (size_t)WIDTH;
                    const unsigned char* U0 = framesYUV[camIdx]->U + (size_t)y0 * (size_t)(WIDTH / 2);
                    const unsigned char* V0 = framesYUV[camIdx]->V + (size_t)y0 * (size_t)(WIDTH / 2);
                    const unsigned char* Y1 = framesYUV[camIdx]->Y + (size_t)(y0 + 1) * (size_t)WIDTH;
                    const unsigned char* U1 = framesYUV[camIdx]->U + (size_t)(y0 + 1) * (size_t)(WIDTH / 2);
                    const unsigned char* V1 = framesYUV[camIdx]->V + (size_t)(y0 + 1) * (size_t)(WIDTH / 2);

                    cache.loadRowPtr(0, Y0, U0, V0, y0);
                    cache.loadRowPtr(1, Y1, U1, V1, y0 + 1);

                    const int cwCam = und_per_cam_w / 2;
                    uint64_t okLocal = 0, vioLocal = 0;

                    for (const RowRun& r : localRuns) {
                        if (r.sy >= (uint16_t)und_per_cam_h) continue;

                        int slot = und_rowWindow.allocate_slot_for_sy((int)r.sy);
                        if (slot < 0) continue;

                        unsigned char* dstY = und_rowWindow.rowY(slot);
                        unsigned char* dstU = und_rowWindow.rowU(slot);
                        unsigned char* dstV = und_rowWindow.rowV(slot);

                        int32_t ax0 = r.ax0_q16;
                        int32_t ay0 = r.ay0_q16;
                        int32_t dax = ((int32_t)r.dax_q12_4) << DRUN_TO_Q16;
                        int32_t day = ((int32_t)r.day_q12_4) << DRUN_TO_Q16;

                        int ox0 = (int)r.ox0;
                        int len = (int)r.len;
                        if (ox0 < 0 || ox0 >= und_per_cam_w) continue;
                        if (len <= 0) continue;
                        if (ox0 + len > und_per_cam_w) len = und_per_cam_w - ox0;

                        for (int i = 0; i < len; ++i) {
                            int ox = ox0 + i;

                            int32_t ax_q = ax0 + (int32_t)((int64_t)i * dax);
                            int32_t ay_q = ay0 + (int32_t)((int64_t)i * day);

                            int y_chk = (ay_q >> WFIX_SHIFT);
                            if (y_chk != y0) { vioLocal++; continue; }
                            okLocal++;

                            int x0 = (ax_q >> WFIX_SHIFT);
                            if ((unsigned)x0 >= (unsigned)(WIDTH - 1)) continue;

                            if (und_use_nn) {
                                dstY[ox] = cache.Y[0].data()[x0];
                            } else {
                                uint32_t wx = (uint32_t)(ax_q & 0xFFFF);
                                uint32_t wy = (uint32_t)(ay_q & 0xFFFF);
                                uint32_t W_x0 = 65536u - wx;
                                uint32_t W_y0 = 65536u - wy;
                                uint32_t W_y1 = wy;

                                uint32_t W00 = (uint32_t)(((uint64_t)W_x0 * W_y0) >> 16);
                                uint32_t W10 = (uint32_t)(((uint64_t)wx * W_y0) >> 16);
                                uint32_t W01 = (uint32_t)(((uint64_t)W_x0 * W_y1) >> 16);
                                uint32_t W11 = (uint32_t)(((uint64_t)wx * W_y1) >> 16);

                                const unsigned char* p0Y = cache.Y[0].data() + x0;
                                const unsigned char* p1Y = cache.Y[1].data() + x0;
                                dstY[ox] = sat_u8((int)((p0Y[0] * W00 + p0Y[1] * W10 + p1Y[0] * W01 + p1Y[1] * W11) >> 16));
                            }

                            int32_t ax_ch_q = ax_q >> 1;
                            int xch0 = (ax_ch_q >> WFIX_SHIFT);
                            if ((unsigned)xch0 >= (unsigned)((WIDTH / 2) - 1)) continue;

                            int cidx = ox >> 1;
                            if ((unsigned)cidx < (unsigned)cwCam) {
                                if (und_use_nn) {
                                    dstU[cidx] = cache.U[0].data()[xch0];
                                    dstV[cidx] = cache.V[0].data()[xch0];
                                } else {
                                    uint32_t wx_ch = (uint32_t)(ax_ch_q & 0xFFFF);
                                    uint32_t wy = (uint32_t)(ay_q & 0xFFFF);
                                    uint32_t Wcx0 = 65536u - wx_ch;
                                    uint32_t W_y0 = 65536u - wy;
                                    uint32_t W_y1 = wy;

                                    uint32_t Wc00 = (uint32_t)(((uint64_t)Wcx0 * W_y0) >> 16);
                                    uint32_t Wc10 = (uint32_t)(((uint64_t)wx_ch * W_y0) >> 16);
                                    uint32_t Wc01 = (uint32_t)(((uint64_t)Wcx0 * W_y1) >> 16);
                                    uint32_t Wc11 = (uint32_t)(((uint64_t)wx_ch * W_y1) >> 16);

                                    const unsigned char* p0U = cache.U[0].data() + xch0;
                                    const unsigned char* p1U = cache.U[1].data() + xch0;
                                    const unsigned char* p0V = cache.V[0].data() + xch0;
                                    const unsigned char* p1V = cache.V[1].data() + xch0;

                                    dstU[cidx] = sat_u8((int)((p0U[0] * Wc00 + p0U[1] * Wc10 + p1U[0] * Wc01 + p1U[1] * Wc11) >> 16));
                                    dstV[cidx] = sat_u8((int)((p0V[0] * Wc00 + p0V[1] * Wc10 + p1V[0] * Wc01 + p1V[1] * Wc11) >> 16));
                                }
                            }
                        }
                    }

                    und_cache_ok.fetch_add(okLocal, std::memory_order_relaxed);
                    und_cache_violation.fetch_add(vioLocal, std::memory_order_relaxed);

                    und_barrier.wait(); // end y0
                }
            });

            printf("\n===== UNDISTORT: PANORAMA HW PIPELINE (1 cam, %d frames) =====\n", NUM_FRAMES);

            std::vector<double> frame_ms(NUM_FRAMES, 0.0);
            auto total_t0 = std::chrono::high_resolution_clock::now();

            for (int f = 0; f < NUM_FRAMES; ++f) {
                reset_stats();
                und_cache_ok.store(0);
                und_cache_violation.store(0);
                und_peakScheduleBytes.store(0);

                und_rowWindow.reset_mappings();

                // Clear DDR output (same as panorama)
                std::fill(und_panoOut.ddrY.begin(), und_panoOut.ddrY.end(), (unsigned char)0);
                std::fill(und_panoOut.ddrU.begin(), und_panoOut.ddrU.end(), (unsigned char)128);
                std::fill(und_panoOut.ddrV.begin(), und_panoOut.ddrV.end(), (unsigned char)128);
                MEM_WRITE((uint64_t)und_panoOut.ddrY.size() + (uint64_t)und_panoOut.ddrU.size() + (uint64_t)und_panoOut.ddrV.size());

                und_panoOut.active = 0;
                und_panoOut.started = false;
                und_panoOut.base_py = -1;
                und_panoOut.fill = 0;
                und_panoOut.clear_active();

                auto frame_t0 = std::chrono::high_resolution_clock::now();

                // Streaming emit (same as panorama main loop)
                std::vector<unsigned char> row_ready((size_t)und_per_cam_h, 0);
                int sy_out = 0;

                for (int y0 = 0; y0 < HEIGHT - 1; ++y0) {
                    und_current_y0.store(y0, std::memory_order_relaxed);
                    und_barrier.wait(); // start worker
                    und_barrier.wait(); // wait done

                    // Mark destination rows whose last source dependency is this y0
                    for (int sy = 0; sy < und_per_cam_h; ++sy) {
                        if (und_maxY0Bound_sy[sy] == y0) row_ready[sy] = 1;
                    }

                    // Emit completed rows: RowWindow → PanoSlicePingPong → DDR
                    // For undistort: 1 camera, no blend, direct copy from RowWindow cam 0
                    while (sy_out < und_per_cam_h && row_ready[sy_out]) {
                        if (sy_out < und_roi.y0 || sy_out >= und_roi.y0 + und_out_h) {
                            und_rowWindow.free_sy(sy_out);
                            row_ready[sy_out] = 0;
                            sy_out++;
                            continue;
                        }

                        int py = sy_out - und_roi.y0;
                        unsigned char* dstY = nullptr;
                        unsigned char* dstU = nullptr;
                        unsigned char* dstV = nullptr;

                        if (und_panoOut.begin_row(py, dstY, dstU, dstV)) {
                            int slot = und_rowWindow.sy_to_slot[sy_out];
                            if (slot >= 0) {
                                StageTimer _b(ST_BLEND);
                                const unsigned char* srcY = und_rowWindow.rowY(slot) + und_roi.x0;
                                const unsigned char* srcU = und_rowWindow.rowU(slot) + (und_roi.x0 / 2);
                                const unsigned char* srcV = und_rowWindow.rowV(slot) + (und_roi.x0 / 2);
                                memcpy(dstY, srcY, (size_t)und_out_w);
                                memcpy(dstU, srcU, (size_t)(und_out_w / 2));
                                memcpy(dstV, srcV, (size_t)(und_out_w / 2));
                                MEM_READ((uint64_t)und_out_w + 2ull * (uint64_t)(und_out_w / 2));
                                MEM_WRITE((uint64_t)und_out_w + 2ull * (uint64_t)(und_out_w / 2));
                            } else {
                                // Row not filled — stays at clear values (Y=0, U/V=128)
                            }
                            und_panoOut.end_row();
                        }

                        und_rowWindow.free_sy(sy_out);
                        row_ready[sy_out] = 0;
                        sy_out++;
                    }
                }

                // Emit remaining rows
                for (int sy = sy_out; sy < und_per_cam_h; ++sy) {
                    if (sy < und_roi.y0 || sy >= und_roi.y0 + und_out_h) {
                        und_rowWindow.free_sy(sy);
                        continue;
                    }

                    int py = sy - und_roi.y0;
                    unsigned char* dstY = nullptr;
                    unsigned char* dstU = nullptr;
                    unsigned char* dstV = nullptr;

                    if (und_panoOut.begin_row(py, dstY, dstU, dstV)) {
                        int slot = und_rowWindow.sy_to_slot[sy];
                        if (slot >= 0) {
                            StageTimer _b(ST_BLEND);
                            memcpy(dstY, und_rowWindow.rowY(slot) + und_roi.x0, (size_t)und_out_w);
                            memcpy(dstU, und_rowWindow.rowU(slot) + (und_roi.x0 / 2), (size_t)(und_out_w / 2));
                            memcpy(dstV, und_rowWindow.rowV(slot) + (und_roi.x0 / 2), (size_t)(und_out_w / 2));
                            MEM_READ((uint64_t)und_out_w + 2ull * (uint64_t)(und_out_w / 2));
                            MEM_WRITE((uint64_t)und_out_w + 2ull * (uint64_t)(und_out_w / 2));
                        }
                        und_panoOut.end_row();
                    }
                    und_rowWindow.free_sy(sy);
                }

                und_panoOut.finalize();

                auto frame_t1 = std::chrono::high_resolution_clock::now();
                double ms = std::chrono::duration<double, std::milli>(frame_t1 - frame_t0).count();
                frame_ms[f] = ms;

                // Save from DDR (same as panorama)
                if (SAVE_EVERY_K > 0 && ((f + 1) % SAVE_EVERY_K == 0)) {
                    YUV422Image undDDR{};
                    undDDR.width = und_out_w;
                    undDDR.height = und_out_h;
                    undDDR.Y = und_panoOut.ddrY.data();
                    undDDR.U = und_panoOut.ddrU.data();
                    undDDR.V = und_panoOut.ddrV.data();

                    std::wstring prefW = (camEO ? L"undist_eo_cam" : L"undist_ir_cam") + std::to_wstring(camIdx);
                    std::string bmpPath = SaveImageFilePath(captureDirW, prefW, f);
                    Image* undBGR = yuv422ToBGR(&undDDR);
                    if (undBGR) { writeBMP(bmpPath.c_str(), undBGR); printf("[Saved BMP] %s\n", bmpPath.c_str()); freeImage(undBGR); }
                }
            }

            auto total_t1 = std::chrono::high_resolution_clock::now();
            double total_ms = std::chrono::duration<double, std::milli>(total_t1 - total_t0).count();
            double sum_ms = 0.0;
            for (int i = 0; i < NUM_FRAMES; ++i) sum_ms += frame_ms[i];
            double avg_ms = (NUM_FRAMES > 0) ? sum_ms / NUM_FRAMES : 0.0;
            double throughput_fps = 1000.0 * NUM_FRAMES / total_ms;
            double latency_fps = avg_ms > 0.0 ? 1000.0 / avg_ms : 0.0;

            // Shutdown worker
            und_shutdownFlag.store(true, std::memory_order_release);
            und_current_y0.store(0, std::memory_order_relaxed);
            und_barrier.wait();
            und_barrier.wait();
            if (und_worker.joinable()) und_worker.join();

            // Memory report (same format as panorama)
            size_t undPeakSched = und_peakScheduleBytes.load(std::memory_order_relaxed);
            print_internal_memory_report_v17(und_num_cams, WIDTH, WIDTH, HEIGHT, und_per_cam_w_vec, undPeakSched, ACTIVE_BUFFER, PING_PONG_PUSH_BUFFER);

            printf("\n===== MULTI-FRAME SUMMARY (UNDISTORT) =====\n");
            printf("Frames: %d\n", NUM_FRAMES);
            printf("Avg latency per frame: %.3f ms (%.2f FPS)\n", avg_ms, latency_fps);
            printf("Total time: %.3f ms (Throughput %.2f FPS)\n", total_ms, throughput_fps);

            for (auto* fimg : framesYUV) freeYUV422ImageHeap(fimg);
            return 0;
        }

        // Per-camera target widths
        const int overlap_resized = div_round_i64((int64_t)overlap_px * resize_factor_q16, 1ll << 16);
        const int pano_w_crop = NUM_CAMS * crop_w - (NUM_CAMS - 1) * overlap_resized;
        const int SCALE_X_SHIFT = 20;
        const int64_t scale_x_q20 = ((int64_t)TARGET_W << SCALE_X_SHIFT) / std::max(1, pano_w_crop);

        int overlap_target = div_round_i64((int64_t)overlap_resized * scale_x_q20, 1ll << SCALE_X_SHIFT);
        int per_cam_w_base = div_round_i64((int64_t)crop_w * scale_x_q20, 1ll << SCALE_X_SHIFT);

        std::vector<int> per_cam_w(NUM_CAMS, per_cam_w_base);
        {
            int total_now = NUM_CAMS * per_cam_w_base - (NUM_CAMS - 1) * overlap_target;
            int delta = TARGET_W - total_now;
            for (int i = 0; delta > 0 && i < NUM_CAMS; ++i, --delta) per_cam_w[i]++;
            for (int i = 0; delta < 0 && i < NUM_CAMS; ++i, ++delta) if (per_cam_w[i] > 1) per_cam_w[i]--;
        }
        const int per_cam_h = crop_h;
        int per_cam_w_max = 0;
        for (int w : per_cam_w) per_cam_w_max = std::max(per_cam_w_max, w);

        // Base maps (original coords): load if present, else build and save
        std::vector<int32_t> base_x_q16, base_y_q16;
        if (!loadBaseMapsQ16(base_x_q16, base_y_q16, per_cam_w_max, crop_h, exePath, mapPrefix)) {
            buildBaseMapsOriginal(comp_x_q, comp_y_q, resized_w, resized_h, crop_x0, crop_y0, crop_w, crop_h,
                per_cam_w_max, resize_factor, base_x_q16, base_y_q16);
            saveBaseMapsQ16(base_x_q16, base_y_q16, exePath, mapPrefix);
        }

        // Feather alpha
        const int cw_out = TARGET_W / 2;
        std::vector<uint16_t> alphaY(std::max(overlap_target, 1));
        std::vector<uint16_t> alphaC(std::max(overlap_target / 2, 1));
        {
            StageTimer _t(ST_BLEND);
            if (overlap_target > 0) {
                const double inv_overlap = 1.0 / overlap_target;
                const double gamma = 1.0 / std::max<float>(0.1f, feather_strength);
                for (int x = 0; x < overlap_target; ++x) {
                    double t = (x + 0.5) * inv_overlap;
                    double s = std::pow(t, gamma);
                    alphaY[x] = (uint16_t)std::lround(s * 65535.0);
                }
            }
            int overlapC_local = overlap_target / 2;
            if (overlapC_local > 0) {
                const double inv_overlap = 1.0 / overlapC_local;
                const double gamma = 1.0 / std::max<float>(0.1f, feather_strength);
                for (int x = 0; x < overlapC_local; ++x) {
                    double t = (x + 0.5) * inv_overlap;
                    double s = std::pow(t, gamma);
                    alphaC[x] = (uint16_t)std::lround(s * 65535.0);
                }
            }
        }

        // Row window (fixed size for FPGA - sized for worst case)
        printf("[CONFIG] Using fixed ACTIVE_BUFFER=%d (FPGA constraint), PING_PONG_PUSH_BUFFER=%d\n", ACTIVE_BUFFER, PING_PONG_PUSH_BUFFER);

        RowWindow rowWindow;
        rowWindow.init(NUM_CAMS, per_cam_h, per_cam_w);

        // Sync
        SimpleBarrier rowBarrier(NUM_CAMS + 1);
        std::atomic<bool> shutdownFlag(false);
        std::atomic<int> current_y0(0);
        std::atomic<int> render_cam_turn(0);

        // Interpolation mode (shared)
        const bool use_nn_interp = camCfg.use_nearest_neighbor;

        // Shared per-frame cam params (read-only during y0 loop)
        std::vector<int32_t> y_shift_q_cam(NUM_CAMS, 0);
        std::vector<int16_t> c15_cam(NUM_CAMS, 0);
        std::vector<int16_t> s15_cam(NUM_CAMS, 0);

        std::vector<int> cam_x0(NUM_CAMS, 0);
        std::vector<int> cam_x0_c(NUM_CAMS, 0);
        for (int cam = 1; cam < NUM_CAMS; ++cam) {
            cam_x0[cam] = cam_x0[cam - 1] + per_cam_w[cam - 1] - overlap_target;
        }
        for (int cam = 0; cam < NUM_CAMS; ++cam) {
            cam_x0_c[cam] = cam_x0[cam] / 2;
        }

        // Shared spans per frame
        std::vector<std::vector<int16_t>> spanMin_cam_sy, spanMax_cam_sy;

        // Reverse index: y0 -> sy rows (per cam, flat-encoded)
        std::vector<Y0SyIndex> y0_to_sy_cam;

        // Cache hit counters
        std::atomic<uint64_t> cache_ok(0), cache_violation(0);

        // Peak schedule size (bytes) across all workers for this frame
        std::atomic<size_t> peakScheduleBytesAtomic(0);

        // Workers threads: build runs + render for their camera
        std::vector<std::thread> workers;
        workers.reserve(NUM_CAMS);

        // constants for mapping bounds
        const int32_t cx_q = (int32_t)((int64_t)WIDTH << (WFIX_SHIFT - 1));
        const int32_t cy_q = (int32_t)((int64_t)HEIGHT << (WFIX_SHIFT - 1));
        const int32_t max_x_q = (int32_t)((WIDTH - 2) << WFIX_SHIFT);
        const int32_t max_y_q = (int32_t)((HEIGHT - 2) << WFIX_SHIFT);

        for (int cam = 0; cam < NUM_CAMS; ++cam) {
            workers.emplace_back([&, cam]() {
                LineCache cache;
                std::vector<RowRun> localRuns;
                size_t localPeakBytes = 0;

                while (true) {
                    rowBarrier.wait(); // start y0 phase

                    if (shutdownFlag.load(std::memory_order_acquire)) {
                        rowBarrier.wait(); // end phase
                        break;
                    }

                    int y0 = current_y0.load(std::memory_order_relaxed);
                    if (y0 < 0 || y0 >= HEIGHT - 1) {
                        rowBarrier.wait();
                        continue;
                    }

                    // 1) Build schedule ON THE FLY here (parallel across cams)
                    build_runs_for_y0_on_the_fly_rowptr(
                        y0,
                        per_cam_w[cam],
                        per_cam_h,
                        per_cam_w_max,
                        base_x_q16, base_y_q16,
                        cx_q, cy_q, max_x_q, max_y_q,
                        y_shift_q_cam[cam],
                        c15_cam[cam], s15_cam[cam],
                        spanMin_cam_sy[cam], spanMax_cam_sy[cam],
                        y0_to_sy_cam[cam],
                        localRuns,
                        localPeakBytes
                    );

                    // publish peak schedule bytes (atomic max)
                    {
                        size_t cur = peakScheduleBytesAtomic.load(std::memory_order_relaxed);
                        while (localPeakBytes > cur &&
                            !peakScheduleBytesAtomic.compare_exchange_weak(cur, localPeakBytes, std::memory_order_relaxed)) {
                            // loop
                        }
                    }

                    // 2) Render using 2-line cache (cam-ordered into shared RowWindow)
                    StageTimer _r(ST_RENDER);

                    while (render_cam_turn.load(std::memory_order_acquire) != cam) {
                        std::this_thread::yield();
                    }

                    const unsigned char* Y0 = framesYUV[cam]->Y + (size_t)y0 * (size_t)WIDTH;
                    const unsigned char* U0 = framesYUV[cam]->U + (size_t)y0 * (size_t)(WIDTH / 2);
                    const unsigned char* V0 = framesYUV[cam]->V + (size_t)y0 * (size_t)(WIDTH / 2);

                    const unsigned char* Y1 = framesYUV[cam]->Y + (size_t)(y0 + 1) * (size_t)WIDTH;
                    const unsigned char* U1 = framesYUV[cam]->U + (size_t)(y0 + 1) * (size_t)(WIDTH / 2);
                    const unsigned char* V1 = framesYUV[cam]->V + (size_t)(y0 + 1) * (size_t)(WIDTH / 2);

                    cache.loadRowPtr(0, Y0, U0, V0, y0);
                    cache.loadRowPtr(1, Y1, U1, V1, y0 + 1);

                    const int wCam = per_cam_w[cam];
                    const int cwCam = wCam / 2;

                    uint64_t okLocal = 0;
                    uint64_t vioLocal = 0;

                    for (const RowRun& r : localRuns) {
                        if (r.sy >= (uint16_t)per_cam_h) continue;

                        int slot = rowWindow.allocate_slot_for_sy((int)r.sy);
                        if (slot < 0) continue; // bounded buffer: drop

                        unsigned char* dstY = rowWindow.rowY(slot);
                        unsigned char* dstU = rowWindow.rowU(slot);
                        unsigned char* dstV = rowWindow.rowV(slot);

                        int32_t ax0 = r.ax0_q16;
                        int32_t ay0 = r.ay0_q16;
                        int32_t dax = ((int32_t)r.dax_q12_4) << DRUN_TO_Q16;
                        int32_t day = ((int32_t)r.day_q12_4) << DRUN_TO_Q16;

                        int ox0 = (int)r.ox0;
                        int len = (int)r.len;
                        if (ox0 < 0 || ox0 >= wCam) continue;
                        if (len <= 0) continue;
                        if (ox0 + len > wCam) len = wCam - ox0;

                        for (int i = 0; i < len; ++i) {
                            int ox = ox0 + i;

                            int32_t ax_q = ax0 + (int32_t)((int64_t)i * dax);
                            int32_t ay_q = ay0 + (int32_t)((int64_t)i * day);

                            int y_chk = (ay_q >> WFIX_SHIFT);
                            if (y_chk != y0) { vioLocal++; continue; }
                            okLocal++;

                            int x0 = (ax_q >> WFIX_SHIFT);
                            if ((unsigned)x0 >= (unsigned)(WIDTH - 1)) continue;

                            int dst_x = cam_x0[cam] + ox;
                            if ((unsigned)dst_x >= (unsigned)TARGET_W) continue;

                            unsigned char y_val = 0;
                            if (use_nn_interp) {
                                const unsigned char* p0Y = cache.Y[0].data() + x0;
                                y_val = p0Y[0];
                            } else {
                                uint32_t wx = (uint32_t)(ax_q & 0xFFFF);
                                uint32_t wy = (uint32_t)(ay_q & 0xFFFF);

                                uint32_t W_x0 = 65536u - wx;
                                uint32_t W_x1 = wx;
                                uint32_t W_y0 = 65536u - wy;
                                uint32_t W_y1 = wy;

                                uint32_t W00 = (uint32_t)(((uint64_t)W_x0 * W_y0) >> 16);
                                uint32_t W10 = (uint32_t)(((uint64_t)W_x1 * W_y0) >> 16);
                                uint32_t W01 = (uint32_t)(((uint64_t)W_x0 * W_y1) >> 16);
                                uint32_t W11 = (uint32_t)(((uint64_t)W_x1 * W_y1) >> 16);

                                const unsigned char* p0Y = cache.Y[0].data() + x0;
                                const unsigned char* p2Y = cache.Y[1].data() + x0;

                                int Y_val = (int)((p0Y[0] * W00 + p0Y[1] * W10 + p2Y[0] * W01 + p2Y[1] * W11) >> 16);
                                y_val = sat_u8(Y_val);
                            }

                            if (cam == 0 || ox >= overlap_target) {
                                // Fade-out on right edge for cameras that have a neighbor to the right
                                int right_dist = wCam - 1 - ox; // distance from right edge
                                if (cam < NUM_CAMS - 1 && right_dist < overlap_target) {
                                    int complement_idx = std::min(overlap_target - 1 - right_dist, (int)alphaY.size() - 1);
                                    unsigned a_right = 65535u - alphaY[complement_idx];
                                    y_val = (unsigned char)(((unsigned)y_val * a_right) >> 16);
                                    unsigned prev = dstY[dst_x];
                                    dstY[dst_x] = sat_u8((int)prev + (int)y_val);
                                } else {
                                    dstY[dst_x] = y_val;
                                }
                            } else {
                                unsigned a = alphaY[std::min(ox, (int)alphaY.size() - 1)];
                                unsigned scaled = ((unsigned)y_val * a) >> 16;
                                unsigned prev = dstY[dst_x];
                                dstY[dst_x] = sat_u8((int)prev + (int)scaled);
                            }

                            // Only write chroma on even ox to avoid double-accumulation
                            // (each chroma sample covers a pixel pair; writing on both
                            //  even and odd ox causes the blend contribution to be added twice)
                            if ((ox & 1) != 0) continue;

                            int32_t ax_ch_q = ax_q >> 1;
                            int xch0 = (ax_ch_q >> WFIX_SHIFT);
                            if ((unsigned)xch0 >= (unsigned)((WIDTH / 2) - 1)) continue;

                            int cidx = ox >> 1;
                            if ((unsigned)cidx < (unsigned)cwCam) {
                                int dst_c = cam_x0_c[cam] + cidx;
                                if ((unsigned)dst_c >= (unsigned)(TARGET_W / 2)) continue;

                                unsigned char u_val = 0;
                                unsigned char v_val = 0;
                                if (use_nn_interp) {
                                    const unsigned char* p0U = cache.U[0].data() + xch0;
                                    const unsigned char* p0V = cache.V[0].data() + xch0;
                                    u_val = p0U[0];
                                    v_val = p0V[0];
                                } else {
                                    uint32_t wx_ch = (uint32_t)(ax_ch_q & 0xFFFF);
                                    uint32_t Wcx0 = 65536u - wx_ch;
                                    uint32_t Wcx1 = wx_ch;

                                    uint32_t wy = (uint32_t)(ay_q & 0xFFFF);
                                    uint32_t W_y0 = 65536u - wy;
                                    uint32_t W_y1 = wy;

                                    uint32_t Wc00 = (uint32_t)(((uint64_t)Wcx0 * W_y0) >> 16);
                                    uint32_t Wc10 = (uint32_t)(((uint64_t)Wcx1 * W_y0) >> 16);
                                    uint32_t Wc01 = (uint32_t)(((uint64_t)Wcx0 * W_y1) >> 16);
                                    uint32_t Wc11 = (uint32_t)(((uint64_t)Wcx1 * W_y1) >> 16);

                                    const unsigned char* p0U = cache.U[0].data() + xch0;
                                    const unsigned char* p2U = cache.U[1].data() + xch0;

                                    const unsigned char* p0V = cache.V[0].data() + xch0;
                                    const unsigned char* p2V = cache.V[1].data() + xch0;

                                    int U_val = (int)((p0U[0] * Wc00 + p0U[1] * Wc10 + p2U[0] * Wc01 + p2U[1] * Wc11) >> 16);
                                    int V_val = (int)((p0V[0] * Wc00 + p0V[1] * Wc10 + p2V[0] * Wc01 + p2V[1] * Wc11) >> 16);

                                    u_val = sat_u8(U_val);
                                    v_val = sat_u8(V_val);
                                }

                                int overlapC = overlap_target / 2;
                                int u_centered = (int)u_val - 128;
                                int v_centered = (int)v_val - 128;

                                if (cam == 0 || cidx >= overlapC) {
                                    // Fade-out on right edge for chroma
                                    int right_cidx = cwCam - 1 - cidx;
                                    if (cam < NUM_CAMS - 1 && right_cidx < overlapC) {
                                        int complement_cidx = std::min(overlapC - 1 - right_cidx, (int)alphaC.size() - 1);
                                        unsigned a_right = 65535u - alphaC[complement_cidx];
                                        int u_sc = (u_centered * (int)a_right) >> 16;
                                        int v_sc = (v_centered * (int)a_right) >> 16;
                                        int uprev = (int)dstU[dst_c] - 128;
                                        int vprev = (int)dstV[dst_c] - 128;
                                        dstU[dst_c] = sat_u8(uprev + u_sc + 128);
                                        dstV[dst_c] = sat_u8(vprev + v_sc + 128);
                                    } else {
                                        dstU[dst_c] = u_val;
                                        dstV[dst_c] = v_val;
                                    }
                                } else {
                                    unsigned a = alphaC[std::min(cidx, (int)alphaC.size() - 1)];
                                    int u_sc = (u_centered * (int)a) >> 16;
                                    int v_sc = (v_centered * (int)a) >> 16;
                                    int uprev = (int)dstU[dst_c] - 128;
                                    int vprev = (int)dstV[dst_c] - 128;
                                    dstU[dst_c] = sat_u8(uprev + u_sc + 128);
                                    dstV[dst_c] = sat_u8(vprev + v_sc + 128);
                                }
                            }
                        }
                    }

                    render_cam_turn.fetch_add(1, std::memory_order_release);

                    cache_ok.fetch_add(okLocal, std::memory_order_relaxed);
                    cache_violation.fetch_add(vioLocal, std::memory_order_relaxed);

                    rowBarrier.wait(); // end y0 phase
                }
                });
        }

        // pano slice/DDR output
        PanoSlicePingPong panoOut;
        panoOut.init(TARGET_W, TARGET_H, PING_PONG_PUSH_BUFFER);
        panoOut.clear_active();

        const int ypad = (TARGET_H - per_cam_h) / 2;

        // IMU -> per cam (constant across frames for static test images)
        for (int cam = 0; cam < NUM_CAMS; ++cam) {
            int32_t roll_cam_centi = 0, pitch_cam_centi = 0, yaw_cam_centi = 0;
            convert_rpy_sensor_to_cam_centi(
                roll_Sensor_centi, pitch_Sensor_centi,
                (cam < (int)yaw_off.size() ? yaw_off[cam] : 0),
                (cam < (int)pit_off.size() ? pit_off[cam] : 0),
                (cam < (int)rol_off.size() ? rol_off[cam] : 0),
                &roll_cam_centi, &pitch_cam_centi, &yaw_cam_centi
            );

            const float roll_cam = centi_to_deg(roll_cam_centi);
            const float pitch_cam = centi_to_deg(pitch_cam_centi);
            const float yaw_cam_deg = centi_to_deg(yaw_cam_centi);
            printf("[CAMERA %d] Computed Angles: roll=%.2f, pitch=%.2f, yaw=%.2f\n", cam, roll_cam, pitch_cam, yaw_cam_deg);

            int y_shift_px = div_round_i64((int64_t)PITCH_TO_TRANSLATION_FACTOR_q16 * pitch_cam_centi, 100ll * (1ll << 16));
            y_shift_q_cam[cam] = (int32_t)y_shift_px << WFIX_SHIFT;

            int16_t c15 = 0, s15 = 0;
            get_cos_sin_q15_centi(-roll_cam_centi, &c15, &s15);
            c15_cam[cam] = c15;
            s15_cam[cam] = s15;
        }

        // Precompute spans and global maxY0Bound per sy (one-time, constant across frames)
        std::vector<int> maxY0Bound_sy;
        precompute_spans_and_bounds(
            per_cam_h, per_cam_w_max,
            base_x_q16, base_y_q16,
            per_cam_w,
            y_shift_q_cam,
            c15_cam, s15_cam,
            cx_q, cy_q, max_x_q, max_y_q,
            spanMin_cam_sy, spanMax_cam_sy,
            maxY0Bound_sy,
            y0_to_sy_cam
        );

        // Print geometry info once
        printf("[GEOMETRY] WIDTH=%d HEIGHT=%d resized=%dx%d crop=%dx%d @ (%d,%d)\n",
               WIDTH, HEIGHT, resized_w, resized_h, crop_w, crop_h, crop_x0, crop_y0);
        printf("[GEOMETRY] per_cam_h=%d per_cam_w_max=%d TARGET=%dx%d ypad=%d\n",
               per_cam_h, per_cam_w_max, TARGET_W, TARGET_H, ypad);

        int invalidBoundCount = 0;
        for (int sy = 0; sy < per_cam_h; ++sy) {
            if (maxY0Bound_sy[sy] < 0) invalidBoundCount++;
        }
        if (invalidBoundCount > 0) {
            printf("[WARNING] %d destination rows have no valid y0 mapping (will be black)\n", invalidBoundCount);
        }

        // Timing
        size_t globalPeakScheduleBytes = 0;
        std::vector<double> frame_ms(NUM_FRAMES, 0.0);
        auto total_t0 = std::chrono::high_resolution_clock::now();

        printf("\n===== PANORAMA: %d frames =====\n", NUM_FRAMES);

        for (int f = 0; f < NUM_FRAMES; ++f) {
            reset_stats();
            cache_ok.store(0);
            cache_violation.store(0);
            peakScheduleBytesAtomic.store(0);

            rowWindow.reset_mappings();

            std::fill(panoOut.ddrY.begin(), panoOut.ddrY.end(), (unsigned char)0);
            std::fill(panoOut.ddrU.begin(), panoOut.ddrU.end(), (unsigned char)128);
            std::fill(panoOut.ddrV.begin(), panoOut.ddrV.end(), (unsigned char)128);
            MEM_WRITE((uint64_t)panoOut.ddrY.size() + (uint64_t)panoOut.ddrU.size() + (uint64_t)panoOut.ddrV.size());

            panoOut.active = 0;
            panoOut.started = false;
            panoOut.base_py = -1;
            panoOut.fill = 0;
            panoOut.clear_active();

            auto frame_t0 = std::chrono::high_resolution_clock::now();

            // Streaming-safe emit logic
            std::vector<unsigned char> row_ready((size_t)per_cam_h, 0);
            int sy_out = 0;

            // Source-driven execution over y0 (main thread just coordinates)
            for (int y0 = 0; y0 < HEIGHT - 1; ++y0) {
                current_y0.store(y0, std::memory_order_relaxed);
                render_cam_turn.store(0, std::memory_order_release);
                rowBarrier.wait(); // start workers (they build schedule + render)
                rowBarrier.wait(); // wait done

                for (int sy = 0; sy < per_cam_h; ++sy) {
                    if (maxY0Bound_sy[sy] == y0) row_ready[sy] = 1;
                }

                while (sy_out < per_cam_h && row_ready[sy_out]) {
                    int py = sy_out + ypad;
                    unsigned char* dstY = nullptr;
                    unsigned char* dstU = nullptr;
                    unsigned char* dstV = nullptr;

                    if (panoOut.begin_row(py, dstY, dstU, dstV)) {
                        blend_row_sy_into_dst(
                            sy_out,
                            TARGET_W,
                            overlap_target,
                            alphaY, alphaC,
                            per_cam_w,
                            rowWindow,
                            dstY, dstU, dstV
                        );
                        panoOut.end_row();
                    }

                    rowWindow.free_sy(sy_out);
                    row_ready[sy_out] = 0;
                    sy_out++;
                }
            }

            // Emit remaining rows
            for (int sy = sy_out; sy < per_cam_h; ++sy) {
                int py = sy + ypad;
                unsigned char* dstY = nullptr;
                unsigned char* dstU = nullptr;
                unsigned char* dstV = nullptr;

                if (panoOut.begin_row(py, dstY, dstU, dstV)) {
                    blend_row_sy_into_dst(
                        sy,
                        TARGET_W,
                        overlap_target,
                        alphaY, alphaC,
                        per_cam_w,
                        rowWindow,
                        dstY, dstU, dstV
                    );
                    panoOut.end_row();
                }
                rowWindow.free_sy(sy);
            }

            panoOut.finalize();

            // Track peak schedule bytes across all frames
            size_t peakScheduleBytes = peakScheduleBytesAtomic.load(std::memory_order_relaxed);
            if (peakScheduleBytes > globalPeakScheduleBytes) globalPeakScheduleBytes = peakScheduleBytes;

            // Save
            if (SAVE_EVERY_K > 0 && ((f + 1) % SAVE_EVERY_K == 0)) {
                YUV422Image panoDDR{};
                panoDDR.width = TARGET_W;
                panoDDR.height = TARGET_H;
                panoDDR.Y = panoOut.ddrY.data();
                panoDDR.U = panoOut.ddrU.data();
                panoDDR.V = panoOut.ddrV.data();

                std::string bmpPath = SaveImageFilePath(captureDirW, L"panorama_v17", f);
                Image* panoBGR = yuv422ToBGR(&panoDDR);
                if (panoBGR) {
                    writeBMP(bmpPath.c_str(), panoBGR);
                    printf("[Saved BMP] %s\n", bmpPath.c_str());
                    freeImage(panoBGR);
                }
            }

            auto frame_t1 = std::chrono::high_resolution_clock::now();
            double ms = std::chrono::duration<double, std::milli>(frame_t1 - frame_t0).count();
            frame_ms[f] = ms;
            print_stats_summary("PROFILING SUMMARY (PANORAMA)");
        }

        auto total_t1 = std::chrono::high_resolution_clock::now();
        double total_ms = std::chrono::duration<double, std::milli>(total_t1 - total_t0).count();
        double throughput_fps = 1000.0 * NUM_FRAMES / total_ms;

        double sum_ms = 0.0;
        for (int i = 0; i < NUM_FRAMES; ++i) sum_ms += frame_ms[i];
        double avg_ms = (NUM_FRAMES > 0) ? sum_ms / NUM_FRAMES : 0.0;
        double latency_fps = avg_ms > 0.0 ? 1000.0 / avg_ms : 0.0;

        // Internal memory report (once)
        print_internal_memory_report_v17(NUM_CAMS, WIDTH, TARGET_W, TARGET_H, per_cam_w, globalPeakScheduleBytes, ACTIVE_BUFFER, PING_PONG_PUSH_BUFFER);

        printf("\n===== MULTI-FRAME SUMMARY (PANORAMA) =====\n");
        printf("Frames: %d\n", NUM_FRAMES);
        printf("Avg latency per frame: %.3f ms (%.2f FPS)\n", avg_ms, latency_fps);
        printf("Total time: %.3f ms (Throughput %.2f FPS)\n", total_ms, throughput_fps);

        // Shutdown workers
        shutdownFlag.store(true, std::memory_order_release);
        current_y0.store(0, std::memory_order_relaxed);
        rowBarrier.wait();
        rowBarrier.wait();
        for (auto& th : workers) if (th.joinable()) th.join();

        // Cleanup
        for (auto* fimg : framesYUV) freeYUV422ImageHeap(fimg);

        if (SUCCEEDED(hrCoInit)) CoUninitialize();
        return 0;
    }
    catch (const std::exception& e) {
        printf("[FATAL] exception: %s\n", e.what());
        if (SUCCEEDED(hrCoInit)) CoUninitialize();
        return 1;
    }
    catch (...) {
        printf("[FATAL] unknown exception\n");
        if (SUCCEEDED(hrCoInit)) CoUninitialize();
        return 1;
    }
}
