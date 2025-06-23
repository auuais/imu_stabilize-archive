#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

//for data read from file
/////////////////////////////////////////////////

#include <Shlwapi.h>  // PathRemoveFileSpecW
#pragma comment(lib, "Shlwapi.lib")
#include <filesystem>
#include "ini_manager.h"

//////////////////////////////////////////////




#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// === Global Parameters ===
int WIDTH, HEIGHT, NUM_CAMS, BUFFER_LINES,overlap_px;
float fx, fy, cx, cy;
float K1, K2, K3, P1, P2;
float fx_new = 864.452f, fy_new = 860.023f, cx_new = 947.559f, cy_new = 534.965f;  // keep default if not overridden

float roll_Sensor = 0.0f;
float pitch_Sensor = 0.0f;
float yaw_Sensor = 0.0f;

bool Stabilization = true;

float resize_factor = 0.5;

float cylindrical_H_fov_deg, cylindrical_V_fov_deg;
float panorama_crop_height_scale, panorama_crop_width_scale;
float original_crop_height_scale, original_crop_width_scale;
float PITCH_TO_TRANSLATION_FACTOR, YAW_TO_TRANSLATION_FACTOR;
float OVERLAP_RATIO;
int NON_OVERLAP_WIDTH, PANORAMA_WIDTH;



#pragma pack(push, 1)
typedef struct {
    unsigned short bfType;
    unsigned int   bfSize;
    unsigned short bfReserved1;
    unsigned short bfReserved2;
    unsigned int   bfOffBits;
} CustomBitmapFileHeader;


typedef struct {
    unsigned int   biSize;
    int            biWidth;
    int            biHeight;
    unsigned short biPlanes;
    unsigned short biBitCount;
    unsigned int   biCompression;
    unsigned int   biSizeImage;
    int            biXPelsPerMeter;
    int            biYPelsPerMeter;
    unsigned int   biClrUsed;
    unsigned int   biClrImportant;
} CustomBitmapInfoHeader;
#pragma pack(pop)


typedef struct {
    int width;
    int height;
    int channels;
    unsigned char* data; 
} Image;


#define PIXEL(img, y, x, c) ((img)->data[((y) * (img)->width + (x)) * (img)->channels + (c)])


static unsigned char saturate_cast(float val) {
    if (val < 0) return 0;
    if (val > 255) return 255;
    return (unsigned char)(val + 0.5f);
}


Image* readBMP(const char* filename) {
    FILE* fp = fopen(filename, "rb");
    CustomBitmapFileHeader fileHeader;
    CustomBitmapInfoHeader infoHeader;

    fread(&fileHeader, sizeof(BITMAPFILEHEADER), 1, fp);
    fread(&infoHeader, sizeof(BITMAPINFOHEADER), 1, fp);

    Image* img = (Image*)malloc(sizeof(Image));
    img->width = infoHeader.biWidth;
    img->height = abs(infoHeader.biHeight);
    img->channels = 3;
    int row_padded = ((img->width * 3 + 3) / 4) * 4;
    img->data = (unsigned char*)malloc(img->width * img->height * 3);
    fseek(fp, fileHeader.bfOffBits, SEEK_SET);
    for (int i = 0; i < img->height; i++) {
        fread(img->data + i * img->width * 3, 1, img->width * 3, fp);
        fseek(fp, row_padded - img->width * 3, SEEK_CUR);
    }
    fclose(fp);
    return img;
}


void free_image(Image* img) {
    if (img) {
        if (img->data) free(img->data);
        free(img);
    }
}


int writeBMP(const char* filename, const Image* img) {
    FILE* fp = fopen(filename, "wb");
    int row_padded = ((img->width * 3 + 3) / 4) * 4;
    int fileSize = 54 + row_padded * img->height;
    unsigned char bmpfileheader[14] = { 'B','M', 0,0,0,0, 0,0, 0,0, 54,0,0,0 };
    unsigned char bmpinfoheader[40] = { 0 };
    unsigned char bmppad[3] = { 0,0,0 };
    bmpfileheader[2] = (unsigned char)(fileSize);
    bmpfileheader[3] = (unsigned char)(fileSize >> 8);
    bmpfileheader[4] = (unsigned char)(fileSize >> 16);
    bmpfileheader[5] = (unsigned char)(fileSize >> 24);
    bmpinfoheader[0] = 40;
    bmpinfoheader[4] = (unsigned char)(img->width);
    bmpinfoheader[5] = (unsigned char)(img->width >> 8);
    bmpinfoheader[6] = (unsigned char)(img->width >> 16);
    bmpinfoheader[7] = (unsigned char)(img->width >> 24);
    bmpinfoheader[8] = (unsigned char)(img->height);
    bmpinfoheader[9] = (unsigned char)(img->height >> 8);
    bmpinfoheader[10] = (unsigned char)(img->height >> 16);
    bmpinfoheader[11] = (unsigned char)(img->height >> 24);
    bmpinfoheader[12] = 1;
    bmpinfoheader[14] = 24;
    fwrite(bmpfileheader, 1, 14, fp);
    fwrite(bmpinfoheader, 1, 40, fp);
    for (int i = 0; i < img->height; i++) {
        fwrite(img->data + i * img->width * 3, 1, img->width * 3, fp);
        fwrite(bmppad, 1, row_padded - img->width * 3, fp);
    }
    fclose(fp);
    return 1;
}

// === Interpolation Function ===
void interpolation(const Image* img, float x, float y, unsigned char* result) {
    int x0 = (int)floorf(x);
    int y0 = (int)floorf(y);
    float dx = x - x0;
    float dy = y - y0;

    for (int c = 0; c < 3; ++c) result[c] = 0;

    if (x0 < 0 || x0 >= img->width - 1 || y0 < 0 || y0 >= img->height - 1) return;

    for (int c = 0; c < 3; ++c) {
        float p00 = PIXEL(img, y0, x0, c);
        float p10 = PIXEL(img, y0, x0 + 1, c);
        float p01 = PIXEL(img, y0 + 1, x0, c);
        float p11 = PIXEL(img, y0 + 1, x0 + 1, c);
        float val = (1 - dx) * (1 - dy) * p00 + dx * (1 - dy) * p10 + (1 - dx) * dy * p01 + dx * dy * p11;
        result[c] = (unsigned char)(val > 255 ? 255 : (val < 0 ? 0 : val));
    }
}




// === Manual Undistortion Function ===
Image* Undistort(const Image* img) {
    Image* output = (Image*)malloc(sizeof(Image));
    output->width = img->width;
    output->height = img->height;
    output->channels = 3;
    output->data = (unsigned char*)calloc(img->width * img->height * 3, sizeof(unsigned char));

    for (int v = 0; v < img->height; ++v) {
        for (int u = 0; u < img->width; ++u) {
            float x = (u - cx_new) / fx_new;
            float y = (v - cy_new) / fy_new;
            float r2 = x * x + y * y;

            float radial = 1 + K1 * r2 + K2 * r2 * r2 + K3 * r2 * r2 * r2;
            float dx = 2 * P1 * x * y + P2 * (r2 + 2 * x * x);
            float dy = P1 * (r2 + 2 * y * y) + 2 * P2 * x * y;

            float x_dist = x * radial + dx;
            float y_dist = y * radial + dy;

            float u_d = x_dist * fx + cx;
            float v_d = y_dist * fy + cy;

            unsigned char pixel[3];
            interpolation(img, u_d, v_d, pixel);

            for (int c = 0; c < 3; ++c) {
                PIXEL(output, v, u, c) = pixel[c];
            }
        }
    }
    return output;
}





// === Affine Transformation Function ===
Image* affineTransform(const Image* img, float angle_deg, float translate_y) {
    int w = img->width, h = img->height;
    float angle_rad = angle_deg * M_PI / 180.0f;
    float cos_a = cosf(angle_rad);
    float sin_a = sinf(angle_rad);
    float cx_center = w / 2.0f, cy_center = h / 2.0f;

    Image* output = (Image*)malloc(sizeof(Image));
    output->width = w;
    output->height = h;
    output->channels = 3;
    output->data = (unsigned char*)calloc(w * h * 3, sizeof(unsigned char));

    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            float dx = x - cx_center;
            float dy = y - cy_center - translate_y;
            float src_x = cos_a * dx + sin_a * dy + cx_center;
            float src_y = -sin_a * dx + cos_a * dy + cy_center;
            unsigned char pixel[3];
            interpolation(img, src_x, src_y, pixel);
            for (int c = 0; c < 3; ++c) {
                PIXEL(output, y, x, c) = pixel[c];
            }
        }
    }
    printf("Affine applied with angle=%.2f, shift=%.2f\n", angle_deg, translate_y);

    return output;
}

void loadCylindricalMaps(Image** map_x, Image** map_y, int width, int height, const std::filesystem::path& exePath) {
    *map_x = (Image*)malloc(sizeof(Image));
    *map_y = (Image*)malloc(sizeof(Image));

    (*map_x)->width = width;
    (*map_x)->height = height;
    (*map_x)->channels = 1;
    (*map_x)->data = (unsigned char*)malloc(width * height * sizeof(float));

    (*map_y)->width = width;
    (*map_y)->height = height;
    (*map_y)->channels = 1;
    (*map_y)->data = (unsigned char*)malloc(width * height * sizeof(float));

    std::filesystem::path mapx_path = exePath.parent_path() / "map_x.bin";
    std::filesystem::path mapy_path = exePath.parent_path() / "map_y.bin";

    FILE* fx = fopen(mapx_path.string().c_str(), "rb");
    fread((*map_x)->data, sizeof(float), width * height, fx);
    fclose(fx);

    FILE* fy = fopen(mapy_path.string().c_str(), "rb");
    fread((*map_y)->data, sizeof(float), width * height, fy);
    fclose(fy);

    printf("[INFO] Cylindrical maps loaded from disk.\n");
}


void generateCylindricalRemap(Image** map_x, Image** map_y, int width, int height, float fov_h_deg, float fov_v_deg, float x_shift, const std::filesystem::path& exePath)
{
    float fov_h_rad = fov_h_deg * (M_PI / 180.0f);
    float fov_v_rad = fov_v_deg * (M_PI / 180.0f);

    float fx_cyl = width / (2.0f * tanf(fov_h_rad / 2.0f));
    float fy_cyl = height / (2.0f * tanf(fov_v_rad / 2.0f));
    float cx_cyl = width / 2.0f;
    float cy_cyl = height / 2.0f;

    // Allocate map_x
    *map_x = (Image*)malloc(sizeof(Image));
    (*map_x)->width = width;
    (*map_x)->height = height;
    (*map_x)->channels = 1;
    (*map_x)->data = (unsigned char*)malloc(width * height * sizeof(float));

    // Allocate map_y
    *map_y = (Image*)malloc(sizeof(Image));
    (*map_y)->width = width;
    (*map_y)->height = height;
    (*map_y)->channels = 1;
    (*map_y)->data = (unsigned char*)malloc(width * height * sizeof(float));

    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            float theta = (x - cx_cyl + x_shift) / fx_cyl;
            float h = (y - cy_cyl) / fy_cyl;

            float x_source = sinf(theta);
            float y_source = h;
            float z_source = cosf(theta);

            float src_x = fx*resize_factor * (x_source / z_source) + cx*resize_factor;
            float src_y = fy*resize_factor * (y_source / z_source) + cy*resize_factor;

            int index = y * width + x;
            ((float*)(*map_x)->data)[index] = src_x;
            ((float*)(*map_y)->data)[index] = src_y;
        }
    }

    // Save map_x and map_y to binary files
    std::filesystem::path mapx_path = exePath.parent_path() / "map_x.bin";
    std::filesystem::path mapy_path = exePath.parent_path() / "map_y.bin";

    FILE* fx = fopen(mapx_path.string().c_str(), "wb");
    fwrite((*map_x)->data, sizeof(float), width * height, fx);
    fclose(fx);

    FILE* fy = fopen(mapy_path.string().c_str(), "wb");
    fwrite((*map_y)->data, sizeof(float), width * height, fy);
    fclose(fy);

    printf("[INFO] Cylindrical maps saved to %s and %s\n",
        mapx_path.string().c_str(), mapy_path.string().c_str());

}





// === Cylindrical Remapping Function ===
//Image* cylindricalRemap(const Image* img, const Image* map_x, const Image* map_y) {
//    Image* output = (Image*)malloc(sizeof(Image));
//    output->width = img->width;
//    output->height = img->height;
//    output->channels = img->channels;
//    output->data = (unsigned char*)calloc(img->width * img->height * img->channels, sizeof(unsigned char));
//
//    for (int y = 0; y < img->height; ++y) {
//        for (int x = 0; x < img->width; ++x) {
//            int index = y * img->width + x;
//            float src_x = ((float*)map_x->data)[index];
//            float src_y = ((float*)map_y->data)[index];
//
//            unsigned char pixel[3];
//            interpolation(img, src_x, src_y, pixel);
//
//            for (int c = 0; c < 3; ++c) {
//                PIXEL(output, y, x, c) = pixel[c];
//            }
//        }
//    }
//    return output;
//}
//
//
//// === Manual Crop Function ===
//Image* cropFrame(const Image* img, int start_x, int start_y, int crop_w, int crop_h) {
//    Image* cropped = (Image*)malloc(sizeof(Image));
//    cropped->width = crop_w;
//    cropped->height = crop_h;
//    cropped->channels = img->channels;
//    cropped->data = (unsigned char*)calloc(crop_w * crop_h * img->channels, sizeof(unsigned char));
//
//    for (int y = 0; y < crop_h; ++y) {
//        for (int x = 0; x < crop_w; ++x) {
//            int src_x = start_x + x;
//            int src_y = start_y + y;
//            if (src_x >= 0 && src_x < img->width && src_y >= 0 && src_y < img->height) {
//                for (int c = 0; c < img->channels; ++c) {
//                    PIXEL(cropped, y, x, c) = PIXEL(img, src_y, src_x, c);
//                }
//            }
//        }
//    }
//    return cropped;
//}
//
//// === Center Crop Function ===
//Image* cropFrameHW(const Image* img, float crop_scale_h, float crop_scale_w) {
//    int crop_h = (int)(img->height * crop_scale_h);
//    int crop_w = (int)(img->width * crop_scale_w);
//    int start_y = (img->height - crop_h) / 2;
//    int start_x = (img->width - crop_w) / 2;
//    return cropFrame(img, start_x, start_y, crop_w, crop_h);
//}


Image* processFramePipeline(
    const Image* input,
    const Image* map_x,
    const Image* map_y,
    float resize_factor,
    float roll_angle_deg,
    float y_shift,
    float crop_h_scale,
    float crop_w_scale
) {
    // Determine intermediate sizes
    int resized_w = static_cast<int>(input->width * resize_factor);
    int resized_h = static_cast<int>(input->height * resize_factor);

    // Determine crop area
    int crop_w = static_cast<int>(resized_w * crop_w_scale);
    int crop_h = static_cast<int>(resized_h * crop_h_scale);
    int crop_x0 = (resized_w - crop_w) / 2;
    int crop_y0 = (resized_h - crop_h) / 2;

    // Prepare output image
    Image* output = (Image*)malloc(sizeof(Image));
    output->width = crop_w;
    output->height = crop_h;
    output->channels = 3;
    output->data = (unsigned char*)calloc(crop_w * crop_h * 3, sizeof(unsigned char));

    // Precompute rotation parameters
    float angle_rad = roll_angle_deg * M_PI / 180.0f;
    float cos_a = cosf(angle_rad);
    float sin_a = sinf(angle_rad);
    float cx = resized_w / 2.0f;
    float cy = resized_h / 2.0f;

    // Combined processing loop
    for (int y = 0; y < crop_h; ++y) {
        for (int x = 0; x < crop_w; ++x) {
            int target_x = crop_x0 + x;
            int target_y = crop_y0 + y;

            // Cylindrical source from map
            int map_idx = target_y * resized_w + target_x;
            float src_x_cyl = ((float*)map_x->data)[map_idx];
            float src_y_cyl = ((float*)map_y->data)[map_idx];

            // Affine transform
            float dx = src_x_cyl - cx;
            float dy = src_y_cyl - cy - y_shift;

            float affine_x = cos_a * dx + sin_a * dy + cx;
            float affine_y = -sin_a * dx + cos_a * dy + cy;

            // Resize: inverse scale to original image
            float final_x = affine_x / resize_factor;
            float final_y = affine_y / resize_factor;

            // Interpolate and store
            unsigned char pixel[3];
            interpolation(input, final_x, final_y, pixel);
            for (int c = 0; c < 3; ++c) {
                PIXEL(output, y, x, c) = pixel[c];
            }
        }
    }

    return output;
}



Image* ResizeImage(const Image* input, float scale) {
    int new_width = static_cast<int>(input->width * scale);
    int new_height = static_cast<int>(input->height * scale);

    // Allocate new image manually
    Image* output = (Image*)malloc(sizeof(Image));
    output->width = new_width;
    output->height = new_height;
    output->channels = input->channels;
    output->data = (unsigned char*)calloc(new_width * new_height * input->channels, sizeof(unsigned char));

    for (int y = 0; y < new_height; ++y) {
        for (int x = 0; x < new_width; ++x) {
            float src_x = x / scale;
            float src_y = y / scale;

            unsigned char pixel[3]; // Assuming 3 channels max; extend if needed
            interpolation(input, src_x, src_y, pixel);

            for (int c = 0; c < input->channels; ++c) {
                output->data[(y * new_width + x) * input->channels + c] = pixel[c];
            }
        }
    }

    return output;
}


Image* featherBlendFit(const std::vector<Image*>& frames, int overlap, int target_width, int target_height) {
    if (frames.empty()) return NULL;

    int height = frames[0]->height;
    int channels = frames[0]->channels;
    int total_width = frames[0]->width;

    for (int i = 1; i < frames.size(); ++i)
        total_width += (frames[i]->width - overlap);

    // Step 1: Allocate intermediate panorama buffer
    Image* blended = (Image*)malloc(sizeof(Image));
    blended->width = total_width;
    blended->height = height;
    blended->channels = channels;
    blended->data = (unsigned char*)calloc(total_width * height * channels, sizeof(unsigned char));

    int current_x = 0;
    for (int i = 0; i < frames.size(); ++i) {
        Image* img = frames[i];
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < img->width; ++x) {
                int out_x = current_x + x;

                if (i > 0 && x < overlap) {
                    float alpha = (float)x / (float)overlap;
                    for (int c = 0; c < channels; ++c) {
                        unsigned char prev = PIXEL(blended, y, out_x, c);
                        unsigned char curr = PIXEL(img, y, x, c);
                        PIXEL(blended, y, out_x, c) = (unsigned char)((1 - alpha) * prev + alpha * curr);
                    }
                }
                else {
                    for (int c = 0; c < channels; ++c) {
                        PIXEL(blended, y, out_x, c) = PIXEL(img, y, x, c);
                    }
                }
            }
        }
        current_x += (img->width - overlap);
    }

    // Step 2: Resize to target width, compute scale factor
    float scale = (float)target_width / blended->width;
    int new_height = (int)(blended->height * scale);

    Image* resized = ResizeImage(blended, scale);
    free(blended->data);
    free(blended);

    // Step 3: Create final image with vertical padding to target height
    Image* final_output = (Image*)malloc(sizeof(Image));
    final_output->width = target_width;
    final_output->height = target_height;
    final_output->channels = channels;
    final_output->data = (unsigned char*)calloc(target_width * target_height * channels, sizeof(unsigned char));

    int y_offset = (target_height - new_height) / 2;

    for (int y = 0; y < new_height; ++y) {
        for (int x = 0; x < target_width; ++x) {
            for (int c = 0; c < channels; ++c) {
                PIXEL(final_output, y + y_offset, x, c) = PIXEL(resized, y, x, c);
            }
        }
    }

    free(resized->data);
    free(resized);
    return final_output;
}




void convert_rpy_sensor_to_cam(
    double roll_sys1, double pitch_sys1, double yaw_offset,
    double pitch_offset, double roll_offset,
    float* roll_deg_out, float* pitch_deg_out, float* yaw_deg_out
) {
    // Convert to radians
    double roll_rad = roll_sys1 * M_PI / 180.0;
    double pitch_rad = pitch_sys1 * M_PI / 180.0;
    double roll_offset_rad = roll_offset * M_PI / 180.0;
    double pitch_offset_rad = pitch_offset * M_PI / 180.0;
    double yaw_offset_rad = yaw_offset * M_PI / 180.0;

    // Rotation matrix Rx (roll)
    double Rx[3][3] = {
        {1, 0, 0},
        {0, cos(roll_rad), -sin(roll_rad)},
        {0, sin(roll_rad),  cos(roll_rad)}
    };

    // Rotation matrix Ry (pitch)
    double Ry[3][3] = {
        {cos(pitch_rad), 0, sin(pitch_rad)},
        {0, 1, 0},
        {-sin(pitch_rad), 0, cos(pitch_rad)}
    };

    // R_sys1 = Rx * Ry
    double R_sys1[3][3];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            R_sys1[i][j] = Rx[i][0] * Ry[0][j] + Rx[i][1] * Ry[1][j] + Rx[i][2] * Ry[2][j];

    // Rotation matrices for offsets
    double R_roll_offset[3][3] = {
        {1, 0, 0},
        {0, cos(roll_offset_rad), -sin(roll_offset_rad)},
        {0, sin(roll_offset_rad),  cos(roll_offset_rad)}
    };
    double R_pitch_offset[3][3] = {
        {cos(pitch_offset_rad), 0, sin(pitch_offset_rad)},
        {0, 1, 0},
        {-sin(pitch_offset_rad), 0, cos(pitch_offset_rad)}
    };
    double R_yaw_offset[3][3] = {
        {cos(yaw_offset_rad), -sin(yaw_offset_rad), 0},
        {sin(yaw_offset_rad),  cos(yaw_offset_rad), 0},
        {0, 0, 1}
    };

    // R_offset = R_yaw * R_pitch * R_roll
    double R_temp[3][3], R_offset[3][3];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            R_temp[i][j] = R_yaw_offset[i][0] * R_pitch_offset[0][j] +
            R_yaw_offset[i][1] * R_pitch_offset[1][j] +
            R_yaw_offset[i][2] * R_pitch_offset[2][j];

    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            R_offset[i][j] = R_temp[i][0] * R_roll_offset[0][j] +
            R_temp[i][1] * R_roll_offset[1][j] +
            R_temp[i][2] * R_roll_offset[2][j];

    // Final rotation R_cam(i) = R_sensor * R_offset
    double R_sysX[3][3];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            R_sysX[i][j] = R_sys1[i][0] * R_offset[0][j] +
            R_sys1[i][1] * R_offset[1][j] +
            R_sys1[i][2] * R_offset[2][j];

    // Extract Euler angles
    double pitchX = asin(-R_sysX[2][0]);
    double cy = cos(pitchX);
    double rollX, yawX;

    if (fabs(cy) > 1e-6) {
        rollX = atan2(R_sysX[2][1], R_sysX[2][2]);
        yawX = atan2(R_sysX[1][0], R_sysX[0][0]);
    }
    else {
        rollX = 0;
        yawX = atan2(-R_sysX[0][1], R_sysX[1][1]);
    }

    *roll_deg_out = (float)(rollX * 180.0 / M_PI);
    *pitch_deg_out = (float)(pitchX * 180.0 / M_PI);
    *yaw_deg_out = (float)(yawX * 180.0 / M_PI);
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

std::string SaveImageFilePath(const std::wstring& captureDir, const std::wstring& prefix)
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







float clamp(float val) {
    return (val > 5.0f) ? 5.0f : (val < -5.0f) ? -5.0f : val;
}


int main() {
    
    
    
    // Get base path
    wchar_t szPath[MAX_PATH] = L"";
    GetModuleFileNameW(NULL, szPath, MAX_PATH);

    // Convert to std::filesystem::path
    std::filesystem::path exePath(szPath);
    std::filesystem::path imuIniPath = exePath;
    std::filesystem::path paramsIniPath = exePath;
    imuIniPath.replace_extension(L".ini");                 // IMU_Stabilize_Gyro.ini
    paramsIniPath.replace_filename(L"parameters.ini");     // parameters.ini in same folder

    // Create INI managers for both
    INIManager imuIni(imuIniPath);
    INIManager paramIni(paramsIniPath);


    


    // === Read from parameters.ini ===
    WIDTH = paramIni.ReadInteger(L"image", L"width", 1920);
    HEIGHT = paramIni.ReadInteger(L"image", L"height", 1080);
    NUM_CAMS = paramIni.ReadInteger(L"image", L"num_cams", 6);
    BUFFER_LINES = paramIni.ReadInteger(L"image", L"buffer_lines", 15);

    fx = paramIni.ReadFloat(L"calibration", L"fx", 1204.505857f);
    fy = paramIni.ReadFloat(L"calibration", L"fy", 1204.505857f);
    float cx_ratio = paramIni.ReadFloat(L"calibration", L"cx_ratio", 0.5f);
    float cy_ratio = paramIni.ReadFloat(L"calibration", L"cy_ratio", 0.5f);
    cx = WIDTH * cx_ratio;
    cy = HEIGHT * cy_ratio;

    K1 = paramIni.ReadFloat(L"calibration", L"k1", -0.343698f);
    K2 = paramIni.ReadFloat(L"calibration", L"k2", 0.105503f);
    K3 = paramIni.ReadFloat(L"calibration", L"k3", 0.0f);
    P1 = paramIni.ReadFloat(L"calibration", L"p1", -0.001558f);
    P2 = paramIni.ReadFloat(L"calibration", L"p2", -0.002416f);

    cylindrical_H_fov_deg = paramIni.ReadFloat(L"projection", L"cylindrical_H_fov_deg", 60.0f);
    cylindrical_V_fov_deg = paramIni.ReadFloat(L"projection", L"cylindrical_V_fov_deg", 51.3f);

    panorama_crop_height_scale = paramIni.ReadFloat(L"cropping", L"panorama_crop_height_scale", 0.80f);
    panorama_crop_width_scale = paramIni.ReadFloat(L"cropping", L"panorama_crop_width_scale", 0.80f);
    original_crop_height_scale = paramIni.ReadFloat(L"cropping", L"original_crop_height_scale", 0.70f);
    original_crop_width_scale = paramIni.ReadFloat(L"cropping", L"original_crop_width_scale", 0.54f);

    PITCH_TO_TRANSLATION_FACTOR = paramIni.ReadFloat(L"transform", L"pitch_to_translation_factor", -17.44153917818162f);
    PITCH_TO_TRANSLATION_FACTOR *= resize_factor; // when resizing applied
    YAW_TO_TRANSLATION_FACTOR = paramIni.ReadFloat(L"transform", L"yaw_to_translation_factor", 17.84f);

    overlap_px = paramIni.ReadInteger(L"blend", L"overlap_px ", 100);
    
    resize_factor = paramIni.ReadFloat(L"resize", L"resize_factor ", 0.5);


    // === Read from IMU_Stabilize_Gyro.ini ===
    PathRemoveFileSpecW(szPath);
    std::wstring dirCapture = std::wstring(szPath) + L"\\Capture";
    CreateDirectoryW(dirCapture.c_str(), NULL);

    std::vector<std::string> filenames;
    for (int i = 0; i < NUM_CAMS; ++i) {
        std::wstring key = L"file_" + std::to_wstring(i);
        std::wstring val = imuIni.ReadString(L"images", key, L"");
        filenames.push_back(WStringToUtf8(val));
    }

    std::vector<float> yaw_offset_cam = imuIni.ReadFloatArray(L"offset", L"yaw");
    std::vector<float> pitch_offset_cam = imuIni.ReadFloatArray(L"offset", L"pitch");
    std::vector<float> roll_offset_cam = imuIni.ReadFloatArray(L"offset", L"roll");



    //-----------------------------Read in realtime later------------------------------
    Stabilization = imuIni.ReadBoolean(L"Sensor_Orientation", L"stabilization", true);

    if (Stabilization) {
        roll_Sensor = imuIni.ReadFloat(L"Sensor_Orientation", L"roll", 0.0f);
        pitch_Sensor = imuIni.ReadFloat(L"Sensor_Orientation", L"pitch", 0.0f);
        yaw_Sensor = imuIni.ReadFloat(L"Sensor_Orientation", L"yaw", 0.0f);

        // Clamp values to ±5 degrees
        roll_Sensor = clamp(roll_Sensor);
        pitch_Sensor = clamp(pitch_Sensor);
        yaw_Sensor = clamp(yaw_Sensor);
    }
    else {
        roll_Sensor = 0.0f;
        pitch_Sensor = 0.0f;
        yaw_Sensor = 0.0f;
    }

    //----------------------------------------------------------------------------------

    // ===== DEBUG PRINT: IMU INI values =====
    printf("\n[DEBUG] Sensor Orientation:\n");
    printf("  Roll:  %.3f\n", roll_Sensor);
    printf("  Pitch: %.3f\n", pitch_Sensor);
    printf("  Yaw:   %.3f\n", yaw_Sensor);

    printf("\n[DEBUG] Camera Offsets:\n");
    for (int i = 0; i < yaw_offset_cam.size(); ++i) {
        printf("  Cam %d - Yaw: %.3f, Pitch: %.3f, Roll: %.3f\n",
            i,
            i < yaw_offset_cam.size() ? yaw_offset_cam[i] : -999.0f,
            i < pitch_offset_cam.size() ? pitch_offset_cam[i] : -999.0f,
            i < roll_offset_cam.size() ? roll_offset_cam[i] : -999.0f
        );
    }

    // ===== DEBUG PRINT: Parameters INI values =====
    printf("\n[DEBUG] Image & Calibration Parameters:\n");
    printf("  WIDTH: %d, HEIGHT: %d, NUM_CAMS: %d, BUFFER_LINES: %d\n", WIDTH, HEIGHT, NUM_CAMS, BUFFER_LINES);
    printf("  fx: %.3f, fy: %.3f, cx: %.3f, cy: %.3f\n", fx, fy, cx, cy);
    printf("  K1: %.6f, K2: %.6f, K3: %.6f\n", K1, K2, K3);
    printf("  P1: %.6f, P2: %.6f\n", P1, P2);

    printf("\n[DEBUG] Projection & Cropping:\n");
    printf("  FOV: %.2f°\n", cylindrical_H_fov_deg);
    printf("  Panorama Crop (H x W): %.2f x %.2f\n", panorama_crop_height_scale, panorama_crop_width_scale);
    printf("  Original Crop (H x W): %.2f x %.2f\n", original_crop_height_scale, original_crop_width_scale);

    printf("\n[DEBUG] Transform & Blending:\n");
    printf("  Pitch to Translation: %.6f\n", PITCH_TO_TRANSLATION_FACTOR);
    printf("  Yaw to Translation:   %.6f\n", YAW_TO_TRANSLATION_FACTOR);
    printf("  Overlap Ratio:        %.2f\n", OVERLAP_RATIO);
    printf("  NON_OVERLAP_WIDTH:    %d\n", NON_OVERLAP_WIDTH);
    printf("  PANORAMA_WIDTH:       %d\n", PANORAMA_WIDTH);


    printf("\n........Processing........\n");


    std::vector<Image*> frames(NUM_CAMS);
    std::vector<Image*> stabilized_frames;
    std::vector<Image*> original_frames;
    Image* map_x, * map_y;
    std::filesystem::path mapx_path = exePath.parent_path() / "map_x.bin";
    std::filesystem::path mapy_path = exePath.parent_path() / "map_y.bin";

    //if (std::filesystem::exists(mapx_path) && std::filesystem::exists(mapy_path)) {
        loadCylindricalMaps(&map_x, &map_y, WIDTH * resize_factor, HEIGHT * resize_factor, exePath);
    /*}
    else {
        generateCylindricalRemap(&map_x, &map_y, WIDTH * resize_factor, HEIGHT * resize_factor, cylindrical_H_fov_deg, cylindrical_V_fov_deg, 0.0f, exePath);
    }*/


    if (filenames.size() != NUM_CAMS) {
        printf("[ERROR] filenames.size() = %zu, but NUM_CAMS = %d\n", filenames.size(), NUM_CAMS);
        exit(1);
    }
    
    for (int cam_idx = 0; cam_idx < NUM_CAMS; ++cam_idx) {
        
        
        float roll_cam = 0, pitch_cam = 0, yaw_cam = 0;
        convert_rpy_sensor_to_cam(
            roll_Sensor, pitch_Sensor,
            yaw_offset_cam[cam_idx],
            pitch_offset_cam[cam_idx],
            roll_offset_cam[cam_idx],
            &roll_cam, &pitch_cam, &yaw_cam
        );

        int y_shift = (int)(PITCH_TO_TRANSLATION_FACTOR * pitch_cam);
        frames[cam_idx] = readBMP(filenames[cam_idx].c_str());

        if (!frames[cam_idx]) {
            printf("[ERROR] Failed to read image for camera %d: %s\n", cam_idx, filenames[cam_idx].c_str());
            continue;
        }

        printf("[INFO] Camera %d: Width = %d, Height = %d\n",
            cam_idx, frames[cam_idx]->width, frames[cam_idx]->height);

            
        // The frame processing pipeline 
        Image* undistorted = Undistort(frames[cam_idx]);
        Image* cropped = processFramePipeline(
            undistorted, map_x, map_y,
            resize_factor,
            -roll_cam,
            -(float)y_shift,
            panorama_crop_height_scale,
            panorama_crop_width_scale
        );


        stabilized_frames.push_back(cropped);
        

        //// Save each original frame
        wchar_t label[32];
        //swprintf_s(label, L"original_cam%d", cam_idx);
        //std::string frame_path = SaveImageFilePath(dirCapture, label);
        //writeBMP(frame_path.c_str(), f_orig);
        //printf("[Saved Original] %s\n", frame_path.c_str());

        //// Save each undistorted frame
        //
        swprintf_s(label, L"undistorted_cam%d", cam_idx);
        std::string frame_path2 = SaveImageFilePath(dirCapture, label);
        writeBMP(frame_path2.c_str(), undistorted);
        printf("[Saved undistorted] %s\n", frame_path2.c_str());

        //
        free(undistorted->data); free(undistorted);
       
    }



    Image* panorama = featherBlendFit(stabilized_frames, overlap_px * resize_factor, 3840, 480);



   

    
    std::string result_path = SaveImageFilePath(dirCapture, L"panorama");

    writeBMP(result_path.c_str(), panorama);

    

    free(map_x->data); free(map_x);
    free(map_y->data); free(map_y);

    return 0;
}
