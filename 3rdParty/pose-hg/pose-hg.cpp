#include "pose-hg.h"

#include <stdio.h>
#include <string>

// Torch 7
#include "lua.hpp"
#include "TH/TH.h"
#include "luaT.h"

// impl pattern
class PoseHg::Impl {
public:
    Impl() {
        L = lua_open();
        luaL_openlibs(L);

        // require "torch"
        lua_getglobal(L, "require");
        lua_pushstring(L, "torch");
        lua_pcall(L, 1, 0, 0);

        // Load lua file
        if (luaL_dofile(L, "../3rdParty/pose-hg/pose-hg.lua")) {
            std::string error_message = lua_tostring(L, -1);
            printf("Failed to load 'pose-hg.lua'\n");
            printf("Eror massage: %s\n", error_message.c_str());
            exit(1);
        }
    }
    ~Impl() {}
    void estimate(const cv::Mat& src, std::vector<float>& coords) {
        CV_DbgAssert(src.channels() == 3 && src.type() == CV_32F);
        const int H = src.rows;
        const int W = src.cols;
        const int C = 3;

        const int LEN = H * W * C;
        const int STRIDE_1 = H * W;
        const int STRIDE_2 = W;
        const int STRIDE_3 = 1;

        float *input_data = new float[LEN];
        // Change array order
        for (int y = 0; y < H; y++) {
            for (int x = 0; x < W; x++) {
                const cv::Vec3f& col = src.at<cv::Vec3f>(y, x);
                for (int c = 0; c < C; c++) {
                    int dst_idx = c * STRIDE_1 + y * STRIDE_2 + x * STRIDE_3;
                    input_data[dst_idx] = col[c];
                }
            }
        }

        // Memory released when tensor input is freed by lua garbadge collector
        THFloatStorage *storage = THFloatStorage_newWithData(input_data, LEN);
        THFloatTensor* input =
            THFloatTensor_newWithStorage3d(storage, 0, // Offset
                                           C, STRIDE_1, // Channels
                                           H, STRIDE_2, // Height
                                           W, STRIDE_3); // Width

        // The function name
        lua_getglobal(L, "predict");

        // Send the tensor to lua
        luaT_pushudata(L, (void*)input, "torch.FloatTensor");

        // call the function with 1 arguments, return 1 result
        lua_call(L, 1, 1);

        // Copy to destination
        THFloatTensor* output =
            (THFloatTensor*)luaT_toudata(L, -1 ,"torch.FloatTensor");
        coords.resize(output->storage->size);
        for (int i = 0; i < output->storage->size; i++) {
            coords[i] = output->storage->data[i];
        }

        THFloatTensor_free(output);
//         THFloatTensor_free(input);  // segmentation fault
        THFloatStorage_free(storage);

//         delete [] input_data;  // segmentation fault
    }

private:
    lua_State* L;
};

PoseHg::PoseHg() { impl = new Impl; }
PoseHg::~PoseHg() { delete impl; }

void PoseHg::estimate(const cv::Mat& src, std::vector<float>& coords) {
    impl->estimate(src, coords);
}
