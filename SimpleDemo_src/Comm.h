/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2023. All rights reserved.
 * Description: 一些通用定义。
 * Author: 张元龙
 * Date: 2023-04-28
 */

#pragma once
#include <stdint.h>
#include <time.h>
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN // 从 Windows 头文件中排除极少使用的内容
#include <Windows.h>
#else
#include <unistd.h>
#include <sys/time.h>
#include <sys/types.h>
#include <fcntl.h>
#include <errno.h>
#include <signal.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/types.h>
#endif

#include "alg.h"

// 机器人状态
enum RobotStat{
    STAT_NORMAL_ROBOT = 0,
    STAT_CARRY_ROBOT = 1
};

// 行列定义
struct RowCol {
    int r, c;

    bool operator==(const RowCol &b) const
    {
        return r == b.r && c == b.c;
    }

    RowCol operator+(const RowCol &b) const
    {
        return {r + b.r, c + b.c};
    }

    RowCol &operator+=(const RowCol &b)
    {
        r += b.r;
        c += b.c;
        return *this;
    }
};

constexpr float SQRT_0_5 = 0.70710678f;

// 8 个遍历方向
enum {DIR_DOWN, DIR_UP, DIR_RIGHT, DIR_LEFT, DIR_LEFT_UP, DIR_RIGHT_DOWN, DIR_RIGHT_UP, DIR_LEFT_DOWN};
constexpr Vec2 CIRCLE[8] = { {0, -1}, {0, 1}, {1, 0}, {-1, 0}, {-SQRT_0_5, SQRT_0_5}, {SQRT_0_5, -SQRT_0_5}, {SQRT_0_5, SQRT_0_5}, {-SQRT_0_5, -SQRT_0_5} };
constexpr RowCol DIR[8] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {-1, -1}, {1, 1}, {-1, 1}, {1, -1}};
constexpr int DIR_DIST[8] = {10000, 10000, 10000, 10000, 14142, 14142, 14142, 14142}; // 直角边和斜边距离

inline uint64_t GetMillTime()
{
#ifdef _WIN32
    static LARGE_INTEGER s_Freq, s_FirstTime;

    if (s_Freq.QuadPart == 0) {
        QueryPerformanceFrequency(&s_Freq);
        // assert(QueryPerformanceCounter(&s_FirstTime));
    }

    LARGE_INTEGER lTime;
    QueryPerformanceCounter(&lTime);

    return (lTime.QuadPart - s_FirstTime.QuadPart) * 1000 / s_Freq.QuadPart;
#else
    timespec stStartTime;
    clock_gettime(CLOCK_MONOTONIC, &stStartTime);
    return (uint64_t)stStartTime.tv_sec * 1000ull + stStartTime.tv_nsec / 1000000;
#endif
}