/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2023. All rights reserved.
 * Description: 地图数据处理实现。
 * Author: 张元龙
 * Date: 2023-04-28
 */

#pragma once
#include <stdio.h>
#include <vector>
#include "alg.h"
#include "Comm.h"

constexpr int MAP_DISCRETE_MULS = 2;
constexpr float DISCRETE_SIZE = CELL_SIZE / MAP_DISCRETE_MULS;

constexpr int MAP_DISCRETE_HEIGHT = MAP_FILE_ROW_NUMS * MAP_DISCRETE_MULS + 1;
constexpr int MAP_DISCRETE_WIDTH = MAP_FILE_COL_NUMS * MAP_DISCRETE_MULS + 1;

constexpr float CONFLICT_LENGTH = 8; // 冲突检测距离



inline float ToRealDist(int dist)
{
    return dist * (DISCRETE_SIZE / DIR_DIST[0]);
}

// 离散坐标转换为物理坐标
inline Vec2 DiscreteToPos(int row, int col)
{
    return Vec2{col * DISCRETE_SIZE, MAP_HEIGHT - row * DISCRETE_SIZE};
}

// 地图坐标转换为物理坐标
inline Vec2 MapToPos(int row, int col)
{
    return Vec2{(col + 0.5f) * CELL_SIZE, (MAP_FILE_ROW_NUMS - row - 0.5f) * CELL_SIZE};
}

// 物理坐标转换为地图行列
inline RowCol PosToMap(Vec2 pos)
{
    return {int((MAP_HEIGHT - pos.y) / CELL_SIZE), int(pos.x / CELL_SIZE)};
}

// 物理坐标转换为离散坐标
inline RowCol PosToDiscrete(Vec2 pos)
{
    return {int((MAP_HEIGHT - pos.y) / DISCRETE_SIZE + 0.5), int(pos.x / DISCRETE_SIZE + 0.5)}; // 四舍五入
}

// 物理坐标转换为离散坐标
inline void PosToDiscrete(Vec2 pos, RowCol out[4])
{
    out[0] = {int((MAP_HEIGHT - pos.y) / DISCRETE_SIZE + 0.5), int(pos.x / DISCRETE_SIZE + 0.5)}; // 四舍五入

    out[1] = out[2] = out[3] = out[0];

    auto realPos = DiscreteToPos(out[0].r, out[0].c);

    if (pos.x < realPos.x) {
        out[1].c--;
        out[3].c--;
    } else {
        out[1].c++;
        out[3].c++;
    }

    if (pos.y < realPos.y) {
        out[2].r++;
        out[3].r++;
    } else {
        out[2].r--;
        out[3].r--;
    }
}

struct Map {
    bool SetMap(char mapData[MAP_FILE_ROW_NUMS][MAP_FILE_COL_NUMS]);

    bool CanReachDiscrete(int row, int col, int allowBit) const;

    char GetChar(int row, int col) const;

    bool IsWall(int row, int col) const;

    // 判定s 是否能直达 t
    bool CanDirectMove(Vec2 s, Vec2 t, float radius) const;

    // 基于起点pos, 从begin - end队列中判定一个最近能到达的点
    template <typename ITER> ITER GetDirectPos(Vec2 pos, float radius, ITER begin, ITER end) const
    {
        while (begin + 1 < end) {
            auto mid = begin + ((end - begin) >> 1);
            if (CanDirectMove(pos, *mid, radius)) {
                begin = mid;
            } else {
                end = mid;
            }
        }
        return begin;
    }

    // 优化路径, 避免弯弯绕绕, 至少优化10米和未来的2个点
    void OptimizePath(Vec2 pos, float radius, std::vector<Vec2> &path) const;

    // 路径冲突检查 0 表示不冲突, 否则表示一个冲突位表
    uint8_t CheckConflict(int robotId, Vec2 pos, std::vector<Vec2> &path);

    void ResetConflict();

    bool MapValid(RowCol rc) const;

    static bool DiscreteValid(RowCol rc);

private:
    bool IntersectWall(const Line &l) const;
    

    void InitDiscrete();

    // 判断地图上某个坐标能否容纳半径为radius的圆
    bool CanReach(Vec2 pos, float radius) const;

    char m_mapData[MAP_FILE_ROW_NUMS][MAP_FILE_COL_NUMS]{0};
    int8_t m_discrete[MAP_DISCRETE_HEIGHT][MAP_DISCRETE_WIDTH]{0};
    std::vector<Line> walls;

public:
    uint8_t m_conflictBits[MAP_DISCRETE_HEIGHT][MAP_DISCRETE_WIDTH]{0};
    uint8_t m_conflictRobot[MAP_DISCRETE_HEIGHT][MAP_DISCRETE_WIDTH];
};
