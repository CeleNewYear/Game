/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2023. All rights reserved.
 * Description: 常量定义。
 * Author: 张元龙
 * Date: 2023-04-28
 */

#pragma once
#include <assert.h>
constexpr float PI = 3.14159265358979;

constexpr float MAP_HEIGHT = 50;    // 地图高度, 单位
constexpr float MAP_WIDTH = 50;     // 地图宽度, 单位
constexpr int MAP_FILE_ROW_NUMS = 100;    // 地图文件行数
constexpr int MAP_FILE_COL_NUMS = 100;    // 地图文件列数

constexpr float CELL_SIZE = MAP_HEIGHT / MAP_FILE_ROW_NUMS;   // 每个地图格子的高度和宽度

static_assert(MAP_HEIGHT * MAP_FILE_COL_NUMS  == MAP_WIDTH * MAP_FILE_ROW_NUMS, "check cell size err");

constexpr int FPS = 50;                   // 每秒运行帧数

constexpr int ROBOTS_PER_PLAYER = 4;    // 每个玩家的机器人数


// 雷达划分的条数
constexpr int RADAR_PARTS = 360;

// 机器人定义
constexpr float ROBOT_RADIUS = 0.45;     // 机器人半径
constexpr float ROBOT_CARRY_RADIUS = 0.53;     // 持有物品的机器人半径
constexpr float ROBOT_DENSITY_BASE = 20;     // 机器人密度

constexpr float MAX_FORWARD_VELOCITY_BASE[2] = {6, 7};                      // 最大前进速度 (m/s)
constexpr float MIN_FORWARD_VELOCITY_BASE = -MAX_FORWARD_VELOCITY_BASE[0] / 3;  // 最小前进速度 (m/s) 即后退速度

constexpr float MAX_ROTATION_VELOCITY_BASE = PI;     // 最大旋转速度 (弧度/s)
constexpr float MAX_FORCE_BASE = 250;               // 最大牵引力, 决定加速能力

constexpr float MAX_TORQUE_BASE = 50;              // 最大力矩, 决定旋转加速能力
