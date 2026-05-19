/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2023. All rights reserved.
 * Description: 背景线程类定义，背景线程负责在后台不停的刷新雷达扫描的地图数据和最短路数据，
 *   主线程只需要查询即可，这样可避免主线程出现跳帧的情况。
 * Author: 张元龙
 * Date: 2023-04-28
 */

#pragma once
#include <stdint.h>
#include <atomic>
#include <memory>
#include <vector>
#include <thread>
#include <mutex>
#include "ConstDef.h"
#include "alg.h"
#include "Map.h"
#include "Dijkstra.h"


// 背景线程类，负责在后台计算激光扫描的地图数据、最短路等
class BackgroundThread
{
    BackgroundThread() = default;   // 单例模式

public:

    struct Output {

        std::shared_ptr<Map> map; // 当前扫描出来的地图

        std::shared_ptr<std::vector<Dijkstra>> workbenchDij; // 基于当前地图的各个工作台最短路

    };

    struct Input {
        struct Robot {
            Vec2 pos;
            float angle;
            std::vector<float> lidar;
        };

        struct Workbench {
            int type;
            Vec2 pos;
        };

        Robot robot[ROBOTS_PER_PLAYER]; // 机器人的输入信息

        std::vector<Workbench> workbench; // 工作台的位置信息, 用于求最短路
    };

    void SetMapData(char mapData[MAP_FILE_ROW_NUMS][MAP_FILE_COL_NUMS]);

    void Init();

    // 该函数用于始终多线程之间投递最新的雷达数据,以及获取最新的雷达数据
    // 如果一个已投递数据没有被另一个线程获取，那么会被第二次投递替换为更新的
    Input *SwapInput(Input *input)
    {
        return std::atomic_exchange(&m_lidarInput, input);
    }


    // 更新最新结果
    bool UpdateResult() ;

    // 获取结果
    const Output &GetResult()
    {
        return m_returnResult;
    }

    void ExitThread()
    {
        m_exited = true;
        if (m_thread) {
            m_thread->join();
        }
    }

    bool IsWorking()
    {
        return m_working;
    }

    static BackgroundThread &Instance();

private:

    bool UpdateLidar(const Input &input, Output *result);

    void UpdateMap(const Input &input, Output *result);

    void UpdateDijkstra(const std::vector<Input::Workbench> &workbench, Output *result);

    void ThreadMain();
    
    bool m_exited{false};
    bool m_staticMap{false};
    bool m_working{false};
    std::atomic<Input *> m_lidarInput{nullptr};
    uint64_t m_obstacle[MAP_FILE_ROW_NUMS][MAP_FILE_COL_NUMS / 64 + 1]{0};
    uint64_t m_canReach[MAP_FILE_ROW_NUMS][MAP_FILE_COL_NUMS / 64 + 1]{0};
    std::unique_ptr<std::thread> m_thread;
    Output m_result;
    Output m_returnResult;
    std::mutex m_lock;
    
};
