/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2023. All rights reserved.
 * Description: 背景线程类定义，背景线程负责在后台不停的刷新雷达扫描的地图数据和最短路数据，
 *   主线程只需要查询即可，这样可避免主线程出现跳帧的情况。
 * Author: 张元龙
 * Date: 2023-04-28
 */
#include <memory>
#include <vector>
#include "BackgroundThread.h"

using namespace std;
// 位表操作
#define TestBit(bitmap, idx) (((uint8_t *)(bitmap))[(idx) >> 3] & (1 << (idx & 7)))
#define FlipBit(bitmap, idx) (((uint8_t *)(bitmap))[(idx) >> 3] ^= (1 << (idx & 7)))
#define SetBit(bitmap, idx) (((uint8_t *)(bitmap))[(idx) >> 3] |= (1 << (idx & 7)))
#define ClearBit(bitmap, idx) (((uint8_t *)(bitmap))[(idx) >> 3] &= ~(1 << (idx & 7)))

inline bool ValidRC(int row, int col) 
{
    return row >= 0 && col >= 0 && row < MAP_FILE_ROW_NUMS && col < MAP_FILE_COL_NUMS;
}

void BackgroundThread::Init()
{
    m_thread = make_unique<thread>(&BackgroundThread::ThreadMain, this);

}

void BackgroundThread::SetMapData(char mapData[MAP_FILE_ROW_NUMS][MAP_FILE_COL_NUMS])
{
    m_staticMap = true;
    m_result.map = make_shared<Map>();
    m_result.workbenchDij = make_shared<std::vector<Dijkstra>>();
    bool ret = m_result.map->SetMap(mapData);
    assert(ret);

    std::vector<Input::Workbench> workbench;

    for (int i = 0; i < MAP_FILE_ROW_NUMS; i++)
        for (int j = 0; j < MAP_FILE_COL_NUMS; j++) {
            if (mapData[i][j] >= '1' && mapData[i][j] <= '9') {
                workbench.push_back({mapData[i][j] - '0', MapToPos(i, j)});
            }
        }
    UpdateDijkstra(workbench, &m_result);

    m_staticMap = true; // 静态地图
}

bool BackgroundThread::UpdateLidar(const Input &input, Output *result)
{
    bool updateMap = false;

    for (int i = 0; i < ROBOTS_PER_PLAYER; i++) {
        const auto &robot = input.robot[i];

        Vec2 units[RADAR_PARTS];
        Vec2 lidarPos[RADAR_PARTS];

        // 可通行格子扫描
        for (int k = 0; k < robot.lidar.size(); k++) {

            float angle = robot.angle + k * 2 * PI / robot.lidar.size();
            angle = StandardizingAngle(angle);
            float x = cosf(angle);
            float y = sinf(angle);

            Vec2 unit(x, y);
            units[k] = unit;
            lidarPos[k] = robot.pos + unit * robot.lidar[k];
            constexpr float STEP = 0.177777771;
            unit *= STEP;
            auto p = robot.pos + unit;

            for (float len = STEP; len < robot.lidar[k]; len += STEP, p += unit) {
                
                if ((int(p.x * 2000) + 1) % 1000 <= 2 || ((int(p.y * 2000) + 1) % 1000 <= 2)) {
                    continue;   // 边界线上的情况
                }

                // 转换到格子
                int col = p.x * 2;
                int row = (MAP_HEIGHT - p.y) * 2;
                if (ValidRC(row, col) && !TestBit(m_canReach[row], col)) {
                    SetBit(m_canReach[row], col);
                    updateMap = true;
                }

            }
        }

        // 障碍物扫描
        for (int k = 0; k < robot.lidar.size(); k++) {
            auto pre = lidarPos[(k + robot.lidar.size() - 1) % robot.lidar.size()];
            auto p = lidarPos[k];
            auto next = lidarPos[(k + 1) % robot.lidar.size()];

            // 判断三点共线

            if (FloatEqual(p.x, pre.x) && FloatEqual(p.x, next.x) && ((int(p.x * 2000) + 1) % 1000 <= 2) ||
                FloatEqual(p.y, pre.y) && FloatEqual(p.y, next.y) && ((int(p.y * 2000) + 1) % 1000 <= 2)) {
                // 障碍物
                p += units[k] * 0.001;
                int col = p.x * 2;
                int row = (MAP_HEIGHT - p.y) * 2;
                if (ValidRC(row, col) && !TestBit(m_canReach[row], col) && !TestBit(m_obstacle[row], col)) {
                    SetBit(m_obstacle[row], col);
                    updateMap = true;
                }
            }  
        }
    }
    return updateMap;
}

// 处理激光扫描后的地图数据
void BackgroundThread::UpdateMap(const Input &input, Output *result)
{
    char mapData[MAP_FILE_ROW_NUMS][MAP_FILE_COL_NUMS];

    for (int i = 0; i < MAP_FILE_ROW_NUMS; i++) 
        for (int j = 0; j < MAP_FILE_COL_NUMS; j++) {
            mapData[i][j] = !TestBit(m_canReach[i], j) && TestBit(m_obstacle[i], j) ? '#' : '.';
        }

    for (auto &it : input.workbench) {
        auto rc = PosToMap(it.pos);
        assert(ValidRC(rc.r, rc.c));
        mapData[rc.r][rc.c] = it.type + '0';    // 工作台在地图上的标记
    }
    result->map = make_shared<Map>();
    bool ret = result->map->SetMap(mapData);
    assert(ret);
}

// 重新计算各个工作台的路径
void BackgroundThread::UpdateDijkstra(const std::vector<Input::Workbench> &workbench, Output *result)
{
    result->workbenchDij = make_shared<std::vector<Dijkstra>>();
    result->workbenchDij->resize(workbench.size());

    for (int i = 0; i < workbench.size(); i++) {
        (*result->workbenchDij)[i].Init(workbench[i].pos, result->map.get());
        (*result->workbenchDij)[i].Update(STAT_NORMAL_ROBOT);
        (*result->workbenchDij)[i].Update(STAT_CARRY_ROBOT);
    }

}

void BackgroundThread::ThreadMain()
{
    while (!m_exited) {
        Input *input = SwapInput(nullptr);
        if (input == nullptr) {
            // not update
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            //std::this_thread::yield();
            continue;
        }
        m_working = true;
        Output result;
        bool updateMap = false;
        if (!m_staticMap) {
            updateMap = UpdateLidar(*input, &result);
            if (updateMap) {
                UpdateMap(*input, &result);
                UpdateDijkstra(input->workbench, &result);
                std::lock_guard<mutex> lck(m_lock);
                m_result = move(result);
            }
        }
        delete input;
        m_working = false;
    }

}

bool BackgroundThread::UpdateResult()
{   
    m_lock.lock();
    m_returnResult = m_result;
    m_lock.unlock();
    if (!m_returnResult.map || !m_returnResult.workbenchDij) {
        return false;
    }
    for (auto &it : *m_returnResult.workbenchDij) {
        it.ResetMap(m_returnResult.map.get());
    }
    return true;
}

BackgroundThread &BackgroundThread::Instance()
{
    static BackgroundThread obj;
    return obj;
}
