/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2023. All rights reserved.
 * Description: 主策略实现。
 * Author: 张元龙
 * Date: 2023-04-28
 */
#include <stdio.h>
#include <string>
#include <vector>
#include <array>
#include <assert.h>
#include <string.h>
#include <stdlib.h>
#include <queue>
#include <iostream>
#include <functional>
#include <queue>
#include <algorithm>
#include "alg.h"
#include "ConstDef.h"
#include "Map.h"
#include "Dijkstra.h"
#include "BackgroundThread.h"
using namespace std;


constexpr float ROBOT_MASS = ROBOT_RADIUS * ROBOT_RADIUS * PI * ROBOT_DENSITY_BASE;    // 机器人质量
constexpr float ROBOT_INERTIA = ROBOT_MASS * ROBOT_RADIUS * ROBOT_RADIUS / 2;

constexpr float ROBOT_CARRY_MASS = ROBOT_CARRY_RADIUS * ROBOT_CARRY_RADIUS * PI * ROBOT_DENSITY_BASE; // 机器人质量
constexpr float ROBOT_CARRY_INERTIA = ROBOT_CARRY_MASS * ROBOT_CARRY_RADIUS * ROBOT_CARRY_RADIUS / 2;


struct ProductAttr {
    int pid;
    int cost;
    int maxVal;
    int procTime;
};
const ProductAttr g_productArr[] = {
    {0, 99999999, 99999999, 9999}, {1, 3000, 6000, 50},    {2, 4400, 7600, 50},    {3, 5800, 9200, 50},
    {4, 15400, 22500, 500},        {5, 17200, 25000, 500}, {6, 19200, 27500, 500}, {7, 76000, 105000, 1000},
};


constexpr int MATERIAL_TABLE[10] = {
    0,
    0,
    0,
    0,
    (1 << 1) | (1 << 2),
    (1 << 1) | (1 << 3),
    (1 << 2) | (1 << 3),
    (1 << 4) | (1 << 5) | (1 << 6),
    1 << 7,
    (1 << 8) - 2,

};

static int g_gameFrames = 5 * 60 * FPS;
static int g_frameId;

// 机器人执行动作
enum RobotAction
{
    RA_BUY,
    RA_SELL,
};

#define g_map (*BackgroundThread::Instance().GetResult().map)



// 二分反函数计算, 通过一个结果查值
// f函数必须单调递增
inline float InversIncFunc(function<float (float)> f, float result, float minVal = 0, float maxVal = 10000)
{
    while (minVal + 0.0001 < maxVal) {
        float mid = (minVal + maxVal) / 2;
        if (f(mid) < result) {
            minVal = mid;
        } else {
            maxVal = mid;
        }
    }
    return (minVal + maxVal) / 2;
}


struct SpeedCtrl {
    SpeedCtrl(float m, float maxf)
    {
        this->m = m;
        this->maxf = maxf;

        distAndVel.push_back({0.0f, 0.0f});
        for (int i = 1; i < 100; i++) {
            distAndVel.push_back({distAndVel.back().dist + distAndVel.back().vel / FPS,
                                  InversIncFunc([this](float v) { return this->DecVel(v); }, distAndVel.back().vel)});
        }
    }

    struct DistVal
    {
        float dist;
        float vel;
        bool operator<(const DistVal &b) const
        {
            return dist < b.dist;
        }
    };

    float DecVel(float curv)
    {
        float t = 1.0f / FPS;
        float a = maxf / m;
        float nextv = curv - a * t;
        nextv /= (1.0f + t * 0.1);
        return nextv;
    }

    float QueryVel(float dist)
    {
        bool sign = false;
        if (dist < 0) {
            sign = true;
            dist = -dist;
        }

        if (dist <= distAndVel[2].dist) {   // 一帧内可到达
            return sign ? -dist / FPS : dist / FPS;
        }

        auto it = lower_bound(distAndVel.begin(), distAndVel.end(), DistVal{dist, 0});
        assert(it != distAndVel.begin());
        return sign ? -prev(it)->vel : prev(it)->vel;
    }

    float m;
    float maxf;
    vector<DistVal> distAndVel;

};

static SpeedCtrl g_radCtrl(ROBOT_INERTIA, MAX_TORQUE_BASE);
static SpeedCtrl g_carryRadCtrl(ROBOT_CARRY_INERTIA, MAX_TORQUE_BASE);


static SpeedCtrl g_lineVelCtrl(ROBOT_MASS, MAX_FORCE_BASE);
static SpeedCtrl g_carryLineVelCtrl(ROBOT_CARRY_MASS, MAX_FORCE_BASE);

struct Workbench
{
    int id;
    int type;
    int materialStatus;
    int origMaterialStatus;
    int productStatus;
    int productFrames;
    Vec2 pos;
    uint32_t usedStat = 0;
    bool tryProducted = false;

    float minCost;  // 生产出来的最小代价
    float sellMinCost;  // 生产出来并且卖掉的最小代价

    const Dijkstra& Dij() const
    {
        return (*BackgroundThread::Instance().GetResult().workbenchDij)[id];
    }

    void Input()
    {
        int ret = scanf("%d %f %f %d %d %d", &type, &pos.x, &pos.y, &productFrames, &materialStatus, &productStatus);
        if (type <= 3) {
            productStatus = true;
        }
        assert(ret == 6);
        usedStat = 0;
        tryProducted = false;
        origMaterialStatus = materialStatus;
    }
};

struct Adversary {  // 对手信息
    Vec2 pos;
    Vec2 lineVel;
    bool carryProduct;
    bool attacked{false};
    Adversary() = default;

    Adversary(Vec2 p, bool carry) : pos(p), lineVel(0, 0), carryProduct(carry)
    {
    }
};

struct Robot
{
    int robotId;
    int carryId;
    float timeRate;
    float impRate;
    float angleVel;
    Vec2 lineVel;
    float angle;
    Vec2 pos;
    int workbenchId;
    bool assigned = false;
    int priority = 0;
    Workbench *targetWorkbench;
    vector<Vec2> m_path;
    int beConflicted = 0;  // 被冲突
    int forcePri = 0;
    deque<Vec2> m_posList;

    float totMoved = 0; // 移动总长度

    vector<float> m_radar;
    Dijkstra    m_dij;
    void Input() 
    {
        auto origPos = pos;
        int ret = scanf("%d %d %f %f %f %f %f %f %f %f", &workbenchId, &carryId, &timeRate, &impRate, &angleVel, &lineVel.x, &lineVel.y, &angle, &pos.x, &pos.y);
        if (g_frameId > 1) {
            totMoved += Distance(origPos, pos);
        }
        assert(ret == 10);
        assigned = false;
        priority = 0;
        targetWorkbench = nullptr;
        m_path.clear();
        m_posList.push_back(pos);
        if (m_posList.size() > FPS) {
            m_posList.pop_front();
        }
    }


    bool Stuck() const
    {
        // 是否卡住
        return m_posList.size() >= 3 * FPS && (DistanceSqr(m_posList.front(), m_posList.back()) < 0.25);
    }

    float AvgSpeed() const
    {
        return totMoved / g_frameId;    
    }

    float Radius() const
    {
        return carryId ? ROBOT_CARRY_RADIUS : ROBOT_RADIUS;
    }

    void MoveTo(vector<Vec2> path)
    {
        if (path.empty())
            return;
        auto target = path[0];
        auto  vecDiff = target - pos;
        if (vecDiff.LengthSqr() < 0.02 && path.size() > 1) {
            path.erase(path.begin());   // 第一个点已经到达
            target = path[0];
            vecDiff = target - pos;
        }

        if (vecDiff.LengthSqr() < 0.01) {
            // 已到达
            return;
        }
        
        // 计算方向
        float tarAngle = StandardizingAngle(atan2f(vecDiff.y, vecDiff.x) - angle);
        
        auto *ctrl = carryId ? &g_carryRadCtrl : &g_radCtrl;
        Rotate(ctrl->QueryVel(tarAngle));

        if (lineVel.LengthSqr() > 0.01 && Dot(vecDiff, lineVel) <= 0) {
            // 线速度方向如果偏离目标则停下来
            Forward(0);
            return;
        }
        else if (tarAngle >= PI / 2 - 0.1 || tarAngle <= -PI / 2 + 0.1) {
            Forward(0); // 角度差距太大, 先停下来转向
            return;
        } 
        
        if (lineVel.LengthSqr() > 0.01) {
            // 计算最短弧线的圆心
            
            bool left = Xmult(lineVel, vecDiff) > 0;
            float r = lineVel.Length() / MAX_ROTATION_VELOCITY_BASE;
            // 圆心
            Vec2 dir = left ? Vec2(-lineVel.y, lineVel.x).Unit() : Vec2(lineVel.y, -lineVel.x).Unit();
            
            Vec2 center = pos + dir * r;
            
            for (auto it = path.begin(); it != path.end();) {
                if (DistanceSqr(center, *it) <= r * r ) {

                    // 小于最大转弯弧度, 此时要减速转弯
                    Rotate(tarAngle > 0 ? 999 : -999);

                    // 求线速度
                    float cosa = Dot(dir, vecDiff) / (vecDiff.Length());
                    float newr = vecDiff.Length() / (2 * cosa);
                    float newv = newr * MAX_ROTATION_VELOCITY_BASE;
                    Forward(newv);
                    return;
                }
                it++;
                if (it == path.end()) {
                    break;
                }
                if ((Xmult(*(it - 1), *it, pos) > 0) != left) {
                    break;
                }
            }

        }

        if (fabs(tarAngle) <= 0.01) {
            Forward(999); // 已经对准
            return;
        }

        Line l(pos, Vec2(pos.x + cosf(angle), pos.y + sinf(angle)));

        float a = DisPtoLine(target, l);
        float c_2 = vecDiff.LengthSqr();
        float b = sqrtf(c_2 - a * a);

        // 移动距离不能超过b
        ctrl = carryId ? &g_carryLineVelCtrl : &g_lineVelCtrl;
        Forward(ctrl->QueryVel(b));
    }

    void AssignPath(vector<Vec2>& path, const Map& map) {
        if (path.size() < 2) {
            return;
        }
        

        map.OptimizePath(pos, carryId ? ROBOT_CARRY_RADIUS : ROBOT_RADIUS, path);
        assert(!path.empty());

        m_path = move(path);
    }

    void MovePath()
    {
        auto &map = *BackgroundThread::Instance().GetResult().map;
        uint8_t confRobot = map.CheckConflict(robotId, pos, m_path);
        if (confRobot) {
            // 路径冲突
            for (int i = 0; i < ROBOTS_PER_PLAYER; i++) {
                if (confRobot & (1 << i)) {
                    (this - robotId + i)->beConflicted = FPS;
                    //fprintf(stderr, "Conflict:%d - %d\n", robotId, i);
                }
            }
            Dijkstra dij;
            dij.Init(pos, &map);
            vector<RowCol> result;
            auto stat = carryId ? STAT_CARRY_ROBOT : STAT_NORMAL_ROBOT;
            dij.Update(stat, map.m_conflictBits, &result, 100);

            // 选一个最优点过去
            RowCol select{-1, -1};
            int64_t minDist = 0x7f7f7f7f7f7f7f7f;
            for (auto &it : result) {
                int64_t dist = dij.GetDist(stat, it) * 2;
                if (targetWorkbench) {
                    dist += targetWorkbench->Dij().GetDist(stat, it);
                }
                if (dist < minDist) {
                    minDist = dist;
                    select = it;
                }
            }
            if (select.r == -1) {
                forcePri = 1;
                beConflicted = FPS;
            } else {
                m_path = dij.MoveToDiscrete(stat, select);
                map.OptimizePath(pos, carryId ? ROBOT_CARRY_RADIUS : ROBOT_RADIUS, m_path);
            }
            map.CheckConflict(robotId, pos, m_path);
        }

        MoveTo(m_path);
    }

    // 躲避对手
    void Dodge() 
    {
        if (carryId <= 3) return;   // 不需要躲避

    }
 

    void Forward(float v) {
        if (beConflicted > 0 && forcePri == 0 && v > 4) {
            v = 4;
        }
        printf("forward %d %f\n", robotId, v);
        //fprintf(stderr, "forward %d %f\n", robotId, v);
    }

    void Rotate(float v)
    {
        printf("rotate %d %f\n", robotId, v);
        //fprintf(stderr, "rotate %d %f\n", robotId, v);
    }

    void Buy() {
        printf("buy %d\n", robotId);
    }

    void Sell() {
        printf("sell %d\n", robotId);
    }

    void Destroy()
    {
        printf("destroy %d\n", robotId);
    }
};



struct Strategy
{
    bool InputMap()
    {
        char mapData[MAP_FILE_ROW_NUMS + 10][MAP_FILE_COL_NUMS];

        fgets(mapData[0], MAP_FILE_COL_NUMS * 2, stdin);

        if (mapData[0][0] == 'R' || mapData[0][0] == 'B') {
            // 颜色输入
            m_playerId = mapData[0][0] == 'R' ? 1 : 0;
            fgets(mapData[0], MAP_FILE_COL_NUMS * 2, stdin);
        } 

        if (mapData[0][0] == 'O') {
            // OK, 没有地图数据
            return true;
        }

        

        for (int i = 0; i < MAP_FILE_ROW_NUMS; fgets(mapData[++i], MAP_FILE_COL_NUMS * 2, stdin)) {
            if (strlen(mapData[i]) < MAP_FILE_COL_NUMS) {
                return false;
            }
            if (m_playerId) {
                for (int j = 0; j < MAP_FILE_COL_NUMS; j++) {
                    auto &it = mapData[i][j];
                    if (it == 'A') {
                        it = 'B';
                    } else if (it == 'B') {
                        it = 'A';
                    } else if (it >= 'a' && it <= 'i') {
                        it = it - 'a' + '1';

                    } else if (it >= '1' && it <= '9') {
                        it = it - '1' + 'a';
                    }
                }
            }
            
        }

        assert(mapData[MAP_FILE_ROW_NUMS][0] == 'O'); // OK
        BackgroundThread::Instance().SetMapData(mapData);

        // 判定是否初赛还是复赛
        bool isSemi = false;
        for (int i = 0; !isSemi && i < MAP_FILE_ROW_NUMS; i++) 
            for (int j = 0; j < MAP_FILE_COL_NUMS; j++) {
                if (mapData[i][j] == '#') {
                    isSemi = true;
                    break;
                }
            }
        
        g_gameFrames = isSemi ? 5 * 60 * FPS : 3 * 60 * FPS;

        return true;
    }

    void Init()
    {
        InputMap();

        for (int i = 0; i < ROBOTS_PER_PLAYER; i++) {
            m_robots[i].robotId = i;
        }
        Finish();
    }

    void InitOnFirstFrame()
    {
        // 第一帧时的初始化, 因不再给地图数据之后, 某些针对工作台的初始化只能放到第一帧

        for (auto &it : m_workbench) {
            m_typeWorkbench[it.type].push_back(&it);
        }

        memset(m_isHighestProduct, true, sizeof m_isHighestProduct);
        for (int i = 1; i <= 7; i++) {
            if (!m_typeWorkbench[i].empty()) {
                for (int j = 1; j <= 7; ++j) {
                    if (MATERIAL_TABLE[i] & (1 << j)) {
                        m_isHighestProduct[j] = false;
                    }
                }
            }
        }
    }

    void Finish()
    {
        puts("OK");
        fflush(stdout);
    }

    bool ReadOK()
    {
        char OK[256];
        scanf("%s", OK);
        if (strcmp(OK, "OK")) {
            fprintf(stderr, "check OK err: %s\n", OK);
            abort();
        }
        return true;
    }

    bool ReadUntilOK()
    {
        char line[1024];
        while (fgets(line, sizeof line, stdin)) {
            if (line[0] == 'O' && line[1] == 'K') {
                return true;
            }
        }
        return false; // EOF
    }

    bool Input()
    {
        if (scanf("%d %d", &g_frameId, &m_money) == EOF) {
            return false;
        }
        
        // 工作台
        int workbenchNums;
        scanf("%d", &workbenchNums);
        m_workbench.resize(workbenchNums);
        int id = 0;
        for (auto &it : m_workbench) {
            it.Input();
            it.id = id++;
        }
      
        // 机器人
        for (auto& it: m_robots) {
            it.Input();
        }
        
        // 雷达数据,
        char s[256];
        scanf("%s", s);
        if (strcmp(s, "OK") == 0) {
            return true;    // 如果是初赛和复赛则没有
        }

        // 决赛数据, 有雷达
        g_gameFrames = 4 * 60 * FPS;
        static BackgroundThread::Input *input;
        if (input == nullptr) {
            input = new BackgroundThread::Input();
        }

        for (int i = 0; i < ROBOTS_PER_PLAYER; i++) {
            input->robot[i].pos = m_robots[i].pos;
            input->robot[i].angle = m_robots[i].angle;
            input->robot[i].lidar.resize(RADAR_PARTS);
            for (int j = 0; j < RADAR_PARTS; ++j) {
                if (i == 0 && j == 0) {
                    sscanf(s, "%f", &input->robot[i].lidar[j]);
                } else {
                    scanf("%f", &input->robot[i].lidar[j]);
                }
            }
            m_robots[i].m_radar = input->robot[i].lidar;
        }
        
        input->workbench.clear();
        for (const auto &it : m_workbench) {
            input->workbench.push_back({it.type, it.pos});
        }

        input = BackgroundThread::Instance().SwapInput(input);

        if (g_frameId == 1) {
            BackgroundThread::Instance().Init();
        }
        CalcAdversary();
        return ReadOK();
    }

    static float GetLength(const vector<Vec2> &path)
    {
        float len = 0;
        for (int i = 1; i < path.size(); i++) {
            len += Distance(path[i], path[i - 1]);
            //TODO: 转弯因子
            len += 0.5;
        }
        return len;
    }

    
    void ParseRadarPos(Vec2 radarPos[RADAR_PARTS]) 
    {
        for (int i = 0; i < RADAR_PARTS; i++) {
            Vec2 p0 = radarPos[i];
            Vec2 p1 = radarPos[(i + 1) % RADAR_PARTS];
            Vec2 p2 = radarPos[(i + 2) % RADAR_PARTS];

            // 三点求圆
            Vec2 c;
            if (!GetCircleCenter(p0, p1, p2, c)) {
                continue;
            }

            // 半径的平方
            float radSqr = (c - p0).LengthSqr();

            // 判断半径的平方是否机器人
            bool carryProduct;
            if (fabs(radSqr - ROBOT_RADIUS * ROBOT_RADIUS) < ALG_EPS) {
                carryProduct = false;
            } else if (fabs(radSqr - ROBOT_CARRY_RADIUS * ROBOT_CARRY_RADIUS) < ALG_EPS) {
                carryProduct = true;
            } else {
                continue;
            }
            
            // 判断是不是己方机器人
            for (auto &myRobot : m_robots) {
                if (DistanceSqr(c, myRobot.pos) < 0.4 * 0.4) {
                    goto loopEnd;
                    
                }
            }
            // 判断是不是已经识别到的对方机器人
            for (auto &myRobot : m_adversary) {
                if (DistanceSqr(c, myRobot.pos) < 0.4 * 0.4) {
                    goto loopEnd;
                }
            }
            m_adversary.emplace_back(c, carryProduct);
            i += 2;
        loopEnd:;
        }
    }
    
    void CalcAdversary()
    {
        m_prevFrameAdversary = move(m_adversary);

        // 基于三点计算圆坐标
        for (const auto &robot : m_robots) {
            Vec2 radarPos[RADAR_PARTS];
            assert(robot.m_radar.size() == RADAR_PARTS);

            // 计算雷达投射点
            for (int k = 0; k < robot.m_radar.size(); k++) {
                float angle = robot.angle + k * 2 * PI / RADAR_PARTS;
                float x = robot.m_radar[k] * cosf(angle);
                float y = robot.m_radar[k] * sinf(angle);
                radarPos[k] = robot.pos + Vec2{x, y};
            }

            ParseRadarPos(radarPos);
        }

        // 基于上一帧的位置计算对方机器人的线速度
        for (auto &it : m_adversary) {
            it.attacked = false;
            Adversary *select = nullptr;
            float minDistSqr = sqr(10.0f/FPS);
            for (auto &pre : m_prevFrameAdversary) {
                float distSqr = DistanceSqr(pre.pos, it.pos);
                if (distSqr < minDistSqr) {
                    minDistSqr = distSqr;
                    select = &pre;
                }
            }
            if (select) {
                it.lineVel = (it.pos - select->pos) * FPS;
            }
        }
    }


    bool BuyStratege(Robot &robot, Workbench &workbench)
    {
        // 必须保证能卖出去才允许买
        // 主要看时间
        if (workbench.productStatus == 0 || g_frameId < g_productArr[1].procTime) {
            robot.Forward(0);   // 不走了
            return false;
        }

        int productId = workbench.type;
        do {
            if (g_frameId + 60 * FPS < g_gameFrames) {
                break; // 还差1分钟
            }
            int minDist = 0x7F7F7F7F;
            Workbench *select = nullptr;

            // 选一个最短距离的能出售工作台
            for (auto &workbench : m_workbench) {
                if ((MATERIAL_TABLE[workbench.type] & (1 << productId)) &&
                    (workbench.materialStatus & (1 << productId)) == 0) {
                    // 能卖的地方
                    int dist = workbench.Dij().GetDist(STAT_CARRY_ROBOT, robot.pos);
                    if (dist < minDist) {
                        minDist = dist;
                        select = &workbench;
                    }
                }
            }
            if (select == nullptr) {
                return false;
            }
            auto path = select->Dij().MoveFrom(robot.pos, STAT_CARRY_ROBOT);
            if (path.empty()) {
                return false;
            }
            g_map.OptimizePath(robot.pos, ROBOT_CARRY_RADIUS, path);
            float len = GetLength(path);
            
            if (robot.AvgSpeed() * (g_gameFrames - g_frameId - 4 * FPS) * 0.9 < len) {
                return false;   // 时间不够
            }

        } while (0);

        robot.Buy();
        return true;
    }


    void LastSellStrategy()
    {
        // 最后的物品出售策略
        if (g_frameId + 60 * FPS < g_gameFrames) {
            return; //  最后1分钟才看
        }

        for (auto &robot : m_robots) {
            if (robot.assigned || robot.carryId == 0)
                continue;

            priority_queue<pair<int, Workbench *>> heap;    // 始终保持距离最小的2个可出售工作台


            int minDist = 0x7F7F7F7F;
            Workbench *select = nullptr;
            int productId = robot.carryId;

            // 选一个最短距离的能出售工作台
            for (auto &workbench : m_workbench) {
                if ((MATERIAL_TABLE[workbench.type] & (1 << productId)) &&
                    (workbench.materialStatus & (1 << productId)) == 0) {
                    // 能卖的地方
                    int dist = workbench.Dij().GetDist(STAT_CARRY_ROBOT, robot.pos);

                    heap.push({dist, &workbench});
                    if (heap.size() > 2) {
                        heap.pop();
                    }
                }
            }

            if (heap.empty()) {
                continue;
            }
            
            if (heap.size() == 2) {
                auto select = heap.top().second;
                heap.pop();

                auto path = select->Dij().MoveFrom(robot.pos, STAT_CARRY_ROBOT);
                g_map.OptimizePath(robot.pos, ROBOT_CARRY_RADIUS, path);
                float len = GetLength(path);

                if (robot.AvgSpeed() * (g_gameFrames - g_frameId - 5 * FPS) * 0.8 >= len) {
                    continue;
                }
            }
            // 第二条路时间不够了, 就只能去第一条路
            AssignRobot(*heap.top().second, robot, RA_SELL);
        }

    
    }


    bool AssignRobot(Workbench &workbench, Robot &robot, RobotAction action)
    {
        if (robot.workbenchId == workbench.id) {
            // 已经位于工作台
            robot.assigned = true;
            if (action == RA_BUY)
                BuyStratege(robot, workbench);
            else
                robot.Sell();
            if (action == RA_SELL && workbench.productStatus && m_isHighestProduct[workbench.type]) {
                BuyStratege(robot, workbench);// 顺带买了去卖
            }
            if (action == RA_SELL && workbench.type <= 7) {
                workbench.materialStatus |= 1 << robot.carryId;
            } else if (action == RA_BUY && workbench.type > 3) {
                if (workbench.productStatus) {
                    workbench.productStatus = 0;
                } else {
                    workbench.productFrames = -1;
                }
            }
            return true;
        }

        vector<Vec2> path =
            workbench.Dij().MoveFrom(robot.pos, robot.carryId ? STAT_CARRY_ROBOT : STAT_NORMAL_ROBOT);
        if (path.empty()) {
            return false; // 不可达
        }

        robot.assigned = true;
        robot.targetWorkbench = &workbench;
        robot.priority = workbench.type;
        robot.AssignPath(path, g_map);
        if (action == RA_SELL && workbench.type <= 7) {
            workbench.materialStatus |= 1 << robot.carryId;
        }
        else if (action == RA_BUY && workbench.type > 3) {
            if (workbench.productStatus) {
                workbench.productStatus = 0;
            } else {
                workbench.productFrames = -1;
            }
        }
        return true;
    }


    // 贪心出售策略
    bool GreedySell() 
    {
        struct Stat {
            Robot *robot;
            Workbench *workben;
            float profit;
            bool operator<(const Stat &b) const
            {
                return profit > b.profit;
            }
        };

        vector<Stat> stat;

        for (auto &robot : m_robots) {
            if (robot.assigned || robot.carryId == 0) {
                continue;
            }
            
            // 选一个最近的能卖的
            Workbench *select = nullptr;
            int minDist = 0x7F7F7F7F;
            for (auto &workbench : m_workbench) {
                if ((MATERIAL_TABLE[workbench.type] & (1 << robot.carryId)) &&
                    !(workbench.materialStatus & (1 << robot.carryId))) {
                    int dist = workbench.Dij().GetDist(STAT_CARRY_ROBOT, robot.pos);
                    if (dist < minDist) {
                        minDist = dist;
                        select = &workbench;
                    }
                }
            }
            if (select) {
                stat.push_back({&robot, select,
                                (g_productArr[robot.carryId].maxVal - g_productArr[robot.carryId].cost) /
                                    ((float)minDist * (float)minDist + 0.0000001f)});
            }

        }
        if (stat.empty())
            return false;

        sort(stat.begin(), stat.end());
        AssignRobot(*stat[0].workben, *stat[0].robot, RA_SELL);
        return true;
    }

    // 贪心购买策略
    bool GreedyBuy()
    {
        struct Stat {
            Robot *robot;
            Workbench *workbench;
            Workbench *seller;
            float profit;
            bool operator<(const Stat &b) const
            {
                return profit > b.profit;
            }
        };

        vector<Stat> stat;

        for (auto &workbench : m_workbench) {
            if (workbench.productStatus == 0 && workbench.productFrames == -1)
                continue;

            if (workbench.type > 7)
                continue;

            if (workbench.productStatus == 0 && workbench.productFrames > 0) {
                // 一种特殊的情况, 正在生产中
                bool needWait = g_frameId >= g_gameFrames - FPS * 30 ||
                                MATERIAL_TABLE[workbench.type] == workbench.origMaterialStatus;
                if (!needWait)
                    continue;
            }



            // 选一个最近的机器人
            Robot *select = nullptr;
            int minDist = 0x7F7F7F7F;
            for (auto &robot : m_robots) {
                if (robot.assigned || robot.carryId)
                    continue;
                int dist = workbench.Dij().GetDist(STAT_NORMAL_ROBOT, robot.pos);

                if (dist < minDist) {
                    minDist = dist;
                    select = &robot;
                }
            }
            if (select == nullptr) {
                continue;
            }

            float realDist = ToRealDist(minDist);
            if (workbench.productStatus == 0 && workbench.productFrames > 0) {
                float dist = workbench.productFrames * select->AvgSpeed();
                realDist = max(realDist, dist);
            }

            // 选一个最近的可出售点
            Workbench *selectWorkbench = nullptr;
            minDist = 0x7F7F7F7F;
            for (auto &b : m_workbench) {
                if ((MATERIAL_TABLE[b.type] & (1 << workbench.type) && !(b.materialStatus & (1 << workbench.type)))) {
                    int dist = workbench.Dij().GetDist(STAT_CARRY_ROBOT, b.pos);
                    if (b.type <= 7 && (b.materialStatus | (1 << workbench.type)) == MATERIAL_TABLE[b.type]) {
                        // 只差1种材料就满的时候
                        dist /= 4;    // 增加一点收益
                    }
                    if (dist < minDist) {
                        minDist = dist;
                        selectWorkbench = &b;
                    }
                }
            }
            if (!selectWorkbench)
                continue;

            
            float sellDist = ToRealDist(minDist);

/* if (workbench.type > 3 && workbench.type <= 7 &&
                   workbench.materialStatus == MATERIAL_TABLE[workbench.type] &&
                workbench.productStatus) {

                //sellDist /= 2;
            }*/

            float profit = (g_productArr[workbench.type].maxVal - g_productArr[workbench.type].cost) / (sellDist * (sellDist + realDist) + 0.000001);
            stat.push_back({select, &workbench, selectWorkbench, profit});
        }

        if (stat.empty())
            return false;

        sort(stat.begin(), stat.end());
        if (AssignRobot(*stat[0].workbench, *stat[0].robot, RA_BUY)) {
            if (stat[0].seller->type <= 7) {
                stat[0].seller->materialStatus |= 1 << stat[0].workbench->type;
            }
        }
        return true;
    }

    // 进攻逻辑
    void Attack() 
    {
        std::vector<RowCol> result[4];
        for (auto &it : m_robots) {
            if (it.assigned)
                continue;

            if (it.carryId) {
                it.assigned = true;
                it.Destroy();
                continue;
            }

            it.m_dij.Init(it.pos, &g_map);
            it.m_dij.Update(STAT_NORMAL_ROBOT, nullptr, &result[it.robotId], 1000);
        }
        
        for (auto &ad : m_adversary) {
            // 选一个最近未指派的机器人攻击

            int minDist = 0x7f7f7f7f;
            Robot *select = nullptr;
            for (auto &it : m_robots) {
                if (it.assigned || it.carryId)
                    continue;
                int dist = it.m_dij.GetDist(STAT_NORMAL_ROBOT, ad.pos);
                if (dist < minDist) {
                    minDist = dist;
                    select = &it;
                }
            }

            if (select) {
                auto path = select->m_dij.MoveFrom(ad.pos, STAT_NORMAL_ROBOT);
                reverse(path.begin(), path.end());

                select->AssignPath(path, g_map);
                select->assigned = true;
            }
        }

        for (auto &it : m_robots) {
            if (it.assigned || it.carryId || result[it.robotId].empty())
                continue;

            auto elem = result[it.robotId][rand() % result[it.robotId].size()];
            auto path = it.m_dij.MoveToDiscrete(STAT_NORMAL_ROBOT, elem);
            it.AssignPath(path, g_map);
            it.assigned = true;
        }
    }

    void Dispatch()
    {
        g_map.ResetConflict();

        LastSellStrategy();
        while (GreedySell());
        while (GreedyBuy());
        
        Attack();


        // 没有分配到的机器人就先停下来
        for (auto& it : m_robots) {
            if (!it.assigned) {
                it.Forward(0);
                it.Rotate(0);
                
            }
        }


        // 按优先级排序, 优先行动高优先级机器人, 低优先级的需要让路
        Robot *robots[ROBOTS_PER_PLAYER];
        for (int i = 0; i < ROBOTS_PER_PLAYER; i++) {
            robots[i] = &m_robots[i];
        }

        sort(robots, robots + ROBOTS_PER_PLAYER, [](Robot *a, Robot *b) {
            if (a->forcePri != b->forcePri) {
                return a->forcePri > b->forcePri;
            }
            if (a->carryId != b->carryId) {
                return a->carryId > b->carryId;
            }
            return a->priority != b->priority ? a->priority > b->priority : a->robotId < b->robotId;
        });

        for (int i = 0; i < ROBOTS_PER_PLAYER; i++) {
            robots[i]->MovePath();
        }

        for (int i = 0; i < ROBOTS_PER_PLAYER; i++) {
            if (robots[i]->beConflicted-- < 0 && robots[i]->forcePri) {
                robots[i]->forcePri = false;
            }
        }

        // 卡住逻辑
        for (int i = ROBOTS_PER_PLAYER - 1; i >= 0; i--) {
            if (robots[i]->Stuck()) {
                //fprintf(stderr, "stuck\n");
                robots[i]->forcePri = i + 2;
                robots[i]->beConflicted = FPS;
                robots[i]->m_posList.clear();
            }
        }
    }

    void MainLoop() 
    {
        while (Input()) {
            uint64_t beginTime = GetMillTime();
            printf("%d\n", g_frameId);

            if (g_frameId == 1) {
                InitOnFirstFrame();
            }
            
            bool ret = BackgroundThread::Instance().UpdateResult();
            if (ret) {
                Dispatch();
            }

            while (BackgroundThread::Instance().IsWorking()  && beginTime + 12 < GetMillTime())
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
            Finish();
        }

        BackgroundThread::Instance().ExitThread();
    }


    int m_money{0};
    int m_playerId{0};
    vector<Workbench>       m_workbench;
    array<Robot, ROBOTS_PER_PLAYER> m_robots;
    vector<Workbench *> m_typeWorkbench[10];  
    vector<Adversary> m_adversary;
    vector<Adversary> m_prevFrameAdversary;
    bool m_isHighestProduct[10]{0}; // 是否最高级物品

    float m_workbenchDist[64][64];

};


int main()
{
    //std::this_thread::sleep_for(std::chrono::milliseconds(15000));
    static Strategy strategy;
    strategy.Init();
    strategy.MainLoop();
}
