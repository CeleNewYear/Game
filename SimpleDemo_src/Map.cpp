/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2023. All rights reserved.
 * Description: 地图数据处理实现。
 * Author: 张元龙
 * Date: 2023-04-28
 */
#include "Map.h"
#include <queue>
using namespace std;

bool Map::SetMap(char mapData[MAP_FILE_ROW_NUMS][MAP_FILE_COL_NUMS])
{
    memcpy(m_mapData, mapData, sizeof m_mapData);
    constexpr int n = MAP_FILE_COL_NUMS;
    constexpr int m = MAP_FILE_ROW_NUMS;
    auto m_mapInner = vector<vector<char>>(n, vector<char>(m));
    for (int i = 0; i < MAP_FILE_ROW_NUMS; i++) {
        for (int j = 0; j < MAP_FILE_COL_NUMS; j++) {
            m_mapInner[j][m - i - 1] = m_mapData[i][j];
        }
    }

    vector<vector<bool>> bfs(n, vector<bool>(m, false));
    auto is_wall = [&](int x, int y) { return x >= 0 && x < n && y >= 0 && y < m && m_mapInner[x][y] == '#'; };

    auto id = [&](int x, int y) { return x * (m + 1) + y; };
    const int dx[4] = {1, -1, 0, 0};
    const int dy[4] = {0, 0, 1, -1};

    vector<vector<bool>> X(n, vector<bool>(m + 1, false));
    vector<vector<bool>> Y(m, vector<bool>(n + 1, false));
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < m; j++) {
            if (m_mapInner[i][j] == '#' && !bfs[i][j]) {
                queue<pair<int, int>> q;
                q.emplace(i, j);
                bfs[i][j] = true;
                while (!q.empty()) {
                    int u = q.front().first, v = q.front().second;
                    q.pop();
                    for (int d = 0; d < 4; d++) {
                        int x = u + dx[d], y = v + dy[d];
                        if (is_wall(x, y)) {
                            if (!bfs[x][y]) {
                                bfs[x][y] = true;
                                q.emplace(x, y);
                            }
                        } else {
                            if (u == x) {
                                X[x][max(v, y)] = true;
                            } else {
                                Y[y][max(u, x)] = true;
                            }
                        }
                    }
                }
            }
        }
    }
    for (int y = 0; y <= m; y++) {
        for (int x = 0; x < n; x++) {
            if (X[x][y]) {
                int nx = x;
                for (; nx < n && X[nx][y]; nx++)
                    ;
                walls.push_back({{x * CELL_SIZE, y * CELL_SIZE}, {nx * CELL_SIZE, y * CELL_SIZE}});
                x = nx; // X[nx][y] must be zero, so y can set as nx + 1
            }
        }
    }
    for (int x = 0; x <= n; x++) {
        for (int y = 0; y < m; y++) {
            if (Y[y][x]) {
                int ny = y;
                for (; ny < m && Y[ny][x]; ny++)
                    ;
                walls.push_back({{x * CELL_SIZE, y * CELL_SIZE}, {x * CELL_SIZE, ny * CELL_SIZE}});
                y = ny; // Y[ny][x] must be zero, so y can set as ny + 1
            }
        }
    }

    walls.push_back({{0, 0}, {0, MAP_HEIGHT}});
    walls.push_back({{0, MAP_HEIGHT}, {MAP_WIDTH, MAP_HEIGHT}});
    walls.push_back({{MAP_WIDTH, 0}, {MAP_WIDTH, MAP_HEIGHT}});
    walls.push_back({{0, 0}, {MAP_WIDTH, 0}});
    InitDiscrete();
    return true;
}


bool Map::CanReachDiscrete(int row, int col, int allowBit) const
{
    return (row > 0 && row < MAP_DISCRETE_HEIGHT && col > 0 && col < MAP_DISCRETE_WIDTH &&
            (m_discrete[row][col] >= allowBit));
}

char Map::GetChar(int row, int col) const
{
    return m_mapData[row][col];
}

bool Map::IsWall(int row, int col) const
{
    return row < 0 || col < 0 || row >= MAP_FILE_ROW_NUMS || col >= MAP_FILE_COL_NUMS || m_mapData[row][col] == '#';
}

// 判定s 是否能直达 t
bool Map::CanDirectMove(Vec2 s, Vec2 t, float radius) const
{
    Line l{s, t};
    if (IntersectWall(l)) {
        return false;
    }

    Vec2 v = GetVerticalVec(t - s).Unit() * radius;
    if (IntersectWall({s + v, t + v}) || IntersectWall({s - v, t - v})) {
        return false;
    }
    return true;
}



// 优化路径, 避免弯弯绕绕, 至少优化10米和未来的2个点
void Map::OptimizePath(Vec2 pos, float radius, vector<Vec2> &path) const
{
    float totLen = 0;
    auto newEnd = path.begin();
    auto it = path.begin();

    while (it != path.end()) {
        if (totLen >= CONFLICT_LENGTH && newEnd >= path.begin() + 2) {
            break;
        }

        auto next = GetDirectPos(pos, radius, it, path.end());
        *newEnd++ = *next;
        totLen += Distance(*next, pos);
        pos = *next;
        it = next + 1;
    }

    path.erase(newEnd, it);
}

// 路径冲突检查 0 表示不冲突, 否则表示一个冲突位表
uint8_t Map::CheckConflict(int robotId, Vec2 pos, vector<Vec2> &path)
{
    if (path.empty()) {
        auto rc = PosToDiscrete(pos);
        return (DiscreteValid(rc) && m_conflictBits[rc.r][rc.c] != 0);
    }

    struct ConflictStat {
        RowCol rc;
        uint8_t conflictBits;
    };
    vector<ConflictStat> conflictStat;

    float totLen = 0;

    constexpr float STEP = 0.3;
    for (int i = 0; i < path.size() && totLen < CONFLICT_LENGTH; i++) {
        Vec2 v = path[i] - pos;
        Vec2 unit = v.Unit() * STEP;

        // 求方向
        pair<float, int> cosAndDir[8];
        for (int d = 0; d < 8; d++) {
            float cosa = Dot(CIRCLE[d], unit); // 最大的就是近方向, 最小的就是反方向
            cosAndDir[d] = {cosa, d};
        }
        sort(cosAndDir, cosAndDir + 8);
        uint8_t conflictBits = (1 << cosAndDir[0].second) | (1 << cosAndDir[1].second) | (1 << cosAndDir[2].second);

        Vec2 p = pos;
        int j = 1 + (int)(min(v.Length(), (CONFLICT_LENGTH - totLen)) / STEP);
        for (int k = 0; k < j; k++) {
            auto rc = PosToDiscrete(p);

            if (DiscreteValid(rc) && ((m_conflictBits[rc.r][rc.c] & (1 << cosAndDir[7].second)) /* ||
                                          (totLen == 0 && k <= 1 && m_conflictBits[rc.r][rc.c])*/)) {
                return m_conflictRobot[rc.r][rc.c]; // 路径冲突
            }
            conflictStat.push_back({rc, /* totLen == 0 && k <= 1 ? (uint8_t)0xFF :*/ conflictBits});
            p += unit;
        }
        totLen += v.Length();
        pos = path[i];
    }
    constexpr int DYE_WIDTH = 4;
    for (auto &it : conflictStat) {
        for (int i = -DYE_WIDTH; i <= DYE_WIDTH; i++)
            for (int j = -DYE_WIDTH; j <= DYE_WIDTH; j++) {
                if (i * i + j * j > DYE_WIDTH * DYE_WIDTH) {
                    continue;
                }
                RowCol rc = it.rc;
                rc.r += i;
                rc.c += j;
                if (DiscreteValid(rc)) {
                    m_conflictBits[rc.r][rc.c] |= it.conflictBits;
                    m_conflictRobot[rc.r][rc.c] |= 1 << robotId;
                }
            }
    }

    return 0;
}

void Map::ResetConflict()
{
    memset(m_conflictBits, 0, sizeof m_conflictBits);
    memset(m_conflictRobot, 0, sizeof m_conflictRobot);
}

bool Map::MapValid(RowCol rc) const
{
    return rc.r >= 0 && rc.r < MAP_FILE_ROW_NUMS && rc.c >= 0 && rc.c < MAP_FILE_COL_NUMS &&
           m_mapData[rc.r][rc.c] != '#';
}

bool Map::DiscreteValid(RowCol rc)
{
    return rc.r >= 0 && rc.r < MAP_DISCRETE_HEIGHT && rc.c >= 0 && rc.c < MAP_DISCRETE_WIDTH;
}

bool Map::IntersectWall(const Line &l) const
{
    for (auto &it : walls) {
        if (Intersect(it, l))
            return true;
    }
    return false;
}

void Map::InitDiscrete()
{
    memset(m_discrete, -1, sizeof m_discrete);
    for (int i = 0; i < MAP_DISCRETE_HEIGHT; i++) {
        for (int j = 0; j < MAP_DISCRETE_WIDTH; j++) {
            Vec2 pos = DiscreteToPos(i, j);

            if (CanReach(pos, ROBOT_CARRY_RADIUS)) {
                m_discrete[i][j] = STAT_CARRY_ROBOT;
            } else if (CanReach(pos, ROBOT_RADIUS)) {
                m_discrete[i][j] = STAT_NORMAL_ROBOT;
            }
        }
    }
}

// 判断地图上某个坐标能否容纳半径为radius的圆
bool Map::CanReach(Vec2 pos, float radius) const
{
    // 对圆心 + 圆上8个点进行检测
    Vec2 checkPoints[9];
    memcpy(checkPoints, CIRCLE, sizeof CIRCLE);

    for (int i = 0; i < 8; i++) {
        checkPoints[i] *= radius;
        checkPoints[i] += pos;
    }
    checkPoints[8] = pos;

    // 9个点必须全部合法
    for (const auto &it : checkPoints) {
        if (it.x < 0 || it.y < 0 || it.x > MAP_WIDTH || it.y > MAP_HEIGHT)
            return false;
        auto mapit = PosToMap(it);
        if (mapit.r < 0 || mapit.c < 0 || mapit.r >= MAP_FILE_ROW_NUMS || mapit.c >= MAP_FILE_COL_NUMS ||
            m_mapData[mapit.r][mapit.c] == '#') {
            return false;
        }
    }
    return true;
}