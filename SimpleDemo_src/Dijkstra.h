/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2023. All rights reserved.
 * Description: 最短路算法实现。
 * Author: 张元龙
 * Date: 2023-04-28
 */

#pragma once
#include "Comm.h"
#include "Map.h"
#include <algorithm>
#include <queue>
// 最短路
class Dijkstra {
public:
    void Init(Vec2 target, const Map *map)
    {
        m_target = target;
        m_map = map;
    }

    void ResetMap(const Map *map)
    {
        m_map = map;
    }

    Vec2 GetTarget() const
    {
        return m_target;
    }

    RowCol PosToValidDiscrete(Vec2 pos, RobotStat stat) const
    {
        RowCol s = PosToDiscrete(pos);
        if (m_map->CanReachDiscrete(s.r, s.c, stat)) {
            return s;
        }

        std::vector<RowCol> v;
        for (int i = -2; i <= 2; i++)
            for (int j = -2; j <= 2; j++) {
                int r = i + s.r;
                int c = j + s.c;
                if (m_map->CanReachDiscrete(r, c, stat)) {
                    v.push_back({r, c});
                }
            }

        sort(v.begin(), v.end(), [pos](RowCol a, RowCol b) {
            return DistanceSqr(DiscreteToPos(a.r, a.c), pos) < DistanceSqr(DiscreteToPos(b.r, b.c), pos);
        });

        return v.empty() ? RowCol{-1, -1} : v[0];
    }

    bool Update(RobotStat stat,
                uint8_t m_conflictBits[MAP_DISCRETE_HEIGHT][MAP_DISCRETE_WIDTH] = nullptr,
                std::vector<RowCol> *result = nullptr, int maxResult = 0, int maxDist = 0x7fffffff)
    {
        // 求最短路径
        RowCol s = PosToValidDiscrete( m_target, stat);

        struct HeapElem {
            int dist;
            RowCol p;
            bool operator<(const HeapElem &b) const
            {
                return dist > b.dist;
            }
        };

        std::priority_queue<HeapElem> heap;

        memset(cs[stat], 0x7F, sizeof cs[stat]);
        if (s.r == -1) {
            fprintf(stderr, "Update dij err: %f,%f stat:%d\n", m_target.x, m_target.y, stat);
            return false;
        }

        heap.push({0, s});
        cs[stat][s.r][s.c] = {0, 0};

        while (!heap.empty()) {
            auto top = heap.top();
            heap.pop();
            if (top.dist != cs[stat][top.p.r][top.p.c].dist) {
                continue;
            }
            if (top.dist >= maxDist)
                break;
            RowCol rc = top.p;

            if (result && (m_conflictBits == nullptr || m_conflictBits[rc.r][rc.c] == 0) && top.dist) {
                result->push_back(top.p);
                if (result->size() >= maxResult) {
                    break;
                }
            }

            // 遍历8个方向
            for (uint8_t i = 0; i < sizeof(DIR) / sizeof(DIR[0]); i++) {
                uint8_t dirIdx = i ^ cs[stat][top.p.r][top.p.c].dir; // 优先遍历上一次过来的方向

                if (m_conflictBits) {
                    if ((m_conflictBits[rc.r][rc.c] & (1 << dirIdx))) {
                        continue; // 方向不可达
                    }
                }

                auto dir = DIR[dirIdx];
                int dr = top.p.r + dir.r;
                int dc = top.p.c + dir.c;

                if (!m_map->CanReachDiscrete(dr, dc, stat)) {
                    continue; // 不可达
                }
                int newDist = top.dist + DIR_DIST[dirIdx];
                if (newDist < cs[stat][dr][dc].dist) {
                    cs[stat][dr][dc] = {newDist, dirIdx};
                    heap.push({newDist, {dr, dc}});
                }
            }
        }
        return true;
    }

    bool CanMoveFrom(Vec2 source, RobotStat stat)
    {
        RowCol s = PosToValidDiscrete(source, stat);
        if (s.r == -1)
            return false;
        return cs[stat][s.r][s.c].dist < 0x7F7F7F7F;
    }

    std::vector<Vec2> MoveFrom(Vec2 source, RobotStat stat, int *dist = nullptr) const
    {
        RowCol t = PosToValidDiscrete(source, stat);

        std::vector<Vec2> result;
        if (t.c < 0 || t.r < 0 || cs[stat][t.r][t.c].dist >= 0x7f7f7f7f) {
            if (dist) {
                *dist = 0;
            }
            return result; // 不可达
        }
        if (dist) {
            *dist = cs[stat][t.r][t.c].dist;
        }

        result.push_back(DiscreteToPos(t.r, t.c));
        while (cs[stat][t.r][t.c].dist) {
            t += DIR[cs[stat][t.r][t.c].dir ^ 1];
            result.push_back(DiscreteToPos(t.r, t.c));
        }

        if (DistanceSqr(result.back(), m_target) > 0.0001) {
            result.push_back(m_target);
        }

        return result;
    }

    std::vector<Vec2> MoveToDiscrete(RobotStat stat, RowCol t) const
    {
        assert(cs[stat][t.r][t.c].dist < 0x7f7f7f7f);
        std::vector<Vec2> result;
        result.push_back(DiscreteToPos(t.r, t.c));
        while (cs[stat][t.r][t.c].dist) {
            t += DIR[cs[stat][t.r][t.c].dir ^ 1];
            result.push_back(DiscreteToPos(t.r, t.c));
        }
        reverse(result.begin(), result.end());
        return result;
    }

    int GetDist(RobotStat stat, RowCol t) const
    {
        return cs[stat][t.r][t.c].dist;
    }

    int GetDist(RobotStat stat, Vec2 source) const
    {
        RowCol t = PosToValidDiscrete(source, stat);
        std::vector<Vec2> result;
        if (t.c < 0 || t.r < 0) {
            return 0x7f7f7f7f; // 不可达
        }
        return cs[stat][t.r][t.c].dist;
    }

private:
    Vec2 m_target;
    const Map *m_map;
    int m_allowBit;
    struct CellStat {
        int dist; // 距离
        uint8_t dir;
    };
    CellStat cs[2][MAP_DISCRETE_HEIGHT][MAP_DISCRETE_WIDTH];
};
