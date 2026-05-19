/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2019-2023. All rights reserved.
 * Description: 常用几何定义与算法
 * Author: 张元龙
 * Date: 2023-04-28
 */
#pragma once
#include <stdio.h>
#include <assert.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <algorithm>
#include "ConstDef.h"

constexpr float ALG_EPS = 0.00001;

inline bool FloatEqual(float a, float b)
{
    return fabsf(a - b) < ALG_EPS;
}

struct Vec2 // 向量或坐标
{
    float x, y;

    Vec2() : x(0), y(0)
    {
    }

    constexpr Vec2(float _x, float _y) : x(_x), y(_y)
    {
    }

    Vec2 operator - () const
    {
        return Vec2(-x, -y);
    }

    Vec2 operator + (Vec2 v) const
    {
        return Vec2(x + v.x, y + v.y);
    }

    Vec2 operator - (Vec2 v) const
    {
        return Vec2(x - v.x, y - v.y);
    }

    Vec2 operator * (float v) const
    {
        return Vec2(x * v, y * v);
    }

    Vec2 operator / (float v) const
    {
        return Vec2(x / v, y / v);
    }

    Vec2& operator += (const Vec2& v)
    {
        x += v.x;
        y += v.y;
        return *this;
    }

    Vec2& operator -= (const Vec2& v)
    {
        x -= v.x;
        y -= v.y;
        return *this;
    }

    Vec2& operator *= (float fMul)
    {
        x *= fMul;
        y *= fMul;
        return *this;
    }

    Vec2& operator /= (float fDiv)
    {
        x /= fDiv;
        y /= fDiv;
        return *this;
    }

    bool operator == (const Vec2& v) const
    {
        return fabs(x - v.x) < 0.0001 && fabs(y - v.y) < 0.0001;
    }

    float Length() const
    {
        return sqrtf(x * x + y * y);
    }

    float LengthSqr() const
    {
        return x * x + y * y;
    }

    bool IsZero() const
    {
        return fabsf(x) < ALG_EPS && fabsf(y) < ALG_EPS;
    }

    Vec2 Unit() const   // 求单位向量
    {
        float len = Length();
        return Vec2(x / len, y / len);
    }
};

const Vec2 ZERO_VEC(0, 0);

struct Line
{
    Vec2 a, b;  // 两个点标识一条线
    Line()
    {

    }

    Line(Vec2 a, Vec2 b)
    {
        Line::a = a;
        Line::b = b;
    }
};



// 将弧度标准化为 [-pi, pi]
inline float StandardizingAngle(float fAngle)
{
    fAngle -= roundf(fAngle / (2 * PI)) * (2 * PI);

    while (fAngle > PI)
    {
        fAngle -= 2 * PI;
    }

    while (fAngle < -PI)
    {
        fAngle += 2 * PI;
    }

    return fAngle;
}

inline float sqr(float x)
{
    return x * x;
}

inline double sqr(double x)
{
    return x * x;
}

// 求距离的平方
inline float DistanceSqr(Vec2 p1, Vec2 p2)
{
    return sqr(p1.x - p2.x) + sqr(p1.y - p2.y);
}

// 求距离
inline float Distance(Vec2 p1, Vec2 p2)
{
    return sqrtf(sqr(p1.x - p2.x) + sqr(p1.y - p2.y));
}


inline float Xmult(Vec2 p1, Vec2 p2, Vec2 p0)
{
    return (p1.x - p0.x) * (p2.y - p0.y) - (p2.x - p0.x) * (p1.y - p0.y);
}

inline float Xmult(Vec2 v1, Vec2 v2)
{
    return (v1.x) * (v2.y) - (v2.x) * (v1.y);
}

inline float Dot(Vec2 a, Vec2 b)
{
    return a.x * b.x + a.y * b.y;
}

inline Vec2 Cross(float s, Vec2 a)
{
    return Vec2(-s * a.y, s * a.x);
}


//计算两直线交点,注意事先判断直线是否平行
inline Vec2 Intersection(Vec2 u1, Vec2 u2, Vec2 v1, Vec2 v2)
{
    Vec2 ret = u1;
    float t = ((u1.x - v1.x) * (v1.y - v2.y) - (u1.y - v1.y) * (v1.x - v2.x))
        / ((u1.x - u2.x) * (v1.y - v2.y) - (u1.y - u2.y) * (v1.x - v2.x));
    ret.x += (u2.x - u1.x) * t;
    ret.y += (u2.y - u1.y) * t;
    return ret;
}


//点到线段上的最近点
inline Vec2 PtoSeg(Vec2 p, const Line& l)
{
    Vec2 t = p;
    t.x += l.a.y - l.b.y, t.y += l.b.x - l.a.x;
    if (Xmult(l.a, t, p) * Xmult(l.b, t, p) > 0)
        return DistanceSqr(p, l.a) < DistanceSqr(p, l.b) ? l.a : l.b;
    return Intersection(p, t, l.a, l.b);
}

//点到线段距离
inline float PtoSegDistance(Vec2 p, const Line& l)
{
    Vec2 t = p;
    t.x += l.a.y - l.b.y, t.y += l.b.x - l.a.x;
    if (Xmult(l.a, t, p) * Xmult(l.b, t, p) > 0)
    {
        float fDistanceSqrA = DistanceSqr(p, l.a);
        float fDistanceSqrB = DistanceSqr(p, l.b);

        return fDistanceSqrA < fDistanceSqrB ? sqrtf(fDistanceSqrA) : sqrtf(fDistanceSqrB);
    }
    return fabsf(Xmult(p, l.a, l.b)) / Distance(l.a, l.b);
}

//点到线段距离, 并且当不能映射到线段时返回默认值
inline float PtoSegDistance(Vec2 p, const Line& l, float fDefault)
{
    Vec2 t = p;
    t.x += l.a.y - l.b.y, t.y += l.b.x - l.a.x;
    return Xmult(l.a, t, p) * Xmult(l.b, t, p) > 0 ? fDefault : fabsf(Xmult(p, l.a, l.b)) / Distance(l.a, l.b);
}

// 点到直线上的最近点
inline Vec2 PtoLine(Vec2 p, const Line& l)
{
    Vec2 t = p;
    t.x += l.a.y - l.b.y, t.y += l.b.x - l.a.x;
    return Intersection(p, t, l.a, l.b);
}

//点到直线距离
inline double DisPtoLine(Vec2 p, const Line& l)
{
    return fabs(Xmult(p, l.a, l.b)) / Distance(l.a, l.b);
}


//判两点在线段异侧,点在线段上返回0
inline bool OppositeSide(Vec2 p1, Vec2 p2, Line l)
{
    return Xmult(l.a, p1, l.b) * Xmult(l.a, p2, l.b) < -ALG_EPS;
}



//判两线段相交,不包括端点和部分重合
inline bool Intersect(const Line& u, const Line& v)
{
    return OppositeSide(u.a, u.b, v) && OppositeSide(v.a, v.b, u);
}

// 判断直线相交
inline Vec2 Intersection(const Line& u, const Line& v)
{
    Vec2 ret = u.a;
    float t = ((u.a.x - v.a.x) * (v.a.y - v.b.y) - (u.a.y - v.a.y) * (v.a.x - v.b.x))
        / ((u.a.x - u.b.x) * (v.a.y - v.b.y) - (u.a.y - u.b.y) * (v.a.x - v.b.x));
    ret.x += (u.b.x - u.a.x) * t;
    ret.y += (u.b.y - u.a.y) * t;
    return ret;
}

// 判断圆和直线相交
inline bool IntersecLineCircle(Vec2 c, float r, const Line& line)
{
    return DisPtoLine(c, line) < r + ALG_EPS;
}


// 计算圆和直线的交点
inline void IntersectionLineCircle(Vec2 c, float r, const Line& line, Vec2& p1, Vec2& p2)
{
    Vec2 p = c;
    float t;
    p.x += line.a.y - line.b.y;
    p.y += line.b.x - line.a.x;
    p = Intersection(p, c, line.a, line.b);
    t = sqrtf(r * r - Distance(p, c) * Distance(p, c)) / Distance(line.a, line.b);
    p1.x = p.x + (line.b.x - line.a.x) * t;
    p1.y = p.y + (line.b.y - line.a.y) * t;
    p2.x = p.x - (line.b.x - line.a.x) * t;
    p2.y = p.y - (line.b.y - line.a.y) * t;
}

// 三点求圆心


inline bool GetCircleCenter(Vec2 p1, Vec2 p2, Vec2 p3, Vec2 &out)
{
    double x1 = p1.x;
    double y1 = p1.y;
    double x2 = p2.x;
    double y2 = p2.y;
    double x3 = p3.x;
    double y3 = p3.y;

    double A = x1 * (y2 - y3) - y1 * (x2 - x3) + x2 * y3 - x3 * y2;

    if (fabs(A) < ALG_EPS)
        return false;

    double B = (sqr(x1) + sqr(y1)) * (y3 - y2) + (sqr(x2) + sqr(y2)) * (y1 - y3) + (sqr(x3) + sqr(y3)) * (y2 - y1);
    double C = (sqr(x1) + sqr(y1)) * (x2 - x3) + (sqr(x2) + sqr(y2)) * (x3 - x1) + (sqr(x3) + sqr(y3)) * (x1 - x2);

    out.x = -B / (2 * A);
    out.y = -C / (2 * A);
    return true;
}


// 计算向量在指定角度上的投影长度
inline float GetShadow(Vec2 sVec2, float fAngle)
{
    Vec2 stAnother(cosf(fAngle), sinf(fAngle));
    return Dot(sVec2, stAnother);
}
/*
inline float GetShadow(Vec2 sVec2, Vec2 stAnother)
{
    return Dot(sVec2, stAnother);
}*/

// 判断两个区间是否重叠
inline bool IntervalOverlaped(float a1, float b1, float a2, float b2)
{
    return !(b1 < a2 || b2 < a1);

}

// 求垂直向量
inline Vec2 GetVerticalVec(Vec2 v)
{
    return Vec2(v.y, -v.x);
}


