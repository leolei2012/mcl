/**
 * @file    mcl_math.c
 * @brief   mcl 电机控制库：数学库实现（float 查表；Q15/Q31 基础实现）
 *
 * 精度跟随 mcl_scalar（由 MCL_USE_Q15 / MCL_USE_Q31 决定）。
 * 当前 Q15/Q31 为「基础实现」（转 float 计算再转回），后续可优化为纯定点查表。
 */

#include "mcl_math.h"
#include <math.h>
#include <stdbool.h>

#if defined(MCL_MATH_USE_CUSTOM)

/* 用户自定义实现：不在此提供，由用户映射到芯片 API（如 CORDIC） */

#else

/* ============================ float 查表（float 精度专用） ============================ */

#if !defined(MCL_USE_Q15) && !defined(MCL_USE_Q31)

#define MCL_MATH_TABLE_BITS  8
#define MCL_MATH_TABLE_SIZE  (1 << MCL_MATH_TABLE_BITS)

static float s_sin_table[MCL_MATH_TABLE_SIZE + 1];
static bool  s_table_ready = false;

/** 首次调用时用标准 sinf 填充 1/4 周期正弦表（之后高频调用不再依赖 libm） */
static void mcl_math_table_init(void)
{
    int i;
    const float step = MCL_PI_2 / (float)MCL_MATH_TABLE_SIZE;
    for (i = 0; i <= MCL_MATH_TABLE_SIZE; i++)
    {
        s_sin_table[i] = sinf((float)i * step);
    }
    s_table_ready = true;
}

/** 角度归一到 [0, 2π) */
static float mcl_math_norm(float x)
{
    while (x >= MCL_TWO_PI)
    {
        x -= MCL_TWO_PI;
    }
    while (x < 0.0f)
    {
        x += MCL_TWO_PI;
    }
    return x;
}

mcl_scalar mcl_math_sin(mcl_scalar x)
{
    if (!s_table_ready)
    {
        mcl_math_table_init();
    }

    x = mcl_math_norm(x);
    float sign = 1.0f;
    if (x > MCL_PI)
    {
        x -= MCL_PI;
        sign = -1.0f;
    }
    if (x > MCL_PI_2)
    {
        x = MCL_PI - x;
    }

    float idx_f = x * ((float)MCL_MATH_TABLE_SIZE / MCL_PI_2);
    int idx = (int)idx_f;
    float frac = idx_f - (float)idx;
    float v = s_sin_table[idx] * (1.0f - frac) + s_sin_table[idx + 1] * frac;
    return sign * v;
}

mcl_scalar mcl_math_cos(mcl_scalar x)
{
    return mcl_math_sin(x + MCL_PI_2);
}

void mcl_math_sincos(mcl_scalar x, mcl_scalar *sin, mcl_scalar *cos)
{
    if (!s_table_ready)
    {
        mcl_math_table_init();
    }

    x = mcl_math_norm(x);
    float sign_s = 1.0f;
    float sign_c = 1.0f;

    if (x < MCL_PI_2)
    {
        /* 第一象限 */
    }
    else if (x < MCL_PI)
    {
        x = MCL_PI - x;
        sign_c = -1.0f;
    }
    else if (x < 3.0f * MCL_PI_2)
    {
        x = x - MCL_PI;
        sign_s = -1.0f;
        sign_c = -1.0f;
    }
    else
    {
        x = MCL_TWO_PI - x;
        sign_s = -1.0f;
    }

    float idx_f = x * ((float)MCL_MATH_TABLE_SIZE / MCL_PI_2);
    int idx = (int)idx_f;
    if (idx >= MCL_MATH_TABLE_SIZE)
    {
        idx = MCL_MATH_TABLE_SIZE - 1;
    }
    float frac = idx_f - (float)idx;

    float s = s_sin_table[idx] * (1.0f - frac) + s_sin_table[idx + 1] * frac;
    float c = s_sin_table[MCL_MATH_TABLE_SIZE - idx] * (1.0f - frac)
            + s_sin_table[MCL_MATH_TABLE_SIZE - idx - 1] * frac;

    *sin = sign_s * s;
    *cos = sign_c * c;
}

mcl_scalar mcl_math_atan2(mcl_scalar y, mcl_scalar x)
{
    return atan2f(y, x);
}

mcl_scalar mcl_math_sqrt(mcl_scalar x)
{
    return sqrtf(x);
}

#else

/* ============================ Q15/Q31：纯定点查表 sin/cos ============================ */

/* 1/4 周期（90°）正弦表，Q 格式（mcl_scalar）。首次调用用 float 填充一次，
   之后按归一化角度纯整数查表 + Q 格式线性插值，不再调用 sinf/cosf。 */
#define MCL_MATH_TABLE_BITS  8
#define MCL_MATH_TABLE_SIZE  (1 << MCL_MATH_TABLE_BITS)

static mcl_scalar mcl_q_sin_table[MCL_MATH_TABLE_SIZE + 1];
static bool      mcl_q_table_ready = false;

/* 首次调用：用标准 sinf 填充 1/4 周期表（Q 格式） */
static void mcl_q_table_init(void)
{
    int i;
    for (i = 0; i <= MCL_MATH_TABLE_SIZE; i++)
    {
        float a = MCL_PI_2 * (float)i / (float)MCL_MATH_TABLE_SIZE;   /* 0 ~ π/2 */
        mcl_q_sin_table[i] = MCL_FROM_FLOAT(sinf(a));
    }
    mcl_q_table_ready = true;
}

/* 归一化角度 x ∈ [0,1) → sin(x·2π)。
   x 是归一化角度（1.0 = 2π）。
   纯整数实现：不转 float、不调 MCL_TO_FLOAT/MCL_FROM_FLOAT。索引与插值分数
   由折叠后的 90° 区间（0.25 圈 = 256 项表）纯移位得到：
   Q31 取 >>21 当表索引、低 21 位 <<10 当插值分数；
   Q15 取 >>5 当表索引、低 5 位 <<10 当插值分数。
   配 mcl_q_sin_table 做 Q 格式整数乘加插值（MCL_MUL 已内置半加舍入 + int64 防溢出）。 */
static mcl_scalar mcl_q_sin(mcl_scalar x)
{
    mcl_scalar sign = (mcl_scalar)1;
    mcl_scalar v0, v1;

#if defined(MCL_USE_Q31)
    /* Q31：1.0 = 0x7FFFFFFF。象限折叠用 0x2000…（0.25）、0x4000…（0.5）、0x6000…（0.75）。
       归一化角度本就在 [0,1)，负值用 int32 取模溢出即可 wrap（见下方 mask）。 */
    int32_t u = (int32_t)x;
    int32_t idx;

    /* wrap 到 [0,1)：负值取低 31 位即回卷，正值本就在界内（abs < 2^31）。 */
    u &= 0x7FFFFFFF;

    /* 折叠到 [0, 0.25)（1/4 圈 = 90°） */
    if ((uint32_t)u >= 0x40000000u) { u -= 0x40000000; sign = -sign; }   /* 下半圈：符号翻转 */
    if ((uint32_t)u >= 0x20000000u) { u = 0x40000000 - u; }              /* 折 90°~180° 回 90° */

    idx = (int32_t)((uint32_t)u >> 21);                    /* 索引 = u×1024/2^31：0.25圈 → 256项表 → /1024，故 >>(31-10)=21 */
    if (idx >= MCL_MATH_TABLE_SIZE) { idx = MCL_MATH_TABLE_SIZE - 1; }

    v0 = mcl_q_sin_table[idx];
    v1 = mcl_q_sin_table[idx + 1];
    /* 插值分数 frac = (u 的低 21 位) / 2^21 ∈ [0,1)。Q31 分数 = 低21位 << 10。 */
    {
        int32_t f = (int32_t)(((uint32_t)u & 0x001FFFFFu) << 10);   /* Q31 分数 */
        mcl_scalar r = MCL_ADD(v0, MCL_MUL(MCL_SUB(v1, v0), f));
        mcl_scalar neg = (sign < (mcl_scalar)0) ? MCL_NEG(r) : r;
        return neg;
    }

#elif defined(MCL_USE_Q15)
    /* Q15：1.0 = 0x7FFF。象限折叠用 0x2000（0.25）、0x4000（0.5）、0x6000（0.75）。 */
    int16_t u = (int16_t)x;
    int16_t idx;

    u &= 0x7FFF;   /* wrap：负值取低 15 位回卷 */

    if ((uint16_t)u >= 0x4000u) { u = (int16_t)(u - 0x4000); sign = (mcl_scalar)(-1); }
    if ((uint16_t)u >= 0x2000u) { u = (int16_t)(0x4000 - u); }

    idx = (int16_t)((uint16_t)u >> 5);                    /* 索引 = u×1024/2^15：0.25圈 → 256项表，故 >>(15-10)=5 */
    if (idx >= MCL_MATH_TABLE_SIZE) { idx = MCL_MATH_TABLE_SIZE - 1; }

    v0 = mcl_q_sin_table[idx];
    v1 = mcl_q_sin_table[idx + 1];
    {
        int16_t f = (int16_t)(((uint16_t)u & 0x001Fu) << 10);   /* Q15 分数 = 低5位 << 10 */
        mcl_scalar r = MCL_ADD(v0, MCL_MUL(MCL_SUB(v1, v0), f));
        mcl_scalar neg = (sign < (mcl_scalar)0) ? MCL_NEG(r) : r;
        return neg;
    }
#endif
}

mcl_scalar mcl_math_sin(mcl_scalar x)
{
    if (!mcl_q_table_ready) { mcl_q_table_init(); }
    return mcl_q_sin(x);
}

mcl_scalar mcl_math_cos(mcl_scalar x)
{
    if (!mcl_q_table_ready) { mcl_q_table_init(); }
    /* cos(x) = sin(x + 0.25)（归一化角度 +90°）。
       纯整数 +90°：Q31/Q15 加 0.25（= 0x20000000 / 0x2000），负数不回卷由 mcl_q_sin 内部 mask 处理。 */
#if defined(MCL_USE_Q31)
    return mcl_q_sin((mcl_scalar)((int32_t)x + 0x20000000));
#elif defined(MCL_USE_Q15)
    return mcl_q_sin((mcl_scalar)((int16_t)x + 0x2000));
#endif
}

void mcl_math_sincos(mcl_scalar x, mcl_scalar *sin, mcl_scalar *cos)
{
    if (!mcl_q_table_ready) { mcl_q_table_init(); }
    *sin = mcl_q_sin(x);
#if defined(MCL_USE_Q31)
    *cos = mcl_q_sin((mcl_scalar)((int32_t)x + 0x20000000));
#elif defined(MCL_USE_Q15)
    *cos = mcl_q_sin((mcl_scalar)((int16_t)x + 0x2000));
#endif
}

mcl_scalar mcl_math_atan2(mcl_scalar y, mcl_scalar x)
{
    float a = atan2f(MCL_TO_FLOAT(y), MCL_TO_FLOAT(x));
    return MCL_FROM_FLOAT(a * MCL_INV_TWO_PI);
}

mcl_scalar mcl_math_sqrt(mcl_scalar x)
{
    return MCL_FROM_FLOAT(sqrtf(MCL_TO_FLOAT(x)));
}

#endif

/* ============================ 快速 atan2 近似（各精度共享） ============================ */

static float mcl_math_fast_atan2_f(float y, float x)
{
    float abs_y = fabsf(y) + 1.0e-10f;
    float r;
    float angle;

    if (x >= 0.0f)
    {
        r = (x - abs_y) / (x + abs_y);
        angle = 0.1963f * r * r * r - 0.9817f * r + MCL_PI_4;
    }
    else
    {
        r = (x + abs_y) / (abs_y - x);
        angle = 0.1963f * r * r * r - 0.9817f * r + 3.0f * MCL_PI_4;
    }

    return (y < 0.0f) ? -angle : angle;
}

mcl_scalar mcl_math_fast_atan2(mcl_scalar y, mcl_scalar x)
{
#if !defined(MCL_USE_Q15) && !defined(MCL_USE_Q31)
    return mcl_math_fast_atan2_f(y, x);
#else
    float a = mcl_math_fast_atan2_f(MCL_TO_FLOAT(y), MCL_TO_FLOAT(x));
    return MCL_FROM_FLOAT(a * MCL_INV_TWO_PI);
#endif
}

#endif /* MCL_MATH_USE_CUSTOM */
