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

/* ============================ Q15/Q31：基础实现（转 float，TODO 优化纯定点） ============================ */

mcl_scalar mcl_math_sin(mcl_scalar x)
{
    float rad = MCL_TO_FLOAT(x) * MCL_TWO_PI;
    return MCL_FROM_FLOAT(sinf(rad));
}

mcl_scalar mcl_math_cos(mcl_scalar x)
{
    float rad = MCL_TO_FLOAT(x) * MCL_TWO_PI;
    return MCL_FROM_FLOAT(cosf(rad));
}

void mcl_math_sincos(mcl_scalar x, mcl_scalar *sin, mcl_scalar *cos)
{
    float rad = MCL_TO_FLOAT(x) * MCL_TWO_PI;
    *sin = MCL_FROM_FLOAT(sinf(rad));
    *cos = MCL_FROM_FLOAT(cosf(rad));
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
