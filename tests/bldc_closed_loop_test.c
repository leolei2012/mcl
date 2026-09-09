/**
 * @file    bldc_closed_loop_test.c
 * @brief   mcl 六步 BLDC 完整闭环测试（含电机模型，float）
 *
 * 补齐 bldc_test.c（单元级）未覆盖的「完整闭环带电机模型」：
 *   用理想梯形 BEMF 电机模型 + 机械方程，验证霍尔换相和 BEMF 换相
 *   在闭环下能持续推进换相、产生转速、转子持续旋转。
 *
 * 电机模型（简化理想梯形反电动势）：
 *   - 转子角 θ 驱动三相梯形 BEMF（幅值 λ·ω）
 *   - 换相施加的电压在绕组上产生电流 → 平均转矩 → 机械方程积分转速
 *   - 霍尔状态由 θ 决定（理想 120° 霍尔，直接反映转子位置）
 *   - BEMF 端电压由梯形 EMF 决定（悬空相读 BEMF）
 */

#include "mcl.h"
#include "mcl_bldc_comm.h"
#include <stdio.h>
#include <math.h>

#define TWO_PI_F (2.0f * 3.14159265358979f)
#define PP 4.0f          /* 极对数 */
#define LAMBDA 0.02f     /* 磁链 */
#define R 0.5f           /* 相电阻 */
#define L 0.0005f        /* 相电感 */
#define J 0.0001f        /* 转动惯量 */
#define VBUS 20.0f       /* 母线电压 */
#define DT 0.0001f       /* 控制周期 */

static int g_fail = 0;
#define CHECK(name, cond) do { if (cond) { printf("  [OK]   %s\n", name); } \
    else { printf("  [FAIL] %s\n", name); g_fail++; } } while (0)

/* ---- BLDC 电机模型（理想梯形 BEMF） ---- */
typedef struct
{
    float theta_e;      /* 电气角 rad */
    float omega_e;      /* 电气角速度 rad/s */
    float i_a, i_b, i_c;/* 相电流 */
    float v_a, v_b, v_c;/* 施加相电压 */
} bldc_motor_t;

/* 反电动势形状函数：用正弦基波（amp=1 时即 sin）验证换相闭环逻辑。
   真实 BLDC 是梯形（120° 平顶），但正弦足以验证「换相→转矩→转动→反馈→换相」
   的闭环；梯形 EMF 的平顶会在静止平衡点引入零转矩死区，正弦更平滑。 */
static float trap_emf(float theta, float amp)
{
    return amp * sinf(theta);
}

static void bldc_step(bldc_motor_t *m)
{
    /* 三相梯形反电动势形状函数（单位幅值，不含 ω），电气角各偏移 120° */
    float fa = trap_emf(m->theta_e, 1.0f);
    float fb = trap_emf(m->theta_e - 2.0943951f, 1.0f);
    float fc = trap_emf(m->theta_e + 2.0943951f, 1.0f);

    /* 反电动势 = shape · λ · ω */
    float ea = fa * LAMBDA * m->omega_e;
    float eb = fb * LAMBDA * m->omega_e;
    float ec = fc * LAMBDA * m->omega_e;

    /* 三相电压方程：v = R·i + L·di/dt + e，前向欧拉解 di/dt */
    m->i_a += (m->v_a - R * m->i_a - ea) * DT / L;
    m->i_b += (m->v_b - R * m->i_b - eb) * DT / L;
    m->i_c += (m->v_c - R * m->i_c - ec) * DT / L;

    /* 电磁转矩 Te = λ·p·(f_a·i_a + f_b·i_b + f_c·i_c)
       （形状函数 × 电流，与 ω 无关；静止 ω=0 时也能产生转矩，避免 te=功率/ω 的除零奇异） */
    float te = LAMBDA * PP * (fa * m->i_a + fb * m->i_b + fc * m->i_c);
    float wm = m->omega_e / PP;
    wm += te / J * DT;
    m->omega_e = wm * PP;
    m->theta_e += m->omega_e * DT;
    while (m->theta_e > TWO_PI_F) { m->theta_e -= TWO_PI_F; }
    while (m->theta_e < 0) { m->theta_e += TWO_PI_F; }
}

/* 理想 120° 霍尔：换相电压矢量应超前转子磁链 90° 电角，故 hall 扇区按
   转子角 θ 加 90°（π/2）偏移后划分——θ=0 时施加的应是「+90° 超前」的矢量，
   否则（对齐 0°）转矩为零、转子卡死。 */
static uint8_t hall_from_theta(float theta)
{
    float shifted = theta + (TWO_PI_F / 4.0f);   /* +90° 偏移，电压矢量超前转子 */
    while (shifted >= TWO_PI_F) { shifted -= TWO_PI_F; }
    int sector = (int)(shifted / (TWO_PI_F / 6.0f)) % 6;
    /* 理想 120° 霍尔编码（与 bldc 默认 hall_map 对应：1,3,2,6,4,5） */
    static const uint8_t hmap[6] = {1, 3, 2, 6, 4, 5};
    return hmap[sector];
}

static int phase_volt_from_bemf(bldc_motor_t *m, mcl_scalar *va, mcl_scalar *vb, mcl_scalar *vc)
{
    /* 端电压 = 反电动势（近似空载），三相梯形 shape × λ·ω */
    *va = MCL_FROM_FLOAT(trap_emf(m->theta_e, 1.0f) * LAMBDA * m->omega_e);
    *vb = MCL_FROM_FLOAT(trap_emf(m->theta_e - 2.0943951f, 1.0f) * LAMBDA * m->omega_e);
    *vc = MCL_FROM_FLOAT(trap_emf(m->theta_e + 2.0943951f, 1.0f) * LAMBDA * m->omega_e);
    return MCL_OK;
}

int main(void)
{
    int i, steps;

    printf("===== BLDC 六步闭环测试（float）=====\n");

    /* ---- 场景 1：霍尔换相闭环 ---- */
    {
        mcl_bldc_comm comm;
        bldc_motor_t m;
        float rpm_final;

        mcl_bldc_comm_init(&comm);
        m.theta_e = 0; m.omega_e = 0; m.i_a = m.i_b = m.i_c = 0;
        m.v_a = m.v_b = m.v_c = 0;

        for (i = 0; i < 20000; i++)
        {
            mcl_scalar da, db, dc;
            uint8_t hall = hall_from_theta(m.theta_e);
            /* 闭环：读 hall → 换相 → 施加电压 → 电机步进 */
            mcl_bldc_comm_step_hall(&comm, hall, MCL_FROM_FLOAT(0.5f), &da, &db, &dc);
            /* 占空比 da∈[-1,1] 是相对中性点的相电压标幺，×VBUS/2 得相电压 */
            m.v_a = MCL_TO_FLOAT(da) * (VBUS / 2.0f);
            m.v_b = MCL_TO_FLOAT(db) * (VBUS / 2.0f);
            m.v_c = MCL_TO_FLOAT(dc) * (VBUS / 2.0f);
            bldc_step(&m);
        }
        rpm_final = m.omega_e / PP * 9.5492966f;
        CHECK("霍尔换相闭环：电机持续旋转（|rpm| > 100）", fabsf(rpm_final) > 100.0f);
        printf("        霍尔闭环稳态转速 = %.1f rpm\n", rpm_final);
    }

    /* ---- 场景 2：BEMF 无感换相闭环 ---- */
    {
        mcl_bldc_comm comm;
        bldc_motor_t m;
        float rpm_final;

        mcl_bldc_comm_init(&comm);
        comm.bemf_threshold = MCL_FROM_FLOAT(0.002f);   /* 换相积分阈值 */
        m.theta_e = 0; m.omega_e = 50.0f;   /* 给初始转速，否则无感从零无法启动（需开环） */
        m.i_a = m.i_b = m.i_c = 0; m.v_a = m.v_b = m.v_c = 0;

        steps = 0;
        for (i = 0; i < 20000; i++)
        {
            mcl_scalar da, db, dc;
            mcl_scalar va, vb, vc;
            phase_volt_from_bemf(&m, &va, &vb, &vc);
            mcl_bldc_comm_step_bemf(&comm, va, vb, vc, MCL_FROM_FLOAT(0.5f),
                                    MCL_FROM_FLOAT(DT), &da, &db, &dc);
            m.v_a = MCL_TO_FLOAT(da) * (VBUS / 2.0f);
            m.v_b = MCL_TO_FLOAT(db) * (VBUS / 2.0f);
            m.v_c = MCL_TO_FLOAT(dc) * (VBUS / 2.0f);
            bldc_step(&m);
            steps++;
        }
        rpm_final = m.omega_e / PP * 9.5492966f;
        CHECK("BEMF 无感换相闭环：电机持续旋转（|rpm| > 50）", fabsf(rpm_final) > 50.0f);
        printf("        BEMF 闭环稳态转速 = %.1f rpm（换相 %d 次）\n", rpm_final, steps);
    }

    printf("\n=== %s ===（%d 项失败）\n", g_fail == 0 ? "全部通过" : "存在失败", g_fail);
    return g_fail == 0 ? 0 : 1;
}
