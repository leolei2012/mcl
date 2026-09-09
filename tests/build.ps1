# mcl PC 仿真测试构建脚本
# 用法：pwsh -File tests\build.ps1

$ErrorActionPreference = 'Stop'

# 自动编译 src/ 下所有 .c
$srcs = Get-ChildItem src -Filter *.c | ForEach-Object { $_.FullName }

gcc -std=c99 -Iinclude -Wall -Wextra tests\sim_test.c $srcs -lm -o tests\sim_test.exe
if ($LASTEXITCODE -ne 0) { Write-Error "编译失败 (sim_test)"; exit 1 }

gcc -std=c99 -Iinclude -Wall -Wextra tests\term_sim.c $srcs -lm -o tests\term_sim.exe
if ($LASTEXITCODE -ne 0) { Write-Error "编译失败 (term_sim)"; exit 1 }

# 定点基础验证（三种精度）
gcc -std=c99 -Iinclude -Wall -Wextra tests\fixed_point_test.c $srcs -lm -o tests\fp_float.exe
if ($LASTEXITCODE -ne 0) { Write-Error "编译失败 (fp_float)"; exit 1 }
gcc -std=c99 -Iinclude -DMCL_USE_Q15 -Wall -Wextra tests\fixed_point_test.c $srcs -lm -o tests\fp_q15.exe
if ($LASTEXITCODE -ne 0) { Write-Error "编译失败 (fp_q15)"; exit 1 }
gcc -std=c99 -Iinclude -DMCL_USE_Q31 -Wall -Wextra tests\fixed_point_test.c $srcs -lm -o tests\fp_q31.exe
if ($LASTEXITCODE -ne 0) { Write-Error "编译失败 (fp_q31)"; exit 1 }

# 定点观测器验证（per-unit 归一化，三种精度）
gcc -std=c99 -Iinclude -Wall -Wextra tests\fixed_point_observer_test.c $srcs -lm -o tests\fpo_float.exe
if ($LASTEXITCODE -ne 0) { Write-Error "编译失败 (fpo_float)"; exit 1 }
gcc -std=c99 -Iinclude -DMCL_USE_Q15 -Wall -Wextra tests\fixed_point_observer_test.c $srcs -lm -o tests\fpo_q15.exe
if ($LASTEXITCODE -ne 0) { Write-Error "编译失败 (fpo_q15)"; exit 1 }
gcc -std=c99 -Iinclude -DMCL_USE_Q31 -Wall -Wextra tests\fixed_point_observer_test.c $srcs -lm -o tests\fpo_q31.exe
if ($LASTEXITCODE -ne 0) { Write-Error "编译失败 (fpo_q31)"; exit 1 }

# 定点 FOC 电流环闭环（per-unit 归一化，三种精度）
gcc -std=c99 -Iinclude -Wall -Wextra tests\fixed_point_foc_test.c $srcs -lm -o tests\fpf_float.exe
if ($LASTEXITCODE -ne 0) { Write-Error "编译失败 (fpf_float)"; exit 1 }
gcc -std=c99 -Iinclude -DMCL_USE_Q15 -Wall -Wextra tests\fixed_point_foc_test.c $srcs -lm -o tests\fpf_q15.exe
if ($LASTEXITCODE -ne 0) { Write-Error "编译失败 (fpf_q15)"; exit 1 }
gcc -std=c99 -Iinclude -DMCL_USE_Q31 -Wall -Wextra tests\fixed_point_foc_test.c $srcs -lm -o tests\fpf_q31.exe
if ($LASTEXITCODE -ne 0) { Write-Error "编译失败 (fpf_q31)"; exit 1 }

# 定点 FOC 速度环闭环（含机械方程，三种精度）
gcc -std=c99 -Iinclude -Wall -Wextra tests\fixed_point_speed_test.c $srcs -lm -o tests\fps_float.exe
if ($LASTEXITCODE -ne 0) { Write-Error "编译失败 (fps_float)"; exit 1 }
gcc -std=c99 -Iinclude -DMCL_USE_Q15 -Wall -Wextra tests\fixed_point_speed_test.c $srcs -lm -o tests\fps_q15.exe
if ($LASTEXITCODE -ne 0) { Write-Error "编译失败 (fps_q15)"; exit 1 }
gcc -std=c99 -Iinclude -DMCL_USE_Q31 -Wall -Wextra tests\fixed_point_speed_test.c $srcs -lm -o tests\fps_q31.exe
if ($LASTEXITCODE -ne 0) { Write-Error "编译失败 (fps_q31)"; exit 1 }

# SMO 无感速度环闭环（float，验证滑模观测器闭环 + 相位补偿）
gcc -std=c99 -Iinclude -Wall -Wextra tests\smo_closed_loop_test.c $srcs -lm -o tests\smo_cl.exe
if ($LASTEXITCODE -ne 0) { Write-Error "编译失败 (smo_cl)"; exit 1 }

# SMO 无感速度环闭环（三种精度，per-unit 归一化）
gcc -std=c99 -Iinclude -Wall -Wextra tests\smo_closed_loop_fp_test.c $srcs -lm -o tests\smofp_float.exe
if ($LASTEXITCODE -ne 0) { Write-Error "编译失败 (smofp_float)"; exit 1 }
gcc -std=c99 -Iinclude -DMCL_USE_Q15 -Wall -Wextra tests\smo_closed_loop_fp_test.c $srcs -lm -o tests\smofp_q15.exe
if ($LASTEXITCODE -ne 0) { Write-Error "编译失败 (smofp_q15)"; exit 1 }
gcc -std=c99 -Iinclude -DMCL_USE_Q31 -Wall -Wextra tests\smo_closed_loop_fp_test.c $srcs -lm -o tests\smofp_q31.exe
if ($LASTEXITCODE -ne 0) { Write-Error "编译失败 (smofp_q31)"; exit 1 }

# 校准流程仿真（R/L 测量，float）
gcc -std=c99 -Iinclude -Wall -Wextra tests\calibration_test.c $srcs -lm -o tests\cal.exe
if ($LASTEXITCODE -ne 0) { Write-Error "编译失败 (cal)"; exit 1 }

# MTPA / 弱磁单元测试（三种精度）
gcc -std=c99 -Iinclude -Wall -Wextra tests\mtpa_fw_test.c $srcs -lm -o tests\mtpa_float.exe
if ($LASTEXITCODE -ne 0) { Write-Error "编译失败 (mtpa_float)"; exit 1 }
gcc -std=c99 -Iinclude -DMCL_USE_Q15 -Wall -Wextra tests\mtpa_fw_test.c $srcs -lm -o tests\mtpa_q15.exe
if ($LASTEXITCODE -ne 0) { Write-Error "编译失败 (mtpa_q15)"; exit 1 }
gcc -std=c99 -Iinclude -DMCL_USE_Q31 -Wall -Wextra tests\mtpa_fw_test.c $srcs -lm -o tests\mtpa_q31.exe
if ($LASTEXITCODE -ne 0) { Write-Error "编译失败 (mtpa_q31)"; exit 1 }

# BLDC 六步换相单元测试
gcc -std=c99 -Iinclude -Wall -Wextra tests\bldc_test.c $srcs -lm -o tests\bldc.exe
if ($LASTEXITCODE -ne 0) { Write-Error "编译失败 (bldc)"; exit 1 }

# 保护功能单元测试（三种精度）
gcc -std=c99 -Iinclude -Wall -Wextra tests\protection_test.c $srcs -lm -o tests\prot_float.exe
if ($LASTEXITCODE -ne 0) { Write-Error "编译失败 (prot_float)"; exit 1 }
gcc -std=c99 -Iinclude -DMCL_USE_Q15 -Wall -Wextra tests\protection_test.c $srcs -lm -o tests\prot_q15.exe
if ($LASTEXITCODE -ne 0) { Write-Error "编译失败 (prot_q15)"; exit 1 }
gcc -std=c99 -Iinclude -DMCL_USE_Q31 -Wall -Wextra tests\protection_test.c $srcs -lm -o tests\prot_q31.exe
if ($LASTEXITCODE -ne 0) { Write-Error "编译失败 (prot_q31)"; exit 1 }

# 开环路径仿真（三种精度：VF / 预定位 / 自动开环切闭环）
gcc -std=c99 -Iinclude -Wall -Wextra tests\openloop_test.c $srcs -lm -o tests\ol_float.exe
if ($LASTEXITCODE -ne 0) { Write-Error "编译失败 (ol_float)"; exit 1 }
gcc -std=c99 -Iinclude -DMCL_USE_Q15 -Wall -Wextra tests\openloop_test.c $srcs -lm -o tests\ol_q15.exe
if ($LASTEXITCODE -ne 0) { Write-Error "编译失败 (ol_q15)"; exit 1 }
gcc -std=c99 -Iinclude -DMCL_USE_Q31 -Wall -Wextra tests\openloop_test.c $srcs -lm -o tests\ol_q31.exe
if ($LASTEXITCODE -ne 0) { Write-Error "编译失败 (ol_q31)"; exit 1 }

# 故障恢复时序 + 现场快照测试（float）
gcc -std=c99 -Iinclude -Wall -Wextra tests\fault_recovery_test.c $srcs -lm -o tests\fault_rec.exe
if ($LASTEXITCODE -ne 0) { Write-Error "编译失败 (fault_rec)"; exit 1 }

# 有感位置环闭环（三种精度）
gcc -std=c99 -Iinclude -Wall -Wextra tests\fixed_point_position_test.c $srcs -lm -o tests\fppos_float.exe
if ($LASTEXITCODE -ne 0) { Write-Error "编译失败 (fppos_float)"; exit 1 }
gcc -std=c99 -Iinclude -DMCL_USE_Q15 -Wall -Wextra tests\fixed_point_position_test.c $srcs -lm -o tests\fppos_q15.exe
if ($LASTEXITCODE -ne 0) { Write-Error "编译失败 (fppos_q15)"; exit 1 }
gcc -std=c99 -Iinclude -DMCL_USE_Q31 -Wall -Wextra tests\fixed_point_position_test.c $srcs -lm -o tests\fppos_q31.exe
if ($LASTEXITCODE -ne 0) { Write-Error "编译失败 (fppos_q31)"; exit 1 }

# 校准子功能测试（零漂 / 编码器对齐 / 相序 / 磁链，float）
gcc -std=c99 -Iinclude -Wall -Wextra tests\calibration_full_test.c $srcs -lm -o tests\cal_full.exe
if ($LASTEXITCODE -ne 0) { Write-Error "编译失败 (cal_full)"; exit 1 }

# BLDC 六步完整闭环带电机模型（float）
gcc -std=c99 -Iinclude -Wall -Wextra tests\bldc_closed_loop_test.c $srcs -lm -o tests\bldc_cl.exe
if ($LASTEXITCODE -ne 0) { Write-Error "编译失败 (bldc_cl)"; exit 1 }

Write-Host "构建成功，运行测试..."
& tests\sim_test.exe
Write-Host ""
Write-Host "===== 终端可视化仿真 ====="
& tests\term_sim.exe
Write-Host ""
Write-Host "===== 定点基础验证 ====="
& tests\fp_float.exe
Write-Host ""
& tests\fp_q15.exe
Write-Host ""
& tests\fp_q31.exe
Write-Host ""
Write-Host "===== 定点观测器验证（per-unit）====="
& tests\fpo_float.exe
Write-Host ""
& tests\fpo_q15.exe
Write-Host ""
& tests\fpo_q31.exe
Write-Host ""
Write-Host "===== 定点 FOC 电流环闭环（per-unit）====="
& tests\fpf_float.exe
Write-Host ""
& tests\fpf_q15.exe
Write-Host ""
& tests\fpf_q31.exe
Write-Host ""
Write-Host "===== 定点 FOC 速度环闭环（含机械方程）====="
& tests\fps_float.exe
Write-Host ""
& tests\fps_q15.exe
Write-Host ""
& tests\fps_q31.exe
Write-Host ""
Write-Host "===== SMO 无感速度环闭环（float，相位补偿）====="
& tests\smo_cl.exe
Write-Host ""
Write-Host "===== SMO 无感速度环闭环（三精度，per-unit）====="
& tests\smofp_float.exe
Write-Host ""
& tests\smofp_q15.exe
Write-Host ""
& tests\smofp_q31.exe
Write-Host ""
Write-Host "===== 校准流程仿真（R/L 测量）====="
& tests\cal.exe
Write-Host ""
Write-Host "===== MTPA/弱磁单元测试 ====="
& tests\mtpa_float.exe
Write-Host ""
& tests\mtpa_q15.exe
Write-Host ""
& tests\mtpa_q31.exe
Write-Host ""
Write-Host "===== BLDC 六步换相单元测试 ====="
& tests\bldc.exe
Write-Host ""
Write-Host "===== 保护功能单元测试 ====="
& tests\prot_float.exe
Write-Host ""
& tests\prot_q15.exe
Write-Host ""
& tests\prot_q31.exe
Write-Host ""
Write-Host "===== 开环路径仿真（VF / 预定位 / 自动开环切闭环）====="
& tests\ol_float.exe
Write-Host ""
& tests\ol_q15.exe
Write-Host ""
& tests\ol_q31.exe
Write-Host ""
Write-Host "===== 故障恢复时序 + 现场快照 ====="
& tests\fault_rec.exe
Write-Host ""
Write-Host "===== 有感位置环闭环（三精度）====="
& tests\fppos_float.exe
Write-Host ""
& tests\fppos_q15.exe
Write-Host ""
& tests\fppos_q31.exe
Write-Host ""
Write-Host "===== 校准子功能（零漂 / 编码器对齐 / 相序 / 磁链）====="
& tests\cal_full.exe
Write-Host ""
Write-Host "===== BLDC 六步完整闭环（霍尔 + BEMF，带电机模型）====="
& tests\bldc_cl.exe
