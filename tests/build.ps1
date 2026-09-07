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
