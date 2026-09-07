# 嵌入式工程架构规范

本规范定义嵌入式固件工程的分层、目录结构、依赖规则、组合根模式、初始化顺序、命名与 include 约定。它**与具体芯片 / 板卡 / 工具链无关**，可复用到任何嵌入式项目。

占位符说明：

```text
<chip>          目标芯片（STM32 / PY32 / GD32 / AT32 …）
<board>         目标板卡 / 硬件版本
<toolchain>     构建工具（Keil MDK / IAR / GCC / CMake）
<vendor_sdk>    厂商 SDK / CMSIS / HAL / LL
<rtos>          RTOS 内核（RT-Thread / FreeRTOS / ThreadX / 裸机）
<层>            分层名（app / drivers / middleware / hal / bsp …）
<模块>          层内自包含模块名
```

> 本仓库（STM32G474VET6 开发板模板工程）是这套规范的一套落地实例。

---

## 1. 分层总览

依赖方向自上而下，**只允许上层调用下层，禁止反向 / 循环依赖**：

```text
app          应用层：业务逻辑、状态机、产品功能
  ↓
middleware   中间件：协议栈、监控、OTA、调试接口（自包含模块）
  ↓
drivers      驱动层：具体设备驱动（sensor / motor / lcd / eeprom …）
  ↓
hal          HAL 抽象层：外设实例薄封装（uart / spi / i2c / adc / tim …）
  ↓
mcal         厂商 SDK（寄存器级）
```

`bsp`（板级支持包：main、时钟树、引脚、NVIC、启动文件、链接脚本）是**横向层**，
不在上面的垂直链里，依赖方向为：

```text
bsp → mcal     bsp 直接操作寄存器做时钟 / NVIC / 看门狗，并提供板级配置（引脚宏）
```

`drivers → bsp`、`hal → bsp` 允许：读 bsp 的板级配置（引脚 / 有效电平等），实际 GPIO 操作走 hal。
禁止：`drivers → mcal`（drivers 访问硬件寄存器必须经 hal）。

职责划分（以 GPIO 为例）：
- `hal` 只做外设薄封装：引脚模式配置用 bsp 的引脚宏（单一来源），hal 不做设备逻辑。
- `drivers` 体现设备语义：知道「设备 → 引脚」映射（来自 bsp）与有效电平，只做电平/数据控制，不再配引脚模式。

独立于这条链之外的两类：

| 目录 | 定位 | 规则 |
|---|---|---|
| `mcal/` | 厂商 SDK（`<vendor_sdk>`，版本钉死） | 不修改、可整体升级，是寄存器级代码 |
| `third_party/` | 第三方库 | 自包含、可整体摘走复用 |

`config/`（全局配置）与 `utils/`（平台无关工具）属于横向支撑。

---

## 2. 目录结构（形态 C）

第一方代码统一采用「**层 / 模块 / {include, src}**」，层的组合根文件直接放层根目录：

```text
<层>/
├── <层>.h                层的公共头
├── <层>.c                层的组合根（分布式组合根，见 §4）
└── <模块>/
    ├── include/          模块公开头（进 -I）
    └── src/              模块实现（只编译，不对外暴露）
```

### 2.1 两个例外

1. **`hal/` 可平铺**：hal 是「按外设实例」的薄封装（`hal_<periph>`），本质是板级文件集而非可复用模块，可把 `hal.h` / `hal.c` 与所有 `hal_*.{h,c}` 直接平铺在 `hal/` 根，不拆模块。
2. **`bsp/` 保持 `include/ src/ linker/ startup/`**：bsp 与具体 PCB 绑定，不是可复用组件，沿用传统 include/src 布局。

### 2.2 bsp 中断服务文件（bsp_isr）

- 板级中断 / 异常处理统一收敛到 `bsp/src/bsp_isr.c`（公开声明在 `bsp/include/bsp_isr.h`）。
- 命名用 `bsp_isr`（isr = Interrupt Service Routine），**不用** CubeMX 的 `stm32g4xx_it.c`：与工程 `bsp_` 前缀命名统一，且不把芯片型号焊死在文件名里，契合「换板只改 bsp」。
- 该文件等价于 CubeMX 生成工程里的 `stm32g4xx_it.c`，职责有三类：
  1. 内核异常向量（NMI / MemManage / BusFault / UsageFault / SVC / DebugMon / SysTick）；
  2. 外设中断服务程序（ADC / TIM / USART / DMA …），在 ISR 内转发到 hal / drivers 层回调；
  3. 故障现场捕获（fault 诊断，属调试工具逻辑）。
- `xxx_Handler` / `xxx_IRQHandler` 符号由启动文件（`startup_xxx.s`）的向量表引用，**文件名与符号名解耦**，重命名不影响链接。
- 若 RTOS 已接管某些异常（如 RT-Thread 接管 HardFault / PendSV），相关符号不在此文件重复定义。

---

## 3. 依赖规则（无环 DAG）

- 依赖图必须是 **DAG（有向无环）**。出现 A↔B 环形依赖 = 模块边界画错了，必须拆，不能绕过。
- 拆环四手段（按优先级）：

| 手段 | 适用场景 |
|---|---|
| ① 提取公共低层模块 | 两模块共享同一份数据/概念 |
| ② 依赖倒置（接口/ops 注入） | A 需要 B 的能力，但不该绑死 B |
| ③ 事件/消息队列 | A、B 异步生产/消费 |
| ④ 上提组合根 | A、B 只在某处被同时用到 |

- **模块内部不 include 对方实现**：依赖一律走 `init(self, 依赖…)` 的构造注入，由组合根接线。这样才能保证无环、可复用、可测试。

---

## 4. 组合根（分布式）

**组合根 = 唯一负责「实例化 + 依赖注入」的地方**。采用分布式组合根：每个层的 `<层>.c` 是它那一层的接线点，靠 RTOS 的 auto-init（或等价机制）拼成完整依赖图。

三条铁律：

1. **组合根零逻辑**：`<层>_init` 里只有「声明实例 + init 注入 + 建线程」，不出现业务判断（if/for/状态机）。
2. **构造注入，禁止内部 include**：依赖走 `init` 参数，模块内部不 `#include` 对方实现。
3. **实例化与线程分离**：实例化挂 `INIT_*_EXPORT`（早期），线程建在 `<层>_task_init`，二者永不混。

样板：

```c
// <层>/<层>.c —— 本层组合根，只接线
static <模块> s_<模块>;                        // 实例归组合根持有，模块不自持

static int <层>_init(void)
{
    <模块>_init(&s_<模块>,
                /* 依赖1 */,                  // 传输 / 数据源（注入）
                /* 依赖2 */);                 // 适配器 / 回调（注入）
    return 0;
}
/* 注册到初始化链：裸机在 main 按序调用，RTOS 用各自的 auto-init（见 §5） */
```

模块侧只认接口，不认具体实现。

---

## 5. 初始化顺序（分层、依赖有序）

初始化遵循「**分层、依赖有序**」：按 `bsp → hal → drivers → middleware → app` 的顺序，被依赖的层先初始化。**这条原则与是否用 RTOS 无关**，只是「注册方式」随运行环境不同：

| 运行环境 | 注册方式 |
|---|---|
| 裸机（无 RTOS） | 在 `main` 里按依赖顺序**手动调用**各 `<层>_init()` |
| RT-Thread | `INIT_*_EXPORT` 自动分级初始化 |
| FreeRTOS / ThreadX / 其它 RTOS | 在 `main`（或 ThreadX 的 `tx_application_define()`）里按顺序**手动调用**（通常无 auto-init 宏机制） |

RT-Thread 的自动分级示例：

| 等级 | 宏 | 用途 |
|---|---|---|
| 最早 | `INIT_BOARD_EXPORT` | 板级（hal / 时钟） |
| ↓ | `INIT_PREV_EXPORT` | 前置初始化 |
| ↓ | `INIT_DEVICE_EXPORT` | 设备 / 驱动 |
| ↓ | `INIT_ENV_EXPORT` | 中间件 / 环境 |
| 最晚 | `INIT_APP_EXPORT` | 应用 |

约定：**`<层>_init` 只做实例化 + 注入，`<层>_task_init` 只建线程（有 RTOS 时）**。跨层依赖靠初始化顺序保证：上层 init 晚于它所依赖的下层 init。

---

## 6. 命名约定

- **对象 / 结构体类型用带 tag 的 `struct`，不 typedef、不带 `_t`**：`struct drv_output`、`struct drv`（第三方类型保持原样）。
- **组合根**：`<层>.c` + `<层>_init()`。
- **线程**：`<层>_task.c` + `<层>_task_init()`（建线程）/ `<层>_task_entry()`（线程体）。
- **模块文件**：头文件与实现同名（`<模块>.h` ↔ `<模块>.c`），模块内尽量加模块前缀避免裸名冲突。

---

## 7. include 约定

- **裸文件名 include**：第一方代码统一 `#include "<模块>.h"`，不写层/模块前缀。
- **每个模块的 `include/` 目录进 `-I`**，层的根目录也进 `-I`（用于 `<层>.h`）。
- 第三方库保留各自前缀（如 `#include "modbus/modbus_slave.h"`）。

IncludePath 形态（以 Keil MDK 为例）：

```text
<toolchain>/<target>/…
..\bsp\include
..\hal
..\drivers
..\drivers\<模块>\include
..\app
..\app\<模块>\include
..\utils\<模块>\include
..\config
（third_party / rtos / mcal 同前）
```

> 形态 C 的代价就是 `-I` 条目随模块数增长；这是可复用/自包含换来的显式成本。

---

## 8. 回调与上下文（ctx）

C 回调是裸函数指针，无法天然拿到「自己是哪个对象」。两个可选方案：

- **加 `void *ctx`**：库在 `bind/init` 时存下用户对象，回调时原样还回——这是可多实例、可复用的终态（需要第三方库的回调签名支持 ctx）。
- **静态单例桥接**：模块内留一个 `static xxx *s_self`，回调里靠它找回实例——单实例够用，但牺牲多实例。

单实例场景用「实例由组合根持有 + 模块内 `s_self` 桥接」即可；要纯多实例，先给第三方回调加 `void *ctx`。

---

## 9. 构建

- 每个 `.c` 在工程里**显式列出**，不用目录通配。
- 不同工具链（Keil / IAR / CMake）可共存，每个 target 只引用当前平台需要的文件。
- 换板子：只改 `bsp/`（必要时 `hal/`），`drivers/`、`middleware/`、`app/` 不改。

---

## 10. 新增一个模块的 checklist

1. 建 `<层>/<模块>/{include,src}`。
2. 写 `<模块>.h`（公开接口）与 `<模块>.c`（实现），对象类型用带 tag 的 `struct`（不 typedef、不带 `_t`）。
3. 依赖一律走 `init(self, 依赖…)`，模块内部不 include 对方实现。
4. 把 `<模块>/include` 加进构建系统的 IncludePath。
5. 把 `<模块>/src/*.c` 加进构建系统的源文件列表。
6. 在对应层的 `<层>.c` 组合根里实例化 + 注入，并按 §5 注册到初始化链（裸机在 main 调用 / RTOS 用 auto-init）。
7. 需要线程时，写 `<层>_task.c`，`<层>_task_init` 建线程。

---

## 11. 面向对象设计

C 没有语言级 OOP，用「**struct + 自由函数**」模拟对象。约定：

1. **对象 = 带 tag 的 struct**：`struct <模块> { ... };`（不 typedef、不带 `_t`），对象只保存**运行时状态**。
2. **方法 = 自由函数 + `self`**：`void <模块>_<动作>(struct <模块> *self, ...)`，`self` 恒为第一个参数；公开方法放 `.h`，实现放 `.c`，内部辅助函数 `static`。
3. **实例归组合根持有**：`static struct <模块> s_<模块>;` 放在 `<层>.c`（组合根，见 §4），模块不自持实例。
4. **状态在对象里、配置在模块里**：
   - 运行时状态 → 对象成员（`self->xxx`），不用全局 / 文件静态变量。
   - 与板子 / 设备相关的**静态配置**（输出通道表、引脚映射等）→ 模块 `.c` 里的 `static const`，组合根不掺和（组合根零逻辑）。
5. **构造注入**：`<模块>_init(struct <模块> *self, 依赖...)` 初始化对象（`memset` + 填状态 + 硬件配置），依赖走 `init` 参数注入。
6. **入参校验**：公开方法对 `self` 做 NULL 检查；越界 / 非法参数静默返回（后续可接断言）。

示例（`drv_output`）：

```c
/* drivers/output/include/drv_output.h */
struct drv_output
{
    bool state[DRV_OUTPUT_COUNT];      /* 运行时状态 */
};

void drv_output_init(struct drv_output *self);
void drv_output_set(struct drv_output *self, drv_output_id_t id, bool on);

/* drivers/output/src/drv_output.c */
static const struct drv_output_channel s_channel[DRV_OUTPUT_COUNT] = { /* 静态配置 */ };

void drv_output_set(struct drv_output *self, drv_output_id_t id, bool on)
{
    if (self == NULL || (uint16_t)id >= (uint16_t)DRV_OUTPUT_COUNT)
    {
        return;
    }
    self->state[id] = on;
    /* ... */
}

/* drivers/drv.c —— 组合根：实例化 + 注入，零逻辑 */
static struct drv_output s_output;

int drv_init(void)
{
    drv_output_init(&s_output);
    g_drv.output = &s_output;
    return 0;
}
```

---

## 参考

- 编码规范：`docs/规范/coding_standard.md`
