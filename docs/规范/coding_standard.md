# 编码规范

本文档定义本工程（及按 `docs/规范/architecture.md` 组织的工程）的 C 编码规范。与架构规范一致，它**与具体芯片 / 工具链无关**。

约定优先级：本文档 > 架构规范中的命名/分层约束 > 第三方库自带的风格（第三方库不改，保持原样）。

---

## 1. 命名规范

### 1.1 文件

- 头文件与实现同名：`<模块>.h` ↔ `<模块>.c`。
- 全小写 + 下划线：`debug_monitor.c`、`drv_ain_sensor.c`、`ring_buffer.c`。
- 一个模块 = 一对 .h/.c（公开接口在 .h，实现在 .c）。

### 1.2 类型

- **对象 / 结构体类型用带 tag 的 `struct`，不 typedef**：`struct debug_monitor`、`struct dm_adapter`、`struct drv_motor`，使用处写 `struct 名称`。
- 定义：`struct 名称 { ... };`（struct tag 保留，便于前向声明、自引用与不透明指针）。
- 第三方类型（`mb_slave_handle`、`struct uart_control`、`mb_err_t`）保持原样不改。

```c
struct dm_adapter
{
    void (*init)(struct mb_slave_handle *slave);
    void (*poll)(void);
};
```

### 1.3 函数

- 全小写 + 下划线，`模块_动作` 格式：`debug_monitor_init`、`dm_adapter_refresh`、`hal_gpio_write`。
- 组合根：`<层>_init`；线程：`<层>_task_init` / `<层>_task_entry`。

### 1.4 变量

| 作用域 | 前缀 | 示例 |
|---|---|---|
| 文件内静态变量 | `s_` | `s_self`、`s_adapter`、`s_regs` |
| 全局变量 | `g_` | `g_drv`、`g_version`、`g_middleware` |
| 局部变量 | 无前缀 | `len`、`buf`、`val` |

- 实例归组合根持有：`static <模块> s_<模块>;`（模块内部不自己 new 实例）。
- **对象指针一律用 `self`**：函数操作的实例 / 句柄指针参数统一命名为 `self`（如 `debug_monitor *self`、`struct uart_control *self`、`struct mb_slave_handle *self`），不用 `hdl` / `uart` / `this` 等。

### 1.5 宏 / 常量

- 宏全大写 + 下划线：`UART_RX_BUF_SIZE`、`HAL_GPIO_LED_RED`。
- 枚举常量全大写 + 模块前缀：`TEST_REG_MAGIC`、`CTRL_REG_TARGET_SPEED`、`MB_FC_IAP`。
- 禁止魔法数字直接散落代码，用命名常量 / 枚举表达。
- **每个库暴露版本宏**：`XXX_VERSION_MAJOR` / `_MINOR` / `_PATCH`、`XXX_VERSION_STRING`（字符串）、`XXX_VERSION_NUM`（`major<<16 | minor<<8 | patch`，用于版本比较）。

### 1.6 枚举

- 用 `enum` 而非一串 `#define`，语义集中、可被编译器检查。
- 寄存器/状态类枚举用 `模块_REG_xxx` 前缀（见 §1.5）。

---

## 2. 代码风格

### 2.1 缩进与括号

- **4 空格**缩进，不用 Tab。
- **Allman 括号**：左括号独占一行，控制语句/函数体不省略括号。

```c
static void dm_on_tx_done(struct uart_control *uart)
{
    (void)uart;
    uart_control_enable_rx(s_uart);
}
```

### 2.2 空格

- 二元运算符两侧加空格：`a + b`、`x == 0u`。
- 关键字后加空格：`if (`、`while (`、`for (`、`switch (`。
- 逗号后加空格：`func(a, b, c)`。

### 2.3 行长度

- 尽量 ≤ 100 字符；超长时在运算符处换行，续行对齐。

### 2.4 空行

- 函数之间空一行；函数内逻辑段之间空一行（别挤成一坨）。

---

## 3. 头文件

### 3.1 包含保护

```c
#ifndef DM_ADAPTER_H
#define DM_ADAPTER_H
...
#endif /* DM_ADAPTER_H */
```

### 3.2 自包含

- 头文件自身可独立编译：用到的类型自己 `#include`，不依赖包含顺序。
- 只 include 必要头，能前置声明就前置声明（指针/引用可 `struct xxx;`）。

### 3.3 公开 / 私有边界

- `.h` 只放对外接口（类型 + 函数声明），不暴露内部实现细节。
- 模块内部用的函数/变量一律 `static`，不放 .h。
- 芯片寄存器头、HAL/LL 细节留在 .c 或私有配置里，不向 app/drivers/middleware 暴露。

---

## 4. 函数

### 4.1 职责单一

- 一个函数只做一件事；组合根零逻辑（见架构规范 §4）。

### 4.2 参数与返回

- 依赖走 `init(self, 依赖…)` 构造注入，模块内部不 include 对方实现。
- 出错返回错误码（`MB_OK` / `MB_ERR_*` 这类枚举），不用裸 `-1`/魔法值。

### 4.3 NULL 检查

- 对传入指针做入参校验：

```c
void debug_monitor_init(debug_monitor *self, uint8_t slave_addr, struct uart_control *uart)
{
    if (self == NULL || uart == NULL)
    {
        return;
    }
    ...
}
```

---

## 5. 变量与类型

### 5.1 定长整型

- 一律用 `stdint.h`：`uint8_t` / `uint16_t` / `uint32_t` / `int16_t` / `bool`，不用裸 `int`/`char` 表示位宽敏感数据。

### 5.2 static 限定

- 文件内可见的一律 `static`，避免污染符号表。

### 5.3 const

- 不该被修改的指针/参数加 `const`：`const struct mb_reg_map *`、`const uint8_t *buf`。

### 5.4 类型转换

- 收窄/有符号转换用显式 cast，别靠隐式：`(uint16_t)(...)`、`(int16_t)(...)`、`(float)(int16_t)val`。
- 字面量后缀：无符号 `0u`、`0x1234u`，长整型 `UL`。

---

## 6. 注释（Doxygen 风格）

注释统一采用 **Doxygen 风格**，便于生成 API 文档与 IDE 悬浮提示。一律用 `/** ... */` 块注释，行内用 `/**< ... */`。

### 6.1 文件头

```c
/**
 * @file    dm_adapter.h
 * @brief   debug_monitor 库的测试适配器
 */
```

### 6.2 函数注释

公开接口用完整 Doxygen 标签（`@brief` / `@param` / `@retval`）：

```c
/**
 * @brief 初始化调试接口（绑定 uart_control 传输 + dm_adapter 寄存器/适配器）
 * @param self       对象实例
 * @param slave_addr Modbus 从站地址（如 0x01）
 * @param uart       uart_control 实例（由上层注入）
 */
void debug_monitor_init(struct debug_monitor *self, uint8_t slave_addr, struct uart_control *uart);
```

- 简单内部函数可用一行 `/** ... */`。
- 注释说明「为什么」，不是复述代码「做什么」。

### 6.3 行内注释

- 用 `/**< ... */` 跟在字段/变量后：`uint16_t *data;  /**< 连续数组 */`。
- 中文注释与英文注释均可，保持模块内一致。

---

## 7. 控制流

### 7.1 if / else

- 单语句也加大括号（见 §2.1）。
- 常量放左侧防误写：`if (0u == val)`（团队可选，保持一致即可）。

### 7.2 switch-case

- 按地址/枚举分发用 `switch`（比 if-else 链清晰、易扩展），必须带 `default`：

```c
switch (addr)
{
case CTRL_REG_TEST_WORD:
    s_ctrl_regs[CTRL_REG_TEST_WORD] = val;
    return MB_OK;
default:
    return MB_ERR_ADDR;
}
```

### 7.3 循环

- 死循环写 `while (1)` 或 `for (;;)`，模块内统一。

---

## 8. 回调与上下文（ctx）

- 判断是否需要 ctx（见架构规范 §8）：回调参数里拿不到实例句柄、又需访问实例状态、且可能多实例 → 加 `void *ctx`。
- 明确单实例的业务模块用 `static xxx *s_self` 桥接即可。
- 可复用库 / third_party 级别的回调**一律带 ctx**。

---

## 9. 其它

### 9.1 初始化

- 变量声明时初始化，别依赖隐式零值：`static uint16_t s_ctrl_regs[2] = { 0x0000u, 0x0000u };`。
- 结构体用 `memset(&obj, 0, sizeof(obj))` 或指定初始化器 `{ .init = ..., .poll = ... }`。

### 9.2 未使用参数

- 回调/接口的未用参数用 `(void)param;` 显式标记，别留编译器警告。

### 9.3 修改第三方库

- **不改第三方库源码**；需要扩展时在外部包一层，或通过注入（ops/ctx）适配。

---

## 参考

- 架构规范：`docs/规范/architecture.md`
- 规范索引：`docs/规范/README.md`
