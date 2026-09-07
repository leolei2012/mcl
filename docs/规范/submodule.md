# 子模块使用说明

本文档说明本工程第三方库（`third_party/`）的 git submodule 使用约定与操作流程。

> 前置：本文档配套 `.gitmodules`（工程根目录）与 `third_party/README.md`（库清单）。

## 1. 概念

子模块（submodule）= 主仓库里嵌一个「指针」，指向另一个 git 仓库的某个 commit。
- 主仓库**只存 SHA**（gitlink），不存子模块的代码内容。
- 子模块的代码、历史、提交，都在**它自己的仓库**里。

## 2. 本工程的子模块

| 目录 | 远端仓库 |
|---|---|
| `third_party/uart_control` | https://github.com/leolei2012/uart_control.git |
| `third_party/mb_rtu` | https://github.com/leolei2012/mb_rtu |

> 其余库暂为 vendor（直接拷贝），待有远端仓库后再迁移，见 `third_party/README.md`。

## 3. 首次克隆

```bash
# 克隆时连带拉取子模块
git clone --recurse-submodules <主仓库URL>

# 或已克隆后补拉
git submodule update --init --recursive
```

## 4. 更新子模块到最新

```bash
# 拉取某个子模块远程最新 commit
git submodule update --remote third_party/uart_control

# 回主仓库记录新 SHA 并提交
git add third_party/uart_control
git commit -m "update uart_control"
```

> 子模块本质是「钉死某个 commit」；「更新到最新」是一次显式动作，更新完它又变回钉死状态。

## 5. 改动子模块代码（核心流程）

顺序很重要，**先提交子模块、推远端，再在主仓库记录 SHA**：

```bash
# ① 在子模块目录内改代码并提交（提交到子模块自己的仓库）
cd third_party/uart_control
git add -A
git commit -m "fix: xxx"
git push origin HEAD
cd ../..

# ② 回主仓库记录新 SHA
git add third_party/uart_control
git commit -m "update uart_control submodule"
```

**为什么必须先 push 子模块**：主仓库只记 SHA；如果那个 commit 只在你本地子模块仓库、没推到远端，
别人 `git submodule update` 会报 `commit not found`。

## 6. 新增子模块

```bash
git submodule add <url> <path>

# 路径已存在且是 git 仓库时，用 --force 复用本地仓库（不重新 clone）
git submodule add --force <url> <path>
```

## 7. 迁移 vendor 库为子模块（本工程步骤）

以把现有 `third_party/uart_control` 转成子模块为例：

```bash
# ① 子模块本地改动先提交并推送
cd third_party/uart_control && git add -A && git commit -m "sync" && git push origin HEAD && cd ../..

# ② 主工程初始化（若还不是 git 仓库）
git init

# ③ 复用本地仓库注册为子模块
git submodule add --force https://github.com/leolei2012/uart_control.git third_party/uart_control

# ④ 首次提交
git add -A && git commit -m "add uart_control submodule"
```

## 8. 常见坑

- **clone 后子模块目录是空的**：忘了 `--recurse-submodules`，补 `git submodule update --init --recursive`。
- **`commit not found`**：子模块的 commit 没 push 到远端，主仓库就记录了这个 SHA。
- **子模块有未提交改动时 `git add <子模块>` 收不进去**：必须先在子模块里 commit，主仓库只记 SHA。
- **更新后忘了提交主仓库**：别人还是拿到旧 SHA，主仓库 `git status` 会显示 `(new commits)`。
- **子模块改完只 push 子模块、忘 push 主仓库的 SHA 更新**：两者都要提交。

## 9. 删除 / 重命名子模块

```bash
# 删除
git submodule deinit -f <path>
git rm -f <path>
rm -rf .git/modules/<path>   # 清理元数据

# 重命名：先改 .gitmodules 的 path + git mv 目录，再同步 .git/modules 路径
```

> 删除/重命名较繁琐，建议谨慎操作；临时排查可用 `git submodule status` 看当前各子模块的 SHA 状态。