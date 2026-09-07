# 工程规范

本目录存放**平台无关**的工程规范，可复用到任何嵌入式项目。

## 文档索引

| 文档 | 内容 |
|---|---|
| [architecture.md](./architecture.md) | 架构规范：分层、目录结构（形态 C）、依赖规则（DAG）、组合根、面向对象、初始化、命名与 include 约定 |
| [coding_standard.md](./coding_standard.md) | 编码规范：命名、代码风格、头文件、函数、注释（Doxygen）、控制流 |
| [submodule.md](./submodule.md) | 子模块使用说明：git submodule 的克隆、更新、改动提交、迁移与常见坑 |

## 阅读顺序

新项目先读 [architecture.md](./architecture.md) 定分层与目录，再读 [coding_standard.md](./coding_standard.md) 定写法；两份交叉引用，保持一致。
