# docs/ 文档地图

本目录收「怎么动手」的文档，与仓库根的 `AGENTS.md`（「怎么改代码」的硬约束）分工不同。

| 文件 | 作用 | 权威性 |
|---|---|---|
| `../AGENTS.md` | 代码事实与硬规则：布局、构建、引脚、任务、协议、参数、已知缺陷、上板闸门 | **代码事实以此为准** |
| `skill/SKILL.md` | 可执行 skill：12 阶段闸门、逐阶段执行卡、安全拦截、排障速查、回答格式 | **上板流程以此为准** |
| `bringup-tutorial.md` | 新手保姆级落地教程全文（约 2200 行），含每步完整措辞、`〔外部〕` 来源链接、完整「需补充」清单 | 细则与出处 |
| `sync_skill.py` | 把上面两份同步到本机 skill 目录 | — |

## 为什么要有 sync_skill.py

skill 必须放在 `~/.workbuddy/skills/stm32-robot-car-bringup/` 才会被自动加载，而那个目录不在版本管理里，换电脑就丢。所以：

- **仓库 `docs/` 是唯一权威副本**，随 `git` 走
- **skill 目录是运行副本**，由脚本从仓库覆盖写入

改文档的流程固定为：改 `docs/` → 跑一次同步 → 提交。

```bash
# 覆盖写入本机 skill 目录
python docs/sync_skill.py

# 只比对不写入（不一致时退出码 1，适合提交前自查）
python docs/sync_skill.py --check
```

换新电脑：`git clone` 仓库 → `python docs/sync_skill.py` → skill 恢复。

## 不要做的事

- 不要直接改 `~/.workbuddy/skills/...` 下的文件 —— 下次同步会被覆盖，改动就丢了
- 不要把 `EIDE/.pack/`、`EIDE/build/` 之类产物放进本目录（已在 `.gitignore` 排除）
- 本仓库上游**没有项目级 LICENSE**：这些文档是对该仓库的分析与自己写的流程，可以随本仓库版本管理，但不要把上游源码当作自己的原创作品对外发布

## 备份与推送（当前未闭环）

本地已有 7 个提交（`a33df53` 之后），**全部只在本地**，因为 `origin` 指向的是上游作者仓库：

```bash
git remote -v
# origin  https://github.com/wokaka209/stm32f103-freertos-robot-car.git
```

直接 `git push origin main` 会被拒（无写权限），也不应该往上推。要让提交可留存的两种做法：

1. **Fork 后改 remote（推荐）**：在 GitHub fork 一份 → `git remote add mine https://github.com/<你的账号>/stm32f103-freertos-robot-car.git` → `git push mine main`。
2. **只在本地留档**：把仓库整体复制/打包到备份盘，或用 `git bundle`：

   ```bash
   git bundle create ../car-$(date +%Y%m%d).bundle --all
   ```

在此之前，任何「已推送/已备份」的说法都不成立；换机或误删目录会丢掉全部文档与配置改动（`EIDE/.pack/`、`.cmsis/`、`build/` 不在版本控制里，需重装/重建）。

