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

## 备份与推送（进行中）

`a33df53` 之后本地已有 10 个提交。`origin` 指向上游作者仓库，**没有写权限，也不该往那推**；个人远端已配好，等 fork 建好后一条命令即可：

```bash
git remote -v
# origin  https://github.com/wokaka209/stm32f103-freertos-robot-car.git
# mine    https://github.com/sunx77891-ship-it/stm32f103-freertos-robot-car.git
```

1. **Fork 上游**（2026-11-21 选定此路线）：打开 <https://github.com/wokaka209/stm32f103-freertos-robot-car/fork> 建一份到自己的账号，仓库名保持 `stm32f103-freertos-robot-car`（`mine` 已按此地址配好；若改名，先 `git remote set-url mine <新地址>`）。
2. **首次推送**：

   ```bash
   git push -u mine main
   ```

   凭证由 Git Credential Manager 提供（本机已登录 `sunx77891-ship-it`）；若弹出浏览器登录或 2FA，按提示完成即可。
3. **确认落地**：`git ls-remote mine` 能看到 `refs/heads/main` 指向 `d3c7275` 一类的新提交，才算「已推送」。

本地兜底已存在：`../car-20261121.bundle`（1.3 MB，`--all` 全历史，`git bundle verify` 通过）。要恢复：

```bash
git clone ../car-20261121.bundle car-restore
```

**在 `git ls-remote mine` 成功之前，任何「已推送/已备份到远端」的说法都不成立**；换机或误删目录会丢掉全部文档与配置改动（`EIDE/.pack/`、`.cmsis/`、`build/` 不在版本控制里，需重装/重建）。

