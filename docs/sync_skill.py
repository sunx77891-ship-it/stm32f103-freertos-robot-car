#!/usr/bin/env python3
"""把仓库 docs/ 下的文档同步到本机 skill 目录。

权威副本在仓库：docs/skill/SKILL.md、docs/bringup-tutorial.md。
skill 目录（~/.workbuddy/skills/stm32-robot-car-bringup/）是运行副本，由本脚本覆盖写入。
改完 docs/ 后跑一次；换电脑时先 clone 仓库再跑一次即可恢复 skill。

用法：
    python docs/sync_skill.py           # 覆盖写入 skill 目录
    python docs/sync_skill.py --check   # 只比对，不写入；不一致时退出码 1
"""

import shutil
import sys
from pathlib import Path

SKILL_NAME = "stm32-robot-car-bringup"

# (仓库内源文件, skill 目录内相对路径)
PAIRS = [
    ("docs/skill/SKILL.md", "SKILL.md"),
    ("docs/bringup-tutorial.md", "references/bringup-tutorial.md"),
]


def normalized(path: Path) -> bytes:
    """按行尾归一化读取，避免 Windows 编辑器存成 CRLF 时误报不一致。"""
    return path.read_bytes().replace(b"\r\n", b"\n")


def main() -> int:
    check_only = "--check" in sys.argv
    repo = Path(__file__).resolve().parent.parent
    target = Path.home() / ".workbuddy" / "skills" / SKILL_NAME

    if not (repo / "docs").is_dir():
        print(f"[x] 找不到 {repo / 'docs'}，请在仓库内运行本脚本")
        return 2

    dirty = 0
    for src_rel, dst_rel in PAIRS:
        src = repo / src_rel
        dst = target / dst_rel
        if not src.is_file():
            print(f"[x] 缺源文件：{src_rel}")
            return 2

        same = dst.is_file() and normalized(dst) == normalized(src)
        if same:
            print(f"[=] 已一致：{dst_rel}")
            continue

        dirty += 1
        if check_only:
            state = "缺失" if not dst.is_file() else "不一致"
            print(f"[!] {state}：{dst_rel}")
            continue

        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(src, dst)
        print(f"[+] 已写入：{dst_rel}  ({src.stat().st_size} 字节)")

    if check_only:
        print(f"\n结论：{dirty} 个文件与仓库不一致" if dirty else "\n结论：skill 与仓库完全一致")
        return 1 if dirty else 0

    print(f"\n结论：同步到 {target}" + ("" if dirty else "（无需改动）"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
