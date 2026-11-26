# Repository Guidelines

复杂工程任务（涉及 3 个以上文件、架构决策或跨模块改动）必须使用
`.agents/skills/engineer/SKILL.md` 定义的 `engineer` Skill，通过
`.codex/agents/planner.toml` 和 `.codex/agents/implementer.toml`
子代理分层执行，并在每个验证通过的改动后提交 Git。

## Project Structure & Module Organization

This is an offline chassis reference and reuse bundle, not a complete deployable robot application. Start with `README.md` and `模块导航.md`.

- `architecture/`: hardware interfaces and connection documentation.
- `analysis/`: extracted binaries, firmware, logs, captures, decompiled pseudocode, and upstream references. Preserve provenance and licenses.
- `navigation_runtime/reverse/`: protocol documentation, verification scripts, and compilable C models under `recovered/`.
- `navigation_runtime/replacement/`: chassis, lidar, and IMU adapters with adjacent tests; `native_ros/` contains integration excerpts.
- Root assets include vendor manuals, the APK, `文件清单.json`, and `SHA256SUMS.txt`.

## Build, Test, and Development Commands

Run from the bundle root:

```sh
python3 校验资料包.py
cmake -S navigation_runtime/reverse/recovered -B /tmp/feedbot-factory-model-build
cmake --build /tmp/feedbot-factory-model-build -j 2
python3 navigation_runtime/replacement/chassis/run_tests.py
```

These commands respectively verify archived file sizes/hashes, configure and build offline models, and run disconnected chassis tests. Builds require CMake and a C compiler; chassis tests require Python 3 and `clang++` with C++17 and sanitizer support.

For differential verification, use a working copy:

```sh
sh navigation_runtime/reverse/verify_all.sh /path/to/venv/bin/python
```

This requires compatible dependencies, including Unicorn and pyelftools, and can overwrite historical reports. There is no bundle-wide application launch command.

## Coding Style & Naming Conventions

Match surrounding formatting; indentation varies across recovered and adapter code. Use four spaces for new Python blocks, descriptive `snake_case` filenames, and existing module namespaces. No bundle-wide formatter or linter is configured. Preserve relative paths, protocol units, byte ordering, and fixed frame layouts. Distinguish original binaries, decompiled output, recovered models, and upstream references in documentation.

## Testing Guidelines

Tests include standalone C++ executables, Python checks, and binary differential verification. Follow adjacent names such as `test.cpp`, `*_test.cpp`, and `test_*.py`. Cover malformed frames, boundary values, timeouts, and recovery behavior. No bundle-wide coverage threshold is documented. Follow `navigation_runtime/replacement/AGENTS.md` for regression and incident requirements; some referenced integration tooling may be absent from this extracted bundle.

## Commit & Pull Request Guidelines

This bundle has no `.git` history, so commit conventions cannot be inferred. Use concise imperative subjects, such as `Fix chassis feedback bounds`. Describe affected modules, evidence, commands run, results, and limitations. Separate offline validation from deployment or physical testing.

## Evidence & Hardware Safety

Keep archived evidence unchanged; develop in a working copy. Do not regenerate manifests merely to hide mismatches. Never automatically replay motion captures, flash firmware, or contend with factory device connections. Exclude private keys and tokens; review extracted configuration and logs before public sharing.
