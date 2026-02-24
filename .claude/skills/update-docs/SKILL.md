---
name: update-docs
description: Update CLAUDE.md and project_status.md to reflect the current state of the codebase
---

# Update Project Documentation

Regenerate both `CLAUDE.md` and `project_status.md` to reflect the current state of the codebase.

## Process

### 1. Gather current state

Explore the codebase to build an accurate picture:

- **MCU firmware**: Read `mcu_ws/platformio.ini` for environments, `mcu_ws/src/robot/machines/RobotLogic.h` for active subsystems, `mcu_ws/lib/` for drivers and libraries.
- **ROS2 packages**: List directories in `ros2_ws/src/`, read each `package.xml` for version/status, check for actual source code vs scaffolding.
- **CI/CD**: Read all files in `.gitea/workflows/` to document pipeline status.
- **Scripts**: Read key scripts (`scripts/deploy-all.py`, `scripts/flash_mcu.sh`, `scripts/button-trigger-workflow.py`) for deployment pipeline state.
- **Tests**: Read `mcu_ws/test/` structure and `mcu_ws/scripts/run_all_mcu_tests.sh` for test coverage.
- **TODOs**: Grep for `TODO`, `FIXME`, `HACK` in `mcu_ws/src/` and `ros2_ws/src/` to find outstanding work.
- **Git**: Check recent commits (`git log --oneline -10`) and current branch for activity context.

### 2. Update CLAUDE.md

Read the existing `CLAUDE.md` and update it to accurately reflect:

- Repository structure (platforms, source organization, libraries)
- Development commands (build, flash, test, deploy)
- Architecture notes (subsystem design, PlatformIO config, micro-ROS, message sharing)
- Common workflows (adding field elements, adding subsystems, testing)
- Test architecture and commands

Preserve the existing structure and style. Only change sections where the codebase has diverged from what's documented. Do NOT remove sections that are still accurate.

### 3. Update project_status.md

Read the existing `project_status.md` and regenerate it with:

- **Maturity summary table**: Every MCU subsystem, ROS2 package, test suite, and CI workflow with current status (Mature / Implemented / Scaffolding / Not Integrated / Deprecated)
- **Critical path items**: Blocking issues with specific file references and line numbers
- **Hardware drivers table**: All implemented sensor/actuator drivers with IC, bus, and location
- **CI/CD workflows table**: All workflows with triggers, runners, and purpose
- **Deployment pipeline diagram**: ASCII flow from button press to running nodes
- **Recent changes section**: Notable changes on the current branch vs master

Update the "Last Updated" date and branch/commit info at the top.

### 4. Verify accuracy

After writing both files:

- Confirm every environment listed in CLAUDE.md actually exists in `platformio.ini`
- Confirm every ROS2 package listed actually exists in `ros2_ws/src/`
- Confirm deployment commands match the actual scripts
- Confirm test commands match the actual test infrastructure

### 5. Report changes

Summarize what was updated in each file and highlight any new findings (new packages, removed features, status changes).
