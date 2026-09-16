# Design: Trim RumbleBot to Drivebase-Only

**Date:** 2026-09-16
**Branch:** `DEV/brennan/trim_old_code` (off `DEV/Trimming`)
**Target event:** Oct 24-25, 2026 offseason

## 1. Goal and Non-Goals

**Goal.** Reduce the robot program to the drivebase and its supporting
infrastructure, so that on Oct 24 the only code that can fail is code we need.
Consolidate 2026 game and field information into a `FieldData` class, the first
piece of the xUML model to land in the tree.

**Non-goals.**

- No behavior change to the drivebase. Driving after the trim must be
  indistinguishable from driving before it.
- No re-tuning of PID or feedforward constants.
- No re-architecting `SwerveDrive` or `SwerveModule`.
- No swerve module offset calibration. The offsets in `Constants.hpp` are all
  zero and stay zero; correcting them is physical bring-up, tracked separately.

## 2. Context

The repository currently builds clean and produces a deployable roboRIO binary.
This is a reduction of working code, not a repair of broken code.

The dependency graph makes this a clean cut rather than a disentangling job.
`SwerveDrive` depends only on `PoseEstimator`, `POIGenerator`, `PoseFilter`,
`Constants.hpp`, and `SwerveModule`. No path leads from the drivebase into any
turret, wrist, intake, or LED code.

Note on terminology: the hub/fuel code is **current-season 2026 logic**, not
stale 2025 residue. The October event is an offseason for the same game. This
is why field and game data is preserved rather than deleted.

## 3. Removals

| Category | Files |
| --- | --- |
| Subsystems | `Turret`, `Turret_Shooter`, `TurretIntake`, `Wrist`, `LEDController`, `LED_Groups` (`.cpp` + header each) |
| Commands | `Intake`, `Shoot`, `FlattenMoonKnight`, `HalfRaiseIntake`, `BetterHalfRaise` |
| Ballistics | `utils/LaunchCalculator`, `utils/BallisticsInterpolator`, `include/utils/ballistics_rv_hub.h`, `include/utils/ballistics_rv_gnd.h` |
| Deploy asset | `src/main/deploy/LaunchCalculator_Points.csv` |
| Constants | `FuelConstants` and `OperatorConstants` namespaces in `Constants.hpp` |

`TurretConstants` and `WristConstants` are defined inside `Turret.h` and
`Wrist.h` respectively, so they are removed with their headers.

Both removed namespaces are already dead. `FuelConstants` (feeder and launcher
motor IDs and voltages) has no references anywhere in `src/`. `OperatorConstants`
is likewise unreferenced and duplicates values that live in `DriveConstants`:
the controller ports in active use are `DriveConstants::kDriverPort` and
`kOperatorPort`. Removing both is therefore a no-op at compile time.

Approximately 2,000 lines are deleted. `Robot.cpp` drops from 518 lines to
roughly 200.

## 4. Retentions

Kept and untouched: `SwerveDrive`, `SwerveModule`, `PoseEstimator`,
`PoseFilter`, `POIGenerator`, the `AutoWheelOffsets` command, PhotonVision,
PathPlanner `AutoBuilder`, `ElectricalConstants`, `DriveConstants`,
`ModuleConstants`, `MathUtilNK`.

Three retentions are deliberate and warrant stating:

- **`NetworkTableMap` and `DeployFileUtils`.** Both are generic, not turret
  specific. `NetworkTableMap` is a templated key-traits map syncing
  NetworkTables against CSV; `DeployFileUtils` resolves deploy paths and is
  simulation aware. Only `LaunchCalculator` above them is turret logic. Keeping
  them also preserves `NetworkTableMapTest.cpp`, the repository's only test.
- **`Sharp_IRDistanceSensor`.** Kept deliberately unreferenced. `Wrist` is its
  only consumer today, so the trim orphans it, but it is recent work (merged
  from `IR_Sensor` in August) that the modeled Intake and Indexer classes will
  likely want. It continues to compile as part of the build.
- **Vision enabled by default.** Baseline simulation confirms PhotonVision
  degrades to warnings, not crashes, when all three cameras are absent. This
  keeps drivebase behavior unchanged, per the non-goals.

## 5. New Component: `FieldData`

New files `src/main/include/subsystems/FieldData.h` and
`src/main/cpp/subsystems/FieldData.cpp`.

`FieldData` absorbs the `FieldConstants` namespace (hub translations, the
`FieldZone` struct with `IsInside()`, and the alliance/neutral zone definitions)
together with the `CheckActiveHub()` logic relocated off `Robot`.

It is a plain class, not a `SubsystemBase`. It owns no hardware and needs no
scheduler slot.

**Pure relocation, with exactly one behavioral change.** `CheckActiveHub()`
currently reaches the end of a non-void function when the game message is `"R"`
or `"B"` but the match timer falls outside every defined window, returning an
indeterminate value. The relocated version returns an explicit default on that
path. No other logic changes and no new API surface.

**The relocation has no call sites to update.** Neither `FieldConstants` nor
`CheckActiveHub()` is referenced anywhere in `src/` today — `CheckActiveHub` is
declared in `Robot.hpp` and defined in `Robot.cpp` and called from nowhere. The
move therefore cannot break a caller.

**`FieldData` is intentionally unreferenced after the trim**, in the same way
and for the same reason as `Sharp_IRDistanceSensor`: it preserves live 2026 game
logic in the shape the model calls for, ready for `Robot State` and `Autonomous`
to consume during the rebuild. It still compiles as part of the build, so it
cannot silently rot. It is not dead code left behind by accident, and should not
be removed as such.

## 6. `Robot` Changes

Removed from `Robot.hpp`: includes for the deleted subsystems and commands;
members `m_wrist`, `m_turret`, `m_intake`, `m_led`; the 3D model pose publishers
(`stageOne3dPOS`, `carage3dPOS`, `wrist3dPOS`, `wristPOS`, `climb3dPOS`,
`modelPosePublisher`); and `m_operatorController`.

Removed from `Robot.cpp`:

- Six `NamedCommands::registerCommand` registrations in `CreateRobot()`.
- **Both `AddPeriodic` callbacks** — wrist at 10 ms, turret at 5 ms.
- `DisabledInit`: `m_turret.SaveLaunchMapToFile()`, `m_turret.PublishLaunchMap()`,
  `m_led.DefaultAnimation()`. The `m_swerveDrive` simulation resets stay.
- `DisabledPeriodic`: `m_turret.PublishLaunchMap()`.
- `TeleopInit`: `m_turret.Reset()`, `m_led.TeleopInit()`.
- `TeleopPeriodic`: `m_led.TeleopPeriodic()` (leaving the body empty).
- `RobotPeriodic`: the model pose block building `IntakePose3D`,
  `SpindexerPose3D`, `ShooterPose3D`, `HoodPose3D` and publishing `ModelPoses`.
- `CheckActiveHub()`, relocated to `FieldData`.

`m_pdh.SetSwitchableChannel` is currently driven by a turret shooting flag read
from SmartDashboard. With the turret gone nothing writes that key, so it is
pinned to `false`.

### Controller bindings after the trim

Removed: driver buttons 1, 2, 5, 6; all operator bindings; the operator POV
event loop and both operator `POVButton` bindings. The **driver** POV-down
binding for `MakeX` is kept.

The complete surviving control set:

| Input | Action |
| --- | --- |
| Left stick X/Y, right stick X | Field-relative drive (4 m/s, 6 rad/s, 0.15 deadband) |
| Button 3 | Reset heading |
| Button 4 (hold) | Slow mode (1 m/s, 2 rad/s) |
| D-pad down | X-lock wheels |
| SmartDashboard "Set" | `AutoWheelOffsets` calibration |

`m_DebugController` is retained despite being unused; it costs nothing and is
useful during bring-up.

## 7. Execution Strategy

Top-down in three staged commits, each ending at a tree that passes both gates
in section 8.

1. **Gut `Robot`.** Remove members, includes, bindings, periodic callbacks, and
   the model pose block. Definitions still exist, so the compiler reports
   dangling references precisely (`m_turret` not declared) rather than a cascade
   of missing headers.
2. **Extract `FieldData`.** Move `FieldConstants` and `CheckActiveHub`, fix the
   missing-return path.
3. **Delete files.** Remove the now-unreferenced subsystems, commands,
   ballistics, deploy asset, and `FuelConstants`.

Deleting last is deliberate: it keeps error messages pointing at the actual
problem throughout. Each stage is independently revertable and bisectable.

## 8. Verification

Both gates must pass before each of the three commits.

### Gate A — Build

1. `./gradlew clean build` succeeds reporting `N actionable tasks: N executed`.
   A cached `UP-TO-DATE` result is not acceptable evidence.
2. `NetworkTableMapTest` passes.
3. `grep -ri "turret\|wrist\|LEDController\|LED_Groups" src/main` returns
   nothing (final stage only).
4. The `linuxathena/release/frcUserProgram` binary is produced.

Requires `JAVA_HOME` set to the WPILib JDK; see section 10.

### Gate B — Simulation smoke test

Run `./gradlew simulateNativeRelease` headless under a 60-second timeout and
compare against the captured baseline:

| Assertion | Baseline |
| --- | --- |
| `Robot program startup complete` occurrences | exactly 1 |
| `unhandled`, `terminate called`, `segmentation`, `Aborted`, `what():` | 0 |
| Distinct `Error at` categories | 1 (`PrintLoopOverrunMessage`, a startup transient) |
| Process alive at 60s | yes — `timeout` exit code 124 is the pass condition |

**Why this gate exists.** `AddPeriodic` callbacks run on the `TimedRobot`
schedule regardless of enable state, so even a disabled simulation run exercises
the 10 ms wrist and 5 ms turret timers. If either survives the trim as a
dangling callback into a destroyed object, the build stays green and only
simulation catches it. This is the highest-risk edit in the job.

**Additional post-trim confirmation.** Every REVLib device (`SparkMax`,
`SparkBase`) lives in `Turret`, `Turret_Shooter`, and `Wrist`; every Phoenix 5
device (`CANdle`) lives in `LED_Groups`. After the trim the baseline's
`SparkBase::SetInverted` warning and all Phoenix 5 deprecation warnings should
disappear. Their absence is positive evidence the removal was complete.

**Simulation currently destroys tuning data — check the working tree after every
Gate B run until stage 3 lands.** On the pre-trim code, `DisabledInit()` calls
`m_turret.SaveLaunchMapToFile()`. The launch map is empty in simulation, so this
writes an empty table over `src/main/deploy/LaunchCalculator_Points.csv`,
silently discarding all fifteen rows of shooter tuning. This was observed while
capturing the baseline and reverted with `git checkout --`.

Two consequences. First, `git status` must be checked after each Gate B run on
stages 1 and 2, and any modification to that CSV reverted. Second, this is an
independent argument for the trim: removing `Turret` removes the call, and the
footgun disappears with it. Until then, anyone running simulation on this branch
loses that tuning without warning.

**Known gap.** Disabled-mode simulation does not execute `TeleopInit` or
`TeleopPeriodic`, so teleop-only paths are not covered at runtime. Those are
compile-time references, so a missed one fails Gate A instead. Driving the
driver station into teleop headless is possible but does not earn its
complexity here. The gap is named rather than papered over.

## 9. Risks

| Risk | Severity | Mitigation |
| --- | --- | --- |
| A surviving `AddPeriodic` callback into a destroyed object | High — crash on boot, invisible to the compiler | Gate B |
| Missed reference in a rarely-compiled path | Medium | Gate A stage-by-stage, plus the grep sweep |
| `ModelPoses` dashboard 3D widget stops updating | Low, cosmetic and expected | Documented here |
| Zeroed module offsets make the robot drive wrong | High, but out of scope | Explicit non-goal; hardware bring-up on the 24th |

## 10. Build Environment

Building from a terminal rather than the VS Code WPILib extension requires:

```bash
export JAVA_HOME="/c/Users/Public/wpilib/2026/jdk"
./gradlew clean build
```

Three traps produce false signals and are worth knowing:

- Without `JAVA_HOME`, Gradle cannot start. The VS Code extension sets it, so
  builds succeed in the IDE and fail in a bare shell.
- Without `clean`, Gradle reports `BUILD SUCCESSFUL` while compiling nothing.
  Always confirm `N executed`.
- `-x test` fails this project. C++ has no `test` task; the candidates are
  `testExternalNativeDebug` and `testExternalNativeRelease`.
- Piping Gradle into `tail` or `head` replaces Gradle's exit code with the
  pager's.

A full clean build takes roughly five minutes.

## 11. Follow-Up Work (Out of Scope)

- Swerve module offset calibration via `AutoWheelOffsets` — required before the
  robot drives correctly, and gated on assembled hardware.
- `SwerveDrive::GetHeading()` has a no-return path in its NavX branch, currently
  unreachable because `m_usingPigeon` is hardcoded true. Worth fixing, but it is
  drivebase code and outside a trim.
- `REVLib.json` and the Phoenix 5 vendordep become unused once their only
  consumers are deleted, and could be dropped.
- Remaining modeled classes: Robot State, Input, Intake, Extender, Indexer,
  Dumper, Autonomous, Vision/Pose.
