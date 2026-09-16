# Known Bugs

Running list of defects found in the RumbleBot codebase. Add new entries at the
bottom with the next `B` number; do not renumber existing entries, since commit
messages and specs reference them.

Each entry records how it was verified, because "the compiler said so" and "this
looks wrong to me" deserve different amounts of trust.

**Status key:** `OPEN` · `FIXED` (with commit) · `SCHEDULED` (fix planned, not
yet applied) · `DISMISSED` (investigated, not a bug)

| ID | Summary | Severity | Status |
| --- | --- | --- | --- |
| B1 | X-lock command declares no subsystem requirement | High | FIXED `e88b8d4` |
| B2 | Vision std devs overwritten every cycle | Medium | FIXED `4eaa4eb` |
| B3 | Acceleration limiter stores state in NetworkTables | Medium | OPEN |
| B4 | `GetHeading()` has a path returning an indeterminate value | Medium | OPEN |
| B5 | `CheckActiveHub()` throws when no alliance is known | Medium | OPEN |
| B6 | `CheckActiveHub()` missing return path | Low | FIXED `f9cb0a8` |
| B7 | Simulation destroys shooter tuning CSV | Low | FIXED `e88b8d4` |
| B8 | Dead PID code with inverted logic | Cosmetic | OPEN |
| B9 | `Drive()` suspected of producing NaN on zero input | — | DISMISSED |
| B10 | PathPlanner GUI config still draws a turret outline | Cosmetic | OPEN |

**Open items after the drivebase-only trim:** B3, B4, B5, B8, B10. B4 and B5 are
the two that matter, because both are currently unreachable and will become
reachable as the rebuild adds callers.

---

## B1 — X-lock command declares no subsystem requirement

**Severity:** High · **Status:** FIXED in `e88b8d4` · **Verified:** by inspection

> **Fix applied.** The `RunCommand` now declares `{&m_swerveDrive}`, so it
> interrupts the drive default command instead of running alongside it.
> **Still needs on-robot confirmation:** simulation proves it does not crash, but
> only a driver holding the D-pad with a real controller can confirm the lock
> actually holds. Put this on the Oct 24 bring-up checklist.

**Location:** `src/main/cpp/Robot.cpp:350` (pre-fix)

The D-pad-down X-lock binding creates a `RunCommand` with no requirements:

```cpp
POVDownTrigBR.WhileTrue(
    frc2::CommandPtr(frc2::RunCommand([this] {
        m_swerveDrive.MakeX(true);      // no {&m_swerveDrive}
    })))
```

The drive default command *does* declare `{&m_swerveDrive}`. A command with no
requirements never interrupts anything, so both run every scheduler cycle and
both call `SetDesiredState` on all four modules. Last writer wins.

**Failure:** holding D-pad down makes the X-lock fight the drive command. Wheels
jitter or ignore the X rather than locking.

**Fix:** pass `{&m_swerveDrive}` as the second argument to `RunCommand`.

---

## B2 — Vision standard deviations overwritten every cycle

**Severity:** Medium · **Status:** FIXED in `4eaa4eb` · **Verified:** by inspection

> **Fix applied.** The per-cycle override is gone; `SetVisionMeasurementStdDevs`
> now appears exactly once, in the constructor. **Still needs on-robot
> confirmation:** validating that pose estimation actually improved requires
> cameras and AprilTags, which simulation does not provide.

**Location:** `src/main/cpp/subsystems/SwerveDrive.cpp:375` (pre-fix)

The constructor sets deliberate asymmetric values at line 62:

```cpp
auto visionStdDevs = wpi::array<double, 3U>{0.2, 0.2, 0.9};
```

Lower means more trust, so this trusts vision X/Y strongly and vision heading
weakly — correct, because the Pigeon 2 measures heading far better than AprilTag
geometry. But `UpdatePoseEstimate()` runs every `Periodic()` while vision is on
and overwrites it:

```cpp
m_poseEstimator.SetVisionMeasurementStdDevs({1.0, 1.0, 1.0});
```

**Failure:** the constructor's tuning is dead code. Vision is trusted uniformly,
and heading is trusted more than intended relative to the gyro. No crash, just
silently degraded pose estimation.

**Fix:** delete the line in `UpdatePoseEstimate()`. Do not replace it — the
constructor already sets the right values once, in the right place.

---

## B3 — Acceleration limiter stores its state in NetworkTables

**Severity:** Medium · **Status:** OPEN
**Location:** `src/main/cpp/subsystems/SwerveDrive.cpp:197-199` · **Verified:** by inspection and test

`Drive()` reads its previous-velocity state back out of the dashboard every
20 ms instead of from a member variable:

```cpp
auto prevVX = frc::SmartDashboard::GetNumber("drive/vx", 0.0);
auto prevVY = frc::SmartDashboard::GetNumber("drive/vy", 0.0);
double accelLimit = frc::SmartDashboard::GetNumber("drive/accelLim", 4.0);
```

The limiter arithmetic itself is correct — a throwaway test confirmed a 1.0 m/s
command yields 0.08 m/s on the first cycle, a proper 4 m/s² ramp. The storage
location is the problem.

**Failure modes:**

1. NetworkTables topics are writable by any client, so anyone with a dashboard
   can write `drive/vx` and alter drive behavior.
2. `Strafe()` and `MakeX()` set module states without updating `drive/vx`, so
   the limiter's notion of current speed goes stale. Coming out of X-lock it
   believes the robot is still moving at the pre-lock speed and permits an
   instant jump to it instead of ramping.

**Suggested fix:** hold previous velocity in a private member. Keep publishing
it to the dashboard for telemetry, but never read control state back from there.
Leave `accelLim` on the dashboard if live tuning is wanted — that one is an
input, not state.

---

## B4 — `GetHeading()` has a path returning an indeterminate value

**Severity:** Medium (latent) · **Status:** OPEN
**Location:** `src/main/cpp/subsystems/SwerveDrive.cpp:262-281` · **Verified:** by compiler

```
warning: control reaches end of non-void function [-Wreturn-type]
```

The NavX branch of the `if (m_usingPigeon)` check has every line commented out,
so it falls off the end of a non-void function and returns an indeterminate
`Rotation2d`. Unreachable today only because `m_usingPigeon` is hardcoded `true`
at `SwerveDrive.hpp:122`.

**Why it matters anyway:** the README records that NavX was removed
"temporarily". Whoever restores it gets a robot whose heading is garbage, and
field-relative drive on a garbage heading is dangerous rather than merely broken.

**Suggested fix:** either delete the NavX branch entirely along with
`m_usingPigeon`, or restore it properly. A dead branch that silently returns
garbage is the worst of the three options.

---

## B5 — `CheckActiveHub()` throws when no alliance is known

**Severity:** Medium (latent) · **Status:** OPEN
**Location:** `Robot::CheckActiveHub()`, moving to `FieldData::CheckActiveHub()` · **Verified:** by inspection

```cpp
frc::DriverStation::Alliance AllianceColor = frc::DriverStation::GetAlliance().value();
```

`GetAlliance()` returns a `std::optional` that is empty whenever no alliance is
set — including a bench robot with no driver station attached, which is the
normal state during bring-up. Calling `.value()` on an empty optional throws
`std::bad_optional_access`, and an uncaught throw in robot code terminates the
program.

Unreachable today because nothing calls `CheckActiveHub`. It becomes live the
moment the rebuild adds a caller, and the first caller is likely to be written
by someone who does not know this is here.

**Suggested fix:** use `GetAlliance().value_or(...)` with an explicit default, or
branch on the optional and return `"None"` when the alliance is unknown.

---

## B6 — `CheckActiveHub()` missing return path

**Severity:** Low (latent) · **Status:** FIXED in `f9cb0a8` · **Verified:** by compiler

> **Fix applied and compiler-confirmed.** After the `FieldData` extraction, the
> `-Wreturn-type` warnings list only `SwerveDrive.cpp:281` (B4). `Robot.cpp` has
> dropped off the list and `FieldData.cpp` never appears, which is the direct
> evidence the fix landed.

**Location:** `Robot::CheckActiveHub()` (pre-fix)

```
warning: control reaches end of non-void function [-Wreturn-type]
```

When the game-specific message is `"R"` or `"B"` but the match timer falls
outside every defined window, the function reaches its closing brace without
returning, producing an indeterminate `std::string`.

**Fix:** returns `"None"` explicitly on that path, as part of the `FieldData`
extraction.

---

## B7 — Simulation destroys shooter tuning CSV

**Severity:** Low · **Status:** FIXED in `e88b8d4` · **Verified:** reproduced, then confirmed gone

> **Resolved one task earlier than predicted.** The spec expected this to persist
> until `Turret` was deleted in Task 3. In fact it was gone after Task 1: once
> `Robot` stopped constructing `Turret`, `DisabledInit()` no longer called
> `SaveLaunchMapToFile()`. Every simulation run from Task 1 onward left the
> working tree clean with no manual revert. The file itself was deleted in Task 3.

**Location:** `Robot::DisabledInit()` → `Turret::SaveLaunchMapToFile()` (pre-fix)

Running simulation calls `DisabledInit()`, which saves the turret launch map to
`src/main/deploy/LaunchCalculator_Points.csv`. The map is empty in simulation,
so this writes an empty table over the file, discarding all 15 rows of shooter
tuning. Observed while capturing a simulation baseline on 2026-09-16 and
reverted with `git checkout --`.

**Workaround until fixed:** check `git status` after every simulation run and
revert the CSV if it shows modified.

**Resolution:** disappears when `Turret` is deleted, since nothing will write
the file. Until then the hazard is live for anyone running simulation.

---

## B8 — Dead PID code with inverted logic

**Severity:** Cosmetic · **Status:** OPEN
**Location:** `src/main/cpp/subsystems/SwerveDrive.cpp:355` · **Verified:** by inspection

```cpp
if ((!pidX.AtSetpoint() && !pidY.AtSetpoint()) | !hasRun)
```

Three problems in one line:

1. Bitwise `|` where logical `||` is meant.
2. `&&` where `||` is almost certainly meant — driving should continue while
   *either* axis is off its setpoint, not only while both are.
3. `hasRun` is set to `false` in `InitializePID()` and **never set to `true`
   anywhere**, so `!hasRun` is permanently true and the entire condition is
   inert.

Additionally, `SetReference()` and `InitializePID()` have no callers at all.

**Suggested action:** leave it. This is dead code that the model-first rebuild
will replace. Recorded so nobody mistakes it for a working reference
implementation.

---

## B10 — PathPlanner GUI config still draws a turret outline

**Severity:** Cosmetic · **Status:** OPEN
**Location:** `src/main/deploy/pathplanner/settings.json` · **Verified:** by inspection

The PathPlanner GUI robot-outline config still contains a shape named `Turret`:

```json
{"name":"Turret","type":"circle","data":{"center":{"x":-0.3,"y":0.3},"radius":0.15,...}}
```

This is a GUI visualization asset, not code, so it has no runtime effect. It
will draw a turret circle on a robot that no longer has one when someone opens
PathPlanner.

**Suggested action:** update it alongside the real robot geometry once the new
chassis is assembled, rather than editing it now against an unknown layout.

---

## B9 — `Drive()` suspected of producing NaN on zero input — DISMISSED

**Severity:** n/a · **Status:** DISMISSED (2026-09-16) · **Verified:** by test

`Drive()` calls `Eigen::Vector2d::normalized()` on a vector built from the
commanded speeds. When the driver releases the sticks both components are zero,
which looked like a 0/0 NaN that would propagate into every module setpoint.

A throwaway GoogleTest replicating the exact arithmetic showed `normalized()`
returns `(0, 0)` for a zero vector in this Eigen version, and the resulting
speeds are `(0, 0)`. **Not a bug.**

Recorded so the same suspicion is not re-investigated later.
