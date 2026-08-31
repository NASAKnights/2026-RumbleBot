# REBUILT 2026 - FRC TEAM 122

This is the main robot code base for FRC 122 - NASA Knights for the 2026 season.

**Planning and work tracking:** [Road to Competition board](https://github.com/orgs/NASAKnights/projects/3) — every issue lands there. Board conventions and the subsystem architecture are in [docs/PROJECT_BOARD.md](docs/PROJECT_BOARD.md).

## Changes of Note

- CANBus for 2026 is now its own structure. It still requires the CANBus String name
- Quick changes were made to the swerve module and swerve drive class to account for the CANBus changes
    - Will need to go back and adjust as needed
- Configs.hpp has been separated into different configs for ctre
- NavX libraries have been temporarily removed for sake of build, will assess if needed for actual robot