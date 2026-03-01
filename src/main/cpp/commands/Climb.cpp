#include "commands/Climb.h"

Climb::Climb(Climber* climber, bool extend) : m_climber(climber), m_extend(extend) {
  AddRequirements(m_climber);
}

// Called when the command is initially scheduled.
void Climb::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void Climb::Execute() {
    if (m_extend) {
        m_climber->extend();
    } else {
        m_climber->retract();
    }
}

// Called once the command ends or is interrupted.
void Climb::End(bool interrupted) {
    m_climber->stopMotor();
}

// Returns true when the command should end.
bool Climb::IsFinished() {
  return false;
}
