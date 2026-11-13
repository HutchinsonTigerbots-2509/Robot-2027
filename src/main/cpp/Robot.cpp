// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>

Robot::Robot() {}

void Robot::RobotPeriodic() {
    m_timeAndJoystickReplay.Update();
    frc2::CommandScheduler::GetInstance().Run();
    
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
    SignalLogger::Stop;
    m_autonomousCommand = m_AutoContainer.GetAutonomousCommand();

    if (m_autonomousCommand) {
        frc2::CommandScheduler::GetInstance().Schedule(m_autonomousCommand.value());
    }
}

void Robot::AutonomousPeriodic() {
    SignalLogger::Stop();
}

void Robot::AutonomousExit() {
    SignalLogger::Stop();
}

void Robot::TeleopInit() {
    SignalLogger::Stop();
    if (m_autonomousCommand) {
        frc2::CommandScheduler::GetInstance().Cancel(m_autonomousCommand.value());
    }
    RobotContainer m_Container;
    
}

void Robot::TeleopPeriodic() {
    SignalLogger::Stop();
}

void Robot::TeleopExit() {}

void Robot::TestInit() {
    frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() {
    return frc::StartRobot<Robot>();
}
#endif
