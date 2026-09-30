// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include "subsystems/CommandSwerveDrivetrain.h"
#include "Telemetry.h"
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/InstantCommand.h>
#include <frc/smartdashboard/SmartDashboard.h>

//Autonomous includes below this
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/DriverStation.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>

class RobotContainer {
private:
    units::meters_per_second_t MaxSpeed = 1.0 * TunerConstants::kSpeedAt12Volts; // kSpeedAt12Volts desired top speed
    units::radians_per_second_t MaxAngularRate = 0.75_tps; // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    swerve::requests::FieldCentric drive = swerve::requests::FieldCentric{}
        .WithDeadband(MaxSpeed * 0.1).WithRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .WithDriveRequestType(swerve::DriveRequestType::OpenLoopVoltage); // Use open-loop control for drive motors
    swerve::requests::SwerveDriveBrake brake{};
    swerve::requests::PointWheelsAt point{};

    /* Note: This must be constructed before the drivetrain, otherwise we need to
     *       define a destructor to un-register the telemetry from the drivetrain */
    Telemetry logger{MaxSpeed};

    frc2::CommandXboxController joystick{0};
    
    double creepMult=1;
    //Auto stuff goes below this
    frc::SendableChooser<frc2::Command*> autoChooser;
    frc::Pose2d startPose=frc::Pose2d{0_m,0_m,0_deg};
    frc::Rotation2d rotation2d = drivetrain.GetPigeon2().GetRotation2d();
        // Locations for the swerve drive modules relative to the robot center.
    frc::Translation2d m_frontLeftLocation{8.5_in, 8.5_in};
    frc::Translation2d m_frontRightLocation{8.5_in, -8.5_in};
    frc::Translation2d m_backLeftLocation{-8.5_in, 8.5_in};
    frc::Translation2d m_backRightLocation{-8.5_in, -8.5_in};
        // Creating my kinematics object using the module locations.
    frc::SwerveDriveKinematics<4> kinematics{
    m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation,m_backRightLocation};
    swerve::requests::ApplyRobotSpeeds autoRobotSpeedsRequest = swerve::requests::ApplyRobotSpeeds();
    frc::SwerveDrivePoseEstimator<4>* poseEstimator;
public:
    subsystems::CommandSwerveDrivetrain drivetrain{TunerConstants::CreateDrivetrain()};

    RobotContainer();
    //Auto stuff goes below this
    frc2::Command* GetAutonomousCommand();

private:
    //Methods
    void ConfigureBindings();

    //Methods for Auto

    void ApplyStart();
    void ConfigureAutoBuilder();
    frc::SendableChooser<frc2::Command*> GetSelection();
};
