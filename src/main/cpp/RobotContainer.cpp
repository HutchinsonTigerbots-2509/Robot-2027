// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include <frc2/command/button/RobotModeTriggers.h>

RobotContainer::RobotContainer()
{
    ConfigureBindings();
    // ConfigureAutoBuilder();
    // GetAutonomousCommand();
}

void RobotContainer::ConfigureBindings()
{
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.SetDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.ApplyRequest([this]() -> auto&& {
            return drive.WithVelocityX(-joystick.GetLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                .WithVelocityY(-joystick.GetLeftX() * MaxSpeed) // Drive left with negative X (left)
                .WithRotationalRate(-joystick.GetRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left)
        })
    );

    joystick.LeftBumper().WhileTrue(frc2::InstantCommand([this]{creepMult=.5;}).ToPtr()).OnFalse(frc2::InstantCommand([this]{creepMult=1;}).ToPtr());
    joystick.POVUp().OnTrue(frc2::InstantCommand([this]{drivetrain.GetPigeon2().SetYaw(0.0_deg);}).ToPtr());

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    frc2::RobotModeTriggers::Disabled().WhileTrue(
        drivetrain.ApplyRequest([] {
            return swerve::requests::Idle{};
        }).IgnoringDisable(true)
    );
    // reset the field-centric heading on left bumper press
    joystick.LeftBumper().OnTrue(drivetrain.RunOnce([this] { drivetrain.SeedFieldCentric(); }));

    drivetrain.RegisterTelemetry([this](auto const &state) { logger.Telemeterize(state); });
}
frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
    // Simple drive forward auton
    return frc2::cmd::Sequence(
        // Reset our field centric heading to match the robot
        // facing away from our alliance station wall (0 deg).
        drivetrain.RunOnce([this] { drivetrain.SeedFieldCentric(frc::Rotation2d{0_deg}); }),
        // Then slowly drive forward (away from us) for 5 seconds.
        drivetrain.ApplyRequest([this]() -> auto&& {
            return drive.WithVelocityX(0.5_mps)
                .WithVelocityY(0_mps)
                .WithRotationalRate(0_tps);
        })
        .WithTimeout(5_s),
        // Finally idle for the rest of auton
        drivetrain.ApplyRequest([] { return swerve::requests::Idle{}; })
    );
}
// //From here down will be auto code

// void RobotContainer::ConfigureAutoBuilder(){
//     //poseEstimator= new frc::SwerveDrivePoseEstimator<4>{kinematics,frc::Rotation2d{},{drivetrain.GetModule(0).GetPosition(false),drivetrain.GetModule(1).GetPosition(false),drivetrain.GetModule(2).GetPosition(false),drivetrain.GetModule(3).GetPosition(false)},frc::Pose2d{},{0,0,0},{0,0,0}};
//    pathplanner::RobotConfig config = pathplanner::RobotConfig::fromGUISettings();
//     // Configure the AutoBuilder last
//     pathplanner::AutoBuilder::configure(
//         [this](){ return drivetrain.GetState().Pose;}, // Robot pose supplier   Test #1 Replacing drivetrain.GetState().Pose; with PoseEstimator->GetEstimatedPostion()
//         [this](frc::Pose2d pose){ drivetrain.ResetPose(pose);}, // Method to reset odometry (will be called if your auto has a starting pose)
//         [this](){ return drivetrain.GetState().Speeds; }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
//         [this](frc::ChassisSpeeds speeds, pathplanner::DriveFeedforwards feedforwards){ drivetrain.SetControl(autoRobotSpeedsRequest.WithSpeeds(frc::ChassisSpeeds::Discretize(speeds.vx,speeds.vy,speeds.omega, 0.020_s))
//             .WithWheelForceFeedforwardsX(feedforwards.robotRelativeForcesX)
//             .WithWheelForceFeedforwardsY(feedforwards.robotRelativeForcesY));}, 
        
//         // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
//         std::make_shared<pathplanner::PPHolonomicDriveController>( // PPHolonomicController is the built in path following controller for holonomic drive trains
//             pathplanner::PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
//             pathplanner::PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
//         ),
//         config, // The robot configuration
//         []() {
//             // Boolean supplier that controls when the path will be mirrored for the red alliance
//             // This will flip the path being followed to the red side of the field.
//             // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

//             auto alliance = frc::DriverStation::GetAlliance();
//             if (alliance) {
//                 return alliance.value() == frc::DriverStation::Alliance::kRed;
//             }
//             return false;
//         },
//         &drivetrain // Reference to this subsystem to set requirements
//     );
// }
// //frc2::Command* RobotContainer::GetAutonomousCommand()
// // {
// //     return autoChooser.GetSelected();
// // }
