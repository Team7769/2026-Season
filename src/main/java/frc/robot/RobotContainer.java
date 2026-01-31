// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.configuration.FieldConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.states.DrivetrainState;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.utilities.GeometryUtil;

public class RobotContainer {

    private final CommandXboxController DRIVER_CONTROLLER = new CommandXboxController(0);

    public final Vision VISION = new Vision();
    public final Drivetrain DRIVETRAIN = new Drivetrain(DRIVER_CONTROLLER, VISION);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        // DRIVETRAIN.setDefaultCommand(
        //     // Drivetrain will execute this command periodically
        //     DRIVETRAIN.applyRequest(() ->
        //         DRIVE.withVelocityX(-DRIVER_CONTROLLER.getLeftY() * _maxSpeed) // Drive forward with negative Y (forward)
        //             .withVelocityY(-DRIVER_CONTROLLER.getLeftX() * _maxSpeed) // Drive left with negative X (left)
        //             .withRotationalRate(-DRIVER_CONTROLLER.getRightX() * _maxAngularRate) // Drive counterclockwise with negative X (left)
        //     )
        // );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        RobotModeTriggers.disabled().onTrue(
            Commands.runOnce(() -> DRIVETRAIN.setWantedState(DrivetrainState.IDLE))
        );

        RobotModeTriggers.teleop().onTrue(
          Commands.runOnce(() -> DRIVETRAIN.setWantedState(DrivetrainState.OPEN_LOOP))
        );

        DRIVER_CONTROLLER.leftTrigger().whileTrue(
          Commands.runOnce(() -> DRIVETRAIN.setWantedState(DrivetrainState.AIM))
        ).onFalse(
          Commands.runOnce(() -> DRIVETRAIN.setWantedState(DrivetrainState.OPEN_LOOP))
        );

        DRIVER_CONTROLLER.a().onTrue(
          Commands.runOnce(() -> DRIVETRAIN.setTargetHub(GeometryUtil::isRedAlliance))
        );
        
        DRIVER_CONTROLLER.b().onTrue(
          Commands.runOnce(() -> DRIVETRAIN.setTargetDepot(GeometryUtil::isRedAlliance))
        );

        DRIVER_CONTROLLER.x().onTrue(
          Commands.runOnce(() -> DRIVETRAIN.setTargetZoneA(GeometryUtil::isRedAlliance))
        );
        
        DRIVER_CONTROLLER.y().onTrue(
          Commands.runOnce(() -> DRIVETRAIN.setTargetZoneB(GeometryUtil::isRedAlliance))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // DRIVER_CONTROLLER.back().and(DRIVER_CONTROLLER.y()).whileTrue(DRIVETRAIN.sysIdDynamic(Direction.kForward));
        // DRIVER_CONTROLLER.back().and(DRIVER_CONTROLLER.x()).whileTrue(DRIVETRAIN.sysIdDynamic(Direction.kReverse));
        // DRIVER_CONTROLLER.start().and(DRIVER_CONTROLLER.y()).whileTrue(DRIVETRAIN.sysIdQuasistatic(Direction.kForward));
        // DRIVER_CONTROLLER.start().and(DRIVER_CONTROLLER.x()).whileTrue(DRIVETRAIN.sysIdQuasistatic(Direction.kReverse));
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        //final var idle = new SwerveRequest.Idle();
        // return Commands.sequence(
        //     // Reset our field centric heading to match the robot
        //     // facing away from our alliance station wall (0 deg).
        //     DRIVETRAIN.runOnce(() -> DRIVETRAIN.seedFieldCentric(Rotation2d.kZero)),
        //     // Then slowly drive forward (away from us) for 5 seconds.
        //     DRIVETRAIN.applyRequest(() ->
        //         DRIVE.withVelocityX(0.5)
        //             .withVelocityY(0)
        //             .withRotationalRate(0)
        //     )
        //     .withTimeout(5.0),
        //     // Finally idle for the rest of auton
        //     DRIVETRAIN.applyRequest(() -> idle)
        // );

        return new InstantCommand();
    }
}
