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
import frc.robot.states.ShooterState;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.utilities.GeometryUtil;

public class RobotContainer {

    private final CommandXboxController DRIVER_CONTROLLER = new CommandXboxController(0);

    public final Vision VISION = new Vision();
    public final Drivetrain DRIVETRAIN = new Drivetrain(DRIVER_CONTROLLER, VISION);
    public final Shooter SHOOTER = new Shooter();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
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

        DRIVER_CONTROLLER.y().onTrue(
          Commands.runOnce(() -> DRIVETRAIN.setTargetHub(GeometryUtil::isRedAlliance))
        );
        
        DRIVER_CONTROLLER.a().onTrue(
          Commands.runOnce(() -> DRIVETRAIN.setTargetDepot(GeometryUtil::isRedAlliance))
        );

        DRIVER_CONTROLLER.x().onTrue(
          Commands.runOnce(() -> DRIVETRAIN.setTargetZoneA(GeometryUtil::isRedAlliance))
        );
        
        DRIVER_CONTROLLER.b().onTrue(
          Commands.runOnce(() -> DRIVETRAIN.setTargetZoneB(GeometryUtil::isRedAlliance))
        );

        DRIVER_CONTROLLER.rightBumper().onTrue(
          Commands.runOnce(() -> SHOOTER.setWantedState(ShooterState.SHOOT))
        ).onFalse(
          Commands.runOnce(() -> SHOOTER.setWantedState(ShooterState.INTAKE))
        );

        DRIVER_CONTROLLER.leftBumper().onTrue(
          Commands.runOnce(() -> SHOOTER.setWantedState(ShooterState.IDLE))
        );

        DRIVER_CONTROLLER.rightTrigger().onTrue(
          Commands.runOnce(() -> SHOOTER.play())
        ).onFalse(
          Commands.runOnce(() -> SHOOTER.pause())
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
