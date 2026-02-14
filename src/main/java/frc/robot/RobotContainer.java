// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.EventMarker;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.configuration.FieldConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.states.ClimbState;
import frc.robot.states.DrivetrainState;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Drivetrain;
import frc.robot.states.ShooterState;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.utilities.GeometryUtil;

public class RobotContainer {

    private final SendableChooser<Command> autoChooser;
    private final CommandXboxController DRIVER_CONTROLLER = new CommandXboxController(0);

    public final Vision VISION = new Vision();
    public final Drivetrain DRIVETRAIN = new Drivetrain(DRIVER_CONTROLLER, VISION);
    public final Shooter SHOOTER = new Shooter();
    public final Climb CLIMB = new Climb();

    public RobotContainer() {
        NamedCommands.registerCommand("Auto Climb", 
          Commands.sequence( 
            Commands.runOnce(() ->DRIVETRAIN.setTargetEngageLeftClimb(GeometryUtil::isRedAlliance)),
            Commands.runOnce(() ->DRIVETRAIN.setWantedState(DrivetrainState.CLIMB_ENGAGE))
            ));

        NamedCommands.registerCommand("Start Shooting", 
            Commands.runOnce(() -> SHOOTER.setWantedState(ShooterState.SHOOT))
            );

        NamedCommands.registerCommand("Stop Shooting", 
            Commands.runOnce(() -> SHOOTER.setWantedState(ShooterState.IDLE))
            );
        
        NamedCommands.registerCommand("Intake", 
            Commands.runOnce(() -> SHOOTER.setWantedState(ShooterState.INTAKE))
            );

        autoChooser = AutoBuilder.buildAutoChooser();

        // Another option that allows you to specify the default auto by its name
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

        SmartDashboard.putData("Auto Chooser", autoChooser);
        configureBindings();
    }

    private void configureBindings() {
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        RobotModeTriggers.disabled().onTrue(
            Commands.runOnce(() -> DRIVETRAIN.setWantedState(DrivetrainState.IDLE))
        );

        RobotModeTriggers.autonomous().onTrue(
          Commands.runOnce(() -> DRIVETRAIN.setWantedState(DrivetrainState.AUTO))
        );

        RobotModeTriggers.teleop().onTrue(
          Commands.sequence(
          Commands.runOnce(() -> DRIVETRAIN.setWantedState(DrivetrainState.OPEN_LOOP)),
          Commands.runOnce(() -> CLIMB.setWantedState(ClimbState.EXTEND)))
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

        DRIVER_CONTROLLER.back().onTrue(
          Commands.runOnce(() -> DRIVETRAIN.seedFieldCentric())
        );

        DRIVER_CONTROLLER.start().onTrue(
          Commands.runOnce(() -> {
            DRIVETRAIN.setWantedState(DrivetrainState.CLIMB_STAGE);
            DRIVETRAIN.setTargetStageLeftClimb(GeometryUtil::isRedAlliance);
         })
        ).onFalse(
          Commands.runOnce(() -> DRIVETRAIN.setWantedState(DrivetrainState.OPEN_LOOP))
        );

        new Trigger(DRIVETRAIN::isStagedForClimb).onTrue(
          Commands.sequence(
            Commands.runOnce(() -> DRIVETRAIN.setTargetEngageLeftClimb(GeometryUtil::isRedAlliance)),
            Commands.runOnce(() -> DRIVETRAIN.setWantedState(DrivetrainState.CLIMB_ENGAGE)),
            Commands.runOnce(() -> CLIMB.setWantedState(ClimbState.EXTEND))
             )
        );
        
        new Trigger(DRIVETRAIN::isReadyToClimb).onTrue(
          Commands.runOnce(() -> {
            SmartDashboard.putBoolean("isClimbed", true);
            DRIVETRAIN.setWantedState(DrivetrainState.OPEN_LOOP);
            CLIMB.setWantedState(ClimbState.RETRACT);
          })
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
      return autoChooser.getSelected();
    }
}
