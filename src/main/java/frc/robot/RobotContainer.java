// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.states.DrivetrainState;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.states.ShooterState;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Kitbot.KitbotIntake;
import frc.robot.subsystems.Kitbot.KitbotShooter;
import frc.robot.subsystems.Kitbot.Shooter;
import frc.robot.utilities.GeometryUtil;

public class RobotContainer {
  // Set this to false to setup as Kitbot
  private final boolean _isComp = false;

  private final SendableChooser<Command> autoChooser;
  private final CommandXboxController DRIVER_CONTROLLER = new CommandXboxController(0);

  public final Vision VISION = new Vision();
  public final Drivetrain DRIVETRAIN = new Drivetrain(DRIVER_CONTROLLER, VISION, _isComp);
  public final KitbotShooter KITBOT_SHOOTER = _isComp ? null : new KitbotShooter();
  public final KitbotIntake KITBOT_INTAKE = _isComp ? null : new KitbotIntake();
  public final Hopper HOPPER = _isComp ? new Hopper() : null;
  public final Shooter SHOOTER = _isComp ? new Shooter() : null;

  public RobotContainer() {

    registerNamedCommands();

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
        Commands.runOnce(() -> DRIVETRAIN.setWantedState(DrivetrainState.IDLE)));

    RobotModeTriggers.autonomous().onTrue(
        Commands.runOnce(() -> DRIVETRAIN.setWantedState(DrivetrainState.AUTO)));

    RobotModeTriggers.teleop().onTrue(
        Commands.runOnce(() -> DRIVETRAIN.setWantedState(DrivetrainState.OPEN_LOOP)));

    DRIVER_CONTROLLER.leftTrigger().whileTrue(
        Commands.runOnce(() -> DRIVETRAIN.setWantedState(DrivetrainState.AIM))).onFalse(
            Commands.runOnce(() -> DRIVETRAIN.setWantedState(DrivetrainState.OPEN_LOOP)));

    DRIVER_CONTROLLER.y().onTrue(
        Commands.runOnce(() -> DRIVETRAIN.setTargetHub(GeometryUtil::isRedAlliance)));

    DRIVER_CONTROLLER.a().onTrue(
        Commands.runOnce(() -> DRIVETRAIN.setTargetDepot(GeometryUtil::isRedAlliance)));

    DRIVER_CONTROLLER.x().onTrue(
        Commands.runOnce(() -> DRIVETRAIN.setTargetZoneA(GeometryUtil::isRedAlliance)));

        DRIVER_CONTROLLER.rightBumper().onTrue(
          Commands.runOnce(() -> SHOOTER.setWantedState(ShooterState.PREPSHOOTER))
        ).onFalse(
          Commands.runOnce(() -> SHOOTER.setWantedState(ShooterState.IDLE))
        );

        DRIVER_CONTROLLER.rightTrigger().onTrue(
          Commands.runOnce(() -> SHOOTER.setWantedState(ShooterState.SHOOT))
        ).onFalse(
          Commands.runOnce(() -> SHOOTER.setWantedState(ShooterState.IDLE))
        );
    DRIVER_CONTROLLER.b().onTrue(
        Commands.runOnce(() -> DRIVETRAIN.setTargetZoneB(GeometryUtil::isRedAlliance)));
    DRIVER_CONTROLLER.back().onTrue(
        Commands.runOnce(() -> DRIVETRAIN.seedFieldCentric()));

    DRIVER_CONTROLLER.start().onTrue(
        Commands.runOnce(() -> {
          DRIVETRAIN.setWantedState(DrivetrainState.CLIMB_STAGE);
          DRIVETRAIN.setTargetStageLeftClimb(GeometryUtil::isRedAlliance);
        })).onFalse(
            Commands.runOnce(() -> DRIVETRAIN.setWantedState(DrivetrainState.OPEN_LOOP)));

    new Trigger(DRIVETRAIN::isStagedForClimb).onTrue(
        Commands.sequence(
            Commands.runOnce(() -> DRIVETRAIN.setTargetEngageLeftClimb(GeometryUtil::isRedAlliance)),
            Commands.runOnce(() -> DRIVETRAIN.setWantedState(DrivetrainState.CLIMB_ENGAGE))));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // DRIVER_CONTROLLER.back().and(DRIVER_CONTROLLER.y()).whileTrue(DRIVETRAIN.sysIdDynamic(Direction.kForward));
        // DRIVER_CONTROLLER.back().and(DRIVER_CONTROLLER.x()).whileTrue(DRIVETRAIN.sysIdDynamic(Direction.kReverse));
        // DRIVER_CONTROLLER.start().and(DRIVER_CONTROLLER.y()).whileTrue(DRIVETRAIN.sysIdQuasistatic(Direction.kForward));
        // DRIVER_CONTROLLER.start().and(DRIVER_CONTROLLER.x()).whileTrue(DRIVETRAIN.sysIdQuasistatic(Direction.kReverse));
    new Trigger(DRIVETRAIN::isReadyToClimb).onTrue(
        Commands.runOnce(() -> {
          SmartDashboard.putBoolean("isClimbed", true);
          DRIVETRAIN.setWantedState(DrivetrainState.OPEN_LOOP);
        }));

    if (_isComp) {
      registerCompBindings();
    } else {
      registerKitbotBindings();
    }
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

  private void registerKitbotBindings() {

  }

  private void registerCompBindings() {
    // Register triggers/bindings for comp bot here
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand("Auto Climb",
        Commands.sequence(
            Commands.runOnce(() -> DRIVETRAIN.setTargetEngageLeftClimb(GeometryUtil::isRedAlliance)),
            Commands.runOnce(() -> DRIVETRAIN.setWantedState(DrivetrainState.CLIMB_ENGAGE))));

    if (_isComp) {
      registerCompNamedCommands();
    } else {
      registerKitbotNamedCommands();
    }
  }

  private void registerKitbotNamedCommands() {
    
  }

  private void registerCompNamedCommands() {

  }
}
