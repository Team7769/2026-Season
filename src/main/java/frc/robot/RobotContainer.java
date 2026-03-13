// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Consumer;

import com.ctre.phoenix6.configs.LEDConfigs;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.configuration.FieldConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.states.ClimbState;
import frc.robot.states.ClimbType;
import frc.robot.states.DrivetrainState;
import frc.robot.subsystems.Climb;
import frc.robot.states.ClimbType;
import frc.robot.states.HopperState;
import frc.robot.states.IntakeState;
import frc.robot.states.LedState;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.GameManager;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Ledinator;
import frc.robot.subsystems.Shooter;
import frc.robot.states.ShooterState;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Kitbot.KitbotIntake;
import frc.robot.subsystems.Kitbot.KitbotShooter;
import frc.robot.utilities.GeometryUtil;

public class RobotContainer {
  // Set this to false to setup as Kitbot
  private final boolean _isComp = true;

  private final SendableChooser<Command> autoChooser;
  private final SendableChooser<ClimbType> climbChooser;
  private final CommandXboxController DRIVER_CONTROLLER = new CommandXboxController(0);
  private final CommandXboxController OPERATOR_CONTROLLER = new CommandXboxController(1);

  public final Climb CLIMB = new Climb();
  public final Vision VISION = new Vision();
  public final Drivetrain DRIVETRAIN = new Drivetrain(DRIVER_CONTROLLER, VISION, _isComp);
  public final KitbotShooter KITBOT_SHOOTER = _isComp ? null : new KitbotShooter();
  public final KitbotIntake KITBOT_INTAKE = _isComp ? null : new KitbotIntake();
  public final Hopper HOPPER = _isComp ? new Hopper() : null;
  public final Shooter SHOOTER = _isComp ? new Shooter(DRIVETRAIN) : null;
  public final Ledinator LEDINATOR = _isComp ? new Ledinator() : null;
  //private final GameManager GAME_MANAGER = new GameManager(DRIVER_CONTROLLER);


  public RobotContainer() {

    registerNamedCommands();

    autoChooser = AutoBuilder.buildAutoChooser();
    climbChooser = new SendableChooser<ClimbType>();
    climbChooser.addOption("Front Climb", ClimbType.FRONT);
    climbChooser.addOption("Side Climb", ClimbType.SIDE);
    climbChooser.setDefaultOption("Front Climb", ClimbType.FRONT);
    climbChooser.onChange(value -> {
      DRIVETRAIN.setWantedClimb(value);
    });

    
    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Climb Chooser", climbChooser);
    SmartDashboard.putData("Auto Chooser", autoChooser);
    configureBindings();
  }

  private void configureBindings() {
    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    RobotModeTriggers.disabled().onTrue(
        Commands.runOnce(() -> {
          DRIVETRAIN.setWantedState(DrivetrainState.IDLE);
          LEDINATOR.setWantedState(LedState.CREW);
        }));

    RobotModeTriggers.autonomous().onTrue(Commands.runOnce(() -> DRIVETRAIN.setWantedState(DrivetrainState.AUTO)));

    RobotModeTriggers.teleop().onTrue(Commands.runOnce(() ->  DRIVETRAIN.setWantedState(DrivetrainState.OPEN_LOOP)));

    DRIVER_CONTROLLER.back().onTrue(
        Commands.runOnce(() -> DRIVETRAIN.seedFieldCentric()));

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

    // Aim and Spin Up for Shot
    DRIVER_CONTROLLER.rightBumper().onTrue(
        Commands.runOnce(() -> {
          SHOOTER.setWantedState(ShooterState.PREPSHOOTER);
          DRIVETRAIN.setWantedState(DrivetrainState.AIM);
        })).onFalse(
            Commands.runOnce(() -> {
              SHOOTER.setWantedState(ShooterState.IDLE);
              HOPPER.setWantedState(HopperState.STOW);
              DRIVETRAIN.setWantedState(DrivetrainState.OPEN_LOOP);
            }));

    // Shoot
    DRIVER_CONTROLLER.rightTrigger().onTrue(
        Commands.runOnce(() -> {
          SHOOTER.setWantedState(ShooterState.SHOOT);
          HOPPER.setWantedState(HopperState.INJECTING_HOPPER_OUT);
        })).onFalse(
            Commands.runOnce(() -> {
              SHOOTER.setWantedState(ShooterState.SHOOT);
              HOPPER.setWantedState(HopperState.INJECTING);
            }));

    // Intake out
    DRIVER_CONTROLLER.leftBumper().onTrue(
        Commands.runOnce(() -> {
          HOPPER.setWantedState(HopperState.FLOOR_INTAKE);
        }));

    // Intake in
    DRIVER_CONTROLLER.leftTrigger().onTrue(
        Commands.runOnce(() -> {
          HOPPER.setWantedState(HopperState.STOW);
        }));

      OPERATOR_CONTROLLER.b().onTrue(
        Commands.runOnce(() -> {
          SHOOTER.setWantedState(ShooterState.EMERGENCY);
        })).onFalse(
          Commands.runOnce( () -> {
            SHOOTER.setWantedState(ShooterState.IDLE);
          })
        );

    // Brake mode
    DRIVER_CONTROLLER.y().onTrue(
        Commands.runOnce(() -> DRIVETRAIN.setWantedState(DrivetrainState.IDLE))).onFalse(
            Commands.runOnce(() -> DRIVETRAIN.setWantedState(DrivetrainState.OPEN_LOOP)));

    // Reseed heading
    DRIVER_CONTROLLER.back().onTrue(
        Commands.runOnce(() -> DRIVETRAIN.seedFieldCentric()));
        
    DRIVER_CONTROLLER.povDown().onTrue(
        Commands.runOnce(() -> CLIMB.setWantedState(ClimbState.RETRACT)));

    DRIVER_CONTROLLER.povUp().onTrue(
        Commands.runOnce(() -> CLIMB.setWantedState(ClimbState.EXTEND)));

    OPERATOR_CONTROLLER.povDown().onTrue(
        Commands.runOnce(() -> CLIMB.setWantedState(ClimbState.RETRACT)));

    OPERATOR_CONTROLLER.povUp().onTrue(
        Commands.runOnce(() -> CLIMB.setWantedState(ClimbState.EXTEND)));

    OPERATOR_CONTROLLER.a().onTrue(
      Commands.runOnce(() -> HOPPER.setWantedState(HopperState.EMERGENCY))
    ).onFalse(Commands.runOnce(() -> HOPPER.setWantedState(HopperState.IDLE)));

    OPERATOR_CONTROLLER.x().onTrue(
      Commands.runOnce(() -> HOPPER.setWantedState(HopperState.JAM))
    ).onFalse(Commands.runOnce(() -> HOPPER.setWantedState(HopperState.IDLE)));

      OPERATOR_CONTROLLER.leftTrigger().onTrue(
        Commands.runOnce(() -> {
          LEDINATOR.setWantedState(LedState.WARNING);
        })).onFalse(
            Commands.runOnce(() -> {
              LEDINATOR.setWantedState(LedState.CREW);
            }));

    OPERATOR_CONTROLLER.rightTrigger().onTrue(
        Commands.runOnce(() -> {
          LEDINATOR.setWantedState(LedState.ACTIVE);
          DRIVER_CONTROLLER.setRumble(RumbleType.kBothRumble, 100);
        })).onFalse(
            Commands.runOnce(() -> {
              LEDINATOR.setWantedState(LedState.CREW);
              DRIVER_CONTROLLER.setRumble(RumbleType.kBothRumble, 0);
            }));;
        
        
    OPERATOR_CONTROLLER.rightBumper().onTrue(
            Commands.runOnce(() -> {
              LEDINATOR.setWantedState(LedState.CREW);
            }));

    // Climb sequence. Release start to abort.
    DRIVER_CONTROLLER.start().onTrue(
        beginClimbSequenceCommand()
    )
    .onFalse(Commands.runOnce(() -> DRIVETRAIN.setWantedState(DrivetrainState.OPEN_LOOP)));

    // Trigger during climb sequence to move from stage -> engage
    new Trigger(DRIVETRAIN::isStagedForClimb).onTrue(
        Commands.sequence(
            Commands.runOnce(() -> DRIVETRAIN.setTargetEngageLeftClimb(GeometryUtil::isRedAlliance)),
            Commands.runOnce(() -> DRIVETRAIN.setWantedState(DrivetrainState.CLIMB_ENGAGE))));
           
        DRIVER_CONTROLLER.x().onTrue(
        Commands.runOnce(() -> {
          DRIVETRAIN.setTargetLeftTrench(GeometryUtil::isRedAlliance);
          DRIVETRAIN.setWantedState(DrivetrainState.TRENCH_LEFT);
        })).onFalse(
          Commands.runOnce(() -> DRIVETRAIN.setWantedState(DrivetrainState.OPEN_LOOP)));    


    // Trigger during climb sequence to activate climb if engaged
    new Trigger(DRIVETRAIN::isReadyToClimb).onTrue(
        Commands.runOnce(() -> {
          SmartDashboard.putBoolean("isClimbed", true);
          DRIVETRAIN.setWantedState(DrivetrainState.OPEN_LOOP);
          CLIMB.setWantedState(ClimbState.ENGAGE);
        }));

  }

  private void registerNamedCommands() {
    if (_isComp) {
      registerCompNamedCommands();
    } else {
      registerKitbotNamedCommands();
    }
  }

  private void registerKitbotNamedCommands() {

  }

  private void registerCompNamedCommands() {
    NamedCommands.registerCommand("IntakeOut",
        Commands.runOnce(() -> HOPPER.setWantedState(HopperState.FLOOR_INTAKE)));

    NamedCommands.registerCommand("IntakeIn",
        Commands.runOnce(() -> HOPPER.setWantedState(HopperState.STOW)));

    NamedCommands.registerCommand("Injecting",
        Commands.runOnce(() -> HOPPER.setWantedState(HopperState.INJECTING)));

    NamedCommands.registerCommand("Start Shooting",
        Commands.sequence(
            Commands.waitUntil(SHOOTER::shooterReady),
            Commands.runOnce(() -> {
              SHOOTER.setWantedState(ShooterState.SHOOT);
              // DRIVETRAIN.setWantedState(DrivetrainState.AIM);
            })));

    NamedCommands.registerCommand("Idle",
        Commands.runOnce(() -> {
          SHOOTER.setWantedState(ShooterState.IDLE);
          HOPPER.setWantedState(HopperState.IDLE);
          DRIVETRAIN.setWantedState(DrivetrainState.AUTO);
        }));

    NamedCommands.registerCommand("PrepShooter",
        Commands.runOnce(() -> {
          SHOOTER.setWantedState(ShooterState.PREPSHOOTER);
          DRIVETRAIN.setWantedState(DrivetrainState.AIM);
        }));
        
    NamedCommands.registerCommand("Auto Climb", beginClimbSequenceCommand());
  }

  private Command beginClimbSequenceCommand() {
    return Commands.sequence(
        Commands.runOnce(() -> {
          CLIMB.setWantedState(ClimbState.EXTEND);
          DRIVETRAIN.setWantedState(DrivetrainState.CLIMB_STAGE);
        }),
        Commands.runOnce(() -> DRIVETRAIN.setTargetStageLeftClimb(GeometryUtil::isRedAlliance)));
  }

}
