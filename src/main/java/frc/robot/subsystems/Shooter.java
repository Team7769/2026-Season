package frc.robot.subsystems;


import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;

import frc.robot.generated.TunerConstants;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.states.ShooterState;


public class Shooter extends SubsystemBase {
    private ShooterState _currentState = ShooterState.IDLE;
    private TalonFX _leftShooter1;
    private TalonFX _rightShooter1;
    private TalonFX _leftShooter2;
    private TalonFX _rightShooter2;
    private TalonFX _injector;
    private VelocityVoltage _shooterVelocity = new VelocityVoltage(0);
    private DigitalInput _leftPhotoEye;
    private DigitalInput _rightPhotoEye;

    private PIDController _hoodPID;
    private boolean _isReadyToShoot = false;
    private double _shooterPosition;
    private double _shooterTarget;
    private double _hoodPosition;
    private double _hoodTarget;
    private Vision VISION = new Vision();

    private final double[] kDistanceIDs = {1.77, 2, 2.5, 3, 3.5, 4};
    private final double[] kHoodAngles = {4.5, 5.1, 5.55, 5.85, 6.2, 6.35};
    private final double[] kShooterSpeeds = {67, 67, 67, 67, 67, 67};

    public Shooter() {
        configShooter();
        configInjector();
        configHood();
    }

    private void configShooter(){
        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
        _leftShooter1 = new TalonFX(15);
        _leftShooter2 = new TalonFX(16);//follow
        _rightShooter1 = new TalonFX(17);
        _rightShooter2 = new TalonFX(18);//follow
        _leftPhotoEye = new DigitalInput(0);
        _rightPhotoEye = new DigitalInput(1);

        var slot0 = shooterConfig.Slot0;

        slot0.kV = 0.2;
        slot0.kP = 0.2;
        slot0.kI = 0.0;
        slot0.kD = 0.0;

        _leftShooter1.setNeutralMode(NeutralModeValue.Coast);
        _leftShooter2.setNeutralMode(NeutralModeValue.Coast);
        _rightShooter1.setNeutralMode(NeutralModeValue.Coast);
        _rightShooter2.setNeutralMode(NeutralModeValue.Coast);

        _leftShooter1.getConfigurator().apply(shooterConfig);
        _leftShooter2.getConfigurator().apply(shooterConfig);
        _rightShooter1.getConfigurator().apply(shooterConfig);
        _rightShooter2.getConfigurator().apply(shooterConfig);

        _leftShooter2.setControl(new Follower(_leftShooter1, false));
        _rightShooter2.setControl(new Follower(_rightShooter1, false));

    }

    private void configInjector(){
        _injector = new TalonFX(19);
        _injector.setNeutralMode(NeutralModeValue.Brake);
    }

    private void configHood(){
        _hoodPID = new PIDController(0.1,0,0.01);

    }

    private void prepShooter() {
        _leftShooter1.setControl(_shooterVelocity.withVelocity(30));
        _rightShooter1.setControl(_shooterVelocity.withVelocity(30));
        injectorOff();
    }

    private void shoot(double shot) {
        _leftShooter1.setControl(_shooterVelocity.withVelocity(shot));
        _rightShooter1.setControl(_shooterVelocity.withVelocity(shot));
        injectorOn();
    }

    private void stop() {
        _leftShooter1.set(0);
        _rightShooter1.set(0);
        injectorOff();
    }

    private void setIdle() {
        _leftShooter1.setControl(_shooterVelocity.withVelocity(10));
        _rightShooter1.setControl(_shooterVelocity.withVelocity(10));
        injectorOff();
    }

    private void injectorOn() {
        _injector.set(0.3);
    }

        private void injectorOff() {
        _injector.set(0);
    }

    public void setWantedState(ShooterState wantedState) {
        if (wantedState != _currentState) {
            _currentState = wantedState;
        }
    }

    @Override
    public void periodic() {
        handleCurrentState();
    }

    private void handleCurrentState() {
        switch (_currentState) {
            case IDLE:
                setIdle();
                break;
            case SHOOT:
                shoot(_shooterPosition);
                break;
            case PREPSHOOTER:
                prepShooter();
                break;
            case STOP:
                stop();
                break;
            default:
                setIdle();
                break;
        }
    }
}
