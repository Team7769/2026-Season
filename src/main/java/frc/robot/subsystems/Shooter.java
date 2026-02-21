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
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.fasterxml.jackson.annotation.Nulls;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import frc.robot.states.ShooterState;


public class Shooter extends SubsystemBase {
    private ShooterState _currentState = ShooterState.IDLE;
    private TalonFX _leftShooter1;
    private TalonFX _rightShooter1;
    private TalonFX _leftShooter2;
    private TalonFX _rightShooter2;
    private VelocityVoltage _shooterVelocity = new VelocityVoltage(0);
    private final VoltageOut SHOOTER_VOLTAGE = new VoltageOut(6);
    private DigitalInput _leftPhotoEye;
    private DigitalInput _rightPhotoEye;
    private Servo _leftHood;
    private Servo _rightHood;

    private SimpleMotorFeedforward _ff = new SimpleMotorFeedforward(0, 0);
    private boolean _isReadyToShoot = false;
    private double _shooterPosition;
    private double _shooterTarget;
    private double _shooterError = 0.1;
    private double _hoodPosition;
    private double _hoodTarget;
    private Vision VISION = new Vision();

    private final double[] kDistanceIDs = {1.77, 2, 2.5, 3, 3.5, 4};
    private final double[] kHoodAngles = {4.5, 5.1, 5.55, 5.85, 6.2, 6.35};
    private final double[] kShooterSpeeds = {67, 67, 67, 67, 67, 67};

    public Shooter() {
        configShooter();
        configHood();
    }

    private void configShooter(){
        _leftShooter1 = new TalonFX(15);
        _leftShooter2 = new TalonFX(16);//follow
        _rightShooter1 = new TalonFX(17);
        _rightShooter2 = new TalonFX(18);//follow

        TalonFXConfiguration rightShooterConfig = new TalonFXConfiguration();
        var rightSlot0 = rightShooterConfig.Slot0;

        rightSlot0.kV = 0.2;
        rightSlot0.kP = 0.2;
        rightSlot0.kI = 0.0;
        rightSlot0.kD = 0.0;

        TalonFXConfiguration leftShooterConfig = new TalonFXConfiguration();
        leftShooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        var leftSlot0 = rightShooterConfig.Slot0;

        leftSlot0.kV = 0.2;
        leftSlot0.kP = 0.2;
        leftSlot0.kI = 0.0;
        leftSlot0.kD = 0.0;

        _leftShooter1.getConfigurator().apply(leftShooterConfig);
        _leftShooter2.getConfigurator().apply(leftShooterConfig);
        _rightShooter1.getConfigurator().apply(rightShooterConfig);
        _rightShooter2.getConfigurator().apply(rightShooterConfig);

        _leftShooter1.setNeutralMode(NeutralModeValue.Coast);
        _leftShooter2.setNeutralMode(NeutralModeValue.Coast);
        _rightShooter1.setNeutralMode(NeutralModeValue.Coast);
        _rightShooter2.setNeutralMode(NeutralModeValue.Coast);

        _leftShooter2.setControl(new Follower(_leftShooter1.getDeviceID(), MotorAlignmentValue.Aligned));
        _rightShooter2.setControl(new Follower(_rightShooter1.getDeviceID(), MotorAlignmentValue.Aligned));

    }

    private void configHood(){
        _leftHood = new Servo(0);
        _rightHood = new Servo(1);
    }

    private void prepShooter() {
        _leftShooter1.setControl(SHOOTER_VOLTAGE);
        _rightShooter1.setControl(SHOOTER_VOLTAGE);
        hoodUp();
    }

    private void shoot(double shot) {
        _leftShooter1.setControl(SHOOTER_VOLTAGE);
        _rightShooter1.setControl(SHOOTER_VOLTAGE);
    }

    private void stop() {
        _leftShooter1.set(0);
        _rightShooter1.set(0);
        hoodDown();
    }

    private void setIdle() {
        _leftShooter1.setControl(_shooterVelocity.withVelocity(10));
        _rightShooter1.setControl(_shooterVelocity.withVelocity(10));
        hoodDown();
    }

    private void hoodUp() {
        _leftHood.set(.5);
        _rightHood.set(.5);
    }

    private void hoodDown() {
        _leftHood.set(0);
        _rightHood.set(0);
    }

    public boolean shooterReady(){
        if(_shooterPosition - _shooterTarget <= _shooterError){
            return true;
        }
        return false;
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
