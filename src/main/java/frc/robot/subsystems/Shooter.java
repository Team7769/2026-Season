package frc.robot.subsystems;


import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;

import frc.robot.generated.TunerConstants;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.states.ShooterState;


public class Shooter extends SubsystemBase {
    private ShooterState _currentState = ShooterState.IDLE;
    private TalonFX _leftShooter1;
    private TalonFX _rightShooter1;
    private TalonFX _leftShooter2;
    private TalonFX _rightShooter2;
    private TalonFX _injector;

    private PIDController _hoodPID;
    private double _shooterTarget;
    private boolean _isReadyToShoot = false;

    public Shooter() {
        configShooter();
        configInjector();
        _hoodPID = new PIDController(0.1,0,0.01);
    }

    private void configShooter(){
        _leftShooter1 = new TalonFX(kLeftShooter1);
        _leftShooter2 = new TalonFX(kLeftShooter2);//follow
        _rightShooter1 = new TalonFX(kRightShooter1);
        _rightShooter2 = new TalonFX(kRightShooter2);//follow
        _leftShooter1.setNeutralMode(NeutralModeValue.Coast);
        _leftShooter2.setNeutralMode(NeutralModeValue.Coast);
        _rightShooter1.setNeutralMode(NeutralModeValue.Coast);
        _rightShooter2.setNeutralMode(NeutralModeValue.Coast);
        _leftShooter2.setControl(new Follower(kLeftShooter1, false));
        _rightShooter2.setControl(new Follower(kRightShooter1, false));

    }

    private void configInjector(){
        _injector = new TalonFX(kInjectorRoller);
        _injector.setNeutralMode(NeutralModeValue.Coast);
    }

    private void prepShooter() {
        _leftShooter1.setControl(new VelocityVoltage(30));
        _rightShooter1.setControl(new VelocityVoltage(30));
        injectorOff();
    }

    private void shoot() {
        _leftShooter1.setControl(new VelocityVoltage(30));
        _rightShooter1.setControl(new VelocityVoltage(30));
        injectorOn();
    }

    private void stop() {
        _leftShooter1.set(0);
        _rightShooter1.set(0);
        injectorOff();
    }

    private void setIdle() {
        _leftShooter1.setControl(new VelocityVoltage(10));
        _rightShooter1.setControl(new VelocityVoltage(10));
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
                shoot();
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
