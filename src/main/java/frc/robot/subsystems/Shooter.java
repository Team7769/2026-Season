package frc.robot.subsystems;


import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;


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
    private TalonFX _topInjector;
    private TalonFX _bottomInjector;

    private PIDController _hoodPID;
    private double _shooterTarget;
    private boolean _isReadyToShoot = false;

    public Shooter() {
        configShooter();
        configInjector();
    }

    private void configShooter(){
        _leftShooter1.setControl();
        _leftShooter2.setControl();//follow
        _rightShooter1.setControl();
        _rightShooter2.setControl();//follow
    }

    private void configInjector(){
        _topInjector.setControl();
        _bottomInjector.setControl();//follow
    }

    private void prepShooter() {
        _leftShooter1.setControl();
        _rightShooter1.setControl();
        injectorOff();
    }

    private void shoot() {
        _leftShooter1.setControl();
        _rightShooter1.setControl();
        injectorOn();
    }

    private void stop() {
        _leftShooter1.set(0);
        _rightShooter1.set(0);
        injectorOff();
    }

    private void setIdle() {
        _leftShooter1.setControl();
        _rightShooter1.setControl();
        injectorOff();
    }

    private void injectorOn() {
        _topInjector.set(0.3);
        _bottomInjector.set(0.3);
    }

        private void injectorOff() {
        _topInjector.set(0);
        _bottomInjector.set(0);
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
