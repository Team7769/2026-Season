package frc.robot.subsystems;


import java.util.function.Supplier;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
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
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.states.ShooterState;
import frc.robot.configuration.FieldConstants;


public class Shooter extends SubsystemBase {
    private ShooterState _currentState = ShooterState.IDLE;
    private TalonFX _leftShooter1;
    private TalonFX _rightShooter1;
    private TalonFX _leftShooter2;
    private TalonFX _rightShooter2;
    private double _hoodPosition = .5;
    private VelocityVoltage _shooterVelocity = new VelocityVoltage(0);
    private final VelocityTorqueCurrentFOC shotVelocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(52);
    private final VelocityTorqueCurrentFOC idleVelocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(35);
    private final PositionDutyCycle HOOD_DOWN = new PositionDutyCycle(.05);
    private final PositionDutyCycle HOOD_MOVE = new PositionDutyCycle(.6);
    private final VoltageOut SHOOTER_VOLTAGE = new VoltageOut(3);
    private DigitalInput _leftPhotoEye;
    private DigitalInput _rightPhotoEye;
    private TalonFX _hoodMotor;
    private final PositionDutyCycle EMERGENCY_HOOD = new PositionDutyCycle(0.44);
    private final VelocityTorqueCurrentFOC EMERGENCY_SHOT = new VelocityTorqueCurrentFOC(51.5);
    private final PositionDutyCycle EMERGENCY_FEED_HOOD = new PositionDutyCycle(0.7);
    private final VelocityTorqueCurrentFOC EMERGENCY_FEED_SHOT = new VelocityTorqueCurrentFOC(61.5);


    private SimpleMotorFeedforward _ff = new SimpleMotorFeedforward(0, 0);
    private boolean _isReadyToShoot = false;
    private double _shooterPosition;
    private double _shooterTarget;
    private double _shooterError = 0.1;
    // private double _hoodPosition = .5;
    private double _hoodTarget;
    
    private final Drivetrain DRIVETRAIN;
    private InterpolatingDoubleTreeMap _hoodMap = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap _shooterMap = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap _feedHoodMap = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap _feedShooterMap = new InterpolatingDoubleTreeMap();
    private InterpolatingDoubleTreeMap _currentShooterMap;
    private InterpolatingDoubleTreeMap _currentHoodMap;

    private final double[] kDistanceIDs = {1.77, 2, 2.5, 3, 3.5, 4};
    private final double[] kHoodAngles = {4.5, 5.1, 5.55, 5.85, 6.2, 6.35};
    private final double[] kShooterSpeeds = {67, 67, 67, 67, 67, 67};

    public Shooter(Drivetrain drivetrain) {
        DRIVETRAIN = drivetrain;
        // _hoodMap.put(2.0, .3);
        // _hoodMap.put(3.5, .4);
        // _hoodMap.put(2.0, .4);
        // _hoodMap.put(3.5, .56);
        // _hoodMap.put(5.0, .6);

        // _hoodMap.put(1.0, .4);
        // _hoodMap.put(2.54, .7);
        // _hoodMap.put(3.1, .9);
        // _hoodMap.put(4.0, 1.15);
        //everything -.38
        _hoodMap.put(1.0, .06);
        _hoodMap.put(2.0, .23);
        _hoodMap.put(3.0, .32);
        _hoodMap.put(3.5, .4);
        _hoodMap.put(4.0, 0.48);
        _hoodMap.put(5.0, 0.55);

        _feedHoodMap.put(5.0, .055);
        // _feedHoodMap.put(2.0, .15);
        // _feedHoodMap.put(3.0, .4);
        _feedHoodMap.put(7.5, 0.7);
        _feedHoodMap.put(10.0, 0.77);
        _feedHoodMap.put(12.0, 0.85);
        
        _feedShooterMap.put(5.0, 58.5);
        // _feedShooterMap.put(2.0, .15);
        //_feedShooterMap.put(3.0, .4);
        _feedShooterMap.put(7.5, 70.0);
        _feedShooterMap.put(10.0, 80.0);
        _feedShooterMap.put(12.0, 85.0);

        // _shooterMap.put(2.0, 60.0);

        // _shooterMap.put(3.5, 60.0);
        _shooterMap.put(1.0, 45.0);
        _shooterMap.put(2.0, 47.0);
        _shooterMap.put(3.0, 51.5);
        _shooterMap.put(3.5, 53.0);        
        _shooterMap.put(4.0, 54.5);
        _shooterMap.put(5.0, 58.5);

        configShooter();
        configHood();
        
        _currentShooterMap = _shooterMap;
        _currentHoodMap = _hoodMap;
    }

    private void configShooter(){
        _leftShooter1 = new TalonFX(15);
        _leftShooter2 = new TalonFX(16);//follow
        _rightShooter1 = new TalonFX(17);
        _rightShooter2 = new TalonFX(18);//follow

        TalonFXConfiguration rightShooterConfig = new TalonFXConfiguration();
        var rightSlot0 = rightShooterConfig.Slot0;

        rightSlot0.kV = 0.3;
        rightSlot0.kP = 3;
        rightSlot0.kS = 0.7;

        TalonFXConfiguration leftShooterConfig = new TalonFXConfiguration();
        leftShooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        var leftSlot0 = leftShooterConfig.Slot0;

        leftSlot0.kV = 0.03;
        leftSlot0.kP = 4;
        leftSlot0.kS = 3.7;

        _leftShooter1.getConfigurator().apply(leftShooterConfig);
        _leftShooter2.getConfigurator().apply(leftShooterConfig);
        _rightShooter1.getConfigurator().apply(rightShooterConfig);
        _rightShooter2.getConfigurator().apply(rightShooterConfig);

        _leftShooter1.setNeutralMode(NeutralModeValue.Coast);
        _leftShooter2.setNeutralMode(NeutralModeValue.Coast);
        _rightShooter1.setNeutralMode(NeutralModeValue.Coast);
        _rightShooter2.setNeutralMode(NeutralModeValue.Coast);

        _leftShooter2.setControl(new Follower(_leftShooter1.getDeviceID(), MotorAlignmentValue.Aligned));
        _rightShooter1.setControl(new Follower(_leftShooter1.getDeviceID(), MotorAlignmentValue.Opposed));
        _rightShooter2.setControl(new Follower(_leftShooter1.getDeviceID(), MotorAlignmentValue.Opposed));

    }

    private void configHood(){
        _hoodMotor = new TalonFX(40);

        TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
        hoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        var hoodSlot0 = hoodConfig.Slot0;

        hoodSlot0.kP = 1.5;
        //raise
        hoodSlot0.kD = 0.02;
        
        _hoodMotor.getConfigurator().apply(hoodConfig);
        _hoodMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void setShotMap(boolean isFeedShot) {
        if (isFeedShot) {
            _currentHoodMap = _feedHoodMap;
            _currentShooterMap = _feedShooterMap;

        } else {
            _currentHoodMap = _hoodMap;
            _currentShooterMap = _shooterMap;
        }
        
        SmartDashboard.putBoolean("ShotMapFeed", isFeedShot);
    }

    private void prepShooter() {
        _leftShooter1.setControl(shotVelocityTorqueCurrentFOC);
       // _rightShooter1.setControl(shotVelocityTorqueCurrentFOC);
        _hoodMotor.setControl(HOOD_MOVE);
    }

    private void shoot(double shot) {
        _leftShooter1.setControl(shotVelocityTorqueCurrentFOC);
       // _rightShooter1.setControl(shotVelocityTorqueCurrentFOC);
    }

    private void emergencyShot(){
        _hoodMotor.setControl(EMERGENCY_HOOD);
        _leftShooter1.setControl(EMERGENCY_SHOT);
       // _rightShooter1.setControl(EMERGENCY_SHOT);
        
    }

        private void emergencyFeed(){
        _hoodMotor.setControl(EMERGENCY_FEED_HOOD);
        _leftShooter1.setControl(EMERGENCY_FEED_SHOT);
       // _rightShooter1.setControl(EMERGENCY_SHOT);
        
    }

    private void feedShot(){

    }

    private void stop() {
        _leftShooter1.set(0);
       // _rightShooter1.set(0);
        _hoodMotor.setControl(HOOD_DOWN);
    }

    private void setIdle() {
        _leftShooter1.setControl(idleVelocityTorqueCurrentFOC);
       // _rightShooter1.setControl(idleVelocityTorqueCurrentFOC);
        _hoodMotor.setControl(HOOD_DOWN);
    }

    public boolean shooterReady(){
        if(_shooterPosition - _shooterTarget <= _shooterError){
            return true;
        }
        return false;
    }

    //     public boolean hoodReady(){
    //     if(_hoodPosition - _hoodTarget <= _shooterError){
    //         return true;
    //     }
    //     return false;
    // }

    public void setWantedState(ShooterState wantedState) {
        if (wantedState != _currentState) {
            _currentState = wantedState;
        }
    }

    public void setShooterTargetSpeed(Supplier<Double> distanceSupplier){
        var distance = distanceSupplier.get();
        setShotMap(distance > 5);

        var targetSpeed = _currentShooterMap.get(distance);
        shotVelocityTorqueCurrentFOC.Velocity = targetSpeed;
        
        HOOD_MOVE.Position = _currentHoodMap.get(distance);
    }

    @Override
    public void periodic() {
        setShooterTargetSpeed(DRIVETRAIN::getDistanceToTarget);
        // SmartDashboard.putNumber("Left Hood", _leftHood.getPosition());
        // SmartDashboard.putNumber("Left Hood Speed", _leftHood.getSpeed());
        SmartDashboard.putString("Shooter State", _currentState.name());
        SmartDashboard.putNumber("Set Hood Position", _hoodPosition);
        SmartDashboard.putNumber("Shooter Target Velocity", shotVelocityTorqueCurrentFOC.Velocity);
        SmartDashboard.putBoolean("Shooter At Speed", _leftShooter1.getClosedLoopError().getValueAsDouble()<=1);
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
            case EMERGENCY:
                emergencyShot();
                break;
            case EMERGENCY_FEED:
                emergencyFeed();
                break;
            default:
                setIdle();
                break;
        }
    }
}
