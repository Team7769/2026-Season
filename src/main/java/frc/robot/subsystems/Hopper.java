package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.states.HopperState;

public class Hopper extends SubsystemBase {
    private HopperState _currentState = HopperState.IDLE;
    private TalonFX _floor;
    private TalonFX _injector;
    private TalonFX _intake;
    private TalonFX _slide;
    private CANdi _candiRight;

    private final VoltageOut HALT = new VoltageOut(0);
    private final VoltageOut FLOOR_INJECT = new VoltageOut(11);
    private final VoltageOut INTAKE = new VoltageOut(7);//7


    private final PositionDutyCycle INTAKE_IN = new PositionDutyCycle(.1);
    private final PositionDutyCycle INTAKE_OUT = new PositionDutyCycle(14.25);
    private final PositionDutyCycle INTAKE_SHOOT = new PositionDutyCycle(4.5);

    public void setWantedState(HopperState wantedState) {
        if (wantedState != _currentState) {
            _currentState = wantedState;
        }
    }

    public Hopper() {
        configInjector();
        configHopper();

    }

    private void configHopper() {
        _intake = new TalonFX(20);

        var slideConfiguration = new TalonFXConfiguration();
        var currentLimits = new CurrentLimitsConfigs()
                                .withStatorCurrentLimit(60)
                                .withSupplyCurrentLimit(20)
                                .withSupplyCurrentLowerTime(1)
                                .withSupplyCurrentLowerLimit(10);
        var slideSlot0Configs = new Slot0Configs()
                                 .withKP(0.25)
                                 .withKD(0.001);

        slideConfiguration.withSlot0(slideSlot0Configs);
        slideConfiguration.withCurrentLimits(currentLimits);
        slideConfiguration.MotorOutput.Inverted= InvertedValue.Clockwise_Positive;
        
        _slide = new TalonFX(21);
        _slide.getConfigurator().apply(slideConfiguration);
        _candiRight = new CANdi(33);
    }

    private void configInjector(){
        _injector = new TalonFX(19);
        _injector.setNeutralMode(NeutralModeValue.Coast);
        _floor = new TalonFX(22);
    }

    public boolean isMiddleEmpty(){
        return _candiRight.getS2Closed().getValue();
    }

    @Override
    public void periodic() {
        handleCurrentState();
        SmartDashboard.putBoolean("Middle Sensor", isMiddleEmpty());
    }

    private void handleCurrentState() {
        switch (_currentState) {
            case IDLE:
                handleIdle();
                break;
            case FLOOR_INTAKE:
                handleFloorIntake();
                break;
            case INJECTING:
                handleInject();
                break;
            case INJECTING_HOPPER_OUT:
                handleInjectHopperOut();
                break;
            default:
                handleStow();
                break;
        }
    }

    private void handleFloorIntake() {
        _slide.setControl(INTAKE_OUT);
        _intake.setControl(INTAKE);
        _floor.setControl(HALT);
        _injector.setControl(HALT);
    }

    private void handleIdle() {
        _intake.setControl(HALT);
        _floor.setControl(HALT);
        _injector.setControl(HALT);
    }

    private void handleStow() {
        _slide.setControl(INTAKE_IN);
        _intake.setControl(HALT);
        _floor.setControl(HALT);
        _injector.setControl(HALT);
    }

    private void handleInject() {
        // if(_candiRight.getS2Closed().getValue()){
        // _slide.setControl(INTAKE_OUT);
        // _intake.setControl(INTAKE);
        // _floor.setControl(FLOOR_INJECT);
        // _injector.setControl(FLOOR_INJECT);
        // } else {
        _slide.setControl(INTAKE_SHOOT);
        _intake.setControl(INTAKE);
        _floor.setControl(FLOOR_INJECT);
        _injector.setControl(FLOOR_INJECT);  
        // }
    }

    private void handleInjectHopperOut() {
        _slide.setControl(INTAKE_OUT);
        _intake.setControl(INTAKE);
        _floor.setControl(FLOOR_INJECT);
        _injector.setControl(FLOOR_INJECT);
    }
}