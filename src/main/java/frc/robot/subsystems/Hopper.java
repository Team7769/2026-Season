package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.S1CloseStateValue;
import com.ctre.phoenix6.signals.S2CloseStateValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.enums.DriveTarget;
import frc.robot.states.HopperState;

public class Hopper extends SubsystemBase {
    private HopperState _currentState = HopperState.IDLE;
    private TalonFX _floor;
    private TalonFX _injector;
    private TalonFX _intake;
    private TalonFX _intake2;
    private TalonFX _slide;
    private CANdi _candiFuel;

    private final VoltageOut HALT = new VoltageOut(0);
     private final VoltageOut SLOW = new VoltageOut(5);
    private final VoltageOut FLOOR_INJECT = new VoltageOut(11);
    private final VoltageOut FLOOR_INJECT_ZONE = new VoltageOut(9);
    private final VoltageOut REVERSE = new VoltageOut(-11);
    //private final VelocityTorqueCurrentFOC INTAKE = new VelocityTorqueCurrentFOC(85);
    private final VoltageOut INTAKE = new VoltageOut(11);

    private Drivetrain _drivetrain;
    private int _timer = 0; 
    private final PositionDutyCycle INTAKE_IN = new PositionDutyCycle(.1);
    private final PositionDutyCycle INTAKE_OUT = new PositionDutyCycle(14.25);
    private final PositionDutyCycle INTAKE_SHOOT = new PositionDutyCycle(4.5);

    public void setWantedState(HopperState wantedState) {
        if (wantedState != _currentState) {
            _currentState = wantedState;
        }
    }

    public Hopper(Drivetrain drivetrain) {
        configInjector();
        configHopper();
        _drivetrain = drivetrain;
    }

    private void configHopper() {
        _intake = new TalonFX(20);
        _intake2 = new TalonFX(36);

        var slideConfiguration = new TalonFXConfiguration();
        var currentLimits = new CurrentLimitsConfigs()
                                .withStatorCurrentLimit(60)
                                .withSupplyCurrentLimit(30)
                                .withSupplyCurrentLowerTime(1)
                                .withSupplyCurrentLowerLimit(20);
        var slideSlot0Configs = new Slot0Configs()
                                 .withKP(0.25)
                                 .withKD(0.001);
        var intakeConfiguration = new TalonFXConfiguration();
        var intakeSlot0Configs = new Slot0Configs()
                                 .withKP(0.68)
                                 .withKS(4.7)
                                 .withKV(0.03);
        // var intakeCurrentLimits = new CurrentLimitsConfigs()
        //                             .withStatorCurrentLimit(80) //60
        //                             .withSupplyCurrentLimit(60) //40
        //                             .withSupplyCurrentLowerTime(1)
        //                             .withSupplyCurrentLowerLimit(40); //30
        var intakeCurrentLimits = new CurrentLimitsConfigs()
                                    .withStatorCurrentLimit(100) //60
                                    .withSupplyCurrentLimit(60) //40
                                    .withSupplyCurrentLowerTime(0)//1
                                    .withSupplyCurrentLowerLimit(40); //30
        intakeConfiguration.withCurrentLimits(intakeCurrentLimits);

        slideConfiguration.withSlot0(slideSlot0Configs);
        intakeConfiguration.withSlot0(intakeSlot0Configs);
        slideConfiguration.withSlot0(slideSlot0Configs);
        slideConfiguration.withCurrentLimits(currentLimits);
        slideConfiguration.MotorOutput.Inverted= InvertedValue.Clockwise_Positive;
        
        _slide = new TalonFX(21);
        _slide.getConfigurator().apply(slideConfiguration);
        _candiFuel = new CANdi(34);
        var candiConfig = new CANdiConfiguration();
        candiConfig.DigitalInputs.S1CloseState = S1CloseStateValue.CloseWhenHigh;
        candiConfig.DigitalInputs.S2CloseState = S2CloseStateValue.CloseWhenHigh;
        _candiFuel.getConfigurator().apply(candiConfig);
        _intake.getConfigurator().apply(intakeConfiguration);
        _intake2.getConfigurator().apply(intakeConfiguration);
        _slide.setPosition(0.0);

        _intake2.setControl(new Follower(_intake.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    private void configInjector(){
        _injector = new TalonFX(19);
        _injector.setNeutralMode(NeutralModeValue.Coast);
        _floor = new TalonFX(22);
    }

    public boolean isMiddleEmpty(){
        return _candiFuel.getS2Closed().getValue();
    }

    public boolean isFuelStaged(){
        return _candiFuel.getS1Closed().getValue();
    }

    @Override
    public void periodic() {
        handleCurrentState();
        SmartDashboard.putBoolean("Middle Sensor", isMiddleEmpty());
        SmartDashboard.putBoolean("Middle Sensor 2", _candiFuel.getS2Closed().getValue());
        _timer++;
        if(_timer>60){//50
            _timer = 0;
        }
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
            case EMERGENCY:
                handleEmergency();
                break;
            case JAM:
                handleJam();
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

    private void handleJam() {
        _injector.setControl(REVERSE);
    }

    private void handleInject() {
        _intake.setControl(SLOW);
        if(_drivetrain.getDriveTarget()==DriveTarget.ZONE){
        _injector.setControl(FLOOR_INJECT_ZONE);
        } else {
            _injector.setControl(FLOOR_INJECT);
        }
         


        if(_timer > 55){//45
        //_floor.setControl(REVERSE);
       }else{
        _floor.setControl(FLOOR_INJECT);
       }
        if(_timer <= 15){//20
            _slide.setControl(INTAKE_IN);
        } else if(_timer > 15 && _timer < 30) {//20,30
            _slide.setControl(INTAKE_SHOOT);//maybe do intake_shoot?
        } else if(_timer >= 30 && _timer < 45) {//30,40
            _slide.setControl(INTAKE_IN);
        } else {
            _slide.setControl(INTAKE_SHOOT);//maybe do intake_shoot?
        }
               // if(_timer <= 20){
        //     _injector.setControl(FLOOR_INJECT);
        // } else if(_timer >= 20 && _timer < 30) {
        //     _injector.setControl(SLOW);
        // } else if(_timer >= 30 && _timer < 40) {
        //     _injector.setControl(FLOOR_INJECT);
        // } else {
        //     _injector.setControl(SLOW);
       // }
    }

    private void handleEmergency(){
        _slide.setControl(INTAKE_OUT);
        _intake.setControl(REVERSE);
        _floor.setControl(REVERSE);
        _injector.setControl(REVERSE);
    }

    private void handleInjectHopperOut() {
        _slide.setControl(INTAKE_OUT);
        _intake.setControl(INTAKE);
        _injector.setControl(FLOOR_INJECT);

       if(_timer > 45){
       _floor.setControl(REVERSE);
       }else{
        _floor.setControl(FLOOR_INJECT);
       }
        
    }
}