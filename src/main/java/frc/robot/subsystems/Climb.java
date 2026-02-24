package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionDutyCycle;

import com.ctre.phoenix6.hardware.TalonFX;
import com.fasterxml.jackson.annotation.JsonTypeInfo.None;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.states.ClimbState;

public class Climb extends SubsystemBase {
    private ClimbState _currentState = ClimbState.IDLE;
    private TalonFX _climb = new TalonFX(23); // Must find Talon ID
    private PositionDutyCycle _ClosedPosition = new PositionDutyCycle(0);
    private PositionDutyCycle _OpenPosition = new PositionDutyCycle(85.5);
    private String _TargetPosition = "None";


    public Climb() {
    }

    public void setWantedState(ClimbState wantedState) {
        if (wantedState != _currentState) {
            _currentState = wantedState;
        }
    }

    public void Extend() {
        _climb.setControl(_OpenPosition);
        _TargetPosition = "Open";
    }

    public void Retract() {
        _climb.setControl(_ClosedPosition);
                _TargetPosition = "Closed";
    }

    public void Stop() {
        _climb.set(0);
    }

    public void periodic() {
        handleCurrentState();
        SmartDashboard.putNumber("ClimbPosition", _climb.getPosition().getValueAsDouble());
        SmartDashboard.putString("TargetClimbPosition", _TargetPosition);
    }

    public void teleopInit() {
        Extend();
    }

    private void handleCurrentState() {
        switch (_currentState) {
            case IDLE:
                Stop();
                break;
            case EXTEND:
                Extend();
                break;
            case RETRACT:
                Retract();
                break;
            // case CLIMB_PREP:

            // break;
            default:
                Stop();
                break;
        }
    }

}
