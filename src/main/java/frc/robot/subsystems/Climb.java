package frc.robot.subsystems;



import com.ctre.phoenix6.controls.PositionDutyCycle;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.states.ClimbState;

public class Climb extends SubsystemBase {
    private ClimbState _currentState = ClimbState.IDLE;
    private TalonFX _climb = new TalonFX(0); // Must find Talon ID
    private PositionDutyCycle _climbTargetPosition = new PositionDutyCycle(0);

    public Climb() {
    }

    public void setWantedState(ClimbState wantedState) {
        if (wantedState != _currentState) {
            _currentState = wantedState;
        }
    }

    public void Extend() {
        // _climb.set(0.3);
        _climbTargetPosition.Position = 50; //Must find target climb position
         _climb.setControl(_climbTargetPosition);
    }


    public void Retract() {
        // _climb.set(-0.3);
        _climbTargetPosition.Position = 0;
        _climb.setControl(_climbTargetPosition);
    }

    public void Stop() {
        _climb.set(0);
    }

    public void periodic() {
        handleCurrentState();
        SmartDashboard.putNumber("ClimbPosition", _climb.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("TargetClimbPosition", _climbTargetPosition.Position);
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
                
            //     break;
            default:
                Stop();
                break;
        }
    }

}
