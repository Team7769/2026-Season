package frc.robot.subsystems;


// import java.util.Random;
// import java.util.stream.IntStream;
// import com.ctre.phoenix6.Orchestra;
// import edu.wpi.first.units.measure.Voltage;
// import edu.wpi.first.wpilibj2.command.Subsystem;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.states.ClimbState;

public class Climb extends SubsystemBase {
    private ClimbState _currentState = ClimbState.IDLE;
    private TalonFX Climb = new TalonFX(0);

    public void setWantedState(ClimbState wantedState) {
        if (wantedState != _currentState) {
            _currentState = wantedState;
        }
    }

    public void Extend() {
        Climb.set(0.3);
    }


    public void Retract() {
        Climb.set(-0.3);
    }

    public void Stop() {
        Climb.set(0);
    }

    public void periodic() {
        handleCurrentState();
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
