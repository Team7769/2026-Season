package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.states.IntakeState;
import frc.robot.generated.TunerConstants;


public class Intake extends SubsystemBase {
    private IntakeState _currentState = IntakeState.IDLE;
    private TalonFX _intake;
    private TalonFX _slide;

    // _swerve.getModule(0)
    public Intake() {
        _intake = new TalonFX(20);
        _slide = new TalonFX(21);
        
    }
    public void startIntake() {
        _intake.set(-0.3);
    }

    public void stopIntake() {
        _intake.set(0);
    }

    public void intakeIn() {
        _slide.set(.2);
    }

    public void intakeOut() {
        _slide.set(-.2);
    }

    public void stopSlide() {
        _slide.set(0);
    }

    public void setWantedState(IntakeState wantedState) {
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
            case STOP:
                stopIntake();
                break;
            case INTAKE:
                startIntake();
                break;
            case OUT:
                intakeOut();
                break;
            case IN:
                intakeIn();
                break;
            case HOLD:
                stopSlide();
                break;
            default:
                stopIntake();
                stopSlide();
                break;
        }
    }
}

