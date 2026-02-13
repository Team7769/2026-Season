package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.states.IntakeState;


public class Intake extends SubsystemBase {
    private IntakeState _currentState = IntakeState.IDLE;
    private TalonFX _intake = new TalonFX(16);
    private TalonFX _slide = new TalonFX(15);

    // _swerve.getModule(0)
    public Intake() {

    }
    public void startIntake() {
        _intake.set(-0.3);
    }

    public void stopIntake() {
        _intake.set(0);
    }

    public void slideIn() {
        _slide.set(.55);
    }

    public void slideOut() {
        _slide.set(-.55);
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
                slideOut();
                break;
            case IN:
                slideIn();
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

