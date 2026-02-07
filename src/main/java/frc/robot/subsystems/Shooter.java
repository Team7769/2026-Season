package frc.robot.subsystems;


import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.states.ShooterState;

public class Shooter extends SubsystemBase {
    private ShooterState _currentState = ShooterState.PAUSE;
    private Orchestra _orchestra;
    private TalonFX index = new TalonFX(16);
    private TalonFX shooter = new TalonFX(15);

    // _swerve.getModule(0)
    public Shooter() {
        _orchestra = new Orchestra();
        _orchestra.loadMusic("SeekAndDestroy.chrp");
        _orchestra.addInstrument(index);
        _orchestra.addInstrument(shooter);
    }

    public void intake() {
        index.set(0.3);
        shooter.set(0.65);
    }

    public void shoot() {
        index.set(-0.3);
        shooter.set(0.65);
    }

    public void stopIndex() {
        index.set(0);
    }

    public void moveShooter() {
        shooter.set(.55);
    }

    public void stopShooter() {
        shooter.set(0);
    }

    public void setWantedState(ShooterState wantedState) {
        if (wantedState != _currentState) {
            _currentState = wantedState;
        }
    }

    public void play() {
        _orchestra.play();
    }

    public void pause() {
        _orchestra.pause();
    }

    @Override
    public void periodic() {
        handleCurrentState();
    }

    private void handleCurrentState() {
        switch (_currentState) {
            case PAUSE:
                _orchestra.pause();
                break;
            case PLAY:
                _orchestra.play();
                break;
            case IDLE:
                stopIndex();
                stopShooter();
                break;
            case SHOOT:
                shoot();
                break;
            case INTAKE:
                intake();
                break;
            default:
                _orchestra.stop();
                break;
        }
    }
}
