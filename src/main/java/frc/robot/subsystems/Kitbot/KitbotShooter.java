package frc.robot.subsystems.Kitbot;


import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.states.ShooterState;

public class KitbotShooter extends SubsystemBase {
    private ShooterState _currentState = ShooterState.PAUSE;
    private Orchestra _orchestra;
    private TalonFX index = new TalonFX(16);
    private TalonFX shooter = new TalonFX(15);
        private TalonFX a = new TalonFX(2);
    private TalonFX b = new TalonFX(3);
        private TalonFX c = new TalonFX(5);
    private TalonFX d = new TalonFX(6);

    // _swerve.getModule(0)
    public KitbotShooter() {
        _orchestra = new Orchestra();
        _orchestra.loadMusic("heyYa.chrp");
        _orchestra.addInstrument(index);
        _orchestra.addInstrument(shooter);
        _orchestra.addInstrument(a);
        _orchestra.addInstrument(b);
        _orchestra.addInstrument(c);
        _orchestra.addInstrument(d);
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
                stopIndex();
                stopShooter();
                break;
        }
    }
}
