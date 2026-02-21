package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.states.HopperState;

public class Hopper extends SubsystemBase {
    private HopperState _currentState = HopperState.IDLE;
    private TalonFX _floor;

    public void setWantedState(HopperState wantedState) {
        if (wantedState != _currentState) {
            _currentState = wantedState;
        }
    }

    public Hopper() {
        _floor = new TalonFX(22);
    }

    public void startFloor() {
        _floor.set(-0.3);
    }

    public void stopFloor() {
        _floor.set(0);
    }

    @Override
    public void periodic() {
        handleCurrentState();
    }

    private void handleCurrentState() {
        switch (_currentState) {
            case IDLE:
                stopFloor();
                break;

            case FLOOR_INTAKE:
                startFloor();
                break;

            case AUTO:
                startFloor();

            default:
                stopFloor();
                break;
        }
    }
}