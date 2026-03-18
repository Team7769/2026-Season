package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class GameManager extends SubsystemBase {

    private String _autoWinner = "";
    private boolean _activeFirst = true;
    private boolean _hasSetActive = false;
    private final CommandXboxController DRIVER_CONTROLLER;

    public GameManager(CommandXboxController driverController) {
        DRIVER_CONTROLLER = driverController;
    }

    @Override
    public void periodic() {
        if (!_hasSetActive) {
            _autoWinner = DriverStation.getGameSpecificMessage();

            var currentAlliance = DriverStation.getAlliance();
            if (currentAlliance.isPresent() && _autoWinner != "" && _autoWinner != null) {
                var currentAllianceColor = currentAlliance.get();
                if ((_autoWinner == "R" && currentAllianceColor == Alliance.Red)
                        || (_autoWinner == "B" && currentAllianceColor == Alliance.Blue)) {
                    _activeFirst = false;
                } else {
                    _activeFirst = true;
                }

                _hasSetActive = true;
            }
        }

        var matchTime = DriverStation.getMatchTime();
        SmartDashboard.putString("Auto Winner", _autoWinner);
        SmartDashboard.putBoolean("Hub Active First", _activeFirst);
        SmartDashboard.putNumber("Match Time", matchTime);
        SmartDashboard.putBoolean("Has Set Active", _hasSetActive);

        //DRIVER_CONTROLLER.setRumble(RumbleType.kBothRumble, 1);
    }
}
