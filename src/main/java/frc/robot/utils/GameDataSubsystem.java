package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GameDataSubsystem extends SubsystemBase {
    private final RumbleSubsystem rumble;
    private int currentShift = 0;
    private double nextShiftThreshold = 130.0;
    private final int MAX_SHIFTS = 5;

    public GameDataSubsystem(RumbleSubsystem rumble) {
        this.rumble = rumble;
    }

    @Override
    public void periodic() {
        if (!DriverStation.isTeleop()) return;

        double matchTime = DriverStation.getMatchTime();
        
        if (currentShift < MAX_SHIFTS && matchTime <= nextShiftThreshold && matchTime > 0) {
            triggerShift();
        }
    }

    private void triggerShift() {
        boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        
        if ((currentShift % 2 == 0) != isRed) {
            rumble.rumble(Constants.OperatorConstants.kGameShiftRumble);
        }

        currentShift++;
        nextShiftThreshold -= Constants.OperatorConstants.kTeleopInterval;
        
        if (nextShiftThreshold < Constants.OperatorConstants.kEndGameTime) {
            nextShiftThreshold = -1;
        }
    }
}