package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Superstructure.WantedState;

public class FeederSubsystem extends SubsystemBase {
    public enum FeederState {
        SPINNING,
        IDLE
    }

    private final SparkMax m_SparkMax;
    private FeederState state;
    private WantedState currentWantedState;

    public FeederSubsystem() {
        m_SparkMax = new SparkMax(Constants.FeederConstants.kMotorID, MotorType.kBrushless);
        state = FeederState.IDLE;
    }

    public void setVoltage(double power) {
        state = power != 0 ? FeederState.SPINNING : FeederState.IDLE;
        m_SparkMax.setVoltage(power);
    }

    public Command feederSpinCommand(double power) {
        return runOnce(() -> {
            setVoltage(power);
        });
    }

    public FeederState getState() {
        return state;
    }
    
    @Override
    public void periodic() {
        if (currentWantedState == WantedState.HOME) {
            setVoltage(Constants.IntakeRollerConstants.kVoltage);
        }
        else {
            setVoltage(0);
        }
    }

    @Override
    public void simulationPeriodic() {
        // Called once per scheduler run during simulation
    }

    public boolean isReady() {
        if (currentWantedState != WantedState.HOME) {
            return state == FeederState.IDLE;
        }
        return state == FeederState.SPINNING;
    }

    public void setWantedState(WantedState wantedState) {
        this.currentWantedState = wantedState;
    }
}
