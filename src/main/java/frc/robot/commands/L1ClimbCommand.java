package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class L1ClimbCommand extends Command {
    private final ClimberSubsystem climberSubsystem;
    private final double kL1ExtendHeight;
    private final double kL1CloseHeight;

    public L1ClimbCommand(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
        kL1ExtendHeight = Constants.ClimberConstants.kL1ExtendHeight;
        kL1CloseHeight = Constants.ClimberConstants.kL1CloseHeight;
        addRequirements(climberSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (climberSubsystem.getState() == climberSubsystem.ClimberState.AT_TARGET_GROUND)
            climberSubsystem.setPositionHang(y);
        else
            climberSubsystem.setPositionGround(x);
    }

    @Override
    public void end(boolean interrupted) {
    }
}
