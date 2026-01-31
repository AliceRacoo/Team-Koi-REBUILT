package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class L1ClimbCommand extends Command {
    private final ClimberSubsystem climberSubsystem;
    private final double x;
    private final double y;

    public L1ClimbCommand(ClimberSubsystem climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
        x = Constants.L1ClimbCommand.x;
        y = Constants.L1ClimbCommand.y;
        addRequirements(climberSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        climberSubsystem.setPositionGround(x);
        if (climberSubsystem.getState() == climberSubsystem.ClimberState.AT_TARGET_GROUND)
            climberSubsystem.setPositionHang(y);
    }

    @Override
    public void end(boolean interrupted) {
    }
}
