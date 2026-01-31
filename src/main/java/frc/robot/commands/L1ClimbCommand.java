package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ClimberSubsystem.ClimberState;

public class L1ClimbCommand extends Command {
    private final ClimberSubsystem climberSubsystem;
    private final double kL1ExtendHeight;
    private final double kL1CloseHeight;

    private boolean GrabbedBar = false;

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
        if (climberSubsystem.getState() == ClimberState.AT_TARGET_GROUND) {
            GrabbedBar = true;
        }

        if (GrabbedBar) {
            climberSubsystem.setPositionHang(kL1CloseHeight);
        } else {
            climberSubsystem.setPositionGround(kL1ExtendHeight);
        }
    }

    @Override
    public void end(boolean interrupted) {}
}
