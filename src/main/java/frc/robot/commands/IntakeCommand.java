package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeRollerSubsytem;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.IntakeArmSubsystem.IntakeArmState;

public class IntakeCommand extends Command {

    private final IntakeArmSubsystem arm;
    private final IntakeRollerSubsytem rollers;

    private boolean firstTimeReady = true;

    public IntakeCommand(IntakeArmSubsystem arm, IntakeRollerSubsytem rollers) {
        this.arm = arm;
        this.rollers = rollers;
        addRequirements(arm, rollers);
    }

    @Override
    public void execute() {
        arm.OpenArm();
        if (arm.getState() != IntakeArmState.OPEN)
            return;
        if (firstTimeReady) {
            Superstructure.getInstance().getRumbleSubsystem().rumble(Constants.IntakeRollerConstants.kIntakeReadyRumble);
            firstTimeReady = false;
        }
        rollers.setTargetRpm(Constants.IntakeRollerConstants.kIntakePower);
    }

    public void end(boolean interrupted) {
        rollers.setTargetRpm(0);
        firstTimeReady = true;
    }
}
