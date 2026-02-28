package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeArmConstants;
import frc.robot.subsystems.IntakeArmSubsystem;
import frc.robot.subsystems.IntakeArmSubsystem.IntakeArmState;

/* You're allowed to refer to this command as the thug shake, please do */
public class ArmShakeCommand extends Command {
    private final IntakeArmSubsystem intakeArmSubsystem;
    

    public ArmShakeCommand(IntakeArmSubsystem intakeArmSubsystem) {
        this.intakeArmSubsystem = intakeArmSubsystem;
        addRequirements(intakeArmSubsystem);
    
    }

    @Override
    public void initialize() {
    }   
     
    @Override
    public void execute() {

    }

    // changes the rotation direction of the arm
    private void changeDirection(double angle) {
        intakeArmSubsystem.setAngle(angle);
    }

    @Override
    public void end(boolean interrupted) {
        intakeArmSubsystem.CloseArm();
    }
}