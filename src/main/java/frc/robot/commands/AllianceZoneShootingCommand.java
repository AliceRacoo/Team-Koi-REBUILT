package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AllianceZoneShootingCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final HoodSubsystem hoodSubsystem;
    private final FeederSubsystem feederSubsystem;

    public AllianceZoneShootingCommand(ShooterSubsystem shooterSubsystem, HoodSubsystem hoodSubsystem, FeederSubsystem feederSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        this.hoodSubsystem = hoodSubsystem;
        this.feederSubsystem = feederSubsystem;
        addRequirements(shooterSubsystem, hoodSubsystem, feederSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.setTargetRPM(Constants.ShooterConstants.kNeutralZoneShootingRPM);

        hoodSubsystem.setAngle(Constants.HoodConstants.kAllianceAngle);
    }

    @Override
    public void execute() {
        if (shooterSubsystem.isAtTargetVelocity()) {
            feederSubsystem.setTargetRpm(Constants.FeederConstants.kGrabRpm);
        }
        else {
            feederSubsystem.setTargetRpm(0);
        }


    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setTargetRPM(0.0);
        feederSubsystem.setTargetRpm(0);
    }
}
