package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.RumbleSubsystem;
import swervelib.SwerveInputStream;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  private final SendableChooser<Command> autonChooser = new SendableChooser<>();

  private final CommandXboxController m_driverController;
  private final CommandXboxController m_operatorController;

  private final Superstructure superstructure;

  private final RumbleSubsystem rumbleSubsystem;
  private final SwerveSubsystem drivebase;

  private final SwerveInputStream driveAngularVelocity;

  public RobotContainer() {

    // Controllers
    m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
    m_operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);

    // Superstructure & subsystems (lazy init)
    superstructure = Superstructure.getInstance();
    drivebase = superstructure.getDrivebase();
    rumbleSubsystem = superstructure.getRumbleSubsystem();

    setupAutons();

    // Input streams
    driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
        () -> m_driverController.getLeftY() * (-1),
        () -> m_driverController.getLeftX() * (-1))
        .withControllerRotationAxis(() -> m_driverController.getRightX() * -1)
        .deadband(OperatorConstants.kDeadband)
        .scaleTranslation(1)
        .allianceRelativeControl(true);

    configureBindings();
  }

  private void setupAutons() {
    autonChooser.setDefaultOption("get off the line", new RunCommand(() -> {
      drivebase.zeroGyro();
      drivebase.drive(new ChassisSpeeds(1, 0, 0));

    }, drivebase).withTimeout(4));

    System.out.println(FieldConstants.leftStartBlue.getX() + " | " + FieldConstants.leftStartBlue.getX());

    //autonChooser.addOption("Depot intake + shoot", drivebase.getAutonomousCommand("DepotIntake.path"));

    SmartDashboard.putData("Auton/Auton Chooser", autonChooser);
  }

  private void configureBindings() {
    rumbleSubsystem.setControllers(m_driverController, m_operatorController);

    Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);

    m_driverController.rightBumper().onTrue(
        superstructure.setPREPARING_SHOOTERstate());

    m_driverController.rightBumper().onFalse(
        superstructure.setIDLEstate());

    m_driverController.a().onTrue(
        Commands.runOnce(drivebase::zeroGyro));

    //Commands.
  }

  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
    
  }
}
