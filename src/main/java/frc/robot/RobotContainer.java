package frc.robot;

// Import statements
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Define robot subsystems and input devices
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  CommandJoystick driverController = new CommandJoystick(1);
  XboxController driverXbox = new XboxController(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure trigger bindings
    configureBindings();

    // Define commands for controlling the drivebase
    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
        () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
        driverXbox::getYButtonPressed,
        driverXbox::getAButtonPressed,
        driverXbox::getXButtonPressed,
        driverXbox::getBButtonPressed);

    // Set default command for the drivebase
    drivebase.setDefaultCommand(closedAbsoluteDriveAdv);
  }

  /**
   * Use this method to define trigger->command mappings.
   */
  private void configureBindings() {
    // Example trigger bindings
    new JoystickButton(driverXbox, 1).onTrue(new InstantCommand(drivebase::zeroGyro));
    new JoystickButton(driverXbox, 3).onTrue(new InstantCommand(drivebase::addFakeVisionReading));
    new JoystickButton(driverXbox, 2).whileTrue(Commands.deferredProxy(() -> drivebase.driveToPose(
        new Pose2d(new Translation2d(2.6, 4.2), Rotation2d.fromDegrees(161.16)))));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return drivebase.getAutonomousCommand("Short", true);
  }

  public void setMotorBrake(boolean b) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setMotorBrake'");
  }

  public void setDriveMode() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setDriveMode'");
  }

  // Additional methods for setting drive mode and motor brake
}
