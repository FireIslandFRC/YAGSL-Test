package frc.robot.commands.swervedrive.drivebase;

// Import statements
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.List;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

// Command class for driving a swerve robot in full field-centric mode
public class AbsoluteFieldDrive extends Command {

    // Instance variables
    private final SwerveSubsystem swerve;
    private final DoubleSupplier vX, vY, heading;

    // Constructor
    public AbsoluteFieldDrive(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY,
                            DoubleSupplier heading) {
        this.swerve = swerve;
        this.vX = vX;
        this.vY = vY;
        this.heading = heading;

        // Require the swerve subsystem
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        // No initialization needed
    }

    // Execute method called repeatedly
    @Override
    public void execute() {
        // Get the desired chassis speeds based on joystick inputs and heading
        ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(vX.getAsDouble(), vY.getAsDouble(),
                                                             new Rotation2d(heading.getAsDouble() * Math.PI));

        // Limit velocity to prevent tipping
        Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
        translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(), swerve.getPose(),
                                               Constants.LOOP_TIME, Constants.ROBOT_MASS,
                                               List.of(Constants.CHASSIS), swerve.getSwerveDriveConfiguration());

        // Debugging output
        SmartDashboard.putNumber("LimitedTranslation", translation.getX());
        SmartDashboard.putString("Translation", translation.toString());

        // Make the robot move
        swerve.drive(translation, desiredSpeeds.omegaRadiansPerSecond, true);
    }

    @Override
    public void end(boolean interrupted) {
        // No action needed when the command ends
    }

    @Override
    public boolean isFinished() {
        // Command never finishes on its own
        return false;
    }
}
