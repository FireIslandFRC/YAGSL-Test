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

// Command class for absolute drive control of swerve drive subsystem
public class AbsoluteDrive extends Command {

    // Instance variables
    private final SwerveSubsystem swerve;
    private final DoubleSupplier vX, vY;
    private final DoubleSupplier headingHorizontal, headingVertical;
    private boolean initRotation = false;

    // Constructor
    public AbsoluteDrive(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY,
                         DoubleSupplier headingHorizontal, DoubleSupplier headingVertical) {
        this.swerve = swerve;
        this.vX = vX;
        this.vY = vY;
        this.headingHorizontal = headingHorizontal;
        this.headingVertical = headingVertical;
        
        // Require the swerve subsystem
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        initRotation = true;
    }

    // Execute method called repeatedly
    @Override
    public void execute() {
        // Get the desired chassis speeds based on joystick inputs and heading
        ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(vX.getAsDouble(), vY.getAsDouble(),
                                                             headingHorizontal.getAsDouble(),
                                                             headingVertical.getAsDouble());

        // Prevent movement after auto
        if (initRotation) {
            if (headingHorizontal.getAsDouble() == 0 && headingVertical.getAsDouble() == 0) {
                // Get current heading
                Rotation2d firstLoopHeading = swerve.getHeading();
                // Set the current heading to the desired heading
                desiredSpeeds = swerve.getTargetSpeeds(0, 0, firstLoopHeading.getSin(), firstLoopHeading.getCos());
            }
            // Don't initialize rotation again
            initRotation = false;
        }

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
