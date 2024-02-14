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
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;
import swervelib.math.SwerveMath;

// Command class for advanced absolute drive control of swerve drive subsystem
public class AbsoluteDriveAdv extends Command {

    // Instance variables
    private final SwerveSubsystem swerve;
    private final DoubleSupplier vX, vY, headingAdjust;
    private final BooleanSupplier lookAway, lookTowards, lookLeft, lookRight;
    private boolean resetHeading = false;

    // Constructor
    public AbsoluteDriveAdv(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier headingAdjust,
                            BooleanSupplier lookAway, BooleanSupplier lookTowards, BooleanSupplier lookLeft,
                            BooleanSupplier lookRight) {
        this.swerve = swerve;
        this.vX = vX;
        this.vY = vY;
        this.headingAdjust = headingAdjust;
        this.lookAway = lookAway;
        this.lookTowards = lookTowards;
        this.lookLeft = lookLeft;
        this.lookRight = lookRight;
        
        // Require the swerve subsystem
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        resetHeading = true;
    }

    // Execute method called repeatedly
    @Override
    public void execute() {
        double headingX = 0;
        double headingY = 0;

        // Set heading based on direction buttons
        if (lookAway.getAsBoolean()) {
            headingY = -1; // Face away from drivers
        }
        if (lookRight.getAsBoolean()) {
            headingX = 1; // Face right
        }
        if (lookLeft.getAsBoolean()) {
            headingX = -1; // Face left
        }
        if (lookTowards.getAsBoolean()) {
            headingY = 1; // Face towards drivers
        }

        // Prevent movement after auto
        if (resetHeading) {
            if (headingX == 0 && headingY == 0 && Math.abs(headingAdjust.getAsDouble()) > 0) {
                // Get current heading
                Rotation2d currentHeading = swerve.getHeading();
                // Set the current heading to the desired heading
                headingX = currentHeading.getSin();
                headingY = currentHeading.getCos();
            }
            resetHeading = false; // Don't reset heading again
        }

        // Get desired chassis speeds
        ChassisSpeeds desiredSpeeds = swerve.getTargetSpeeds(vX.getAsDouble(), vY.getAsDouble(), headingX, headingY);

        // Limit velocity to prevent tipping
        Translation2d translation = SwerveController.getTranslation2d(desiredSpeeds);
        translation = SwerveMath.limitVelocity(translation, swerve.getFieldVelocity(), swerve.getPose(),
                                               Constants.LOOP_TIME, Constants.ROBOT_MASS,
                                               List.of(Constants.CHASSIS), swerve.getSwerveDriveConfiguration());

        // Debugging output
        SmartDashboard.putNumber("LimitedTranslation", translation.getX());
        SmartDashboard.putString("Translation", translation.toString());

        // Make the robot move
        if (headingX == 0 && headingY == 0 && Math.abs(headingAdjust.getAsDouble()) > 0) {
            resetHeading = true;
            // Adjust heading based on constant
            swerve.drive(translation, (Constants.OperatorConstants.TURN_CONSTANT * -headingAdjust.getAsDouble()), true);
        } else {
            swerve.drive(translation, desiredSpeeds.omegaRadiansPerSecond, true);
        }
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
