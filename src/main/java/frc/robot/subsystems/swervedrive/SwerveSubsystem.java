// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants.AutonConstants;
import java.io.File;
import java.util.function.DoubleSupplier;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {

    // Swerve drive object
    private final SwerveDrive swerveDrive;
    // Maximum speed of the robot in meters per second, used to limit acceleration
    public double maximumSpeed = Units.feetToMeters(14.5);

    // Initialize SwerveSubsystem with the directory provided
    public SwerveSubsystem(File directory) {
        // Angle conversion factor calculation
        double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(12.8);
        // Drive conversion factor calculation
        double driveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(4), 6.75);
        System.out.println("\"conversionFactor\": {");
        System.out.println("\t\"angle\": " + angleConversionFactor + ",");
        System.out.println("\t\"drive\": " + driveConversionFactor);
        System.out.println("}");

        // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot via angle

        setupPathPlanner();
    }

    // Construct the swerve drive with provided configurations
    public SwerveSubsystem(SwerveDriveConfiguration driveCfg, SwerveControllerConfiguration controllerCfg) {
        swerveDrive = new SwerveDrive(driveCfg, controllerCfg, maximumSpeed);
    }

    // Setup AutoBuilder for PathPlanner
    public void setupPathPlanner() {
        // Configuration for path following
        AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry
            this::getRobotVelocity, // ChassisSpeeds supplier
            this::setChassisSpeeds, // Method to drive the robot
            new HolonomicPathFollowerConfig(
                AutonConstants.TRANSLATION_PID, // Translation PID constants
                AutonConstants.ANGLE_PID, // Rotation PID constants
                4.5, // Max module speed, in m/s
                swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(), // Drive base radius in meters
                new ReplanningConfig() // Default path replanning config
            ),
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                var alliance = DriverStation.getAlliance();
                return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
            },
            this // Reference to this subsystem to set requirements
        );
    }

    // Get the autonomous command to follow a specified path
    public Command getAutonomousCommand(String pathName, boolean setOdomToStart) {
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

        if (setOdomToStart) {
            resetOdometry(new Pose2d(path.getPoint(0).position, getHeading()));
        }

        return AutoBuilder.followPath(path);
    }

    // Drive to a specified pose using PathPlanner Path finding
    public Command driveToPose(Pose2d pose) {
        PathConstraints constraints = new PathConstraints(
            swerveDrive.getMaximumVelocity(), 4.0,
            swerveDrive.getMaximumAngularVelocity(), Units.degreesToRadians(720)
        );

        return AutoBuilder.pathfindToPose(
            pose,
            constraints,
            0.0, // Goal end velocity in meters/sec
            0.0 // Rotation delay distance in meters
        );
    }

    // Drive the robot using translative values and heading as a setpoint
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
                                DoubleSupplier headingX, DoubleSupplier headingY) {
        return run(() -> {
            double xInput = Math.pow(translationX.getAsDouble(), 3);
            double yInput = Math.pow(translationY.getAsDouble(), 3);
            driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(xInput, yInput,
                headingX.getAsDouble(), headingY.getAsDouble(),
                swerveDrive.getOdometryHeading().getRadians(), swerveDrive.getMaximumVelocity()));
        });
    }

    // Drive the robot using translative values and heading as a setpoint
    public Command simDriveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
                                   DoubleSupplier rotation) {
        return run(() -> {
            driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(translationX.getAsDouble(),
                translationY.getAsDouble(), rotation.getAsDouble() * Math.PI,
                swerveDrive.getOdometryHeading().getRadians(), swerveDrive.getMaximumVelocity()));
        });
    }

    // Command to characterize the robot drive motors using SysId
    public Command sysIdDriveMotorCommand() {
        return SwerveDriveTest.generateSysIdCommand(
            SwerveDriveTest.setDriveSysIdRoutine(new Config(), this, swerveDrive, 12), 3.0, 5.0, 3.0);
    }

    // Command to characterize the robot angle motors using SysId
    public Command sysIdAngleMotorCommand() {
        return SwerveDriveTest.generateSysIdCommand(
            SwerveDriveTest.setAngleSysIdRoutine(new Config(), this, swerveDrive), 3.0, 5.0, 3.0);
    }

    // Drive the robot using translative values and heading as angular velocity
    public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
                                DoubleSupplier angularRotationX) {
        return run(() -> {
            swerveDrive.drive(new Translation2d(Math.pow(translationX.getAsDouble(), 3) * swerveDrive.getMaximumVelocity(),
                Math.pow(translationY.getAsDouble(), 3) * swerveDrive.getMaximumVelocity()),
                Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumAngularVelocity(),
                true, false);
        });
    }

    // Control the drivebase with translative values and rotation
    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        swerveDrive.drive(translation, rotation, fieldRelative, false);
    }

    // Drive the robot given field-oriented velocity
    public void driveFieldOriented(ChassisSpeeds velocity) {
        swerveDrive.driveFieldOriented(velocity);
    }

    // Drive the robot given robot-oriented velocity
    public void drive(ChassisSpeeds velocity) {
        swerveDrive.drive(velocity);
    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
    }

    // Get the swerve drive kinematics object
    public SwerveDriveKinematics getKinematics() {
        return swerveDrive.kinematics;
    }

    // Reset odometry to the given pose
    public void resetOdometry(Pose2d initialHolonomicPose) {
        swerveDrive.resetOdometry(initialHolonomicPose);
    }

    // Get the current pose of the robot
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    // Set chassis speeds with closed-loop velocity control
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        swerveDrive.setChassisSpeeds(chassisSpeeds);
    }

    // Post the trajectory to the field
    public void postTrajectory(Trajectory trajectory) {
        swerveDrive.postTrajectory(trajectory);
    }

    // Reset the gyro angle to zero and reset odometry
    public void zeroGyro() {
        swerveDrive.zeroGyro();
    }

    // Set the drive motors to brake/coast mode
    public void setMotorBrake(boolean brake) {
        swerveDrive.setMotorIdleMode(brake);
    }

    // Get the current yaw angle of the robot
    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    // Get the chassis speeds based on controller input
    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
        xInput = Math.pow(xInput, 3);
        yInput = Math.pow(yInput, 3);
        return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, headingX, headingY,
            getHeading().getRadians(), maximumSpeed);
    }

    // Get the chassis speeds based on controller input
    public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
        xInput = Math.pow(xInput, 3);
        yInput = Math.pow(yInput, 3);
        return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, angle.getRadians(),
            getHeading().getRadians(), maximumSpeed);
    }

    // Get the current field-relative velocity of the robot
    public ChassisSpeeds getFieldVelocity() {
        return swerveDrive.getFieldVelocity();
    }

    // Get the current velocity of the robot
    public ChassisSpeeds getRobotVelocity() {
        return swerveDrive.getRobotVelocity();
    }

    // Get the SwerveController in the swerve drive
    public SwerveController getSwerveController() {
        return swerveDrive.swerveController;
    }

    // Get the SwerveDriveConfiguration object
    public SwerveDriveConfiguration getSwerveDriveConfiguration() {
        return swerveDrive.swerveDriveConfiguration;
    }

    // Lock the swerve drive to prevent it from moving
    public void lock() {
        swerveDrive.lockPose();
    }

    // Get the current pitch angle of the robot
    public Rotation2d getPitch() {
        return swerveDrive.getPitch();
    }

    // Add a fake vision reading for testing purposes
    public void addFakeVisionReading() {
        swerveDrive.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp());
    }
}