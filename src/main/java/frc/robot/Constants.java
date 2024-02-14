package frc.robot;

// Import statements
import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants.
 * This class should not be used for any other purpose. All constants should be declared globally (i.e. public static).
 * Do not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes) wherever the constants are needed,
 * to reduce verbosity.
 */
public final class Constants {

    // Robot mass in kilograms
    public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound

    // Chassis matter representing the robot's mass and center of mass
    public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);

    // Loop time in seconds
    public static final double LOOP_TIME = 0.13; //s, 20ms + 110ms spark max velocity lag

    // Constants related to autonomous mode
    public static final class AutonConstants {
        // PID constants for translation and angle control during autonomous mode
        public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7 , 0, 0);
        public static final PIDConstants ANGLE_PID = new PIDConstants(0.4, 0, 0.01);
    }

    // Constants related to the drivebase
    public static final class DrivebaseConstants {
        // Hold time on motor brakes when disabled
        public static final double WHEEL_LOCK_TIME = 10; // seconds
    }

    // Constants related to operator controls
    public static class OperatorConstants {
        // Joystick Deadband
        public static final double LEFT_X_DEADBAND = 0.1;
        public static final double LEFT_Y_DEADBAND = 0.1;
        public static final double RIGHT_X_DEADBAND = 0.1;

        // Constant used for turning control
        public static final double TURN_CONSTANT = 6;
    }
}
