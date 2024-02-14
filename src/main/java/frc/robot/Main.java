package frc.robot;

// Import statement
import edu.wpi.first.wpilibj.RobotBase;

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what you are doing, do
 * not modify this file except to change the parameter class to the startRobot call.
 */
public final class Main {

    // Private constructor to prevent instantiation of the Main class
    private Main() {
    }

    /**
     * Main initialization function. Do not perform any initialization here.
     *
     * <p>If you change your main robot class, change the parameter type.
     */
    public static void main(String... args) {
        // Start the robot by passing the Robot class as a constructor reference to startRobot
        RobotBase.startRobot(Robot::new);
    }
}
