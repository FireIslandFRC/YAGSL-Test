package frc.robot;

// Import statements
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.io.File;
import java.io.IOException;
import swervelib.parser.SwerveParser;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot {

    // Static instance of the Robot class
    private static Robot instance;
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;
    private Timer disabledTimer;

    // Constructor
    public Robot() {
        instance = this;
    }

    // Method to get the instance of the Robot class
    public static Robot getInstance() {
        return instance;
    }

    // Initialization code run when the robot is first started up
    @Override
    public void robotInit() {
        m_robotContainer = new RobotContainer();
        disabledTimer = new Timer();
    }

    // Periodic code run every 20 ms, regardless of mode
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    // Code run when the robot enters Disabled mode
    @Override
    public void disabledInit() {
        m_robotContainer.setMotorBrake(true);
        disabledTimer.reset();
        disabledTimer.start();
    }

    @Override
    public void disabledPeriodic() {
        if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME)) {
            m_robotContainer.setMotorBrake(false);
            disabledTimer.stop();
        }
    }

    // Code run when Autonomous mode is initiated
    @Override
    public void autonomousInit() {
        m_robotContainer.setMotorBrake(true);
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    // Code run when Teleop mode is initiated
    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        m_robotContainer.setDriveMode();
        m_robotContainer.setMotorBrake(true);
    }

    @Override
    public void teleopPeriodic() {
    }

    // Code run when Test mode is initiated
    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
        try {
            new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"));
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public void testPeriodic() {
    }

    // Code run when simulation is initiated
    @Override
    public void simulationInit() {
    }

    // Periodic code run during simulation
    @Override
    public void simulationPeriodic() {
    }
}
