
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Climb.MoveLeftStatic;
import frc.robot.commands.Climb.MoveRightSolenoid;
import frc.robot.commands.Climb.MoveRightStatic;
import frc.robot.commands.Climb.MoveLeftSolenoid;
import frc.robot.commands.Climb.MoveTalon;
import frc.robot.commands.Intake.MoveIntakeTalon;
import frc.robot.commands.Intake.MoveLeftServo;
import frc.robot.commands.Intake.MoveRightServo;
import frc.robot.subsystems.Climb.SimLeftServoClimb;
import frc.robot.subsystems.Climb.SimLeftPiston;
import frc.robot.subsystems.Climb.SimPivotMotor;
import frc.robot.subsystems.Climb.SimRightPiston;
import frc.robot.subsystems.Climb.SimRightServoClimb;
import frc.robot.subsystems.Intake.SimIntakeArmMotor;
import frc.robot.subsystems.Intake.SimLeftServoClaw;
import frc.robot.subsystems.Intake.SimRightServoClaw;
public class Robot2 extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private SimLeftPiston m_lPiston;
  private SimLeftServoClaw m_lServo;
  private SimPivotMotor m_climbMotor;
  private SimLeftServoClimb m_staticLeftServo;
  private SimRightPiston m_rPiston;
  private SimRightServoClimb m_staticRightServo;
  private SimRightServoClaw m_rServo;
  private SimIntakeArmMotor m_intakeTalon;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    m_lPiston = new SimLeftPiston();
    m_rPiston = new SimRightPiston();
    m_lServo = new SimLeftServoClaw();
    m_climbMotor = new SimPivotMotor();
    m_staticLeftServo = new SimLeftServoClimb();
    m_staticRightServo = new SimRightServoClimb();
    m_rServo = new SimRightServoClaw();
    m_intakeTalon = new SimIntakeArmMotor();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    ControlMap.m_z.whenPressed(new MoveLeftSolenoid(m_lPiston));
    ControlMap.m_j.whenPressed(new MoveRightSolenoid(m_rPiston));
    ControlMap.m_n.whenPressed(new MoveLeftServo(m_lServo));
    ControlMap.m_m.whenPressed(new MoveTalon(m_climbMotor));
    ControlMap.m_l.whenPressed(new MoveLeftStatic(m_staticLeftServo));
    ControlMap.m_q.whenPressed(new MoveRightStatic(m_staticRightServo));
    ControlMap.m_y.whenPressed(new MoveRightServo(m_rServo));
    ControlMap.m_u.whenPressed(new MoveIntakeTalon(m_intakeTalon));
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
