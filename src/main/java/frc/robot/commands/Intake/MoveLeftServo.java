// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import java.util.Random;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ControlMap;
import frc.robot.subsystems.Intake.SimLeftServoClaw;

public class MoveLeftServo extends CommandBase {
  private final SimLeftServoClaw m_claw;
  public int max = 180;
  public int min = 1;
  /** Creates a new MoveServo. */
  public MoveLeftServo(SimLeftServoClaw claw) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_claw = claw;
    addRequirements(claw);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Random num = new Random();
    int output = num.nextInt(max);
    output += min;
    if(ControlMap.m_b.get()){
      m_claw.setServoAngle(output);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
