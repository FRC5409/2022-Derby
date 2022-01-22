// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DefaultDrive extends CommandBase {
  private final DriveTrain sys_drive;
  private final XboxController m_joystick;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DefaultDrive(DriveTrain subsystem, XboxController joystick) {
    sys_drive = subsystem;
    m_joystick = joystick;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      // Getting the value for the right trigger
      double acceleration = m_joystick.getRightTriggerAxis();
      // Getting the value for the left trigger
      double deceleration = m_joystick.getRightTriggerAxis();

      // Getting the hand 
      double turnValue = m_joystick.getLeftX();

      // Executing the command
      sys_drive.aadlDrive(acceleration, deceleration, turnValue);
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
