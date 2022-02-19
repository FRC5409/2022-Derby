// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Pigeon;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** A simple drive straight command that can be used by the DriveTrain */
public class TurnToAngle extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveTrain sys_drive;
  private final Pigeon sys_pigeon;
  private final double angle;
  private double startAngle;

  double range;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TurnToAngle(DriveTrain driveTrain, Pigeon pigeon, double toAngle) {
    sys_drive = driveTrain;
    sys_pigeon = pigeon;
    angle = toAngle;

    addRequirements(sys_drive, sys_pigeon);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Turning to angle");
    startAngle = Math.abs(sys_pigeon.getAngle() % 360);

    range = Math.abs(angle - startAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double cAngle = Math.abs(sys_pigeon.getAngle() % 360);

    System.out.println(cAngle);

    int sign = ((cAngle + startAngle) / range < 0) ? -1 : 1;
    // double left = -(1 * Math.pow(Math.abs(angle) - Math.abs(sys_pigeon.getAngle() % 360) / Math.abs(angle), 2));
    // double right = (1 * Math.pow(Math.abs(angle) - Math.abs(sys_pigeon.getAngle() % 360) / Math.abs(angle), 2));

    double left = -(1 - sign * Math.pow((cAngle - startAngle) / range, 3));
    double right = (1 - sign * Math.pow((cAngle - startAngle) / range, 3));

    System.out.println(right);



    // if (Math.abs(left) < 0.8)
    //   left = -0.8;

    // if (Math.abs(right) < 0.8)
    //   right = 0.8;

    sys_drive.tankDrive((float) left, (float) right);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sys_drive.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(angle - (sys_pigeon.getAngle() % 360)) <= 2;
    // return true;
  }
}
