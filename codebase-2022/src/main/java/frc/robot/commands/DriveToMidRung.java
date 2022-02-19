// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** A simple drive straight command that can be used by the DriveTrain */
public class DriveToMidRung extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveTrain sys_drive;
  boolean distanceValid = false;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveToMidRung(DriveTrain driveTrain) {
    sys_drive = driveTrain;

    addRequirements(sys_drive);

    System.out.println("Starting drive");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sys_drive.clearDistances();
    distanceValid = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distance = sys_drive.getDistance();

    if (!distanceValid)
      sys_drive.tankDrive(0.7f, 0.7f);

    if (sys_drive.getValidDistance() && distance <= 1.3) {
      sys_drive.addDistance(distance);
      // System.out.println(distance);
      System.out.println(sys_drive.measuredDistances.toString());
      distanceValid = true;
    } else {
      distanceValid = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sys_drive.tankDrive(0, 0);
    System.out.println("Ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return sys_drive.getNumOfDistances() >= 32;
    // return true;
  }
}
