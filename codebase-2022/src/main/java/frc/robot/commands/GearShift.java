package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Pneumatics;

public class GearShift extends CommandBase {
  private final DriveTrain sys_driveSubsystem;
  private final Pneumatics sys_pneumatics;

  /**
   * Creates a new FastGearShift.
   */
  public GearShift(DriveTrain driveTrain, Pneumatics pneumatics) {
    sys_driveSubsystem = driveTrain;
    sys_pneumatics = pneumatics;
    addRequirements(sys_driveSubsystem, sys_pneumatics);

    System.out.println("Gear Shift command executed");
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Shifts to fast gear
    sys_driveSubsystem.fastShift();
    sys_pneumatics.incrementCount();
  }

  @Override
  public void end(boolean interrupted) {
    sys_driveSubsystem.slowShift();
    sys_pneumatics.incrementCount();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // sys_driveSubsystem.slowShift();
    // sys_pneumatics.incrementCount();
    return true;
  }
}
