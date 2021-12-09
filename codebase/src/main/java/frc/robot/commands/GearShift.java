package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class GearShift extends CommandBase {
    private final DriveTrain sys_driveSubsystem;
  
    /**
     * Creates a new FastGearShift.
     */
    public GearShift(DriveTrain subsystem) {
      sys_driveSubsystem = subsystem; 
      addRequirements(sys_driveSubsystem);
      System.out.println("Gear Shift command executed");
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      // Shifts to fast gear
      sys_driveSubsystem.fastShift();
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      sys_driveSubsystem.slowShift();
      return true;
    }
  }
