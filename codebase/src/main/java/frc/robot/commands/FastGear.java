package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class FastGear extends CommandBase {
    private final DriveTrain sys_driveSubsystem;
  
    /**
     * Creates a new FastGearShift.
     */
    public FastGear(DriveTrain subsystem) {
      sys_driveSubsystem = subsystem; 
      addRequirements(sys_driveSubsystem);
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
      return true;
    }
  }
