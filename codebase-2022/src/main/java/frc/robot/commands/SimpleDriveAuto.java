package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;

/**
 * Very basic autonomous command
 */
public class SimpleDriveAuto extends SequentialCommandGroup {
    /**
     * Constructor for the SimpleDriveAuto command. Will queue a command that drives straight for 0.5 seconds (500ms)
     * @param sys_drive     DriveTrain subsystem
     */
    public SimpleDriveAuto(DriveTrain sys_drive) {
        addCommands(new DriveStraight(sys_drive, (float)0.3).withTimeout(0.5));
    }
}
