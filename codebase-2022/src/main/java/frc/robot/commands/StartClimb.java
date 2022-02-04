package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class StartClimb extends ParallelCommandGroup {
    public StartClimb(DriveTrain driveTrain) {
        addCommands(
            new MoveToPosition(driveTrain, driveTrain.getDistance() - Constants.kDriveTrain.DISTANCE_TO_MID_RUN_FROM_WALL)
        );
    }
}
