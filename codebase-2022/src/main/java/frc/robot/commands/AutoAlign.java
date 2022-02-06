package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.kDriveTrain;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Pigeon;

public class AutoAlign extends SequentialCommandGroup {
    public AutoAlign(DriveTrain driveTrain, Pigeon pigeon, double toAngle) {
        super();

        addRequirements(driveTrain, pigeon);

        addCommands(
                new TurnToAngle(driveTrain, pigeon, toAngle),
                new DriveToMidRung(driveTrain),
                new MoveToPosition(driveTrain));

    }
}
