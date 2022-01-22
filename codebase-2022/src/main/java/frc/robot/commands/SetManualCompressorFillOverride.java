package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pneumatics;

public class SetManualCompressorFillOverride extends CommandBase {
    private final Pneumatics sys_pneumatics;
    private final boolean m_state;

    /**
     * Creates a new FastGearShift.
     */
    public SetManualCompressorFillOverride(Pneumatics pneumatics, boolean state) {
        sys_pneumatics = pneumatics;
        m_state = state;

        addRequirements(sys_pneumatics);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        sys_pneumatics.setManualOverride(m_state);;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
