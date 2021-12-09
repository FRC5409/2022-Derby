package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {
    /**
     * Creates a new Pneumatics.
     */
    Compressor compressor1;

    public Pneumatics() {
        compressor1 = new Compressor(0);
        compressor1.start();
        compressAir();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void compressAir() {
        compressor1.setClosedLoopControl(true);
    }

    public void toggle() {
        if (compressor1.getClosedLoopControl())
            compressor1.setClosedLoopControl(false);
        else
            compressor1.setClosedLoopControl(true);
    }

    public void turnOff() {
        // releaseAir();
        compressor1.close();
    }
}