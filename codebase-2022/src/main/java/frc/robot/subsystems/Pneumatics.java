package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pneumatics extends SubsystemBase {
    /**
     * Creates a new Pneumatics.
     */
    Compressor compressor1;

    private int m_shiftCount = 0;
    private boolean m_autoShift = false;
    private boolean m_manualAutoFillOverride = false;
    final private Timer m_fillTimer;

    /**
     * Constructor for the Pneumatics class
     */
    public Pneumatics() {
        compressor1 = new Compressor(Constants.kPneumatics.MODULE, PneumaticsModuleType.CTREPCM);
        startLoop();

        m_fillTimer = new Timer();
        m_fillTimer.reset();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        if (m_autoShift && m_fillTimer.get() > 24) {
            endLoop();

            m_autoShift = false;
            m_fillTimer.reset();
        }
    }

    /**
     * This method will set the closed loop to true.
     */
    private void startLoop() {
        compressor1.enableDigital();
    }

    /**
     * This method will set the closed loop to be false.
     */
    private void endLoop() {
        compressor1.disable();
    }

    /**
     * This method will close the compressor loop control if it is open and open it
     * if it is closed.
     */
    public void toggle() {
        if (compressor1.enabled())
            endLoop();
        else
            startLoop();
    }

    /**
     * This method will increment the number of gearshifts (compressor uses)
     */
    public void incrementCount() {
        m_shiftCount++;

        if (!m_manualAutoFillOverride)
            checkCount();
    }

    /**
     * This method will check to see if the shifts have exceeded a set constant and
     * will begin to refill the compressor.
     */
    private void checkCount() {
        if (m_shiftCount > 11) {
            startLoop();
            m_fillTimer.reset();
            m_fillTimer.start();

            m_autoShift = true;
            m_shiftCount = 0;
        }
    }

    /**
     * This method will close the compressor.
     */
    public void turnOff() {
        compressor1.close();
    }

    /**
     * This method will set the value of m_manualAutoFillOverride
     * 
     * @param state New value of m_manualAutoFillOverride
     */
    public void setManualOverride(boolean state) {
        m_manualAutoFillOverride = state;
    }
}