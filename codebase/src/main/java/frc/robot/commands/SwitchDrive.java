package frc.robot.commands;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.Constants.kDriveTrain;

/**
 * SwitchDrive command
 */
public class SwitchDrive extends CommandBase {
    private final DriveTrain sys_drive;
    private final XboxController m_joystick;

    /**
     * Constructor for the SwitchDrive command
     * 
     * @param sys_drive DruveTrain subsystem
     * @param joystick  Joystick
     */
    public SwitchDrive(DriveTrain sys_drive, XboxController joystick) {
        this.sys_drive = sys_drive;
        m_joystick = joystick;

        addRequirements(sys_drive);
    }

    /**
     * Called every time the scheduler runs while the command is scheduled. Will
     * call the drive mode based on the current drive mode.
     */
    @Override
    public void execute() {
        switch (sys_drive.getDriveMode()) {
            case kDriveTrain.ARCADE_DRIVE: // 1
                arcadeDriveExecute();
                break;
            case kDriveTrain.AADL_DRIVE: // 2
                addlDriveExecute();
                break;
            case kDriveTrain.C_DRIVE: // 3
                curvatureDriveExecute();
                break;
            case kDriveTrain.T_DRIVE: // 4
                tankDriveExecute();
                break;
            default:
                arcadeDriveExecute();
                break;
        }

        SmartDashboard.putData("DriveMode Data", (Sendable) sys_drive.getCurrentCommand());
    }

    /**
     * This method will get input values and run the arcade drive method.
     */
    private void arcadeDriveExecute() {
        double lAxisX = m_joystick.getX(Hand.kLeft) * -1;

        double lAxisY = m_joystick.getY(Hand.kLeft) * -1;

        // if (lAxisY < 0)
        //     lAxisX *= -1;

        sys_drive.arcadeDrive(lAxisY, lAxisX);
    }

    /**
     * This method will get input values and run the addlDrive method.
     */
    private void addlDriveExecute() {
        double rightTrigger = m_joystick.getTriggerAxis(Hand.kRight);
        double leftTrigger = m_joystick.getTriggerAxis(Hand.kLeft);

        double lAxis = m_joystick.getX(Hand.kLeft) * -1;

        sys_drive.aadlDrive(rightTrigger, leftTrigger, lAxis);
    }

    /**
     * This method will get input values and run the curvDrive method.
     */
    private void curvatureDriveExecute() {
        double speed = m_joystick.getY(Hand.kLeft) * -1;
        double turn = m_joystick.getX(Hand.kLeft);

        // true if the 'b' button on the controller is pressed
        boolean quickTurn = m_joystick.getBButton();

        sys_drive.curvDrive(speed, turn, quickTurn);
    }

    /**
     * This method will get input values and run the tankDrive method.
     */
    public void tankDriveExecute() {
        double leftSpeed = m_joystick.getY(Hand.kLeft) * -1;
        double rightSpeed = m_joystick.getY(Hand.kRight) * -1;

        sys_drive.tankDrive((float) leftSpeed, (float) rightSpeed);
    }
}
