package frc.robot.commands;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.XboxController;
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
     * @param sys_drive     DruveTrain subsystem
     * @param joystick      Joystick
     */
    public SwitchDrive(DriveTrain sys_drive, XboxController joystick) {
        this.sys_drive = sys_drive;
        m_joystick = joystick;

        addRequirements(sys_drive);
    }

    /** Called every time the scheduler runs while the command is scheduled.
     *  Will call the drive mode based on the current drive mode. 
     */
    @Override
    public void execute() {
        switch (sys_drive.getDriveMode()) {
            case kDriveTrain.ARCADE_DRIVE:  // 1
                arcadeDriveExecute();
                break;
            case kDriveTrain.AADL_DRIVE: // 2
                defaultDriveExecute();
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

    private void arcadeDriveExecute() {
        double acceleration = m_joystick.getLeftY() * -1;
        double turn = m_joystick.getLeftX();

        sys_drive.arcadeDrive(acceleration, turn);
    }

    /**
     * This method will get input values and run the addlDrive method.
     */
    private void defaultDriveExecute() {
        double rightTrigger = m_joystick.getRightTriggerAxis();
        double leftTrigger = m_joystick.getLeftTriggerAxis();

        double lAxis = m_joystick.getLeftX() * -1;

        sys_drive.aadlDrive(rightTrigger, leftTrigger, lAxis);
    }

    /**
     * This method will get input values and run the curvDrive method.
     */
    private void curvatureDriveExecute() {
        double speed = m_joystick.getLeftY() * -1;
        double turn = m_joystick.getLeftX() * -1;

        // true if the 'b' button on the controller is pressed
        boolean quickTurn = m_joystick.getBButton();

        sys_drive.curvDrive(speed, turn, quickTurn);
    }

    /**
     * This method will get input values and run the tankDrive method.
     */
    public void tankDriveExecute() {
        double leftSpeed = m_joystick.getLeftY() * -1;
        double rightSpeed = m_joystick.getRightY() * -1;

        sys_drive.tankDrive((float)leftSpeed, (float)rightSpeed);
    }
}
