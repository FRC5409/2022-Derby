// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kDriveTrain;

/**
 * DriveTrain subsystem
 */
public class DriveTrain extends SubsystemBase {
    private final WPI_TalonSRX mot_leftFrontDrive;
    private final WPI_TalonSRX mot_leftRearDrive;
    private final WPI_TalonSRX mot_rightFrontDrive;
    private final WPI_TalonSRX mot_rightRearDrive;

    private final DifferentialDrive m_drive;
    private int m_driveMode = kDriveTrain.AADL_DRIVE;

    /** Creates a new DriveTrain. */
    public DriveTrain() {
        mot_leftFrontDrive = new WPI_TalonSRX(kDriveTrain.CANLeftDriveFront); // Has encoder
        mot_leftFrontDrive.setInverted(true);

        mot_leftRearDrive = new WPI_TalonSRX(kDriveTrain.CANLeftDriveRear);
        mot_leftRearDrive.follow(mot_leftFrontDrive);

        mot_rightFrontDrive = new WPI_TalonSRX(kDriveTrain.CANRightDriveFront); // Has encoder
        // mot_rightFrontDrive.setInverted(true);

        mot_rightRearDrive = new WPI_TalonSRX(kDriveTrain.CANRightDriveRear);
        mot_rightRearDrive.follow(mot_rightFrontDrive);

        // flt_leftFront = mot_leftFrontDrive.getFaults(toFill)
        m_drive = new DifferentialDrive(mot_leftFrontDrive, mot_rightFrontDrive);
    }

    /**
     * This method is called once per scheuler run and is used to update smart dashboard data.
     */
    public void periodic() {
        displayDriveModeData();
        // displayEncoderData();

        SmartDashboard.putString("Drive Mode", getDriveModeName());
    }

    @Override
    public void simulationPeriodic() {

    }

    /**
     * This method sets the drive mode to the next one
     */
    public void nextDriveMode() {
        m_driveMode++;

        if (m_driveMode > 3)
            m_driveMode = kDriveTrain.AADL_DRIVE;
    }

    /**
     * This method sets the drive mode to the previous one
     */
    public void previousDriveMode() {
        m_driveMode--;

        if (m_driveMode < 1)
            m_driveMode = kDriveTrain.T_DRIVE;
    }

    /**
     * This method returns the current drive mode
     * 
     * @return int value corresponding to the current drive mode
     */
    public int getDriveMode() {
        return m_driveMode;
    }

    /**
     * This method will return the string value corresponding to the current drive
     * mode. Ideally should be used as a way to display the data to the driver
     * station.
     * 
     * @return String value for current drive mode
     */
    public String getDriveModeName() {
        switch (m_driveMode) {
            case kDriveTrain.AADL_DRIVE:
                return "AADL DRIVE";
            case kDriveTrain.C_DRIVE:
                return "CURVATURE DRIVE";
            case kDriveTrain.T_DRIVE:
                return "TANK DRIVE";
            default:
                return "AADL DRIVE";
        }
    }

    /**
     * This method will display values to the smart dashboard that relate to the
     * current drive mode.
     */
    public void displayDriveModeData() {
        switch (m_driveMode) {
            case kDriveTrain.AADL_DRIVE:
                SmartDashboard.delete("CD_Speed");
                SmartDashboard.delete("CD_Turn");
                SmartDashboard.delete("CD_Quik Turn");

                SmartDashboard.delete("TD_Left Speed");
                SmartDashboard.delete("TD_Right Speed");
                break;
            case kDriveTrain.C_DRIVE:
                SmartDashboard.delete("AD_Acceleration");

                SmartDashboard.delete("TD_Left Speed");
                SmartDashboard.delete("TD_Right Speed");
                break;
            case kDriveTrain.T_DRIVE:
                SmartDashboard.delete("AD_Acceleration");

                SmartDashboard.delete("CD_Speed");
                SmartDashboard.delete("CD_Turn");
                SmartDashboard.delete("CD_Quik Turn");
                break;
        }
    }

    /**
     * This method will display encoder data.
     */
    // public void displayEncoderData() {
    //     double velocity = mot_leftFrontDrive.getSelectedSensorVelocity();

    //     SmartDashboard.putNumber("MOT_FL_VEL", velocity);
    // }

    // public double getVelocity() {
    //     return mot_leftFrontDrive.getSelectedSensorVelocity();
    // }

    /**
     * This method will rotate the motors with the given parameters
     * 
     * @param acceleration Right acceleration
     * @param deceleration Left accelertation
     * @param turn         Turn percentage
     */
    public void aadlDrive(final double acceleration, final double deceleration, final double turn) {
        double accelrate = acceleration - deceleration;

        m_drive.arcadeDrive(accelrate, turn, true);
        SmartDashboard.putNumber("AD_Acceleration", accelrate);
    }

    /**
     * This method will move the robot in a direction based on the given speeds
     * 
     * @param speed     Motor straight speed (-1.0 - 1.0)
     * @param turn      Motor horizontal speed (-1.0 - 1.0)
     * @param quickTurn Boolean fast turn value
     */
    public void curvDrive(final double speed, final double turn, final boolean quickTurn) {
        m_drive.curvatureDrive(speed, turn, quickTurn);
        SmartDashboard.putNumber("CD_Speed", speed);
        SmartDashboard.putNumber("CD_Turn", turn);
        SmartDashboard.putBoolean("CD_Quik Turn", quickTurn);
    }

    /**
     * This method will move the robot in a direction based on the given speeds
     * 
     * @param leftSpeed  Left motor speed
     * @param rightSpeed Right motor speed
     */
    public void tankDrive(final float leftSpeed, final float rightSpeed) {
        m_drive.tankDrive(leftSpeed, rightSpeed);

        SmartDashboard.putNumber("TD_Left Speed", leftSpeed);
        SmartDashboard.putNumber("TD_Right Speed", rightSpeed);
    }
}
