// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.kDriveTrain;

/**
 * DriveTrain subsystem
 */
public class DriveTrain extends SubsystemBase {
    private final WPI_TalonSRX leftTalon;
    private final CANSparkMax leftNeo;
    private final WPI_TalonSRX rightTalon;
    private final CANSparkMax rightNeo;

    private final DifferentialDrive m_drive;
    private int m_driveMode = kDriveTrain.AADL_DRIVE;

    // private final Solenoid ssl_gear;
    private boolean m_allowShift = false;
    private long m_timeSinceShift = 0;

    // private final WPI_Pigeon2 gyro_pigeon;
    // private final DifferentialDriveOdometry dOdometry;

    /** Creates a new DriveTrain. */
    public DriveTrain() {
        leftTalon = new WPI_TalonSRX(kDriveTrain.CAN_LEFT_TALON); // Has encoder
        leftTalon.setInverted(false);

        leftNeo = new CANSparkMax(kDriveTrain.CAN_LEFT_NEO, MotorType.kBrushless);
        leftNeo.follow(CANSparkMax.ExternalFollower.kFollowerPhoenix, kDriveTrain.CAN_LEFT_TALON, false);

        rightTalon = new WPI_TalonSRX(kDriveTrain.CAN_RIGHT_TALON); // Has encoder
        rightTalon.setInverted(true);

        rightNeo = new CANSparkMax(kDriveTrain.CAN_RIGHT_NEO, MotorType.kBrushless);
        rightNeo.follow(CANSparkMax.ExternalFollower.kFollowerPhoenix, kDriveTrain.CAN_RIGHT_TALON, true);

        leftTalon.configPeakCurrentLimit(kDriveTrain.MOTOR_CURRENT_LIMIT);
        leftNeo.setSmartCurrentLimit(kDriveTrain.MOTOR_CURRENT_LIMIT);
        rightTalon.configPeakCurrentLimit(kDriveTrain.MOTOR_CURRENT_LIMIT);
        rightNeo.setSmartCurrentLimit(kDriveTrain.MOTOR_CURRENT_LIMIT);

        leftTalon.setNeutralMode(NeutralMode.Coast);
        leftNeo.setIdleMode(IdleMode.kCoast);
        rightTalon.setNeutralMode(NeutralMode.Coast);
        rightNeo.setIdleMode(IdleMode.kCoast);

        // mot_leftFrontDrive.configPul

        // mot_rightFrontDrive.getStatorCurrent();

        m_drive = new DifferentialDrive(leftTalon, rightTalon);

        // ssl_gear = new Solenoid(PneumaticsModuleType.CTREPCM, 0);

        // SmartDashboard.putData(ssl_gear);

        // gyro_pigeon = new WPI_Pigeon2(Constants.kDriveTrain.CANPigeon);
        // dOdometry = new DifferentialDriveOdometry(gyro_pigeon.getRotation2d());
    }

    /**
     * This method is called once per scheuler run and is used to update smart
     * dashboard data.
     */
    public void periodic() {
        // double vel = getSensorVelocity();

        // long currentTime = System.currentTimeMillis();

        // if (currentTime - m_timeSinceShift >= 1000) {
        // if (Math.abs(vel) >= Constants.kDriveTrain.UP_SHIFT && !ssl_gear.get()) {
        // fastShift();
        // } else if (Math.abs(vel) <= Constants.kDriveTrain.DOWN_SHIFT &&
        // ssl_gear.get()) {
        // slowShift();
        // }

        // m_timeSinceShift = currentTime;
        // }

        displayDriveModeData();
        displayEncoderData();

        SmartDashboard.putString("Drive Mode", getDriveModeName());

        // dOdometry.update(gyro_pigeon.getRotation2d(), mot_leftFrontDrive.getSelectedSensorPosition(),
        //         mot_rightFrontDrive.getSelectedSensorPosition());
    }

    @Override
    public void simulationPeriodic() {

    }

    /**
     * This method sets the drive mode to the next one
     */
    public void nextDriveMode() {
        m_driveMode++;

        if (m_driveMode > 4)
            m_driveMode = kDriveTrain.ARCADE_DRIVE;
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
            case kDriveTrain.ARCADE_DRIVE:
                return "ARCADE DRIVE";
            case kDriveTrain.AADL_DRIVE:
                return "AADL DRIVE";
            case kDriveTrain.C_DRIVE:
                return "CURVATURE DRIVE";
            case kDriveTrain.T_DRIVE:
                return "TANK DRIVE";
            default:
                return "ARCADE DRIVE";
        }
    }

    /**
     * This method will display values to the smart dashboard that relate to the
     * current drive mode.
     */
    public void displayDriveModeData() {
        switch (m_driveMode) {
            case kDriveTrain.ARCADE_DRIVE:
                SmartDashboard.delete("ADT_Acceleration");

                SmartDashboard.delete("CD_Speed");
                SmartDashboard.delete("CD_Turn");
                SmartDashboard.delete("CD_Quik Turn");

                SmartDashboard.delete("TD_Left Speed");
                SmartDashboard.delete("TD_Right Speed");

            case kDriveTrain.AADL_DRIVE:
                SmartDashboard.delete("ADS_Acceleration");

                SmartDashboard.delete("CD_Speed");
                SmartDashboard.delete("CD_Turn");
                SmartDashboard.delete("CD_Quik Turn");

                SmartDashboard.delete("TD_Left Speed");
                SmartDashboard.delete("TD_Right Speed");
                break;
            case kDriveTrain.C_DRIVE:
                SmartDashboard.delete("ADS_Acceleration");

                SmartDashboard.delete("ADT_Acceleration");

                SmartDashboard.delete("TD_Left Speed");
                SmartDashboard.delete("TD_Right Speed");
                break;
            case kDriveTrain.T_DRIVE:
                SmartDashboard.delete("ADS_Acceleration");
                SmartDashboard.delete("ADT_Acceleration");

                SmartDashboard.delete("CD_Speed");
                SmartDashboard.delete("CD_Turn");
                SmartDashboard.delete("CD_Quik Turn");
                break;
        }
    }

    /**
     * This method will display encoder data.
     */
    public void displayEncoderData() {
        // SmartDashboard.putNumber("MOT_FL_VEL", getSensorVelocity());
        // SmartDashboard.putNumber("MOT_FL_POS", getSensorPosition());
        // SmartDashboard.putNumber(key, value)
    }

    public double getSensorVelocity() {
        return leftTalon.getSelectedSensorVelocity();
    }

    public double getSensorPosition() {
        return leftTalon.getSelectedSensorPosition();
    }

    /**
     * This method will move the robot in a direction based on the given parameters
     * 
     * @param acceleration Motor speed (-1.0 - 1.0)
     * @param turn         Motor horizontal speed (-1.0 - 1.0)
     */
    public void arcadeDrive(final double acceleration, final double turn) {
        m_drive.arcadeDrive(acceleration, turn, true);
        SmartDashboard.putNumber("ADS_Acceleration", acceleration);
    }

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
        SmartDashboard.putNumber("ADT_Acceleration", accelrate);
    }

    /**
     * This method will move the robot in a direction based on the given parameters
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

    /**
     * This method will put the robot in high gear
     */
    public void fastShift() {
        // ssl_gear.set(true);
    }

    /**
     * This method will put the robot is low gear
     */
    public void slowShift() {
        // ssl_gear.set(false);
    }

    /**
     * Returns the current wheel speeds of the robot in m/s.
     *
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftTalon.getSelectedSensorVelocity() * 10,
                rightTalon.getSelectedSensorVelocity() * 10);
    }

    /**
     * This method will reset the encoders
     */
    // public void resetEncoders() {
    //     leftTalon.configFactoryDefault();
    //     rightTalon.configFactoryDefault();


    // }

    // public void resetOdometery() {
    //     resetEncoders();
    // }
}
