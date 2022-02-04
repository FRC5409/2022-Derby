// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;
import java.util.function.Consumer;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.kDriveTrain;

/**
 * DriveTrain subsystem
 */
public class DriveTrain extends SubsystemBase {
    private final WPI_TalonSRX right_FrontTalon;
    private final WPI_TalonSRX right_BackTalon;
    private final WPI_TalonSRX left_FrontTalon;
    private final WPI_TalonSRX left_BackTalon;

    private final DifferentialDrive m_drive;
    private int m_driveMode = kDriveTrain.AADL_DRIVE;

    private final TimeOfFlight tof;

    // private final Solenoid ssl_gear;
    private boolean m_allowShift = false;
    private long m_timeSinceShift = 0;

    private NetworkTableEntry pSlider;

    private NetworkTableEntry iSlider;

    private NetworkTableEntry dSlider;

    // private final WPI_Pigeon2 gyro_pigeon;
    // private final DifferentialDriveOdometry dOdometry;

    /** Creates a new DriveTrain. */
    public DriveTrain() {
        /**
         * ------------------ RIGHT MOTOTRS ------------------
         * Declerations:
         */
        right_FrontTalon = new WPI_TalonSRX(Constants.kDriveTrain.CAN_RIGHT_FRONT_TALON); // Has encoder
        right_FrontTalon.setInverted(false);

        right_BackTalon = new WPI_TalonSRX(kDriveTrain.CAN_RIGHT_BACK_TALON);
        right_BackTalon.follow(right_FrontTalon);
        right_BackTalon.setInverted(InvertType.FollowMaster);

        // Configurations:
        right_FrontTalon.configFactoryDefault();
        right_BackTalon.configFactoryDefault();

        right_FrontTalon.configPeakCurrentLimit(kDriveTrain.MOTOR_CURRENT_LIMIT);
        right_BackTalon.configPeakCurrentLimit(kDriveTrain.MOTOR_CURRENT_LIMIT);

        right_FrontTalon.setNeutralMode(NeutralMode.Brake);
        right_BackTalon.setNeutralMode(NeutralMode.Brake);

        right_FrontTalon.configNominalOutputForward(0, kDriveTrain.kTimeoutMs);
        right_FrontTalon.configNominalOutputReverse(0, kDriveTrain.kTimeoutMs);
        right_FrontTalon.configPeakOutputForward(1, kDriveTrain.kTimeoutMs);
        right_FrontTalon.configPeakOutputReverse(-1, kDriveTrain.kTimeoutMs);
        right_FrontTalon.configAllowableClosedloopError(0, kDriveTrain.kPIDLoopIdx, kDriveTrain.kTimeoutMs);

        right_FrontTalon.config_kF(kDriveTrain.kPIDLoopIdx, kDriveTrain.kDistanceGains.kF, kDriveTrain.kTimeoutMs);
        right_FrontTalon.config_kP(kDriveTrain.kPIDLoopIdx, kDriveTrain.kDistanceGains.kP, kDriveTrain.kTimeoutMs);
        right_FrontTalon.config_kI(kDriveTrain.kPIDLoopIdx, kDriveTrain.kDistanceGains.kI, kDriveTrain.kTimeoutMs);
        right_FrontTalon.config_kD(kDriveTrain.kPIDLoopIdx, kDriveTrain.kDistanceGains.kD, kDriveTrain.kTimeoutMs);

        right_BackTalon.configNominalOutputForward(0, kDriveTrain.kTimeoutMs);
        right_BackTalon.configNominalOutputReverse(0, kDriveTrain.kTimeoutMs);
        right_BackTalon.configPeakOutputForward(1, kDriveTrain.kTimeoutMs);
        right_BackTalon.configPeakOutputReverse(-1, kDriveTrain.kTimeoutMs);
        right_BackTalon.configAllowableClosedloopError(0, kDriveTrain.kPIDLoopIdx, kDriveTrain.kTimeoutMs);

        right_BackTalon.config_kF(kDriveTrain.kPIDLoopIdx, kDriveTrain.kDistanceGains.kF, kDriveTrain.kTimeoutMs);
        right_BackTalon.config_kP(kDriveTrain.kPIDLoopIdx, kDriveTrain.kDistanceGains.kP, kDriveTrain.kTimeoutMs);
        right_BackTalon.config_kI(kDriveTrain.kPIDLoopIdx, kDriveTrain.kDistanceGains.kI, kDriveTrain.kTimeoutMs);
        right_BackTalon.config_kD(kDriveTrain.kPIDLoopIdx, kDriveTrain.kDistanceGains.kD, kDriveTrain.kTimeoutMs);

        /**
         * ------------------ LEFT MOTOTRS ------------------
         * Declerations:
         */
        left_FrontTalon = new WPI_TalonSRX(kDriveTrain.CAN_LEFT_FRONT_TALON);
        left_FrontTalon.setInverted(true);

        left_BackTalon = new WPI_TalonSRX(kDriveTrain.CAN_LEFT_BACK_TALON); // Back left talon (follower, has encoder)
        left_BackTalon.follow(left_FrontTalon);
        left_BackTalon.setInverted(InvertType.FollowMaster);

        // Configurations:
        left_FrontTalon.configFactoryDefault();
        left_BackTalon.configFactoryDefault();

        left_FrontTalon.configPeakCurrentLimit(kDriveTrain.MOTOR_CURRENT_LIMIT);
        left_BackTalon.configPeakCurrentLimit(kDriveTrain.MOTOR_CURRENT_LIMIT);

        left_FrontTalon.setNeutralMode(NeutralMode.Brake);
        left_BackTalon.setNeutralMode(NeutralMode.Brake);

        left_FrontTalon.configNominalOutputForward(0, kDriveTrain.kTimeoutMs);
        left_FrontTalon.configNominalOutputReverse(0, kDriveTrain.kTimeoutMs);
        left_FrontTalon.configPeakOutputForward(1, kDriveTrain.kTimeoutMs);
        left_FrontTalon.configPeakOutputReverse(-1, kDriveTrain.kTimeoutMs);
        left_FrontTalon.configAllowableClosedloopError(0, kDriveTrain.kPIDLoopIdx, kDriveTrain.kTimeoutMs);

        left_FrontTalon.config_kF(kDriveTrain.kPIDLoopIdx, kDriveTrain.kDistanceGains.kF, kDriveTrain.kTimeoutMs);
        left_FrontTalon.config_kP(kDriveTrain.kPIDLoopIdx, kDriveTrain.kDistanceGains.kP, kDriveTrain.kTimeoutMs);
        left_FrontTalon.config_kI(kDriveTrain.kPIDLoopIdx, kDriveTrain.kDistanceGains.kI, kDriveTrain.kTimeoutMs);
        left_FrontTalon.config_kD(kDriveTrain.kPIDLoopIdx, kDriveTrain.kDistanceGains.kD, kDriveTrain.kTimeoutMs);

        left_BackTalon.configNominalOutputForward(0, kDriveTrain.kTimeoutMs);
        left_BackTalon.configNominalOutputReverse(0, kDriveTrain.kTimeoutMs);
        left_BackTalon.configPeakOutputForward(1, kDriveTrain.kTimeoutMs);
        left_BackTalon.configPeakOutputReverse(-1, kDriveTrain.kTimeoutMs);
        left_BackTalon.configAllowableClosedloopError(0, kDriveTrain.kPIDLoopIdx, kDriveTrain.kTimeoutMs);

        left_BackTalon.config_kF(kDriveTrain.kPIDLoopIdx, kDriveTrain.kDistanceGains.kF, kDriveTrain.kTimeoutMs);
        left_BackTalon.config_kP(kDriveTrain.kPIDLoopIdx, kDriveTrain.kDistanceGains.kP, kDriveTrain.kTimeoutMs);
        left_BackTalon.config_kI(kDriveTrain.kPIDLoopIdx, kDriveTrain.kDistanceGains.kI, kDriveTrain.kTimeoutMs);
        left_BackTalon.config_kD(kDriveTrain.kPIDLoopIdx, kDriveTrain.kDistanceGains.kD, kDriveTrain.kTimeoutMs);

        // mot_rightFrontDrive.getStatorCurrent();

        /**
         * ------------------ DIFFERENTIAL DRIVE ------------------
         * Decleration:
         */
        m_drive = new DifferentialDrive(left_FrontTalon, right_FrontTalon);

        /**
         * ------------------ SENSORS ------------------
         * Declerations:
         */

        // Time of flight
        tof = new TimeOfFlight(Constants.kDriveTrain.CAN_TOF_SENSOR);
        tof.setRangingMode(RangingMode.Medium, 1000);

        // ssl_gear = new Solenoid(PneumaticsModuleType.CTREPCM, 0);

        // SmartDashboard.putData(ssl_gear);

        // gyro_pigeon = new WPI_Pigeon2(Constants.kDriveTrain.CANPigeon);
        // dOdometry = new DifferentialDriveOdometry(gyro_pigeon.getRotation2d());

        pSlider = Shuffleboard.getTab("PID_TUNING").add("P_VAL", Constants.kDriveTrain.kP)
                .withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("min", 0, "max", 10)).getEntry();

        iSlider = Shuffleboard.getTab("PID_TUNING").add("I_VAL", Constants.kDriveTrain.kI)
                .withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("min", 0, "max", 10)).getEntry();

        dSlider = Shuffleboard.getTab("PID_TUNING").add("D_VAL", Constants.kDriveTrain.kD)
                .withWidget(BuiltInWidgets.kTextView).withProperties(Map.of("min", 0, "max", 1000)).getEntry();

        SmartDashboard.putNumber("Mode", right_FrontTalon.getControlMode().value);
    }

    public void resetMotorConfigs() {
        right_FrontTalon.configFactoryDefault();
        right_BackTalon.configFactoryDefault();

        right_FrontTalon.configPeakCurrentLimit(kDriveTrain.MOTOR_CURRENT_LIMIT);
        right_BackTalon.configPeakCurrentLimit(kDriveTrain.MOTOR_CURRENT_LIMIT);

        right_FrontTalon.setNeutralMode(NeutralMode.Brake);
        right_BackTalon.setNeutralMode(NeutralMode.Brake);

        right_FrontTalon.configNominalOutputForward(0, kDriveTrain.kTimeoutMs);
        right_FrontTalon.configNominalOutputReverse(0, kDriveTrain.kTimeoutMs);
        right_FrontTalon.configPeakOutputForward(1, kDriveTrain.kTimeoutMs);
        right_FrontTalon.configPeakOutputReverse(-1, kDriveTrain.kTimeoutMs);
        right_FrontTalon.configAllowableClosedloopError(0, kDriveTrain.kPIDLoopIdx, kDriveTrain.kTimeoutMs);

        right_FrontTalon.config_kF(kDriveTrain.kPIDLoopIdx, kDriveTrain.kDistanceGains.kF, kDriveTrain.kTimeoutMs);
        right_FrontTalon.config_kP(kDriveTrain.kPIDLoopIdx, kDriveTrain.kDistanceGains.kP, kDriveTrain.kTimeoutMs);
        right_FrontTalon.config_kI(kDriveTrain.kPIDLoopIdx, kDriveTrain.kDistanceGains.kI, kDriveTrain.kTimeoutMs);
        right_FrontTalon.config_kD(kDriveTrain.kPIDLoopIdx, kDriveTrain.kDistanceGains.kD, kDriveTrain.kTimeoutMs);

        right_BackTalon.configNominalOutputForward(0, kDriveTrain.kTimeoutMs);
        right_BackTalon.configNominalOutputReverse(0, kDriveTrain.kTimeoutMs);
        right_BackTalon.configPeakOutputForward(1, kDriveTrain.kTimeoutMs);
        right_BackTalon.configPeakOutputReverse(-1, kDriveTrain.kTimeoutMs);
        right_BackTalon.configAllowableClosedloopError(0, kDriveTrain.kPIDLoopIdx, kDriveTrain.kTimeoutMs);

        right_BackTalon.config_kF(kDriveTrain.kPIDLoopIdx, kDriveTrain.kDistanceGains.kF, kDriveTrain.kTimeoutMs);
        right_BackTalon.config_kP(kDriveTrain.kPIDLoopIdx, kDriveTrain.kDistanceGains.kP, kDriveTrain.kTimeoutMs);
        right_BackTalon.config_kI(kDriveTrain.kPIDLoopIdx, kDriveTrain.kDistanceGains.kI, kDriveTrain.kTimeoutMs);
        right_BackTalon.config_kD(kDriveTrain.kPIDLoopIdx, kDriveTrain.kDistanceGains.kD, kDriveTrain.kTimeoutMs);

        left_FrontTalon.configFactoryDefault();
        left_BackTalon.configFactoryDefault();

        left_FrontTalon.configPeakCurrentLimit(kDriveTrain.MOTOR_CURRENT_LIMIT);
        left_BackTalon.configPeakCurrentLimit(kDriveTrain.MOTOR_CURRENT_LIMIT);

        left_FrontTalon.setNeutralMode(NeutralMode.Brake);
        left_BackTalon.setNeutralMode(NeutralMode.Brake);

        left_FrontTalon.configNominalOutputForward(0, kDriveTrain.kTimeoutMs);
        left_FrontTalon.configNominalOutputReverse(0, kDriveTrain.kTimeoutMs);
        left_FrontTalon.configPeakOutputForward(1, kDriveTrain.kTimeoutMs);
        left_FrontTalon.configPeakOutputReverse(-1, kDriveTrain.kTimeoutMs);
        left_FrontTalon.configAllowableClosedloopError(0, kDriveTrain.kPIDLoopIdx, kDriveTrain.kTimeoutMs);

        left_FrontTalon.config_kF(kDriveTrain.kPIDLoopIdx, kDriveTrain.kDistanceGains.kF, kDriveTrain.kTimeoutMs);
        left_FrontTalon.config_kP(kDriveTrain.kPIDLoopIdx, kDriveTrain.kDistanceGains.kP, kDriveTrain.kTimeoutMs);
        left_FrontTalon.config_kI(kDriveTrain.kPIDLoopIdx, kDriveTrain.kDistanceGains.kI, kDriveTrain.kTimeoutMs);
        left_FrontTalon.config_kD(kDriveTrain.kPIDLoopIdx, kDriveTrain.kDistanceGains.kD, kDriveTrain.kTimeoutMs);

        left_BackTalon.configNominalOutputForward(0, kDriveTrain.kTimeoutMs);
        left_BackTalon.configNominalOutputReverse(0, kDriveTrain.kTimeoutMs);
        left_BackTalon.configPeakOutputForward(1, kDriveTrain.kTimeoutMs);
        left_BackTalon.configPeakOutputReverse(-1, kDriveTrain.kTimeoutMs);
        left_BackTalon.configAllowableClosedloopError(0, kDriveTrain.kPIDLoopIdx, kDriveTrain.kTimeoutMs);

        left_BackTalon.config_kF(kDriveTrain.kPIDLoopIdx, kDriveTrain.kDistanceGains.kF, kDriveTrain.kTimeoutMs);
        left_BackTalon.config_kP(kDriveTrain.kPIDLoopIdx, kDriveTrain.kDistanceGains.kP, kDriveTrain.kTimeoutMs);
        left_BackTalon.config_kI(kDriveTrain.kPIDLoopIdx, kDriveTrain.kDistanceGains.kI, kDriveTrain.kTimeoutMs);
        left_BackTalon.config_kD(kDriveTrain.kPIDLoopIdx, kDriveTrain.kDistanceGains.kD, kDriveTrain.kTimeoutMs);
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

        SmartDashboard.putNumber("Distance", getDistance());
        SmartDashboard.putBoolean("Distance Valid", getValidDistance());

        SmartDashboard.putString("Drive Mode", getDriveModeName());

        // dOdometry.update(gyro_pigeon.getRotation2d(),
        // mot_leftFrontDrive.getSelectedSensorPosition(),
        // mot_rightFrontDrive.getSelectedSensorPosition());

        SmartDashboard.putNumber("P", pSlider.getDouble(Constants.kDriveTrain.kP));
        SmartDashboard.putNumber("I", iSlider.getDouble(Constants.kDriveTrain.kI));
        SmartDashboard.putNumber("D", dSlider.getDouble(Constants.kDriveTrain.kD));
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
        SmartDashboard.putNumber("MOT_RB_VEL", right_BackTalon.getSelectedSensorVelocity());
        SmartDashboard.putNumber("MOT_RB_POS", right_BackTalon.getSelectedSensorPosition());

        SmartDashboard.putNumber("MOT_LB_VEL", left_BackTalon.getSelectedSensorVelocity());
        SmartDashboard.putNumber("MOT_LB_POS", left_BackTalon.getSelectedSensorPosition());

        SmartDashboard.putNumber("MOT_RB_VOL", right_BackTalon.getStatorCurrent());
        SmartDashboard.putNumber("MOT_RF_VOL", right_FrontTalon.getStatorCurrent());

        SmartDashboard.putNumber("MOT_LB_VOL", left_BackTalon.getStatorCurrent());
        SmartDashboard.putNumber("MOT_LF_VOL", left_FrontTalon.getStatorCurrent());
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

    public double getDistance() {
        return tof.getRange() / 1000;
    }

    public boolean getValidDistance() {
        return tof.isRangeValid();
    }

    public void zeroEncoders() {
        right_FrontTalon.setSelectedSensorPosition(0);
        right_BackTalon.setSelectedSensorPosition(0);

        left_FrontTalon.setSelectedSensorPosition(0);
        left_BackTalon.setSelectedSensorPosition(0);

        right_FrontTalon.setSelectedSensorPosition(0, Constants.kDriveTrain.kPIDLoopIdx, Constants.kDriveTrain.kTimeoutMs);
        right_BackTalon.setSelectedSensorPosition(0, Constants.kDriveTrain.kPIDLoopIdx, Constants.kDriveTrain.kTimeoutMs);

        left_FrontTalon.setSelectedSensorPosition(0, Constants.kDriveTrain.kPIDLoopIdx, Constants.kDriveTrain.kTimeoutMs);
        left_BackTalon.setSelectedSensorPosition(0, Constants.kDriveTrain.kPIDLoopIdx, Constants.kDriveTrain.kTimeoutMs);
    }

    public void setDefaultControlMode() {
        right_FrontTalon.set(TalonSRXControlMode.PercentOutput, 0);
        right_FrontTalon.setInverted(false);

        right_BackTalon.follow(right_FrontTalon);
        right_BackTalon.setInverted(InvertType.FollowMaster);

        left_FrontTalon.set(TalonSRXControlMode.PercentOutput, 0);
        left_FrontTalon.setInverted(true);
        
        left_BackTalon.follow(left_FrontTalon);
        left_BackTalon.setInverted(InvertType.FollowMaster);
    }

    public void setControlMode(TalonSRXControlMode mode, double setpoint) {
        right_FrontTalon.set(mode, setpoint);
        right_BackTalon.set(mode, setpoint);

        left_BackTalon.set(mode, setpoint);
        left_FrontTalon.set(mode, setpoint);

        right_FrontTalon.config_kF(kDriveTrain.kPIDLoopIdx, kDriveTrain.kDistanceGains.kF, kDriveTrain.kTimeoutMs);
        right_FrontTalon.config_kP(kDriveTrain.kPIDLoopIdx, pSlider.getDouble(Constants.kDriveTrain.kDistanceGains.kP),
                kDriveTrain.kTimeoutMs);
        right_FrontTalon.config_kI(kDriveTrain.kPIDLoopIdx, iSlider.getDouble(Constants.kDriveTrain.kDistanceGains.kI),
                kDriveTrain.kTimeoutMs);
        right_FrontTalon.config_kD(kDriveTrain.kPIDLoopIdx, dSlider.getDouble(Constants.kDriveTrain.kDistanceGains.kD),
                kDriveTrain.kTimeoutMs);

        right_BackTalon.config_kF(kDriveTrain.kPIDLoopIdx, kDriveTrain.kDistanceGains.kF, kDriveTrain.kTimeoutMs);
        right_BackTalon.config_kP(kDriveTrain.kPIDLoopIdx, pSlider.getDouble(Constants.kDriveTrain.kDistanceGains.kP),
                kDriveTrain.kTimeoutMs);
        right_BackTalon.config_kI(kDriveTrain.kPIDLoopIdx, iSlider.getDouble(Constants.kDriveTrain.kDistanceGains.kI),
                kDriveTrain.kTimeoutMs);
        right_BackTalon.config_kD(kDriveTrain.kPIDLoopIdx, dSlider.getDouble(Constants.kDriveTrain.kDistanceGains.kD),
                kDriveTrain.kTimeoutMs);

        left_FrontTalon.config_kF(kDriveTrain.kPIDLoopIdx, kDriveTrain.kDistanceGains.kF, kDriveTrain.kTimeoutMs);
        left_FrontTalon.config_kP(kDriveTrain.kPIDLoopIdx, pSlider.getDouble(Constants.kDriveTrain.kDistanceGains.kP),
                kDriveTrain.kTimeoutMs);
        left_FrontTalon.config_kI(kDriveTrain.kPIDLoopIdx, iSlider.getDouble(Constants.kDriveTrain.kDistanceGains.kI),
                kDriveTrain.kTimeoutMs);
        left_FrontTalon.config_kD(kDriveTrain.kPIDLoopIdx, dSlider.getDouble(Constants.kDriveTrain.kDistanceGains.kD),
                kDriveTrain.kTimeoutMs);

        left_BackTalon.config_kF(kDriveTrain.kPIDLoopIdx, kDriveTrain.kDistanceGains.kF, kDriveTrain.kTimeoutMs);
        left_BackTalon.config_kP(kDriveTrain.kPIDLoopIdx, pSlider.getDouble(Constants.kDriveTrain.kDistanceGains.kP),
                kDriveTrain.kTimeoutMs);
        left_BackTalon.config_kI(kDriveTrain.kPIDLoopIdx, iSlider.getDouble(Constants.kDriveTrain.kDistanceGains.kI),
                kDriveTrain.kTimeoutMs);
        left_BackTalon.config_kD(kDriveTrain.kPIDLoopIdx, dSlider.getDouble(Constants.kDriveTrain.kDistanceGains.kD),
                kDriveTrain.kTimeoutMs);

        System.out.println("P: " + pSlider.getDouble(Constants.kDriveTrain.kDistanceGains.kP));
        System.out.println("I: " + iSlider.getDouble(Constants.kDriveTrain.kDistanceGains.kI));
        System.out.println("D: " + dSlider.getDouble(Constants.kDriveTrain.kDistanceGains.kD));

    }

    /**
     * @return the average position of all four encoders
     * 
     */
    public double getEncoderPosition() {
        return (getEncoderPositionLeft() + getEncoderPositionRight()) / 2;
    }

    /**
     * @return the average position of the left encoders
     * 
     */
    public double getEncoderPositionLeft() {
        return (left_BackTalon.getSelectedSensorPosition());
    }

    /**
     * @return the average position of the right encoders
     * 
     */
    public double getEncoderPositionRight() {
        return (right_BackTalon.getSelectedSensorPosition());
    }

    // /**
    // * Returns the current wheel speeds of the robot in m/s.
    // *
    // * @return The current wheel speeds.
    // */
    // public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    // return new
    // DifferentialDriveWheelSpeeds(left_FrontTalon.getSelectedSensorVelocity() *
    // 10,
    // right_FrontTalon.getSelectedSensorVelocity() * 10);
    // }

    // /**
    // * This method will reset the encoders
    // */
    // public void resetEncoders() {
    // left_FrontTalon.configFactoryDefault();
    // right_FrontTalon.configFactoryDefault();
    // }

    // public void resetOdometery() {
    // resetEncoders();
    // }
}
