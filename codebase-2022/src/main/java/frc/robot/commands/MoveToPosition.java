package frc.robot.commands;

import javax.swing.text.Position;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.kDriveTrain;
import frc.robot.subsystems.DriveTrain;

public class MoveToPosition extends CommandBase {

    private DriveTrain sys_drive;
    private double setpoint;
    private boolean passedSetPoint = false;

    public MoveToPosition(DriveTrain _drive, double _setpoint) {
        System.out.println("Constructing");
        sys_drive = _drive;
        setpoint = _setpoint;

        addRequirements(_drive);
        passedSetPoint = true;
    }

    public MoveToPosition(DriveTrain _drive) {
        System.out.println("Constructing 2");
        sys_drive = _drive;

        addRequirements(_drive);
        passedSetPoint = false;
    }

    @Override
    public void initialize() {
        System.out.println("----------------------------- start -----------------------------");
        sys_drive.setDefaultControlMode();
        sys_drive.zeroEncoders();

        SmartDashboard.putString("mode", "Position");
        SmartDashboard.putBoolean("Called", true);

        System.out.println(setpoint);

        if (!passedSetPoint)
            setpoint = sys_drive.getAvgDistance() - kDriveTrain.DISTANCE_TO_MID_RUN_FROM_WALL;
        
        setpoint = setpoint * Constants.kDriveTrain.METERS_TO_RSU;
        System.out.println(setpoint);

        sys_drive.setControlMode(TalonSRXControlMode.Position, setpoint);

        SmartDashboard.putNumber("setpoint", setpoint);

        SmartDashboard.putBoolean("PositionIsFinished", false);

    }

    @Override
    public void end(boolean interrupted) {
        // sys_drive.setControlMode(TalonSRXControlMode.PercentOutput, 0);
        sys_drive.setDefaultControlMode();
        sys_drive.zeroEncoders();

        SmartDashboard.putString("mode", "PercentOutput");
        SmartDashboard.putBoolean("PositionIsFinished", true);
        System.out.println("----------------------------- end -----------------------------");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // System.out.println(Math.abs(drive.getEncoderPosition() - setpoint) /
        // setpoint);
        return Math.abs(sys_drive.getEncoderPosition()) > Math.abs(setpoint);
    }
}