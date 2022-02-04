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
    private boolean useSmartDashboard;

    public MoveToPosition(DriveTrain _drive, double _setpoint){
        sys_drive = _drive;
        setpoint = _setpoint;
        useSmartDashboard = false;
    }

    @Override
    public void initialize(){
        SmartDashboard.putString("mode", "Position");
        
        setpoint = setpoint * Constants.kDriveTrain.METERS_TO_RSU;

        
        SmartDashboard.putNumber("setpoint", setpoint);
        sys_drive.zeroEncoders();
        sys_drive.setControlMode(TalonSRXControlMode.Position, setpoint);

    }
    

    @Override
    public void end(boolean interrupted){
        sys_drive.setControlMode(TalonSRXControlMode.PercentOutput, 0);
        SmartDashboard.putString("mode", "PercentOutput");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // System.out.println(Math.abs(drive.getEncoderPosition() - setpoint) / setpoint);
        return Math.abs(sys_drive.getEncoderPosition()) >= setpoint;
    }
}