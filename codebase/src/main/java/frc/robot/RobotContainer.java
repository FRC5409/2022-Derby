// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.FastGear;
import frc.robot.commands.SimpleDriveAuto;
import frc.robot.commands.SlowGear;
import frc.robot.commands.SwitchDrive;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Pneumatics;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Define main joystick
  private final XboxController m_joystick_main; // = new XboxController(0);
  private final JoystickButton but_main_A, but_main_B, but_main_X, but_main_Y, but_main_LBumper, but_main_RBumper,
      but_main_LAnalog, but_main_RAnalog, but_main_Back, but_main_Start;

  private final Pneumatics sys_Pneumatics;

  // Define drive train subsystem
  private final DriveTrain sys_DriveTrain;

  // Define default command
  private final SwitchDrive cmd_defaultDrive;

  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Init controller
    m_joystick_main = new XboxController(0);

    // Init button binds
    but_main_A = new JoystickButton(m_joystick_main, XboxController.Button.kA.value);
    but_main_B = new JoystickButton(m_joystick_main, XboxController.Button.kB.value);
    but_main_X = new JoystickButton(m_joystick_main, XboxController.Button.kX.value);
    but_main_Y = new JoystickButton(m_joystick_main, XboxController.Button.kY.value);
    but_main_LBumper = new JoystickButton(m_joystick_main, XboxController.Button.kBumperLeft.value);
    but_main_RBumper = new JoystickButton(m_joystick_main, XboxController.Button.kBumperRight.value);
    but_main_LAnalog = new JoystickButton(m_joystick_main, XboxController.Button.kStickLeft.value);
    but_main_RAnalog = new JoystickButton(m_joystick_main, XboxController.Button.kStickRight.value);
    but_main_Back = new JoystickButton(m_joystick_main, XboxController.Button.kBack.value);
    but_main_Start = new JoystickButton(m_joystick_main, XboxController.Button.kStart.value);

    sys_Pneumatics = new Pneumatics();

    // Init sub systems
    sys_DriveTrain = new DriveTrain();

    // Init commands
    cmd_defaultDrive = new SwitchDrive(sys_DriveTrain, m_joystick_main);

    // Configure the button bindings
    configureButtonBindings();

    sys_DriveTrain.setDefaultCommand(cmd_defaultDrive);
    System.out.println(RobotBase.isReal() ? "Entering controlled robot." : "Entering simulated environment.");
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Bind left analog to switch to previous drive mode
    but_main_LAnalog.whenPressed(() -> sys_DriveTrain.previousDriveMode());

    // Bind right analog to switch to next drive mode
    but_main_RAnalog.whenPressed(() -> sys_DriveTrain.nextDriveMode());

    but_main_RBumper.whenPressed(() -> new FastGear(sys_DriveTrain));
    but_main_RBumper.whenReleased(() -> new SlowGear(sys_DriveTrain));

    but_main_Start.whenPressed(() -> sys_Pneumatics.turnOff());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new SimpleDriveAuto(sys_DriveTrain);
  }
}
