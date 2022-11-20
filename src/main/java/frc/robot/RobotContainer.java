// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ArcadeDriveCmd;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ReverseDTCmd;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Robot subsystems.
  private final DriveTrain driveTrain;

  // Default auto subsystem.
  private final ExampleSubsystem exampleSubsystem;

  // Controller.
  private final XboxController xboxController;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Robot subsystems creation.
    driveTrain = new DriveTrain();
    exampleSubsystem = new ExampleSubsystem();
    
    // Controller setup.
    xboxController = new XboxController(Controller.kPort);
    configureButtonBindings();
    
    driveTrain.setDefaultCommand(new ArcadeDriveCmd(driveTrain, () -> -xboxController.getLeftY(), () -> xboxController.getRightX()));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(xboxController, XboxController.Button.kLeftStick.value)
      .whileActiveOnce(new ReverseDTCmd(driveTrain));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new ExampleCommand(exampleSubsystem);
  }
}
