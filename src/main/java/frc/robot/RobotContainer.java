// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import org.photonvision.PhotonCamera;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.apriltaglock;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrain m_driveTrain = new DriveTrain();
  // private final Leds m_leds = new Leds();

  private final CommandXboxController m_controller = new CommandXboxController(0);

  private final DoubleSupplier translation = () -> m_controller.getLeftY();
  private final DoubleSupplier rotation = () -> -m_controller.getRightX();
  private final PhotonCamera camera = new PhotonCamera("aprilVision");

  private final apriltaglock APRILTAGLOCK = new apriltaglock(m_driveTrain, translation, rotation, camera, 10);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
     // Drive train default is arcade drive
     m_driveTrain.setDefaultCommand(new RunCommand(() -> {
      // Controller y axis is positive right, but z rotation is NWU (positive left)
      double x = m_controller.getLeftY();
      double z = -m_controller.getRightX();
      m_driveTrain.driveArcade(x, z, true);
    }, m_driveTrain));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    m_controller.a().whileTrue(APRILTAGLOCK);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // No autonomous for now
    return null;
  }
}
