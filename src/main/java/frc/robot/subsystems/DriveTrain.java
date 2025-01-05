// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.DriveTrain.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class DriveTrain extends SubsystemBase {

  private final WPI_TalonSRX m_leftMaster = new WPI_TalonSRX(kLeftMasterId);
  private final WPI_TalonSRX m_rightMaster = new WPI_TalonSRX(kRightMasterId);
  private final WPI_TalonSRX m_leftFollower = new WPI_TalonSRX(kLeftFollowerId);
  private final WPI_TalonSRX m_rightFollower = new WPI_TalonSRX(kRightFollowerId);

  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMaster, m_rightMaster);

  /** Creates a new DriveTrain. */
  public DriveTrain() {


    m_leftMaster.configFactoryDefault();
    m_rightMaster.configFactoryDefault();
    m_leftFollower.configFactoryDefault();
    m_rightFollower.configFactoryDefault();

    m_leftMaster.setInverted(kLeftMasterInversion);
    m_leftFollower.setInverted(kLeftFollowerInversion);
    m_rightMaster.setInverted(kRightMasterInversion);
    m_rightFollower.setInverted(kRightFollowerInversion);

    m_leftFollower.follow(m_leftMaster);
    m_rightFollower.follow(m_rightMaster);



  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Open loop drive with arcade style inputs
   * @param x [-1 .. 1] positive is forward
   * @param z [-1 .. 1] positive is counter-clockwise
   */
  public void driveArcade(double x, double z, boolean squareInputs) {

    // Flip turn axis because arcadeDrive is not NWU compliant
    m_drive.arcadeDrive(x, -z, squareInputs);
  }
}
