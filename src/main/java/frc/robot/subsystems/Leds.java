// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Leds.*;

public class Leds extends SubsystemBase {

  private final Solenoid m_vcc = new Solenoid(PneumaticsModuleType.CTREPCM, kVccChannel);
  private final Solenoid m_red = new Solenoid(PneumaticsModuleType.CTREPCM, kRedChannel);
  private final Solenoid m_green = new Solenoid(PneumaticsModuleType.CTREPCM, kGreenChannel);
  private final Solenoid m_blue = new Solenoid(PneumaticsModuleType.CTREPCM, kBlueChannel);

  /** Creates a new Leds. */
  public Leds() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void off() {
    m_red.set(false);
    m_green.set(false);
    m_blue.set(false);
    m_vcc.set(false);
  }

  public void red() {
    m_red.set(true);
    m_green.set(false);
    m_blue.set(false);
    m_vcc.set(true);
  }

  public void green() {
    m_red.set(false);
    m_green.set(true);
    m_blue.set(false);
    m_vcc.set(true);
  }

  public void blue() {
    m_red.set(false);
    m_green.set(false);
    m_blue.set(true);
    m_vcc.set(true);
  }
}
