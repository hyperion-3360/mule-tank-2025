package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

import java.io.IOException;
import java.util.function.DoubleSupplier;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/***
 * HISTORY:
 * 2024-12-16:  - camera vision2 recalibrated 640x480 50 images instead of 12
 * 2024-12-16:  - camera vision2 calibrated 640x480 (Microsoft LifeCam HD-3000)
 * 2024-12-09:  - Need to calibrate the camera possibly try with something different than onboard picam if
 *              the detection distance isn't enough.
 *              - Once calibrated and the correct distance is obtained, use the LED module to signal the
 *              driver he's within shooting distance then used the elevator match the distance and angle
 *              - then integrate the swerve to allow the driver to control translate and strafe
 */

public class apriltaglock extends Command {
  private PhotonCamera m_camera;
  private DriveTrain m_driveTrain;
  // private Elevator m_elevator;
  private DoubleSupplier m_forwardSupplier;
  private DoubleSupplier m_rotationSupplier;
  // private LEDs m_led;
  private int m_alliance_index;
  private AprilTagFieldLayout m_aprilTagFieldLayout;
  private PIDController controller = new PIDController(.03, 0.015, 0);
  private LinearFilter m_filter = LinearFilter.singlePoleIIR(0.1, 0.02);
  /**
   * Command to keep the aiming at the speaker while keeping the robot in motion
   *
   * @param s_swerve swerve submodule instance
   * @param s_led led submodule instance
   * @param forwardSupplier forward or backward
   * @param rotationSupplier rotation
   * @param camera photon camera instance
   */
  public apriltaglock(
      // LEDs s_led,
      DriveTrain driveTrain,
      DoubleSupplier forwardSupplier,
      DoubleSupplier rotationSupplier,
      PhotonCamera camera) {
    this.m_camera = camera;

    try {
      m_aprilTagFieldLayout =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    } catch (IOException e) {
      m_aprilTagFieldLayout = null;
    }

    if (m_aprilTagFieldLayout != null) {

      this.m_driveTrain = driveTrain;
      // this.m_led = s_led;

      // find out the current alliance with fail safe
      m_alliance_index = 0;
      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) m_alliance_index = alliance.get() == Alliance.Red ? 0 : 1;

      addRequirements(m_driveTrain);
      // addRequirements(s_led);

      this.m_forwardSupplier = forwardSupplier;
      this.m_rotationSupplier = rotationSupplier;
    }
  }

  @Override
  public void initialize() {
    controller.reset();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    // m_swerve.drive(new Translation2d(0, 0), 0, false, true);
    // m_elevator.extendTheElevator(Elevator.elevatorHeight.LOW);
    // m_led.setState(LEDs.State.IDLE);
  }

  @Override
  public void execute() {
    double forwardVal = 0;
    double rotationVal = 0;

    if (m_forwardSupplier != null)
      forwardVal = MathUtil.applyDeadband(m_forwardSupplier.getAsDouble(), Constants.stickDeadband);

    if (m_rotationSupplier != null)
      rotationVal = MathUtil.applyDeadband(m_rotationSupplier.getAsDouble(), Constants.stickDeadband);

    PhotonTrackedTarget target = null;

    // Vision-alignment mode
    // Query the latest result from PhotonVision
    PhotonPipelineResult result = null;

    if (m_camera != null)
      result = m_camera.getLatestResult();

    if (result != null && result.hasTargets()) {
      for (var t : result.getTargets()) {
        if (t.getFiducialId() == 10) {
          // if (t.getFiducialId() ==
          // Constants.VisionConstants.kSpeakerIndex[m_alliance_index]) {
          target = t;
          break;
        }
      }
    }

    // still possible that the visible ids are not the ones we're interested in
    if (target != null) {
      SmartDashboard.putString("Target id", Integer.toString(target.getFiducialId()));

      var targetHeight = m_aprilTagFieldLayout.getTagPose(target.getFiducialId()).get().getZ();
      SmartDashboard.putString("Target height", Double.toString(targetHeight));

      var targetPitch = target.getPitch();
      var targetYaw = target.getYaw();
      var targetYawFiltered = m_filter.calculate(targetYaw);
      var bestPose = target.getBestCameraToTarget();
      var targetX = bestPose.getX();
      var targetY = bestPose.getY();
      var targetZ = bestPose.getZ();
      var distanceToTarget = Math.sqrt(targetX * targetX + targetY * targetY + targetZ * targetZ);

      SmartDashboard.putString("Target pitch", Double.toString(targetPitch));
      SmartDashboard.putNumber("Target yaw", targetYaw);
      SmartDashboard.putNumber("Target Yaw Filtered", targetYawFiltered);

      SmartDashboard.putString("Target X", Double.toString(targetX));
      SmartDashboard.putString("Target Y", Double.toString(targetY));
      SmartDashboard.putString("Target Z", Double.toString(targetZ));
      SmartDashboard.putNumber("rotationVal", rotationVal);

      SmartDashboard.putString("Target distance", Double.toString(distanceToTarget));

      rotationVal = -controller.calculate(targetYawFiltered, 0);
      SmartDashboard.putNumber("rotationVal", rotationVal);
    }
    else{
      controller.reset();
    }

    // Command drivetrain motors based on target speeds

    m_driveTrain.driveArcade(forwardVal, rotationVal, true);
  }
}
