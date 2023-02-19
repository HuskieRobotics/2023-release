package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.team3061.RobotConfig;
import frc.lib.team6328.util.Alert;
import frc.lib.team6328.util.Alert.AlertType;
import frc.robot.FieldConstants;
import frc.robot.operator_interface.OISelector;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.drivetrain.Drivetrain;
import org.littletonrobotics.junction.Logger;

public class MoveToGrid extends CommandBase {
  private Drivetrain drivetrain;
  private OperatorInterface oi;
  private PathPlannerTrajectory trajectory;
  private PPSwerveControllerCommand ppSwerveControllerCommand;
  private double minPathTraversalTime;
  private Alert noPathAlert = new Alert("No path between start and end pose", AlertType.WARNING);

  public MoveToGrid(Drivetrain subsystem, double minPathTraversalTime) {
    // no requirements for movetogrid, drivetrain for ppswerve
    this.drivetrain = subsystem;
    this.minPathTraversalTime = minPathTraversalTime;
    addRequirements(this.drivetrain);

    FieldConstants.COMMUNITY_REGION_1.addNeighbor(
        FieldConstants.COMMUNITY_REGION_2, FieldConstants.REGION_1_2_TRANSITION_POINT);
    FieldConstants.COMMUNITY_REGION_2.addNeighbor(
        FieldConstants.COMMUNITY_REGION_1, FieldConstants.REGION_2_1_TRANSITION_POINT);
    FieldConstants.COMMUNITY_REGION_1.addNeighbor(
        FieldConstants.COMMUNITY_REGION_3, FieldConstants.REGION_1_3_TRANSITION_POINT);
    FieldConstants.COMMUNITY_REGION_3.addNeighbor(
        FieldConstants.COMMUNITY_REGION_1, FieldConstants.REGION_3_1_TRANSITION_POINT);
  }

  public MoveToGrid(Drivetrain subsystem) {
    this(subsystem, 0.0);
  }

  public Pose2d endPose() {
    // When both OI boolean values of the switches are false, the switch is in the middle position
    boolean gridSwitchValue1 = this.oi.getHybridLeftMiddleGridButton().getAsBoolean();
    boolean gridSwitchValue2 = this.oi.getHybridMiddleRightGridButton().getAsBoolean();
    boolean colSwitchValue1 = this.oi.getHybridLeftMiddleColumnButton().getAsBoolean();
    boolean colSwitchValue2 = this.oi.getHybridMiddleRightColumnButton().getAsBoolean();

    if (gridSwitchValue1) {
      if (colSwitchValue1) {
        return FieldConstants.GRID_1_NODE_1;
      } else if (colSwitchValue2) {
        return FieldConstants.GRID_1_NODE_2;
      } else {
        return FieldConstants.GRID_1_NODE_3;
      }
    } else if (gridSwitchValue2) {
      if (colSwitchValue1) {
        return FieldConstants.GRID_2_NODE_1;
      } else if (colSwitchValue2) {
        return FieldConstants.GRID_2_NODE_2;
      } else {
        return FieldConstants.GRID_2_NODE_3;
      }
    } else {
      if (colSwitchValue1) {
        return FieldConstants.GRID_3_NODE_1;
      } else if (colSwitchValue2) {
        return FieldConstants.GRID_3_NODE_2;
      } else {
        return FieldConstants.GRID_3_NODE_3;
      }
    }
  }

  public void initialize() {
    this.oi = OISelector.getOperatorInterface();

    // reset the theta controller such that old accumulated ID values aren't used with the new path
    //      this doesn't matter if only the P value is non-zero, which is the current behavior
    this.drivetrain.getAutoXController().reset();
    this.drivetrain.getAutoYController().reset();
    this.drivetrain.getAutoThetaController().reset();

    Pose2d endPose = endPose();
    double distance = endPose.minus(this.drivetrain.getPose()).getTranslation().getNorm();
    double maxVelocity = RobotConfig.getInstance().getAutoMaxSpeed();
    if (this.minPathTraversalTime != 0) {
      maxVelocity = distance / this.minPathTraversalTime;
    }
    maxVelocity = Math.min(maxVelocity, RobotConfig.getInstance().getAutoMaxSpeed());

    this.trajectory =
        FieldConstants.COMMUNITY_ZONE.makePath(
            this.drivetrain.getPose(),
            endPose(),
            new PathConstraints(maxVelocity, RobotConfig.getInstance().getAutoMaxAcceleration()));

    noPathAlert.set(this.trajectory == null);

    if (this.trajectory != null) {
      this.ppSwerveControllerCommand =
          new PPSwerveControllerCommand(
              this.trajectory,
              this.drivetrain::getPose,
              RobotConfig.getInstance().getSwerveDriveKinematics(),
              this.drivetrain.getAutoXController(),
              this.drivetrain.getAutoYController(),
              this.drivetrain.getAutoThetaController(),
              this.drivetrain::setSwerveModuleStates);
      this.ppSwerveControllerCommand.initialize();

      Logger.getInstance().recordOutput("Odometry/trajectory", trajectory);
    }
    // FIXME: add alert that no path was found
  }

  public void execute() {
    if (this.ppSwerveControllerCommand != null) {
      this.ppSwerveControllerCommand.execute();
    }
  }

  public boolean isFinished() {
    return this.trajectory == null || ppSwerveControllerCommand.isFinished();
  }

  public void end(boolean interrupted) {
    if (this.ppSwerveControllerCommand != null) {
      this.ppSwerveControllerCommand.end(interrupted);
    }

    if (this.trajectory != null) {
      this.drivetrain.stop();
    }
    this.trajectory = null;
    this.ppSwerveControllerCommand = null;
  }

  public double getTotalTimeSeconds() {
    return this.trajectory.getTotalTimeSeconds();
  }
}
