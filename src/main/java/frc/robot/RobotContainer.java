// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;
import static frc.robot.subsystems.drivetrain.DrivetrainConstants.*;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.team3061.gyro.GyroIO;
import frc.lib.team3061.gyro.GyroIOPigeon2;
import frc.lib.team3061.pneumatics.Pneumatics;
import frc.lib.team3061.pneumatics.PneumaticsIO;
import frc.lib.team3061.pneumatics.PneumaticsIORev;
import frc.lib.team3061.swerve.SwerveModule;
import frc.lib.team3061.swerve.SwerveModuleIO;
import frc.lib.team3061.swerve.SwerveModuleIOSim;
import frc.lib.team3061.swerve.SwerveModuleIOTalonFX;
import frc.lib.team3061.vision.Vision;
import frc.lib.team3061.vision.VisionConstants;
import frc.lib.team3061.vision.VisionIO;
import frc.lib.team3061.vision.VisionIOPhotonVision;
import frc.lib.team3061.vision.VisionIOSim;
import frc.robot.Constants.Mode;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.FeedForwardCharacterization.FeedForwardCharacterizationData;
import frc.robot.commands.FollowPath;
import frc.robot.commands.TeleopSwerve;
import frc.robot.operator_interface.OISelector;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.drivetrain.Drivetrain;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private OperatorInterface oi = new OperatorInterface() {};

  private Drivetrain drivetrain;

  // use AdvantageKit's LoggedDashboardChooser instead of SendableChooser to ensure accurate logging
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Routine");

  // RobotContainer singleton
  private static RobotContainer robotContainer = new RobotContainer();

  /** Create the container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // create real, simulated, or replay subsystems based on the mode and robot specified
    if (Constants.getMode() != Mode.REPLAY) {
      switch (Constants.getRobot()) {
        case ROBOT_2022_PRESEASON:
          {
            GyroIO gyro = new GyroIOPigeon2(PIGEON_ID);

            SwerveModule flModule =
                new SwerveModule(
                    new SwerveModuleIOTalonFX(
                        0,
                        FRONT_LEFT_MODULE_DRIVE_MOTOR,
                        FRONT_LEFT_MODULE_STEER_MOTOR,
                        FRONT_LEFT_MODULE_STEER_ENCODER,
                        FRONT_LEFT_MODULE_STEER_OFFSET),
                    0,
                    MAX_VELOCITY_METERS_PER_SECOND);

            SwerveModule frModule =
                new SwerveModule(
                    new SwerveModuleIOTalonFX(
                        1,
                        FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                        FRONT_RIGHT_MODULE_STEER_MOTOR,
                        FRONT_RIGHT_MODULE_STEER_ENCODER,
                        FRONT_RIGHT_MODULE_STEER_OFFSET),
                    1,
                    MAX_VELOCITY_METERS_PER_SECOND);

            SwerveModule blModule =
                new SwerveModule(
                    new SwerveModuleIOTalonFX(
                        2,
                        BACK_LEFT_MODULE_DRIVE_MOTOR,
                        BACK_LEFT_MODULE_STEER_MOTOR,
                        BACK_LEFT_MODULE_STEER_ENCODER,
                        BACK_LEFT_MODULE_STEER_OFFSET),
                    2,
                    MAX_VELOCITY_METERS_PER_SECOND);

            SwerveModule brModule =
                new SwerveModule(
                    new SwerveModuleIOTalonFX(
                        3,
                        BACK_RIGHT_MODULE_DRIVE_MOTOR,
                        BACK_RIGHT_MODULE_STEER_MOTOR,
                        BACK_RIGHT_MODULE_STEER_ENCODER,
                        BACK_RIGHT_MODULE_STEER_OFFSET),
                    3,
                    MAX_VELOCITY_METERS_PER_SECOND);

            drivetrain = new Drivetrain(gyro, flModule, frModule, blModule, brModule);
            new Pneumatics(new PneumaticsIORev());
            new Vision(new VisionIOPhotonVision(CAMERA_NAME));
            break;
          }
        case ROBOT_SIMBOT:
          {
            SwerveModule flModule =
                new SwerveModule(new SwerveModuleIOSim(), 0, MAX_VELOCITY_METERS_PER_SECOND);

            SwerveModule frModule =
                new SwerveModule(new SwerveModuleIOSim(), 1, MAX_VELOCITY_METERS_PER_SECOND);

            SwerveModule blModule =
                new SwerveModule(new SwerveModuleIOSim(), 2, MAX_VELOCITY_METERS_PER_SECOND);

            SwerveModule brModule =
                new SwerveModule(new SwerveModuleIOSim(), 3, MAX_VELOCITY_METERS_PER_SECOND);
            drivetrain = new Drivetrain(new GyroIO() {}, flModule, frModule, blModule, brModule);
            new Pneumatics(new PneumaticsIO() {});
            AprilTagFieldLayout layout;
            try {
              layout = new AprilTagFieldLayout(VisionConstants.APRILTAG_FIELD_LAYOUT_PATH);
            } catch (IOException e) {
              layout = new AprilTagFieldLayout(new ArrayList<>(), 16.4592, 8.2296);
            }
            new Vision(
                new VisionIOSim(layout, drivetrain::getPose, VisionConstants.ROBOT_TO_CAMERA));

            break;
          }
        default:
          break;
      }

    } else {
      SwerveModule flModule =
          new SwerveModule(new SwerveModuleIO() {}, 0, MAX_VELOCITY_METERS_PER_SECOND);

      SwerveModule frModule =
          new SwerveModule(new SwerveModuleIO() {}, 1, MAX_VELOCITY_METERS_PER_SECOND);

      SwerveModule blModule =
          new SwerveModule(new SwerveModuleIO() {}, 2, MAX_VELOCITY_METERS_PER_SECOND);

      SwerveModule brModule =
          new SwerveModule(new SwerveModuleIO() {}, 3, MAX_VELOCITY_METERS_PER_SECOND);
      drivetrain = new Drivetrain(new GyroIO() {}, flModule, frModule, blModule, brModule);
      new Pneumatics(new PneumaticsIO() {});
      new Vision(new VisionIO() {});
    }

    // disable all telemetry in the LiveWindow to reduce the processing during each iteration
    LiveWindow.disableAllTelemetry();

    updateOI();

    configureAutoCommands();
  }

  /**
   * This method scans for any changes to the connected joystick. If anything changed, it creates
   * new OI objects and binds all of the buttons to commands.
   */
  public void updateOI() {
    if (!OISelector.didJoysticksChange()) {
      return;
    }

    CommandScheduler.getInstance().getActiveButtonLoop().clear();
    oi = OISelector.findOperatorInterface();

    /*
     * Set up the default command for the drivetrain. The joysticks' values map to percentage of the
     * maximum velocities. The velocities may be specified from either the robot's frame of
     * reference or the field's frame of reference. In the robot's frame of reference, the positive
     * x direction is forward; the positive y direction, left; position rotation, CCW. In the field
     * frame of reference, the origin of the field to the lower left corner (i.e., the corner of the
     * field to the driver's right). Zero degrees is away from the driver and increases in the CCW
     * direction. This is why the left joystick's y axis specifies the velocity in the x direction
     * and the left joystick's x axis specifies the velocity in the y direction.
     */
    drivetrain.setDefaultCommand(
        new TeleopSwerve(drivetrain, oi::getTranslateX, oi::getTranslateY, oi::getRotate));

    configureButtonBindings();
  }

  /**
   * Factory method to create the singleton robot container object.
   *
   * @return the singleton robot container object
   */
  public static RobotContainer getInstance() {
    return robotContainer;
  }

  /** Use this method to define your button->command mappings. */
  private void configureButtonBindings() {
    // field-relative toggle
    oi.getFieldRelativeButton()
        .toggleOnTrue(
            Commands.either(
                Commands.runOnce(drivetrain::disableFieldRelative, drivetrain),
                Commands.runOnce(drivetrain::enableFieldRelative, drivetrain),
                drivetrain::getFieldRelative));

    // reset gyro to 0 degrees
    oi.getResetGyroButton().onTrue(Commands.runOnce(drivetrain::zeroGyroscope, drivetrain));

    // x-stance
    oi.getXStanceButton().onTrue(Commands.runOnce(drivetrain::enableXstance, drivetrain));
    oi.getXStanceButton().onFalse(Commands.runOnce(drivetrain::disableXstance, drivetrain));
  }

  /** Use this method to define your commands for autonomous mode. */
  private void configureAutoCommands() {
    AUTO_EVENT_MAP.put("event1", Commands.print("passed marker 1"));
    AUTO_EVENT_MAP.put("event2", Commands.print("passed marker 2"));
    AUTO_EVENT_MAP.put("IntakeCone", Commands.print("passed marker: IntakeCone"));
    AUTO_EVENT_MAP.put("PrepareToScoreHigh", Commands.print("passed marker: PrepareToScoreHigh"));
    AUTO_EVENT_MAP.put("ScoreHigh", Commands.print("passed marker: ScoreHigh"));
    AUTO_EVENT_MAP.put("RetractArm", Commands.print("passed marker: RetractArm"));

    // build auto path commands
    List<PathPlannerTrajectory> auto1Paths =
        PathPlanner.loadPathGroup(
            "testPaths1",
            AUTO_MAX_SPEED_METERS_PER_SECOND,
            AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    Command autoTest =
        Commands.sequence(
            new FollowPathWithEvents(
                new FollowPath(auto1Paths.get(0), drivetrain, true),
                auto1Paths.get(0).getMarkers(),
                AUTO_EVENT_MAP),
            Commands.runOnce(drivetrain::enableXstance, drivetrain),
            Commands.waitSeconds(5.0),
            Commands.runOnce(drivetrain::disableXstance, drivetrain),
            new FollowPathWithEvents(
                new FollowPath(auto1Paths.get(1), drivetrain, false),
                auto1Paths.get(1).getMarkers(),
                AUTO_EVENT_MAP));

    // add commands to the auto chooser
    autoChooser.addDefaultOption("Do Nothing", new InstantCommand());

    // demonstration of PathPlanner path group with event markers
    autoChooser.addOption("Test Path", autoTest);

    // "auto" command for tuning the drive velocity PID
    autoChooser.addOption(
        "Drive Velocity Tuning",
        Commands.sequence(
            Commands.runOnce(drivetrain::disableFieldRelative, drivetrain),
            Commands.deadline(
                Commands.waitSeconds(5.0),
                Commands.run(() -> drivetrain.drive(1.5, 0.0, 0.0), drivetrain))));

    // "auto" command for characterizing the drivetrain
    autoChooser.addOption(
        "Drive Characterization",
        new FeedForwardCharacterization(
            drivetrain,
            true,
            new FeedForwardCharacterizationData("drive"),
            drivetrain::runCharacterizationVolts,
            drivetrain::getCharacterizationVelocity));

    // "auto" path for Blue-CableSide 2 Cone + Engage
    List<PathPlannerTrajectory> blueCableSide2ConeEngagePath =
        PathPlanner.loadPathGroup("Blue-CableSide 2 Cone + Engage", 2.0, 2.0);
    Command blueCableSide2ConeEngageCommand =
        new FollowPathWithEvents(
            new FollowPath(blueCableSide2ConeEngagePath.get(0), drivetrain, true),
            blueCableSide2ConeEngagePath.get(0).getMarkers(),
            AUTO_EVENT_MAP);
    autoChooser.addOption(
        "Blue-CableSide 2 Cone + Engage (test due to event markers, may fail)",
        blueCableSide2ConeEngageCommand);

    // "auto" path for Blue-CableSide 3 Cone
    List<PathPlannerTrajectory> blueCableSide3ConePath =
        PathPlanner.loadPathGroup("Blue-CableSide 3 Cone", 1.5, 1.5);
    Command blueCableSide3ConeCommand =
        new FollowPathWithEvents(
            new FollowPath(blueCableSide3ConePath.get(0), drivetrain, true),
            blueCableSide3ConePath.get(0).getMarkers(),
            AUTO_EVENT_MAP);
    autoChooser.addOption(
        "Blue-CableSide 3 Cone (over cable connector)", blueCableSide3ConeCommand);

    // "auto" path for Blue-CenterCable 2 Cone + Engage Copy
    List<PathPlannerTrajectory> blueCenterCable2ConeEngageCopyPath =
        PathPlanner.loadPathGroup("Blue-CenterCable 2 Cone + Engage Copy", 2.0, 2.0);
    Command blueCenterCable2ConeEngageCopyCommand =
        new FollowPathWithEvents(
            new FollowPath(blueCenterCable2ConeEngageCopyPath.get(0), drivetrain, true),
            blueCenterCable2ConeEngageCopyPath.get(0).getMarkers(),
            AUTO_EVENT_MAP);
    autoChooser.addOption(
        "Blue Center Cable 2 Cone Engage Copy Path", blueCenterCable2ConeEngageCopyCommand);

    // "auto" path for Blue-CenterLoad 2 Cone + Engage
    List<PathPlannerTrajectory> blueCenterLoad2ConeEngagePath =
        PathPlanner.loadPathGroup("Blue-CenterLoad 2 Cone + Engage", 2.0, 2.0);
    Command blueCenterLoad2ConeEngageCommand =
        new FollowPathWithEvents(
            new FollowPath(blueCenterLoad2ConeEngagePath.get(0), drivetrain, true),
            blueCenterLoad2ConeEngagePath.get(0).getMarkers(),
            AUTO_EVENT_MAP);
    autoChooser.addOption("Blue Center Load 2 Cone Engage Path", blueCenterLoad2ConeEngageCommand);

    // "auto" path for Blue-LoadingSide 2 Cone + Engage
    List<PathPlannerTrajectory> blueLoadingSide2ConeEngagePath =
        PathPlanner.loadPathGroup("Blue-LoadingSide 2 Cone + Engage", 2.0, 2.0);
    Command blueLoadingSide2ConeEngageCommand =
        new FollowPathWithEvents(
            new FollowPath(blueLoadingSide2ConeEngagePath.get(0), drivetrain, true),
            blueLoadingSide2ConeEngagePath.get(0).getMarkers(),
            AUTO_EVENT_MAP);
    autoChooser.addOption(
        "Blue Loading Side 2 Cone Engage Path", blueLoadingSide2ConeEngageCommand);

    // "auto" for Blue-LoadingSide 3 Cone
    List<PathPlannerTrajectory> blueLoadingSide3ConePath =
        PathPlanner.loadPathGroup("Blue-LoadingSide 3 Cone", 2.0, 2.0);
    Command blueLoadingSide3ConeCommand =
        new FollowPathWithEvents(
            new FollowPath(blueLoadingSide3ConePath.get(0), drivetrain, true),
            blueLoadingSide3ConePath.get(0).getMarkers(),
            AUTO_EVENT_MAP);
    autoChooser.addOption("Blue Loading Side 3 Cone Path", blueLoadingSide3ConeCommand);

    // "auto" path for Blue-LoadingSide 4 Cone
    List<PathPlannerTrajectory> blueLoadingSide4ConePath =
        PathPlanner.loadPathGroup("Blue-LoadingSide 4 Cone", 2.0, 2.0);
    Command blueLoadingSide4ConeCommand =
        new FollowPathWithEvents(
            new FollowPath(blueLoadingSide4ConePath.get(0), drivetrain, true),
            blueLoadingSide4ConePath.get(0).getMarkers(),
            AUTO_EVENT_MAP);
    autoChooser.addOption("Blue Loading Side 4 Cone Path", blueLoadingSide4ConeCommand);

    Shuffleboard.getTab("MAIN").add(autoChooser.getSendableChooser());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
