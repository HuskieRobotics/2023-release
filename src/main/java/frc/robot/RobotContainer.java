// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;
import static frc.robot.FieldRegionConstants.*;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.team3061.RobotConfig;
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
import frc.lib.team6328.util.TunableNumber;
import frc.robot.Constants.Mode;
import frc.robot.commands.AutoBalanceNonStop;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.FeedForwardCharacterization.FeedForwardCharacterizationData;
import frc.robot.commands.FollowPath;
import frc.robot.commands.GrabGamePiece;
import frc.robot.commands.MoveToGrid;
import frc.robot.commands.MoveToLoadingZone;
import frc.robot.commands.ReleaseGamePiece;
import frc.robot.commands.TeleopSwerve;
import frc.robot.configs.MK4IRobotConfig;
import frc.robot.configs.SierraRobotConfig;
import frc.robot.configs.TestBoardConfig;
import frc.robot.operator_interface.OISelector;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.manipulator.ManipulatorIOSim;
import frc.robot.subsystems.manipulator.ManipulatorIOTalonFX;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private OperatorInterface oi = new OperatorInterface() {};
  private RobotConfig config;
  private Drivetrain drivetrain;
  private Alliance lastAlliance = DriverStation.Alliance.Invalid;
  private Manipulator manipulator;
  private Vision vision;

  private final TunableNumber squaringSpeed;
  private final TunableNumber squaringDuration;

  // use AdvantageKit's LoggedDashboardChooser instead of SendableChooser to ensure accurate logging
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Routine");

  // RobotContainer singleton
  private static RobotContainer robotContainer = new RobotContainer();
  private final Map<String, Command> autoEventMap = new HashMap<>();

  /** Create the container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    /*
     * IMPORTANT: The RobotConfig subclass object *must* be created before any other objects
     * that use it directly or indirectly. If this isn't done, a null pointer exception will result.
     */

    // create real, simulated, or replay subsystems based on the mode and robot specified
    if (Constants.getMode() != Mode.REPLAY) {
      switch (Constants.getRobot()) {
        case ROBOT_DEFAULT:
        case ROBOT_2023_MK4I:
        case ROBOT_2022_SIERRA:
          {
            // create the specific RobotConfig subclass instance first
            if (Constants.getRobot() == Constants.RobotType.ROBOT_2022_SIERRA) {
              config = new SierraRobotConfig();
            } else { // default to ROBOT_2023_MK4I
              config = new MK4IRobotConfig();
            }

            GyroIO gyro = new GyroIOPigeon2(config.getGyroCANID());

            int[] driveMotorCANIDs = config.getSwerveDriveMotorCANIDs();
            int[] steerMotorCANDIDs = config.getSwerveSteerMotorCANIDs();
            int[] steerEncoderCANDIDs = config.getSwerveSteerEncoderCANIDs();
            double[] steerOffsets = config.getSwerveSteerOffsets();
            SwerveModule flModule =
                new SwerveModule(
                    new SwerveModuleIOTalonFX(
                        0,
                        driveMotorCANIDs[0],
                        steerMotorCANDIDs[0],
                        steerEncoderCANDIDs[0],
                        steerOffsets[0]),
                    0,
                    config.getRobotMaxVelocity());

            SwerveModule frModule =
                new SwerveModule(
                    new SwerveModuleIOTalonFX(
                        1,
                        driveMotorCANIDs[1],
                        steerMotorCANDIDs[1],
                        steerEncoderCANDIDs[1],
                        steerOffsets[1]),
                    1,
                    config.getRobotMaxVelocity());

            SwerveModule blModule =
                new SwerveModule(
                    new SwerveModuleIOTalonFX(
                        2,
                        driveMotorCANIDs[2],
                        steerMotorCANDIDs[2],
                        steerEncoderCANDIDs[2],
                        steerOffsets[2]),
                    2,
                    config.getRobotMaxVelocity());

            SwerveModule brModule =
                new SwerveModule(
                    new SwerveModuleIOTalonFX(
                        3,
                        driveMotorCANIDs[3],
                        steerMotorCANDIDs[3],
                        steerEncoderCANDIDs[3],
                        steerOffsets[3]),
                    3,
                    config.getRobotMaxVelocity());

            drivetrain = new Drivetrain(gyro, flModule, frModule, blModule, brModule);

            manipulator = new Manipulator(new ManipulatorIOTalonFX());

            vision = new Vision(new VisionIOPhotonVision(config.getCameraName()));

            if (Constants.getRobot() == Constants.RobotType.ROBOT_2022_SIERRA) {
              new Pneumatics(new PneumaticsIORev());
            }

            break;
          }
        case TEST_BOARD:
          {
            // create the specific RobotConfig subclass instance first
            config = new TestBoardConfig();
            SwerveModule flModule =
                new SwerveModule(new SwerveModuleIOSim(), 0, config.getRobotMaxVelocity());

            SwerveModule frModule =
                new SwerveModule(new SwerveModuleIOSim(), 1, config.getRobotMaxVelocity());

            SwerveModule blModule =
                new SwerveModule(new SwerveModuleIOSim(), 2, config.getRobotMaxVelocity());

            SwerveModule brModule =
                new SwerveModule(new SwerveModuleIOSim(), 3, config.getRobotMaxVelocity());
            drivetrain = new Drivetrain(new GyroIO() {}, flModule, frModule, blModule, brModule);
            manipulator = new Manipulator(new ManipulatorIOTalonFX());
            break;
          }
        case ROBOT_SIMBOT:
          {
            config = new MK4IRobotConfig();
            SwerveModule flModule =
                new SwerveModule(new SwerveModuleIOSim(), 0, config.getRobotMaxVelocity());

            SwerveModule frModule =
                new SwerveModule(new SwerveModuleIOSim(), 1, config.getRobotMaxVelocity());

            SwerveModule blModule =
                new SwerveModule(new SwerveModuleIOSim(), 2, config.getRobotMaxVelocity());

            SwerveModule brModule =
                new SwerveModule(new SwerveModuleIOSim(), 3, config.getRobotMaxVelocity());
            drivetrain = new Drivetrain(new GyroIO() {}, flModule, frModule, blModule, brModule);
            manipulator = new Manipulator(new ManipulatorIOSim());
            new Pneumatics(new PneumaticsIO() {});
            AprilTagFieldLayout layout;
            try {
              layout = new AprilTagFieldLayout(VisionConstants.APRILTAG_FIELD_LAYOUT_PATH);
            } catch (IOException e) {
              layout = new AprilTagFieldLayout(new ArrayList<>(), 16.4592, 8.2296);
            }
            vision =
                new Vision(
                    new VisionIOSim(
                        layout,
                        drivetrain::getPose,
                        RobotConfig.getInstance().getRobotToCameraTransform()));

            break;
          }
        default:
          break;
      }

    } else {
      SwerveModule flModule =
          new SwerveModule(new SwerveModuleIO() {}, 0, config.getRobotMaxVelocity());

      SwerveModule frModule =
          new SwerveModule(new SwerveModuleIO() {}, 1, config.getRobotMaxVelocity());

      SwerveModule blModule =
          new SwerveModule(new SwerveModuleIO() {}, 2, config.getRobotMaxVelocity());

      SwerveModule brModule =
          new SwerveModule(new SwerveModuleIO() {}, 3, config.getRobotMaxVelocity());
      drivetrain = new Drivetrain(new GyroIO() {}, flModule, frModule, blModule, brModule);
      vision = new Vision(new VisionIO() {});
    }

    // FIXME: delete after testing
    this.squaringSpeed =
        new TunableNumber(
            "RobotContainer/SquaringSpeed", RobotConfig.getInstance().getSquaringSpeed());
    this.squaringDuration =
        new TunableNumber(
            "RobotContainer/SquaringDuration", RobotConfig.getInstance().getSquaringDuration());

    // tab for gyro

    // disable all telemetry in the LiveWindow to reduce the processing during each iteration
    LiveWindow.disableAllTelemetry();

    constructField();

    updateOI();

    configureAutoCommands();
  }

  public void constructField() {
    Field2d.getInstance()
        .setRegions(
            new Region2d[] {
              COMMUNITY_REGION_1,
              COMMUNITY_REGION_2,
              COMMUNITY_REGION_3,
              LOADING_ZONE_REGION_1,
              LOADING_ZONE_REGION_2,
              FIELD_ZONE_REGION_1,
              FIELD_ZONE_REGION_2,
              FIELD_ZONE_REGION_3
            });

    COMMUNITY_REGION_1.addNeighbor(COMMUNITY_REGION_2, COMMUNITY_REGION_1_2_TRANSITION_POINT);
    COMMUNITY_REGION_2.addNeighbor(COMMUNITY_REGION_1, COMMUNITY_REGION_2_1_TRANSITION_POINT);
    COMMUNITY_REGION_1.addNeighbor(COMMUNITY_REGION_3, COMMUNITY_REGION_1_3_TRANSITION_POINT);
    COMMUNITY_REGION_3.addNeighbor(COMMUNITY_REGION_1, COMMUNITY_REGION_3_1_TRANSITION_POINT);
    COMMUNITY_REGION_2.addNeighbor(FIELD_ZONE_REGION_1, COMMUNITY_2_TO_FIELD_1_TRANSITION_POINT);
    COMMUNITY_REGION_3.addNeighbor(FIELD_ZONE_REGION_1, COMMUNITY_3_TO_FIELD_1_TRANSITION_POINT);

    LOADING_ZONE_REGION_1.addNeighbor(
        LOADING_ZONE_REGION_2, LOADING_ZONE_REGION_1_2_TRANSITION_POINT);
    LOADING_ZONE_REGION_2.addNeighbor(
        LOADING_ZONE_REGION_1, LOADING_ZONE_REGION_2_1_TRANSITION_POINT);
    LOADING_ZONE_REGION_1.addNeighbor(FIELD_ZONE_REGION_3, LOADING_1_TO_FIELD_3_TRANSITION_POINT);

    FIELD_ZONE_REGION_1.addNeighbor(FIELD_ZONE_REGION_2, FIELD_ZONE_REGION_1_2_TRANSITION_POINT);
    FIELD_ZONE_REGION_1.addNeighbor(FIELD_ZONE_REGION_3, FIELD_ZONE_REGION_1_3_TRANSITION_POINT);
    FIELD_ZONE_REGION_2.addNeighbor(FIELD_ZONE_REGION_1, FIELD_ZONE_REGION_2_1_TRANSITION_POINT);
    FIELD_ZONE_REGION_3.addNeighbor(FIELD_ZONE_REGION_1, FIELD_ZONE_REGION_3_1_TRANSITION_POINT);
    FIELD_ZONE_REGION_1.addNeighbor(COMMUNITY_REGION_2, FIELD_1_TO_COMMUNITY_2_TRANSITION_POINT);
    FIELD_ZONE_REGION_1.addNeighbor(COMMUNITY_REGION_3, FIELD_1_TO_COMMUNITY_3_TRANSITION_POINT);
    FIELD_ZONE_REGION_2.addNeighbor(LOADING_ZONE_REGION_1, FIELD_2_TO_LOADING_1_TRANSITION_POINT);
    FIELD_ZONE_REGION_2.addNeighbor(LOADING_ZONE_REGION_2, FIELD_2_TO_LOADING_2_TRANSITION_POINT);
  }

  /**
   * This method scans for any changes to the connected joystick. If anything changed, it creates
   * new OI objects and binds all of the buttons to commands.
   */
  public void updateOI() {
    OperatorInterface prevOI = oi;
    oi = OISelector.getOperatorInterface();
    if (oi == prevOI) {
      return;
    }

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

    // slow mode toggle
    oi.getTranslationSlowModeButton()
        .onTrue(Commands.runOnce(drivetrain::enableTranslationSlowMode, drivetrain));
    oi.getTranslationSlowModeButton()
        .onFalse(Commands.runOnce(drivetrain::disableTranslationSlowMode, drivetrain));

    oi.getRotationSlowModeButton()
        .onTrue(Commands.runOnce(drivetrain::enableRotationSlowMode, drivetrain));
    oi.getRotationSlowModeButton()
        .onFalse(Commands.runOnce(drivetrain::disableRotationSlowMode, drivetrain));
    // reset gyro to 0 degrees
    oi.getResetGyroButton().onTrue(Commands.runOnce(drivetrain::zeroGyroscope, drivetrain));

    // x-stance
    oi.getXStanceButton().onTrue(Commands.runOnce(drivetrain::enableXstance, drivetrain));
    oi.getXStanceButton().onFalse(Commands.runOnce(drivetrain::disableXstance, drivetrain));

    // move to grid / loading zone
    oi.getMoveToGridButton().onTrue(new MoveToGrid(drivetrain));
    oi.getIntakeShelfRightButton()
        .onTrue(new MoveToLoadingZone(drivetrain, FieldRegionConstants.DOUBLE_SUBSTATION_LOWER));
    oi.getIntakeShelfLeftButton()
        .onTrue(new MoveToLoadingZone(drivetrain, FieldRegionConstants.DOUBLE_SUBSTATION_UPPER));
    oi.getIntakeChuteButton()
        .onTrue(new MoveToLoadingZone(drivetrain, FieldRegionConstants.SINGLE_SUBSTATION));

    // toggle manipulator open/close
    oi.toggleManipulatorOpenCloseButton()
        .toggleOnTrue(
            Commands.either(
                new GrabGamePiece(manipulator),
                new ReleaseGamePiece(manipulator),
                manipulator::isOpened));

    // turbo
    oi.getTurboButton().onTrue(Commands.runOnce(drivetrain::enableTurbo, drivetrain));
    oi.getTurboButton().onFalse(Commands.runOnce(drivetrain::disableTurbo, drivetrain));

    // enable/disable vision
    oi.getVisionIsEnabledSwitch().onTrue(Commands.runOnce(() -> vision.enable(true), vision));
    oi.getVisionIsEnabledSwitch()
        .onFalse(
            Commands.parallel(
                Commands.runOnce(() -> vision.enable(false)),
                Commands.runOnce(drivetrain::resetPoseRotationToGyro)));
  }

  private Command moveAndScoreGamePiece(int replaceWithEnumeratedValueForElevatorPosition) {
    // The move to grid command needs to know how long it will take to position the elevator to
    // optimize when it starts moving the robot and to ensure that the held game piece is not
    // smashed into a field element because the elevator isn't in the final position.
    Command setElevatorPosition =
        Commands.sequence(
            Commands.print("replace with elevator SetPosition command"), Commands.waitSeconds(2.0));

    return Commands.sequence(
        Commands.parallel(
            Commands.print("replace with command to set LED color for auto control"),
            setElevatorPosition,
            Commands.sequence(
                new MoveToGrid(
                    drivetrain), // , 2.0), // replace 2.0 with the time to position the elevator
                // (e.g., setElevatorPosition.getTimeToPosition())
                Commands.runOnce(
                    () -> drivetrain.drive(-squaringSpeed.get(), 0.0, 0.0, true, true), drivetrain),
                Commands.waitSeconds(squaringDuration.get()),
                Commands.runOnce(drivetrain::stop))),
        new ReleaseGamePiece(manipulator),
        Commands.print("replace with command to set LED color for driver control"));
  }

  /*
   * This is a conservative command builder that completely moves the arm before the move to grid command is executed.
   *
   * Other stuff to consider:
   *
   * Should we back away after grabbing the game piece to reduce the chance the driver smashes the elevator into a field element?
   *
   * Should we move the squaring step into the move to grid command (since it knows which way to move)?
   */
  private Command moveAndGrabGamePiece(int replaceWithEnumeratedValueForElevatorPosition) {
    // Other commands will need to query how long the move to grid command will take (e.g., we want
    // to signal the human player x seconds before the robot reaching the game piece); so, we need
    // to store a reference to the command in a variable that can be passed along to other commands.
    MoveToGrid moveToGridCommand = new MoveToGrid(drivetrain);

    return Commands.sequence(
        // First step in the sequence is to move the elevator into position, while that is
        // occurring, allow the robot to continue to be driven. Once the elevator is in position,
        // this command group will finish.
        Commands.deadline(
            Commands.sequence(
                Commands.print("replace with elevator SetPosition command"),
                Commands.waitSeconds(2.0)), // simulate delay of SetPosition
            new TeleopSwerve(drivetrain, oi::getTranslateX, oi::getTranslateY, oi::getRotate)),

        // Second step in the sequence is to automatically move the robot to the specified
        // substation location. This command group will complete as soon as the manipulator grabs a
        // game piece, which should occur while the move to grid (or the squaring command) is still
        // executing. If a game piece is never required, the driver has to reset and interrupt this
        // entire command group.
        Commands.deadline(
            new GrabGamePiece(manipulator),
            Commands.print("replace with command to set LED color for auto control"),
            Commands.sequence(
                Commands.parallel(
                    Commands.print(
                        "replace with command to set LED color after delay and pass reference to move to grid command from which the time can be queried"),
                    moveToGridCommand),
                Commands.sequence(
                    Commands.runOnce(
                        // this won't be the right direction....
                        () -> drivetrain.drive(-squaringSpeed.get(), 0.0, 0.0, true, true),
                        drivetrain),
                    Commands.waitSeconds(squaringDuration.get()),
                    Commands.runOnce(drivetrain::stop)))),

        // Third step in the sequence is to move the elevator into the transit position. While the
        // elevator is moving, allow the driver to start to drive the robot.
        Commands.deadline(
            Commands.sequence(
                Commands.print("replace with elevator SetPosition command for transit position"),
                Commands.waitSeconds(2.0)), // simulate delay of SetPosition
            Commands.print("replace with command to set LED color for driver control"),
            new TeleopSwerve(drivetrain, oi::getTranslateX, oi::getTranslateY, oi::getRotate)));
  }

  /** Use this method to define your commands for autonomous mode. */
  private void configureAutoCommands() {
    autoEventMap.put("event1", Commands.print("passed marker 1"));
    autoEventMap.put("event2", Commands.print("passed marker 2"));
    autoEventMap.put("Prepare To Intake Cone", Commands.print("preparing to intake cone"));
    autoEventMap.put("intake cone", Commands.print("cone intake"));
    autoEventMap.put("Raise Elevator", Commands.print("raising elevator"));
    autoEventMap.put("Bring in Elevator", Commands.print("brining in collector"));
    // autoEventMap.put("Bring In Elevator", Commands.print("brining in collector"));

    // creates 2 Path Constraints to be used in auto paths
    PathConstraints overCableConnector = new PathConstraints(1.0, 1.0);
    PathConstraints regularSpeed = new PathConstraints(2.0, 2.0);

    // build auto path commands
    List<PathPlannerTrajectory> auto1Paths =
        PathPlanner.loadPathGroup(
            "testPaths1", config.getAutoMaxSpeed(), config.getAutoMaxAcceleration());
    Command autoTest =
        Commands.sequence(
            new FollowPathWithEvents(
                new FollowPath(auto1Paths.get(0), drivetrain, true, true),
                auto1Paths.get(0).getMarkers(),
                autoEventMap),
            Commands.runOnce(drivetrain::enableXstance, drivetrain),
            Commands.waitSeconds(5.0),
            Commands.runOnce(drivetrain::disableXstance, drivetrain),
            new FollowPathWithEvents(
                new FollowPath(auto1Paths.get(1), drivetrain, false, true),
                auto1Paths.get(1).getMarkers(),
                autoEventMap));

    PathPlannerTrajectory startPointPath =
        PathPlanner.loadPath(
            "StartPoint", config.getAutoMaxSpeed(), config.getAutoMaxAcceleration());
    Command startPoint =
        Commands.runOnce(
            () -> drivetrain.resetOdometry(startPointPath.getInitialState()), drivetrain);

    // add commands to the auto chooser
    autoChooser.addDefaultOption("Do Nothing", new InstantCommand());

    // demonstration of PathPlanner path group with event markers
    autoChooser.addOption("Test Path", autoTest);

    autoChooser.addOption("Start Point", startPoint);
    // "auto" command for tuning the drive velocity PID
    autoChooser.addOption(
        "Drive Velocity Tuning",
        Commands.sequence(
            Commands.runOnce(drivetrain::disableFieldRelative, drivetrain),
            Commands.deadline(
                Commands.waitSeconds(5.0),
                Commands.run(() -> drivetrain.drive(1.5, 0.0, 0.0, false, false), drivetrain))));

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
        PathPlanner.loadPathGroup(
            "Blue-CableSide 2 Cone + Engage",
            overCableConnector,
            overCableConnector,
            regularSpeed,
            regularSpeed,
            overCableConnector,
            regularSpeed);
    Command blueCableSide2ConeEngageCommand =
        Commands.sequence(
            // hold for cone place code
            Commands.waitSeconds(0),
            new FollowPath(blueCableSide2ConeEngagePath.get(0), drivetrain, true, true),
            Commands.waitSeconds(0),
            new FollowPath(blueCableSide2ConeEngagePath.get(1), drivetrain, false, true),
            Commands.waitSeconds(0),
            new FollowPath(blueCableSide2ConeEngagePath.get(2), drivetrain, false, true),
            Commands.waitSeconds(0),
            new FollowPath(blueCableSide2ConeEngagePath.get(3), drivetrain, false, true),
            new FollowPath(blueCableSide2ConeEngagePath.get(4), drivetrain, false, true),
            new FollowPath(blueCableSide2ConeEngagePath.get(5), drivetrain, false, true),
            Commands.runOnce(
                () -> drivetrain.drive(-squaringSpeed.get(), 0.0, 0.0, true, true), drivetrain),
            Commands.waitSeconds(squaringDuration.get()),
            Commands.runOnce(drivetrain::enableXstance, drivetrain),
            Commands.waitSeconds(0.5),
            Commands.runOnce(drivetrain::disableXstance, drivetrain));
    autoChooser.addOption("Blue-CableSide 2 Cone + Engage ", blueCableSide2ConeEngageCommand);

    // "auto" path for Blue-CableSide 3 Cone
    List<PathPlannerTrajectory> blueCableSide3ConePath =
        PathPlanner.loadPathGroup("Blue-CableSide 3 Cone", 2.0, 2.0);
    // regularSpeed,
    // overCableConnector,
    // regularSpeed,
    // overCableConnector,
    // regularSpeed,
    // overCableConnector);
    Command blueCableSide3ConeCommand =
        Commands.sequence(
            new FollowPath(blueCableSide3ConePath.get(0), drivetrain, true, true),
            new FollowPath(blueCableSide3ConePath.get(1), drivetrain, false, true),
            new FollowPath(blueCableSide3ConePath.get(2), drivetrain, false, true),
            new FollowPath(blueCableSide3ConePath.get(3), drivetrain, false, true),
            Commands.runOnce(
                () -> drivetrain.drive(-squaringSpeed.get(), 0.0, 0.0, true, true), drivetrain),
            Commands.waitSeconds(squaringDuration.get()),
            Commands.runOnce(drivetrain::enableXstance, drivetrain),
            Commands.waitSeconds(2.0),
            Commands.runOnce(drivetrain::disableXstance, drivetrain),
            new FollowPath(blueCableSide3ConePath.get(4), drivetrain, false, true),
            new FollowPath(blueCableSide3ConePath.get(5), drivetrain, false, true),
            new FollowPath(blueCableSide3ConePath.get(6), drivetrain, false, true),
            new FollowPath(blueCableSide3ConePath.get(7), drivetrain, false, true),
            Commands.runOnce(
                () -> drivetrain.drive(-squaringSpeed.get(), 0.0, 0.0, true, true), drivetrain),
            Commands.waitSeconds(squaringDuration.get()),
            Commands.runOnce(drivetrain::enableXstance, drivetrain),
            Commands.waitSeconds(0.5),
            Commands.runOnce(drivetrain::disableXstance, drivetrain));
    autoChooser.addOption(
        "Blue-CableSide 3 Cone (over cable connector)", blueCableSide3ConeCommand);

    // "auto" path for Blue-CenterCable 2 Cone + Engage Copy
    List<PathPlannerTrajectory> blueCenterCable2ConeEngageCopyPath =
        PathPlanner.loadPathGroup("Blue-CenterCable 2 Cone + Engage Copy", 2.0, 2.0);
    Command blueCenterCable2ConeEngageCopyCommand =
        Commands.sequence(
            new FollowPath(blueCenterCable2ConeEngageCopyPath.get(0), drivetrain, true, true),
            Commands.waitSeconds(5),
            new FollowPath(blueCenterCable2ConeEngageCopyPath.get(1), drivetrain, false, true));
    autoChooser.addOption(
        "Blue Center Cable 2 Cone Engage Copy Path", blueCenterCable2ConeEngageCopyCommand);

    // "auto" path for Blue-CenterLoad 2 Cone + Engage
    PathPlannerTrajectory blueCenterLoad2ConeEngagePath =
        PathPlanner.loadPath("Blue-CenterLoad 2 Cone + Engage", 2.0, 2.0);
    Command blueCenterLoad2ConeEngageCommand =
        new FollowPath(blueCenterLoad2ConeEngagePath, drivetrain, true, true);
    autoChooser.addOption("Blue Center Load 2 Cone Engage Path", blueCenterLoad2ConeEngageCommand);

    // "auto" path for Blue-LoadingSide 2 Cone + Engage
    List<PathPlannerTrajectory> blueLoadingSide2ConePath =
        PathPlanner.loadPathGroup("Blue-LoadingSide 2 Cone", 2.0, 2.0);
    Command blueLoadingSide2ConeCommand =
        Commands.sequence(
            new FollowPathWithEvents(
                new FollowPath(blueLoadingSide2ConePath.get(0), drivetrain, true, true),
                blueLoadingSide2ConePath.get(0).getMarkers(),
                autoEventMap),
            new DriveToPose(drivetrain, FieldRegionConstants.GRID_3_NODE_1),
            Commands.runOnce(
                () -> drivetrain.drive(-squaringSpeed.get(), 0.0, 0.0, true, true), drivetrain),
            Commands.waitSeconds(squaringDuration.get()),
            Commands.runOnce(drivetrain::enableXstance, drivetrain),
            Commands.waitSeconds(2.0),
            Commands.runOnce(drivetrain::disableXstance, drivetrain));
    autoChooser.addOption(
        "Blue Loading Side 2 Cone Engage Path ( with event markers)", blueLoadingSide2ConeCommand);

    List<PathPlannerTrajectory> driveToTag6Path =
        PathPlanner.loadPathGroup("Drive to Tag 6", 2.0, 2.0);
    Command driveToTag6Command =
        Commands.sequence(new FollowPath(driveToTag6Path.get(0), drivetrain, true, true));
    // new DriveToPose(drivetrain, FieldConstants.GRID_3_NODE_1),
    // // Commands.print("DRIVE TO POSE FINISHED"),
    // Commands.runOnce(
    //     () -> drivetrain.drive(-squaringSpeed.get(), 0.0, 0.0, true, true), drivetrain),
    // Commands.waitSeconds(squaringDuration.get()),
    // Commands.runOnce(drivetrain::enableXstance, drivetrain),
    // Commands.waitSeconds(2.0),
    // Commands.runOnce(drivetrain::disableXstance, drivetrain));
    autoChooser.addOption("Drive to Pose Test Path(Tag 6)", driveToTag6Command);

    // auto path for mobility bonus and preparing to engage
    PathPlannerTrajectory blueMobilityPrepareToDockPath =
        PathPlanner.loadPath("Blue-Mobility Prepare To Dock", 2.0, 2.0);
    Command blueMobilityPrepareToDockCommand =
        new FollowPath(blueMobilityPrepareToDockPath, drivetrain, true, true);
    autoChooser.addOption(
        "Blue Mobility Bonus and Prepare to Engage", blueMobilityPrepareToDockCommand);

    // "auto" for Blue-LoadingSide 3 Cone
    List<PathPlannerTrajectory> blueLoadingSide3ConePath =
        PathPlanner.loadPathGroup("Blue-Loading Side 3 Cone", 2.0, 2.0);
    Command blueLoadingSide3ConeCommand =
        Commands.sequence(
            new FollowPath(blueLoadingSide3ConePath.get(0), drivetrain, true, true),
            Commands.runOnce(
                () -> drivetrain.drive(-squaringSpeed.get(), 0.0, 0.0, true, true), drivetrain),
            Commands.waitSeconds(squaringDuration.get()),
            Commands.runOnce(drivetrain::enableXstance, drivetrain),
            Commands.waitSeconds(1),
            Commands.runOnce(drivetrain::disableXstance, drivetrain),
            new FollowPath(blueLoadingSide3ConePath.get(1), drivetrain, false, true),
            Commands.runOnce(
                () -> drivetrain.drive(-squaringSpeed.get(), 0.0, 0.0, true, true), drivetrain),
            Commands.waitSeconds(squaringDuration.get()),
            Commands.runOnce(drivetrain::enableXstance, drivetrain),
            Commands.waitSeconds(1),
            Commands.runOnce(drivetrain::disableXstance, drivetrain));
    autoChooser.addOption("Blue Loading Side 3 Cone Path", blueLoadingSide3ConeCommand);

    // "auto" path for Blue-LoadingSide 4 Cone
    PathPlannerTrajectory blueLoadingSide4ConePath =
        PathPlanner.loadPath("Blue-LoadingSide 4 Cone", 2.0, 2.0);
    Command blueLoadingSide4ConeCommand =
        new FollowPath(blueLoadingSide4ConePath, drivetrain, true, true);
    autoChooser.addOption("Blue Loading Side 4 Cone Path", blueLoadingSide4ConeCommand);

    // "auto" path for Tuning auto turn PID
    PathPlannerTrajectory autoTurnPidTuningPath =
        PathPlanner.loadPath("autoTurnPidTuning", 1.0, 1.0);
    Command autoTurnPidTuningCommand =
        new FollowPath(autoTurnPidTuningPath, drivetrain, true, true);
    autoChooser.addOption("Auto Turn PID Tuning", autoTurnPidTuningCommand);

    // "auto" for Blue-CenterLoad 1 Cone + Engage
    PathPlannerTrajectory blueCenterLoad1ConeEngagePath =
        PathPlanner.loadPath("Blue-CenterLoad 1 Cone + Engage", 1.0, 1.0);
    Command blueCenterLoad1ConeEngageCommand =
        Commands.sequence(
            new FollowPath(blueCenterLoad1ConeEngagePath, drivetrain, true, true),
            new AutoBalanceNonStop(drivetrain));
    autoChooser.addOption("Blue CenterLoad 1 Cone and Engage", blueCenterLoad1ConeEngageCommand);

    // "auto" path with no holonomic rotation
    PathPlannerTrajectory noHolonomicRotationPath =
        PathPlanner.loadPath("constantHolonomicRotationPath", 1.0, 1.0);
    Command noHolonomicRotationCommand =
        new FollowPath(noHolonomicRotationPath, drivetrain, true, true);
    autoChooser.addOption("No Holonomic Rotation", noHolonomicRotationCommand);

    Shuffleboard.getTab("MAIN").add(autoChooser.getSendableChooser());

    if (TUNING_MODE) {
      PathPlannerServer.startServer(3061);
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void checkAllianceColor() {
    if (DriverStation.getAlliance() != lastAlliance) {
      lastAlliance = DriverStation.getAlliance();
      vision.updateAlliance(lastAlliance);
      Field2d.getInstance().updateAlliance(lastAlliance);
    }
  }
}
