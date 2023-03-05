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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.Constants.Mode;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.FeedForwardCharacterization.FeedForwardCharacterizationData;
import frc.robot.commands.FollowPath;
import frc.robot.commands.GrabGamePiece;
import frc.robot.commands.MoveToGrid;
import frc.robot.commands.MoveToLoadingZone;
import frc.robot.commands.ReleaseGamePiece;
import frc.robot.commands.RotateToAngle;
import frc.robot.commands.SetElevatorPosition;
import frc.robot.commands.SetIntakeState;
import frc.robot.commands.StallAgainstElement;
import frc.robot.commands.TeleopSwerve;
import frc.robot.configs.MK4IRobotConfig;
import frc.robot.configs.NovaRobotConfig;
import frc.robot.configs.SierraRobotConfig;
import frc.robot.configs.TestBoardConfig;
import frc.robot.operator_interface.OISelector;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.Position;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.manipulator.ManipulatorIO;
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
  private Elevator elevator;
  private RobotConfig config;
  private Drivetrain drivetrain;
  private Alliance lastAlliance = DriverStation.Alliance.Invalid;
  private Manipulator manipulator;
  private Intake intake;
  private Vision vision;
  private LEDs led;

  // use AdvantageKit's LoggedDashboardChooser instead of SendableChooser to ensure accurate logging
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Routine");

  // FIXME: delete after testing
  private final LoggedDashboardChooser<ElevatorConstants.Position> armChooser =
      new LoggedDashboardChooser<>("Arm Position");

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
        case ROBOT_2023_NOVA:
        case ROBOT_2023_MK4I:
        case ROBOT_2022_SIERRA:
          {
            // create the specific RobotConfig subclass instance first
            if (Constants.getRobot() == Constants.RobotType.ROBOT_2022_SIERRA) {
              config = new SierraRobotConfig();
            } else if (Constants.getRobot() == Constants.RobotType.ROBOT_2023_MK4I) {
              config = new MK4IRobotConfig();
            } else { // default to ROBOT_2023_NOVA
              config = new NovaRobotConfig();
            }
            elevator = new Elevator(new ElevatorIOTalonFX());
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

            intake = new Intake(new IntakeIOTalonFX());

            vision = new Vision(new VisionIOPhotonVision(config.getCameraName()));

            led = new LEDs();

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
            intake = new Intake(new IntakeIOTalonFX());
            led = new LEDs();
            break;
          }
        case ROBOT_SIMBOT:
          {
            config = new MK4IRobotConfig();

            elevator = new Elevator(new ElevatorIOSim());

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
            intake = new Intake(new IntakeIOSim());
            led = new LEDs();
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
      manipulator = new Manipulator(new ManipulatorIO() {});
      intake = new Intake(new IntakeIO() {});
      vision = new Vision(new VisionIO() {});
    }

    // tab for gyro

    // disable all telemetry in the LiveWindow to reduce the processing during each iteration
    LiveWindow.disableAllTelemetry();

    constructField();

    updateOI();

    configureAutoCommands();

    // FIXME: remove after testing
    armChooser.addDefaultOption("CONE_STORAGE", ElevatorConstants.Position.CONE_STORAGE);
    armChooser.addOption("CUBE_STORAGE", ElevatorConstants.Position.CUBE_STORAGE);
    armChooser.addOption("AUTO_STORAGE", ElevatorConstants.Position.AUTO_STORAGE);
    armChooser.addOption("CONE_INTAKE_FLOOR", ElevatorConstants.Position.CONE_INTAKE_FLOOR);
    armChooser.addOption("CUBE_INTAKE_BUMPER", ElevatorConstants.Position.CUBE_INTAKE_BUMPER);
    armChooser.addOption("CONE_INTAKE_SHELF", ElevatorConstants.Position.CONE_INTAKE_SHELF);
    armChooser.addOption("CUBE_INTAKE_SHELF", ElevatorConstants.Position.CUBE_INTAKE_SHELF);
    armChooser.addOption("CONE_INTAKE_CHUTE", ElevatorConstants.Position.CONE_INTAKE_CHUTE);
    armChooser.addOption("CUBE_INTAKE_CHUTE", ElevatorConstants.Position.CUBE_INTAKE_CHUTE);
    armChooser.addOption("CONE_HYBRID_LEVEL", ElevatorConstants.Position.CONE_HYBRID_LEVEL);
    armChooser.addOption("CONE_MID_LEVEL", ElevatorConstants.Position.CONE_MID_LEVEL);
    armChooser.addOption("CONE_HIGH_LEVEL", ElevatorConstants.Position.CONE_HIGH_LEVEL);
    armChooser.addOption("CUBE_HYBRID_LEVEL", ElevatorConstants.Position.CUBE_HYBRID_LEVEL);
    armChooser.addOption("CUBE_MID_LEVEL", ElevatorConstants.Position.CUBE_MID_LEVEL);
    armChooser.addOption("CUBE_HIGH_LEVEL", ElevatorConstants.Position.CUBE_HIGH_LEVEL);

    Shuffleboard.getTab("Elevator").add(armChooser.getSendableChooser());
  }

  public void constructField() {
    // FIXME: adjust boundary and transition points after reviewing field region slides
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
              FIELD_ZONE_REGION_3,
              FIELD_ZONE_REGION_4
            });

    COMMUNITY_REGION_1.addNeighbor(COMMUNITY_REGION_2, COMMUNITY_REGION_1_2_TRANSITION_POINT);
    COMMUNITY_REGION_2.addNeighbor(COMMUNITY_REGION_1, COMMUNITY_REGION_2_1_TRANSITION_POINT);
    COMMUNITY_REGION_1.addNeighbor(COMMUNITY_REGION_3, COMMUNITY_REGION_1_3_TRANSITION_POINT);
    COMMUNITY_REGION_3.addNeighbor(COMMUNITY_REGION_1, COMMUNITY_REGION_3_1_TRANSITION_POINT);
    COMMUNITY_REGION_2.addNeighbor(FIELD_ZONE_REGION_1, COMMUNITY_2_TO_FIELD_1_TRANSITION_POINT);
    COMMUNITY_REGION_3.addNeighbor(FIELD_ZONE_REGION_2, COMMUNITY_3_TO_FIELD_2_TRANSITION_POINT);

    LOADING_ZONE_REGION_1.addNeighbor(
        LOADING_ZONE_REGION_2, LOADING_ZONE_REGION_1_2_TRANSITION_POINT);
    LOADING_ZONE_REGION_2.addNeighbor(
        LOADING_ZONE_REGION_1, LOADING_ZONE_REGION_2_1_TRANSITION_POINT);
    LOADING_ZONE_REGION_2.addNeighbor(FIELD_ZONE_REGION_4, LOADING_2_TO_FIELD_4_TRANSITION_POINT);

    FIELD_ZONE_REGION_1.addNeighbor(FIELD_ZONE_REGION_2, FIELD_ZONE_REGION_1_2_TRANSITION_POINT);
    FIELD_ZONE_REGION_2.addNeighbor(FIELD_ZONE_REGION_1, FIELD_ZONE_REGION_2_1_TRANSITION_POINT);
    FIELD_ZONE_REGION_1.addNeighbor(FIELD_ZONE_REGION_3, FIELD_ZONE_REGION_1_3_TRANSITION_POINT);
    FIELD_ZONE_REGION_3.addNeighbor(FIELD_ZONE_REGION_1, FIELD_ZONE_REGION_3_1_TRANSITION_POINT);
    FIELD_ZONE_REGION_1.addNeighbor(FIELD_ZONE_REGION_4, FIELD_ZONE_REGION_1_4_TRANSITION_POINT);
    FIELD_ZONE_REGION_4.addNeighbor(FIELD_ZONE_REGION_1, FIELD_ZONE_REGION_4_1_TRANSITION_POINT);
    FIELD_ZONE_REGION_1.addNeighbor(COMMUNITY_REGION_2, FIELD_1_TO_COMMUNITY_2_TRANSITION_POINT);
    FIELD_ZONE_REGION_2.addNeighbor(COMMUNITY_REGION_3, FIELD_2_TO_COMMUNITY_3_TRANSITION_POINT);
    FIELD_ZONE_REGION_3.addNeighbor(LOADING_ZONE_REGION_1, FIELD_3_TO_LOADING_1_TRANSITION_POINT);
    FIELD_ZONE_REGION_4.addNeighbor(LOADING_ZONE_REGION_2, FIELD_4_TO_LOADING_2_TRANSITION_POINT);
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

    configureDrivetrainCommands();
    configureElevatorCommands();
    configureManipulatorCommands();
    configureIntakeButtons();
    configureAutomatedSequenceCommands();

    // enable/disable vision
    oi.getVisionIsEnabledSwitch().onTrue(Commands.runOnce(() -> vision.enable(true)));
    oi.getVisionIsEnabledSwitch()
        .onFalse(
            Commands.parallel(
                Commands.runOnce(() -> vision.enable(false), vision),
                Commands.runOnce(drivetrain::resetPoseRotationToGyro)));
    oi.getInterruptAll()
        .onTrue(
            Commands.parallel(
                Commands.runOnce(manipulator::stop),
                Commands.runOnce(elevator::stopElevator),
                Commands.runOnce(intake::stopIntake),
                new TeleopSwerve(drivetrain, oi::getTranslateX, oi::getTranslateY, oi::getRotate)));
  }

  /** Use this method to define your commands for autonomous mode. */
  private void configureAutoCommands() {
    autoEventMap.put("event1", Commands.print("passed marker 1"));
    autoEventMap.put("event2", Commands.print("passed marker 2"));
    autoEventMap.put("bring in elevator", new SetElevatorPosition(elevator, Position.CONE_STORAGE));
    autoEventMap.put("prepare to intake cone", collectGamePieceAuto());
    autoEventMap.put(
        "set elevator auto position", new SetElevatorPosition(elevator, Position.CONE_STORAGE));
    autoEventMap.put("collect game piece", collectGamePieceAuto());

    // autoEventMap.put("Bring In Elevator", Commands.print("brining in collector"));

    // creates 2 Path Constraints to be used in auto paths
    PathConstraints overCableConnector = new PathConstraints(1.0, 1.0);
    PathConstraints regularSpeed = new PathConstraints(2.0, 2.0);
    PathConstraints hybridConeSpeed = new PathConstraints(2.0, 2.0);
    PathConstraints engageSpeed = new PathConstraints(1.5, 2.0);

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
    // 1.8465179792483901,1.0401321541158826,-9.588751578589954e-17
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
            "Blue-CableSide 2 Cone",
            overCableConnector,
            overCableConnector,
            regularSpeed,
            regularSpeed,
            overCableConnector,
            regularSpeed,
            overCableConnector);
    List<PathPlannerTrajectory> cableSideEngagePath =
        PathPlanner.loadPathGroup("CableSideEngage", 2.0, 2.0);
    Command blueCableSide2ConeEngageCommand =
        Commands.sequence(
            scoreGamePieceAuto(Position.CONE_MID_LEVEL),
            // new SetElevatorPosition(elevator, Position.AUTO_STORAGE),
            new FollowPathWithEvents(
                new FollowPath(blueCableSide2ConeEngagePath.get(0), drivetrain, true, true),
                blueCableSide2ConeEngagePath.get(0).getMarkers(),
                autoEventMap),
            new FollowPathWithEvents(
                new FollowPath(blueCableSide2ConeEngagePath.get(1), drivetrain, false, true),
                blueCableSide2ConeEngagePath.get(1).getMarkers(),
                autoEventMap),
            new FollowPathWithEvents(
                new FollowPath(blueCableSide2ConeEngagePath.get(2), drivetrain, false, true),
                blueCableSide2ConeEngagePath.get(2).getMarkers(),
                autoEventMap),
            new FollowPathWithEvents(
                new FollowPath(blueCableSide2ConeEngagePath.get(3), drivetrain, false, true),
                blueCableSide2ConeEngagePath.get(3).getMarkers(),
                autoEventMap),
            new FollowPathWithEvents(
                new FollowPath(blueCableSide2ConeEngagePath.get(4), drivetrain, false, true),
                blueCableSide2ConeEngagePath.get(4).getMarkers(),
                autoEventMap),
            new FollowPathWithEvents(
                new FollowPath(blueCableSide2ConeEngagePath.get(5), drivetrain, false, true),
                blueCableSide2ConeEngagePath.get(5).getMarkers(),
                autoEventMap),
            Commands.parallel(
                driveAndStallCommand(FieldRegionConstants.GRID_1_NODE_3),
                new SetElevatorPosition(elevator, Position.CONE_MID_LEVEL)),
            new ReleaseGamePiece(manipulator),
            Commands.parallel(
                new FollowPath(cableSideEngagePath.get(0), drivetrain, false, true),
                new SetElevatorPosition(elevator, Position.CONE_STORAGE)),
            new FollowPath(cableSideEngagePath.get(1), drivetrain, false, true),
            new AutoBalance(drivetrain, true));
    autoChooser.addOption("Blue-CableSide 2 Cone + Engage ", blueCableSide2ConeEngageCommand);

    List<PathPlannerTrajectory> blueCableSide2ConePath =
        PathPlanner.loadPathGroup(
            "Blue-CableSide 2 Cone",
            overCableConnector,
            overCableConnector,
            regularSpeed,
            regularSpeed,
            overCableConnector,
            regularSpeed);
    Command blueCableSide2ConeCommand =
        Commands.sequence(
            scoreGamePieceAuto(Position.CONE_MID_LEVEL),
            // new SetElevatorPosition(elevator, Position.AUTO_STORAGE),
            new FollowPathWithEvents(
                new FollowPath(blueCableSide2ConePath.get(0), drivetrain, true, true),
                blueCableSide2ConePath.get(0).getMarkers(),
                autoEventMap),
            new FollowPathWithEvents(
                new FollowPath(blueCableSide2ConePath.get(1), drivetrain, false, true),
                blueCableSide2ConePath.get(1).getMarkers(),
                autoEventMap),
            new FollowPathWithEvents(
                new FollowPath(blueCableSide2ConePath.get(2), drivetrain, false, true),
                blueCableSide2ConePath.get(2).getMarkers(),
                autoEventMap),
            new FollowPathWithEvents(
                new FollowPath(blueCableSide2ConePath.get(3), drivetrain, false, true),
                blueCableSide2ConePath.get(3).getMarkers(),
                autoEventMap),
            new FollowPathWithEvents(
                new FollowPath(blueCableSide2ConePath.get(4), drivetrain, false, true),
                blueCableSide2ConePath.get(4).getMarkers(),
                autoEventMap),
            new FollowPathWithEvents(
                new FollowPath(blueCableSide2ConePath.get(5), drivetrain, false, true),
                blueCableSide2ConePath.get(5).getMarkers(),
                autoEventMap),
            Commands.parallel(
                driveAndStallCommand(FieldRegionConstants.GRID_1_NODE_3),
                new SetElevatorPosition(elevator, Position.CONE_MID_LEVEL)),
            new ReleaseGamePiece(manipulator));
    autoChooser.addOption("Blue-CableSide 2 Cone", blueCableSide2ConeCommand);

    // "auto" path for Blue-CableSide 3 Cone
    // not updated for elevator
    List<PathPlannerTrajectory> blueCableSide3ConePath =
        PathPlanner.loadPathGroup(
            "Blue-CableSide 3 Cone",
            overCableConnector,
            overCableConnector,
            engageSpeed,
            overCableConnector,
            engageSpeed,
            overCableConnector,
            engageSpeed,
            overCableConnector);
    Command blueCableSide3ConeCommand =
        Commands.sequence(
            new FollowPath(blueCableSide3ConePath.get(0), drivetrain, true, true),
            new FollowPath(blueCableSide3ConePath.get(1), drivetrain, false, true),
            new FollowPath(blueCableSide3ConePath.get(2), drivetrain, false, true),
            new FollowPath(blueCableSide3ConePath.get(3), drivetrain, false, true),
            new DriveToPose(
                drivetrain,
                () ->
                    Field2d.getInstance()
                        .mapPoseToCurrentAlliance(
                            adjustPoseForRobot(FieldRegionConstants.GRID_1_NODE_3))),
            new StallAgainstElement(
                drivetrain,
                () ->
                    Field2d.getInstance()
                        .mapPoseToCurrentAlliance(FieldRegionConstants.GRID_3_NODE_1)),
            Commands.runOnce(drivetrain::enableXstance, drivetrain),
            Commands.waitSeconds(2.0),
            Commands.runOnce(drivetrain::disableXstance, drivetrain),
            new FollowPath(blueCableSide3ConePath.get(4), drivetrain, false, true),
            new FollowPath(blueCableSide3ConePath.get(5), drivetrain, false, true),
            new FollowPath(blueCableSide3ConePath.get(6), drivetrain, false, true),
            new FollowPath(blueCableSide3ConePath.get(7), drivetrain, false, true),
            new DriveToPose(
                drivetrain,
                () ->
                    Field2d.getInstance()
                        .mapPoseToCurrentAlliance(
                            adjustPoseForRobot(FieldRegionConstants.GRID_1_NODE_3))),
            new StallAgainstElement(
                drivetrain,
                () ->
                    Field2d.getInstance()
                        .mapPoseToCurrentAlliance(FieldRegionConstants.GRID_3_NODE_1)),
            Commands.runOnce(drivetrain::enableXstance, drivetrain),
            Commands.waitSeconds(0.5),
            Commands.runOnce(drivetrain::disableXstance, drivetrain));
    autoChooser.addOption(
        "Blue-CableSide 3 Cone (over cable connector)", blueCableSide3ConeCommand);

    // "auto" path for Blue-CenterCable 2 Cone + Engage Copy
    /*
    List<PathPlannerTrajectory> blueCenterCable2ConeEngageCopyPath =
        PathPlanner.loadPathGroup("Blue-CenterCable 2 Cone + Engage Copy", 2.0, 2.0);
    Command blueCenterCable2ConeEngageCopyCommand =
        Commands.sequence(
            new FollowPath(blueCenterCable2ConeEngageCopyPath.get(0), drivetrain, true, true),
            Commands.waitSeconds(5),
            new FollowPath(blueCenterCable2ConeEngageCopyPath.get(1), drivetrain, false, true));
    autoChooser.addOption(
        "Blue Center Cable 2 Cone Engage Copy Path", blueCenterCable2ConeEngageCopyCommand);

        */

    // "auto" path for Blue-CenterLoad 2 Cone + Engage
    // not updated for elevator
    List<PathPlannerTrajectory> blueCenterLoad2ConeEngagePath =
        PathPlanner.loadPathGroup("Blue-CenterLoad 2 Cone + Engage", 2.0, 2.0);
    Command blueCenterLoad2ConeEngageCommand =
        Commands.sequence(
            new FollowPathWithEvents(
                new FollowPath(blueCenterLoad2ConeEngagePath.get(0), drivetrain, true, true),
                blueCenterLoad2ConeEngagePath.get(0).getMarkers(),
                autoEventMap),
            new DriveToPose(
                drivetrain,
                () ->
                    Field2d.getInstance()
                        .mapPoseToCurrentAlliance(
                            adjustPoseForRobot(FieldRegionConstants.GRID_2_NODE_1))),
            new StallAgainstElement(
                drivetrain,
                () ->
                    Field2d.getInstance()
                        .mapPoseToCurrentAlliance(FieldRegionConstants.GRID_2_NODE_1)),
            Commands.runOnce(drivetrain::enableXstance, drivetrain),
            Commands.waitSeconds(1.0),
            Commands.runOnce(drivetrain::disableXstance, drivetrain),
            new FollowPathWithEvents(
                new FollowPath(blueCenterLoad2ConeEngagePath.get(1), drivetrain, true, true),
                blueCenterLoad2ConeEngagePath.get(1).getMarkers(),
                autoEventMap));
    autoChooser.addOption("Blue Center Load 2 Cone Engage Path", blueCenterLoad2ConeEngageCommand);

    // "auto" path for Blue-LoadingSide 2 Cone + Engage
    List<PathPlannerTrajectory> blueLoadingSide2ConePath =
        PathPlanner.loadPathGroup("Blue-LoadingSide 2 Cone", 2.0, 2.0);
    List<PathPlannerTrajectory> loadingSideEngagePath =
        PathPlanner.loadPathGroup("LoadingSideEngage", 2.0, 2.0);
    Command blueLoadingSide2ConeEngageCommand =
        Commands.sequence(
            scoreGamePieceAuto(Position.CONE_MID_LEVEL),
            new FollowPathWithEvents(
                new FollowPath(blueLoadingSide2ConePath.get(0), drivetrain, true, true),
                blueLoadingSide2ConePath.get(0).getMarkers(),
                autoEventMap),
            Commands.parallel(
                driveAndStallCommand(FieldRegionConstants.GRID_3_NODE_1),
                new SetElevatorPosition(elevator, Position.CONE_MID_LEVEL)),
            new ReleaseGamePiece(manipulator),
            Commands.parallel(
                new FollowPath(loadingSideEngagePath.get(0), drivetrain, false, true),
                new SetElevatorPosition(elevator, Position.CONE_STORAGE)),
            new FollowPath(loadingSideEngagePath.get(1), drivetrain, false, true),
            new AutoBalance(drivetrain, true));
    autoChooser.addOption(
        "Blue Loading Side 2 Cone Engage Path", blueLoadingSide2ConeEngageCommand);

    // FIXME: create event marker to move elevator into auto safe position
    // verify there is an event marker to move to cone floor collection
    // verify there is an event marker to grab the game piece
    // verify there is an event marker to move to the high cone position
    // check on FollowPathWithEvents; when does it finish?

    // "auto" path for Blue-LoadingSide 2 Cone
    Command blueLoadingSide2ConeCommand =
        Commands.sequence(
            scoreGamePieceAuto(Position.CONE_MID_LEVEL),
            new FollowPathWithEvents(
                new FollowPath(blueLoadingSide2ConePath.get(0), drivetrain, true, true),
                blueLoadingSide2ConePath.get(0).getMarkers(),
                autoEventMap),
            Commands.parallel(
                driveAndStallCommand(FieldRegionConstants.GRID_3_NODE_1),
                new SetElevatorPosition(elevator, Position.CONE_MID_LEVEL)),
            new ReleaseGamePiece(manipulator));
    autoChooser.addOption("Blue Loading Side 2 Cone", blueLoadingSide2ConeCommand);

    // "auto" path for Blue-LoadingGetOutTheWay
    PathPlannerTrajectory getOutTheWay = PathPlanner.loadPath("LoadingSideGetOutTheWay", 1.0, 1.0);
    Command blueLoadingGetOutTheWay =
        Commands.sequence(new FollowPath(getOutTheWay, drivetrain, true, true));
    autoChooser.addOption("Blue Loading Get Out The Way", blueLoadingGetOutTheWay);

    // not used
    /*
    List<PathPlannerTrajectory> driveToTag6Path =
        PathPlanner.loadPathGroup("Drive to Tag 6", 2.0, 2.0);
    Command driveToTag6Command =
        Commands.sequence(new FollowPath(driveToTag6Path.get(0), drivetrain, true, true)),
    new DriveToPose(drivetrain, FieldRegionConstants.GRID_3_NODE_1),
    Commands.print("DRIVE TO POSE FINISHED"),
    Commands.runOnce(
        () -> drivetrain.drive(-squaringSpeed.get(), 0.0, 0.0, true, true), drivetrain),
    Commands.waitSeconds(squaringDuration.get()),
    Commands.runOnce(drivetrain::enableXstance, drivetrain),
    Commands.waitSeconds(2.0),
    Commands.runOnce(drivetrain::disableXstance, drivetrain));
    autoChooser.addOption("Drive to Pose Test Path(Tag 6)", driveToTag6Command);
    */

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
            new DriveToPose(
                drivetrain,
                () ->
                    Field2d.getInstance()
                        .mapPoseToCurrentAlliance(
                            adjustPoseForRobot(FieldRegionConstants.GRID_3_NODE_1))),
            new StallAgainstElement(
                drivetrain,
                () ->
                    Field2d.getInstance()
                        .mapPoseToCurrentAlliance(FieldRegionConstants.GRID_3_NODE_1)),
            Commands.runOnce(drivetrain::enableXstance, drivetrain),
            Commands.waitSeconds(1),
            Commands.runOnce(drivetrain::disableXstance, drivetrain),
            new FollowPath(blueLoadingSide3ConePath.get(1), drivetrain, false, true),
            new DriveToPose(
                drivetrain,
                () ->
                    Field2d.getInstance()
                        .mapPoseToCurrentAlliance(
                            adjustPoseForRobot(FieldRegionConstants.GRID_3_NODE_1))),
            new StallAgainstElement(
                drivetrain,
                () ->
                    Field2d.getInstance()
                        .mapPoseToCurrentAlliance(FieldRegionConstants.GRID_3_NODE_1)),
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

    // "auto" path for Blue-CenterLoad 1 Cone + Engage
    PathPlannerTrajectory blueCenterLoad1ConeEngagePath =
        PathPlanner.loadPath("Blue-CenterLoad 1 Cone + Engage", 1.0, 1.0);
    Command blueCenterLoad1ConeEngageCommand =
        Commands.sequence(
            new FollowPath(blueCenterLoad1ConeEngagePath, drivetrain, true, true),
            new AutoBalance(drivetrain, true));
    autoChooser.addOption("Blue CenterLoad 1 Cone and Engage", blueCenterLoad1ConeEngageCommand);

    // "auto" path for Hybrid Cone Center Position + Engage
    List<PathPlannerTrajectory> hybridConeCenterPositionEngagePath =
        PathPlanner.loadPathGroup(
            "Hybrid Cone Center Position + Engage", hybridConeSpeed, engageSpeed);
    Command hybridConeCenterPositionEngageCommand =
        Commands.sequence(
            new FollowPath(hybridConeCenterPositionEngagePath.get(0), drivetrain, true, true),
            new FollowPath(hybridConeCenterPositionEngagePath.get(1), drivetrain, false, true),
            new AutoBalance(drivetrain, true));
    autoChooser.addOption(
        "Hybrid Cone Center Position + Engage", hybridConeCenterPositionEngageCommand);

    // "auto" path for Hybrid Cone Center Position + Mobility + Engage
    List<PathPlannerTrajectory> hybridConeCenterPositionMobilityEngagePath =
        PathPlanner.loadPathGroup(
            "Hybrid Cone Center Position + Mobility + Engage",
            hybridConeSpeed,
            engageSpeed,
            engageSpeed);
    Command hybridConeCenterPositionMobilityEngageCommand =
        Commands.sequence(
            new FollowPath(
                hybridConeCenterPositionMobilityEngagePath.get(0), drivetrain, true, true),
            new FollowPath(
                hybridConeCenterPositionMobilityEngagePath.get(1), drivetrain, false, true),
            new RotateToAngle(
                drivetrain,
                () ->
                    new Pose2d(
                        drivetrain.getPose().getX(),
                        drivetrain.getPose().getY(),
                        Rotation2d.fromDegrees(0.0))),
            new FollowPath(
                hybridConeCenterPositionMobilityEngagePath.get(2), drivetrain, false, true),
            new AutoBalance(drivetrain, true));
    autoChooser.addOption(
        "Hybrid Cone Center Position + Mobility + Engage",
        hybridConeCenterPositionMobilityEngageCommand);

    // "auto" path for 1 Cone + Engage (Center, Left) path
    PathPlannerTrajectory oneConeEngageCenterLeftPath =
        PathPlanner.loadPath("1 Cone + Engage (Center, Left)", overCableConnector);
    PathPlannerTrajectory centerEngagePath = PathPlanner.loadPath("Center Engage", engageSpeed);
    Command oneConeEngageCenterLeftCommand =
        Commands.sequence(
            scoreGamePieceAuto(Position.CONE_MID_LEVEL),
            new SetElevatorPosition(elevator, Position.CONE_STORAGE),
            new FollowPath(oneConeEngageCenterLeftPath, drivetrain, true, true),
            new RotateToAngle(
                drivetrain,
                () ->
                    new Pose2d(
                        drivetrain.getPose().getX(),
                        drivetrain.getPose().getY(),
                        Rotation2d.fromDegrees(0.0))),
            new FollowPath(centerEngagePath, drivetrain, false, true),
            new AutoBalance(drivetrain, true));
    autoChooser.addOption("1 Cone + Engage (Center, Left)", oneConeEngageCenterLeftCommand);

    // "auto" path for 1 Cone + Engage (Center, Right) path
    PathPlannerTrajectory oneConeEngageCenterRightPath =
        PathPlanner.loadPath("1 Cone + Engage (Center, Right)", overCableConnector);
    Command oneConeEngageCenterRightCommand =
        Commands.sequence(
            scoreGamePieceAuto(Position.CONE_MID_LEVEL),
            new SetElevatorPosition(elevator, Position.CONE_STORAGE),
            new FollowPath(oneConeEngageCenterRightPath, drivetrain, true, true),
            new RotateToAngle(
                drivetrain,
                () ->
                    new Pose2d(
                        drivetrain.getPose().getX(),
                        drivetrain.getPose().getY(),
                        Rotation2d.fromDegrees(0.0))),
            new FollowPath(centerEngagePath, drivetrain, false, true),
            new AutoBalance(drivetrain, true));
    autoChooser.addOption("1 Cone + Engage (Center, Right)", oneConeEngageCenterRightCommand);

    // "auto" path for 1 Cone + Engage + Mobility(Center, Left, High) path
    List<PathPlannerTrajectory> oneConeEngageMobilityCenterLeftPath =
        PathPlanner.loadPathGroup(
            "1 Cone + Engage + Mobility(Center, Left, High)",
            overCableConnector,
            engageSpeed,
            overCableConnector,
            engageSpeed);
    Command oneConeEngageMobilityCenterLeftCommand =
        Commands.sequence(
            scoreGamePieceAuto(Position.CONE_MID_LEVEL),
            new SetElevatorPosition(elevator, Position.CONE_STORAGE),
            new FollowPath(oneConeEngageMobilityCenterLeftPath.get(0), drivetrain, true, true),
            new FollowPath(oneConeEngageMobilityCenterLeftPath.get(1), drivetrain, false, true),
            new FollowPath(oneConeEngageMobilityCenterLeftPath.get(2), drivetrain, false, true),
            new FollowPath(oneConeEngageMobilityCenterLeftPath.get(3), drivetrain, false, true),
            new AutoBalance(drivetrain, true));
    autoChooser.addOption(
        "1 Cone + Engage + Mobility(Center, Left, High)", oneConeEngageMobilityCenterLeftCommand);

    // "auto" path for 1 Cone + Engage + Mobility(Center, Right, High) path
    List<PathPlannerTrajectory> oneConeEngageMobilityCenterRightPath =
        PathPlanner.loadPathGroup(
            "1 Cone + Engage + Mobility(Center, Right, High)",
            overCableConnector,
            engageSpeed,
            overCableConnector,
            engageSpeed);
    Command oneConeEngageMobilityCenterRightCommand =
        Commands.sequence(
            scoreGamePieceAuto(Position.CONE_MID_LEVEL),
            new SetElevatorPosition(elevator, Position.CONE_STORAGE),
            new FollowPath(oneConeEngageMobilityCenterRightPath.get(0), drivetrain, true, true),
            new SetElevatorPosition(elevator, Position.AUTO_STORAGE),
            new FollowPath(oneConeEngageMobilityCenterRightPath.get(1), drivetrain, false, true),
            new FollowPath(oneConeEngageMobilityCenterRightPath.get(2), drivetrain, false, true),
            new FollowPath(oneConeEngageMobilityCenterRightPath.get(3), drivetrain, false, true),
            new AutoBalance(drivetrain, true));
    autoChooser.addOption(
        "1 Cone + Engage + Mobility(Center, Right, High)", oneConeEngageMobilityCenterRightCommand);

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

  private void configureDrivetrainCommands() {
    // field-relative toggle
    oi.getFieldRelativeButton()
        .toggleOnTrue(
            Commands.either(
                Commands.runOnce(drivetrain::disableFieldRelative, drivetrain),
                Commands.runOnce(drivetrain::enableFieldRelative, drivetrain),
                drivetrain::getFieldRelative));

    // slow-mode toggle
    oi.getTranslationSlowModeButton()
        .onTrue(Commands.runOnce(drivetrain::enableTranslationSlowMode, drivetrain));
    oi.getTranslationSlowModeButton()
        .onFalse(Commands.runOnce(drivetrain::disableTranslationSlowMode, drivetrain));
    oi.getRotationSlowModeButton()
        .onTrue(Commands.runOnce(drivetrain::enableRotationSlowMode, drivetrain));
    oi.getRotationSlowModeButton()
        .onFalse(Commands.runOnce(drivetrain::disableRotationSlowMode, drivetrain));

    // reset gyro to 0 degrees
    oi.getResetGyroButton()
        .onTrue(
            new RotateToAngle(
                drivetrain,
                () ->
                    new Pose2d(
                        drivetrain.getPose().getX(),
                        drivetrain.getPose().getY(),
                        Rotation2d.fromDegrees(0.0))));

    // reset pose based on vision
    oi.getResetPoseToVisionButton()
        .onTrue(Commands.runOnce(() -> drivetrain.resetPoseToVision(() -> vision.getRobotPose())));

    // x-stance
    oi.getXStanceButton().onTrue(Commands.runOnce(drivetrain::enableXstance, drivetrain));
    oi.getXStanceButton().onFalse(Commands.runOnce(drivetrain::disableXstance, drivetrain));

    // turbo
    oi.getTurboButton().onTrue(Commands.runOnce(drivetrain::enableTurbo, drivetrain));
    oi.getTurboButton().onFalse(Commands.runOnce(drivetrain::disableTurbo, drivetrain));

    // auto balance
    oi.getAutoBalanceButton().onTrue(new AutoBalance(drivetrain, false));
  }

  private void configureElevatorCommands() {
    oi.getConeCubeLEDTriggerButton()
        .toggleOnTrue(
            Commands.either(
                Commands.parallel(
                    Commands.runOnce(elevator::toggleToCube), Commands.runOnce(led::enableCubeLED)),
                Commands.parallel(
                    Commands.runOnce(elevator::toggleToCone), Commands.runOnce(led::enableConeLED)),
                elevator::getToggledToCone));

    oi.getMoveArmToChuteButton()
        .onTrue(
            new SetElevatorPosition(elevator, ElevatorConstants.Position.CONE_INTAKE_CHUTE)
                .unless(() -> !elevator.isManualPresetEnabled()));
    oi.getMoveArmToShelfButton()
        .onTrue(
            new SetElevatorPosition(elevator, ElevatorConstants.Position.CONE_INTAKE_SHELF)
                .unless(() -> !elevator.isManualPresetEnabled()));
    oi.getMoveArmToStorageButton()
        .onTrue(
            new SetElevatorPosition(elevator, ElevatorConstants.Position.CONE_STORAGE)
                .unless(() -> !elevator.isManualPresetEnabled()));
    oi.getMoveArmToLowButton()
        .onTrue(
            new SetElevatorPosition(elevator, ElevatorConstants.Position.CONE_HYBRID_LEVEL)
                .unless(() -> !elevator.isManualPresetEnabled()));
    oi.getMoveArmToMidButton()
        .onTrue(
            new SetElevatorPosition(elevator, ElevatorConstants.Position.CONE_MID_LEVEL)
                .unless(() -> !elevator.isManualPresetEnabled()));
    oi.getMoveArmToHighButton()
        .onTrue(
            new SetElevatorPosition(elevator, ElevatorConstants.Position.CONE_HIGH_LEVEL)
                .unless(() -> !elevator.isManualPresetEnabled()));
    oi.getIntakeGroundConeButton()
        .onTrue(new SetElevatorPosition(elevator, ElevatorConstants.Position.CONE_INTAKE_FLOOR));

    // enable/disable manual elevator control
    oi.getEnableManualElevatorControlButton()
        .onTrue(Commands.runOnce(elevator::enableManualControl, elevator));
    oi.getDisableManualElevatorControlButton()
        .onTrue(Commands.runOnce(elevator::disableManualControl, elevator));

    // enable/disable manual elevator preset control
    oi.getEnableManualElevatorPresetButton()
        .onTrue(Commands.runOnce(elevator::enableManualPreset, elevator));
    oi.getDisableManualElevatorPresetButton()
        .onTrue(Commands.runOnce(elevator::disableManualPreset, elevator));

    elevator.setDefaultCommand(
        Commands.sequence(
            Commands.runOnce(
                () -> elevator.setElevatorExtensionMotorPower(oi.getMoveElevator()), elevator),
            Commands.runOnce(
                () -> elevator.setElevatorRotationMotorPower(oi.getRotateArm()), elevator)));
  }

  private void configureManipulatorCommands() {
    // toggle manipulator open/close
    oi.getToggleManipulatorOpenCloseButton()
        .toggleOnTrue(
            Commands.either(
                Commands.runOnce(manipulator::close),
                Commands.runOnce(manipulator::open),
                manipulator::isOpened));

    // toggle manipulator sensor enable/disable
    oi.getToggleManipulatorSensorButton()
        .onTrue(Commands.runOnce(() -> manipulator.enableManipulatorSensor(true)));
    oi.getToggleManipulatorSensorButton()
        .onFalse(Commands.runOnce(() -> manipulator.enableManipulatorSensor(false), manipulator));
  }

  private void configureIntakeButtons() {
    // intake.setDefaultCommand(
    //     Commands.sequence(
    //         Commands.runOnce(
    //             () -> intake.setRotationMotorPercentage(oi.getIntakeDeployPower()), intake),
    //         Commands.runOnce(
    //             () -> intake.setRotationMotorPercentage(oi.getIntakeRetractPower()), intake)));

    oi.getToggleIntakeRollerButton()
        .toggleOnTrue(
            Commands.either(
                Commands.runOnce(intake::stopRoller, intake),
                Commands.runOnce(intake::enableRoller, intake),
                intake::isRollerSpinning));

    oi.getPositionIntakeToPushCubeCone()
        .onTrue(new SetIntakeState(intake, IntakeConstants.Position.PUSH_CONE_CUBE));
  }

  private void configureAutomatedSequenceCommands() {
    // move to grid / loading zone
    oi.getIntakeShelfRightButton()
        .onTrue(moveAndGrabGamePiece(Position.CONE_INTAKE_SHELF, DOUBLE_SUBSTATION_LOWER));
    oi.getIntakeShelfLeftButton()
        .onTrue(moveAndGrabGamePiece(Position.CONE_INTAKE_SHELF, DOUBLE_SUBSTATION_UPPER));
    oi.getIntakeChuteButton()
        .onTrue(moveAndGrabGamePiece(Position.CONE_INTAKE_CHUTE, SINGLE_SUBSTATION));

    // move to grid
    oi.getMoveToGridButton().onTrue(moveAndScoreGamePiece());

    // enable/disable move to grid
    oi.getMoveToGridEnabledSwitch()
        .onTrue(Commands.runOnce(() -> drivetrain.enableMoveToGrid(true)));
    oi.getMoveToGridEnabledSwitch()
        .onFalse(Commands.runOnce(() -> drivetrain.enableMoveToGrid(false), drivetrain));
  }

  private Command moveAndScoreGamePiece() {
    // The move to grid command needs to know how long it will take to position the elevator to
    // optimize when it starts moving the robot and to ensure that the held game piece is not
    // smashed into a field element because the elevator isn't in the final position.
    Command setElevatorPositionCommand =
        new SetElevatorPosition(
            elevator, () -> SetElevatorPosition.convertGridRowToPosition(oi.getGridRow()));
    MoveToGrid moveToGridCommand =
        new MoveToGrid(drivetrain); // , 2.0), // replace 2.0 with the time to position the elevator
    // (e.g., setElevatorPosition.getTimeToPosition())

    /*
     * If move-to-grid is disabled, this command will never finish since TeleopSwerve never finishes.
     * In this case, the operator will have to manually drop the game piece, which will interrupt this command.
     */

    return Commands.sequence(
        Commands.parallel(
            Commands.either(
                Commands.runOnce(led::enableAutoLED),
                Commands.runOnce(led::enableTeleopLED),
                () -> oi.getMoveToGridEnabledSwitch().getAsBoolean()),
            setElevatorPositionCommand,
            Commands.either(
                Commands.sequence(
                    moveToGridCommand,
                    new DriveToPose(drivetrain, moveToGridCommand.endPoseSupplier()),
                    new StallAgainstElement(drivetrain, moveToGridCommand.endPoseSupplier())),
                new TeleopSwerve(drivetrain, oi::getTranslateX, oi::getTranslateY, oi::getRotate),
                () -> oi.getMoveToGridEnabledSwitch().getAsBoolean())),
        new ReleaseGamePiece(manipulator),
        Commands.runOnce(led::enableTeleopLED));
  }

  private Command driveAndStallCommand(Pose2d moveToGridPosition) {
    return Commands.sequence(
        new DriveToPose(
            drivetrain,
            () ->
                Field2d.getInstance()
                    .mapPoseToCurrentAlliance(adjustPoseForRobot(moveToGridPosition))),
        new StallAgainstElement(
            drivetrain, () -> Field2d.getInstance().mapPoseToCurrentAlliance(moveToGridPosition)));
  }

  private Command scoreGamePieceAuto(Position elevatorPosition) {
    Command setElevatorPositionToScoreAuto = new SetElevatorPosition(elevator, elevatorPosition);
    Command dropGamePieceAuto = new ReleaseGamePiece(manipulator);
    Command stallOnGamePieceAuto = new GrabGamePiece(manipulator);

    return Commands.sequence(
        stallOnGamePieceAuto, setElevatorPositionToScoreAuto, dropGamePieceAuto);
  }

  private Command collectGamePieceAuto() {
    return Commands.sequence(
        new SetElevatorPosition(elevator, Position.CONE_INTAKE_FLOOR),
        new GrabGamePiece(manipulator),
        new SetElevatorPosition(elevator, Position.AUTO_STORAGE));
  }

  private Command moveAndGrabGamePiece(Position elevatorPosition, Pose2d moveToGridPosition) {
    // Other commands will need to query how long the move to grid command will take (e.g., we want
    // to signal the human player x seconds before the robot reaching the game piece); so, we need
    // to store a reference to the command in a variable that can be passed along to other commands.
    // FIXME: pass the time to position the elevator
    Command setElevatorPositionCommandCollection =
        new SetElevatorPosition(elevator, elevatorPosition);
    MoveToLoadingZone moveToLoadingZoneCommand =
        new MoveToLoadingZone(drivetrain, moveToGridPosition);

    return Commands.sequence(
        /*
         * If move-to-grid is enabled, automatically move the robot to the specified
         * substation location. This command group will complete as soon as the manipulator grabs a
         * game piece, which should occur while the move to grid (or the squaring command) is still
         * executing. If a game piece is never required, the driver has to reset and interrupt this
         * entire command group.
         *
         * If move-to-grid is disabled, automatically position the elevator while the driver
         * positions the robot to grab a game piece. This command group will still complete as soon as the
         * manipulator grabs a game piece, which should occur while the driver is positioning the robot.
         */

        Commands.deadline(
            new GrabGamePiece(manipulator),
            Commands.either(
                Commands.runOnce(led::enableAutoLED),
                Commands.runOnce(led::enableTeleopLED),
                () -> oi.getMoveToGridEnabledSwitch().getAsBoolean()),
            Commands.sequence(
                Commands.parallel(
                    Commands.print(
                        "replace with command to set LED color after delay and pass reference to move to grid command from which the time can be queried"),
                    setElevatorPositionCommandCollection,
                    Commands.either(
                        moveToLoadingZoneCommand,
                        new TeleopSwerve(
                            drivetrain, oi::getTranslateX, oi::getTranslateY, oi::getRotate),
                        () -> oi.getMoveToGridEnabledSwitch().getAsBoolean())),
                Commands.either(
                    Commands.sequence(
                        new DriveToPose(drivetrain, moveToLoadingZoneCommand.endPoseSupplier()),
                        new StallAgainstElement(
                            drivetrain, moveToLoadingZoneCommand.endPoseSupplier())),
                    Commands.none(),
                    () -> oi.getMoveToGridEnabledSwitch().getAsBoolean()))),
        Commands.runOnce(led::enableTeleopLED));
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

  public void autonomousInit() {
    led.enableAutoLED();
  }

  public void teleopInit() {
    led.enableTeleopLED();
  }

  public static Pose2d adjustPoseForRobot(Pose2d pose) {
    return new Pose2d(
        pose.getX() + RobotConfig.getInstance().getRobotWidthWithBumpers() / 2,
        pose.getY(),
        pose.getRotation());
  }
}
