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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
import frc.robot.commands.DeployIntake;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.commands.FeedForwardCharacterization.FeedForwardCharacterizationData;
import frc.robot.commands.FollowPath;
import frc.robot.commands.GrabGamePiece;
import frc.robot.commands.MoveToGrid;
import frc.robot.commands.MoveToLoadingZone;
import frc.robot.commands.ReleaseGamePiece;
import frc.robot.commands.RetractIntake;
import frc.robot.commands.RotateToAngle;
import frc.robot.commands.SetElevatorPosition;
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
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.leds.*;
import frc.robot.subsystems.leds.LEDConstants.RobotStateColors;
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

  private final Map<String, Command> autoEventMap = new HashMap<>();

  private PathConstraints overCableConnector = new PathConstraints(1.0, 1.0);
  private PathConstraints regularSpeed = new PathConstraints(2.0, 2.0);
  private PathConstraints hybridConeSpeed = new PathConstraints(2.0, 2.0);
  private PathConstraints engageSpeed = new PathConstraints(1.5, 2.0);

  private static final double SQUARING_AUTO_TIMEOUT_SECONDS = 0.5;
  private static final double SQUARING_GRID_TIMEOUT_SECONDS = 1.0;
  private static final double SQUARING_LOADING_ZONE_TIMEOUT_SECONDS = 6.0;

  // FIXME: delete after testing
  private final LoggedDashboardChooser<ElevatorConstants.Position> armChooser =
      new LoggedDashboardChooser<>("Arm Position");

  // RobotContainer singleton
  private static RobotContainer robotContainer = new RobotContainer();

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

            vision =
                new Vision(
                    new VisionIOPhotonVision(config.getCameraName0()),
                    new VisionIOPhotonVision(config.getCameraName1()));

            led = new LEDs(new LEDIOCANdle());

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
            led = new LEDs(new LEDIOCANdle());
            intake = new Intake(new IntakeIOTalonFX());
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
            led = new LEDs(new LEDIOCANdle());
            intake = new Intake(new IntakeIO() {});
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
                        RobotConfig.getInstance().getRobotToCameraTransforms()[0]));
            break;
          }
        default:
          break;
      }

    } else {
      config = new NovaRobotConfig();
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
      elevator = new Elevator(new ElevatorIO() {});
      intake = new Intake(new IntakeIO() {});
      vision = new Vision(new VisionIO() {}, new VisionIO() {});
      led = new LEDs(new LEDIO() {});
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
                Commands.runOnce(drivetrain::disableXstance),
                Commands.runOnce(led::enableTeleopLED),
                new TeleopSwerve(drivetrain, oi::getTranslateX, oi::getTranslateY, oi::getRotate)));
  }

  /** Use this method to define your commands for autonomous mode. */
  private void configureAutoCommands() {
    // Waypoints
    autoEventMap.put("event1", Commands.print("passed marker 1"));
    autoEventMap.put("event2", Commands.print("passed marker 2"));
    autoEventMap.put(
        "bring in elevator", new SetElevatorPosition(elevator, Position.AUTO_STORAGE, led));
    autoEventMap.put("prepare to intake cone", collectGamePieceAuto());
    autoEventMap.put(
        "set elevator auto position",
        new SetElevatorPosition(elevator, Position.AUTO_STORAGE, led));

    // AutoPath trajectories
    PathPlannerTrajectory oneConeCenterRightPath =
        PathPlanner.loadPath("1ConeCenterRight", overCableConnector);
    PathPlannerTrajectory oneConeCenterLeftPath =
        PathPlanner.loadPath("1ConeCenterLeft", overCableConnector);
    PathPlannerTrajectory centerEngagePath =
        PathPlanner.loadPath("CenterEngage", overCableConnector);
    PathPlannerTrajectory mobilityCenterEngagePath =
        PathPlanner.loadPath("MobilityEngageCenter", overCableConnector);
    PathPlannerTrajectory oneConeCenterGrabEngage =
        PathPlanner.loadPath("1ConeCenterGrabEngage", overCableConnector);
    autoEventMap.put("collect game piece", collectGamePieceAuto());
    PathPlannerTrajectory cableSide1ConeStayPath =
        PathPlanner.loadPath("cableSide1ConeStay", overCableConnector);
    PathPlannerTrajectory loadingSide1ConeStayPath =
        PathPlanner.loadPath("LoadingSide1ConeStay", overCableConnector);
    PathPlannerTrajectory cableSide2ConePath = PathPlanner.loadPath("CableSide2Cone", 2.0, 2.0);
    PathPlannerTrajectory loadingSide2ConePath =
        PathPlanner.loadPath("LoadingSide2Cone", regularSpeed);
    PathPlannerTrajectory cableSide2ConeHybridPath =
        PathPlanner.loadPath("CableSideHybrid2Cone", 2.0, 2.0);
    PathPlannerTrajectory loadingSide2ConeHybridPath =
        PathPlanner.loadPath("LoadingSideHybrid2Cone", 2.0, 2.0);
    PathPlannerTrajectory cableSideHybridPath =
        PathPlanner.loadPath("CableSideHybridScore", 2.0, 2.0);
    PathPlannerTrajectory loadingSideHybridPath =
        PathPlanner.loadPath("LoadingSideHybridScore", 2.0, 2.0);
    PathPlannerTrajectory cableSideGrabPath = PathPlanner.loadPath("CableSide0.5Cone", 2.0, 2.0);
    PathPlannerTrajectory loadingSideGrabPath =
        PathPlanner.loadPath("LoadingSide0.5Cone", 2.0, 2.0);
    PathPlannerTrajectory loadingSide1ConeGrabPath =
        PathPlanner.loadPath("LoadingSide1ConeGrab", 2.0, 2.0);
    PathPlannerTrajectory cableSide1ConeGrabPath =
        PathPlanner.loadPath("CableSide1ConeGrab", 2.0, 2.0);
    PathPlannerTrajectory centerEngageBackwardsPath =
        PathPlanner.loadPath("CenterEngageBackwards", engageSpeed);

    // autoEventMap.put("Bring In Elevator", Commands.print("brining in collector"));

    // build auto path commands

    // add commands to the auto chooser
    autoChooser.addDefaultOption("Do Nothing", new InstantCommand());

    // ********************************************************************
    // *************** Hybrid Cone Center Position + Engage ***************
    // ********************************************************************

    List<PathPlannerTrajectory> hybridConeCenterPositionEngagePath =
        PathPlanner.loadPathGroup("HybridConeCenterPositionEngage", hybridConeSpeed, engageSpeed);
    Command hybridConeCenterPositionEngageCommand =
        Commands.sequence(
            new FollowPath(hybridConeCenterPositionEngagePath.get(0), drivetrain, true, true),
            Commands.runOnce(elevator::stopElevator, elevator),
            new FollowPath(hybridConeCenterPositionEngagePath.get(1), drivetrain, false, true),
            new AutoBalance(drivetrain, true, led));
    autoChooser.addOption(
        "Hybrid Cone Center Position + Engage", hybridConeCenterPositionEngageCommand);

    // ********************************************************************
    // *************** Hybrid Cone Center Position + Mobility + Engage ****
    // ********************************************************************

    List<PathPlannerTrajectory> hybridConeCenterPositionMobilityEngagePath =
        PathPlanner.loadPathGroup(
            "HybridConeCenterPositionMobilityEngage", hybridConeSpeed, engageSpeed);
    PathPlannerTrajectory farSideEngagePath = PathPlanner.loadPath("EngageFarSide", engageSpeed);
    Command hybridConeCenterPositionMobilityEngageCommand =
        Commands.sequence(
            new FollowPath(
                hybridConeCenterPositionMobilityEngagePath.get(0), drivetrain, true, true),
            Commands.runOnce(elevator::stopElevator, elevator),
            new FollowPath(
                hybridConeCenterPositionMobilityEngagePath.get(1), drivetrain, false, true),
            new RotateToAngle(
                drivetrain,
                () ->
                    new Pose2d(
                        drivetrain.getPose().getX(),
                        drivetrain.getPose().getY(),
                        Rotation2d.fromDegrees(180.0))),
            new FollowPath(farSideEngagePath, drivetrain, false, true),
            new AutoBalance(drivetrain, true, led, false));
    autoChooser.addOption(
        "Hybrid Cone Center Position + Mobility + Engage",
        hybridConeCenterPositionMobilityEngageCommand);

    // ********************************************************************
    // *************** 1 Cone + Engage (Mid, Center, Right) ***************
    // ********************************************************************

    Command oneConeEngageCenterRightMidCommand =
        oneConeCenterEngage(Position.CONE_MID_LEVEL, oneConeCenterRightPath, centerEngagePath);
    autoChooser.addOption(
        "1 Cone + Engage (Mid, Center, Right)", oneConeEngageCenterRightMidCommand);

    // ********************************************************************
    // *************** 1 Cone + Engage (High, Center, Right) **************
    // ********************************************************************

    Command oneConeEngageCenterRightHighCommand =
        oneConeCenterEngage(Position.CONE_HIGH_LEVEL, oneConeCenterRightPath, centerEngagePath);
    autoChooser.addOption(
        "1 Cone + Engage (High, Center, Right)", oneConeEngageCenterRightHighCommand);

    // ********************************************************************
    // *************** 1 Cone + Engage (Mid, Center, Left) ****************
    // ********************************************************************

    Command oneConeEngageCenterLeftMidCommand =
        oneConeCenterEngage(Position.CONE_MID_LEVEL, oneConeCenterLeftPath, centerEngagePath);
    autoChooser.addOption("1 Cone + Engage (Mid, Center, Left)", oneConeEngageCenterLeftMidCommand);

    // ********************************************************************
    // *************** 1 Cone + Engage (High, Center, Left) ***************
    // ********************************************************************

    Command oneConeEngageCenterLeftHighCommand =
        oneConeCenterEngage(Position.CONE_HIGH_LEVEL, oneConeCenterLeftPath, centerEngagePath);
    autoChooser.addOption(
        "1 Cone + Engage (High, Center, Left)", oneConeEngageCenterLeftHighCommand);

    // ********************************************************************
    // ********* 1 Cone + Engage + Mobility (Mid, Center, Right) **********
    // ********************************************************************

    Command oneConeEngageMobilityCenterRightMidCommand =
        oneConeCenterMobilityEngage(
            Position.CONE_MID_LEVEL, oneConeCenterRightPath, mobilityCenterEngagePath);
    // autoChooser.addOption(
    //     "1 Cone + Engage + Mobility (Mid, Center, Right)",
    //     oneConeEngageMobilityCenterRightMidCommand);

    // ********************************************************************
    // ********* 1 Cone + Engage + Mobility (High, Center, Right) *********
    // ********************************************************************

    Command oneConeEngageMobilityCenterRightHighCommand =
        oneConeCenterMobilityEngage(
            Position.CONE_HIGH_LEVEL, oneConeCenterRightPath, mobilityCenterEngagePath);
    // autoChooser.addOption(
    //     "1 Cone + Engage + Mobility (High, Center, Right)",
    //     oneConeEngageMobilityCenterRightHighCommand);

    // ********************************************************************
    // ********* 1 Cone + Engage + Mobility (Mid, Center, Left) ***********
    // ********************************************************************

    Command oneConeEngageMobilityCenterLeftMidCommand =
        oneConeCenterMobilityEngage(
            Position.CONE_MID_LEVEL, oneConeCenterLeftPath, mobilityCenterEngagePath);
    // autoChooser.addOption(
    //     "1 Cone + Engage + Mobility (Mid, Center, Left)",
    //     oneConeEngageMobilityCenterLeftMidCommand);

    // ********************************************************************
    // ********* 1 Cone + Engage + Mobility (High, Center, Left) **********
    // ********************************************************************

    Command oneConeEngageMobilityCenterLeftHighCommand =
        oneConeCenterMobilityEngage(
            Position.CONE_HIGH_LEVEL, oneConeCenterLeftPath, mobilityCenterEngagePath);
    // autoChooser.addOption(
    //     "1 Cone + Engage + Mobility (High, Center, Left)",
    //     oneConeEngageMobilityCenterLeftHighCommand);

    // ********************************************************************
    // ******************** Center 1.5 Cone (Mid,Center,Left) *************
    // ********************************************************************

    Command oneConeGrabEngageCenterLeftMidCommand =
        oneConeGrabCenterEngage(
            Position.CONE_MID_LEVEL, oneConeCenterLeftPath, oneConeCenterGrabEngage);
    autoChooser.addOption(
        "1 Cone + Grab + Engage (Mid, Center, Left)", oneConeGrabEngageCenterLeftMidCommand);

    // ********************************************************************
    // ******************** Center 1.5 Cone (High,Center,Left) ************
    // ********************************************************************
    Command oneConeGrabEngageCenterLeftHighCommand =
        oneConeGrabCenterEngage(
            Position.CONE_HIGH_LEVEL, oneConeCenterLeftPath, oneConeCenterGrabEngage);
    autoChooser.addOption(
        "1 Cone + Grab + Engage (High, Center, Left)", oneConeGrabEngageCenterLeftHighCommand);

    // ********************************************************************
    // ******************** Center 1.5 Cone (Mid,Center,Right) ************
    // ********************************************************************
    Command oneConeGrabEngageCenterRightMidCommand =
        oneConeGrabCenterEngage(
            Position.CONE_MID_LEVEL, oneConeCenterRightPath, oneConeCenterGrabEngage);
    autoChooser.addOption(
        "1 Cone + Grab + Engage (Mid, Center, Right)", oneConeGrabEngageCenterRightMidCommand);

    // ********************************************************************
    // ******************** Center 1.5 Cone (High,Center,Right) ***********
    // ********************************************************************
    Command oneConeGrabEngageCenterRightHighCommand =
        oneConeGrabCenterEngage(
            Position.CONE_HIGH_LEVEL, oneConeCenterRightPath, oneConeCenterGrabEngage);
    autoChooser.addOption(
        "1 Cone + Grab + Engage (High, Center, Left)", oneConeGrabEngageCenterRightHighCommand);

    // ********************************************************************
    // ******************** cableSide 1 cone (Mid, Stay) ******************
    // ********************************************************************
    Command oneConeCableSideMidStayCommand =
        oneConeStayOrMobility(Position.CONE_MID_LEVEL, cableSide1ConeStayPath, false);
    autoChooser.addOption("1 Cone cableSide(Mid, Stay)", oneConeCableSideMidStayCommand);

    // ****************************************************************
    // ******************** cableSide 1 cone (High, Stay) ******************
    // ********************************************************************

    Command oneConeCableSideHighStayCommand =
        oneConeStayOrMobility(Position.CONE_HIGH_LEVEL, cableSide1ConeStayPath, false);
    autoChooser.addOption("1 Cone cableSide(High, Stay)", oneConeCableSideHighStayCommand);

    // *******************************************************************
    // ******************** loadingSide 1 cone (Mid, Stay) ****************
    // ********************************************************************

    Command oneConeLoadingSideMidStayCommand =
        oneConeStayOrMobility(Position.CONE_MID_LEVEL, loadingSide1ConeStayPath, false);
    autoChooser.addOption("1 Cone loadingSide(Mid, Stay)", oneConeLoadingSideMidStayCommand);

    // *******************************************************************
    // ******************** loadingSide 1 cone (High, Stay) **************
    // ********************************************************************

    Command oneConeLoadingSideHighStayCommand =
        oneConeStayOrMobility(Position.CONE_HIGH_LEVEL, loadingSide1ConeStayPath, false);
    autoChooser.addOption("1 Cone loadingSide(High, Stay)", oneConeLoadingSideHighStayCommand);

    // ********************************************************************
    // ******************** cableSide 1 cone (Mid, Mobility) **************
    // ********************************************************************
    Command oneConeCableSideMidMobilityCommand =
        oneConeStayOrMobility(Position.CONE_MID_LEVEL, cableSide1ConeStayPath, true);
    autoChooser.addOption("1 Cone cableSide(Mid,Mobility)", oneConeCableSideMidMobilityCommand);

    // ****************************************************************
    // ******************** cableSide 1 cone (High, Mobility) *************
    // ********************************************************************

    Command oneConeCableSideHighMobilityCommand =
        oneConeStayOrMobility(Position.CONE_HIGH_LEVEL, cableSide1ConeStayPath, true);
    autoChooser.addOption("1 Cone cableSide(High, Mobility)", oneConeCableSideHighMobilityCommand);

    // *******************************************************************
    // ******************** loadingSide 1 cone (Mid, Mobility) ***********
    // *******************************************************************

    Command oneConeLoadingSideMidMobilityCommand =
        oneConeStayOrMobility(Position.CONE_MID_LEVEL, loadingSide1ConeStayPath, true);
    autoChooser.addOption(
        "1 Cone loadingSide(Mid, Mobility)", oneConeLoadingSideMidMobilityCommand);

    // *******************************************************************
    // ******************** loadingSide 1 cone (High, Mobility) ***********
    // ********************************************************************

    Command oneConeLoadingSideHighMobilityCommand =
        oneConeStayOrMobility(Position.CONE_HIGH_LEVEL, loadingSide1ConeStayPath, true);
    autoChooser.addOption(
        "1 Cone loadingSide(High, Mobility)", oneConeLoadingSideHighMobilityCommand);

    // ********************************************************************
    // ******************** cableSide 1.5 cone (High) *********************
    // ********************************************************************

    Command oneConeGrabCableSideHighCommand =
        oneConeGrabCableOrLoadingCommand(
            Position.CONE_HIGH_LEVEL, cableSide1ConeGrabPath, centerEngageBackwardsPath);
    autoChooser.addOption("CableSide 1.5 Cone (High)", oneConeGrabCableSideHighCommand);

    // ********************************************************************
    // ******************** cableSide 1.5 cone (Mid) **********************
    // ********************************************************************

    Command oneConeGrabCableSideMidCommand =
        oneConeGrabCableOrLoadingCommand(
            Position.CONE_MID_LEVEL, cableSide1ConeGrabPath, centerEngageBackwardsPath);
    autoChooser.addOption("CableSide 1.5 Cone (Mid)", oneConeGrabCableSideMidCommand);

    // ********************************************************************
    // ******************** loadingSide 1.5 cone (High) *******************
    // ********************************************************************

    Command oneConeGrabLoadingSideHighCommand =
        oneConeGrabCableOrLoadingCommand(
            Position.CONE_HIGH_LEVEL, loadingSide1ConeGrabPath, centerEngageBackwardsPath);
    autoChooser.addOption("LoadingSide 1.5 Cone (High)", oneConeGrabLoadingSideHighCommand);

    // ********************************************************************
    // ******************** loadingSide 1.5 cone (Mid) ********************
    // ********************************************************************

    Command oneConeGrabLoadingSideMidCommand =
        oneConeGrabCableOrLoadingCommand(
            Position.CONE_MID_LEVEL, loadingSide1ConeGrabPath, centerEngageBackwardsPath);
    autoChooser.addOption("LoadingSide 1.5 Cone (Mid)", oneConeGrabLoadingSideMidCommand);

    // ********************************************************************
    // ******************** cableSide 2 cone (High, High) *****************
    // ********************************************************************

    Command cableSideTwoConeHighHighCommand =
        twoConeCommand(
            Position.CONE_HIGH_LEVEL, cableSide2ConePath, GRID_1_NODE_3, Position.CONE_HIGH_LEVEL);
    autoChooser.addOption("CableSide 2 Cone(High,High)", cableSideTwoConeHighHighCommand);

    // ********************************************************************
    // ******************** cableSide 2 cone (Mid, Mid) *******************
    // ********************************************************************

    Command cableSideTwoConeMidMidCommand =
        twoConeCommand(
            Position.CONE_MID_LEVEL, cableSide2ConePath, GRID_1_NODE_3, Position.CONE_MID_LEVEL);
    autoChooser.addOption("CableSide 2 Cone(Mid,Mid)", cableSideTwoConeMidMidCommand);

    // ********************************************************************
    // ******************** loadingSide 2 cone (High, High) ***************
    // ********************************************************************

    Command loadingSideTwoConeHighHighCommand =
        twoConeCommand(
            Position.CONE_HIGH_LEVEL,
            loadingSide2ConePath,
            GRID_3_NODE_1,
            Position.CONE_HIGH_LEVEL);
    autoChooser.addOption("LoadingSide 2 Cone(High,High)", loadingSideTwoConeHighHighCommand);

    // ********************************************************************
    // ******************** loadingSide 2 cone (Mid, Mid) *****************
    // ********************************************************************

    Command loadingSideTwoConeMidMidCommand =
        twoConeCommand(
            Position.CONE_MID_LEVEL, loadingSide2ConePath, GRID_3_NODE_1, Position.CONE_MID_LEVEL);
    autoChooser.addOption("LoadingSide 2 Cone(Mid,Mid)", loadingSideTwoConeMidMidCommand);

    // ********************************************************************
    // ******************** cableSide 2 cone (Hybrid, High) ***************
    // ********************************************************************

    Command cableSideTwoConeHybridHighCommand =
        twoConeHybridStartCommand(
            cableSideHybridPath, cableSide2ConeHybridPath, GRID_1_NODE_3, Position.CONE_HIGH_LEVEL);
    autoChooser.addOption("CableSide 2 Cone(Hybrid,High)", cableSideTwoConeHybridHighCommand);

    // ********************************************************************
    // ******************** cableSide 2 cone (Hybrid, Mid) ****************
    // ********************************************************************

    Command cableSideTwoConeHybridMidCommand =
        twoConeHybridStartCommand(
            cableSideHybridPath, cableSide2ConeHybridPath, GRID_1_NODE_3, Position.CONE_MID_LEVEL);
    autoChooser.addOption("CableSide 2 Cone(Hybrid,Mid)", cableSideTwoConeHybridMidCommand);

    // ********************************************************************
    // ******************** loading 2 cone (Hybrid, High) *****************
    // ********************************************************************

    Command loadingSideTwoConeHybridHighCommand =
        twoConeHybridStartCommand(
            loadingSideHybridPath,
            loadingSide2ConeHybridPath,
            GRID_3_NODE_1,
            Position.CUBE_HIGH_LEVEL);
    autoChooser.addOption("LoadingSide 2 Cone(Hybrid,High)", loadingSideTwoConeHybridHighCommand);

    // ********************************************************************
    // ******************** loading 2 cone (Hybrid, Mid) ******************
    // ********************************************************************

    Command loadingSideTwoConeHybridMidCommand =
        twoConeHybridStartCommand(
            loadingSideHybridPath,
            loadingSide2ConeHybridPath,
            GRID_3_NODE_1,
            Position.CONE_MID_LEVEL);
    autoChooser.addDefaultOption(
        "LoadingSide 2 Cone(Hybrid,Mid)", loadingSideTwoConeHybridMidCommand);

    // ********************************************************************
    // ******************** cable 2.5 cone (High, High) *******************
    // ********************************************************************

    Command cableSideTwoConeGrabHighHighCommand =
        twoConeGrabCommand(
            Position.CONE_HIGH_LEVEL,
            cableSide2ConePath,
            GRID_1_NODE_3,
            Position.CONE_HIGH_LEVEL,
            cableSideGrabPath);
    autoChooser.addOption("CableSide 2.5 Cone(High,High)", cableSideTwoConeGrabHighHighCommand);

    // ********************************************************************
    // ******************** cable 2.5 cone (Mid, Mid) *********************
    // ********************************************************************

    Command cableSideTwoConeGrabMidMidCommand =
        twoConeGrabCommand(
            Position.CONE_MID_LEVEL,
            cableSide2ConePath,
            GRID_1_NODE_3,
            Position.CONE_MID_LEVEL,
            cableSideGrabPath);
    autoChooser.addOption("CableSide 2.5 Cone(Mid,Mid)", cableSideTwoConeGrabMidMidCommand);

    // ********************************************************************
    // ******************** loading 2.5 cone (High, High) *****************
    // ********************************************************************

    Command loadingSideTwoConeGrabHighHighCommand =
        twoConeGrabCommand(
            Position.CONE_HIGH_LEVEL,
            loadingSide2ConePath,
            GRID_3_NODE_1,
            Position.CONE_HIGH_LEVEL,
            loadingSideGrabPath);
    autoChooser.addOption("LoadingSide 2.5 Cone(High,High)", loadingSideTwoConeGrabHighHighCommand);

    // ********************************************************************
    // ******************** loading 2.5 cone (Mid, Mid) *******************
    // ********************************************************************

    Command loadingSideTwoConeGrabMidMidCommand =
        twoConeGrabCommand(
            Position.CONE_MID_LEVEL,
            loadingSide2ConePath,
            GRID_3_NODE_1,
            Position.CONE_MID_LEVEL,
            loadingSideGrabPath);
    autoChooser.addOption("LoadingSide 2.5 Cone(Mid,Mid)", loadingSideTwoConeGrabMidMidCommand);

    // ********************************************************************
    // ****************** Loading Side Get out of the Way *****************
    // ********************************************************************

    PathPlannerTrajectory getOutTheWay =
        PathPlanner.loadPath("LoadingSideGetOutTheWay", regularSpeed);
    Command loadingGetOutOfTheWay =
        Commands.sequence(new FollowPath(getOutTheWay, drivetrain, true, true));
    autoChooser.addOption("Loading Side Get out of the Way", loadingGetOutOfTheWay);

    // "auto" path for Tuning auto turn PID
    PathPlannerTrajectory autoTurnPidTuningPath =
        PathPlanner.loadPath("autoTurnPidTuning", 1.0, 1.0);
    Command autoTurnPidTuningCommand =
        new FollowPath(autoTurnPidTuningPath, drivetrain, true, true);
    autoChooser.addOption("Auto Turn PID Tuning", autoTurnPidTuningCommand);

    // start point auto
    PathPlannerTrajectory startPointPath =
        PathPlanner.loadPath(
            "StartPoint", config.getAutoMaxSpeed(), config.getAutoMaxAcceleration());
    Command startPoint =
        Commands.runOnce(
            () -> drivetrain.resetOdometry(startPointPath.getInitialState()), drivetrain);
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

    Shuffleboard.getTab("MAIN").add(autoChooser.getSendableChooser());

    if (TUNING_MODE) {
      PathPlannerServer.startServer(3061);
    }
  }

  private Command oneConeCenterEngage(
      Position position, PathPlannerTrajectory path, PathPlannerTrajectory engagePath) {
    return Commands.sequence(
        scoreGamePieceAuto(position),
        new SetElevatorPosition(elevator, Position.CONE_STORAGE, led),
        Commands.runOnce(elevator::stopElevator, elevator),
        new FollowPath(path, drivetrain, true, true),
        new AutoBalance(drivetrain, true, led, true));
  }

  private Command oneConeCenterMobilityEngage(
      Position position, PathPlannerTrajectory path, PathPlannerTrajectory engagePath) {
    return Commands.sequence(
        scoreGamePieceAuto(position),
        new SetElevatorPosition(elevator, Position.CONE_STORAGE, led),
        Commands.runOnce(elevator::stopElevator, elevator),
        new FollowPath(path, drivetrain, true, true),
        new FollowPath(engagePath, drivetrain, false, true),
        new AutoBalance(drivetrain, true, led, false));
  }

  private Command oneConeGrabCenterEngage(
      Position position, PathPlannerTrajectory path, PathPlannerTrajectory grabEngagePath) {
    return Commands.sequence(
        scoreGamePieceAuto(position),
        Commands.parallel(
            new SetElevatorPosition(elevator, Position.AUTO_STORAGE, led),
            new FollowPath(path, drivetrain, true, true)),
        Commands.runOnce(elevator::stopElevator, elevator),
        new RotateToAngle(
            drivetrain,
            () ->
                new Pose2d(
                    drivetrain.getPose().getX(),
                    drivetrain.getPose().getY(),
                    Rotation2d.fromDegrees(0.0))),
        new FollowPathWithEvents(
            new FollowPath(grabEngagePath, drivetrain, true, true),
            grabEngagePath.getMarkers(),
            autoEventMap),
        Commands.runOnce(elevator::stopElevator, elevator),
        new AutoBalance(drivetrain, true, led, false));
  }

  private Command oneConeStayOrMobility(
      Position position, PathPlannerTrajectory path, boolean mobility) {
    if (mobility) {
      return Commands.sequence(
          scoreGamePieceAuto(position),
          new SetElevatorPosition(elevator, Position.CONE_STORAGE, led),
          new FollowPath(path, drivetrain, true, true));
    } else {
      return Commands.sequence(
          Commands.runOnce(
              () ->
                  drivetrain.resetOdometry(
                      PathPlannerTrajectory.transformStateForAlliance(
                          path.getInitialState(), DriverStation.getAlliance())),
              drivetrain),
          scoreGamePieceAuto(position),
          new SetElevatorPosition(elevator, Position.CONE_STORAGE, led));
    }
  }

  private Command twoConeCommand(
      Position position, PathPlannerTrajectory path, Pose2d node, Position secondPosition) {
    return Commands.sequence(
        scoreGamePieceAuto(position),
        new FollowPathWithEvents(
            new FollowPath(path, drivetrain, true, true), path.getMarkers(), autoEventMap),
        Commands.parallel(
            driveAndStallCommand(node), new SetElevatorPosition(elevator, secondPosition, led)),
        new ReleaseGamePiece(manipulator, () -> elevator.getToggledToCone()));
  }

  private Command twoConeGrabCommand(
      Position position,
      PathPlannerTrajectory path,
      Pose2d node,
      Position secondPosition,
      PathPlannerTrajectory grabPath) {
    return Commands.sequence(
        scoreGamePieceAuto(position),
        new FollowPathWithEvents(
            new FollowPath(path, drivetrain, true, true), path.getMarkers(), autoEventMap),
        Commands.parallel(
            driveAndStallCommand(node), new SetElevatorPosition(elevator, secondPosition, led)),
        new ReleaseGamePiece(manipulator, () -> elevator.getToggledToCone()),
        new SetElevatorPosition(elevator, Position.AUTO_STORAGE, led),
        new FollowPathWithEvents(
            new FollowPath(grabPath, drivetrain, false, true),
            grabPath.getMarkers(),
            autoEventMap));
  }

  private Command oneConeGrabCableOrLoadingCommand(
      Position position, PathPlannerTrajectory path, PathPlannerTrajectory engagePath) {
    return Commands.sequence(
        scoreGamePieceAuto(position),
        new FollowPathWithEvents(
            new FollowPath(path, drivetrain, true, true), path.getMarkers(), autoEventMap));
  }

  private Command twoConeHybridStartCommand(
      PathPlannerTrajectory hybridStartPath,
      PathPlannerTrajectory path,
      Pose2d node,
      Position secondPosition) {
    return Commands.sequence(
        new FollowPath(hybridStartPath, drivetrain, true, true),
        new FollowPathWithEvents(
            new FollowPath(path, drivetrain, false, true), path.getMarkers(), autoEventMap),
        Commands.parallel(
            driveAndStallCommand(node), new SetElevatorPosition(elevator, secondPosition, led)),
        new ReleaseGamePiece(manipulator, () -> elevator.getToggledToCone()));
  }

  private void configureDrivetrainCommands() {

    oi.getToggleIntakeButton()
        .toggleOnTrue(
            Commands.either(
                Commands.sequence(
                    new DeployIntake(intake),
                    Commands.waitUntil(() -> intake.hasGamePiece()),
                    new RetractIntake(intake)),
                new RetractIntake(intake),
                intake::isRetracted));

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
    oi.getResetGyroButton().onTrue(Commands.runOnce(drivetrain::zeroGyroscope, drivetrain));

    // Ian release game Piece
    oi.getReleaseTriggerButton()
        .onTrue(
            Commands.sequence(
                new ReleaseGamePiece(manipulator, () -> elevator.getToggledToCone()),
                Commands.parallel(
                    new RetractIntake(intake),
                    new SetElevatorPosition(elevator, Position.CONE_STORAGE, led))));

    // reset pose based on vision
    oi.getResetPoseToVisionButton()
        .onTrue(
            Commands.runOnce(() -> drivetrain.resetPoseToVision(() -> vision.getBestRobotPose())));

    // x-stance
    oi.getXStanceButton().onTrue(Commands.runOnce(drivetrain::enableXstance, drivetrain));
    oi.getXStanceButton().onFalse(Commands.runOnce(drivetrain::disableXstance, drivetrain));

    // turbo
    // oi.getTurboButton().onTrue(Commands.runOnce(drivetrain::enableTurbo, drivetrain));
    // oi.getTurboButton().onFalse(Commands.runOnce(drivetrain::disableTurbo, drivetrain));
    oi.getTurboButton().onTrue(new SetElevatorPosition(elevator, Position.CONE_STORAGE, led));
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

    oi.getDisableArmBackupButton().onTrue(Commands.runOnce(elevator::stopElevator));
    oi.getMoveArmToShelfButton()
        .onTrue(
            new SetElevatorPosition(elevator, ElevatorConstants.Position.CONE_INTAKE_SHELF, led)
                .unless(() -> !elevator.isManualPresetEnabled()));
    oi.getMoveArmToStorageButton()
        .onTrue(
            new SetElevatorPosition(elevator, ElevatorConstants.Position.CONE_STORAGE, led)
                .unless(() -> !elevator.isManualPresetEnabled()));
    oi.getMoveArmToStorageBackupButton()
        .onTrue(new SetElevatorPosition(elevator, ElevatorConstants.Position.CONE_STORAGE, led));
    oi.getMoveArmToLowButton()
        .onTrue(
            new SetElevatorPosition(elevator, ElevatorConstants.Position.CONE_HYBRID_LEVEL, led)
                .unless(() -> !elevator.isManualPresetEnabled()));
    oi.getMoveArmToMidButton()
        .onTrue(
            new SetElevatorPosition(elevator, ElevatorConstants.Position.CONE_MID_LEVEL, led)
                .unless(() -> !elevator.isManualPresetEnabled()));
    oi.getMoveArmToHighButton()
        .onTrue(
            new SetElevatorPosition(elevator, ElevatorConstants.Position.CONE_HIGH_LEVEL, led)
                .unless(() -> !elevator.isManualPresetEnabled()));
    oi.getIntakeGroundConeButton()
        .onTrue(
            new SetElevatorPosition(elevator, ElevatorConstants.Position.CONE_INTAKE_FLOOR, led));

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

    // auto zero the elevator's extension
    oi.getAutoZeroExtensionButton().onTrue(Commands.runOnce(elevator::autoZeroExtension, elevator));

    oi.getDisableArmButton()
        .onTrue(Commands.sequence(Commands.runOnce(() -> elevator.stopElevator())));

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

    oi.getIntakeShootButton()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> intake.setRollerMotorPercentage(-1)),
                Commands.waitSeconds(1),
                new RetractIntake(intake)));

    // intake.setDefaultCommand(
    //     Commands.sequence(
    //         Commands.runOnce(
    //             () -> intake.setRotationMotorPercentage(oi.getIntakeDeployPower()), intake),
    //         Commands.runOnce(
    //             () -> intake.setRotationMotorPercentage(oi.getIntakeRetractPower()), intake)));

    // oi.getToggleIntakeRollerButton()
    //     .toggleOnTrue(
    //         Commands.either(
    //             Commands.runOnce(intake::stopRoller, intake),
    //             Commands.runOnce(intake::enableRoller, intake),
    //             intake::isRollerSpinning));

    // oi.getPositionIntakeToPushCubeCone()
    //     .onTrue(new SetIntakeState(intake, IntakeConstants.Position.PUSH_CONE_CUBE));
  }

  private void configureAutomatedSequenceCommands() {
    // move to grid / loading zone
    oi.getIntakeShelfGridSideButton()
        .onTrue(moveAndGrabGamePiece(Position.CONE_INTAKE_SHELF, DOUBLE_SUBSTATION_LOWER));
    oi.getIntakeShelfWallSideButton()
        .onTrue(moveAndGrabGamePiece(Position.CONE_INTAKE_SHELF, DOUBLE_SUBSTATION_UPPER));
    oi.getIntakeShelfGridSideBackupButton()
        .onTrue(moveAndGrabGamePiece(Position.CONE_INTAKE_SHELF, DOUBLE_SUBSTATION_LOWER));
    oi.getIntakeShelfWallSideBackupButton()
        .onTrue(moveAndGrabGamePiece(Position.CONE_INTAKE_SHELF, DOUBLE_SUBSTATION_UPPER));
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
            elevator, () -> SetElevatorPosition.convertGridRowToPosition(oi.getGridRow()), led);
    MoveToGrid moveToGridCommand =
        new MoveToGrid(drivetrain); // , 2.0), // replace 2.0 with the time to position the elevator
    // (e.g., setElevatorPosition.getTimeToPosition())

    /*
     * If move-to-grid is disabled, this command will never finish since TeleopSwerve never finishes.
     * In this case, the operator will have to manually drop the game piece, which will interrupt this command.
     */

    return Commands.sequence(
        Commands.parallel(
            // FIXME change to setPosition so we do not have to worry about hitting game elements
            new DeployIntake(intake),
            setElevatorPositionCommand,
            Commands.either(
                Commands.sequence(
                    Commands.deadline(
                        Commands.waitUntil(vision::posesInLine),
                        new TeleopSwerve(
                            drivetrain, oi::getTranslateX, oi::getTranslateY, oi::getRotate)),
                    Commands.runOnce(led::enableAutoLED),
                    moveToGridCommand,
                    new StallAgainstElement(
                        drivetrain,
                        moveToGridCommand.endPoseSupplier(),
                        true,
                        SQUARING_GRID_TIMEOUT_SECONDS)),
                Commands.parallel(
                    Commands.runOnce(led::enableTeleopLED),
                    new TeleopSwerve(
                        drivetrain, oi::getTranslateX, oi::getTranslateY, oi::getRotate)),
                () -> oi.getMoveToGridEnabledSwitch().getAsBoolean())),
        Commands.runOnce(led::enableTeleopLED));
  }

  private Command driveAndStallCommand(Pose2d moveToGridPosition) {
    return new StallAgainstElement(
        drivetrain,
        () -> Field2d.getInstance().mapPoseToCurrentAlliance(moveToGridPosition),
        true,
        SQUARING_AUTO_TIMEOUT_SECONDS);
  }

  private Command scoreGamePieceAuto(Position elevatorPosition) {
    Command stallOnGamePieceAuto = new GrabGamePiece(manipulator);
    Command setElevatorPositionToScoreAuto =
        new SetElevatorPosition(elevator, elevatorPosition, led);
    Command dropGamePieceAuto =
        new ReleaseGamePiece(manipulator, () -> elevator.getToggledToCone());

    return Commands.sequence(
        stallOnGamePieceAuto, setElevatorPositionToScoreAuto, dropGamePieceAuto);
  }

  private Command scoreGamePieceAutoHigh() {
    Command setElevatorPositionToScoreMidAuto =
        new SetElevatorPosition(elevator, Position.CONE_MID_LEVEL, led);
    Command dropGamePieceAuto =
        new ReleaseGamePiece(manipulator, () -> elevator.getToggledToCone());
    Command stallOnGamePieceAuto = new GrabGamePiece(manipulator);
    Command setElevatorPositionToScoreHighAuto =
        new SetElevatorPosition(elevator, Position.CONE_HIGH_LEVEL, led);

    return Commands.sequence(
        stallOnGamePieceAuto,
        setElevatorPositionToScoreMidAuto,
        Commands.waitSeconds(0.5),
        setElevatorPositionToScoreHighAuto,
        dropGamePieceAuto);
  }

  private Command collectGamePieceAuto() {
    return Commands.sequence(
        new SetElevatorPosition(elevator, Position.CONE_INTAKE_FLOOR, led),
        new GrabGamePiece(manipulator),
        new SetElevatorPosition(elevator, Position.AUTO_STORAGE, led));
  }

  private Command moveAndGrabGamePiece(Position elevatorPosition, Pose2d moveToGridPosition) {
    // Other commands will need to query how long the move to grid command will take (e.g., we want
    // to signal the human player x seconds before the robot reaching the game piece); so, we need
    // to store a reference to the command in a variable that can be passed along to other commands.
    // FIXME: pass the time to position the elevator
    Command setElevatorPositionCommandCollection =
        new SetElevatorPosition(elevator, elevatorPosition, led);
    MoveToLoadingZone moveToLoadingZoneCommand =
        new MoveToLoadingZone(drivetrain, moveToGridPosition);

    return Commands.sequence(
        new ReleaseGamePiece(manipulator, () -> elevator.getToggledToCone()),
        Commands.waitUntil(() -> manipulator.isOpened()),
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
            Commands.sequence(
                Commands.parallel(
                    setElevatorPositionCommandCollection,
                    Commands.either(
                        Commands.sequence(
                            Commands.deadline(
                                Commands.waitUntil(vision::posesInLine),
                                new TeleopSwerve(
                                    drivetrain,
                                    oi::getTranslateX,
                                    oi::getTranslateY,
                                    oi::getRotate)),
                            Commands.runOnce(led::enableAutoLED),
                            moveToLoadingZoneCommand),
                        Commands.sequence(
                            Commands.runOnce(led::enableTeleopLED),
                            new TeleopSwerve(
                                drivetrain, oi::getTranslateX, oi::getTranslateY, oi::getRotate)),
                        () -> oi.getMoveToGridEnabledSwitch().getAsBoolean())),
                Commands.either(
                    new StallAgainstElement(
                        drivetrain,
                        moveToLoadingZoneCommand.endPoseSupplier(),
                        false,
                        SQUARING_LOADING_ZONE_TIMEOUT_SECONDS),
                    Commands.none(),
                    () -> oi.getMoveToGridEnabledSwitch().getAsBoolean()))),
        Commands.runOnce(() -> led.changeTopStateColor(RobotStateColors.BLINKGREEN)),
        Commands.parallel(
            new SetElevatorPosition(elevator, Position.CONE_STORAGE, led),
            Commands.runOnce(led::enableTeleopLED),
            new TeleopSwerve(drivetrain, oi::getTranslateX, oi::getTranslateY, oi::getRotate)),
        Commands.runOnce(() -> led.changeTopStateColor(RobotStateColors.WHITE)));
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
    CommandScheduler.getInstance()
        .schedule(new SetElevatorPosition(elevator, Position.CONE_STORAGE, led));
  }

  public void teleopInit() {
    led.enableTeleopLED();
    CommandScheduler.getInstance()
        .schedule(new SetElevatorPosition(elevator, Position.CONE_STORAGE, led));
  }

  public void robotInit() {
    // led.changeAnimationTo(AnimationTypes.RAINBOW);
  }

  public void disabledPeriodic() {
    // FIXME: no longer a method; what should replace it?
    led.endMatchLEDs();
  }

  public static Pose2d adjustPoseForRobot(Pose2d pose) {
    return new Pose2d(
        pose.getX() + RobotConfig.getInstance().getRobotWidthWithBumpers() / 2,
        pose.getY(),
        pose.getRotation());
  }
}
