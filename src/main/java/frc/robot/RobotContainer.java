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
import frc.robot.commands.SetElevatorPositionBeforeRetraction;
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
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.leds.LEDs;
import frc.robot.subsystems.leds.LEDs.RobotStateColors;
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

            intake = new Intake(new IntakeIO() {});

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
            intake = new Intake(new IntakeIO() {});
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
                Commands.runOnce(drivetrain::disableXstance),
                new TeleopSwerve(drivetrain, oi::getTranslateX, oi::getTranslateY, oi::getRotate)));
  }

  /** Use this method to define your commands for autonomous mode. */
  private void configureAutoCommands() {
    autoEventMap.put("event1", Commands.print("passed marker 1"));
    autoEventMap.put("event2", Commands.print("passed marker 2"));
    autoEventMap.put(
        "bring in elevator", new SetElevatorPosition(elevator, Position.AUTO_STORAGE, led));
    autoEventMap.put("prepare to intake cone", collectGamePieceAuto());
    autoEventMap.put(
        "set elevator auto position",
        new SetElevatorPosition(elevator, Position.CONE_STORAGE, led));
    autoEventMap.put("collect game piece", collectGamePieceAuto());

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
            Commands.runOnce(elevator::stopRotation, elevator),
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
            Commands.runOnce(elevator::stopRotation, elevator),
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
            new AutoBalance(drivetrain, true, led));
    autoChooser.addOption(
        "Hybrid Cone Center Position + Mobility + Engage",
        hybridConeCenterPositionMobilityEngageCommand);

    // ********************************************************************
    // *************** 1 Cone + Engage (Center, Left) *********************
    // ********************************************************************

    PathPlannerTrajectory centerEngagePath = PathPlanner.loadPath("CenterEngage", engageSpeed);
    Command oneConeEngageCenterLeftCommand =
        Commands.sequence(
            newOneConeCenterLeftCommand(),
            Commands.runOnce(elevator::stopRotation, elevator),
            new FollowPath(centerEngagePath, drivetrain, false, true),
            new AutoBalance(drivetrain, true, led));
    autoChooser.addOption("1 Cone + Engage (Center, Left)", oneConeEngageCenterLeftCommand);

    // ********************************************************************
    // *************** 1 Cone + Engage (Center, Right) ********************
    // ********************************************************************

    Command oneConeEngageCenterRightCommand =
        Commands.sequence(
            newOneConeCenterRightCommand(),
            Commands.runOnce(elevator::stopRotation, elevator),
            new FollowPath(centerEngagePath, drivetrain, false, true),
            new AutoBalance(drivetrain, true, led));
    autoChooser.addOption("1 Cone + Engage (Center, Right)", oneConeEngageCenterRightCommand);

    // ********************************************************************
    // ********* 1 Cone + Engage + Mobility (Center, Left) ****************
    // ********************************************************************

    PathPlannerTrajectory centerMobilityPath = PathPlanner.loadPath("MobilityCenter", engageSpeed);
    Command oneConeEngageMobilityCenterLeftCommand =
        Commands.sequence(
            newOneConeCenterLeftCommand(),
            Commands.runOnce(elevator::stopRotation, elevator),
            new FollowPath(centerMobilityPath, drivetrain, false, true),
            new RotateToAngle(
                drivetrain,
                () ->
                    new Pose2d(
                        drivetrain.getPose().getX(),
                        drivetrain.getPose().getY(),
                        Rotation2d.fromDegrees(180.0))),
            new FollowPath(farSideEngagePath, drivetrain, false, true),
            new AutoBalance(drivetrain, true, led));
    autoChooser.addOption(
        "1 Cone + Engage + Mobility (Center, Left)", oneConeEngageMobilityCenterLeftCommand);

    // ********************************************************************
    // ********* 1 Cone + Engage + Mobility (Center, Right, High) *********
    // ********************************************************************

    Command oneConeEngageMobilityCenterRightCommand =
        Commands.sequence(
            newOneConeCenterRightCommand(),
            Commands.runOnce(elevator::stopRotation, elevator),
            new FollowPath(centerMobilityPath, drivetrain, false, true),
            new RotateToAngle(
                drivetrain,
                () ->
                    new Pose2d(
                        drivetrain.getPose().getX(),
                        drivetrain.getPose().getY(),
                        Rotation2d.fromDegrees(180.0))),
            new FollowPath(farSideEngagePath, drivetrain, false, true),
            new AutoBalance(drivetrain, true, led));
    autoChooser.addOption(
        "1 Cone + Engage + Mobility (Center, Right)", oneConeEngageMobilityCenterRightCommand);

    // ********************************************************************
    // ******************** Cable Side 2 Cone *************************
    // ********************************************************************

    autoChooser.addOption("Cable Side 2 Cone", newCableSide2ConeCommand());

    // ********************************************************************
    // ************ Cable Side 2 Cone Rotate in Place *******************
    // ********************************************************************

    autoChooser.addOption(
        "Cable Side 2 Cone Rotate-in-Place", newCableSide2ConeRotateInPlaceCommand());

    // ********************************************************************
    // ******************** Cable Side 2 Cone + Engage ********************
    // ********************************************************************

    PathPlannerTrajectory cableSidePreRotatePath =
        PathPlanner.loadPath("CableSidePreRotate", regularSpeed);
    PathPlannerTrajectory cableSideEngagePath =
        PathPlanner.loadPath("CableSideEngage", engageSpeed);
    Command cableSide2ConeEngageCommand =
        Commands.sequence(
            newCableSide2ConeCommand(),
            Commands.parallel(
                new FollowPath(cableSidePreRotatePath, drivetrain, false, true),
                new SetElevatorPosition(elevator, Position.CONE_STORAGE, led)),
            new RotateToAngle(
                drivetrain,
                () ->
                    new Pose2d(
                        drivetrain.getPose().getX(),
                        drivetrain.getPose().getY(),
                        Rotation2d.fromDegrees(0.0))),
            new FollowPath(cableSideEngagePath, drivetrain, false, true),
            Commands.runOnce(elevator::stopRotation, elevator),
            new AutoBalance(drivetrain, true, led));
    autoChooser.addOption("Cable Side 2 Cone Engage", cableSide2ConeEngageCommand);

    // ********************************************************************
    // ******** Cable Side 2 Cone + Engage Rotate in Place ****************
    // ********************************************************************

    Command cableSide2ConeEngageRotateInPlaceCommand =
        Commands.sequence(
            newCableSide2ConeRotateInPlaceCommand(),
            Commands.parallel(
                new FollowPath(cableSidePreRotatePath, drivetrain, false, true),
                new SetElevatorPosition(elevator, Position.CONE_STORAGE, led)),
            new RotateToAngle(
                drivetrain,
                () ->
                    new Pose2d(
                        drivetrain.getPose().getX(),
                        drivetrain.getPose().getY(),
                        Rotation2d.fromDegrees(0.0))),
            new FollowPath(cableSideEngagePath, drivetrain, false, true),
            Commands.runOnce(elevator::stopRotation, elevator),
            new AutoBalance(drivetrain, true, led));
    autoChooser.addOption(
        "Cable Side 2 Cone Engage Rotate-in-Place", cableSide2ConeEngageRotateInPlaceCommand);

    // ********************************************************************
    // ******************** Loading Side 2 Cone ***************************
    // ********************************************************************

    autoChooser.addOption("Loading Side 2 Cone", newLoadingSide2ConeCommand());

    // ********************************************************************
    // ************ Loading Side 2 Cone Rotate in Place *******************
    // ********************************************************************

    autoChooser.addOption(
        "Loading Side 2 Cone Rotate-in-Place", newLoadingSide2ConeRotateInPlaceCommand());

    // ********************************************************************
    // ***************** Loading Side 2 Cone + Engage *********************
    // ********************************************************************

    PathPlannerTrajectory loadingSidePreRotatePath =
        PathPlanner.loadPath("LoadingSidePreRotate", regularSpeed);
    PathPlannerTrajectory loadingSideEngagePath =
        PathPlanner.loadPath("LoadingSideEngage", engageSpeed);
    Command loadingSide2ConeEngageCommand =
        Commands.sequence(
            newLoadingSide2ConeCommand(),
            Commands.parallel(
                new FollowPath(loadingSidePreRotatePath, drivetrain, false, true),
                new SetElevatorPosition(elevator, Position.CONE_STORAGE, led)),
            new RotateToAngle(
                drivetrain,
                () ->
                    new Pose2d(
                        drivetrain.getPose().getX(),
                        drivetrain.getPose().getY(),
                        Rotation2d.fromDegrees(0.0))),
            new FollowPath(loadingSideEngagePath, drivetrain, false, true),
            Commands.runOnce(elevator::stopRotation, elevator),
            new AutoBalance(drivetrain, true, led));
    autoChooser.addOption("Loading Side 2 Cone + Engage", loadingSide2ConeEngageCommand);

    // ********************************************************************
    // ********** Loading Side 2 Cone + Engage Rotate in Place ************
    // ********************************************************************

    Command loadingSide2ConeEngageRotateInPlaceCommand =
        Commands.sequence(
            newLoadingSide2ConeRotateInPlaceCommand(),
            Commands.parallel(
                new SetElevatorPosition(elevator, Position.CONE_STORAGE, led),
                Commands.sequence(
                    new FollowPath(loadingSidePreRotatePath, drivetrain, false, true),
                    new RotateToAngle(
                        drivetrain,
                        () ->
                            new Pose2d(
                                drivetrain.getPose().getX(),
                                drivetrain.getPose().getY(),
                                Rotation2d.fromDegrees(0.0))),
                    new FollowPath(loadingSideEngagePath, drivetrain, false, true))),
            Commands.runOnce(elevator::stopRotation, elevator),
            new AutoBalance(drivetrain, true, led));
    autoChooser.addOption(
        "Loading Side 2 Cone + Engage Rotate-in-Place", loadingSide2ConeEngageRotateInPlaceCommand);

    // ********************************************************************
    // ****************** Loading Side Get out of the Way *****************
    // ********************************************************************

    PathPlannerTrajectory getOutTheWay =
        PathPlanner.loadPath("LoadingSideGetOutTheWay", regularSpeed);
    Command loadingGetOutOfTheWay =
        Commands.sequence(new FollowPath(getOutTheWay, drivetrain, true, true));
    autoChooser.addOption("Loading Side Get out of the Way", loadingGetOutOfTheWay);

    PathPlannerTrajectory loadingSide1ConeStay =
        PathPlanner.loadPath("LoadingSide1ConeStay", 1.0, 1.0);
    Command loadingSide1ConeStayCommand =
        Commands.sequence(
            Commands.runOnce(
                () -> drivetrain.resetOdometry(loadingSide1ConeStay.getInitialState()), drivetrain),
            scoreGamePieceAuto(Position.CONE_MID_LEVEL));
    autoChooser.addOption("loadingSide1ConeStay", loadingSide1ConeStayCommand);

    Command loadingSide1ConeStayHighCommand =
        Commands.sequence(
            Commands.runOnce(
                () -> drivetrain.resetOdometry(loadingSide1ConeStay.getInitialState()), drivetrain),
            scoreGamePieceAutoHigh());
    autoChooser.addOption("loadingSide1ConeHighStay", loadingSide1ConeStayHighCommand);

    // "auto" path for Tuning auto turn PID
    PathPlannerTrajectory autoTurnPidTuningPath =
        PathPlanner.loadPath("autoTurnPidTuning", 1.0, 1.0);
    Command autoTurnPidTuningCommand =
        new FollowPath(autoTurnPidTuningPath, drivetrain, true, true);
    autoChooser.addOption("Auto Turn PID Tuning", autoTurnPidTuningCommand);

    // "auto" path with no holonomic rotation
    PathPlannerTrajectory noHolonomicRotationPath =
        PathPlanner.loadPath("constantHolonomicRotationPath", 1.0, 1.0);
    Command noHolonomicRotationCommand =
        new FollowPath(noHolonomicRotationPath, drivetrain, true, true);
    autoChooser.addOption("No Holonomic Rotation", noHolonomicRotationCommand);

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

  private Command newOneConeCenterLeftCommand() {
    PathPlannerTrajectory oneConeEngageCenterLeftPath =
        PathPlanner.loadPath("1ConeEngageCenterLeft", overCableConnector);
    return Commands.sequence(
        scoreGamePieceAuto(Position.CONE_MID_LEVEL),
        new SetElevatorPosition(elevator, Position.CONE_STORAGE, led),
        new FollowPath(oneConeEngageCenterLeftPath, drivetrain, true, true),
        new RotateToAngle(
            drivetrain,
            () ->
                new Pose2d(
                    drivetrain.getPose().getX(),
                    drivetrain.getPose().getY(),
                    Rotation2d.fromDegrees(0.0))),
        Commands.runOnce(elevator::stopRotation, elevator));
  }

  private Command newOneConeCenterRightCommand() {
    PathPlannerTrajectory oneConeEngageCenterRightPath =
        PathPlanner.loadPath("1ConeEngageCenterRight", overCableConnector);
    return Commands.sequence(
        scoreGamePieceAuto(Position.CONE_MID_LEVEL),
        new SetElevatorPosition(elevator, Position.CONE_STORAGE, led),
        new FollowPath(oneConeEngageCenterRightPath, drivetrain, true, true),
        new RotateToAngle(
            drivetrain,
            () ->
                new Pose2d(
                    drivetrain.getPose().getX(),
                    drivetrain.getPose().getY(),
                    Rotation2d.fromDegrees(0.0))),
        Commands.runOnce(elevator::stopRotation, elevator));
  }

  private Command newCableSide2ConeCommand() {
    List<PathPlannerTrajectory> cableSide2ConePath =
        PathPlanner.loadPathGroup(
            "CableSide2Cone",
            overCableConnector,
            overCableConnector,
            regularSpeed,
            regularSpeed,
            overCableConnector,
            regularSpeed);
    return Commands.sequence(
        scoreGamePieceAuto(Position.CONE_MID_LEVEL),
        // new SetElevatorPosition(elevator, Position.AUTO_STORAGE),
        new FollowPathWithEvents(
            new FollowPath(cableSide2ConePath.get(0), drivetrain, true, true),
            cableSide2ConePath.get(0).getMarkers(),
            autoEventMap),
        new FollowPathWithEvents(
            new FollowPath(cableSide2ConePath.get(1), drivetrain, false, true),
            cableSide2ConePath.get(1).getMarkers(),
            autoEventMap),
        new FollowPathWithEvents(
            new FollowPath(cableSide2ConePath.get(2), drivetrain, false, true),
            cableSide2ConePath.get(2).getMarkers(),
            autoEventMap),
        new FollowPathWithEvents(
            new FollowPath(cableSide2ConePath.get(3), drivetrain, false, true),
            cableSide2ConePath.get(3).getMarkers(),
            autoEventMap),
        new FollowPathWithEvents(
            new FollowPath(cableSide2ConePath.get(4), drivetrain, false, true),
            cableSide2ConePath.get(4).getMarkers(),
            autoEventMap),
        new FollowPathWithEvents(
            new FollowPath(cableSide2ConePath.get(5), drivetrain, false, true),
            cableSide2ConePath.get(5).getMarkers(),
            autoEventMap),
        Commands.parallel(
            driveAndStallCommand(FieldRegionConstants.GRID_1_NODE_3),
            new SetElevatorPosition(elevator, Position.CONE_MID_LEVEL, led)),
        new ReleaseGamePiece(manipulator));
  }

  private Command newLoadingSide2ConeCommand() {
    PathPlannerTrajectory loadingSide2ConePath =
        PathPlanner.loadPath("LoadingSide2Cone", regularSpeed);
    return Commands.sequence(
        scoreGamePieceAuto(Position.CONE_MID_LEVEL),
        new FollowPathWithEvents(
            new FollowPath(loadingSide2ConePath, drivetrain, true, true),
            loadingSide2ConePath.getMarkers(),
            autoEventMap),
        Commands.parallel(
            driveAndStallCommand(FieldRegionConstants.GRID_3_NODE_1),
            new SetElevatorPosition(elevator, Position.CONE_MID_LEVEL, led)),
        new ReleaseGamePiece(manipulator));
  }

  private Command newLoadingSide2ConeRotateInPlaceCommand() {
    PathPlannerTrajectory loadingSide2ConePreRotatePath =
        PathPlanner.loadPath("LoadingSide2ConePreRotate", regularSpeed);
    PathPlannerTrajectory loadingSide2ConeRotateInPlacePath =
        PathPlanner.loadPath("LoadingSide2ConeRotateInPlace", regularSpeed);
    PathPlannerTrajectory loadingSide2ConeRotateInPlaceReturnPath =
        PathPlanner.loadPath("LoadingSide2ConeRotateInPlaceReturn", regularSpeed);
    return Commands.sequence(
        scoreGamePieceAuto(Position.CONE_MID_LEVEL),
        new SetElevatorPosition(elevator, Position.AUTO_STORAGE, led),
        Commands.sequence(
            new FollowPath(loadingSide2ConePreRotatePath, drivetrain, true, true),
            new RotateToAngle(
                drivetrain,
                () ->
                    new Pose2d(
                        drivetrain.getPose().getX(),
                        drivetrain.getPose().getY(),
                        Rotation2d.fromDegrees(0.0))),
            new FollowPathWithEvents(
                new FollowPath(loadingSide2ConeRotateInPlacePath, drivetrain, false, true),
                loadingSide2ConeRotateInPlacePath.getMarkers(),
                autoEventMap),
            new RotateToAngle(
                drivetrain,
                () ->
                    new Pose2d(
                        drivetrain.getPose().getX(),
                        drivetrain.getPose().getY(),
                        Rotation2d.fromDegrees(180.0))),
            new FollowPath(loadingSide2ConeRotateInPlaceReturnPath, drivetrain, false, true)),
        Commands.parallel(
            driveAndStallCommand(FieldRegionConstants.GRID_3_NODE_1),
            new SetElevatorPosition(elevator, Position.CONE_MID_LEVEL, led)),
        new ReleaseGamePiece(manipulator));
  }

  private Command newCableSide2ConeRotateInPlaceCommand() {
    PathPlannerTrajectory cableSide2ConePreRotatePath =
        PathPlanner.loadPath("CableSide2ConePreRotate", regularSpeed);
    PathPlannerTrajectory cableSide2ConeRotateInPlacePath =
        PathPlanner.loadPath("CableSide2ConeRotateInPlace", regularSpeed);
    PathPlannerTrajectory cableSide2ConeRotateInPlacePathReturn =
        PathPlanner.loadPath("CableSide2ConeRotateInPlaceReturn", regularSpeed);
    return Commands.sequence(
        scoreGamePieceAuto(Position.CONE_MID_LEVEL),
        new SetElevatorPosition(elevator, Position.AUTO_STORAGE, led),
        Commands.sequence(
            new FollowPath(cableSide2ConePreRotatePath, drivetrain, true, true),
            new RotateToAngle(
                drivetrain,
                () ->
                    new Pose2d(
                        drivetrain.getPose().getX(),
                        drivetrain.getPose().getY(),
                        Rotation2d.fromDegrees(0.0))),
            new FollowPathWithEvents(
                new FollowPath(cableSide2ConeRotateInPlacePath, drivetrain, false, true),
                cableSide2ConeRotateInPlacePath.getMarkers(),
                autoEventMap),
            new SetElevatorPosition(elevator, Position.AUTO_STORAGE, led),
            new RotateToAngle(
                drivetrain,
                () ->
                    new Pose2d(
                        drivetrain.getPose().getX(),
                        drivetrain.getPose().getY(),
                        Rotation2d.fromDegrees(180.0))),
            new FollowPath(cableSide2ConeRotateInPlacePathReturn, drivetrain, false, true)),
        Commands.parallel(
            driveAndStallCommand(FieldRegionConstants.GRID_1_NODE_3),
            new SetElevatorPosition(elevator, Position.CONE_MID_LEVEL, led)),
        new ReleaseGamePiece(manipulator));
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
    oi.getResetGyroButton().onTrue(Commands.runOnce(drivetrain::zeroGyroscope, drivetrain));

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
    //NEW RELEASE GAME PIECE
    oi.getAutoBalanceButton().onTrue(new ReleaseGamePiece(manipulator));
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

    oi.getMoveArmToChuteButton().onTrue(Commands.runOnce(elevator::stopElevator));
    oi.getMoveArmToShelfButton()
        .onTrue(
            new SetElevatorPosition(elevator, ElevatorConstants.Position.CONE_INTAKE_SHELF, led)
                .unless(() -> !elevator.isManualPresetEnabled()));
    oi.getMoveArmToStorageButton()
        .onTrue(
            new SetElevatorPosition(elevator, ElevatorConstants.Position.CONE_STORAGE, led)
                .unless(() -> !elevator.isManualPresetEnabled()));
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
        .onTrue(new SetElevatorPosition(elevator, ElevatorConstants.Position.CONE_STORAGE, led));

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
            Commands.either(
                Commands.runOnce(led::enableAutoLED),
                Commands.runOnce(led::enableTeleopLED),
                () -> oi.getMoveToGridEnabledSwitch().getAsBoolean()),
            setElevatorPositionCommand,
            Commands.either(
                Commands.sequence(
                    moveToGridCommand,
                    new DriveToPose(drivetrain, moveToGridCommand.endPoseSupplier()),
                    new StallAgainstElement(
                        drivetrain,
                        moveToGridCommand.endPoseSupplier(),
                        SQUARING_GRID_TIMEOUT_SECONDS)),
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
            drivetrain,
            () -> Field2d.getInstance().mapPoseToCurrentAlliance(moveToGridPosition),
            SQUARING_AUTO_TIMEOUT_SECONDS));
  }

  private Command scoreGamePieceAuto(Position elevatorPosition) {
    Command setElevatorPositionToScoreAuto =
        new SetElevatorPosition(elevator, elevatorPosition, led);
    Command dropGamePieceAuto = new ReleaseGamePiece(manipulator);
    Command stallOnGamePieceAuto = new GrabGamePiece(manipulator);

    return Commands.sequence(
        stallOnGamePieceAuto, setElevatorPositionToScoreAuto, dropGamePieceAuto);
  }

  private Command scoreGamePieceAutoHigh() {
    Command setElevatorPositionToScoreMidAuto =
        new SetElevatorPosition(elevator, Position.CONE_MID_LEVEL, led);
    Command dropGamePieceAuto = new ReleaseGamePiece(manipulator);
    Command stallOnGamePieceAuto = new GrabGamePiece(manipulator);
    Command setElevatorPositionToScoreHighAuto =
        new SetElevatorPosition(elevator, Position.CONE_HIGH_LEVEL, led);

    return Commands.sequence(
        stallOnGamePieceAuto,
        setElevatorPositionToScoreMidAuto,
        Commands.waitSeconds(1.0),
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
                            drivetrain,
                            moveToLoadingZoneCommand.endPoseSupplier(),
                            SQUARING_LOADING_ZONE_TIMEOUT_SECONDS)),
                    Commands.none(),
                    () -> oi.getMoveToGridEnabledSwitch().getAsBoolean()))),
        Commands.runOnce(() -> led.changeTopStateColor(RobotStateColors.BLINKGREEN)),
        Commands.parallel(
            Commands.sequence(
                new SetElevatorPositionBeforeRetraction(elevator, elevatorPosition, led),
                new SetElevatorPosition(elevator, Position.CONE_STORAGE, led)),
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
    led.setBlueOrangeStaticLed();
  }

  public static Pose2d adjustPoseForRobot(Pose2d pose) {
    return new Pose2d(
        pose.getX() + RobotConfig.getInstance().getRobotWidthWithBumpers() / 2,
        pose.getY(),
        pose.getRotation());
  }
}
