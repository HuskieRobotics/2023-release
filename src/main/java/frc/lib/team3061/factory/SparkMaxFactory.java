package frc.lib.team3061.factory;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.revrobotics.CANSparkMax;

public class SparkMaxFactory {
    private static final int TIMEOUT_MS = 100;

    
    // explain
    private static int[] kPrimePeriods = 
        new int[] {255, 254, 253, 251, 247, 241, 239, 233, 229, 227, 223, 217, 211, 199, 197};
    

    public static class Configuration {
        public NeutralMode NEUTRAL_MODE = NeutralMode.Coast;

        // factory default
        public double NEUTRAL_DEADBAND = 0.04;

        public boolean ENABLE_SOFT_LIMIT = false;
        public boolean ENABLE_LIMIT_SWITCH = false;
        public int FORWARD_SOFT_LIMIT = 0;
        public int REVERSE_SOFT_LIMIT = 0;

        public boolean INVERTED = false;
        public boolean SENSOR_PHASE = false;
        public SensorInitializationStrategy SENSOR_INITIALIZATION_STRATEGY = SensorInitializationStrategy.BootToZero;

        public int CONTROL_FRAME_PERIOD_MS = 20; // 10
        public int MOTION_CONTROL_FRAME_PERIOD_MS = 100;

        public int GENERAL_STATUS_FRAME_RATE_MS = 10;
        public int FEEDBACK_STATUS_FRAME_RATE_MS = 49;
        public int QUAD_ENCODER_STATUS_FRAME_RATE_MS = kPrimePeriods[0];
        public int ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = kPrimePeriods[1];
        public int PULSE_WIDTH_STATUS_FRAME_RATE_MS = kPrimePeriods[2];
        public int MOTION_MAGIC_STATUS_FRAME_RATE_MS = kPrimePeriods[3];
        public int FEEDBACK_1_STATUS_FRAME_RATE_MS = kPrimePeriods[4];
        public int BASE_PIDF0_STATUS_FRAME_RATE_MS = kPrimePeriods[5];
        public int TURN_PIDF1_STATUS_FRAME_RATE_MS = kPrimePeriods[6];
        public int FEEDBACK_INTEGRATED_STATUS_FRAME_RATE_MS = kPrimePeriods[7];

        public SensorVelocityMeasPeriod VELOCITY_MEASUREMENT_PERIOD = SensorVelocityMeasPeriod.Period_100Ms;
        public int VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW = 64;

        public int MOTION_MAGIC_CRUISE_VELOCITY = 0;
        public int MOTION_MAGIC_ACCELERATION = 0;

        public int MOTION_PROFILE_CRUISE_VELOCITY = 0;
        public int MOTION_PROFILE_ACCELERATION = 0;

        public int MOTION_PROFILE_SLOTS = 0;
        public int MOTION_MAGIC_SLOTS = 0;

        public int MOTION_MAGIC_SMOOTH_TRAJECTORY = 0;
        public int MOTION_MAGIC_MINIMUM_OUTPUT = 0;
        public int MOTION_MAGIC_MAXIMUM_OUTPUT = 0;

        public int MOTION_MAGIC_ALLOWABLE_CLOSED_LOOP_ERROR = 0;
    }

    private static final Configuration DEFAULT_CONFIG = new Configuration();
    private static final Configuration FOLLOWER_CONFIG = new Configuration();

    static {
        FOLLOWER_CONFIG.CONTROL_FRAME_PERIOD_MS = 100;
        FOLLOWER_CONFIG.MOTION_CONTROL_FRAME_PERIOD_MS = 1000;
        FOLLOWER_CONFIG.GENERAL_STATUS_FRAME_RATE_MS = 1000;
        FOLLOWER_CONFIG.FEEDBACK_STATUS_FRAME_RATE_MS = 1000;
        FOLLOWER_CONFIG.QUAD_ENCODER_STATUS_FRAME_RATE_MS = 1000;
        FOLLOWER_CONFIG.ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 1000;
        FOLLOWER_CONFIG.PULSE_WIDTH_STATUS_FRAME_RATE_MS = 1000;
        FOLLOWER_CONFIG.ENABLE_SOFT_LIMIT = false;
    }

    public static CANSparkMax createDefaultSparkMax(int id, String canBusName) {
        return createSparkMax(id, canBusName, DEFAULT_CONFIG);
    }

    private static CANSparkMax createSparkMax(int id, String canBusName, Configuration config) {
        CANSparkMax sparkMax = new CANSparkMax(id, CANSparkMax.MotorType.kBrushless);
        sparkMax.restoreFactoryDefaults();
        
        sparkMax.set(0);
        sparkMax.setInverted(config.INVERTED);
        sparkMax.setControlFramePeriodMs(config.CONTROL_FRAME_PERIOD_MS);

        return sparkMax;

    }

}
