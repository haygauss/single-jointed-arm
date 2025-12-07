package frc.robot;

public final class Constants {
  public static final class ArmConstants {
    // Motor ve Mekanik Özellikler
    public static final int MOTOR_CAN_ID = 10;
    public static final double GEAR_RATIO = 10.0;
    public static final double ARM_LENGTH_METERS = 1.0;
    public static final double ARM_MASS_KG = 5.0;
    
    // Açı Limitleri
    public static final double STARTING_ANGLE_DEGREES = 0.0;
    public static final double MIN_ANGLE_DEGREES = -70.0;
    public static final double MAX_ANGLE_DEGREES = 100.0;
    
    // PID Gains - Güçlendirilmiş Değerler
    public static final double KP = 10.0;   // Proportional gain (0.5 → 10.0)
    public static final double KI = 0.0;    // Integral gain
    public static final double KD = 0.5;    // Derivative gain (0.1 → 0.5)
    public static final double KF = 0.5;    // Feed-forward gain (0.05 → 0.5)
    
    // Motion Magic Parametreleri
    public static final double MOTION_CRUISE_VELOCITY = 50.0;      // Rotations per second
    public static final double MOTION_ACCELERATION = 100.0;        // Rotations per second²
    public static final double POSITION_TOLERANCE_DEGREES = 2.0;
    
    // Falcon 500 Motor Özellikleri
    public static final double FALCON_FREE_SPEED_RPM = 6380.0;
    public static final double FALCON_STALL_TORQUE_NM = 4.69;
    public static final double FALCON_STALL_CURRENT_AMPS = 257.0;
    
    // Moment of Inertia (Atalet Momenti)
    public static final double MOMENT_OF_INERTIA = 
        (1.0 / 3.0) * ARM_MASS_KG * ARM_LENGTH_METERS * ARM_LENGTH_METERS;
  }
  
  public static final class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
  }
}