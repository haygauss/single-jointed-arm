package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  private final TalonFX armMotor;
  private final TalonFXSimState motorSim;
  private final SingleJointedArmSim armSim;
  private final Mechanism2d mechanism;
  private final MechanismLigament2d armLigament;
  private final MotionMagicVoltage motionMagicRequest;
  
  public ArmSubsystem() {
    // TEST SATIRI - BURAYA EKLENDİ!
    System.out.println("========================================");
    System.out.println("✅ ArmSubsystem BAŞLATILDI!");
    System.out.println("========================================");
    
    armMotor = new TalonFX(ArmConstants.MOTOR_CAN_ID);
    motorSim = armMotor.getSimState();
    configureTalonFX();
    
    armSim = new SingleJointedArmSim(
        DCMotor.getFalcon500(1),
        ArmConstants.GEAR_RATIO,
        ArmConstants.MOMENT_OF_INERTIA,
        ArmConstants.ARM_LENGTH_METERS,
        Units.degreesToRadians(ArmConstants.MIN_ANGLE_DEGREES),
        Units.degreesToRadians(ArmConstants.MAX_ANGLE_DEGREES),
        true,
        Units.degreesToRadians(ArmConstants.STARTING_ANGLE_DEGREES)
    );
    
    mechanism = new Mechanism2d(3, 3);
    MechanismRoot2d root = mechanism.getRoot("ArmPivot", 1.5, 0.5);
    armLigament = root.append(
        new MechanismLigament2d("Arm", ArmConstants.ARM_LENGTH_METERS,
            ArmConstants.STARTING_ANGLE_DEGREES, 6, new Color8Bit(Color.kYellow))
    );
    root.append(
        new MechanismLigament2d("Target", ArmConstants.ARM_LENGTH_METERS * 0.9,
            ArmConstants.STARTING_ANGLE_DEGREES, 2, new Color8Bit(Color.kGreen))
    );
    
    SmartDashboard.putData("Arm Mechanism", mechanism);
    motionMagicRequest = new MotionMagicVoltage(0);
    
    // DEBUG: Başlangıç değerlerini yazdır
    System.out.println("========================================");
    System.out.println("Motor başlangıç pozisyonu: " + armMotor.getPosition().getValueAsDouble() + " rotations");
    System.out.println("Motor başlangıç açısı: " + getAngleDegrees() + " degrees");
    System.out.println("Simülasyon başlangıç açısı: " + Units.radiansToDegrees(armSim.getAngleRads()) + " degrees");
    System.out.println("========================================");
  }
  
  private void configureTalonFX() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = ArmConstants.KP;
    slot0.kI = ArmConstants.KI;
    slot0.kD = ArmConstants.KD;
    slot0.kV = 0.0;
    slot0.kS = 0.0;
    slot0.kG = ArmConstants.KF;
    slot0.GravityType = GravityTypeValue.Arm_Cosine;
    config.Slot0 = slot0;
    
    MotionMagicConfigs motionMagic = new MotionMagicConfigs();
    motionMagic.MotionMagicCruiseVelocity = ArmConstants.MOTION_CRUISE_VELOCITY;
    motionMagic.MotionMagicAcceleration = ArmConstants.MOTION_ACCELERATION;
    motionMagic.MotionMagicJerk = 0;
    config.MotionMagic = motionMagic;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    
    armMotor.getConfigurator().apply(config);
    
    // Simülasyon için encoder pozisyonunu sıfırla
    armMotor.setPosition(0.0); // 0 derece = 0 rotasyon
    
    // Simülasyon encoder'ına da başlangıç değerini yaz
    motorSim.setRawRotorPosition(0.0);
  }
  
  public void setTargetPosition(double angleDegrees) {
    angleDegrees = Math.max(ArmConstants.MIN_ANGLE_DEGREES,
                            Math.min(ArmConstants.MAX_ANGLE_DEGREES, angleDegrees));
    double targetRotations = angleDegrees / 360.0;
    armMotor.setControl(motionMagicRequest.withPosition(targetRotations));
    SmartDashboard.putNumber("Arm/Target Position (deg)", angleDegrees);
  }
  
  public double getAngleDegrees() {
    return armMotor.getPosition().getValueAsDouble() * 360.0;
  }
  
  public boolean atTargetPosition() {
    double error = Math.abs(armMotor.getClosedLoopError().getValueAsDouble() * 360.0);
    return error < ArmConstants.POSITION_TOLERANCE_DEGREES;
  }
  
  public double getVelocityDegreesPerSecond() {
    return armMotor.getVelocity().getValueAsDouble() * 360.0;
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm/Current Position (deg)", getAngleDegrees());
    SmartDashboard.putNumber("Arm/Velocity (deg_s)", getVelocityDegreesPerSecond());
    SmartDashboard.putNumber("Arm/Motor Current (A)", armMotor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Arm/Motor Voltage (V)", armMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putBoolean("Arm/At Target", atTargetPosition());
    armLigament.setAngle(getAngleDegrees());
  }
  
  @Override
  public void simulationPeriodic() {
    // Motor simülasyonuna batarya voltajını ver
    motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    
    // Kol simülasyonuna motor voltajını ver
    armSim.setInputVoltage(motorSim.getMotorVoltage());
    
    // Simülasyonu güncelle (20ms = 0.02s)
    armSim.update(0.02);
    
    // Kol açısını radyandan dereceye çevir
    double armAngleDegrees = Units.radiansToDegrees(armSim.getAngleRads());
    
    // DÜZELTME: Motor pozisyonu = Kol açısı / 360 (rotasyon cinsinden)
    // Gear ratio TERS yönde çalışır: Motor 10 tur atınca kol 1 tur atar
    double motorPositionRotations = armAngleDegrees / 360.0;
    motorSim.setRawRotorPosition(motorPositionRotations);
    
    // Motor hızını da aynı şekilde hesapla
    double armVelocityDegreesPerSec = Units.radiansToDegrees(armSim.getVelocityRadPerSec());
    double motorVelocityRotationsPerSec = armVelocityDegreesPerSec / 360.0;
    motorSim.setRotorVelocity(motorVelocityRotationsPerSec);
    
    // DEBUG
    System.out.println("Sim açı: " + armAngleDegrees + "° → Motor pos: " + motorPositionRotations + " rot");
  }
}