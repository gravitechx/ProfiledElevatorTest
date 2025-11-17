package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pivot extends SubsystemBase {
  // private SparkMax pivotMotor;
  // private RelativeEncoder pivotEncoder;
  // private DigitalInput sensor;
  private TalonFX pivotMotor;
  private TalonFXConfiguration config;
  private final PositionVoltage anglePOS = new PositionVoltage(0);

  public Pivot() {
    config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 35;
    config.CurrentLimits.SupplyCurrentLowerLimit = 60;
    config.CurrentLimits.SupplyCurrentLowerTime =  .1;
    config.Slot0.kP = 0.5;//Constants.PivotConstants.pivotKP;
    config.Slot0.kI = 0;//Constants.PivotConstants.pivotKI;
    config.Slot0.kD = 0;//Constants.PivotConstants.pivotKD;

    pivotMotor = new TalonFX(9);
    pivotMotor.getConfigurator().apply(config);
    pivotMotor.setPosition(0);
    pivotMotor.setControl(anglePOS.withPosition(3.3));

    // pivotMotor = new SparkMax(18, MotorType.kBrushless);
    // sensor = new DigitalInput(0);

    // pivotEncoder = pivotMotor.getEncoder();
    // pivotMotor.configure(Constants.CoralConstants.coralMotorConfig,
    // Constants.CoralConstants.coralResetMode,
    // Constants.CoralConstants.coralPersistMode);

    // pivotEncoder.setPosition(0);

  }

  // public Boolean hasCoral() {
  // return sensor.get();
  // }

  public void setMotor(double POS) {
    // pivotMotor.set(speed);
    pivotMotor.setControl(anglePOS.withPosition(POS));
  }

  public void kathunk(double kathunkVal) {
    if (kathunkVal == -4) {
      setMotor(17.5);
    }
  }

  public double getEncoderPosition() {
    // return pivotEncoder.getPosition();
    return pivotMotor.getPosition().getValueAsDouble();
  }

  public void periodic() {
    SmartDashboard.putNumber("Pivot encoder", getEncoderPosition());
  }
}