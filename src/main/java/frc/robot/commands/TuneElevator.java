package frc.robot.commands;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import static frc.robot.Constants.Elevator.*;
import static frc.robot.RobotContainer.ntInstance;


public class TuneElevator extends Command{
    private ProfiledPIDController elevatorController;
    private ElevatorFeedforward elevatorFeedForward;
    private Elevator elevatorSub;
    private double encoderGoalPosition;
    public TuneElevator(Elevator elevatorSub, double encoderGoalPosition){
        this.elevatorController = new ProfiledPIDController(
            kControllerkP, 
            kControllerkI, 
            kControllerkD, 
            new Constraints(kMaxVelocity, kMaxAcceleration)
        );
        this.elevatorFeedForward = new ElevatorFeedforward(
            kFeedForwardkS,
            kFeedForwardkG,
            kFeedForwardkV,
            kFeedForwardkA
        );
        this.elevatorSub = elevatorSub;
        this.encoderGoalPosition = encoderGoalPosition;
    }
    public void initialize(){
        this.elevatorController.setGoal(encoderGoalPosition);
        SmartDashboard.putData("Tunable PID Controller", elevatorController);
        SmartDashboard.putNumber("KP", this.elevatorController.getP());
        // SmartDashboard.putData(elevatorFeedForward);
    }

    public void execute(){
        double pidOutput = this.elevatorController.calculate(this.elevatorSub.getEncoderDistance());
        double feedforwardOutput = this.elevatorFeedForward.calculate(
            this.elevatorController.getSetpoint().velocity
        );
        this.elevatorSub.setVoltage(
            pidOutput + feedforwardOutput
        );

        SmartDashboard.putNumber("Measured Output", this.elevatorSub.getEncoderDistance());
        SmartDashboard.putNumber("Setpoint", this.elevatorController.getSetpoint().position);
        
    }

    public boolean isFinished(){
        // ends this command if not in test mode
        return !DriverStation.isTestEnabled();
    }
}
