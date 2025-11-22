// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.TuneElevator;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;

public class RobotContainer {
    public static NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    private Elevator elevatorSub = new Elevator();
    private Pivot pivotSub = new Pivot();
    public RobotContainer() {
        configureBindings();
        DogLog.setEnabled(true);
    }

    private void configureBindings() {
        // CommandXboxController controller = new CommandXboxController(0);
        
        Joystick controller = new Joystick(0);
        JoystickButton L1Btn = new JoystickButton(controller, 4);
        JoystickButton L2Btn = new JoystickButton(controller, 3);
        JoystickButton L3Btn = new JoystickButton(controller, 6);
        JoystickButton L4Btn = new JoystickButton(controller, 5);
        JoystickButton percent = new JoystickButton(controller, 11);
        

        L1Btn.onTrue(elevatorSub.getPositionCommand(0));
        L2Btn.onTrue(elevatorSub.getPositionCommand(8));
        L3Btn.onTrue(elevatorSub.getPositionCommand(15));
        L4Btn.onTrue(elevatorSub.getPositionCommand(20));
        percent.onTrue(new InstantCommand(() -> this.elevatorSub.setPercentOutput(0.3)));

    }

    public Command getAutonomousCommand() {
      return Commands.print("No autonomous command configured");
    }

    public void tuneElevator(){
        // Command elevatorTuningCommand = new TuneElevator(elevatorSub, 10);
        // elevatorTuningCommand.schedule();
        this.elevatorSub.tuneElevator();
    }
}
