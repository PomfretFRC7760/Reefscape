package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LiftSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LiftCommand extends Command {
    private final LiftSubsystem liftSubsystem;
    private int currentPreset = 0;
    private final BooleanSupplier dPadUp;
    private final BooleanSupplier dPadDown;
    private final BooleanSupplier dPadLeft;
    private final BooleanSupplier dPadRight;
    private final DoubleSupplier rightStickY;
    private boolean manualControlEnabled = false;

    public LiftCommand(BooleanSupplier dPadUp, BooleanSupplier dPadDown, BooleanSupplier dPadLeft, BooleanSupplier dPadRight, DoubleSupplier rightStickY, LiftSubsystem liftSubsystem) {
        this.dPadUp = dPadUp;
        this.dPadDown = dPadDown;
        this.dPadLeft = dPadLeft;
        this.dPadRight = dPadRight;
        this.liftSubsystem = liftSubsystem;
        this.rightStickY = rightStickY;
        addRequirements(liftSubsystem);

        
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // Toggle manual control mode
        if (dPadDown.getAsBoolean() || dPadUp.getAsBoolean() || dPadLeft.getAsBoolean() || dPadRight.getAsBoolean()) {
            manualControlEnabled = false;
        }

        if (manualControlEnabled) {
            // Manual control mode
            double speed = rightStickY.getAsDouble();
            liftSubsystem.manualOverrideControl(speed);
        } else {
            // Preset mode
            // boolean currentDPadUp = dPadUp.getAsBoolean();
            // boolean currentDPadDown = dPadDown.getAsBoolean();
        
            // Check if dPadUp switches from false to true
            if (dPadDown.getAsBoolean()) {
                currentPreset = 0;
            }

            // else if (dPadDown.getAsBoolean()) {
            //     currentPreset = 3;
            // }

            else if (dPadLeft.getAsBoolean()) {
                currentPreset = 1;
            }

            else if (dPadRight.getAsBoolean()) {
                currentPreset = 2;
            }
        
            if (currentPreset == 0) {
                liftSubsystem.setLiftPosition(0.1);
            } else if (currentPreset == 1) {
                liftSubsystem.setLiftPosition(21.5);
            } else if (currentPreset == 2) {
                liftSubsystem.setLiftPosition(53);
            }
            // } else if (currentPreset == 3) {
            //     liftSubsystem.setLiftPosition(30);
            // } else if (currentPreset == 4) {
            //     liftSubsystem.setLiftPosition(40);
            // } else if (currentPreset == 5) {
            //     liftSubsystem.setLiftPosition(53);
            // }
            liftSubsystem.updatePosition();
            // Update the previous states
            // previousDPadUp = currentDPadUp;
            // previousDPadDown = currentDPadDown;
        }
        
        // Update the SmartDashboard
        SmartDashboard.putNumber("Current Preset", currentPreset);
    }

    public void resetLiftPosition() {
        liftSubsystem.resetPosition();
    }

    public void manualControlSwitch() {
        manualControlEnabled = true;
    }
}
