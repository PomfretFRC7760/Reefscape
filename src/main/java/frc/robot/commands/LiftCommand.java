package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LiftSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LiftCommand extends Command {
    private final LiftSubsystem liftSubsystem;
    private int currentPreset = 0;
    private final BooleanSupplier dPadUp;
    private final BooleanSupplier dPadDown;
    private final BooleanSupplier dPadLeft;
    private final BooleanSupplier dPadRight;
    private boolean previousDPadUp = false;
    private boolean previousDPadDown = false;
    private final DoubleSupplier rightStickY;
    private final BooleanSupplier override;
    private boolean manualControlEnabled = false;

    public LiftCommand(BooleanSupplier dPadUp, BooleanSupplier dPadDown, BooleanSupplier dPadLeft, BooleanSupplier dPadRight, BooleanSupplier override, DoubleSupplier rightStickY,LiftSubsystem liftSubsystem ) {
        this.dPadUp = dPadUp;
        this.dPadDown = dPadDown;
        this.dPadLeft = dPadLeft;
        this.dPadRight = dPadRight;
        this.liftSubsystem = liftSubsystem;
        this.rightStickY = rightStickY;
        this.override = override;
        addRequirements(liftSubsystem);
    }

    @Override
    public void initialize() {
        liftSubsystem.resetPosition();
    }

    @Override
    public void execute() {
        // Toggle manual control mode
        if (override.getAsBoolean()) {
            if (manualControlEnabled) {
                manualControlEnabled = false;
            }
            else {
                manualControlEnabled = true;
            }
        }
        if (dPadDown.getAsBoolean() || dPadUp.getAsBoolean() || dPadLeft.getAsBoolean() || dPadRight.getAsBoolean()) {
            manualControlEnabled = false;
        }

        if (manualControlEnabled) {
            // Manual control mode
            double speed = rightStickY.getAsDouble();
            liftSubsystem.manualControl(speed);
        } else {
            // Preset mode
            boolean currentDPadUp = dPadUp.getAsBoolean();
            boolean currentDPadDown = dPadDown.getAsBoolean();
        
            // Check if dPadUp switches from false to true
            if (currentDPadUp && !previousDPadUp) {
                if (currentPreset >= 0 && currentPreset < 4) {
                    currentPreset++;
                }
            }
        
            // Check if dPadDown switches from false to true
            if (currentDPadDown && !previousDPadDown) {
                if (currentPreset > 0 && currentPreset <= 5) {
                    currentPreset--;
                }
            }

            if (dPadLeft.getAsBoolean()) {
                currentPreset = 0;
            }

            if (dPadRight.getAsBoolean()) {
                currentPreset = 5;
            }
        
            if (currentPreset == 0) {
                liftSubsystem.setLiftPosition(0.1);
            } else if (currentPreset == 1) {
                liftSubsystem.setLiftPosition(10);
            } else if (currentPreset == 2) {
                liftSubsystem.setLiftPosition(20);
            } else if (currentPreset == 3) {
                liftSubsystem.setLiftPosition(30);
            } else if (currentPreset == 4) {
                liftSubsystem.setLiftPosition(40);
            } else if (currentPreset == 5) {
                liftSubsystem.setLiftPosition(50);
            }
            liftSubsystem.updatePosition();
            // Update the previous states
            previousDPadUp = currentDPadUp;
            previousDPadDown = currentDPadDown;
        }
        
        // Update the SmartDashboard
        SmartDashboard.putBoolean("Manual Control", manualControlEnabled);
        SmartDashboard.putNumber("Current Preset", currentPreset);
    }
}
