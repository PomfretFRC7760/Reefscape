package frc.robot.commands;

import java.util.function.BooleanSupplier;

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
    public LiftCommand(BooleanSupplier dPadUp, BooleanSupplier dPadDown, BooleanSupplier dPadLeft, BooleanSupplier dPadRight, LiftSubsystem liftSubsystem) {
        this.dPadUp = dPadUp;
        this.dPadDown = dPadDown;
        this.dPadLeft = dPadLeft;
        this.dPadRight = dPadRight;
        this.liftSubsystem = liftSubsystem;
        addRequirements(liftSubsystem);
    }

    @Override
    public void initialize() {
        liftSubsystem.resetPosition(); 
    }


    @Override
    public void execute() {
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
    
        // Update the current preset value on the dashboard
        SmartDashboard.putNumber("Current Preset", currentPreset);
    }

}
