package frc.robot.commands;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GyroSubsystem;

public class GyroCommand extends Command {
    private final BooleanSupplier resetButton;
    private final GyroSubsystem gyroSubsystem;
    private long buttonPressStartTime = 0; 
    private boolean isButtonPressed = false;
    public GyroCommand(BooleanSupplier resetButton, GyroSubsystem gyroSubsystem){
        this.resetButton = resetButton;
        this.gyroSubsystem = gyroSubsystem;
        addRequirements(this.gyroSubsystem);
    }
    @Override
  public void initialize() {
    
  }
  @Override
  public void execute() {
    if (resetButton.getAsBoolean()) {
      if (!isButtonPressed) {
          buttonPressStartTime = System.currentTimeMillis();
          isButtonPressed = true; 
      }

      // 0.5 seconds hold, might make longer if i still manage to accidentally reset it
      if (isButtonPressed && (System.currentTimeMillis() - buttonPressStartTime >= 500)) {
        gyroSubsystem.gyroCalibration();  
        gyroSubsystem.gyroReset();
          isButtonPressed = false; 
      }
  } else {
      isButtonPressed = false;
  }
  }
}
