package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import java.util.List;

public class LocationChooser {

    private final SendableChooser<ReefSticks> letterChooser = new SendableChooser<>();
//    private final SendableChooser<Boolean> climbModeChooser = new SendableChooser<>();
    
    private final RobotContainer r;

    public LocationChooser(RobotContainer r) {
        this.r = r;

        // Letter selection
        for (ReefSticks reef : ReefSticks.values()) {
            letterChooser.addOption(reef.name(), reef);
        }

        letterChooser.setDefaultOption("NONE", ReefSticks.NONE);

        // Climb mode
        // climbModeChooser.setDefaultOption("Off", false);
        // climbModeChooser.addOption("On", true);


        // Add choosers to SmartDashboard
        SmartDashboard.putData("Letter", letterChooser);
        //SmartDashboard.putData("Climb Mode", climbModeChooser);
    }

    public static enum ReefSticks {
        A, B, C, D, E, F, G, H, I, J, K, L, AB, CD, EF, GH, IJ, KL, PROCESSOR, BARGE, LEFT, RIGHT, CLOSEST, NONE
    }

    public Pose2d selectCoralStation() {
        switch (letterChooser.getSelected()) {
            case LEFT:
                return Locations.getLeftGatherStationFar();
            case RIGHT:
                return Locations.getRightGatherStationFar();
            case CLOSEST:
                return selectClosestCoralStation();

            case PROCESSOR:
                return Locations.getProcLoc();

            case BARGE:
                return Locations.getBargeLoc();

            case AB, CD, EF, GH, IJ, KL:
                return Locations.getAlgaeReefLocation(letterChooser.getSelected());

            default:
                return Locations.getReefLocation(letterChooser.getSelected());
        }
    }

    public Rotation2d selectGatherAngle() {
        return selectCoralStation().getRotation().plus(Rotation2d.fromDegrees(180));
    }

    public Rotation2d getAlignAngle() {
        ReefSticks selectedReefPos = letterChooser.getSelected();
        Rotation2d scoringPosition;
        switch (selectedReefPos) {
            case A:
            case B:
                scoringPosition = Rotation2d.fromDegrees(0);
                break;
            case C:
            case D:
                scoringPosition = Rotation2d.fromDegrees(60);
                break;
            case E:
            case F:
                scoringPosition = Rotation2d.fromDegrees(120);
                break;
            case G:
            case H:
                scoringPosition = Rotation2d.fromDegrees(180);
                break;
            case I:
            case J:
                scoringPosition = Rotation2d.fromDegrees(240);
                break;
            case K:
            case L:
                scoringPosition = Rotation2d.fromDegrees(300);
                break;
            case PROCESSOR:
                scoringPosition = Rotation2d.fromDegrees(270);
                break;
            case LEFT:
                scoringPosition = Rotation2d.fromDegrees(306);
                break;
            case RIGHT:
                scoringPosition = Rotation2d.fromDegrees(54);
                break;
            case BARGE:
                scoringPosition = Rotation2d.fromDegrees(0);
                break;
            default:
                scoringPosition = Rotation2d.fromDegrees(0);
        }
        return Locations.isBlue() ? scoringPosition : scoringPosition.plus(Rotation2d.fromDegrees(180));
    }

    public Pose2d selectClosestCoralStation() {
        return r.driveSubsystem.getPose().nearest(
            List.of(Locations.getLeftGatherStationFar(), Locations.getRightGatherStationFar())
        );
    }
}
