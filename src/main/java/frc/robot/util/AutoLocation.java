package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.AutoConfig;

public class AutoLocation {

    public static double robotWidth = 0.9398;
    public static double robotLength = 1.0414;
    public static Transform2d halfRobot = new Transform2d(robotLength / 2.0, 0, new Rotation2d());

    public static Transform2d halfRobotGatherLeftFar =
            new Transform2d(
                    robotLength / 2.0 + Units.inchesToMeters(1.5),
                    Units.inchesToMeters(16),
                    Rotation2d.kZero);
    public static Transform2d halfRobotGatherLeftClose =
            new Transform2d(
                    robotLength / 2.0 + Units.inchesToMeters(1.5),
                    Units.inchesToMeters(-16),
                    Rotation2d.kZero);
    public static Transform2d halfRobotGatherRightFar =
            new Transform2d(
                    robotLength / 2.0 + Units.inchesToMeters(1.5),
                    Units.inchesToMeters(-16),
                    Rotation2d.kZero);
    public static Transform2d halfRobotGatherRightClose =
            new Transform2d(
                    robotLength / 2.0 + Units.inchesToMeters(1.5),
                    Units.inchesToMeters(16),
                    Rotation2d.kZero);

    public static Transform2d halfRobotCoralRight =
            new Transform2d(
                    robotLength / 2.0 + Units.inchesToMeters(3.25),
                    Units.inchesToMeters(5.5),
                    Rotation2d.kZero);
    public static Transform2d halfRobotCoralLeft =
            new Transform2d(
                    robotLength / 2.0 + Units.inchesToMeters(3.5),
                    Units.inchesToMeters(-8),
                    Rotation2d.kZero);

    public static Transform2d halfRobotAlgae =
            new Transform2d(robotLength / 2.0 + Units.inchesToMeters(6), 0, Rotation2d.kZero);

    public static Transform2d halfRobotCoralLevel1 =
            new Transform2d(
                    robotLength / 2.0 + Units.inchesToMeters(7.5),
                    Units.inchesToMeters(0),
                    new Rotation2d());

    public static AprilTagFieldLayout tags =
            AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

    static Pose2d[] blueStarts = {
        new Pose2d(6, 3, Rotation2d.k180deg),
        new Pose2d(6, 3, Rotation2d.k180deg),
        new Pose2d(6, 3, Rotation2d.k180deg)
    };

    static Pose2d[] redStarts = {
        new Pose2d(12, 3, Rotation2d.kZero),
        new Pose2d(12, 3, Rotation2d.kZero),
        new Pose2d(12, 3, Rotation2d.kZero)
    };

    public static Pose2d getStartLoc(int idx) {
        if (isBlue()) {
            if (idx >= blueStarts.length) return null;
            return blueStarts[idx];
        } else {
            if (idx >= redStarts.length) return null;
            return redStarts[idx];
        }
    }

    public static Translation2d getReef() {
        Pose2d front, back;
        if (isBlue()) {
            front = tags.getTagPose(18).get().toPose2d();
            back = tags.getTagPose(21).get().toPose2d();
        } else {
            front = tags.getTagPose(7).get().toPose2d();
            back = tags.getTagPose(10).get().toPose2d();
        }

        return front.getTranslation().plus(back.getTranslation()).times(0.5);
    }

    // everyone hates this
    public static Pose2d getReefLocation(AutoConfig.ReefSticks position) {
        switch (position) {
            case B:
                if (isBlue()) {
                    return invert(tags.getTagPose(18).get().toPose2d().plus(halfRobotCoralRight));
                } else {
                    return invert(tags.getTagPose(7).get().toPose2d().plus(halfRobotCoralRight));
                }
            case A:
                if (isBlue()) {
                    return invert(tags.getTagPose(18).get().toPose2d().plus(halfRobotCoralLeft));
                } else {
                    return invert(tags.getTagPose(7).get().toPose2d().plus(halfRobotCoralLeft));
                }
            case D:
                if (isBlue()) {
                    return invert(tags.getTagPose(17).get().toPose2d().plus(halfRobotCoralRight));
                } else {
                    return invert(tags.getTagPose(8).get().toPose2d().plus(halfRobotCoralRight));
                }
            case C:
                if (isBlue()) {
                    return invert(tags.getTagPose(17).get().toPose2d().plus(halfRobotCoralLeft));
                } else {
                    return invert(tags.getTagPose(8).get().toPose2d().plus(halfRobotCoralLeft));
                }
            case F:
                if (isBlue()) {
                    return invert(tags.getTagPose(22).get().toPose2d().plus(halfRobotCoralRight));
                } else {
                    return invert(tags.getTagPose(9).get().toPose2d().plus(halfRobotCoralRight));
                }
            case E:
                if (isBlue()) {
                    return invert(tags.getTagPose(22).get().toPose2d().plus(halfRobotCoralLeft));
                } else {
                    return invert(tags.getTagPose(9).get().toPose2d().plus(halfRobotCoralLeft));
                }
            case H:
                if (isBlue()) {
                    return invert(tags.getTagPose(21).get().toPose2d().plus(halfRobotCoralRight));
                } else {
                    return invert(tags.getTagPose(10).get().toPose2d().plus(halfRobotCoralRight));
                }
            case G:
                if (isBlue()) {
                    return invert(tags.getTagPose(21).get().toPose2d().plus(halfRobotCoralLeft));
                } else {
                    return invert(tags.getTagPose(10).get().toPose2d().plus(halfRobotCoralLeft));
                }
            case J:
                if (isBlue()) {
                    return invert(tags.getTagPose(20).get().toPose2d().plus(halfRobotCoralRight));
                } else {
                    return invert(tags.getTagPose(11).get().toPose2d().plus(halfRobotCoralRight));
                }
            case I:
                if (isBlue()) {
                    return invert(tags.getTagPose(20).get().toPose2d().plus(halfRobotCoralLeft));
                } else {
                    return invert(tags.getTagPose(11).get().toPose2d().plus(halfRobotCoralLeft));
                }
            case L:
                if (isBlue()) {
                    return invert(tags.getTagPose(19).get().toPose2d().plus(halfRobotCoralRight));
                } else {
                    return invert(tags.getTagPose(6).get().toPose2d().plus(halfRobotCoralRight));
                }
            case K:
                if (isBlue()) {
                    return invert(tags.getTagPose(19).get().toPose2d().plus(halfRobotCoralLeft));
                } else {
                    return invert(tags.getTagPose(6).get().toPose2d().plus(halfRobotCoralLeft));
                }
            default:
                return null;
        }
    }

    public static Pose2d getAlgaeReefLocation(AutoConfig.ReefSticks position) {
        switch (position) {
            case A:
            case B:
            default:
                if (isBlue()) {
                    return invert(tags.getTagPose(18).get().toPose2d().plus(halfRobotAlgae));
                } else {
                    return invert(tags.getTagPose(7).get().toPose2d().plus(halfRobotAlgae));
                }
            case C:
            case D:
                if (isBlue()) {
                    return invert(tags.getTagPose(17).get().toPose2d().plus(halfRobotAlgae));
                } else {
                    return invert(tags.getTagPose(8).get().toPose2d().plus(halfRobotAlgae));
                }
            case E:
            case F:
                if (isBlue()) {
                    return invert(tags.getTagPose(22).get().toPose2d().plus(halfRobotAlgae));
                } else {
                    return invert(tags.getTagPose(9).get().toPose2d().plus(halfRobotAlgae));
                }
            case G:
            case H:
                if (isBlue()) {
                    return invert(tags.getTagPose(21).get().toPose2d().plus(halfRobotAlgae));
                } else {
                    return invert(tags.getTagPose(10).get().toPose2d().plus(halfRobotAlgae));
                }
            case I:
            case J:
                if (isBlue()) {
                    return invert(tags.getTagPose(20).get().toPose2d().plus(halfRobotAlgae));
                } else {
                    return invert(tags.getTagPose(11).get().toPose2d().plus(halfRobotAlgae));
                }
            case K:
            case L:
                if (isBlue()) {
                    return invert(tags.getTagPose(19).get().toPose2d().plus(halfRobotAlgae));
                } else {
                    return invert(tags.getTagPose(6).get().toPose2d().plus(halfRobotAlgae));
                }
        }
    }

    public static Pose2d getLeftGatherStationFar() {
        if (!isBlue()) {
            return (tags.getTagPose(1).get().toPose2d().plus(halfRobotGatherLeftFar));
        } else {
            return (tags.getTagPose(13).get().toPose2d().plus(halfRobotGatherLeftFar));
        }
    }

    public static Pose2d getRightGatherStationFar() {
        if (!isBlue()) {
            return (tags.getTagPose(2).get().toPose2d().plus(halfRobotGatherRightFar));
        } else {
            return (tags.getTagPose(12).get().toPose2d().plus(halfRobotGatherRightFar));
        }
    }

    public static Pose2d getLeftGatherStationClose() {
        if (!isBlue()) {
            return invert(tags.getTagPose(1).get().toPose2d().plus(halfRobotGatherLeftClose));
        } else {
            return invert(tags.getTagPose(13).get().toPose2d().plus(halfRobotGatherLeftClose));
        }
    }

    public static Pose2d getRightGatherStationClose() {
        if (!isBlue()) {
            return invert(tags.getTagPose(2).get().toPose2d().plus(halfRobotGatherRightClose));
        } else {
            return invert(tags.getTagPose(12).get().toPose2d().plus(halfRobotGatherRightClose));
        }
    }

    public static Pose2d getTag7() {
        Pose2d tag = tags.getTagPose(7).get().toPose2d().plus(halfRobotCoralLeft);
        return invert(tag);
    }

    public static boolean isBlue() {
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
    }

    public static Pose2d invert(Pose2d in) {
        return new Pose2d(in.getTranslation(), in.getRotation().plus(Rotation2d.k180deg));
    }

    public static Pose2d invertAndOffset(Pose2d in) {
        if (isBlue()) {
            return new Pose2d(
                new Translation2d(in.getX() - 0.75, in.getY()), // Offset x by -0.75m
                in.getRotation().plus(Rotation2d.k180deg) // Invert rotation
            );
        } else {
            return new Pose2d(
                new Translation2d(in.getX() + 0.75, in.getY()), // Offset x by +0.75m
                in.getRotation().plus(Rotation2d.k180deg) // Invert rotation
            );
        }
    }
    

    public static Pose2d getProcLoc() {
        if (isBlue()) {
            return (tags.getTagPose(3).get().toPose2d().plus(halfRobot));
        } else {
            return (tags.getTagPose(16).get().toPose2d().plus(halfRobot));
        }
    }

    public static Pose2d getBargeLoc() {
        if (isBlue()) {
            return invertAndOffset(tags.getTagPose(14).get().toPose2d().plus(halfRobot));
        } else {
            return invertAndOffset(tags.getTagPose(5).get().toPose2d().plus(halfRobot));
        }
    }

    public static Pose2d getStartingLoc(AutoConfig.Start position) {
        if (isBlue()) {
            switch(position) {
                case LEFT:
                    return new Pose2d(7.5565, 6.19, Rotation2d.k180deg);
                case RIGHT:
                    return new Pose2d(7.5565, 1.86, Rotation2d.k180deg);
                case CENTER:
                default:
                    return new Pose2d(7.5565, 4.025, Rotation2d.k180deg);
            }
        } else {
            switch(position) {
                case LEFT:
                    return new Pose2d(9.9935, 1.86, Rotation2d.kZero);
                case RIGHT:
                    return new Pose2d(9.9935, 6.19, Rotation2d.kZero);
                case CENTER:
                default:
                    return new Pose2d(9.9935, 4.025, Rotation2d.kZero);
            }
        }
    }
}