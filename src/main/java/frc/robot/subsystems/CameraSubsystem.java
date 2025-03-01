package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;


public class CameraSubsystem extends SubsystemBase {
    private final VideoSink server;
    private final UsbCamera camera1;
    private final UsbCamera camera2;
    public CameraSubsystem() {
        camera1 = CameraServer.startAutomaticCapture(0);
        camera2 = CameraServer.startAutomaticCapture(1);
        server = CameraServer.getServer();
    }
    public void SwitchCameras(double camera) {
        if (camera == 0) {
            server.setSource(camera1);
        }
        else if (camera == 1) {
            server.setSource(camera2);
        }
    }
}
