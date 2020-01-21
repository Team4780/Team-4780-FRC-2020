package frc.robot.vision;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cameraserver.CameraServer;
import io.github.pseudoresonance.pixy2api.links.I2CLink;

public class Cameras {

    private static UsbCamera cam0 = null;
    private static UsbCamera cam1 = null;
    private static PixyCamera pixy = null;

    public static void setup() {
        initCameras();
        pixy = new PixyCamera(new I2CLink());

    }

    public static void light(boolean state) {
        pixy.light(state);
    }

    public static void run() {
        pixy.run();
    }

    public static PixyCamera getPixyCamera() {
        return pixy;
    }

    public static void initCameras() {
        cam0 = CameraServer.getInstance().startAutomaticCapture("cam0", 0);
        cam1 = CameraServer.getInstance().startAutomaticCapture("cam1", 1);
        if (cam0 != null) {
            cam0.setVideoMode(PixelFormat.kMJPEG, 320, 240, 30);
        if (cam1 != null) {
            cam1.setVideoMode(PixelFormat.kMJPEG, 320, 240, 30);
        }   
      }
   }
}