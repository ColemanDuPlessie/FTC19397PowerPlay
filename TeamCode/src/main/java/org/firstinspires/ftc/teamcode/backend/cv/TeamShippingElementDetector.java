package org.firstinspires.ftc.teamcode.backend.cv;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class TeamShippingElementDetector {

    public static enum POSITIONS {
        LEFT,
        MIDDLE,
        RIGHT
    }

    static final double INCHES_PER_METER = 3.28084 * 12;
    static final int TARGET_TAG = 19;

    private final AprilTagDetectionPipeline pipeline;
    private final OpenCvCamera camera;
    private final double centerWidth; // In inches
    private final double cameraOffset; // In inches. Positive = camera is to the right.

    private final double fx = 822.317; // 670.779069; // 1078.03779?
    private final double fy = 822.317; // 674.806148; // 1084.50988?
    private final double cx = 319.495; // 361.418117; // 580.850545?
    private final double cy = 242.502; // 153.041358; // 245.959325?

    public TeamShippingElementDetector(HardwareMap hardwareMap, double centerWidth, double cameraOffset) {
        this.centerWidth = centerWidth;
        this.cameraOffset = cameraOffset;

        pipeline = new AprilTagDetectionPipeline(3.5 / INCHES_PER_METER, fx, fy, cx, cy);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        FtcDashboard.getInstance().startCameraStream(camera, 0);

        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
    }

    public POSITIONS getTagPosition() {
        ArrayList<AprilTagDetection> detections = pipeline.getDetectionsUpdate();

        // If there's been a new frame...
        if (detections != null) {

            // We do see tags!
            if (detections.size() > 0) {

                for (AprilTagDetection detection : detections) {
                    if (detection.id == TARGET_TAG) {
                        double xpos = detection.pose.x*INCHES_PER_METER+cameraOffset;
                        if (xpos > centerWidth) {return POSITIONS.RIGHT;}
                        else if (xpos < -centerWidth) {return POSITIONS.LEFT;}
                        else {return POSITIONS.MIDDLE;}
                    }
                }
            }
        }
        return null;
    }

    public void close() {
        camera.stopStreaming();
    }

}
