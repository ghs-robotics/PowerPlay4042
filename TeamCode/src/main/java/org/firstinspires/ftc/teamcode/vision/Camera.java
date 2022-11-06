package org.firstinspires.ftc.teamcode.vision;

import static android.os.SystemClock.sleep;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class Camera {

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    private final double fx = 578.272;
    private final double fy = 578.272;
    private final double cx = 402.145;
    private final double cy = 221.506;
    // UNITS ARE METERS
    private final double tagsize = 0.166; //This is 30% of original pdf size

    private int LEFT;
    private int MIDDLE;
    private int RIGHT;

    private AprilTagDetection tagOfInterest;
    private AprilTagDetectionPipeline aprilTagDetectionPipeline;

    private int colorDetection;
    private ColorDetectionPipeline colorDetectionPipeline;

    public Camera(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    public void setupColorDetection() {

        OpenCvCamera camera;

        //Tag IDs for 3 different park locations
        LEFT = 0; //Lime
        MIDDLE = 1; //Magenta
        RIGHT = 2; //Cyan
        colorDetection = -1;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        colorDetectionPipeline = new ColorDetectionPipeline(telemetry);

        camera.setPipeline(colorDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Error: Camera could not open");
                telemetry.update();
            }
        });

        telemetry.setMsTransmissionInterval(50);
    }

    public int runColorDetection() {
        int color = -1;
        ArrayList<Integer> currentDetections = colorDetectionPipeline.getColorDetections();

        if (currentDetections.size() != 0) {
            boolean colorFound = false;

            for (Integer detection : currentDetections) {
                if (detection == LEFT || detection == MIDDLE || detection == RIGHT) {
                    color = detection;
                    colorFound = true;
                }
            }


        if (colorFound) {
            telemetry.addLine("Color of interest is in sight!");
            telemetry.addLine("Spotted color #" + color);

        } else {
            telemetry.addLine("Don't see color of interest :(");

            if (color == -1) {
                telemetry.addLine("(The color has never been seen)");
            } else {
                telemetry.addLine("\nBut we HAVE seen the color before");
            }
        }

        } else {
            telemetry.addLine("Don't see color of interest :(");

            if (color == -1) {
                telemetry.addLine("(The color has never been seen)");
            } else {
                telemetry.addLine("\nBut we HAVE seen the color before");
            }

        }

        telemetry.update();
        sleep(20);


        return color;

    }

    public void setupTagDetection() {
        OpenCvCamera camera;

        // Lens intrinsics
        // UNITS ARE PIXELS
        // NOTE: this calibration is for the C920 webcam at 800x448.
        // You will need to do your own calibration for other configurations!
        double fx = 578.272;
        double fy = 578.272;
        double cx = 402.145;
        double cy = 221.506;
        // UNITS ARE METERS
        //0.166 IRL - This is 30% of original pdf size
        double tagsize = 0.166; //This is an eighth of the IRL size

        //Tag IDs for 3 different park locations
        LEFT = 11;
        MIDDLE = 12;
        RIGHT = 13;
        tagOfInterest = null;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Error: Camera could not open");
                telemetry.update();
            }
        });

        telemetry.setMsTransmissionInterval(50);
    }

    public AprilTagDetection runTagDetection() {

            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest, telemetry);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest, telemetry);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest, telemetry);
                }

            }

            telemetry.update();
            sleep(20);

    ////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////

        return tagOfInterest;
    }

    private void tagToTelemetry(AprilTagDetection detection, Telemetry telemetry)
    {
        final double FEET_PER_METER = 3.28084;
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }


}
