package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.checkerframework.checker.units.qual.Angle;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class PositionFromAprilTag extends OpMode {
    AngleUnit outputUnitsAngle = AngleUnit.DEGREES;
    VisionPortal myVisionPortal;
    AprilTagProcessor myAprilTagProcessor;

    @Override
    public void init() {
        // Create the AprilTag processor and assign it to a variable.
        myAprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();

        VisionPortal.Builder myVisionPortalBuilder;

        // Create a new VisionPortal Builder object.
        myVisionPortalBuilder = new VisionPortal.Builder();

        // Specify the camera to be used for this VisionPortal.
        // Other choices are: RC phone camera and "switchable camera name".
        myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "cam_one"));

        // Add the AprilTag Processor to the VisionPortal Builder.
        myVisionPortalBuilder.addProcessor(myAprilTagProcessor);       // An added Processor is enabled by default.

        // Optional: set other custom features of the VisionPortal (4 are shown here).
        myVisionPortalBuilder.setCameraResolution(new Size(640, 480));  // Each resolution, for each camera model, needs calibration values for good pose estimation.
        myVisionPortalBuilder.setStreamFormat(VisionPortal.StreamFormat.YUY2);  // MJPEG format uses less bandwidth than the default YUY2.
        myVisionPortalBuilder.enableCameraMonitoring(true);      // Enable LiveView (RC preview).
        myVisionPortalBuilder.setAutoStopLiveView(true);     // Automatically stop LiveView (RC preview) when all vision processors are disabled.

        // Create a VisionPortal by calling build()
        myVisionPortal = myVisionPortalBuilder.build();
        myVisionPortal.setProcessorEnabled(myAprilTagProcessor, true);

    }

    @Override
    public void loop() {


        for (AprilTagDetection detection : myAprilTagProcessor.getDetections())  {
            int myAprilTagIdCode = detection.id;

            Orientation rot = Orientation.getOrientation(detection.rawPose.R, AxesReference.INTRINSIC, AxesOrder.XYZ, outputUnitsAngle);

            // Original source data
            double poseX = detection.rawPose.x;
            double poseY = detection.rawPose.y;
            double poseZ = detection.rawPose.z;

            double poseAX = rot.firstAngle;
            double poseAY = rot.secondAngle;
            double poseAZ = rot.thirdAngle;

            // conversion to fcs
            if (detection.rawPose != null)   {
                detection.ftcPose = new AprilTagPoseFtc();

                detection.ftcPose.x =  detection.rawPose.x;
                detection.ftcPose.y =  detection.rawPose.z;
                detection.ftcPose.z = -detection.rawPose.y;

                detection.ftcPose.yaw = -rot.firstAngle;
                detection.ftcPose.roll = rot.thirdAngle;
                detection.ftcPose.pitch = rot.secondAngle;

                detection.ftcPose.range = Math.hypot(detection.ftcPose.x, detection.ftcPose.y);
                detection.ftcPose.bearing = outputUnitsAngle.fromUnit(AngleUnit.RADIANS, Math.atan2(-detection.ftcPose.x, detection.ftcPose.y));
                detection.ftcPose.elevation = outputUnitsAngle.fromUnit(AngleUnit.RADIANS, Math.atan2(detection.ftcPose.z, detection.ftcPose.y));
            }
            telemetry.addData("ftc_pose position (x,y,z): ", "%2d, %2d, %2d", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z);
            telemetry.addData("ftc_pose orientation (yaw,roll,pitch): ",
                    "%2d, %2d, %2d",
                    detection.ftcPose.yaw,
                    detection.ftcPose.roll,
                    detection.ftcPose.pitch
            );

            telemetry.addData("ftc_pose extra (range, bearing, elevation",
                    "%2d, %2d, %2d",
                    detection.ftcPose.range,
                    detection.ftcPose.bearing,
                    detection.ftcPose.elevation
            );

        }

    }
}
