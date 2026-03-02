// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.Optional;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BatteryIdentifierSubsystem extends SubsystemBase {
  Optional<Integer> empty = Optional.empty(); // only create this once
  Optional<Integer> batteryId = empty;

  /** Creates a new BatteryIdentifierSubsystem. */
  public BatteryIdentifierSubsystem() {
    /*
    var visionThread = new Thread(this::apriltagVisionThreadProc);
    visionThread.setDaemon(true);
    visionThread.start();
    */

    setBatteryId(-1);
  }

  public Optional<Integer> getBatteryId() {
    return batteryId;
  }

  public void setBatteryId(int id) {
    if (id < 0) {
      batteryId = empty;
    } else {
      if (id != batteryId.orElse(-1)) {
        batteryId = Optional.of(id);
      }
    }
    DogLog.log("batteryId", id);
  }

  void apriltagVisionThreadProc() {
    var detector = new AprilTagDetector();
    // look for tag36h11, correct 1 error bit (hamming distance 1)
    // hamming 1 allocates 781KB, 2 allocates 27.4 MB, 3 allocates 932 MB
    // max of 1 recommended for RoboRIO 1, while hamming 2 is feasible on the RoboRIO 2
    detector.addFamily("tag36h11", 1);

    // Get the UsbCamera from CameraServer
    UsbCamera camera = CameraServer.startAutomaticCapture();
    // Set the resolution
    camera.setResolution(640, 480);

    // Get a CvSink. This will capture Mats from the camera
    CvSink cvSink = CameraServer.getVideo();
    // Setup a CvSource. This will send images back to the Dashboard
    CvSource outputStream = CameraServer.putVideo("Detected", 640, 480);

    // Mats are very memory expensive. Lets reuse these.
    var mat = new Mat();
    var grayMat = new Mat();

    // Instantiate once
    ArrayList<Integer> tags = new ArrayList<>();
    ArrayList<Integer> validTags = new ArrayList<>();
    var outlineColor = new Scalar(0, 255, 0);
    var crossColor = new Scalar(0, 0, 255);

    // This cannot be 'true'. The program will never exit if it is. This
    // lets the robot stop this thread when restarting robot code or
    // deploying.
    while (!Thread.interrupted()) {
      // Tell the CvSink to grab a frame from the camera and put it
      // in the source mat.  If there is an error notify the output.
      if (cvSink.grabFrame(mat) == 0) {
        // Send the output the error.
        outputStream.notifyError(cvSink.getError());
        // skip the rest of the current iteration
        continue;
      }

      Imgproc.cvtColor(mat, grayMat, Imgproc.COLOR_RGB2GRAY);

      AprilTagDetection[] detections = detector.detect(grayMat);

      // have not seen any tags yet
      tags.clear();
      validTags.clear();

      for (AprilTagDetection detection : detections) {
        // remember we saw this tag
        int id = detection.getId();
        tags.add(id);
        if (id > 100) {
          validTags.add(id);
        }

        // draw lines around the tag
        for (var i = 0; i <= 3; i++) {
          var j = (i + 1) % 4;
          var pt1 = new Point(detection.getCornerX(i), detection.getCornerY(i));
          var pt2 = new Point(detection.getCornerX(j), detection.getCornerY(j));
          Imgproc.line(mat, pt1, pt2, outlineColor, 2);
        }

        // mark the center of the tag
        var cx = detection.getCenterX();
        var cy = detection.getCenterY();
        var ll = 10;
        Imgproc.line(mat, new Point(cx - ll, cy), new Point(cx + ll, cy), crossColor, 2);
        Imgproc.line(mat, new Point(cx, cy - ll), new Point(cx, cy + ll), crossColor, 2);

        // identify the tag
        Imgproc.putText(
            mat,
            Long.toString(id),
            new Point(cx + ll, cy),
            Imgproc.FONT_HERSHEY_SIMPLEX,
            1,
            crossColor,
            3);
      }

      // put list of tags into log
      DogLog.log("apriltags/all", tags.stream().mapToLong(Integer::intValue).toArray());
      DogLog.log("apriltags/valid", validTags.stream().mapToLong(Integer::intValue).toArray());
  
      if (validTags.size() > 1) {
        setBatteryId(-1);
      } else if (validTags.size() == 1) {
        setBatteryId(validTags.get(0));
      } else {
        // we don't see any valid tags. just keep what we had
      }

      // Give the output stream a new image to display
      outputStream.putFrame(mat);
    }

    detector.close();
  }

}
