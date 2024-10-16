// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionTracking extends SubsystemBase {

    PhotonCamera m_camera;

    Pose2d robotPose; 

    Pose2d goalPose;

    public VisionTracking() {
        m_camera = new PhotonCamera("photonvision");

    }

    public void data(){

      var result = m_camera.getLatestResult();

       if(result.hasTargets()){
        List<PhotonTrackedTarget> targets = result.getTargets();
        PhotonTrackedTarget target = result.getBestTarget();
        double yaw = target.getYaw();
        double pitch = target.getPitch();
        double area = target.getArea();
        int targetID = target.getFiducialId();
        double poseAmbiguity = target.getPoseAmbiguity();
        Transform3d bestCameraToTarget = target.getBestCameraToTarget();
        Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();
        
        System.out.println(yaw );
        System.out.println( pitch);
        System.out.println( area);

        //TODO: get the printout to the rioLog

      }
    }

    public void pose(){
      AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

      var result = m_camera.getLatestResult();
      
      List<PhotonTrackedTarget> targets = result.getTargets();
        PhotonTrackedTarget target = result.getBestTarget();

      if (aprilTagFieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
    Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), distanceToGoal());
       }
    }

    public void distanceToGoal(){
    
    //use pitch for horizontal movement

    var result = m_camera.getLatestResult();

    PhotonTrackedTarget target = result.getBestTarget();

    // double pitch = target.getPitch();

    // }

    public double distanceTwoPose(Pose2d robot, Pose2d goal){

      robotPose = robot;

      goalPose = goal;

      double distanceToTarget = PhotonUtils.getDistanceToPose(robotPose, goalPose);

      return distanceToTarget;
    }

    // notes for calculating distances according to photonvision documentation: 
    //so the calculate distance to target is creating the space between the april tag to robot
    // and then the distance between two poses is actually the distance from robot to our goal (target position)

  @Override
  public void periodic() {
   
  }
}
