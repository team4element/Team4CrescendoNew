// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;


public class VisionTracking extends SubsystemBase {

    PhotonCamera m_camera;

    public VisionTracking() {
        m_camera = new PhotonCamera("photovision");

    }


  @Override
  public void periodic() {
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
        
        System.out.println(yaw, pitch, area);


    }
  }
}
