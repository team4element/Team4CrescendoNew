package frc.robot.Subsystems;

import java.util.Collections;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.numbers.N7;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PusherConstants;
import frc.robot.Constants.ShooterConstants;

public class poseEstimator extends SubsystemBase{

    private final PhotonCamera camera = new PhotonCamera("photonvision");

    PhotonPipelineResult result = camera.getLatestResult();

     PhotonTrackedTarget target = result.getBestTarget();
  
    double yaw = target.getYaw();
    double pitch = target.getPitch();
    double area = target.getArea();
    int targetID = target.getFiducialId();
    double poseAmbiguity = target.getPoseAmbiguity();

    // private final CommandSwerveDrivetrain commandSwerveDrivetrain;

    // static double m_degree;

    // public static String direction;

    // double m_distance; //the space between the limelight and robot

    //  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight"); //making limelight show on the dashboard
    //  NetworkTableEntry ty = table.getEntry("ty"); //making the y-value limelight data 

    //gets the values from the limelight 
    
//     PhotonPipelineResult result = camera.getLatestResult();
  
//      public void cameraPeriodic(){
//       if (result.hasTargets()){
//         var target = result.getBestTarget();

//         var yaw = target.getYaw();
//         var pitch = target.getPitch();
//         var camToTarget = target.getBestCameraToTarget();

//         }
//     }

//    public static double degreesToRadians(double degree){

//     m_degree = degree;

//     double radian = ((degree * Math.PI)/180);

//     return radian;
//    }

//    private static final edu.wpi.first.math.Vector<N7> stateStdDevs = VecBuilder.fill(0.05,0.05, Units.degreesToRadians(0), 0.05,0.05,0.05,0.05);

//    private static final edu.wpi.first.math.Vector<N5> localMeasurementStdDevs = VecBuilder.fill(Units.degreesToRadians(0.05), 0.05,0.05,0.05,0.05);

//    private static final edu.wpi.first.math.Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(0.5,0.5, Units.degreesToRadians(0.5));

//    private final SwerveDrivePoseEstimator
//    //<N7, N7, N5> 
//    poseEstimator;

//    private final Field2d field2d = new Field2d();

//    private double previousPipelineTimestamp = 0;

        public poseEstimator ()
           //Photoncamera camera, CommandSwerveDrivetrain commandSwerveDrivetrain
             {
            //this.camera = camera;
            // this.commandSwerveDrivetrain = commandSwerveDrivetrain;
            // this.drivetrainSubsystem = drivetrainSubsystem;

            // ShuffleboardTab tab = Shuffleboard.getTab("Vision");

             System.out.println("hi Jamie");

            // poseEstimator = new SwerveDrivePoseEstimator
            // //<N7, N7, N5>
            // (
            //     Nat.N7(),
            //     Nat.N7(),
            //     Nat.N5(),
            //     commandSwerveDrivetrain.getGryroscopeRotation(),
            //     commandSwerveDrivetrain.getDrivetrainState().getSwerveModulePosition(),
            //     new Pose2d(),
            //     CommmandSwerveDriveTrain.KINEMATICS,
            //     stateStdDevs,
            //     localMeasurementStdDevs,
            //     visionMeasurementStdDevs);

                // tab.addString("Pose", this::getFomattedPose).withPostition(0);          //10 & 11 are filer #s
                // tab.add("Field", field2d).withPosition(2,0).withSize(10, 11 )
       
            }
        
            //pose estimator based on FUN Robotics Network code

            // private static final List<Pose3d> targetPoses = Collections.unmodifiableList(List.of(
            //     new Pose3d(0.5,0.5,0.5, new Rotation3d(0,0, degreesToRadians(180))),
            //     new Pose3d(0.5,0.8,0.5, new Rotation3d(0,0, degreesToRadians(180)))));

        @Override
        public void periodic(){

        //     // cameraPeriodic();

            System.out.println("data accquired");

             System.out.println("yaw is " + yaw);

             System.out.println("pitch is" + pitch);

             System.out.println("area is " + area);

             System.out.println("ID is" + targetID);

             System.out.println("pose ambiguity" + poseAmbiguity);

               //retrieving values from the limelight for apriltag detection

            // var pipelineResult = camera.getLatestResult();
            // var resultTimestamp = pipelineResult.getTimestampSeconds();

            // if(resultTimestamp != previousPipelineTimeStamp && pipelineResult.hasTargets()) {
            //     previousPipelineTimestamp = resultTimestamp;

            //     var target = pipelineResult.getBestTarget();
            //     var fiducialID = target.getFiducialId();
            // }

            // if(target.getPoseAmbiguity() <= 0.2 && fiducialID >= 0 && fiducialID < targetPose.size()){
            //     var targetPose = targetPoses.get(fiducialID);
            //     Transform2d camToTarget = target.getBestCameraToTarget();
            //     Pose2d camPose = targetPose.transformBy(camToTarget.inverse());

            //     var visionMeasurment = camPose.transformBy(CAMERA_TO_ROBOT);
            //     poseEstimator.addVisionMeasurment(visionMeasurement.toPose2d(), resultTimestamp);
            // }
        }
        // //Update pose estimator with drivetrain sensors
        // var drivetrainState = CommandSwerveDrivetrain.getDrivetrainState();
        // poseEstimator.update(
        //     CommandSwerveDrivetrain.getGryroscopeRotation(),
        //     drivetrainState.getSwerveModuleStates(),
        //     drivetrainState.getSwerveModulePosition());
        // field2d.setRobotPose(getCurrentPose());

        }