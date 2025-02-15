package frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.subsystems.LEDsSubSystem;
import org.photonvision.PhotonCamera;

public class Object {
    
    public static PhotonCamera camObj;
    
    // private LEDsSubSystem m_LEDsSubSystem;

    public Object() {
        // m_LEDsSubSystem = ledsSubsystem;
        camObj = new PhotonCamera("camObj"); // Create a new PhotonCamera object
        camObj.setDriverMode(false); // Set the camera to driver mode
    }

    public void periodic() {
    }

    /**
     * Checks if there are any targets detected by PhotonVision.
     * 
     * @return true if targets are found, false otherwise.
     */
    public boolean watchForNote(){
        boolean hasTargets = false;
        var results = camObj.getAllUnreadResults(); // Get all unread results from PhotonVision
        //hasTargets = results.hasTargets();
        for (var result : results) {
            if (result.hasTargets()) {
                hasTargets = true;
                break;
            }
        }
        // if (hasTargets == true){
        //     m_LEDsSubSystem.fadeEffect(150, 255);
        //     //LEDsSubSystem.setSolidLED(150, 255, 50);
        // } else {
        //     if (DriverStation.getAlliance().get() == Alliance.Blue) { 
        //         // LEDsSubSystem.fadeEffect(120, 255);
        //         m_LEDsSubSystem.scanEffect(120, 255, 255);
        //         // m_LEDsSubSystem.strobeEffect(120, 255, 255);
        //         //LEDsSubSystem.setSolidLED(120, 255, 50);
        //     } else {
        //         // LEDsSubSystem.fadeEffect(0, 255);
        //         m_LEDsSubSystem.scanEffect(0, 255, 255);
        //         // m_LEDsSubSystem.strobeEffect(0, 255, 255);
        //         //LEDsSubSystem.setSolidLED(0, 255, 50);
        //     }
        // }
        return hasTargets;
    }
}