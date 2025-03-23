package ca.frc6390.athena.core;

import java.util.HashMap;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class RobotSpeeds {

    public static class SpeedSource {

        private String name;
        private int priority;
        private ChassisSpeeds speeds;
        private boolean enabled, enableX, enableY, enableTheta;

        public SpeedSource(String name, int priority){
            this.name = name;
            this.priority = priority;
            enabled = true;
            enableX = true;
            enableY = true;
            enableTheta = true;
        }

        public int getPriority() {
            return priority;
        }
        
        public String getName() {
            return name;
        }

        public void setInputSpeeds(ChassisSpeeds speeds){
            this.speeds = speeds;
        }

        public ChassisSpeeds getOutputSpeeds() {
            double x = enableX ? speeds.vxMetersPerSecond : 0;
            double y = enableY ? speeds.vyMetersPerSecond : 0;
            double theta = enableTheta ? speeds.omegaRadiansPerSecond : 0;
    
            return enabled ? new ChassisSpeeds(x,y,theta) : new ChassisSpeeds();
        }

        public void stop(){
            speeds = new ChassisSpeeds();
        }

        public boolean isEnabled() {
            return enabled;
        }

        public void setEnabled(boolean enabled) {
            this.enabled = enabled;
        }

        public void setAxisState(SpeedAxis axis, boolean enabled){
            switch (axis) {
                case X:
                    enableX = enabled;
                break;
                case Y:
                    enableY = enabled;
                break;
                case Theta:
                    enableTheta = enabled;
                break;
                default:
                    break;
            }
        }

        public boolean isAxisActive(SpeedAxis axis){
            switch (axis) {
                case X:
                return enableX;
                case Y:
                return enableY;
                case Theta:
                return enableTheta;
                default:
                return false;
            }
        }
    }

    public enum SpeedAxis {
        X,
        Y,
        Theta,
    }

    private HashMap<String, SpeedSource> sources;

    private double maxVelocity, maxAngularVelocity;

    public RobotSpeeds(double maxVelocity, double maxAngularVelocity){
        this.maxVelocity = maxVelocity;
        this.maxAngularVelocity = maxAngularVelocity;
        this.sources = new HashMap<>();

        registerSpeedSource("drive", 0);
    }

    public void registerSpeedSource(String name, int priority){

        if(priority <= 0){
            throw new Error("priority level must be greater than 0, 0 is reserved for drive");
        }

        sources.put(name, new SpeedSource(name, priority));
    }

    public void setSpeeds(String source, ChassisSpeeds speeds) {
        sources.get(source.toLowerCase()).setInputSpeeds(speeds);
    }

    public void setSpeeds(String source, double x, double y, double theta) {
        setSpeeds(source.toLowerCase(), new ChassisSpeeds(x, y, theta));
    }

    public ChassisSpeeds getSpeeds(String source) {
        return sources.get(source.toLowerCase()).getOutputSpeeds();
    }

    public ChassisSpeeds getSpeedsAtPriorityLevel(int priority) {
        ChassisSpeeds speeds = new ChassisSpeeds();
        sources.values().stream().filter(val -> val.getPriority() == priority).forEach(val -> speeds.plus(val.getOutputSpeeds()));
        return speeds;
    }

    public void stop(){
        sources.forEach((key, value) -> value.stop());
    }

    public void stopSpeeds(String source){
        sources.get(source.toLowerCase()).stop();
    }

    public void setSpeedSourceState(String source, boolean enabled){
        sources.get(source.toLowerCase()).setEnabled(enabled);
    }

    public boolean isSpeedsSourceActive(String source){
        return sources.get(source.toLowerCase()).isEnabled();
    }

    public void setAxisState(String source, SpeedAxis axis, boolean enabled){
        sources.get(source.toLowerCase()).setAxisState(axis, enabled);
    }

    public void setAllAxisState(String source, SpeedAxis axis, boolean enabled){
        sources.forEach((key, value)-> value.setAxisState(axis, enabled));
    }

    public boolean isAxisActive(String source, SpeedAxis axis){
        return sources.get(source).isAxisActive(axis);
    }

    public ChassisSpeeds calculate(){

        ChassisSpeeds nonDriveSpeeds = new ChassisSpeeds(); 
        sources.values().stream().filter(val -> val.getPriority() != 0).forEach(val -> nonDriveSpeeds.plus(val.getOutputSpeeds()));
        ChassisSpeeds tempDriver = getSpeeds("drive");

        double finalVx = combineAxis(tempDriver.vxMetersPerSecond, nonDriveSpeeds.vxMetersPerSecond);
        double finalVy = combineAxis(tempDriver.vyMetersPerSecond, nonDriveSpeeds.vyMetersPerSecond);
        double finalOmega = combineAxis(tempDriver.omegaRadiansPerSecond, nonDriveSpeeds.omegaRadiansPerSecond);

        finalVx = clamp(finalVx, maxVelocity);
        finalVy = clamp(finalVy, maxVelocity);
        finalOmega = clamp(finalOmega, maxAngularVelocity);

        return new ChassisSpeeds(finalVx, finalVy, finalOmega);
    }

    private double combineAxis(double driverAxis, double autoFeedbackAxis) {
        // If driver axis is zero, let auto/feedback control that axis completely
        if (Math.abs(driverAxis) < 1E-6) {
            return autoFeedbackAxis;
        }

        // If driver axis and autoFeedback axis have the same sign, sum them
        if (Math.signum(driverAxis) == Math.signum(autoFeedbackAxis)) {
            return driverAxis + autoFeedbackAxis;
        }

        // Otherwise, if they're in conflict, driver wins
        return driverAxis;
    }

    private double clamp(double value, double maxValue) {
        return Math.copySign(Math.min(Math.abs(value), maxValue), value);
    }

    public double getMaxAngularVelocity() {
        return maxAngularVelocity;
    }

    public double getMaxVelocity() {
        return maxVelocity;
    }
}
