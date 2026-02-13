package ca.frc6390.athena.core;

import java.util.HashMap;
import java.util.Locale;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class RobotSpeeds {

    public static class SpeedSource {

        private String name;
        private int priority;
        private double vxMetersPerSecond;
        private double vyMetersPerSecond;
        private double omegaRadiansPerSecond;
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
            if (speeds == null) {
                setInputSpeeds(0.0, 0.0, 0.0);
                return;
            }
            setInputSpeeds(
                    speeds.vxMetersPerSecond,
                    speeds.vyMetersPerSecond,
                    speeds.omegaRadiansPerSecond);
        }

        public void setInputSpeeds(double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond) {
            this.vxMetersPerSecond = vxMetersPerSecond;
            this.vyMetersPerSecond = vyMetersPerSecond;
            this.omegaRadiansPerSecond = omegaRadiansPerSecond;
        }

        public double outputVx() {
            if (!enabled || !enableX) {
                return 0.0;
            }
            return vxMetersPerSecond;
        }

        public double outputVy() {
            if (!enabled || !enableY) {
                return 0.0;
            }
            return vyMetersPerSecond;
        }

        public double outputOmega() {
            if (!enabled || !enableTheta) {
                return 0.0;
            }
            return omegaRadiansPerSecond;
        }

        public ChassisSpeeds getOutputSpeeds() {
            return new ChassisSpeeds(outputVx(), outputVy(), outputOmega());
        }

        public void stop(){
            vxMetersPerSecond = 0.0;
            vyMetersPerSecond = 0.0;
            omegaRadiansPerSecond = 0.0;
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

        sources.put("drive", new SpeedSource("drive", 0));
        registerSpeedSource("auto", 1);
        registerSpeedSource("feedback", 1);

    }

    public void registerSpeedSource(String name, int priority){

        if(priority <= 0){
            throw new Error("priority level must be greater than 0, 0 is reserved for drive");
        }

        sources.put(normalizeKey(name), new SpeedSource(name, priority));
    }

    public void setSpeeds(String source, ChassisSpeeds speeds) {
        sources.get(normalizeKey(source)).setInputSpeeds(speeds);
    }

    public void setSpeeds(String source, double x, double y, double theta) {
        sources.get(normalizeKey(source)).setInputSpeeds(x, y, theta);
    }

    public ChassisSpeeds getSpeeds(String source) {
        return sources.get(normalizeKey(source)).getOutputSpeeds();
    }

    public ChassisSpeeds getSpeedsAtPriorityLevel(int priority) {
        double vx = 0.0;
        double vy = 0.0;
        double omega = 0.0;
        for (SpeedSource source : sources.values()) {
            if (source.getPriority() == priority) {
                vx += source.outputVx();
                vy += source.outputVy();
                omega += source.outputOmega();
            }
        }
        return new ChassisSpeeds(vx, vy, omega);
    }

    public void stop(){
        sources.forEach((key, value) -> value.stop());
    }

    public void stopSpeeds(String source){
        sources.get(normalizeKey(source)).stop();
    }

    public void setSpeedSourceState(String source, boolean enabled){
        sources.get(normalizeKey(source)).setEnabled(enabled);
    }

    public boolean isSpeedsSourceActive(String source){
        return sources.get(normalizeKey(source)).isEnabled();
    }

    public void setAxisState(String source, SpeedAxis axis, boolean enabled){
        sources.get(normalizeKey(source)).setAxisState(axis, enabled);
    }

    public void setAllAxisState(String source, SpeedAxis axis, boolean enabled){
        sources.forEach((key, value)-> value.setAxisState(axis, enabled));
    }

    public boolean isAxisActive(String source, SpeedAxis axis){
        return sources.get(normalizeKey(source)).isAxisActive(axis);
    }

    public ChassisSpeeds calculate(){

        double nonDriveVx = 0.0;
        double nonDriveVy = 0.0;
        double nonDriveOmega = 0.0;
        SpeedSource driverSource = sources.get("drive");
        for (SpeedSource source : sources.values()) {
            if (source.getPriority() != 0) {
                nonDriveVx += source.outputVx();
                nonDriveVy += source.outputVy();
                nonDriveOmega += source.outputOmega();
            }
        }

        double driverVx = driverSource != null ? driverSource.outputVx() : 0.0;
        double driverVy = driverSource != null ? driverSource.outputVy() : 0.0;
        double driverOmega = driverSource != null ? driverSource.outputOmega() : 0.0;

        double finalVx = combineAxis(driverVx, nonDriveVx);
        double finalVy = combineAxis(driverVy, nonDriveVy);
        double finalOmega = combineAxis(driverOmega, nonDriveOmega);

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

        double driverSign = Math.signum(driverAxis);
        double feedbackSign = Math.signum(autoFeedbackAxis);

        if (feedbackSign == 0.0) {
            return driverAxis;
        }

        if (driverSign == feedbackSign) {
            double dominant = Math.max(Math.abs(driverAxis), Math.abs(autoFeedbackAxis));
            // Use the larger magnitude so assistive control cannot spike the commanded speed
            return Math.copySign(dominant, driverAxis);
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

    private String normalizeKey(String source) {
        return source.toLowerCase(Locale.ROOT);
    }
}
