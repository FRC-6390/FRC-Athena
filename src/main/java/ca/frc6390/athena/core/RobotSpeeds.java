package ca.frc6390.athena.core;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class RobotSpeeds {

    public enum SpeedSource {
        DRIVER,
        AUTO,
        FEEDBACK
    }

    public enum SpeedAxis {
        X,
        Y,
        Theta
    }

    private ChassisSpeeds driver;
    private ChassisSpeeds auto;
    private ChassisSpeeds feedback;
    private double maxVelocity, maxAngularVelocity;
    private boolean enableDriver, enableAuto, enableFeedback;
    private boolean enableX, enableY, enableTheta;

    public RobotSpeeds(double maxVelocity, double maxAngularVelocity){
        this.maxVelocity = maxVelocity;
        this.maxAngularVelocity = maxAngularVelocity;

        driver = new ChassisSpeeds();
        auto = new ChassisSpeeds();
        feedback = new ChassisSpeeds();

        enableDriver = true;
        enableAuto = true;
        enableFeedback = true;

        enableX = true;
        enableY = true;
        enableTheta = true;
    }

    public void setDriverSpeeds(ChassisSpeeds speeds) {
        this.driver = speeds;
    }

    public void setDriverSpeeds(double x, double y, double theta) {
        setDriverSpeeds(new ChassisSpeeds(x, y, theta));
    }

    public void setAutoSpeeds(ChassisSpeeds speeds) {
        this.auto = speeds;
    }

    public void setAutoSpeeds(double x, double y, double theta) {
        setAutoSpeeds(new ChassisSpeeds(x, y, theta));
    }

    public void setFeedbackSpeeds(ChassisSpeeds speeds) {
        this.feedback = speeds;
    }

    public void setFeedbackSpeeds(double x, double y, double theta) {
        setFeedbackSpeeds(new ChassisSpeeds(x, y, theta));
    }

    public ChassisSpeeds getDriverSpeeds() {
        return enableDriver ? driver : new ChassisSpeeds();
    }

    public ChassisSpeeds getAutoSpeeds() {
        return enableAuto ? auto : new ChassisSpeeds();
    }

    public ChassisSpeeds getFeedbackSpeeds() {
        return enableFeedback ? feedback : new ChassisSpeeds();
    }

    public void stop(){
        stopAutoSpeeds();
        stopDriverSpeeds();
        stopFeedbackSpeeds();
    }

    public void stopAutoSpeeds(){
        setAutoSpeeds(new ChassisSpeeds());
    }

    public void stopDriverSpeeds(){
        setDriverSpeeds(new ChassisSpeeds());
    }

    public void stopFeedbackSpeeds(){
        setFeedbackSpeeds(new ChassisSpeeds());
    }

    public void enableSpeeds(SpeedSource speed, boolean enabled){
        switch (speed) {
            case DRIVER:
                enableDriver = enabled;
            break;
            case AUTO:
                enableAuto = enabled;
            break;
            case FEEDBACK:
                enableFeedback = enabled;
            break;
        }
    }

    public boolean isSpeedsEnabled(SpeedSource axis){
        switch (axis) {
            case DRIVER:
            return enableDriver;
            case AUTO:
            return enableAuto;
            case FEEDBACK:
            return enableFeedback;
            default:
            return false;
        }
    }

    public void enableAxis(SpeedAxis axis, boolean enabled){
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
        }
    }

    public boolean isAxisEnabled(SpeedAxis axis){
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

    public ChassisSpeeds calculate(){

        ChassisSpeeds autoPlusFeedback = getAutoSpeeds().plus(getFeedbackSpeeds());
        ChassisSpeeds tempDriver = getDriverSpeeds();

        double finalVx = combineAxis(tempDriver.vxMetersPerSecond, autoPlusFeedback.vxMetersPerSecond);
        double finalVy = combineAxis(tempDriver.vyMetersPerSecond, autoPlusFeedback.vyMetersPerSecond);
        double finalOmega = combineAxis(tempDriver.omegaRadiansPerSecond, autoPlusFeedback.omegaRadiansPerSecond);

        finalVx = enableX ? clamp(finalVx, maxVelocity) : 0;
        finalVy = enableY ? clamp(finalVy, maxVelocity) : 0;
        finalOmega = enableTheta ? clamp(finalOmega, maxAngularVelocity) : 0;

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
