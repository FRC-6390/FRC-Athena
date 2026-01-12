package ca.frc6390.athena.core.localization;

import ca.frc6390.athena.core.auto.HolonomicPidConstants;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;

public record RobotLocalizationConfig(
        double xStd,
        double yStd,
        double thetaStd,
        double vXStd,
        double vYStda,
        double vThetaStd,
        double v2XStd,
        double v2YStda,
        double v2ThetaStd,
        double zStd,
        double vZStd,
        double v2ZStd,
        HolonomicPidConstants translation,
        HolonomicPidConstants rotation,
        boolean useVision,
        PoseSpace poseSpace) {

    public enum PoseSpace {
        TWO_D,
        THREE_D
    }

    public RobotLocalizationConfig(double xStd, double yStd, double thetaStd, double vXStd, double vYStda, double vThetaStd) {
        this(
                xStd,
                yStd,
                thetaStd,
                vXStd,
                vYStda,
                vThetaStd,
                vXStd,
                vYStda,
                vThetaStd,
                xStd,
                vXStd,
                vXStd,
                new HolonomicPidConstants(0, 0, 0),
                new HolonomicPidConstants(0, 0, 0),
                true,
                PoseSpace.TWO_D);
    }

    public RobotLocalizationConfig(double xStd, double yStd, double thetaStd) {
        this(xStd, yStd, thetaStd, 0.9, 0.9, 0.9);
    }

    public RobotLocalizationConfig() {
        this(0.1, 0.1, 0.001);
    }

    public static RobotLocalizationConfig vision(double vXStd, double vYStda, double vThetaStd){
        return new RobotLocalizationConfig().setVision(vXStd, vYStda, vThetaStd).setVisionEnabled(true);
    }

    public static RobotLocalizationConfig defualt(){
        return new RobotLocalizationConfig();
    }

    public RobotLocalizationConfig setAutoPlannerPID(double tP, double tI, double tD, double rP, double rI, double rD){
        return setAutoPlannerPID(new HolonomicPidConstants(tP, tI, tD), new HolonomicPidConstants(rP, rI, rD));
    }

    public RobotLocalizationConfig setAutoPlannerPID(HolonomicPidConstants translation, HolonomicPidConstants rotation){
        return new RobotLocalizationConfig(
                xStd,
                yStd,
                thetaStd,
                vXStd,
                vYStda,
                vThetaStd,
                v2XStd,
                v2YStda,
                v2ThetaStd,
                zStd,
                vZStd,
                v2ZStd,
                translation,
                rotation,
                useVision,
                poseSpace);
    }

    public RobotLocalizationConfig setVision(double vXStd, double vYStda, double vThetaStd){
        return setVision(vXStd, vYStda, vXStd, vThetaStd);
    }

    public RobotLocalizationConfig setVision(double vXStd, double vYStda, double vZStd, double vThetaStd) {
        return new RobotLocalizationConfig(
                xStd,
                yStd,
                thetaStd,
                vXStd,
                vYStda,
                vThetaStd,
                v2XStd,
                v2YStda,
                v2ThetaStd,
                zStd,
                vZStd,
                v2ZStd,
                translation,
                rotation,
                useVision,
                poseSpace);
    }

    public RobotLocalizationConfig setVisionMultitag(double v2XStd, double v2YStda, double v2ThetaStd){
        return setVisionMultitag(v2XStd, v2YStda, v2XStd, v2ThetaStd);
    }

    public RobotLocalizationConfig setVisionMultitag(double v2XStd, double v2YStda, double v2ZStd, double v2ThetaStd){
        return new RobotLocalizationConfig(
                xStd,
                yStd,
                thetaStd,
                vXStd,
                vYStda,
                vThetaStd,
                v2XStd,
                v2YStda,
                v2ThetaStd,
                zStd,
                vZStd,
                v2ZStd,
                translation,
                rotation,
                useVision,
                poseSpace);
    }

    public RobotLocalizationConfig setVisionEnabled(boolean useVision){
        return new RobotLocalizationConfig(
                xStd,
                yStd,
                thetaStd,
                vXStd,
                vYStda,
                vThetaStd,
                v2XStd,
                v2YStda,
                v2ThetaStd,
                zStd,
                vZStd,
                v2ZStd,
                translation,
                rotation,
                useVision,
                poseSpace);
    }

    public Matrix<N3, N1> getStd(){
        return VecBuilder.fill(xStd,yStd,Units.degreesToRadians(thetaStd));
    }

    public Matrix<N3, N1> getVisionStd(){
        return VecBuilder.fill(vXStd,vYStda,Units.degreesToRadians(vThetaStd));
    }
    public Matrix<N3, N1> getVisionMultitagStd(){
        return VecBuilder.fill(v2XStd,v2YStda,Units.degreesToRadians(v2ThetaStd));
    }

    public Matrix<N3, N1> getStd2d() {
        return getStd();
    }

    public Matrix<N3, N1> getVisionStd2d() {
        return getVisionStd();
    }

    public Matrix<N3, N1> getVisionMultitagStd2d() {
        return getVisionMultitagStd();
    }

    public Matrix<N4, N1> getStd3d(){
        return VecBuilder.fill(xStd, yStd, zStd, Units.degreesToRadians(thetaStd));
    }

    public Matrix<N4, N1> getVisionStd3d(){
        return VecBuilder.fill(vXStd, vYStda, vZStd, Units.degreesToRadians(vThetaStd));
    }

    public Matrix<N4, N1> getVisionMultitagStd3d(){
        return VecBuilder.fill(v2XStd, v2YStda, v2ZStd, Units.degreesToRadians(v2ThetaStd));
    }

    public RobotLocalizationConfig setPoseSpace(PoseSpace poseSpace) {
        return new RobotLocalizationConfig(
                xStd,
                yStd,
                thetaStd,
                vXStd,
                vYStda,
                vThetaStd,
                v2XStd,
                v2YStda,
                v2ThetaStd,
                zStd,
                vZStd,
                v2ZStd,
                translation,
                rotation,
                useVision,
                poseSpace);
    }

    public RobotLocalizationConfig setZStd(double zStd) {
        return new RobotLocalizationConfig(
                xStd,
                yStd,
                thetaStd,
                vXStd,
                vYStda,
                vThetaStd,
                v2XStd,
                v2YStda,
                v2ThetaStd,
                zStd,
                vZStd,
                v2ZStd,
                translation,
                rotation,
                useVision,
                poseSpace);
    }

    public RobotLocalizationConfig setVisionZStd(double vZStd) {
        return new RobotLocalizationConfig(
                xStd,
                yStd,
                thetaStd,
                vXStd,
                vYStda,
                vThetaStd,
                v2XStd,
                v2YStda,
                v2ThetaStd,
                zStd,
                vZStd,
                v2ZStd,
                translation,
                rotation,
                useVision,
                poseSpace);
    }

    public RobotLocalizationConfig setVisionMultiZStd(double v2ZStd) {
        return new RobotLocalizationConfig(
                xStd,
                yStd,
                thetaStd,
                vXStd,
                vYStda,
                vThetaStd,
                v2XStd,
                v2YStda,
                v2ThetaStd,
                zStd,
                vZStd,
                v2ZStd,
                translation,
                rotation,
                useVision,
                poseSpace);
    }

    public RobotLocalizationConfig use3d() {
        return setPoseSpace(PoseSpace.THREE_D);
    }

    public RobotLocalizationConfig use2d() {
        return setPoseSpace(PoseSpace.TWO_D);
    }

    public RobotLocalizationConfig {
        poseSpace = poseSpace != null ? poseSpace : PoseSpace.TWO_D;
    }
}
