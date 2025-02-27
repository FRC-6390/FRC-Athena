package ca.frc6390.athena.odometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;

public class SlipCompensatingSwerveDrivePoseEstimator extends SwerveDrivePoseEstimator {

    private static final double DEFAULT_SLIP_THRESHOLD = 0.2; 
    private static final double DEFAULT_MIN_SLIP_FACTOR = 0.5;
    private static final double DEFAULT_MAX_SLIP_FACTOR = 1.0;
    private static final double MIN_AVERAGE_DELTA = 0.001;
    
    private static final double TAU = 0.1;

    private double slipThreshold;
    private final double minSlipFactor;
    private final double maxSlipFactor;
    
    private SwerveModulePosition[] previousModulePositions;
    private Pose2d previousRawPose;
    private Translation2d slipOffset = new Translation2d(0, 0);
    
    private double previousUpdateTime;
    private double filteredSlipFactor = 1.0;

    public SlipCompensatingSwerveDrivePoseEstimator(SwerveDriveKinematics kinematics,
            Rotation2d gyroAngle,
            SwerveModulePosition[] modulePositions,
            Pose2d initialPoseMeters,
            Matrix<N3, N1> stateStdDevs,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        super(kinematics, gyroAngle, modulePositions, initialPoseMeters, stateStdDevs, visionMeasurementStdDevs);
        this.previousRawPose = initialPoseMeters;
        
        this.slipThreshold = DEFAULT_SLIP_THRESHOLD;
        this.minSlipFactor = DEFAULT_MIN_SLIP_FACTOR;
        this.maxSlipFactor = DEFAULT_MAX_SLIP_FACTOR;
        
        this.previousUpdateTime = Timer.getFPGATimestamp();
    }
    
    public SlipCompensatingSwerveDrivePoseEstimator(SwerveDriveKinematics kinematics,
            Rotation2d gyroAngle,
            SwerveModulePosition[] modulePositions,
            Pose2d initialPoseMeters) {
        this(kinematics, gyroAngle, modulePositions, initialPoseMeters,
             VecBuilder.fill(0.1, 0.1, 0.1), VecBuilder.fill(0.9, 0.9, 0.9));
    }
    
    @Override
    public Pose2d update(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
        double currentTime = Timer.getFPGATimestamp();
        double dt = currentTime - previousUpdateTime;
        previousUpdateTime = currentTime;


        Pose2d rawPose = super.update(gyroAngle, modulePositions);
        if (previousRawPose == null) {
            previousRawPose = rawPose;
        }

        Translation2d rawDeltaTranslation = rawPose.getTranslation().minus(previousRawPose.getTranslation());

        double computedSlipFactor = computeSlipFactor(modulePositions);
        double alpha = dt / (TAU + dt);
        filteredSlipFactor = alpha * computedSlipFactor + (1 - alpha) * filteredSlipFactor;

        Translation2d compensatedDeltaTranslation = rawDeltaTranslation.times(filteredSlipFactor);

        Translation2d deltaOffset = compensatedDeltaTranslation.minus(rawDeltaTranslation);

        slipOffset = slipOffset.plus(deltaOffset);

        Translation2d finalTranslation = rawPose.getTranslation().plus(slipOffset);
        Pose2d finalPose = new Pose2d(finalTranslation, rawPose.getRotation());

        previousRawPose = rawPose;
        previousModulePositions = modulePositions;

        return finalPose;
    }

    public void setSlipThreshold(double slipThreshold) {
        this.slipThreshold = slipThreshold;
    }

    private double computeSlipFactor(SwerveModulePosition[] currentPositions) {
        if (currentPositions == null || previousModulePositions == null) {
            return maxSlipFactor;
        }
        if (currentPositions.length != previousModulePositions.length) {
            throw new IllegalArgumentException("Mismatch in number of module positions.");
        }
        

        int n = currentPositions.length;
        double totalDelta = 0;
        double[] deltas = new double[n];
        
        for (int i = 0; i < n; i++) {
            double delta = Math.abs(currentPositions[i].distanceMeters - previousModulePositions[i].distanceMeters);
            deltas[i] = delta;
            totalDelta += delta;
        }
        double averageDelta = totalDelta / n;
        if (averageDelta < MIN_AVERAGE_DELTA) {
            return maxSlipFactor;
        }
        
        double variance = 0;
        for (int i = 0; i < n; i++) {
            variance += Math.pow(deltas[i] - averageDelta, 2);
        }
        variance /= n;
        double stdDev = Math.sqrt(variance);
        
        double relativeStdDev = stdDev / averageDelta;
        if (relativeStdDev > slipThreshold) {
            return Math.max(minSlipFactor, maxSlipFactor - (relativeStdDev - slipThreshold));
        }
        return maxSlipFactor;
    }
}

/* 

     //Improved slip factor computation that filters out wheels which might be off the ground.
     
    private double computeSlipFactor(SwerveModulePosition[] currentPositions) {
         if (currentPositions == null || previousModulePositions == null) {
            return maxSlipFactor;
        }
        if (currentPositions.length != previousModulePositions.length) {
            throw new IllegalArgumentException("Mismatch in number of module positions.");
        }
        
        int n = currentPositions.length;
        double[] deltas = new double[n];
        
        for (int i = 0; i < n; i++) {
            double delta = Math.abs(currentPositions[i].distanceMeters - previousModulePositions[i].distanceMeters);
            deltas[i] = delta;
        }
        
        double medianDelta = median(deltas);
        
        // Filter out wheels that might be off the ground.
        List<Double> validDeltas = new ArrayList<>();
        for (double delta : deltas) {
            if (delta >= filteringThresholdFactor * medianDelta) {
                validDeltas.add(delta);
            }
        }
        
        if (validDeltas.isEmpty()) {
            return maxSlipFactor;
        }
        
        double validSum = 0;
        for (double d : validDeltas) {
            validSum += d;
        }
        double validAverage = validSum / validDeltas.size();
        
        if (validAverage < MIN_AVERAGE_DELTA) {
            return maxSlipFactor;
        }
        
        double variance = 0;
        for (double d : validDeltas) {
            variance += Math.pow(d - validAverage, 2);
        }
        variance /= validDeltas.size();
        double stdDev = Math.sqrt(variance);
        
        double relativeStdDev = validAverage > 0 ? stdDev / validAverage : 0;
        if (relativeStdDev > slipThreshold) {
            return Math.max(minSlipFactor, maxSlipFactor - (relativeStdDev - slipThreshold));
        }
        return maxSlipFactor;
    }

    private double median(double[] array) {
        double[] copy = array.clone();
        Arrays.sort(copy);
        int mid = copy.length / 2;
        if (copy.length % 2 == 0) {
            return (copy[mid - 1] + copy[mid]) / 2.0;
        } else {
            return copy[mid];
        }
    }

 */