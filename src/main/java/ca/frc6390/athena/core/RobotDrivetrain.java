package ca.frc6390.athena.core;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public interface RobotDrivetrain {
    
    public enum DriveTrainNeutralMode {
        Coast(NeutralModeValue.Coast, IdleMode.kCoast),
        Brake(NeutralModeValue.Brake, IdleMode.kBrake);

        private final NeutralModeValue ctre;
        private final IdleMode rev;

        DriveTrainNeutralMode(NeutralModeValue ctre, IdleMode rev){
            this.ctre = ctre;
            this.rev = rev;
        }

        public NeutralModeValue asCTRE(){
            return ctre;
        }

        public IdleMode asREV(){
            return rev;
        }

        public boolean asBoolean(){
            return ctre == NeutralModeValue.Brake;
        }

        public static DriveTrainNeutralMode fromBoolean(boolean value) {
            return value ? DriveTrainNeutralMode.Brake : DriveTrainNeutralMode.Coast;
        }
    }

    RobotIMU getIMU();
    DriveTrainNeutralMode getNeutralMode();
    void setNeutralMode(DriveTrainNeutralMode mode);
    void drive(ChassisSpeeds speeds);
    void addFeedbackSpeed(ChassisSpeeds speeds);
    void update();
    Command createDriveCommand(DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier thetaInput);
}
