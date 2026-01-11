package ca.frc6390.athena.devices;

import java.util.HashMap;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;

public class VirtualIMU {

    public class VirtualAxis {
        private Supplier<Rotation2d> supplier;
        private Rotation2d offset;

        public VirtualAxis(Supplier<Rotation2d>  supplier){
            this.supplier = supplier;
            offset = new Rotation2d();
        } 

        public Rotation2d get(){
            return supplier.get().minus(offset);
        }

        public void set(Rotation2d val){
            offset = supplier.get().minus(val);
        }

        public void setOffset(Rotation2d offset){
            this.offset = offset;
        }
    }

    private HashMap<String, VirtualAxis> virtualAxis;

    public VirtualIMU() {
        virtualAxis = new HashMap<>();
    }

    public void addVirtualAxis(String id, Supplier<Rotation2d> axis) {
        virtualAxis.put(id, new VirtualAxis(axis));
    }

    public void setVirtualAxis(String id, Rotation2d val){
        virtualAxis.get(id).set(val);
    }

    public Rotation2d getVirtualAxis(String id) {
       return virtualAxis.get(id).get();
    }

    public Rotation2d getVirtualOffset(String id) {
       return virtualAxis.get(id).offset;
    }

    public void setVirtualOffset(String id, Rotation2d val) {
        virtualAxis.get(id).setOffset(val);
    }
}
