package ca.frc6390.athena.controllers;


import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;

public class EnhancedXboxController extends XboxController {
    
    private enum BUTTONS{
        A(1),
        B(2),
        X(3),
        Y(4),
        LEFT_BUMPER(5),
        RIGHT_BUMPER(6),
        BACK(7),
        START(8),
        LEFT_JOYSTICK(9),
        RIGHT_JOYSTICK(10),
        LEFT_X(0),
        LEFT_Y(1),
        RIGHT_X(4),
        RIGHT_Y(5);
        private int port;
        private BUTTONS(int port){
            this.port = port;
        }

        public int get(){
            return port;
        }
    }

    private enum AXIS{
        LEFT_X(0),
        LEFT_Y(1),
        LEFT_TRIGGER(2),
        RIGHT_TRIGGER(3),
        RIGHT_X(4),
        RIGHT_Y(5);
  
        private int port;
        private AXIS(int port){
            this.port = port;
        }

        public int get(){
            return port;
        }
    }

    public final DebouncedButton a,b,x,y,leftBumper,rightBumper,leftStick,rightStick,back,start;
    public final ModifiedAxis leftX,leftY,rightX,rightY,rightTrigger,leftTrigger;
    public final EnhancedPOV pov;
    

    public EnhancedXboxController(int port) {
        super(port);
        a = new DebouncedButton(this, BUTTONS.A.get());
        b = new DebouncedButton(this, BUTTONS.B.get());
        x = new DebouncedButton(this, BUTTONS.X.get());
        y = new DebouncedButton(this, BUTTONS.Y.get());
        leftBumper = new DebouncedButton(this, BUTTONS.LEFT_BUMPER.get());
        rightBumper = new DebouncedButton(this, BUTTONS.RIGHT_BUMPER.get());
        leftStick = new DebouncedButton(this, BUTTONS.LEFT_JOYSTICK.get());
        rightStick = new DebouncedButton(this, BUTTONS.RIGHT_JOYSTICK.get());

        back = new DebouncedButton(this, BUTTONS.BACK.get());
        start = new DebouncedButton(this, BUTTONS.START.get()); 

        leftX = new ModifiedAxis(this, AXIS.LEFT_X.get());
        leftY = new ModifiedAxis(this, AXIS.LEFT_Y.get());
        rightX = new ModifiedAxis(this, AXIS.RIGHT_X.get());
        rightY = new ModifiedAxis(this, AXIS.RIGHT_Y.get());
        rightTrigger = new ModifiedAxis(this, AXIS.RIGHT_TRIGGER.get());
        leftTrigger = new ModifiedAxis(this, AXIS.LEFT_TRIGGER.get());

        pov = new EnhancedPOV(this);
    }

    public EnhancedXboxController setSticksInverted(boolean inverted){
        leftX.setInverted(inverted);
        leftY.setInverted(inverted);
        rightX.setInverted(inverted);
        rightY.setInverted(inverted);
        return this;
    }

    public EnhancedXboxController setSticksDeadzone(double deadzone){
        leftX.setDeadzone(deadzone);
        leftY.setDeadzone(deadzone);
        rightX.setDeadzone(deadzone);
        rightY.setDeadzone(deadzone);
        return this;
    }

    public EnhancedXboxController setSticksSlewrate(double rate){
        leftX.enableSlewrate(rate);
        leftY.enableSlewrate(rate);
        rightX.enableSlewrate(rate);
        rightY.enableSlewrate(rate);
        return this;
    }

    public EnhancedXboxController setSticksSlewrate(double frate, double rrate){
        leftX.enableSlewrate(frate, rrate);
        leftY.enableSlewrate(frate, rrate);
        rightX.enableSlewrate(frate, rrate);
        rightY.enableSlewrate(frate, rrate);
        return this;
    }



    public class EnhancedPOV {
        public final DebouncedButton center,up,upRight,upLeft,down,downRight,downLeft,right,left;
        public EnhancedPOV(GenericHID hid){
            center = new DebouncedButton(() -> hid.getPOV() == -1);
            up = new DebouncedButton(() -> hid.getPOV() == 0);
            upRight = new DebouncedButton(() -> hid.getPOV() == 45);
            right = new DebouncedButton(() -> hid.getPOV() == 90);
            downRight = new DebouncedButton(() -> hid.getPOV() == 135);
            down = new DebouncedButton(() -> hid.getPOV() == 180);
            downLeft = new DebouncedButton(() -> hid.getPOV() == 225);
            left = new DebouncedButton(() -> hid.getPOV() == 270);
            upLeft = new DebouncedButton(() -> hid.getPOV() == 315);       
        }       
    }
}
