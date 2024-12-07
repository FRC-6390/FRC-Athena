package ca.frc6390.athena.controllers;


import edu.wpi.first.wpilibj.XboxController;

public class DebouncedController extends XboxController {

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
        RIGHT_Y(5),
        TOP(11),
        TOP_RIGHT(12),
        RIGHT(13),
        BOTTOM_RIGHT(14),
        BOTTOM(15),
        BOTTOM_LEFT(16),
        LEFT(17),
        TOP_LEFT(18),
        POV(11);
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

    public DebouncedButton a = new DebouncedButton(this, BUTTONS.A.get()),
    b = new DebouncedButton(this, BUTTONS.B.get()),
    x = new DebouncedButton(this, BUTTONS.X.get()),
    y = new DebouncedButton(this, BUTTONS.Y.get()),
    leftBumper = new DebouncedButton(this, BUTTONS.LEFT_BUMPER.get()),
    rightBumper = new DebouncedButton(this, BUTTONS.RIGHT_BUMPER.get()),
    leftStick = new DebouncedButton(this, BUTTONS.LEFT_JOYSTICK.get()),
    rightStick = new DebouncedButton(this, BUTTONS.RIGHT_JOYSTICK.get()),
    top = new DebouncedButton(this, BUTTONS.TOP.get()),
    topRight = new DebouncedButton(this, BUTTONS.TOP_RIGHT.get()),
    topLeft = new DebouncedButton(this, BUTTONS.TOP_LEFT.get()),
    bottom = new DebouncedButton(this, BUTTONS.BOTTOM.get()),
    bottomRight = new DebouncedButton(this, BUTTONS.BOTTOM_RIGHT.get()),
    bottomLeft = new DebouncedButton(this, BUTTONS.BOTTOM_LEFT.get()),
    right = new DebouncedButton(this, BUTTONS.RIGHT.get()),
    left = new DebouncedButton(this, BUTTONS.LEFT.get()),
    back = new DebouncedButton(this, BUTTONS.BACK.get()),
    start = new DebouncedButton(this, BUTTONS.START.get()), 
    rightTriggerB = new DebouncedButton(this, AXIS.RIGHT_TRIGGER.get()),
    leftTriggerB = new DebouncedButton(this, AXIS.LEFT_TRIGGER.get());
    
    public ModifiedAxis leftX = new ModifiedAxis(this, AXIS.LEFT_X.get()),
    leftY = new ModifiedAxis(this, AXIS.LEFT_Y.get()),
    rightX = new ModifiedAxis(this, AXIS.RIGHT_X.get()),
    rightY = new ModifiedAxis(this, AXIS.RIGHT_Y.get()),
    rightTrigger = new ModifiedAxis(this, AXIS.RIGHT_TRIGGER.get()),
    leftTrigger = new ModifiedAxis(this, AXIS.LEFT_TRIGGER.get());



    public DebouncedController(int port) {
        super(port);
    }

}
