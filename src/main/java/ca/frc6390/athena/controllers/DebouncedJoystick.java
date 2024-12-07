package ca.frc6390.athena.controllers;

import edu.wpi.first.wpilibj.Joystick;

public class DebouncedJoystick extends Joystick {

    private enum BUTTONS{
        ONE(1),
        TWO(2),
        THREE(3),
        FOUR(4),
        FIVE(5),
        SIX(6),
        SEVEN(7),
        EIGHT(8),
        NINE(9),
        TEN(10),
        ELEVEN(11),
        TWELVE(12);
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

    public DebouncedButton one = new DebouncedButton(this, BUTTONS.ONE.get()),
        two = new DebouncedButton(this, BUTTONS.TWO.get()),
        three = new DebouncedButton(this, BUTTONS.THREE.get()),
        four = new DebouncedButton(this, BUTTONS.FOUR.get()),
        five = new DebouncedButton(this, BUTTONS.FIVE.get()),
        six = new DebouncedButton(this, BUTTONS.SIX.get()),
        seven = new DebouncedButton(this, BUTTONS.SEVEN.get()),
        eight = new DebouncedButton(this, BUTTONS.EIGHT.get()),
        nine = new DebouncedButton(this, BUTTONS.NINE.get()),
        ten = new DebouncedButton(this, BUTTONS.TEN.get()),
        eleven = new DebouncedButton(this, BUTTONS.ELEVEN.get()),
        twelve = new DebouncedButton(this, BUTTONS.TWELVE.get());
     

     public ModifiedAxis leftX = new ModifiedAxis(this, AXIS.LEFT_X.get()),
     leftY = new ModifiedAxis(this, AXIS.LEFT_Y.get()),
     leftTrigger = new ModifiedAxis(this, AXIS.LEFT_TRIGGER.get()).withoutSquaring(),
     rightX = new ModifiedAxis(this, AXIS.RIGHT_X.get()),
     rightY = new ModifiedAxis(this, AXIS.RIGHT_Y.get()),
     rightTrigger = new ModifiedAxis(this, AXIS.RIGHT_TRIGGER.get()).withoutSquaring();


    public DebouncedJoystick(int port) {
        super(port);
    }
}