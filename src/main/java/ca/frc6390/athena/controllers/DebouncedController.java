package ca.frc6390.athena.controllers;

/**
 * Please use EnhancedXboxController
 * DebouncedController only exist to ensure minimal issue with teams using the library
 * EnhancedXboxController is the same as DebouncedController but with working POV buttons
 * DebouncedController will be removed once all the teams using the lib have updated their code
 * @deprecated
 */
@Deprecated
public class DebouncedController extends EnhancedXboxController{

    public DebouncedController(int port) {
        super(port);
    }
    
}
