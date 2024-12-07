package ca.frc6390.athena.sensors.blinkin;

public interface BlinkinLedColour {

    public interface FixedPalettePattern {
        
        public interface Rainbow {
            double RAINBOW_PALETTE = -0.99;
            double PARTY_PALETTE = -0.97;
            double OCEAN_PALETTE = -0.95;
            double LAVA_PALETTE = -0.93;
            double FOREST_PALETTE = -0.91;
            double WITH_GLITTER = -0.89;
        }

        double CONFETTI = -0.87;

        public interface Shot {
            double RED = -0.85;
            double BLUE = -0.83;
            double WHITE = -0.81;
        }

        public interface Sinelon {
            double RAINBOW_PALETTE = -0.79;
            double PARTY_PALETTE = -0.77;
            double OCEAN_PALETTE = -0.75;
            double LAVA_PALETTE = -0.73;
            double FOREST_PALETTE = -0.71;
        }

        public interface BeatsPerMinute {
            double RAINBOW_PALETTE = -0.69;
            double PARTY_PALETTE = -0.67;
            double OCEAN_PALETTE = -0.65;
            double LAVA_PALETTE = -0.63;
            double FOREST_PALETTE = -0.61;
        }

        public interface Fire {
            double MEDIUM = -0.59;
            double LARGE = -0.57;
        }

        public interface Twinkles {
            double RAINBOW_PALETTE = -0.55;
            double PARTY_PALETTE = -0.53;
            double OCEAN_PALETTE = -0.51;
            double LAVA_PALETTE = -0.49;
            double FOREST_PALETTE = -0.47;
        }

        public interface ColourWaves {
            double RAINBOW_PALETTE = -0.45;
            double PARTY_PALETTE = -0.43;
            double OCEAN_PALETTE = -0.41;
            double LAVA_PALETTE = -0.39;
            double FOREST_PALETTE = -0.37;
        }

        public interface LarsonScanner {
            double RAINBOW_PALETTE = -0.35;
            double PARTY_PALETTE = -0.33;
            double OCEAN_PALETTE = -0.31;
            double LAVA_PALETTE = -0.29;
            double FOREST_PALETTE = -0.27;
        }
        
        public interface LarsonScannerColour {
            double RED = -0.35;
            double GRAY = -0.33;
        }

        public interface LightChase {
            double RED = -0.31;
            double BLUE = -0.29;
            double GRAY = -0.27;
        }

        public interface Heartbeat {
            double RED = -0.25;
            double BLUE = -0.23;
            double WHITE = -0.21;
            double GRAY = -0.19;
        }

        public interface Breath {
            double RED = -0.17;
            double BLUE = -0.15;
            double GRAY = -0.13;
            
        }

        public interface Strobe {
            double RED = -0.11;
            double BLUE = -0.09;
            double GOLD = -0.07;
            double WHITE = -0.05;
        }
    }

    public interface ColourPattern {
        
        public interface Colour1 {
            double END_TO_END_BLEND_TO_BLACK = 0.03;
            double LARSON_SCANNER_PATTERN_WIDTH = 0.01;
            double LIGHT_CHASE_DIMMING = 0.01;
            double HEARTBEAT_SLOW = 0.03;
            double HEARTBEAT_MEDIUM = 0.05;
            double HEARTBEAT_FAST = 0.07;
            double BREATH_SLOW = 0.09;
            double BREATH_FAST = 0.11;
            double SHOT = 0.13;
            double STROBE = 0.15;
        }

        public interface Colour2 {
            double END_TO_END_BLEND_TO_BLACK =  0.17;
            double LARSON_SCANNER_PATTERN_WIDTH =  0.19;
            double LIGHT_CHASE_DIMMING =  0.21;
            double HEARTBEAT_SLOW =  0.23;
            double HEARTBEAT_MEDIUM =  0.25;
            double HEARTBEAT_FAST =  0.27;
            double BREATH_SLOW =  0.29;
            double BREATH_FAST =  0.31;
            double SHOT =  0.33;
            double STROBE =  0.35;
        }

        public interface Colour1Colour2 {
            double SPARKLE_COLOUR1_ON_COLOUR2 = 0.37;
            double SPARKLE_COLOUR2_ON_COLOUR1 = 0.39;
            double COLOUR_GRADIENT = 0.41;
            double BEATS_PER_MINUTE = 0.43;
            double END_TO_END_BLEND_COLOUR1_TO_COLOUR2 = 0.45;
            double END_TO_END_BLEND_ = 0.47;
            double NO_BLENDING = 0.49;
            double TWINKLES = 0.51;
            double COLOUR_WAVES = 0.53;
            double SINELON = 0.55;
        }
        
    }

    public interface SolidColour {
        
        double HOT_PINK = 0.57;
        double DARK_RED = 0.59;
        double RED = 0.61;
        double RED_ORANGE = 0.63;
        double ORANGE = 0.65;
        double GOLD = 0.67;
        double YELLOW = 0.69;
        double LAWN_GREEN = 0.71;
        double LIME = 0.73;
        double DARK_GREEN = 0.75;
        double GREEN = 0.77;
        double BLUE_GREEN = 0.79;
        double AQUA = 0.81;
        double SKY_BLUE = 0.83;
        double DARK_BLUE = 0.85;
        double BLUE = 0.87;
        double BLUE_VIOLET = 0.89;
        double VIOLET = 0.91;
        double WHITE = 0.93;
        double GRAY = 0.95;
        double DARK_GRAY = 0.97;
        double BLACK = 0.99;
    }

}
