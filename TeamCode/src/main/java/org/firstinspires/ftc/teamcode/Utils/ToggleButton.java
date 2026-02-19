package org.firstinspires.ftc.teamcode.Utils;

public class ToggleButton {
    private boolean toggled = false;
    private boolean lastPressed = false;

    public ToggleButton() {}

    public ToggleButton(boolean initialState) {
        this.toggled = initialState;
    }

    public void update(boolean pressed) {
        if (pressed && !lastPressed) {
            toggled = !toggled;
        }
        lastPressed = pressed;
    }

    public boolean isToggled() {
        return toggled;
    }

    public void set(boolean state) {
        toggled = state;
    }
}
