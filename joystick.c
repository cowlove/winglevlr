#ifndef ESP32
/**
 * Author: Jason White
 *
 * Description:
 * Reads joystick/gamepad events and displays them.
 *
 * Compile:
 * gcc joystick.c -o joystick
 *
 * Run:
 * ./joystick [/dev/input/jsX]
 *
 * See also:
 * https://www.kernel.org/doc/Documentation/input/joystick-api.txt
 */
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <linux/joystick.h>
#include <math.h>

/**
 * Reads a joystick event from the joystick device.
 *
 * Returns 0 on success. Otherwise -1 is returned.
 */
int read_event(int fd, struct js_event *event)
{
    ssize_t bytes;

    bytes = read(fd, event, sizeof(*event));

    if (bytes == sizeof(*event))
        return 0;

    /* Error, could not read full event. */
    return -1;
}

/**
 * Returns the number of axes on the controller or 0 if an error occurs.
 */
size_t get_axis_count(int fd)
{
    __u8 axes;

    if (ioctl(fd, JSIOCGAXES, &axes) == -1)
        return 0;

    return axes;
}

/**
 * Returns the number of buttons on the controller or 0 if an error occurs.
    */
size_t get_button_count(int fd)
{
    __u8 buttons;
    if (ioctl(fd, JSIOCGBUTTONS, &buttons) == -1)
        return 0;

    return buttons;
}

/**
 * Current state of an axis.
 */
struct axis_state {
    short x, y;
};

/**
 * Keeps track of the current axis state.
 *
 * NOTE: This function assumes that axes are numbered starting from 0, and that
 * the X axis is an even number, and the Y axis is an odd number. However, this
 * is usually a safe assumption.
 *
 * Returns the axis that the event indicated.
 */
size_t get_axis_state(struct js_event *event, struct axis_state axes[3])
{
    size_t axis = event->number / 2;

    if (axis < 3)
    {
        if (event->number % 2 == 0)
            axes[axis].x = event->value;
        else
            axes[axis].y = event->value;
    }

    return axis;
}

int main(int argc, char *argv[])
{
    const char *device;
    int js;
    struct js_event event;
    struct axis_state axes[3] = {0};
    size_t axis;

    if (argc > 1)
        device = argv[1];
    else
        device = "/dev/input/js0";

    js = open(device, O_RDONLY);

    if (js == -1)
        perror("Could not open joystick");

    
    
    float xtrim = 0, ytrim = 0;
    float trimstep = .01;

    /* This loop will exit if the controller is unplugged. */
    while(1) { 
        
        while (read_event(js, &event) == 0)
        {
            switch (event.type)
            {
                case JS_EVENT_BUTTON:
                    printf("Button %u %s\n", event.number, event.value ? "pressed" : "released");
                    switch (event.number) {
                        case 2: ytrim -= trimstep; break;
                        case 1: ytrim += trimstep; break;
                        case 3: xtrim -= trimstep; break;
                        case 0: xtrim += trimstep; break;
                    }
                    printf("trim %f %f\n", xtrim, ytrim);
                case JS_EVENT_AXIS:
                    axis = get_axis_state(&event, axes);
                    break;
                default:
                    /* Ignore init events. */
                    break;
            }   
        
            float leftStringX = 8;
            float leftStringY = 8;
        
            float rightStringX = 8;
            float rightStringY = 8;

            float leftLen = sqrt(leftStringX * leftStringX + leftStringY * leftStringY);
            float rightLen = sqrt(rightStringX * rightStringX + rightStringY * rightStringY);

            float servoThrow = 2.0;
            float xScale = +1.0;
            float yScale = -1.0;
        
            float x = axes[0].y / 32767.0 * servoThrow / sqrt(2) + xtrim;
            float y = axes[0].x / 32767.0 * servoThrow / sqrt(2) + ytrim;
            //printf("%6f %6f\n", x, y);

            float x1 = leftStringX + xScale * x;
            float y1 = leftStringY + yScale * y;
            float leftNewLen = sqrt(x1 * x1 + y1 * y1);

            x1 = rightStringX - xScale * x;
            y1 = rightStringY + yScale * y;
            float rightNewLen = sqrt(x1 * x1 + y1 * y1);
        

            float s0 =  +(rightNewLen - rightLen) / servoThrow * 2000 + 1500;
              float s1 =  +(leftNewLen - leftLen) / servoThrow * 2000 + 1500;

            printf("s %4.0f %4.0f\n", s0, s1 );
            fflush(stdout);
        }
    }

    close(js);
    return 0;
}
#endif