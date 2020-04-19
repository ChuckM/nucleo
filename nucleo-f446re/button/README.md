# Simple Button Example

This is a really simple button example. It uses polling to see
when the button was pushed and then waits for it to be released.
It demonstrates using the timer as a delay to debounce the input.

"Bouncing" is a condition occurs in mechanical switch contacts. When
the button is first activated (pressed) it may make intermittent
contact. If you are fast enough, and the computer always is, those
intermittent contacts can appear as multiple presses of the button.

"Debouncing" consists of delaying a short time after the button state
first shows it as pushed, and checking again to be sure it is still pressed.
In that way, small glitches in the button contacts are ignored and a single
button press is recorded.
