# light-synth-pi
Syncing connected Neopixels to sound data sent over bluetooth from [jo12bar/bluesend][bluesend-github].

Upon recieving the data from bluesend, it converts it into the levels that three strips of Neopixel LEDs should be at (high, mid, and low frequencies). This data is passed over serial to a connected Arduino (specifically, an Adafruit Pro Trinket) which actually controls the LED strips. The repo for the Arduino's code is yet to be set up.

[bluesend-github]: https://github.com/jo12bar/bluesend
