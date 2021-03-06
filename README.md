# RBotFirmwareMod

A fork of [Rob Dobson's RBot Firmware](https://github.com/robdobsn/RBotFirmware). The main change is to make it work with the Adafruit V2 Motor Driver. This allows you to use standard hardware, such as the Adafruit Huzzah32 plus the Adadruit DC Motor/Stepper Featherwing. The difficulty in using this library is that the motor control in Rob Dobson's original code happens within an interrupt routine. The function is annotated IRAM_ATTR and all functions it calls must be similarly annotated. So I would have needed to add this annotation to the to the motor control library, the PWM library and the I2C library. Instead I moved the motor control out of the interrupt routine. It makes the code a bit less responsive, but you can still drive the steppers at 50 mm/s using interleave mode.

I forked the original code as I haven't had any response to comments on [Rob Dobson's blog](https://robdobson.com/2018/09/sandbot-software-revamp). I'd be happy to merge my code in with his.

I have only uploaded the PlatformIO part of the code. The rest remains unchanged.
