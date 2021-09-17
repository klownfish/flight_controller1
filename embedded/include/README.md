
If you are an radio amateur please create a file named `callsign.h` that contains your callsign.

It should look like this `#define CALLSIGN "xxxxxx"`

The code will adjust the TX properties depending on if you have a license or not. 

It might be illegal to run this without an license even though the TX power is lowered since the duty cycle can go over 10%.