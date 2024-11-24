Not finalized.

# Signal generation parameters

FOC-Stim uses the [T-Code](https://github.com/multiaxis/TCode-Specification) protocol for receiving commands. See source code for available axis and ranges.

# Boot process

Once the device is ready to accept commands, it will print `"Device ready. Awaiting DSTART.\r\n"`.

Send `DSTART` to start pulse generation, `DSTOP` to stop.

Version string can be received with `D0`. It is highly recommended to check for an exact version string.

# Logs

The device will regularly send parameter updates, for example electrode resistance or device temperature information. Logs start with `$` and end with a newline. Example logline `$R_a:1.5 R_b:2.0 V_BUS:42\r\n`.

If an error occurs, unformatted debug information is printed. Errors can only be recovered by restarting the device.

# Keepalive

The device will stop generating pulses if no valid commands have been received for some time. Spam `DPING` or any other valid command to subvert.
