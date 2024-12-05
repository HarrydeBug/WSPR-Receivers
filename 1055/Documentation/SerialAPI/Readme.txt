Note that the ZachTek Serial API is only in use when the swtich inside the receiver is set to Programming mode.
You will need to remove the lid to access this switch.
In this position the device can be reprogrammed with new firmware using the ArduinoI DE, see source code for information how to do it.
You should not leave the switch in this position unless you intend to upload new firmware or want to run the Factory configuration tool to calibrate the internal TCXO or set the startup frequency or detector in VFO A and B.
(Whatever VFO you have picked and frequency set when you issue the Save to EEPROM command will be what will be used at next startup.
When you are done doing this the switch should be set back to CAT mode to operate the radio using ICOM CAT commands.

//Harry
