# Auto Watering Machine
I use PIC18F45K22 to build a watering machine.
function guide:
* use RS232 to connect your PC.
* set your countdown time (Max time 59:59) which you want to start the watering machine.
  * if the time format isn't correct, the LED screen will show error.
  * the setup time will be recorded into the EEPROM.
* press 'c' to clear the time, 's' to start the countdown.
* when the time is up, the Submersible motor will be turned on. 
