# uCamIII
This is the Driver to use the uCamIII (UART camera) with a raspberry PI written in C code.

## Architecture

Here is the architecture of the driver **after being built**. You can find in the bin folder a test binary and in the libs folder the static library of the driver.
.
├── bin
│   └── test
├── inc
│   └── uCamIII_driver.h
├── libs
│   └── libuCamIII_driver.a
├── LICENSE
├── Makefile
├── README.md
├── src
│   └── uCamIII_driver.c
└── test.c

## Execution

First of all, your UART port should be opened in your raspberry. For more information about this, go to this link : https://www.circuits.dk/setup-raspberry-pi-3-gpio-uart/#:~:text=Setup%20UART%20on%20the%20raspi,1%20in%20%2Fboot%2Fconfig.

Now that you understood everyting on UART for raspberry, it is very simple. Just execute the makefile :

```console
pi@raspberrypi:/Whatever/directory$ make
```

This will build a binary with all the tests I wrote in test.c. And it will also create a libs folder containing the static library of this code. This static library can be used in any other project by including the header and library in your project. Example :

```console
pi@raspberrypi:/Whatever/other/directory$ gcc -I/Whatever/directory/inc -L/Whatever/directory/libs [compilation stuff] -luCamIII_driver
```

### Flexibility

This code was built to be executed in raspberry pi 3. If you want to run it in an other linux device, make sure to change the following lines of code that are in the header file :

```C
#define MODEMDEVICE "/dev/ttyS0"
```

This lines point to the file descriptor of the raspberry 3 UART 0. But if you are using any other linux device, you should search for the specific UART file descriptor that you need and replace the MODEMDEVICE definition.

## Examples
