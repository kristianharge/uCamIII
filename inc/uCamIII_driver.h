//~ High level driver for the uCamIII UART camera
//~
//~ Version pi 1.0
//~ Author : Kristian HARGE
//~ Created on : 04/11/2020
//~
//~ Contains the high level driver API to connect the raspberry pi 3 to
//~ the uCamIII C code

#ifndef _UCAMIII_h_
#define _UCAMIII_h_

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <time.h>
#include <unistd.h>			//Used for UART
#include <fcntl.h>			//Used for UART
#include <termios.h>		//Used for UART
#include <sys/signal.h>
#include <sys/types.h>

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
////////////////UART communication global variables/////////////////////
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

#define BAUDRATE B921600
#define MODEMDEVICE "/dev/ttyS0"

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
//////////////////////uCamIII global variables//////////////////////////
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
//each command is 6 bytes long
#define SIZE_OF_COMMAND       6
//number of times we should try to scync
#define MAXIMUM_SCYNC_TRIES   60

typedef enum
{ uCamIII_CMD_INIT          = 0x01
, uCamIII_CMD_GET_PICTURE   = 0x04
, uCamIII_CMD_SNAPSHOT      = 0x05
, uCamIII_CMD_SET_PACKSIZE  = 0x06
, uCamIII_CMD_SET_BAUDRATE  = 0x07
, uCamIII_CMD_RESET         = 0x08
, uCamIII_CMD_DATA          = 0x0A
, uCamIII_CMD_SYNC          = 0x0D
, uCamIII_CMD_ACK           = 0x0E
, uCamIII_CMD_NAK           = 0x0F
, uCamIII_CMD_SET_FREQ      = 0x13
, uCamIII_CMD_SET_CBE       = 0x14
, uCamIII_CMD_SLEEP         = 0x15
, uCamIII_STARTBYTE         = 0xAA
, uCamIII_DONT_CARE         = 0xFF
} uCamIII_CMD;

typedef enum
{ uCamIII_RAW_8BIT          = 0x03
, uCamIII_RAW_16BIT_RGB565  = 0x06
, uCamIII_COMP_JPEG         = 0x07
, uCamIII_RAW_16BIT_CRYCBY  = 0x08
} uCamIII_IMAGE_FORMAT;

typedef enum
{ uCamIII_80x60             = 0x01
, uCamIII_160x120           = 0x03
, uCamIII_160x128           = 0x03
, uCamIII_320x240           = 0x05
, uCamIII_640x480           = 0x07
, uCamIII_128x96            = 0x08
, uCamIII_128x128           = 0x09
} uCamIII_RES;

typedef enum
{ uCamIII_TYPE_SNAPSHOT     = 0x01
, uCamIII_TYPE_RAW          = 0x02
, uCamIII_TYPE_JPEG         = 0x05
} uCamIII_PIC_TYPE;

typedef enum
{ uCamIII_SNAP_JPEG         = 0x00
, uCamIII_SNAP_RAW          = 0x01
} uCamIII_SNAP_TYPE;

typedef enum
{ uCamIII_RESET_FULL        = 0x00
, uCamIII_RESET_STATE       = 0x01
, uCamIII_RESET_FORCE       = 0xFF
} uCamIII_RESET_TYPE;

typedef enum
{ uCamIII_50Hz              = 0x00
, uCamIII_60Hz              = 0x01
} uCamIII_FREQ;

typedef enum
{ uCamIII_MIN               = 0x00  // Exposure -2
, uCamIII_LOW               = 0x01  //          -1
, uCamIII_DEFAULT           = 0x02  //           0
, uCamIII_HIGH              = 0x03  //          +1
, uCamIII_MAX               = 0x04  //          +2
} uCamIII_CBE;

typedef enum
{ uCamIII_ERROR_PIC_TYPE    = 0x01
, uCamIII_ERROR_PIC_UPSCALE = 0x02
, uCamIII_ERROR_PIC_SCALE   = 0x03
, uCamIII_ERROR_UNEXP_REPLY = 0x04
, uCamIII_ERROR_PIC_TIMEOUT = 0x05
, uCamIII_ERROR_UNEXP_CMD   = 0x06
, uCamIII_ERROR_JPEG_TYPE   = 0x07
, uCamIII_ERROR_JPEG_SIZE   = 0x08
, uCamIII_ERROR_PIC_FORMAT  = 0x09
, uCamIII_ERROR_PIC_SIZE    = 0x0A
, uCamIII_ERROR_PARAM       = 0x0B
, uCamIII_ERROR_SEND_TIMEOUT= 0x0C
, uCamIII_ERROR_CMD_ID      = 0x0D
, uCamIII_ERROR_PIC_NOT_RDY = 0x0F
, uCamIII_ERROR_PKG_NUM     = 0x10
, uCamIII_ERROR_PKG_SIZE    = 0x11
, uCamIII_ERROR_CMD_HEADER  = 0xF0
, uCamIII_ERROR_CMD_LENGTH  = 0xF1
, uCamIII_ERROR_PIC_SEND    = 0xF5
, uCamIII_ERROR_CMD_SEND    = 0xFF
} uCamIII_ERROR;

//the returned value ot the function
typedef enum
{  uCamIII_SUCCESS
, uCamIII_INIT_ERROR
, uCamIII_PACKAGE_ERROR
, uCamIII_SCYNC_ERROR
, uCamIII_SEND_CMD_ERROR
, uCamIII_GET_JPEG_ERROR
, uCamIII_SET_BAUDRATE_ERROR
} uCamIII_FUNCTION_RET;

typedef int (*uCamIII_callback) (uint8_t* buffer, int len, int id);

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
//////////////////////////public functions//////////////////////////////
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

//~ Function : uCamIII_init
//~ ----------------------------
//~ Tries to initialize the camera driver
//~
//~ input : void
//~
//~ output : int representing the success or fail of the function
uCamIII_FUNCTION_RET uCamIII_init();

//~ Function : getPicture
//~ ----------------------------
//~ Sends command to get picture
//~
//~ input : uCamIII_PIC_TYPE picture type
//~
//~ output : int representing the success or fail of the function
uCamIII_FUNCTION_RET getPicture(uCamIII_PIC_TYPE type);

//~ Function : getJpegPicture
//~ ----------------------------
//~ The host issues this command to get the Jpeg picture requested before
//~
//~ input : FILE * file were the jpeg picture is going
//~
//~ output : int representing the success or fail of the function
uCamIII_FUNCTION_RET getJpegPicture(FILE * jpeg_file);

//~ Function : takeSnapshot
//~ ----------------------------
//~ The uCAM-III will hold a single frame of still picture data in its buffer
//~ after receiving this command. This snapshot can then be retrieved from the
//~ buffer multiple times if required.
//~
//~ input : uCamIII_SNAP_TYPE snap type, uint16_t frame (the number of frames to
//~ be skipped)
//~
//~ output : int representing the success or fail of the function
uCamIII_FUNCTION_RET takeSnapshot(uCamIII_SNAP_TYPE type, uint16_t frame);

//~ Function : setPackageSize
//~ ----------------------------
//~ The host issues this command to change the size of the data package which is
//~ used to transmit the compressed JPEG image data from the uCAM-III to the
//~ host. This command should be issued before sending SNAPSHOT or GET PICTURE
//~ commands to the uCAM-III.
//~
//~ input : uint16_t size (the size of the package)
//~
//~ output : int representing the success or fail of the function
uCamIII_FUNCTION_RET setPackageSize(uint16_t size);

//~ Function : setCamBaudrate
//~ ----------------------------
//~ This command changes the communication baudrate of the camera
//~
//~ input : int baudrate (the new baudrate (bits per second))
//~
//~ output : int representing the success or fail of the function
uCamIII_FUNCTION_RET setCamBaudrate(int baudrate);

//~ Function : reset
//~ ----------------------------
//~ Makes a soft or hard reset on the device
//~
//~ input : uCamIII_RESET_TYPE reset type (hard or soft), uint16_t force
//~ (special reset, see datasheet)
//~
//~ output : int representing the success or fail of the function
uCamIII_FUNCTION_RET reset(uCamIII_RESET_TYPE type, bool force);

//~ Function : setFrequency
//~ ----------------------------
//~ The host issues this command to change the light frequency (hum) response of
//~ the uCAM-III.
//~
//~ input : uCamIII_FREQ light frequency in Hz (0 -> 50Hz, 1 -> 60Hz)
//~
//~ output : int representing the success or fail of the function
uCamIII_FUNCTION_RET setFrequency(uCamIII_FREQ frequency);

//~ Function : setCBE
//~ ----------------------------
//~ The host issues this command to change the Contrast, White Balance and
//~ Exposure, based on the 3 parameters with this command.
//~
//~ input : uCamIII_CBE contrast (0-4, 2 is normal), uCamIII_CBE brightness
//~ (0-4, 2 is normal), uCamIII_CBE exposure (0-4, 2 is normal)
//~
//~ output : int representing the success or fail of the function
uCamIII_FUNCTION_RET setCBE(uCamIII_CBE contrast, uCamIII_CBE brightness,
  uCamIII_CBE exposure);

//~ Function : setSleepTime
//~ ----------------------------
//~ This command adjusts the sleep timeout of the uCAM-III from the default of
//~ 15 seconds, from disabled (0) to 255 seconds, using the commands 00h to FFh
//~
//~ input : uint8_t seconds (the new sleep timeout)
//~
//~ output : int representing the success or fail of the function
uCamIII_FUNCTION_RET setSleepTime(uint8_t seconds);

#endif
