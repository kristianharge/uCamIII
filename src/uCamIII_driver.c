//~ High level driver for the uCamIII UART camera
//~
//~ Version pi 1.0
//~ Author : Kristian HARGE
//~ Created on : 04/11/2020
//~
//~ Contains the high level driver API to connect the raspberry pi 3 to
//~ the uCamIII C code
#include "uCamIII_driver.h"

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
//////////////////////////Private variables/////////////////////////////
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

static int               _fd;
// static bool              _UARTInterrupt;
// static int               _resetPin;
static uint32_t          _timeout = 500;
static int32_t           _imageSize;
static uint16_t          _packageSize;
static unsigned short    _packageNumber;
static uCamIII_ERROR     _lastError; //reformat with enum

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
//////////////////////////Private functions/////////////////////////////
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

static inline clock_t _get_milliseconds(void){
  return clock()*1000/CLOCKS_PER_SEC;
}

static inline void _clear_buffer(void){
  int ret = -1;
  uint8_t temp_buffer;

  do{
    ret = read(_fd, &temp_buffer, 1);
    printf("cleared : %x\n", temp_buffer);
  } while(ret != -1);
}// end _clear_buffer

static inline int _wait_for_bytes(int fd, void *buf, size_t size){
  //the counter of _timeout in ms
  int count = 0;
  //index of buffer byte to write on
  int i = 0;
  //initial time in milliseconds
  clock_t initial_time = _get_milliseconds();
  //the returned value
  int ret = -1;

  //read the buffer until we get the total number of bytes or we reach timeout
  do{
    ret = read(fd, &(((uint8_t*) buf)[i]), 1);

    if(ret != -1){
      i++;
    }
    count = _get_milliseconds() - initial_time;
  }while (i < size && count < _timeout);

  if(count < _timeout){
    return size;
  }
  else {
    return -1;
  }

}// end _wait_for_bytes

static int init_linux_driver(void){
  struct termios oldtio;
  struct termios newtio;

  // _UARTInterrupt = false;

  // open the device to be non-blocking (read will return immediatly)
  _fd = open(MODEMDEVICE, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (_fd < 0) {
    perror(MODEMDEVICE);
    return uCamIII_INIT_ERROR;
  }

  //Make the file descriptor non blocking (the manual page says only
  //O_APPEND and O_NONBLOCK, will work with F_SETFL...)
  fcntl(_fd, F_SETFL, O_NONBLOCK);
  //save current port settings
  tcgetattr(_fd, &oldtio);
  //set new port settings for non-canonical input processing
  newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
  newtio.c_iflag = IGNPAR;
  newtio.c_oflag = 0;
  newtio.c_lflag = 0;
  //we set the minimum number of chars to 1 for read function to execute
  newtio.c_cc[VMIN]=1;
  //we set the read timeout to 0
  newtio.c_cc[VTIME]=0;
  tcflush(_fd, TCIFLUSH);
  tcsetattr(_fd,TCSANOW,&newtio);

  return uCamIII_SUCCESS;
}


//~ Function : expectPackage
//~ ----------------------------
//~ Waits for a package to come from the uart port
//~
//~ input : uCamIII_CMD package expected, uint8_t options of the package
//~
//~ output : int representing the return of the recieved package or error
static uint64_t expectPackage(uCamIII_CMD pkg, uint8_t option)
{
  uint8_t buf[SIZE_OF_COMMAND];
  //timeout counter in milliseconds
  clock_t counter;
  //clock value at the beginning of the loop
  clock_t clock_init;
  //the result of the read function -1 is error
  int res;

  memset(buf, 0x00, sizeof(buf));
  counter = _timeout;
  clock_init = clock();
  res = -1;

  //wait some time until timeout (see datasheet)
  res = _wait_for_bytes(_fd, buf, SIZE_OF_COMMAND);
//   while(counter > (clock() - clock_init)*100/CLOCKS_PER_SEC){
//     // recieve the camera package back
//     res = read(_fd, buf, SIZE_OF_COMMAND);
//
//     printf("res = %d\n", res);
//
//     if (res != -1){
//
// #ifdef DEBUG_UCAMIII
//       printf("received: %02X %02X %02X %02X %02X %02X\n", buf[0], buf[1], buf[2],
//         buf[3], buf[4], buf[5]);
//       printf("expected: xx %02X %02X xx xx xx, pkg == buf[1] = %d\n", pkg, option, buf[1] == pkg);
// #endif
//       _lastError = 0;
//       //wait for new input
//       _UARTInterrupt = false;
//       break;
//     } //end wait interruption control
//
//   }//end while counter > 0

  if (res == -1){
    _lastError = uCamIII_ERROR_SEND_TIMEOUT;
  }
  else if (buf[1] == uCamIII_CMD_NAK){
    _lastError = buf[4];
  }
  else if (buf[1] == pkg && (buf[2] == option || option == uCamIII_DONT_CARE)){
    _lastError = 0;
    return buf[3] | buf[4] << 8 | buf[5] << 16 | 0x1000000;
  }

  return uCamIII_PACKAGE_ERROR;
} //end function expectPackage

//~ Function : sendCmd
//~ ----------------------------
//~ Send a command through the UART port
//~
//~ input : uCamIII_CMD command to send, uint8_t p1, p2, p3 and p4 options of
//~ the command (see datasheet)
//~
//~ output : int representing the number of bytes sent or -1 if fail of the
//~ function
int sendCmd(uCamIII_CMD cmd, uint8_t p1, uint8_t p2, uint8_t p3, uint8_t p4)
{
  uint8_t buf[SIZE_OF_COMMAND] = { uCamIII_STARTBYTE, cmd, p1, p2, p3, p4 };

#ifdef DEBUG_UCAMIII
  printf("sendCmd: %02X %02X %02X %02X %02X %02X\n", buf[0], buf[1], buf[2],
    buf[3], buf[4], buf[5]);
#endif
  return write(_fd, buf, SIZE_OF_COMMAND);
} //end function sendCmd

//~ Function : sendCmdWithAck
//~ ----------------------------
//~ Sends a command to UART and waits for the acknowledge
//~
//~ input : uCamIII_CMD command to send, uint8_t p1, p2, p3 and p4 options of
//~ the command (see datasheet)
//~
//~ output : int representing the success or fail of the function
static uCamIII_FUNCTION_RET sendCmdWithAck(uCamIII_CMD cmd, uint8_t p1, uint8_t p2, uint8_t p3,
  uint8_t p4)
{
  //the return value of function expectPackage
  uint64_t ret;

  ret = 0;

  if(sendCmd(cmd, p1, p2, p3, p4) == -1){
#ifdef DEBUG_UCAMIII
    printf("Error sending command in file %s, line %d\n", __FILE__, __LINE__);
#endif
    return uCamIII_SEND_CMD_ERROR;
  }

  ret = expectPackage(uCamIII_CMD_ACK, cmd);

  if(ret != uCamIII_PACKAGE_ERROR){
    return uCamIII_SUCCESS;
  }

  return ret;
} //end function sendCmdWithAck

//~ Function : uCAMIII_sync
//~ ----------------------------
//~ Tries to synchronize the camera maxTry times
//~
//~ input : int maxTry that is the number of sync commands sent
//~
//~ output : int representing the success or fail of the function
static uCamIII_FUNCTION_RET uCAMIII_sync(int maxTry)
{
  //the number of times that we send the sync command
  int tries;
  //true if the device was synchronized successfully
  bool synchronized;
  //temporary store _timeout value
  uint32_t ms = _timeout;
  //return value
  int ret;

  ret = uCamIII_SCYNC_ERROR;
  //try to send synchronize command to the camera maxTry times
  for (tries = maxTry - 1; tries > 0; tries--)
  {
#ifdef DEBUG_UCAMIII
      printf("try numero: %d\n", (maxTry - tries));
#endif
    // start with 5ms timeout between syncs and increase by 1ms
    _timeout = 5 + (maxTry - tries);
    synchronized = (sendCmdWithAck(uCamIII_CMD_SYNC, 0, 0, 0, 0)
      == uCamIII_SUCCESS);
    if (synchronized){
      break;
    }
  } //end syncronize the camera maxTry times

  _timeout = ms;

  //that means that we got a success before the number of maxTry
  if (tries > 0)
  {
#ifdef DEBUG_UCAMIII
    printf("sync after %d tries\n", (maxTry - tries));
#endif

    //After the ACK, we expect to recieve a sync package (see datasheet)
    if (expectPackage(uCamIII_CMD_SYNC, uCamIII_DONT_CARE) != uCamIII_PACKAGE_ERROR) {
      sendCmd(uCamIII_CMD_ACK, uCamIII_CMD_SYNC, 0, 0, 0);
      ret = uCamIII_SUCCESS;
    } //end expect recieve a sync package
  }

  //clear buffer after finish
  // _clear_buffer();

#ifdef DEBUG_UCAMIII
  printf("Synchronization succedded ? : %d \n", ret == uCamIII_SUCCESS);
#endif

  return ret;
}// end uCAMIII_sync

//~ Function : setImageFormat
//~ ----------------------------
//~ The host issues this command to configure the image size and Image Format.
//~ After receiving this command, the module will send out an ACK command to the
//~ host if the configuration was successful. Otherwise, a NAK command will be
//~   sent out.
//~
//~ input : uCamIII_PIC_TYPE picture type
//~
//~ output : int representing the success or fail of the function
static inline uCamIII_FUNCTION_RET setImageFormat(uCamIII_IMAGE_FORMAT format,
  uCamIII_RES raw_resolution, uCamIII_RES jpeg_resolution)
{
    int ret;
    ret = sendCmdWithAck(uCamIII_CMD_INIT, 0x00, format, raw_resolution,
      jpeg_resolution);

#ifdef DEBUG_UCAMIII
    printf("cmd init succedded ? : %d \n", ret == uCamIII_SUCCESS);
#endif

    return ret;
} //end setImageFormat

//~ Function : getLastError
//~ ----------------------------
//~ Gets the last error stored in our code
//~
//~ input : void
//~
//~ output : uint8_t representing the last error local variable value
static inline uint8_t getLastError(void)
{
  return _lastError;
}// end getLastError

//~ Function : getJpegData
//~ ----------------------------
//~ gets the jpeg data after sending the get picture command
//~
//~ input : uint8_t *buffer representing the returned package,
//~         int len lenght of the image data in the package,
//~         uCamIII_callback callback (uint8_t* buffer, int len, int id)
//~                          represents the function executed if success,
//~         int package the package number
//~
//~ output : int representing the success or fail of the function
static uCamIII_FUNCTION_RET getJpegData(uint8_t *buffer, int len, uCamIII_callback callback,
  int package)
{
  //package id
  uint16_t id = 0;
  //package size
  uint16_t size = 0;
  uint32_t ms = _get_milliseconds();
  //the function return value
  uCamIII_FUNCTION_RET ret;

  // request specific package
  if (package >= 0){
    _packageNumber = package;
  }

  ret = uCamIII_SUCCESS;

  //control if the command wa sent correctly
  if (sendCmd(uCamIII_CMD_ACK, 0x00, 0x00, _packageNumber & 0xFF,
    _packageNumber >> 8) != -1)
  {
    uint8_t info[4];
    uint8_t chk[2];

    //should rethink
    // usleep(1000*_timeout);

    if (_wait_for_bytes(_fd, info, sizeof(info)) == sizeof(info))
    {
      id   = info[0] | info[1] << 8;
      size = info[2] | info[3] << 8;
#ifdef DEBUG_UCAMIII
      printf("id %d, size %d\n", id, size);
#endif
    }

    // if the expected data didn't arrive in time
    // allow for extra bytes to trickle in and then
    // flush the RX buffer and
    // prepare for termination of request
    if ((size == 0) || (size > len)
    || (_wait_for_bytes(_fd, buffer, size) != size)
    || _wait_for_bytes(_fd, chk, sizeof(chk)) != sizeof(chk)
    )
    {
#ifdef DEBUG_UCAMIII
      printf("Error in file %s, line %d\n", __FILE__, __LINE__);
#endif

      usleep(1000*100);
      ms = _get_milliseconds();
      //Clean the UART buffer
      while((read(_fd, (uint8_t*)buffer, size) != -1) &&
        (_get_milliseconds() - ms < _timeout));

      ret = uCamIII_GET_JPEG_ERROR;//id = 0xF0F0;
    }//end control over data size

    // report end of final data request to camera
    if ((id * (_packageSize - 6) >= _imageSize)
      || (_get_milliseconds() - ms >= _timeout))
    {
      //tell to the camera that all the data was recieved
      sendCmd(uCamIII_CMD_ACK, 0x00, 0x00, 0xF0, 0xF0);
    }
    else{
      // prepare to request next package
      _packageNumber = id;
    }

    //it means that there was not any error getting the package
    if (ret != uCamIII_GET_JPEG_ERROR)//(id < 0xF0F0)
    {
      if (callback != NULL){
        callback(buffer, size, id);
      }
      ret = uCamIII_SUCCESS;
    }// end package error control
  }
  else{
    //send command failed
    ret = uCamIII_SEND_CMD_ERROR;
  }//end send command control

  return ret;
}// end function getJpegData

////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////
///////////////////////////Public functions/////////////////////////////
////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////

//~ Function : uCamIII_init
//~ ----------------------------
//~ Tries to initialize the camera driver
//~
//~ input : void
//~
//~ output : int representing the success or fail of the function
uCamIII_FUNCTION_RET uCamIII_init(void)
{
  uCamIII_FUNCTION_RET ret;

  // hardReset();
  init_linux_driver();

  _clear_buffer();
  ret = uCAMIII_sync(MAXIMUM_SCYNC_TRIES);

  //continue if success (cas use goto but this is cleaner)
  if (ret == uCamIII_SUCCESS)
  {
#ifdef DEBUG_UCAMIII
    ret = setImageFormat(uCamIII_COMP_JPEG, uCamIII_128x128, uCamIII_640x480);
    printf("Set image return : %d\n", ret);
#else
    ret = setImageFormat(uCamIII_COMP_JPEG, uCamIII_128x128, uCamIII_640x480);
#endif

  }

  if (ret != uCamIII_SUCCESS)
  {
    printf("Error %d in initialisation\n", ret);
    return uCamIII_INIT_ERROR;
  }

  return uCamIII_SUCCESS;
} //end uCamIII_init

//~ Function : getPicture
//~ ----------------------------
//~ Sends command to get picture
//~
//~ input : uCamIII_PIC_TYPE picture type
//~
//~ output : int representing the success or fail of the function
uCamIII_FUNCTION_RET getPicture(uCamIII_PIC_TYPE type)
{
  //the returned value
  int ret;

  ret = sendCmdWithAck(uCamIII_CMD_GET_PICTURE, type, 0, 0, 0);
  //control the value of returned ack
  if (ret == uCamIII_SUCCESS)
  {
    //reset the package number as there is a new image comming
    _packageNumber = 0;
    //when sending get picture, we get the picure size in bytes
    _imageSize = expectPackage(uCamIII_CMD_DATA, type) & 0x00FFFFFF;

#ifdef DEBUG_UCAMIII
    printf("image size is %d\n", _imageSize);
#endif
  }// end returned ACK control

  return ret;
}//end getPicture

//~ Function : getJpegPicture
//~ ----------------------------
//~ The host issues this command to get the Jpeg picture requested before
//~
//~ input : FILE * file were the jpeg picture is going
//~
//~ output : int representing the success or fail of the function
uCamIII_FUNCTION_RET getJpegPicture(FILE *jpeg_file){
  uCamIII_FUNCTION_RET ret = uCamIII_GET_JPEG_ERROR;
  uint16_t data_size = _packageSize - 6;
  uint8_t buf[data_size];
  int number_of_packages = _imageSize/data_size + 1;

  for(int i = 0; i < number_of_packages ; i++){
    printf("number_of_packages : %d, data_size : %d\n", number_of_packages, data_size);
    ret = getJpegData(buf, data_size, NULL, i);
    if(ret != uCamIII_SUCCESS){
      break;
    }
    else{
      fwrite(buf, 1, data_size, jpeg_file);
    }
  }

  return ret;
}// end getJpegPicture

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
uCamIII_FUNCTION_RET takeSnapshot(uCamIII_SNAP_TYPE type, uint16_t frame){
    int ret = sendCmdWithAck(uCamIII_CMD_SNAPSHOT, type, frame & 0xFF,
      (frame >> 8) & 0xFF, 0x00);
    usleep(1000*_timeout);

    return ret;
} //end takeSnapshot

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
uCamIII_FUNCTION_RET setPackageSize(uint16_t size)
{
  //return value of send command function
  int ret;

  ret = sendCmdWithAck(uCamIII_CMD_SET_PACKSIZE, 0x08, size & 0xFF,
    (size >> 8) & 0xFF, 0x00);

  //if the command was a success, we set the new package size value
  if (ret == uCamIII_SUCCESS){
    _packageSize = size;
    ret = uCamIII_SUCCESS;
  }// end return value control

  return ret;
}// end setPackageSize

//~ Function : setCamBaudrate
//~ ----------------------------
//~ This command changes the communication baudrate of the camera
//~
//~ input : int baudrate (the new baudrate (bits per second))
//~
//~ output : int representing the success or fail of the function
uCamIII_FUNCTION_RET setCamBaudrate(int baudrate)
{
  uint8_t first_div;
  uint8_t second_div;

  switch(baudrate){
    case 2400:
      first_div = 0x1F;
      second_div = 0x2F;
      break;
    case 4800:
      first_div = 0x1F;
      second_div = 0x17;
      break;
    case 9600:
      first_div = 0x1F;
      second_div = 0x0B;
      break;
    case 19200:
      first_div = 0x1F;
      second_div = 0x05;
      break;
    case 38400:
      first_div = 0x1F;
      second_div = 0x02;
      break;
    case 57600:
      first_div = 0x1F;
      second_div = 0x01;
      break;
    case 115200:
      first_div = 0x1F;
      second_div = 0x00;
      break;
    case 153600:
      first_div = 0x07;
      second_div = 0x02;
      break;
    case 230400:
      first_div = 0x07;
      second_div = 0x01;
      break;
    case 460800:
      first_div = 0x07;
      second_div = 0x00;
      break;
    case 921600:
      first_div = 0x01;
      second_div = 0x01;
      break;
    case 1228800:
      first_div = 0x02;
      second_div = 0x00;
      break;
    case 1843200:
      first_div = 0x01;
      second_div = 0x00;
      break;
    case 3686400:
      first_div = 0x00;
      second_div = 0x00;
      break;
    default:
#ifndef DEBUG_UCAMIII
      printf("Error setting baudrate, invald value : %d . \
        Please refere to datasheet\n", baudrate);
#endif
      return uCamIII_SET_BAUDRATE_ERROR;
  }
  return sendCmdWithAck(uCamIII_CMD_SET_BAUDRATE, first_div, second_div, 0x00,
    0x00);
}// end setCamBaudrate

//~ Function : reset
//~ ----------------------------
//~ Makes a soft or hard reset on the device
//~
//~ input : uCamIII_RESET_TYPE reset type (hard or soft), uint16_t force
//~ (special reset, see datasheet)
//~
//~ output : int representing the success or fail of the function
uCamIII_FUNCTION_RET reset(uCamIII_RESET_TYPE type, bool force)
{
    uint8_t force_cmd;

    if (force){
      force_cmd = uCamIII_RESET_FORCE;
    }
    else{
      force_cmd = 0x00;
    }// end force control

    return sendCmdWithAck(uCamIII_CMD_RESET, type, 0x00, 0x00, force_cmd);
}// end reset

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
  uCamIII_CBE exposure)
{
  return sendCmdWithAck(uCamIII_CMD_SET_CBE, contrast, brightness, exposure,
    0x00);
}// end setCBE

//~ Function : setFrequency
//~ ----------------------------
//~ The host issues this command to change the light frequency (hum) response of
//~ the uCAM-III.
//~
//~ input : uCamIII_FREQ light frequency in Hz (0 -> 50Hz, 1 -> 60Hz)
//~
//~ output : int representing the success or fail of the function
uCamIII_FUNCTION_RET setFrequency(uCamIII_FREQ frequency)
{
  return sendCmdWithAck(uCamIII_CMD_SET_FREQ, frequency, 0x00, 0x00, 0x00);
}// end setFrequency

//~ Function : setSleepTime
//~ ----------------------------
//~ This command adjusts the sleep timeout of the uCAM-III from the default of
//~ 15 seconds, from disabled (0) to 255 seconds, using the commands 00h to FFh
//~
//~ input : uint8_t seconds (the new sleep timeout)
//~
//~ output : int representing the success or fail of the function
uCamIII_FUNCTION_RET setSleepTime(uint8_t seconds)
{
  return sendCmdWithAck(uCamIII_CMD_SLEEP, seconds, 0x00, 0x00, 0x00);
}// end setIdleTime
