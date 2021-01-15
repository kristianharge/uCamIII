#include "uCamIII_driver.h"

int main(int argc, char * argv[]){

  FILE * jpeg_Picture = fopen("picture_test.jpeg", "w");

  uCamIII_init();

  //printf("setCamBaudrate size is success : %d\n", setCamBaudrate(921600) == uCamIII_SUCCESS);

  //printf("reset is success : %d\n", reset(0, false) == uCamIII_SUCCESS);

  printf("setPackageSize is success : %d\n", setPackageSize(512) == uCamIII_SUCCESS);

  printf("setCBE is success : %d\n", setCBE(uCamIII_DEFAULT, uCamIII_DEFAULT, uCamIII_DEFAULT) == uCamIII_SUCCESS);
  //
  printf("takeSnapshot is success : %d\n", takeSnapshot(uCamIII_SNAP_JPEG, 0x0000) == uCamIII_SUCCESS);
  //
  printf("getPicture is success : %d\n", getPicture(uCamIII_TYPE_JPEG) == uCamIII_SUCCESS);
  //
  printf("getJpegPicture is success : %d\n", getJpegPicture(jpeg_Picture) == uCamIII_SUCCESS);

  fclose(jpeg_Picture);


  return 0;
}
