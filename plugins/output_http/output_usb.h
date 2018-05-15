#include "luareader.h"
#include "../../mjpg_streamer.h"

#define  DISPLAY_WIDTH   96
#define  DISPLAY_HIGH   128
#define  DISPLAY_DEEPTH   16
#define  DISPLAY_SIZE  (DISPLAY_WIDTH*DISPLAY_HIGH)

#define  SEND_BUFFER   2048

char display_start[] = {0xfc,0xd1, 0x01 ,0x01,0x00,0x08,0x00};
char display_end[] =  {0xfc,0xd1, 0x01 ,0x02,0x00};
char display_data[] = {0xfc,0xd1, 0x01 ,0x00,0x00,0x08,0x00};
char display_pos[] =  {0xfc,0xd1, 0x00 ,0x00,0x04,67,96,0x60,0x80};
char display_clear[] =  {0xfc,0xd1, 0x03 ,0x00,0x00};


pthread_t worker_usb;
void *worker_thread_usb(void *arg);

int output_init_usb(output_parameter *param);

int output_run_usb(int id);
int usb_transmit(void *context, const unsigned char * apdu,int apdu_len);

/* adapted from v4l2 spec */
#define fourcc(a, b, c, d)                      \
    ((uint32_t)(d) | ((uint32_t)(c) << 8) |     \
     ((uint32_t)(b) << 16) | ((uint32_t)(a) << 24))


