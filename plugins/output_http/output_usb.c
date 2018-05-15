/*****************************************************************************
 Copyright(C) Tendyron Corporation
 All rights reserved.
 
 @file   : output_usb.c
 @brief  : usb camera for pr11 display
 @author : gexueyuan
 @history:
           2018-5-3    gexueyuan    Created file
           ...
******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <getopt.h>
#include <pthread.h>
#include <syslog.h>
#include <time.h>
#include <errno.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdint.h>




#include <jpeglib.h>
#include "output_usb.h"

#include "../../utils.h"
#include "../../mjpg_streamer.h"

#include "./include/zbar.h"
#include <jerror.h>
#include <setjmp.h>

#define OUTPUT_PLUGIN_NAME "VIEWER output plugin"

static globals *pglobal;
static unsigned char *frame = NULL;
static int input_number = 0;
static int fd;
static char *folder = "/tmp";




void print_send(unsigned char* send,int len)
{

    int i;
    struct timeval tvl;
    struct tm * local_t;
    
    gettimeofday(&tvl, NULL);
    local_t = localtime(&tvl.tv_sec);
    
        printf("\n[%02d:%02d:%02d.%03d]send data len is :%d\n\r",local_t->tm_hour,local_t->tm_min,local_t->tm_sec,(int)(tvl.tv_usec/1000),len);
        for(i = 0;i < len;i++){
        
            printf("%02X ",send[i]);
            
        }
        printf("\n\r\n");
}


void print_rec(unsigned char* rec,int len)
{
        int i;
        struct timeval tvl;
        struct tm * local_t;
        
        gettimeofday(&tvl, NULL);
        local_t = localtime(&tvl.tv_sec);

            printf("\n[%02d:%02d:%02d.%03d]recv data len is :%d\n\r",local_t->tm_hour,local_t->tm_min,local_t->tm_sec,(int)(tvl.tv_usec/1000),len);
            for(i = 0;i < len;i++){
            
                printf("%02X ",rec[i]);
                
            }
            printf("\n\r\n");

}

//#define COLOR_TO_MTK_COLOR_SIMUL(color) ((((color) >> 19) & 0x1f) << 11) \
//                                            |((((color) >> 10) & 0x3f) << 5) \
//                                            |(((color) >> 3) & 0x1f)

unsigned short rgb_24_2_565(int r, int g, int b)  
{  
    return (unsigned short)((((r) << 8) & 0xF800) |   
            (((g) << 3) & 0x7E0)  |  
            (((b) >> 3)));  
}

unsigned char bmp565_write(unsigned char *image, long width, long height, const char *filename)      
{     
    long file_size;     
    long data_size;     
    unsigned char widthAlignBytes;     
    FILE *fp;     
    
    // 文件头     
    unsigned char header[66] = {     
        // BITMAPFILEINFO     
                'B', 'M',       // [0-1] bfType:必须是BM字符     
        0, 0, 0, 0,             // [2-5] bfSize:总文件大小      
        0, 0, 0, 0,             // [6-9] brReserved1,bfReserved2:保留     
        sizeof(header), 0, 0, 0,// [10-13] bfOffBits:到图像数据的偏移     
        // BITMAPFILEHEADER     
        0x28, 0, 0, 0,          // [14-17] biSize:BITMAPINFOHEADER大小40字节     
        0, 0, 0, 0,             // [18-21] biWidth:图片宽度     
        0, 0, 0, 0,             // [22-25] biHeight:图片高度     
        0x01, 0,                // [26-27] biPlanes:必须为1     
        0x10, 0,                // [28-29] biBitCount:16位     
        0x03, 0, 0, 0,          // [30-33] biCompression:BI_BITFIELDS=3     
        0, 0, 0, 0,             // [34-37] biSizeImage:图片大小     
        0x12, 0x0B, 0, 0,       // [38-41] biXPelsPerMeter:单位长度内的像素数     
        0x12, 0x0B, 0, 0,       // [42-45] biYPelsPerMeter:单位长度内的像素数     
        0, 0, 0, 0,             // [46-49] biClrUsed:可用像素数，设为0即可     
        0, 0, 0, 0,             // [50-53] biClrImportant:重要颜色数，设为0即可     
        // RGBQUAD MASK     
        0x0, 0xF8, 0, 0,        // [54-57] 红色掩码     
        0xE0, 0x07, 0, 0,       // [58-61] 绿色掩码     
        0x1F, 0, 0, 0           // [62-65] 蓝色掩码     
    };     
    
    widthAlignBytes = ((width * 16 + 31) & ~31) / 8; // 每行需要的合适字节个数     
    data_size = widthAlignBytes * height;      // 图像数据大小     
    file_size = data_size + sizeof(header);    // 整个文件的大小     
    
    *((long*)(header + 2)) = file_size;     
    *((long*)(header + 18)) = width;     
    *((long*)(header + 22)) = height;     
    *((long*)(header + 34)) = data_size;     
    
    if (!(fp = fopen(filename, "wb")))     
        return 0;     
    
    fwrite(header, sizeof(unsigned char), sizeof(header), fp);     
    
    if (widthAlignBytes == width * 2)     
    {     
        fwrite(image, sizeof(unsigned char), (size_t)data_size, fp);     
    }     
    else     
    {     
        // 每一行单独写入     
        const static long long DWZERO = 0;     
        for (int i = 0; i < height; i++)     
        {     
            fwrite(image + i * width * 2, sizeof(unsigned char),     
                (size_t) width * 2, fp);     
            fwrite(&DWZERO, sizeof(unsigned char),     
                widthAlignBytes - width * 2, fp);     
        }     
    }     
    
    fclose(fp);     
    return 1;     
}

typedef struct                       /**** BMP file header structure ****/  
{  
    unsigned int   bfSize;           /* Size of file */  
    unsigned short bfReserved1;      /* Reserved */  
    unsigned short bfReserved2;      /* ... */  
    unsigned int   bfOffBits;        /* Offset to bitmap data */  
} MyBITMAPFILEHEADER;

typedef struct                       /**** BMP file info structure ****/  
{  
    unsigned int   biSize;           /* Size of info header */  
    int            biWidth;          /* Width of image */  
    int            biHeight;         /* Height of image */  
    unsigned short biPlanes;         /* Number of color planes */  
    unsigned short biBitCount;       /* Number of bits per pixel */  
    unsigned int   biCompression;    /* Type of compression to use */  
    unsigned int   biSizeImage;      /* Size of image data */  
    int            biXPelsPerMeter;  /* X pixels per meter */  
    int            biYPelsPerMeter;  /* Y pixels per meter */  
    unsigned int   biClrUsed;        /* Number of colors used */  
    unsigned int   biClrImportant;   /* Number of important colors */  
} MyBITMAPINFOHEADER;  

typedef struct tagRGBQUAD{  
    unsigned char rgbBlue;//蓝色的亮度（值范围为0-255)  
    unsigned char rgbGreen;//绿色的亮度（值范围为0-255)  
    unsigned char rgbRed;//红色的亮度（值范围为0-255)  
    unsigned char rgbReserved;//保留，必须为0  
}RGBQUAD;  



void MySaveBmp(const char *filename,unsigned char *rgbbuf,int width,int height)  
{  
    MyBITMAPFILEHEADER bfh;  
    MyBITMAPINFOHEADER bih;  
    /* Magic number for file. It does not fit in the header structure due to alignment requirements, so put it outside */  
    unsigned short bfType=0x4d42;             
    bfh.bfReserved1 = 0;  
    bfh.bfReserved2 = 0;  
    bfh.bfSize = 2+sizeof(MyBITMAPFILEHEADER) + sizeof(MyBITMAPINFOHEADER)+width*height*3;  
    bfh.bfOffBits = 0x36;  
  
    bih.biSize = sizeof(MyBITMAPINFOHEADER);  
    bih.biWidth = width;  
    bih.biHeight = height;  
    bih.biPlanes = 1;  
    bih.biBitCount = 24;  
    bih.biCompression = 0;  
    bih.biSizeImage = 0;  
    bih.biXPelsPerMeter = 5000;  
    bih.biYPelsPerMeter = 5000;  
    bih.biClrUsed = 0;  
    bih.biClrImportant = 0;  
  
    FILE *file = fopen(filename, "wb");  
    if (!file)  
    {  
        printf("Could not write file\n");  
        return;  
    }  
  
    /*Write headers*/  
    fwrite(&bfType,sizeof(bfType),1,file);  
    fwrite(&bfh,sizeof(bfh),1, file);  
    fwrite(&bih,sizeof(bih),1, file);  
  
    fwrite(rgbbuf,width*height*3,1,file);  
    fclose(file);  
}  

/******************************************************************************
Description.: print a help message
Input Value.: -
Return Value: -
******************************************************************************/
void help(void)
{
    fprintf(stderr, " ---------------------------------------------------------------\n" \
            " Help for output plugin..: "OUTPUT_PLUGIN_NAME"\n" \
            " ---------------------------------------------------------------\n");
}

/******************************************************************************
Description.: clean up allocated ressources
Input Value.: unused argument
Return Value: -
******************************************************************************/
void worker_cleanup_usb(void *arg)
{
    static unsigned char first_run = 1;

    if(!first_run) {
        DBG("already cleaned up ressources\n");
        return;
    }

    first_run = 0;
    OPRINT("cleaning up ressources allocated by worker thread\n");

    free(frame);
}

typedef struct {
    struct jpeg_source_mgr pub;

    unsigned char *jpegdata;
    int jpegsize;
} my_source_mgr;

static void init_source(j_decompress_ptr cinfo)
{
    return;
}

static int fill_input_buffer(j_decompress_ptr cinfo)
{
    my_source_mgr * src = (my_source_mgr *) cinfo->src;

    src->pub.next_input_byte = src->jpegdata;
    src->pub.bytes_in_buffer = src->jpegsize;

    return TRUE;
}

static void skip_input_data(j_decompress_ptr cinfo, long num_bytes)
{
    my_source_mgr * src = (my_source_mgr *) cinfo->src;

    if(num_bytes > 0) {
        src->pub.next_input_byte += (size_t) num_bytes;
        src->pub.bytes_in_buffer -= (size_t) num_bytes;
    }
}

static void term_source(j_decompress_ptr cinfo)
{
    return;
}

static void jpeg_init_src(j_decompress_ptr cinfo, unsigned char *jpegdata, int jpegsize)
{
    my_source_mgr *src;

    if(cinfo->src == NULL) {  /* first time for this JPEG object? */
        cinfo->src = (struct jpeg_source_mgr *)(*cinfo->mem->alloc_small)((j_common_ptr) cinfo, JPOOL_PERMANENT, sizeof(my_source_mgr));
        src = (my_source_mgr *) cinfo->src;
    }

    src = (my_source_mgr *) cinfo->src;
    src->pub.init_source = init_source;
    src->pub.fill_input_buffer = fill_input_buffer;
    src->pub.skip_input_data = skip_input_data;
    src->pub.resync_to_restart = jpeg_resync_to_restart;
    src->pub.term_source = term_source;
    src->pub.bytes_in_buffer = 0; /* forces fill_input_buffer on first read */
    src->pub.next_input_byte = NULL; /* until buffer loaded */

    src->jpegdata = jpegdata;
    src->jpegsize = jpegsize;
}

static void my_error_exit(j_common_ptr cinfo)
{
    DBG("JPEG data contains an error\n");
}

static void my_error_output_message(j_common_ptr cinfo)
{
    DBG("JPEG data contains an error\n");
}

typedef struct {
    int height;
    int width;
    unsigned char *buffer;
    int buffersize;
} decompressed_image;

//unsigned __int16  rgb_24_2_565(int r, int g, int b)  
//{  
//    return (unsigned __int16)(((unsigned(r) << 8) & 0xF800) |   
//            ((unsigned(g) << 3) & 0x7E0)  |  
//            ((unsigned(b) >> 3)));  
//} 
//读取Jpeg图片的数据并返回，如果出错，返回NULL
unsigned char* ReadJpeg(const char* path, int* width, int* height)
{
	FILE *file = fopen( path, "rb" );
	if ( file == NULL )	{
		return NULL;
	}

	struct jpeg_decompress_struct info; //for our jpeg info

 	struct jpeg_error_mgr err; //the error handler
 	info.err = jpeg_std_error(&err);


	jpeg_create_decompress( &info ); //fills info structure
	jpeg_stdio_src( &info, file );        //void

	int ret_Read_Head = jpeg_read_header( &info, 1 ); //int

	if(ret_Read_Head != JPEG_HEADER_OK){
		printf("jpeg_read_header failed\n");
		fclose(file);
		jpeg_destroy_decompress(&info);
		return NULL;
	}
	info.out_color_space = JCS_GRAYSCALE;
    info.quantize_colors = FALSE;
    /* to scale the decompressed image, the fraction could be changed here */
    info.scale_num   = 1;
    info.scale_denom = 1;
    info.dct_method = JDCT_FASTEST;
    info.do_fancy_upsampling = FALSE;

    jpeg_calc_output_dimensions(&info);
	
	unsigned char  bStart = jpeg_start_decompress( &info );
	if(!bStart){
		printf("jpeg_start_decompress failed\n");
		fclose(file);
		jpeg_destroy_decompress(&info);
		return NULL;
	}
	*width = info.output_width;
	int w = *width;
	*height = info.output_height;
	int h = *height;
	int numChannels = info.output_components; // 3 = RGB, 4 = RGBA
	unsigned long dataSize = w * h * numChannels;

	// read RGB(A) scanlines one at a time into jdata[]
	unsigned char *data = (unsigned char *)malloc( dataSize );
	if(!data) return NULL;

	unsigned char* rowptr;
	while ( info.output_scanline < h )
	{
		rowptr = data + info.output_scanline * w * numChannels;
		jpeg_read_scanlines( &info, &rowptr, 1 );
	}

	jpeg_finish_decompress( &info );    

	fclose( file );

	return data;
}



int write_jpeg_file(const char* jpeg_file, unsigned char* rgb_buffer, int width, int height, int quality)  
{  
    struct jpeg_compress_struct cinfo;  
    struct jpeg_error_mgr jerr;  
    int row_stride = 0;  
    FILE* fp = NULL;  
    JSAMPROW row_pointer[1];  
  
    cinfo.err = jpeg_std_error(&jerr);  
  
    jpeg_create_compress(&cinfo);  
    fp = fopen(jpeg_file, "wb");  
    if (fp == NULL)  
    {  
        printf("open file %s failed.\n", jpeg_file);  
        return -1;  
    }  
    jpeg_stdio_dest(&cinfo, fp);  
    cinfo.image_width = width;  
    cinfo.image_height = height;  
    cinfo.input_components = 1;  
    cinfo.in_color_space = JCS_GRAYSCALE;  
  
    jpeg_set_defaults(&cinfo);  
    //jpeg_set_quality(&cinfo, quality, 1);  // todo 1 == true  
    jpeg_start_compress(&cinfo, TRUE);  
    row_stride = width * cinfo.input_components;  
  
    while (cinfo.next_scanline < cinfo.image_height)  
    {  
        row_pointer[0] = &rgb_buffer[cinfo.next_scanline * row_stride];  
        jpeg_write_scanlines(&cinfo, row_pointer, 1);  
    }  
  
    jpeg_finish_compress(&cinfo);  
    jpeg_destroy_compress(&cinfo);  
    fclose(fp);  
  
    return 0;  
}  

int decompress_jpeg(unsigned char *jpeg, int jpegsize, decompressed_image *image)
{
    struct jpeg_decompress_struct cinfo;
    JSAMPROW rowptr[1];
    struct jpeg_error_mgr jerr;

    /* create an error handler that does not terminate MJPEG-streamer */
    cinfo.err = jpeg_std_error(&jerr);
    jerr.error_exit = my_error_exit;
    jerr.output_message = my_error_output_message;

    /* create the decompressor structures */
    jpeg_create_decompress(&cinfo);

    /* initalize the structures of decompressor */
    jpeg_init_src(&cinfo, jpeg, jpegsize);

    /* read the JPEG header data */
    if(jpeg_read_header(&cinfo, TRUE) < 0) {
        jpeg_destroy_decompress(&cinfo);
        DBG("could not read the header\n");
        return 1;
    }

    /*
     * I just expect RGB colored JPEGs, so the num_components must be three
     */
/*
    if(cinfo.num_components != 3) {
        jpeg_destroy_decompress(&cinfo);
        DBG("unsupported number of components (~colorspace)\n");
        return 1;
    }
*/
	//cinfo.num_components = 1;
    /* just use RGB output and adjust decompression parameters */
    cinfo.out_color_space = JCS_GRAYSCALE;
    cinfo.quantize_colors = FALSE;
    /* to scale the decompressed image, the fraction could be changed here */
    cinfo.scale_num   = 1;
    cinfo.scale_denom = 1;
    cinfo.dct_method = JDCT_FASTEST;
    cinfo.do_fancy_upsampling = FALSE;

    jpeg_calc_output_dimensions(&cinfo);

    /* store the image information */
    image->width = cinfo.output_width;
    image->height = cinfo.output_height;

    /*
     * just allocate a new buffer if not already allocated
     * pay a lot attention, that the calling function has to ensure, that the buffer
     * must be large enough
     */
    if(image->buffer == NULL) {
        image->buffersize = image->width * image->height * cinfo.out_color_components;
        /* the calling function has to ensure that this buffer will become freed after use! */
        image->buffer = malloc(image->buffersize);
        if(image->buffer == NULL) {
            jpeg_destroy_decompress(&cinfo);
            DBG("allocating memory failed\n");
            return 1;
        }
    }

    /* start to decompress */
    if(jpeg_start_decompress(&cinfo) < 0) {
        jpeg_destroy_decompress(&cinfo);
        DBG("could not start decompression\n");
        return 1;
    }
	printf("cinfo.output_components is %d\n",cinfo.output_components);
	
	printf("cinfo.out_color_components is %d\n",cinfo.out_color_components);
    while(cinfo.output_scanline < cinfo.output_height) {
        rowptr[0] = (JSAMPROW)(unsigned char *)image->buffer + cinfo.output_scanline * image->width * cinfo.output_components;

        if(jpeg_read_scanlines(&cinfo, rowptr, (JDIMENSION) 1) < 0) {
            jpeg_destroy_decompress(&cinfo);
            DBG("could not decompress this line\n");
            return 1;
        }
    }

    if(jpeg_finish_decompress(&cinfo) < 0) {
        jpeg_destroy_decompress(&cinfo);
        DBG("could not finish compression\n");
        return 1;
    }

    /* all is done */
    jpeg_destroy_decompress(&cinfo);

    return 0;
}

/*参数为：
 *返回图片的宽度(w_Dest),
 *返回图片的高度(h_Dest),
 *返回图片的位深(bit_depth),
 *源图片的RGB数据(src),
 *源图片的宽度(w_Src),
 *源图片的高度(h_Src)
 */
unsigned char* do_stretch_linear(int w_Dest,int h_Dest,int bit_depth,unsigned char *src,int w_Src,int h_Src)
{
	int sw = w_Src-1, sh = h_Src-1, dw = w_Dest-1, dh = h_Dest-1;
	int B, N, x, y;
	int nPixelSize = bit_depth/8;
	unsigned char *pLinePrev,*pLineNext;
	unsigned char *pDest = (unsigned char*)malloc(w_Dest*h_Dest*bit_depth/8);
	
	//unsigned char pDest[DISPLAY_SIZE*DISPLAY_DEEPTH/8];// = (unsigned char*)malloc(w_Dest*h_Dest*bit_depth/8);
	unsigned char *tmp;
	unsigned char *pA,*pB,*pC,*pD;

	for(int i=0;i<=dh;++i)
	{
		tmp =pDest + i*w_Dest*nPixelSize;
		y = i*sh/dh;
		N = dh - i*sh%dh;
		pLinePrev = src + (y++)*w_Src*nPixelSize;
		//pLinePrev =(unsigned char *)aSrc->m_bitBuf+((y++)*aSrc->m_width*nPixelSize);
		pLineNext = (N==dh) ? pLinePrev : src+y*w_Src*nPixelSize;
		//pLineNext = ( N == dh ) ? pLinePrev : (unsigned char *)aSrc->m_bitBuf+(y*aSrc->m_width*nPixelSize);
		for(int j=0;j<=dw;++j)
		{
			x = j*sw/dw*nPixelSize;
			B = dw-j*sw%dw;
			pA = pLinePrev+x;
			pB = pA+nPixelSize;
			pC = pLineNext + x;
			pD = pC + nPixelSize;
			if(B == dw)
			{
				pB=pA;
				pD=pC;
			}

			for(int k=0;k<nPixelSize;++k)
			{
				*tmp++ = ( unsigned char )( int )(
					( B * N * ( *pA++ - *pB - *pC + *pD ) + dw * N * *pB++
					+ dh * B * *pC++ + ( dw * dh - dh * B - dw * N ) * *pD++
					+ dw * dh / 2 ) / ( dw * dh ) );
			}
		}
	}
	return pDest;
}




int usb_transmit(void *context, const unsigned char * apdu,int apdu_len)
{

    int ret = 0;

	unsigned char apdu_return[1024] = {0};

	unsigned char output[64] = {0};
    
    ret = luareader_transmit(context, apdu, apdu_len, apdu_return, sizeof(apdu_return),3000);
	
	print_rec(apdu_return,ret);
	
    return ret;


}

/******************************************************************************
Description.: this is the main worker thread
              it loops forever, grabs a fresh frame, decompressed the JPEG
              and displays the decoded data using SDL
Input Value.:
Return Value:
******************************************************************************/
void *worker_thread_usb(void *arg)
{
    int frame_size = 0, firstrun = 1;
	int i,j;
	unsigned short  rgb_565_data;
	static int max_frame_size = 2048 * 1024;
	
	unsigned char *push_data;//[DISPLAY_SIZE*DISPLAY_DEEPTH/8];
	
    unsigned char *tmp_framebuffer = NULL;

    decompressed_image rgbimage;

    char buffer1[1024] = {0}, buffer2[1024] = {0};
    unsigned long long counter = 0;
    time_t t;
    struct tm *now;
	int ret = 0;

	unsigned char  send_data[2048 +4+ 2 + 1+2] = {0};
	int send_cnt = 0;
	int last_len =0;

	unsigned char display_RGB565[DISPLAY_SIZE*2] = {0};
	
	void * context = luareader_new(0, NULL, NULL);
	
    ret = luareader_connect(context,"1-1.2");

    if(ret < 0){
		printf("\nopen usb error\n");
    }
    /* initialze the buffer for the decompressed image */
    rgbimage.buffersize = 0;
    rgbimage.buffer = NULL;


    /* just allocate a large buffer for the JPEGs */
    if((frame = malloc(max_frame_size)) == NULL) {
        OPRINT("not enough memory for worker thread\n");
        exit(EXIT_FAILURE);
    }
	
    /* set cleanup handler to cleanup allocated ressources */
    pthread_cleanup_push(worker_cleanup_usb, NULL);
	//printf("\no pos\n");

	//clear dispaly
	print_send(display_clear, sizeof(display_clear));
	usb_transmit(context, display_clear, sizeof(display_clear) );
	//send display position 	
	print_send(display_pos, sizeof(display_pos));
	usb_transmit(context, display_pos, sizeof(display_pos) );


	
    while(!pglobal->stop) {
		
        printf("waiting for fresh frame\n");
		pthread_mutex_lock(&pglobal->in[input_number].db);
		pthread_cond_wait(&pglobal->in[input_number].db_update, &pglobal->in[input_number].db);


        /* read buffer */
        frame_size = pglobal->in[input_number].size;
		
        /* check if buffer for frame is large enough, increase it if necessary */
        if(frame_size > max_frame_size) {
            DBG("increasing buffer size to %d\n", frame_size);

            max_frame_size = frame_size + (1 << 16);
            if((tmp_framebuffer = realloc(frame, max_frame_size)) == NULL) {
                pthread_mutex_unlock(&pglobal->in[input_number].db);
                LOG("not enough memory\n");
                return NULL;
            }

            frame = tmp_framebuffer;
        }

        /* copy frame to our local buffer now */
        memcpy(frame, pglobal->in[input_number].buf, frame_size);

        /* allow others to access the global buffer again */
        pthread_mutex_unlock(&pglobal->in[input_number].db);
		//printf("\n1 pos\n");
        /* decompress the JPEG and store results in memory */
        if(decompress_jpeg(frame, frame_size, &rgbimage)) {
            DBG("could not properly decompress JPEG data\n");
            continue;
        }
		printf("JPEG frame size is %d\n",frame_size);
        push_data = do_stretch_linear(DISPLAY_WIDTH,DISPLAY_HIGH,24, rgbimage.buffer,\
            rgbimage.width, rgbimage.height); 
		printf("RGB frame width is %d,height is %d\n",rgbimage.width,rgbimage.height);
		i = 0;
		j = 0;
		while(i <=DISPLAY_SIZE * 3){
			//push_data
			rgb_565_data = rgb_24_2_565(push_data[i] ,push_data[i + 1], push_data[i +2]);
			display_RGB565[j] = (unsigned char)((rgb_565_data&0xFF00)>>8);
			display_RGB565[j + 1] = (unsigned char)((rgb_565_data&0x00ff));
			j = j + 2;
			i = i + 3;
		}
			
			

#if 1

		
		//send display start and first frame
		print_send(display_start, sizeof(display_start));
		memcpy(send_data,display_start,sizeof(display_start));
		memcpy(&send_data[sizeof(display_start)],display_RGB565,SEND_BUFFER);
		usb_transmit(context, send_data, sizeof(display_start) +SEND_BUFFER );
		send_cnt++;
		
		//send dispaly data
		while((DISPLAY_SIZE*DISPLAY_DEEPTH/8 - SEND_BUFFER*send_cnt) > SEND_BUFFER){

			
			memcpy(send_data,display_data,sizeof(display_data));
			memcpy(&send_data[sizeof(display_data)],display_RGB565 + SEND_BUFFER*send_cnt,SEND_BUFFER);
			
			print_send(send_data, sizeof(display_data) );
			usb_transmit(context, send_data, sizeof(display_data) +SEND_BUFFER);
			
			send_cnt++;
		}

		last_len = DISPLAY_SIZE*DISPLAY_DEEPTH/8 - SEND_BUFFER*send_cnt;
		printf("last frame data is %d\n",last_len);
		//send display end and last frame
		if(last_len > 0){

			
			memcpy(send_data,display_end,sizeof(display_end));

			send_data[sizeof(display_end) ] = (unsigned char)((last_len&0xFF00)>>8);
            send_data[sizeof(display_end) + 1] = (unsigned char)(last_len&0x00FF);
			
			memcpy(&send_data[sizeof(display_end) + 2],display_RGB565 + SEND_BUFFER*send_cnt,last_len);
			
			print_send(send_data, sizeof(display_end));
			usb_transmit(context, send_data, sizeof(display_end) +2+last_len);

		}

		
		send_cnt = 0;
#endif		
#if 0
		//printf("\n2 pos\n");
			/* prepare filename */
			memset(buffer1, 0, sizeof(buffer1));
			memset(buffer2, 0, sizeof(buffer2));

			/* get current time */
			t = time(NULL);
			now = localtime(&t);
			if(now == NULL) {
				perror("localtime");
				return NULL;
			}

			/* prepare string, add time and date values */
			if(strftime(buffer1, sizeof(buffer1), "%%s/%Y_%m_%d_%H_%M_%S_picture_%%09llu.jpg", now) == 0) {
				OPRINT("strftime returned 0\n");
				free(frame); frame = NULL;
				return NULL;
			}

			/* finish filename by adding the foldername and a counter value */
			snprintf(buffer2, sizeof(buffer2), buffer1, folder, counter);

			counter++;
#if 0

			DBG("writing file: %s\n", buffer2);
			
            /* open file for write */
            if((fd = open(buffer2, O_CREAT | O_RDWR | O_TRUNC, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH)) < 0) {
                OPRINT("could not open the file %s\n", buffer2);
                return NULL;
            }

            /* save picture to file */
            if(write(fd, frame, frame_size) < 0) {
                OPRINT("could not write to file %s\n", buffer2);
                perror("write()");
                close(fd);
                return NULL;
            }


            close(fd);
#endif			
		//bmp565_write(rgbimage.buffer, 640, 480, buffer2);
		//MySaveBmp(buffer2, rgbimage.buffer, rgbimage.width, rgbimage.height);
		//write_jpeg_file(buffer2, push_data, DISPLAY_WIDTH, DISPLAY_HIGH, 0);

#endif

		free(push_data);
		//free()
    }
	
	//printf("\nend pos\n");
    pthread_cleanup_pop(1);

    return NULL;
}
/*



int test_jpeg (unsigned char * jpeg)
{
    //zbar_set_verbosity(32);

    zbar_processor_t *proc = zbar_processor_create(0);
    assert(proc);
    if(zbar_processor_init(proc, NULL, 1))
        return(2);

//    zbar_image_t *img = zbar_image_create();
//    zbar_image_set_size(img, 8, 8);
//    zbar_image_set_format(img, fourcc('J','P','E','G'));
//    zbar_image_set_data(img, jpeg, sizeof(jpeg), NULL);

//    zbar_image_t *test = zbar_image_convert(img, fourcc('Y','8','0','0'));
//    if(!test)
//        return(2);
//    printf("converted: %d x %d (%lx) %08lx\n",
//           zbar_image_get_width(test),
//           zbar_image_get_height(test),
//           zbar_image_get_data_length(test),
//           zbar_image_get_format(test));

//    if(zbar_process_image(proc, test) < 0)
//        return(3);
//    if(zbar_processor_set_visible(proc, 1))
//        return(4);

//    zbar_processor_user_wait(proc, -1);
//    return(0);
}
*/

/******************************************************************************
Description.: this is the main worker thread
              it loops forever, grabs a fresh frame, decompressed the JPEG
              and displays the decoded data using SDL
Input Value.:
Return Value:
******************************************************************************/
void *worker_thread_qr(void *arg)
{
    int frame_size = 0, firstrun = 1;
	int i,j;
	unsigned short  rgb_565_data;
	static int max_frame_size = 2048 * 1024;
	
	unsigned char *push_data;//[DISPLAY_SIZE*DISPLAY_DEEPTH/8];
	
    unsigned char *tmp_framebuffer = NULL;

    decompressed_image rgbimage;

    char buffer1[1024] = {0}, buffer2[1024] = {0};
    unsigned long long counter = 0;
    time_t t;
    struct tm *now;
	int ret = 0;

	int send_cnt = 0;
	int last_len =0;

	unsigned char display_RGB565[DISPLAY_SIZE*2] = {0};
	
    /* initialze the buffer for the decompressed image */
    rgbimage.buffersize = 0;
    rgbimage.buffer = NULL;
	/* obtain image data */
	int width = 0, height = 0;
	void *raw = NULL;
	zbar_image_scanner_t *scanner = NULL;

    /* just allocate a large buffer for the JPEGs */
    if((frame = malloc(max_frame_size)) == NULL) {
        OPRINT("not enough memory for worker thread\n");
        exit(EXIT_FAILURE);
    }
	
    /* set cleanup handler to cleanup allocated ressources */
    pthread_cleanup_push(worker_cleanup_usb, NULL);
	//printf("\no pos\n");

    while(!pglobal->stop) {
		
        printf("waiting for fresh frame\n");
		pthread_mutex_lock(&pglobal->in[input_number].db);
		pthread_cond_wait(&pglobal->in[input_number].db_update, &pglobal->in[input_number].db);


        /* read buffer */
        frame_size = pglobal->in[input_number].size;
		//printf("\n0 pos\n");
        /* check if buffer for frame is large enough, increase it if necessary */
        if(frame_size > max_frame_size) {
            DBG("increasing buffer size to %d\n", frame_size);

            max_frame_size = frame_size + (1 << 16);
            if((tmp_framebuffer = realloc(frame, max_frame_size)) == NULL) {
                pthread_mutex_unlock(&pglobal->in[input_number].db);
                LOG("not enough memory\n");
                return NULL;
            }

            frame = tmp_framebuffer;
        }

        /* copy frame to our local buffer now */
        memcpy(frame, pglobal->in[input_number].buf, frame_size);

        /* allow others to access the global buffer again */
        pthread_mutex_unlock(&pglobal->in[input_number].db);
		//printf("\n1 pos\n");
        /* decompress the JPEG and store results in memory */
//        if(decompress_jpeg(frame, frame_size, &rgbimage)) {
//            DBG("could not properly decompress JPEG data\n");
//            continue;
//        }
		rgbimage.buffer = ReadJpeg("/tmp/20180508235848.jpg", &rgbimage.width, &rgbimage.height);
		printf("JPEG frame size is %d\n",rgbimage.width*rgbimage.height);


		//test_jpeg(rgbimage.buffer);

		width = rgbimage.width;
		height = rgbimage.height;
		raw = malloc(rgbimage.buffersize);
		memcpy(raw,rgbimage.buffer,rgbimage.buffersize);
		/* create a reader */
	    scanner = zbar_image_scanner_create();
		printf("create scanner\n");
	    /* configure the reader */
	    zbar_image_scanner_set_config(scanner, 0, ZBAR_CFG_ENABLE, 1);
		printf("scanner  config\n");

	    //get_data(argv[1], &width, &height, &raw);
		//_zbar_convert_jpeg_to_y;

		
	    /* wrap image data */
	    zbar_image_t *image = zbar_image_create();
		printf(" create image\n");
	    zbar_image_set_format(image, fourcc('G','R','E','Y'));
	    zbar_image_set_size(image, width, height);
	    zbar_image_set_data(image, raw, width * height, zbar_image_free_data);
		printf(" set image\n");

/*
		zbar_image_t *img = zbar_image_create();
		zbar_image_set_size(img, 640, 480);
		zbar_image_set_format(img, fourcc('J','P','E','G'));
		zbar_image_set_data(img, frame, frame_size, NULL);
		
		printf(" create image\n");
		zbar_image_t *test = zbar_image_convert(img, fourcc('Y','8','0','0'));
		
		printf(" set image\n");
		if(!test)
			return(2);
		printf("converted: %d x %d (%lx) %08lx\n",
			   zbar_image_get_width(test),
			   zbar_image_get_height(test),
			   zbar_image_get_data_length(test),
			   zbar_image_get_format(test));
*/



		
	    /* scan the image for barcodes */
	    int n = zbar_scan_image(scanner, image);
		printf(" scan image\n");
	    /* extract results */
	    const zbar_symbol_t *symbol = zbar_image_first_symbol(image);
	    for(; symbol; symbol = zbar_symbol_next(symbol)) {
	        /* do something useful with results */
	        zbar_symbol_type_t typ = zbar_symbol_get_type(symbol);
	        const char *data = zbar_symbol_get_data(symbol);
	        printf("decoded %s symbol \"%s\"\n",
	               zbar_get_symbol_name(typ), data);
	    }
		printf(" scan result\n");
	    /* clean up */
	    zbar_image_destroy(image);
	    zbar_image_scanner_destroy(scanner);	
		//free(rgbimage.buffer);
		
		printf(" free\n");
#if 1
		//printf("\n2 pos\n");
			/* prepare filename */
			memset(buffer1, 0, sizeof(buffer1));
			memset(buffer2, 0, sizeof(buffer2));

			/* get current time */
			t = time(NULL);
			now = localtime(&t);
			if(now == NULL) {
				perror("localtime");
				return NULL;
			}

			/* prepare string, add time and date values */
			if(strftime(buffer1, sizeof(buffer1), "%%s/%Y_%m_%d_%H_%M_%S_picture_%%09llu.jpg", now) == 0) {
				OPRINT("strftime returned 0\n");
				free(frame); frame = NULL;
				return NULL;
			}

			/* finish filename by adding the foldername and a counter value */
			snprintf(buffer2, sizeof(buffer2), buffer1, folder, counter);

			counter++;
#if 0
			DBG("writing file: %s\n", buffer2);
			
            /* open file for write */
            if((fd = open(buffer2, O_CREAT | O_RDWR | O_TRUNC, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH)) < 0) {
                OPRINT("could not open the file %s\n", buffer2);
                return NULL;
            }

            /* save picture to file */
            if(write(fd, frame, frame_size) < 0) {
                OPRINT("could not write to file %s\n", buffer2);
                perror("write()");
                close(fd);
                return NULL;
            }


            close(fd);
		#endif	
		//bmp565_write(rgbimage.buffer, 640, 480, buffer2);
		//MySaveBmp(buffer2, rgbimage.buffer, rgbimage.width, rgbimage.height);
		write_jpeg_file(buffer2, rgbimage.buffer, rgbimage.width, rgbimage.height, 0);

#endif

		//free(push_data);

    }
	
    pthread_cleanup_pop(1);

    return NULL;
}

/*** plugin interface functions ***/
/******************************************************************************
Description.: this function is called first, in order to initialise
              this plugin and pass a parameter string
Input Value.: parameters
Return Value: 0 if everything is ok, non-zero otherwise
******************************************************************************/
int output_init_usb(output_parameter *param)
{
    pglobal = param->global;
	if(!(input_number < pglobal->incnt)) {
		OPRINT("ERROR: the %d input_plugin number is too much only %d plugins loaded\n", input_number, pglobal->incnt);
	 	return 1;
	}
		OPRINT("input plugin.....: %d: %s\n", input_number, pglobal->in[input_number].plugin);
    return 0;
}

/******************************************************************************
Description.: calling this function stops the worker thread
Input Value.: -
Return Value: always 0
******************************************************************************/
int output_stop_usb(int id)
{
    DBG("will cancel worker thread\n");
    pthread_cancel(worker_usb);
    return 0;
}

/******************************************************************************
Description.: calling this function creates and starts the worker thread
Input Value.: -
Return Value: always 0
******************************************************************************/
int output_run_usb(int id)
{
    printf("launching usb worker thread\n");
	
//    pthread_create(&(worker_usb), 0, worker_thread_usb, NULL);
//    pthread_detach((worker_usb));

	pthread_create(&(worker_usb), 0, worker_thread_qr, NULL);
    pthread_detach((worker_usb));
    return 0;
}

int output_cmd_usb()
{


}

