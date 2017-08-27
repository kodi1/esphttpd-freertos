/*
 * bmp.h
 *
 *  Created on: Mar 19, 2016
 *      Author: n0ll
 */

#ifndef DRV_BMP_H_
#define DRV_BMP_H_

#include <c_types.h>

typedef struct {
    uint16      type;               /* Magic identifier             */
    uint32      size;               /* File size in bytes           */
    uint32      reserved;
    uint32      offset;             /* Offset to image data, bytes  */
}__attribute__ ((__packed__)) bmp_header_t;

typedef struct {
    uint32      size;               /* Header size in bytes         */
    uint32      width;              /* Width of image               */
    uint32      height;             /* Height of image              */
    uint16      planes;             /* Number of colour planes      */
    uint16      bits;               /* Bits per pixel               */
    uint32      compression;        /* Compression type             */
    uint32      imagesize;          /* Image size in bytes          */
    uint32      xresolution;        /* Pixels per meter             */
    uint32      yresolution;        /* Pixels per meter             */
    uint32      ncolours;           /* Number of colours            */
    uint32      importantcolours;   /* Important colours            */
}__attribute__ ((__packed__)) bmp_info_header_t;

typedef struct {
    uint8   b;
    uint8   g;
    uint8   r;
} bmp_pixel_t;

typedef struct {
    union {
        struct {
            bmp_header_t        header;
            bmp_info_header_t   info;
        };
        uint8   bytes[0];
    };
} bmp_data_t;

#endif /* DRV_BMP_H_ */
