/* Copying and distribution of this file, with or without modification,
 * are permitted in any medium without royalty. This file is offered as-is,
 * without any warranty.
 */

/*! @file process_frame.c
 * @brief Contains the actual algorithm and calculations.
 */

/* Definitions specific to this application. Also includes the Oscar main header file. */
#include "template.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

OSC_ERR OscVisDrawBoundingBoxBW(struct OSC_PICTURE *picIn, struct OSC_VIS_REGIONS *regions, uint8 Color);

void ProcessFrame(uint8 *pInputImg)
{
	int c, r, threshold, i;
	double temp;
	int nc = OSC_CAM_MAX_IMAGE_WIDTH/2;
	int siz = sizeof(data.u8TempImage[GRAYSCALE]);

	int Shift = 7;
	short Beta = 2;//the meaning is that in floating point the value of Beta is = 6/(1 << Shift) = 6/128 = 0.0469
	uint8 MaxForeground = 120;//the maximum foreground counter value (at 15 fps this corresponds to less than 10s)

	struct OSC_PICTURE Pic1, Pic2;//we require these structures to use Oscar functions
	struct OSC_VIS_REGIONS ImgRegions;//these contain the foreground objects

	uint16 histogramm[256];
	uint w0;
	uint w1;
	uint u0;
	uint u1;
	uint o0;
	uint o1;
	double schwellwert[256];

	memset(histogramm, 0, sizeof(histogramm));
	memset(schwellwert, 0, sizeof(schwellwert));

	// Create the histogramm
	for(r = 0; r < siz; r+= nc)
	{
		for(c = 0; c < nc; c++)
		{
			histogramm[data.u8TempImage[GRAYSCALE][r+c]]++;
		}
	}

	for (threshold = 0; threshold < 256; threshold++) {
		w0 = 0; w1=0; u0=0; u1=0; o0=0; o1=0;

		for (i=0; i < 256; i++) {
			if (i < threshold) {
				w0 += histogramm[i];
				u0 += histogramm[i]*i;
			} else {
				w1 += histogramm[i];
				u1 += histogramm[i]*i;
			}
		}

		u0 = u0 / w0;
		u1 = u1 / w1;

		for (i = 0; i < 256; i ++) {
			if (i < threshold) {
				o0 += histogramm[i]*((i-u0)^2);
			} else {
				o1 += histogramm[i]*((i-u1)^2);
			}
		}

		o0 = o0 / w0;
		o1 = o1 / w1;

		schwellwert[threshold] = sqrt(w0*(o0^2)+w1*(o1^2));
	}

	threshold = 0;
	temp = schwellwert[0];
	for (i=1;i<256;i++){
		if (schwellwert[i] < temp) {
			temp = schwellwert[i];
			threshold = i;
		}
	}

	for(r = 0; r < siz; r+= nc)
	{
		for(c = 0; c < nc; c++)
		{
			data.u8TempImage[THRESHOLD][r+c] = (short) data.u8TempImage[GRAYSCALE][r+c] > threshold ? 0 : 0xff;
		}
	}

	/*
	{
		//for debugging purposes we log the background values to console out
		//we chose the center pixel of the image (adaption to other pixel is straight forward)
		int offs = nc*(OSC_CAM_MAX_IMAGE_HEIGHT/2)/2+nc/2;

		OscLog(INFO, "%d %d %d %d %d\n", (int) data.u8TempImage[GRAYSCALE][offs], (int) data.u8TempImage[BACKGROUND][offs], (int) data.u8TempImage[BACKGROUND][offs]-data.ipc.state.nThreshold,
										 (int) data.u8TempImage[BACKGROUND][offs]+data.ipc.state.nThreshold, (int) data.u8TempImage[FGRCOUNTER][offs]);
	}
	*/

	for(r = nc; r < siz-nc; r+= nc)/* we skip the first and last line */
	{
		for(c = 1; c < nc-1; c++)/* we skip the first and last column */
		{
			unsigned char* p = &data.u8TempImage[THRESHOLD][r+c];
			data.u8TempImage[EROSION][r+c] = *(p-nc-1) & *(p-nc) & *(p-nc+1) &
											 *(p-1)    & *p      & *(p+1)    &
											 *(p+nc-1) & *(p+nc) & *(p+nc+1);
		}
	}

	for(r = nc; r < siz-nc; r+= nc)/* we skip the first and last line */
	{
		for(c = 1; c < nc-1; c++)/* we skip the first and last column */
		{
			unsigned char* p = &data.u8TempImage[EROSION][r+c];
			data.u8TempImage[DILATION][r+c] = *(p-nc-1) | *(p-nc) | *(p-nc+1) |
											  *(p-1)    | *p      | *(p+1)    |
											  *(p+nc-1) | *(p+nc) | *(p+nc+1);
		}
	}

	//wrap image DILATION in picture struct
	Pic1.data = data.u8TempImage[DILATION];
	Pic1.width = nc;
	Pic1.height = OSC_CAM_MAX_IMAGE_HEIGHT/2;
	Pic1.type = OSC_PICTURE_GREYSCALE;
	//as well as EROSION (will be used as output)
	Pic2.data = data.u8TempImage[EROSION];
	Pic2.width = nc;
	Pic2.height = OSC_CAM_MAX_IMAGE_HEIGHT/2;
	Pic2.type = OSC_PICTURE_BINARY;//probably has no consequences
	//have to convert to OSC_PICTURE_BINARY which has values 0x01 (and not 0xff)
	OscVisGrey2BW(&Pic1, &Pic2, 0x80, false);

	//now do region labeling and feature extraction
	OscVisLabelBinary( &Pic2, &ImgRegions);
	OscVisGetRegionProperties( &ImgRegions);

	//OscLog(INFO, "number of objects %d\n", ImgRegions.noOfObjects);
	//plot bounding boxes both in gray and dilation image
	Pic2.data = data.u8TempImage[GRAYSCALE];
	OscVisDrawBoundingBoxBW( &Pic2, &ImgRegions, 255);
	OscVisDrawBoundingBoxBW( &Pic1, &ImgRegions, 128);

}


/* Drawing Function for Bounding Boxes; own implementation because Oscar only allows colored boxes; here in Gray value "Color"  */
/* should only be used for debugging purposes because we should not drawn into a gray scale image */
OSC_ERR OscVisDrawBoundingBoxBW(struct OSC_PICTURE *picIn, struct OSC_VIS_REGIONS *regions, uint8 Color)
{
	 uint16 i, o;
	 uint8 *pImg = (uint8*)picIn->data;
	 const uint16 width = picIn->width;
	 for(o = 0; o < regions->noOfObjects; o++)//loop over regions
	 {
		 /* Draw the horizontal lines. */
		 for (i = regions->objects[o].bboxLeft; i < regions->objects[o].bboxRight; i += 1)
		 {
				 pImg[width * regions->objects[o].bboxTop + i] = Color;
				 pImg[width * (regions->objects[o].bboxBottom - 1) + i] = Color;
		 }

		 /* Draw the vertical lines. */
		 for (i = regions->objects[o].bboxTop; i < regions->objects[o].bboxBottom-1; i += 1)
		 {
				 pImg[width * i + regions->objects[o].bboxLeft] = Color;
				 pImg[width * i + regions->objects[o].bboxRight] = Color;
		 }
	 }
	 return SUCCESS;
}


