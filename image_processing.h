#ifndef LIB_H
#define LIB_H

#include <ctime>
#include <cstdlib>
#include <cmath>
#include "Pixy/Pixy2SPI_SS.h"
#include "constants.h"

#define K_MAIN_INTERVAL	(100 / kPit1Period)
#define kSpeedTabSize 100

#define INF 1000

bool debugMode = false;

// Pixycam
Pixy2SPI_SS pixy;

#define debug(fmt, ...) \
		if (debugMode) \
        	printf(fmt, ##__VA_ARGS__);

int img[WIDTH];
int nSamples = 0;
float avgWhite = 0;

int getPixel(int col, int row) {
	uint8_t r, g, b;
	pixy.video.getRGB(col, row, &r, &g, &b, false);
	return (r + g + b) / 3;
}

int* getProcessedImage(const int row) {
	// read image and make grayscale
	for (int i = 0; i < WIDTH; ++i) {
		img[i] = getPixel(i, row);
	}

	return img;
}

void addSample(int color) {
	nSamples++;
	avgWhite = avgWhite * (1.0f * (nSamples - 1) / nSamples);
	avgWhite += 1.0f * color / nSamples;
}

bool isWhite(int color, float threshold=0.1f) {
	float err = fabs(avgWhite - color) / avgWhite;
	debug("%d/100\n", (int) (err * 100));
	return err < threshold;
}

void initialSampling() {
	avgWhite = 0;
	nSamples = 0;
	// assume that the middle is white, so we add samples from the middle of the image
	for (int i = MIDDLE - 5; i <= MIDDLE + 5; ++i) {
		addSample(getPixel(i, bottomRow));
	}
}

void getLeftRight(int &left, int &right) {
	left = -INF;
	right = INF;

	if (nSamples == 0) {
		initialSampling();
	}

	for (int i = MIDDLE; i < WIDTH; ++i) {
		if (!isWhite(img[i])) {
			right = i;
			break;
		}
	}

	for (int i = MIDDLE; i >= 0; --i) {
		if (!isWhite(img[i])) {
			left = i;
			break;
		}
	}
}

#endif
