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

// line buffer
int lines[WIDTH + 1];

// buffer should we allocd with size WIDTH + 1
// returns an int array where the first element is the size and the ones to follow are index values where lines were detected
int* getProcessedImage(const int row) {
	// read image and make greyscale
	int img[WIDTH];
	uint8_t r, g, b;
	for (int i = 0; i < WIDTH; ++i) {
		pixy.video.getRGB(i, row, &r, &g, &b, false);
		img[i] = (r + g + b) / 3;
	}

	// sobel bs
	int sobel[WIDTH];
	sobel[0] = 0;
	sobel[WIDTH - 1] = 0;
	for (int i = 1; i < WIDTH - 1; ++i) {
		sobel[i] = abs(img[i - 1] - img[i + 1]);
	}

	// non-maximum suppresion
	const int filter1 = 15;
	int max_sobel = 0;
	int suppr[WIDTH];
	suppr[0] = 0;
	for (int i = 1; i < WIDTH - 1; ++i) {
		if (sobel[i] < sobel[i - 1] || sobel[i] < sobel[i + 1] || sobel[i] < filter1) {
			suppr[i] = 0;
			continue;
		}

		suppr[i] = sobel[i];
		if (suppr[i] > max_sobel) {
			max_sobel = suppr[i];
		}
	}

	suppr[WIDTH - 1] = 0;

	// gaussian scaling bs
	const double filter2 = 0.15;
	int line_cnt = 0;
	for (int i = 0; i < WIDTH; ++i) {
		if (suppr[i] > 0) {
			// normalize between 0 and 1
			double x = (double) suppr[i] / max_sobel;

			// apply f(x) = (e^(x^2 + 5x - 1) - e^-1) / e^5
			x = (exp(x * x + 5 * x - 1) - 0.367879441) / 148.413159103;
			if (x > filter2) {
				lines[++line_cnt] = i;
			}
		}
	}

	lines[0] = line_cnt;

	return lines;
}

void getLeftRight(int &left, int &right) {
	const int min_line_threshold = 30;
	int line_cnt = lines[0];

	left = -INF;
	right = INF;

	if (line_cnt == 0 || line_cnt > 4) {
		return;
	}

	if (line_cnt == 4) {
		left = lines[2];
		right = lines[3];
	} else if (line_cnt == 3) {
		if (abs(lines[2] - lines[1]) < abs(lines[2] - lines[3])) {
			left = lines[2];
			right = lines[3];
		} else {
			left = lines[1];
			right = lines[2];
		}
	} else if (line_cnt == 2) {
		if (abs(lines[2] - lines[1]) < min_line_threshold) {
			if (lines[2] < WIDTH / 2) {
				left = lines[2];
			} else {
				right = lines[2];
			}
		} else {
			left = lines[1];
			right = lines[2];
		}
	} else {
		if (lines[1] < WIDTH / 2) {
			left = lines[1];
		} else {
			right = lines[1];
		}
	}
}

#endif
