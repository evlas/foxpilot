/*
 * matrix.c
 *
 *  Created on: 18.11.2010
 *      Author: Laurens Mackay
 */
#include <stdio.h>
#include <math/matrix.h>

matrix_t matrix_create(const int rows, const int cols, m_elem * a) {
	matrix_t ret;
	ret.rows = rows;
	ret.cols = cols;
	ret.a = a;
	return ret;
}

void matrix_add(const matrix_t a, const matrix_t b, matrix_t c) {
	int i,j;

	if (a.rows != c.rows || a.cols != c.cols || b.rows != c.rows || b.cols != c.cols) {
		perror("matrix_add: Dimension mismatch");
	}

	for (i = 0; i < c.rows; i++) {
		for (j = 0; j < c.cols; j++) {
			M(c, i, j) = M(a, i, j) + M(b, i, j);
		}
	}
}

void matrix_sub(const matrix_t a, const matrix_t b, matrix_t c) {
	int i,j;

	if (a.rows != c.rows || a.cols != c.cols || b.rows != c.rows || b.cols != c.cols) {
		perror("matrix_sub: Dimension mismatch");
	}
	for (i = 0; i < c.rows; i++) {
		for (j = 0; j < c.cols; j++) {
			M(c, i, j) = M(a, i, j) - M(b, i, j);
		}
	}
}

void matrix_mult(const matrix_t a, const matrix_t b, matrix_t c) {
	int i,j,k;

	if (a.rows != c.rows || b.cols != c.cols || a.cols != b.rows) {
		perror("matrix_mult: Dimension mismatch");
	}
	for (i = 0; i < a.rows; i++) {
		for (j = 0; j < b.cols; j++) {
			M(c, i, j) = 0;
			for (k = 0; k < a.cols; k++) {
				M(c, i, j) += M(a, i, k) * M(b, k, j);
			}
		}
	}
}

void matrix_mult_scalar(const float f, const matrix_t a, matrix_t c) {
	int i,j;

	if (a.rows != c.rows || a.cols != c.cols) {
		perror("matrix_mult_scalar: Dimension mismatch");
	}
	for (i = 0; i < c.rows; i++) {
		for (j = 0; j < c.cols; j++) {
			M(c, i, j) = f * M(a, i, j);
		}
	}
}


void matrix_mult_element(const matrix_t a, const matrix_t b, matrix_t c) {
	int i,j;

	if (a.rows != c.rows || a.cols != c.cols || b.rows != c.rows || b.cols != c.cols) {
		perror("matrix_mult_element: Dimension mismatch");
	}
	for (i = 0; i < c.rows; i++) {
		for (j = 0; j < c.cols; j++) {
			M(c, i, j) = M(a, i, j) * M(b, i, j);
		}
	}
}

