/*
 * defines.h
 *
 *  Created on: 21/ott/2010
 *      Author: vito
 */

#ifndef DEFINES_H_
#define DEFINES_H_

#define ALA_FISSA	1
#define ELICOTTERO  2
#define QUAD		3
#define QUADX		4
#define ESA			5

// Endian Mangles
// ----------------
#if defined(BIG_ENDIAN)

  #define switch16(A) (A)
  #define switch32(A) (A)

#elif defined(LITTLE_ENDIAN)

  #define switch16(A) ((((uint16_t)(A) & 0xff00) >> 8) | \
                       (((uint16_t)(A) & 0x00ff) << 8))
  #define switch32(A) ((((uint32_t)(A) & 0xff000000) >> 24) | \
                       (((uint32_t)(A) & 0x00ff0000) >> 8)  | \
                       (((uint32_t)(A) & 0x0000ff00) << 8)  | \
                       (((uint32_t)(A) & 0x000000ff) << 24))
#endif


//conversioni
#define D2R			0.017453292519940	//decimali <-> radianti
#define R2D			57.29577951308232

//Gravitazionale
#define G			9.80665
#define G_2_MPS2(g) (g * G)		//da g a metri al secondo al quadrato
#define MPS2_2_G(m) (m * 0.10197162)

#endif /* DEFINES_H_ */
