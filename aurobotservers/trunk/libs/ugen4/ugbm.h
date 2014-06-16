 /***************************************************************************
//  ugbm.h  
    -------------------
    Christian Andersen  
		chrand@mail.dk
 ***************************************************************************/
// Function:
// Conversion to/from windows 24bit bitmap
// modified from http://www.nyangau.fsnet.co.uk source
//
// History:
// chr 1 apr 2002 Modified from General Bitmap Module
//
 
#ifndef UGBM_H
#define UGBM_H

#ifndef GBM_H
#define	GBM_H

#ifndef BOOLEAN_DEFINED
#define	BOOLEAN_DEFINED
typedef	int BOOLEAN;
#define	TRUE  1
#define	FALSE 0
#endif

#ifndef BASICTYPES_DEFINED
#define	BASICTYPES_DEFINED
typedef unsigned  char  byte;
typedef unsigned short  word;
typedef unsigned  long dword;
#endif

typedef int GBM_ERR;
#define	GBM_ERR_OK		((GBM_ERR) 0)
#define	GBM_ERR_MEM		((GBM_ERR) 1)
#define	GBM_ERR_NOT_SUPP	((GBM_ERR) 2)
#define	GBM_ERR_BAD_OPTION	((GBM_ERR) 3)
#define	GBM_ERR_NOT_FOUND	((GBM_ERR) 4)
#define	GBM_ERR_BAD_MAGIC	((GBM_ERR) 5)
#define	GBM_ERR_BAD_SIZE	((GBM_ERR) 6)
#define	GBM_ERR_READ		((GBM_ERR) 7)
#define	GBM_ERR_WRITE		((GBM_ERR) 8)
#define	GBM_ERR_BAD_ARG		((GBM_ERR) 9)

#define	GBM_FT_R1		0x0001
#define	GBM_FT_R4		0x0002
#define	GBM_FT_R8		0x0004
#define	GBM_FT_R24	0x0008
#define	GBM_FT_W1		0x0010
#define	GBM_FT_W4		0x0020
#define	GBM_FT_W8		0x0040
#define	GBM_FT_W24	0x0080

typedef struct
	{
	const char *short_name;		/* Eg: "Targa"                       */
	const char *long_name;		/* Eg: "Truevision Targa / Vista"    */
	const char *extensions;		/* Eg: "TGA VST"                     */
	int flags;			/* What functionality exists         */
	} GBMFT;

typedef struct { byte r, g, b; } GBMRGB;

#define	PRIV_SIZE 2000

typedef struct
	{
	int w, h, bpp;			/* Bitmap dimensions                 */
	byte priv[PRIV_SIZE];		/* Private internal buffer           */
	} GBM;

#ifndef _GBM_

#define	GBMEXPORT
#define	GBMENTRY

#endif

#endif

#endif
