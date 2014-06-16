/***************************************************************************
//  ugbmbmp.h  
    -------------------
 ***************************************************************************/
// Function:
// Conversion to/from windows 24bit bitmap
// from http://www.nyangau.fsnet.co.uk
//
// History:
// chr 1 apr 2002 Modified from General Bitmap Module
//

#ifndef UGBMBMP_H
#define UGBMBMP_H


////////////////////////////////////////////////////////////

/*

gbmbmp.h - OS/2 1.1, 1.2, 2.0 and Windows 3.0 support

*/
extern GBM_ERR bmp_qft(GBMFT *gbmft);
extern GBM_ERR bmp_rhdr(const char *fn, int fd, GBM *gbm, const char *opt);
extern GBM_ERR bmp_rpal(int fd, GBM *gbm, GBMRGB *gbmrgb);
extern GBM_ERR bmp_rdata(int fd, GBM *gbm, byte *data);
extern GBM_ERR bmp_w(const char *fn, int fd, const GBM *gbm, const GBMRGB *gbmrgb, const byte *data, const char *opt);
extern const char *bmp_err(GBM_ERR rc);

//class Ugbmbmp {
//public:
//	Ugbmbmp();
//	~Ugbmbmp();
//};

#endif
