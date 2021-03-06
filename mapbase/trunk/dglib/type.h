/* LIBDGL -- a Directed Graph Library implementation
 * Copyright (C) 2002 Roberto Micarelli
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

/*
 * best view tabstop=4
 */

#include <stdint.h>

#ifndef _DGL_TYPE_H_
#define _DGL_TYPE_H_ 1

/*
 * local endianess
 */
#ifdef WORDS_BIGENDIAN
#define  G_XDR 1
#else
#define  G_NDR 1
#endif


/* Changed to use stdint by Anders Billesø Beck (DTU) */
typedef uint8_t dglByte_t;
typedef int32_t dglInt32_t;
typedef int64_t dglInt64_t;
	
#endif
