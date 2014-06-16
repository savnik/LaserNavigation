#define	S2L_IBSIZE	2048
#define S2L_NLINES	50
#define S2L_LINESIZE    255
enum {RECLINE, AFTERCR, AFTERLF};
typedef struct { 
	char ib[S2L_IBSIZE] ;  		    // inputbuffer
	int  Nib;			    // number of characters in inputbuffer
	char lb [S2L_NLINES][S2L_LINESIZE] ; // linebuffer
	int  linein;			    // current input line
	int  lineout;			    // current output line
	int  nlines;			    // number of output lines
	int  state;
	int  cp;			    // character pointer to inputline
	}stream2line_type;
	
void stream2lineinit(stream2line_type *p);
void stream2line(stream2line_type *p);
void getnextline(char *line,stream2line_type * p);

	
	
	
