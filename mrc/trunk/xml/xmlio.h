
struct xml_in {
  /* The following are used to return values from xml_in_nibble().
   * Their meanings depend on the function return value.
   *   XML_IN_NONE:
   *      None used
   *   XML_IN_TAG_START:
   *     'a' is tag name
   *     'n' is number of attributes
   *     'attr' contains attributes
   *   XML_IN_TAG_END:
   *     'a' is tag name
   *   XML_IN_TEXT:
   *     'a' is array of text bytes
   *     'n' is number of bytes (not null terminated)
   */

  char *a;
  int n;
  struct xml_attr {
    char *name, *value;
  } *attr;

  int empty;			/* flag: empty element read, automatic close follows */

  char *buf;
  int next;			/* first unparsed character */
  int free;			/* first free char in buf */

  int buflen;
  int attrmax;
} ;


/* Initialise xml_in structure.
 *  buflen: Number of bytes in input buffer. Must be at least as large as the largest
 *          possible tag.
 * attrmax: Max number of tag attributes. Extra attributes will be discarded.
 */
struct xml_in *xml_in_init(int buflen, int attrmax);

/* Read characters from file descriptor and add to input buffer
 * Return value from read() is returned.
 *  >0: number of bytes read
 *   0: end of stream
 *  -1: error. errno == EAGAIN implies non blocking, no data available.
 */
int xml_in_fd(struct xml_in *p, int fd);



#define ENTITIES 5

enum {XML_IN_NONE, XML_IN_TAG_START, XML_IN_TAG_END, XML_IN_TEXT};

int xml_in_nibble(struct xml_in *p);

int getdouble(double *val,char *name,struct xml_in *x);
int gethex(double *val,char *name,struct xml_in *x);
