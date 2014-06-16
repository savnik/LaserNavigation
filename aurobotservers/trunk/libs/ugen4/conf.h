// unit conf.h;
// ------------------------------------------------------
// Function:
// Unit to hold common configuration functions
// sets and reads a configfile of the form:
//   / comment determined on first character ('/',';', ' ', '%')
//   / first line MUST be a comment
//   / empty lines are ignored (removed)
//   [subject string]
//   key_value = value
//   key2 = 123.456e-12
//   ; this is also a comment
//   [new subject]
//   initvalue=9
//
// One instance is to be created for each thread (not mulitentrant code)
// The full file is read on creation, and saved on destruction
// limited in number of lines and number of chars per line - see below
//
// History
// chr 14 feb 2002 Module created
// chr 16 apr 2002 extended to more general usage
// ------------------------------------------------------
#ifndef _EX39_CONF
#define _EX39_CONF

#include <stdlib.h>
// interface

#define MaxItemsInList 500
#define MaxCharPerLine 100
#define MaxKeyLength 30
#define MaxFileNameLength 500
#define MAX_FILENAME_LENGTH MaxFileNameLength


/**
Configuration storage item */
typedef struct
{
	char type;
  char line[MaxCharPerLine];
}  UConfigItem;

/**
Used by Uconfig */
typedef struct
{
  int intSubj;
	int intKey;
} USubjectKey;

/** general purpose conversion from double to string in
    1.234e-12 format (unles value >1 and <10, then just 1.234 */
char * doubleToKeyStr(double value, char * pv);

/**
The config class is a string-based registry database, that primarily
is used to load and store configuration information to and from disk.
The format is subject-key based:

[subject] <br>
key=value <br>

Methodes for bool, integer, string and double are implemented. */
class Uconfig
{
private:
  int count;
  UConfigItem items[MaxItemsInList];
  //
public:
  /**
  Filename - relative or absolut - where to save the content. */
  char iniFileName[MaxFileNameLength];
  /**
  Should content be saved before destruction?
  If true and filename is set, it will be saved. */
  bool saveOnExit;
  //
  Uconfig(const char *filename);
  Uconfig(void);
	~Uconfig();
  /**
  Check to see if there is additional space for
  this many lines */
  bool HaveSpaceFor(int lines);
  /**
  Get string (pointer) to configuration sring */
  const char * strGet(const char *subj,
                const char *key,
                const char * def = NULL);
  /**
  Save this string as new key value */
  int strPut(const char *subj, const char *key, const char *value);
  /**
  Integer put */
  int intPut(const char *subj, const char *key, const int value);
  /**
  integer get */
  int intGet(const char *subj, const char *key, const int def);
  /**
  Double put */
  int doublePut(const char *subj, const char *key, const double value);
  /**
  Double get */
  double doubleGet(const char *subj, const char *key, const double def);
  /**
  Save value as either "true" or "false"
  Returns -1 if failed (no space) or some non-negative
  value if successful. */
  int boolPut(const char *subj, const char *key, const bool value);
  /**
  Get a boolean variable.
  If the value is "false" ot 0 (zero) then false is returned
  otherwise true is returned.
  If no value is found, then the default value is returned. */
  bool boolGet(const char *subj, const char *key, const bool def);
	// NB! other class types are saved directly by the class
	//
	/**
  Clear content of configuration structure */
  void clear(void) {count = 0;};
  /**
  Clear and read data from this file */
  int readConfig(const char * filename);
  /**
  Read file with name in iniFileName */
  int readConfig(void);
  /**
  Save configuration now */
  int saveConfig(void);
  /**
  Change name and save now under the new name now */
  int saveConfigAs(const char * filename);
//
private:
  USubjectKey getIndex(const char *subj, const char *key);
};

//extern Uconfig conf;

#endif // _EX39_CONF
