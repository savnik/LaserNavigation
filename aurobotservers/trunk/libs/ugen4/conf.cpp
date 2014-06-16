// unit conf.cpp;
// ------------------------------------------------------
// Function:
// Unit to hold common configuration file functions
// sets and reads a configfile of the form:
//   / comment determined on first character ('/',';', ' ', '%')
//   / first line MUST be a comment
//   / lines starting with a space are ignored
//   [subject string]
//   key_value = value
//   key2 = 123.456
//   ; this is also a comment
//   [new subject]
//   initvalue=9
//
// the full file is read on creation, and saved on destruction
// limited in number of lines and number of chars per line - see conf.h file
//
// History:
// chr 14 feb 2002 Created - converted to c++
// ------------------------------------------------------

// implementation

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
//#include <fstream.h>
#include <string.h>
#include "conf.h"

#define eoln 10; 

// main configuration file for aplication

/////////////////////////////////////////////////////////

Uconfig::Uconfig(const char *filename)
{ // constructor
  int err = 0;
  //
  strncpy(iniFileName, filename, MaxFileNameLength);
  count = 0;
  err = readConfig();
  saveOnExit = (err == 0);
}

/////////////////////////////////////////////////////////

Uconfig::Uconfig(void)
{ // constructor create empty config file
  iniFileName[0] = '\0';
  count = 0;
  saveOnExit = false;
}

////////////////////////////////////////////////////////

Uconfig::~Uconfig()
{ // destructor
  if (saveOnExit)
    saveConfig();
}

////////////////////////////////////////////////////////

bool Uconfig::HaveSpaceFor(int lines)

{
  return ((MaxItemsInList - count) >= lines);
}

////////////////////////////////////////////////////////

char * doubleToKeyStr(double value, char * pv)
{
//  int e;
//  double v = value;
  //
  sprintf(pv, "%.8g", value);
/*  e = 0;
  if (v == 0)
    sprintf(pv,"0.0");
  else
  {
    while (fabs(v) < 1.0)
    {
      v*=10;
      e--;
    }
    while (fabs(v) > 10.0)
    {
      v /= 10;
      e++;
    }
    if (e == 0)
      sprintf(pv, "%.8f", v);
    else
      sprintf(pv, "%.8fe%d", v, e);
  }*/
  return pv;
}

//////////////////////////////////////////////////////////
// Read file with provided iniFileName
int Uconfig::readConfig(const char *filename)
{
  int err = 0;
  if (strlen(filename) > 0)
  {
    strncpy(iniFileName, filename, MaxFileNameLength);
    err = readConfig();
  }
  else
    err = -1;
  return err;
}

////////////////////////////////////////////////////////

int Uconfig::readConfig(void)
{ // constructor
  int err = 0;
  FILE *ini;
  char sl[MaxCharPerLine];
  char t;
  int i;
  //
  count = 0; // old content discarded
  ini = fopen(iniFileName, "r");
  //
  if (false and (ini == NULL))
  { // no file - create a file with 2 comment lines
    ini = fopen(iniFileName, "w");
    if (ini != NULL)
    {
      fprintf(ini, "/ This file is created automatically\n");
      fprintf(ini, "/ as none existed.\n");
      fprintf(ini, "/ Always keep at least one comment\n");
      fprintf(ini, "/ line on top of file.\n");
      fprintf(ini, "/ First character in {/;# } = comment\n");
      fprintf(ini, "/ Format is case sensitive.\n");
      fprintf(ini, "/ Format (No leading white space):\n");
      fprintf(ini, "[subject string]\n");
      fprintf(ini, "sample_key_value= value\n");
      fprintf(ini, "small double value= 123.456e-12\n");
      fprintf(ini, "sample double = 1234.56\n");
      fprintf(ini, "[new subject]\n");
      fprintf(ini, "initvalue=9\n");
      fprintf(ini, "; NB! Maksimum lines in file: %d\n", MaxItemsInList);
      fprintf(ini, "; NB! Maksimum line length  : %d\n", MaxCharPerLine);
      fprintf(ini, "; samples above can be deleted\n");
      fclose(ini);
    }
    ini = fopen(iniFileName, "r");
  }
  if (ini != NULL)
  { // get a line at a time
    // NB! 'fgets()' do start at the beginning of a line
    //     if the line is longer than 'MaxCharPerLine'
    while (fgets(sl, MaxCharPerLine, ini) != NULL)
    {
      t='-';
      if (strlen(sl)>2)
      {
        switch (sl[0])
        {
          case '%': // matlab style
          case ';': // windows and assembler style
          case '/': // c++ style
          case '#': // linux config style
          case ' ': // white space
            //regarded as comments and are just stored
            t=' ';
            break;
          case '[':
            // this is a key value
            t='s';
            break;
          default:
            // key values
            t='k';
        }
        if ((t != '-') and (count < MaxItemsInList))
        {
          items[count].type = t;
          i=strlen(sl);
          while ((i > 0) and (sl[i] <= ' '))
            sl[i--]=0; // strip off new line char (and white space)
          strcpy(items[count].line, sl);
          count++;
        }
      }
    }
    fclose(ini);
    if (count == 0)
      err = -1;
  }
  else
    err = -1;
  return err;
}

/////////////////////////////////////////////////////////
// save configuration now
int Uconfig::saveConfig(void)
{
  // Writes configuration to file
  int err = 0;
  FILE *ini;
  int i;
  //
  if (strlen(iniFileName)>0)
  {
    ini = fopen(iniFileName, "w");
    if (ini != NULL)
    { //printf("open %s for writeig\n", iniF);
      //
      if (count == 0)
      { // write a single line - create file
        fprintf(ini, "// configuration file\n");
        fprintf(ini, "// created (no date)\n");
      }
      else
      {
        for (i=0; i<count; i++)
        {
          if (items[i].line[0] == '[')
            fprintf(ini,"\n"); // add empty line before new subjects
          fprintf(ini, "%s\n", items[i].line);
          //j=strlen(items[i].line);
          //items[i].line[j] = eoln;  // add end of line
          //items[i].line[j+1] = 0;  // add end of line
          //fputs(items[i].line, ini);
        }
      }
      fclose(ini);
    }
    else
    {
      err = -2; // can not create file
      printf("Config file not saved - can not open '%s' write", iniFileName);
    }
  }
  else
    err = -1; // no filename
  return err;
}

////////////////////////////////////////////////////////////
// change name and save now under the new name now
int Uconfig::saveConfigAs(const char * filename)
{
  int err =0;
  //
  if (strlen(filename)>0)
    strncpy(iniFileName, filename, MaxFileNameLength);
  else
    err = -1; // no filename
  if (err == 0)
    err = saveConfig();
  return err;
}

/////////////////////////////////////////////////////////

const char * Uconfig::strGet(const char *subj,
                       const char *key,
                       const char * def /*= NULL*/)
{ // NB! returned string at end of pointer may change
  //     as it points to a string in the config stack
  //     and this may wary when new items gets stacked
  //     - so make a copy if you need it for good.
  USubjectKey intsk;
  char * sp;
  const char * v;
  //
  v = def;
  intsk = getIndex(subj, key);
  if (intsk.intKey>0)
  { // there is a key, but a '=' is also required
    sp = strchr(items[intsk.intKey].line, '=');
    if (sp != NULL)
      v = &(sp[1]); // return all after the '='
  }
  return v;
}

/////////////////////////////////////////////////////////

int Uconfig::strPut(const char *subj, const char *key, const char *value)
{ // saves a string value to configuration stack
  // NB! not saved until exit, but valid after this call
  USubjectKey intsk;
  int i;
  //

  intsk = getIndex(subj, key);
  //
  if (count == 0)
  { // add a comment on top
    snprintf(items[count++].line, MaxCharPerLine, "/configuration file");
    snprintf(items[count++].line, MaxCharPerLine, "/file name %s", iniFileName);
  }
  if (intsk.intKey > 0)
  { // there is a key, but a '=' is also required
    if ((strlen(key) + strlen(value) +3) < MaxCharPerLine)
    { //printf("changing found key value '%s'\n", items[intsk.intKey].line);
      // insert full key=value
      snprintf(items[intsk.intKey].line, MaxCharPerLine, "%s=%s", key, value);
    }
    else
    {
      printf("Module conf.cpp\n");
      printf("-Key value too long! %d characters\n", strlen(value));
      printf("-Key value: %s\n", value);
      intsk.intKey = -1;
    }
  }
  else
  { // so add new key
    if (intsk.intSubj == 0)
    { // add subject also



      if ((count + 2) < MaxItemsInList)
      { // there must be room for both subject and key value
        snprintf(items[count].line, MaxCharPerLine, "[%s]", subj);
        items[count].type = 's'; // subject
        intsk.intSubj=count;
        count++;
      }
    }
    if ((intsk.intSubj > 0) and ((count + 1) < MaxItemsInList))
    { // room for new item
      if (intsk.intSubj != (count-1))
      { // make room for new key value after this position
        // put this new value just after sublect line
        for (i = count; i > (intsk.intSubj + 1); i--)
        {
          items[i] = items[i-1];
        }
      }
      intsk.intKey = intsk.intSubj + 1;
      // insert full key=value just after subject line
      snprintf(items[intsk.intKey].line, MaxCharPerLine, "%s=%s", key, value);
      items[intsk.intKey].type = 'k';
      count++;
    }
    else
    {
      printf("Module conf.cpp\n");
      printf("-No more room for config strings. Used %d, dropped '%s'\n",
          count, key);
      intsk.intKey = -1;
    }
  }
  return intsk.intKey;
}

/////////////////////////////////////////////////////////

int Uconfig::intPut(const char *subj, const char *key, const int value)
{
  int result;
  char val[MaxCharPerLine];
  //
  sprintf(val, "%d", value);
  result = strPut(subj, key, val);
  //
  return result;
}

/////////////////////////////////////////////////////////

int Uconfig::intGet(const char *subj, const char *key, const int def)
{
  const char * pv;
  int v;
  // set default
  v = def;
  // get value string
  pv = strGet(subj, key, NULL);
  if (pv != NULL)
    // there is a value
    sscanf(pv, "%d", &v);
  //
  return v;
}


/////////////////////////////////////////////////////////

int Uconfig::doublePut(const char *subj, const char *key, const double value)
{
  char s[MaxCharPerLine - MaxKeyLength] = "";
  //
  doubleToKeyStr(value, s);
  return strPut(subj, key, s);
}


/////////////////////////////////////////////////////////

double Uconfig::doubleGet(const char *subj, const char *key, const double def)
{
  const char *pv;
  double v = def;
  //
  pv = strGet(subj, key, NULL);
  if (pv != NULL)
  {
    //printf("got string '%s'\n", pv);
    sscanf(pv,"%lf", &v);
  }
  return v;
}

/////////////////////////////////////////////////////////

int Uconfig::boolPut(const char *subj, const char *key, const bool value)
{
  int result;
  if (value)
    result = strPut(subj, key, "true");
  else
    result = strPut(subj, key, "false");
  return result;
}

/////////////////////////////////////////////////////////

bool Uconfig::boolGet(const char *subj, const char *key, const bool def)
{
  const char *pv;
  int v = def;
  int n;
  //
  pv = strGet(subj, key, NULL);
  if (pv != NULL)
  {
    n = sscanf(pv,"%d", &v);
    if (n != 1)
      // not integer coded boolean
      v = strcmp(pv, "false");
  }
  return (v != 0);
}

/////////////////////////////////////////////////////////

USubjectKey Uconfig::getIndex(const char *subj, const char *key)
{
  //char sl[MaxCharPerLine];
  //char *sp = &sl[0]; // subj: [key] key: name=value
  int i;

  int j;
  int foundSubj;
  int sln=strlen(subj);
  int kln=strlen(key);
  USubjectKey intsk;
  //
  intsk.intSubj=0;
  intsk.intKey=0;
  foundSubj=0;
  //

  for (i=0; i<count; i++)
  {
    if (!intsk.intSubj)
    {
      if (items[i].type == 's')
      {
        if (strncmp(&items[i].line[1], subj, sln) == 0)
        {  // this is the right subject
          //printf("subject found '%s' looking for '%s'\n", &items[i].line[0], subj);
          intsk.intSubj = i;
          foundSubj = i;
        }
      }
    }
    else
    { // subject is found now find key
      if (items[i].type == 's')
      {
        //printf("new subject found %s", &items[i].line[0]);
        intsk.intSubj=0; // new subject, so no value
        break;
      }
      else if (items[i].type == 'k')
      {
        //printf("Found key '%s' looking for %s\n", items[i].line, key);
        if (strncmp(&items[i].line[0], key, kln)==0)
        {  // this is the right key, now is just a '=' needed
          j=strlen(key);
          //printf("looking for '=' in %s\n", items[i].line);
          //printf("Char at j(%d) '%c' \n", j , items[i].line[j]);
          while ((items[i].line[j] != '=') and
                 (items[i].line[j] <= ' ') and
                 (j < MaxCharPerLine))
          {
            j++;  // skip any white space
            //printf("Char at j(%d) '%c' \n", j , items[i].line[j]);
          }
          if (items[i].line[j] == '=')
          {
            intsk.intKey = i;
            break;
          }
        }
      }
    }
  }
  intsk.intSubj = foundSubj;
  return intsk;
}

////////////////////////////////////////////////////////////



