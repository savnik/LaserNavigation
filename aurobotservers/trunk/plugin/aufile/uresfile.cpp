/***************************************************************************
 *   Copyright (C) 2006 by DTU (Christian Andersen)                        *
 *   jca@elektro.dtu.dk                                                    *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU Lesser General Public License as        *
 *   published by the Free Software Foundation; either version 2 of the    *
 *   License, or (at your option) any later version.                       *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Lesser General Public License for more details.                   *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this program; if not, write to the                 *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#include <termios.h>
#include <stdio.h>
#include <math.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <ctype.h>

#include <urob4/uvarcalc.h>
#include <ugen4/ucommon.h>
#include <urob4/usmltag.h>

#include "uresfile.h"

/////////////////////////////////////////////////////////

UFileItem::UFileItem()
{
  fh = NULL;
  filename[0] = '\0';
}

/////////////////////////////////////////////////////////

UFileItem::~UFileItem()
{
  if (fh != NULL)
  {
    fclose(fh);
    fh = NULL;
  }
}

/////////////////////////////////////////////////////////

bool UFileItem::add(const char * text)
{
  bool result = false;
  if (fh != NULL)
  {
    fprintf(fh, "%s\n", text);
    fflush(fh);
    result = true;
  }
  return result;
}

/////////////////////////////////////////////////////////

void UFileItem::setName(const char * newName)
{
  strncpy(filename, newName, MFNL);
}

/////////////////////////////////////////////////////////

bool UFileItem::closeFile()
{
  bool result;
  if (fh != NULL)
  {
    fclose(fh);
    fh = NULL;
    result = true;
  }
  else
    result = false;
  return result;
}

/////////////////////////////////////////////////////////

bool UFileItem::isOpen()
{
  return fh != NULL;
}

/////////////////////////////////////////////////////////

bool UFileItem::openFile(const char * fileDir, const char * newName)
{
  const int MNL = MAX_FILENAME_LENGTH;
  char fn[MNL];
  //
  if (newName != NULL)
    setName(newName);
  if (fh != NULL)
    closeFile();
  snprintf(fn, MNL, "%s/%s", fileDir, filename);
  fh = fopen(fn, "w");
  return isOpen();
}

/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////

void UResFile::UResFileInit()
{
  setLogName("file");
  openLog();
  // create status variables
  createBaseVar();
  verbose = false;
  openFilesCnt = 0;
}

///////////////////////////////////////////

UResFile::~UResFile()
{
}

///////////////////////////////////////////


void UResFile::createBaseVar()
{
  varFileDir = addVar("filePath",  "./", "s", "(rw) Directory used for all files");
  varOpenFiles = addVar("openFiles",  0.0, "d", "(r) Current number of open files");
  //
}


///////////////////////////////////////////////////////////

bool UResFile::add(const int client, const char * text)
{
  bool result = false;
  UFileItem * fi;
  //
  fi = getItem(client, false);
  if (fi != NULL)
  {
    fi->add(text);
    result = true;
  }
  //
  return result;
}

//////////////////////////////////////////////

bool UResFile::del(const char * name)
{
  const int MNL = MAX_FILENAME_LENGTH;
  char cn[MNL];
  int err;
  //
  snprintf(cn, MNL, "rm -f %s/%s", varFileDir->getValues(), name);
  err = system(cn) ;
  toLog(cn, bool2str(err != -1));
  return err != -1;
}

/////////////////////////////////////////////

const char * UResFile::getList(const char * preStr, char * buff, const int buffCnt)
{
  FILE * pf;
  const int MSL = 500;
  char s[MSL];
  char * p1 = buff;
  const char *p2;
  int n = 0;
  //
  strncat(p1, preStr, buffCnt - n);
  n += strlen(p1);
  p1 = &buff[n];
  //
  snprintf(s, MSL, "ls %s\n", varFileDir->getValues());
  pf = popen(s, "r");
  if (pf != NULL)
  {
    while (not feof(pf))
    {
      p2 = fgets(p1, buffCnt - n, pf);
      if (p2 == NULL)
        strncpy(p1, "(no files)\n", buffCnt - n);
      n += strlen(p1);
      p1 = &buff[n];
      if (buffCnt - n <= 0)
        break;
    }
    pclose(pf);
  }
  return buff;
}

////////////////////////////////////////////////////////

UFileItem * UResFile::getItem(const int client, bool mayCreate)
{
  UFileItem * result = NULL;
  int i;
  for (i = 0; i < openFilesCnt; i++)
  {
    if (openFiles[i]->client == client)
    {
      result = openFiles[i];
      break;
    }
  }
  if (result == NULL and mayCreate and openFilesCnt < MAX_FILE_CNT)
  {
    result = new UFileItem();
    result->client = client;
    openFiles[openFilesCnt++] = result;
  }
  return result;
}

///////////////////////////////////////////////////////

const char * UResFile::getDirName()
{
  return varFileDir->getValues();
}

///////////////////////////////////////////////////////

bool UResFile::newFile(const int client, const char * newName)
{
  bool result = false;
  UFileItem * fi;
  //
  fi = getItem(client, true);
  if (fi != NULL)
  {
    if (fi->isOpen())
      closeFile(client);
    result = fi->openFile(varFileDir->getValues(), newName);
  }
  toLog("created", client, fi->filename, bool2str(result));
  setOpenFilesCnt();
  return varFileDir->getValues();
}

///////////////////////////////////////////////////////

bool UResFile::closeFile(const int client)
{
  UFileItem * fi;
  bool result;
  const int MSL=500;
  char s[MSL];
  //
  fi = getItem(client, false);
  if (fi != NULL)
    result = fi->closeFile();
  else
    result = false;
  toLog("closed", client, getFilename(client, "", s, MSL), bool2str(result));
  setOpenFilesCnt();
  return result;
}

///////////////////////////////////////////////////////

const char * UResFile::getFilename(const int client,
                                   const char * preStr,
                                   char * buff, const int buffCnt)
{
  UFileItem * fi;
  const char * result = NULL;
  //
  fi = getItem(client, false);
  if (fi != NULL)
  {
    snprintf(buff, buffCnt, "%s%s/%s", preStr, varFileDir->getValues(), fi->filename);
    result = buff;
  }
  return result;
}

///////////////////////////////////////////////////////////

const char * UResFile::getFilename(const int client)
{
  UFileItem * fi;
  const char * result = NULL;
  //
  fi = getItem(client, false);
  if (fi != NULL)
    result = fi->filename;
  return result;
}

///////////////////////////////////////////////////////////

int UResFile::setOpenFilesCnt()
{
  int i, n = 0;
  //
  for (i = 0; i < openFilesCnt; i++)
  {
    if (openFiles[i]->isOpen())
      n++;
  }
  varOpenFiles->setInt(n);
  return n;
}
