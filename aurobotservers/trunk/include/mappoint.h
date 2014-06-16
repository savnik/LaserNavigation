# ifndef MAPPOINT_DOT_H
# define MAPPOINT_DOT_H

# include <stdio.h>
# include <string>

# define NAMELEN 128

using namespace std;

/**
    Maptype used in mapbase module. Implements point features.

*/

class mappoint
{
  public:
    /**
    Constructor */
    mappoint();

    /**
    Destructor */
    virtual ~mappoint();

    /**
    \brief Get type of this structure */
    virtual const char *getDataType()
    {
      return("mapbase mappoint");
    }
  
    /**
    \brief Display the map nicely */
    void display();
    
    /**
    \brief Export the object in xml format */
    void xml_export(string *, char *, const char *);
    
  public:
    /// x and y part of point
    double x, y;
    /// Perimeter (based on type)
    double perimeter;
    /// Coordinate base
    //double ccx, ccy, cct;
    /// Belongs to node
    char nodename[NAMELEN];
};

# endif   //MAPPOINT_DOT_H
