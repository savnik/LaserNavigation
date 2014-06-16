# ifndef MAPLINE_DOT_H
# define MAPLINE_DOT_H

# include <stdio.h>
# include <math.h>
# include <string>

# define NAMELEN 128

using namespace std;

/**
    Maptype used in rowfinder module. Implements line features.

*/

class mapline
{
  public:
    /**
    Constructor */
    mapline();

    /**
    Destructor */
    virtual ~mapline();

    /**
    \brief Get type of this structure */
    virtual const char *getDataType()
    {
      return("mapbase mapline");
    }
  
    /**
    \brief Display the map nicely */
    void display();
    
    /**
    \brief Fills ABC line parameters from other features */
    void fill_ABC();
    
    /**
    \brief Export the object in xml format */
    void xml_export(string *, char *, const char *);

  public:
    /// Belongs to node
    char nodename[NAMELEN];
    /// x and y part of start point
    double x_s, y_s;
    /// x and y part of end point
    double x_e, y_e;
    /// line parameters in the form Ax + By + C = 0
    double A, B, C;      
    /// Perimeter (based on type)
    double perimeter;
    /// is this map-line to be assumed to be an obstacle too
    /// non-obstacles can be lines across tree rows for better localization
    bool isObstacle;
    /// Coordinate base
    //double ccx, ccy, cct;
    // max length of number string
    static const int MNL = 32;
    /// name string of this line
    char number[MNL];
};

# endif   //MAPLINE_DOT_H
