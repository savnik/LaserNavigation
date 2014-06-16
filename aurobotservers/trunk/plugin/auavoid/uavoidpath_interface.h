#ifndef __UAVOIDPATH_INTERFACE_H
#define __UAVOIDPATH_INTERFACE_H

class UAvoidPathInterface
{
public:
    UAvoidPathInterface() {}
    virtual ~UAvoidPathInterface() {}


private:
    UAvoidPathInterface( const UAvoidPathInterface& source );
    void operator = ( const UAvoidPathInterface& source );
};


#endif // __UAVOIDPATH_INTERFACE_H
