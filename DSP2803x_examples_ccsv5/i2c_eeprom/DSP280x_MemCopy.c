//###########################################################################
// $TI Release: DSP280x V1.20 $
// $Release Date: September 19, 2005 $
//###########################################################################
//#include "global.h"
#include "User\User_Defines.h"
//---------------------------------------------------
//---------------------------------------------------
void LoadDefault(int16 *SourceAddr, int16* SourceEndAddr, int16* DestAddr)
{
SourceAddr +=2;
    *DestAddr++ = *SourceAddr;
    while(SourceAddr < SourceEndAddr)
    { 
       SourceAddr +=3;
       *DestAddr++ = *SourceAddr;
    }   
}
//---------------------------------------------------
//---------------------------------------------------
void MemCopy(Uint16 *SourceAddr, Uint16* SourceEndAddr, Uint16* DestAddr)
{
    while(SourceAddr < SourceEndAddr)
    { 
       *DestAddr++ = *SourceAddr++;
    }    
}
//---------------------------------------------------
//
//---------------------------------------------------
void MemClear(Uint16 *SourceAddr, Uint16* SourceEndAddr, Uint16* DestAddr)
{
   while(SourceAddr < SourceEndAddr)
    { 
       *DestAddr++ = 0;
       *SourceAddr++;
    }
}
//===========================================================================
// End of file.
//===========================================================================
