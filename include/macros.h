
#pragma once
//===== Debugging macros ========================
 #define _DEBUG_

#ifdef _DEBUG_
#define SerialD Serial
#define _PM(a)               \
    SerialD.print(millis()); \
    SerialD.print(F(": "));  \
    SerialD.println(a)
#define _PP(a) SerialD.print(a)
#define _PL(a) SerialD.println(a)
#define _PX(a) SerialD.println(a, HEX)
#else
#define _PM(a)
#define _PP(a)
#define _PL(a)
#define _PX(a)
#endif