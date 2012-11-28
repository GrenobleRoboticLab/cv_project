#ifndef _LRGTOOLS_H__
#define _LRGTOOLS_H__

enum Status {
    S_OK,
    E_FAIL,
    E_NOTINIT,
    E_CONFLICT,
    E_NULLPTR
};

inline bool ISOK(Status status) { return (status == S_OK); }

#endif // _LRGTOOLS_H__
