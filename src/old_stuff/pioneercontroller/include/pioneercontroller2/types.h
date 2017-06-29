#ifndef INCL_TYPES_H
#define INCL_TYPES_H
#include <exception>
enum ActionType {NONE = 0, LEFTRIGHT, FORWARD, TURN, ENSLAVE, RELEASE, STATUS, OPTIONS, TWIST};

typedef std::pair<ActionType, std::pair<int, int> > pairActionpairInt;
typedef std::pair<ActionType, std::pair<double, double> > pairActionpairDouble;
typedef std::pair<int, int> pairInt;
typedef std::pair<double, double> pairDouble;

#define MAX_SPEED (1500)

#define TO_RAD (M_PI / 180.0)

#endif
