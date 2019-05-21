#include <iostream>

#include <sys/time.h>

#ifndef _RTimer_
#define _RTimer_

class rtimer {
private:
    struct timeval beg;
public:
    rtimer() {
       gettimeofday(&beg, 0);
    }
    void reset() {
       gettimeofday(&beg, 0);
    }
    float timedifference_msec(struct timeval t0, struct timeval t1){
        return (t1.tv_sec - t0.tv_sec) * 1000.0f + (t1.tv_usec - t0.tv_usec) / 1000.0f;
    }
    bool isOver(double seconds){
        struct timeval current;
        gettimeofday(&current, 0);
        long double elapsed = timedifference_msec(current, beg);
        if (elapsed<seconds*1000.0) return false;
        else return true;
    }
    bool isOverCyclic(double seconds){
        struct timeval current;
        gettimeofday(&current, 0);
        long double elapsed = timedifference_msec(beg, current);
        if (elapsed<seconds*1000.0) return false;
        else {
            gettimeofday(&beg, 0);
            return true;
        }
    }
};



#endif  // timer
