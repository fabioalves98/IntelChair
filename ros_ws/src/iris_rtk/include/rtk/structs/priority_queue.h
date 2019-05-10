//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#pragma once

#include <map>
#include <queue>

namespace rtk {

/**
 * Priority queue for integer as priority.
 * A priority queue that uses buckets to group elements with the same priority.
 * The individual buckets are unsorted, which increases efficiency if these groups are large.
 */
template <typename T>
class PriorityQueue {
    typedef std::map<int, std::queue<T> > bucket;

    int count;
    bucket buckets;
    typename bucket::iterator nextPop;

public:
    /** Standard constructor.
    */
    PriorityQueue()
    {
        nextPop = buckets.end();
        count = 0;
    }

    virtual ~PriorityQueue(){}

    inline void clear() { buckets.clear(); }

    //! Checks whether the Queue is empty
    inline bool empty()
    { return (count==0); }

    //! push an element
    void push(int prio, T t)
    {
        buckets[prio].push(t);
        if (nextPop == buckets.end() || prio < nextPop->first)
            nextPop = buckets.find(prio);

        count++;
    }

    //! return and pop the element with the lowest priority */
    T pop()
    {
        while (nextPop!=buckets.end() && nextPop->second.empty())
            ++nextPop;

        T p = nextPop->second.front();
        nextPop->second.pop();
        if (nextPop->second.empty()) {
            typename bucket::iterator it = nextPop;
            nextPop++;
            buckets.erase(it);
        }
        count--;
        return p;
    }

    inline int size()
    { return count; }

    inline int getNumBuckets()
    { return buckets.size(); }

};

} /* rtk */

