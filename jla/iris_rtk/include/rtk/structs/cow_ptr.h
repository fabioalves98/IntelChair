//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#pragma once

#include <memory>
#include <mutex>

namespace rtk {

// https://en.wikibooks.org/wiki/More_C%2B%2B_Idioms/Copy-on-write
template <class T>
class COWPtr {
public:
    typedef std::shared_ptr<T> Ptr;

public:

    COWPtr(T* ptr = 0) : refptr_(ptr)
    {}

    COWPtr(Ptr& refptr) : refptr_(refptr)
    {}

    COWPtr(const COWPtr<T>& other)
    {
        refptr_ = other.refptr_;
    }

    inline COWPtr<T>& operator=(const COWPtr<T>& other)
    {
        refptr_ = other.refptr_;
        return *this;
    }

    inline T* get()
    {
        return refptr_.get();
    }

    inline const T* get() const
    {
        return refptr_.get();
    }

    inline bool unique() const
    {
        return refptr_.unique();
    }

    inline long use_count() const
    {
        return refptr_.use_count();
    }

    inline const T* operator->() const
    {
        return refptr_.operator->();
    }

    inline const T* read_only() const
    {
        return refptr_.operator->();
    }

    inline T* operator->()
    {
        detach();
        return refptr_.operator->();
    }

private:

    void detach()
    {
        if (refptr_.unique())
            return;

        T* tmp = refptr_.get();

        std::unique_lock<std::mutex> lock(mutex_);
        if (not refptr_.unique())
            refptr_ = Ptr( new T( *tmp ) );
    }

private:
    Ptr refptr_;
    std::mutex mutex_;
};

} /* rtk */

