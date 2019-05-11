//
// Copyright (c) 2018, IRIS Laboratory, IEETA, University of Aveiro - Portugal
//
// @date    2018-10-30
// @authors Eurico Pedrosa <efp@ua.pt>
//

#pragma once

#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/median.hpp>

#include "rtk/types.h"

namespace rtk  {
namespace math {

class Stats {
public:

    Stats();
    virtual ~Stats();

    /// Add a sample to the accumulator.
    void operator()(const double& sample);

    void operator()(const DynamicArray<double>& samples);

    /// Number of samples pushed into the accumulator.
    double count() const;
    /// Minimum value of all samples.
    double min() const;
    /// Maximum value of all samples.
    double max() const;
    /// Mean of the samples.
    double mean() const;
    /// Variance of the samples.
    double var() const;
    /// Standard deviation of the samples.
    double std() const;
    /// Median of the samples.
    double median() const;
    /// The sum of all samples.
    double sum() const;

public:

    template<typename Scalar> inline
    static Scalar sum(const DynamicArray<Scalar>& y)
    {
        if (y.size() == 0)
            return Scalar(0);

        Map< const Matrix<Scalar, Dynamic, 1> > vecRef(&y[0], y.size(), 1);
        return vecRef.sum();
    }

    template<typename Scalar> inline
    static Scalar mean(const DynamicArray<Scalar>& y)
    {
        if (y.size() == 0)
            return Scalar(0);

        Map< const Matrix<Scalar, Dynamic, 1> > vecRef(&y[0], y.size(), 1);
        return vecRef.mean();
    }

    template<typename Scalar> inline
    static Scalar std(const DynamicArray<Scalar>& x)
    {
        if (x.size() == 0)
            return Scalar(0);

        Scalar mu = mean(x);
        Scalar s = 0.0;
        const size_t num_samples = x.size();
        for (size_t i = 0; i < num_samples; ++i) {
            s += (x[i] - mu)*(x[i] - mu);
        }
        return std::sqrt(s / (num_samples - 1));
    }

    template<typename Scalar> inline
    static Scalar std(const DynamicArray<Scalar>& x, const Scalar& mu)
    {
        if (x.size() == 0)
            return Scalar(0);

        Scalar s = 0.0;
        const size_t num_samples = x.size();
        for (size_t i = 0; i < num_samples; ++i) {
            s += (x[i] - mu)*(x[i]- mu);
        }

        return std::sqrt(s / (num_samples - 1));
    }

private:
    typedef boost::accumulators::accumulator_set<double,
            boost::accumulators::stats<
                boost::accumulators::tag::count,
                boost::accumulators::tag::min,
                boost::accumulators::tag::max,
                boost::accumulators::tag::mean,
                boost::accumulators::tag::variance,
                boost::accumulators::tag::median(boost::accumulators::with_p_square_quantile),
                boost::accumulators::tag::sum
            > > acc_t;

    acc_t* acc;
};

}} /* rtk::math */

