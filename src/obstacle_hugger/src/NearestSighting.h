//
// Created by stevebillings on 12/4/22.
//

#ifndef OBSTACLE_HUGGER_NEARESTSIGHTING_H
#define OBSTACLE_HUGGER_NEARESTSIGHTING_H


class NearestSighting {
public:
    NearestSighting(const unsigned long range_index, double range)
    : range_index_(range_index)
    , range_(range) {};

    unsigned long get_range_index() const;
    double get_range() const;
private:
    const unsigned long range_index_;
    const double range_;

};


#endif //OBSTACLE_HUGGER_NEARESTSIGHTING_H
