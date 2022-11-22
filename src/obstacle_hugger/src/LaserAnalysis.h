
#ifndef OBSTACLE_HUGGER_LASERANALYSIS_H
#define OBSTACLE_HUGGER_LASERANALYSIS_H

class LaserAnalysis {

private:
    unsigned long min_range_index_;
    double min_range_;
    bool in_sight_;
    bool near_;
    unsigned long leftmost_index_;

public:
    LaserAnalysis(unsigned long min_range_index, double min_range, bool in_sight, bool near, unsigned long leftmost_index);
    unsigned long get_min_range_index();
    double get_min_range();
    bool is_in_sight();
    bool is_near();
    unsigned long get_leftmost_index();
};

#endif //OBSTACLE_HUGGER_LASERANALYSIS_H
