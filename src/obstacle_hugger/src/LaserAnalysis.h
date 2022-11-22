
#ifndef OBSTACLE_HUGGER_LASERANALYSIS_H
#define OBSTACLE_HUGGER_LASERANALYSIS_H

class LaserAnalysis {

private:
    unsigned long min_range_index_;
    double min_range_;
    bool in_sight_;
    bool near_;
    unsigned long leftmost_index_;
    unsigned long straight_index_;
    bool to_right_;
    unsigned long delta_from_perpendicular_right_;
    unsigned long delta_from_perpendicular_left_;

public:
    LaserAnalysis(unsigned long min_range_index, double min_range, bool in_sight, bool near, unsigned long leftmost_index,
                  unsigned long straight_index,
                  bool to_right, unsigned long delta_from_perpendicular_right, unsigned long delta_from_perpendicular_left);
    unsigned long get_min_range_index();
    double get_min_range();
    bool is_in_sight();
    bool is_near();
    unsigned long get_leftmost_index();
    unsigned long get_straight_index();
    bool is_to_right();
    unsigned long get_delta_from_perpendicular_right();
    unsigned long get_delta_from_perpendicular_left();
};

#endif //OBSTACLE_HUGGER_LASERANALYSIS_H
