
#ifndef OBSTACLE_HUGGER_LASERANALYSIS_H
#define OBSTACLE_HUGGER_LASERANALYSIS_H

class LaserAnalysis {

private:
    const unsigned long min_range_index_;
    double min_range_;
    bool in_sight_;
    bool near_;
    bool too_near_;
    unsigned long leftmost_index_;
    unsigned long straight_index_;
    bool to_right_;
    unsigned long delta_from_perpendicular_right_;
    unsigned long delta_from_perpendicular_left_;

public:
    LaserAnalysis(const unsigned long min_range_index, const double min_range, const bool in_sight, const bool near, const bool too_near,
                                 const unsigned long leftmost_index,
                                 const unsigned long straight_index,
                                 const bool to_right,
                                 const unsigned long delta_from_perpendicular_right,
                                 const unsigned long delta_from_perpendicular_left) : min_range_index_(min_range_index)
            , min_range_(min_range)
            , in_sight_(in_sight)
            , near_(near)
            , too_near_(too_near)
            , leftmost_index_(leftmost_index)
            , straight_index_(straight_index)
            , to_right_(to_right)
            , delta_from_perpendicular_right_(delta_from_perpendicular_right)
            , delta_from_perpendicular_left_(delta_from_perpendicular_left) {};
    unsigned long get_min_range_index() const;
    double get_min_range() const;
    bool is_in_sight() const;
    bool is_near() const;
    bool is_too_near() const;
    unsigned long get_leftmost_index() const;
    unsigned long get_straight_index() const;
    bool is_to_right() const;
    unsigned long get_delta_from_perpendicular_right() const;
    unsigned long get_delta_from_perpendicular_left() const;
};

#endif //OBSTACLE_HUGGER_LASERANALYSIS_H
