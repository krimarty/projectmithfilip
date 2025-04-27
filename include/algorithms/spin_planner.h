//
// Created by martin on 25.04.25.
//

#ifndef SPIN_PLANNER_H
#define SPIN_PLANNER_H
#include "nodes/lidar_node.h"

namespace algorithms
{
    /*
    Escape path:              Treasure Path:
    ID = 0 -> straight        ID = 10 -> straight
    ID = 1 -> left            ID = 11 -> left
    ID = 2 -> right           ID = 12 -> right
    */

    enum spin
    {
        around,
        left,
        right,
        straight,
        unknown
    };

    class SpinPlanner {
    public:
        SpinPlanner(): nextEscapeSpin(), nextTreasureSpin(), prevSpin()
        {
        };
        ~SpinPlanner()= default;

        void set_tag(const int tag)
        {
            switch (tag)
            {
                case 0:
                    nextEscapeSpin = straight;
                    break;

                case 1:
                    nextEscapeSpin = left;
                    break;

                case 2:
                    nextEscapeSpin = right;
                    break;

                case 10:
                    nextTreasureSpin = straight;
                    break;

                case 11:
                    nextTreasureSpin = left;
                    break;

                case 12:
                    nextTreasureSpin = right;
                    break;
                default:;
            }
        }

        spin get_spin(nodes::freeCorridor corridor_scan)
        {
            if (nextTreasureSpin == unknown)
            {
                if (nextEscapeSpin == unknown)
                {
                    if (corridor_scan.left)
                        return left;
                    else if (corridor_scan.front)
                        return straight;
                    else
                        return right;
                }
                prevSpin = nextEscapeSpin;
                nextEscapeSpin = unknown;
                return prevSpin;
            }
            set_escape_fromTreasure(corridor_scan);
            prevSpin = nextTreasureSpin;
            nextTreasureSpin = unknown;
            return prevSpin;
        }


    private:
        spin nextEscapeSpin;
        spin nextTreasureSpin;
        spin prevSpin;

        void set_escape_fromTreasure(nodes::freeCorridor corridor_scan)
        {
            if (nextEscapeSpin == unknown)
            {
                if (nextTreasureSpin == left)
                {
                    if (corridor_scan.front) {nextEscapeSpin = left; return;}
                    if (corridor_scan.right) {nextEscapeSpin = straight; return;}
                }
                else if (nextTreasureSpin == straight)
                {
                    if (corridor_scan.left) {nextEscapeSpin = right; return;}
                    if (corridor_scan.right) {nextEscapeSpin = left; return;}
                }
                else if (nextTreasureSpin == right)
                {
                    if (corridor_scan.left) {nextEscapeSpin = straight; return;}
                    if (corridor_scan.front) {nextEscapeSpin = right; return;}
                }
            }
            if (nextTreasureSpin == left)
            {
                if (nextEscapeSpin == right) {nextEscapeSpin = straight; return;}
                if (nextEscapeSpin == straight) {nextEscapeSpin = left; return;}
            }
            else if (nextTreasureSpin == straight)
            {
                if (nextEscapeSpin == right) {nextEscapeSpin = left; return;}
                if (nextEscapeSpin == left) {nextEscapeSpin = right; return;}
            }
            else if (nextTreasureSpin == right)
            {
                if (nextEscapeSpin == left) {nextEscapeSpin = straight; return;}
                if (nextEscapeSpin == straight) {nextEscapeSpin = right; return;}
            }
        }

    };
}

#endif //SPIN_PLANNER_H
