#ifndef ITASK_H
#define ITASK_H

#include <Eigen/Dense>
#include "masterslave/commander/BoundingBox.h"

class ITask
{
    public:

    protected:
        std::unique_ptr<BoundingBox> boundingBox;
};


#endif // ITASK_H
