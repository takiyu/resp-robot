#ifndef MERGER_H_161027
#define MERGER_H_161027

#include "containers.h"

void MergePriorityMovements(const std::vector<MovementCtr>& b_ctrs,
                            MovementCtr& result_ctr);

void MergeWeightedMovements(const std::vector<MovementCtr>& b_ctrs,
                            MovementCtr& result_ctr);

#endif
