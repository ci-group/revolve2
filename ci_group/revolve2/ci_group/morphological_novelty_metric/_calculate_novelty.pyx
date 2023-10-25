#cython: language_level=3
cimport cython
import numpy as np
from numpy cimport ndarray, float64_t, uint32_t
from libc.math cimport sqrt, fabs

@cython.boundscheck(False)
@cython.wraparound(False)
@cython.nonecheck(False)
cdef float64_t move_supply(
        ndarray[float64_t, ndim=2] supply,
        ndarray[float64_t, ndim=2] capacity,
        (uint32_t, uint32_t) from_index,
        (uint32_t, uint32_t) to_index,
):
    cdef float64_t flow, distance
    if supply[from_index] <= capacity[to_index]:
        flow = supply[from_index]
        capacity[to_index] = capacity[to_index]-flow
        supply[from_index] = <float64_t>0.0
    else:
        flow = capacity[to_index]
        supply[from_index] = supply[from_index]-flow
        capacity[to_index] = <float64_t>0.0
    distance = sqrt(fabs(from_index[0]-to_index[0])**2 + fabs(from_index[1]-to_index[1])**2)
    return flow*distance

@cython.boundscheck(False)
@cython.wraparound(False)
cdef (uint32_t, uint32_t) find_first_candidate(ndarray[float64_t, ndim=2] array, uint32_t hist_shape):
    cdef uint32_t ie, je
    cdef (uint32_t, uint32_t) neg_return = (-1, -1)
    for ie in range(hist_shape):
        for je in range(hist_shape):
            if array[ie][je] > <float64_t>0.0:
                return ie, je
    return neg_return

@cython.boundscheck(False)
@cython.wraparound(False)
cdef float64_t wasserstein_distance(
        ndarray[float64_t, ndim=2] supply,
        ndarray[float64_t, ndim=2] capacity,
        uint32_t hist_shape,
):
    cdef float64_t score = 0.0
    cdef uint32_t i, j
    cdef (uint32_t, uint32_t) from_index, to_index
    cdef (uint32_t, uint32_t) neg = (-1, -1)


    for i in range(hist_shape):
        from_index = find_first_candidate(supply, hist_shape)
        for j in range(hist_shape):
            to_index = find_first_candidate(capacity, hist_shape)
            if from_index == neg or to_index  == neg:
                return score
            work = move_supply(supply, capacity, from_index, to_index)
            score += work
    return score

@cython.boundscheck(False)
@cython.wraparound(False)
cpdef ndarray[float64_t, ndim=2] calculate_novelty(ndarray[float64_t, ndim=3] histograms):
    cdef uint32_t i, j
    cdef uint32_t amount_instances = histograms.shape[0], histogram_size = histograms.shape[1]
    cdef ndarray[float64_t, ndim=1] novelty_scores = np.zeros(amount_instances, dtype=np.float64)

    for i in range(amount_instances-1):
        for j in range(i+1, amount_instances):
            score = wasserstein_distance(histograms[i].copy(), histograms[j].copy(), histogram_size)
            novelty_scores[i] += score
            novelty_scores[j] += score
    return novelty_scores