#cython: language_level=3
cimport cython
import numpy as np
from libc.math cimport sqrt
from numpy cimport ndarray


@cython.boundscheck(False)
@cython.wraparound(False)
@cython.nonecheck(False)
cdef double move_supply(
        long[:,:] supply,
        long[:,:] capacity,
        (int, int) from_index,
        (int, int) to_index,
):
    cdef double distance
    cdef long flow
    cdef int zero_state = 0
    if supply[from_index[0], from_index[1]] <= capacity[to_index[0], to_index[1]]:
        flow = supply[from_index[0], from_index[1]]
        capacity[to_index[0], to_index[1]] = capacity[to_index[0], to_index[1]]-flow
        supply[from_index[0], from_index[1]] = zero_state
    else:
        flow = capacity[to_index[0], to_index[1]]
        supply[from_index[0], from_index[1]] = supply[from_index[0], from_index[1]]-flow
        capacity[to_index[0], to_index[1]] = zero_state
    distance = sqrt((from_index[0]-to_index[0])**2 + (from_index[1]-to_index[1])**2)
    return flow*distance

@cython.boundscheck(False)
@cython.wraparound(False)
cdef (int, int) find_first_candidate(long[:,:] array, int hist_shape, (int, int) prev):
    cdef int ie, je
    cdef (int, int) neg_return = (-1, -1)
    cdef int zero_state = 0
    for ie in range(prev[0], hist_shape):
        for je in range(prev[1], hist_shape):
            if array[ie, je] > zero_state:
                return ie, je
    return neg_return

@cython.boundscheck(False)
@cython.wraparound(False)
cdef double wasserstein_distance(
        long[:,:] supply,
        long[:,:] capacity,
        int hist_shape,
):
    cdef double score = 0.0
    cdef int i, j
    cdef (int, int) from_index = (0, 0), to_index = (0, 0)
    cdef int zero_state = 0

    for i in range(hist_shape):
        from_index = find_first_candidate(supply, hist_shape, from_index)
        for j in range(hist_shape):
            to_index = find_first_candidate(capacity, hist_shape, to_index)
            if from_index[0] < zero_state or to_index[0] < zero_state:
                return score
            work = move_supply(supply, capacity, from_index, to_index)
            score += work
    return score

@cython.boundscheck(False)
@cython.wraparound(False)
cpdef ndarray[double, ndim=1] calculate_novelty(ndarray[long, ndim=3, mode="c"] histograms, int amount_instances, int histogram_size):
    cdef ndarray[double, ndim=1, mode="c"] novelty_scores = np.zeros(shape=amount_instances)
    cdef int i, j
    cdef long[:,:] supply, capacity
    cdef ndarray[long, ndim=3, mode="c"] tmp_hist

    for i in range(amount_instances-1):
        for j in range(i+1, amount_instances):
            tmp_hist = histograms.copy()
            supply = tmp_hist[i]
            capacity = tmp_hist[j]
            score = wasserstein_distance(supply, capacity , histogram_size)
            novelty_scores[i] += score
            novelty_scores[j] += score
    return novelty_scores
