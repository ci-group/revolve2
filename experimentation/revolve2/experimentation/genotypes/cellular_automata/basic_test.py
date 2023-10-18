
import numpy as np
from develop import develop
from ca_genotype import CAGenotype
from revolve2.modular_robot import MorphologicalMeasures


def main():
    dsize = 7
    domain = np.zeros((dsize, dsize))
    domain[dsize // 2, dsize // 2] = 1

    g = CAGenotype()
    g.set_params(domain, 10)
    g.generate_body()
    g.set_core(dsize // 2, dsize // 2)

    print('genotype: \n', g.get_grid())
    
    body = develop(g)

    body_measures = MorphologicalMeasures(body=body)
    grid_arr = np.zeros_like(g.ca_grid)
    grid = body_measures.body_as_grid

    for x in range(body_measures.bounding_box_depth):
        for y in range(body_measures.bounding_box_width):
            if grid[x][y][body_measures.core_grid_position[2]] is not None:
                grid_arr[x][y] = 1

    print('body.find_bricks(): \n', body.find_bricks())

    print('body as grid: \n', body_measures.body_as_grid)

    print(grid_arr)




if __name__ == '__main__':
    main()
