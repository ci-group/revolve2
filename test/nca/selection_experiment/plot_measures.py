#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jul  6 11:06:57 2020

@author: fuda
"""

import matplotlib.pyplot as plt
import math
import os
import pandas as pd
import itertools


# %%
def plot_measures(measures_compare, selected_robots):
    data_path = ''
    df = pd.read_csv(os.path.join(data_path, 'all_measures.tsv'), sep='\t')
    chosen_robots = ['bulkyA', 'zappa', 'park', 'babyB', 'penguin', 'snake', 'garrix', 'babyA', 'bohmer', 'salamander',
                     'spider', 'blokky', 'queen', 'frank', 'turtle', 'gecko', 'linkin', 'pentapede', 'insect', 'martin']
    colors = ['white', 'white', 'pink', 'white', 'white', 'red', 'black', 'white', 'white', 'orange', 'green',
                   'blue', 'white', 'white', 'white', 'purple', 'white', 'white', 'white', 'white',]

    font = {'family': 'DejaVu Sans',
            'weight': 'normal',
            'size': 13}
    plt.rc('font', **font)
    #    measures_compare = ['joints', 'proportion', 'hinge_count', 'size']

    measures_compare_list = list(itertools.combinations(measures_compare, 2))
    cols = 6
    rows = math.ceil(len(measures_compare_list) / cols)
    plt.figure(figsize=((cols + 1) * 3, rows * 3))
    for ind, measures in enumerate(measures_compare_list):

        plt.subplot(rows, cols, ind + 1)
        x = df[measures[1]]
        y = df[measures[0]]

        plt.scatter(x, y, c=colors, s=125)

        plt.xlabel(measures[1], fontweight='bold')
        plt.ylabel(measures[0], fontweight='bold')
        plt.grid()

    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    plot_measures(['joints', 'proportion', 'hinge_count', 'size', 'branching', 'limbs', 'coverage', 'symmetry'], )