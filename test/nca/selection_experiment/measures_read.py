import csv
from typing import List

import numpy as np

def get_robot_measures(measures=None):

  measures_file = open("all_measures.tsv")
  read_tsv = csv.reader(measures_file, delimiter='\t')

  robot_names = []
  column_indexes = []
  elements: List[List] = []
  first = True
  for row in read_tsv:
    if not first:
      robot_names.append(row[0])
      elements.append(np.array(row)[column_indexes])
    else:
      for index, element in enumerate(row):
        if element in measures:
          column_indexes.append(index)
      first = False

  measures_file.close()

  array = np.array(elements, dtype=float)

  row_sums = array.sum(axis=0)
  normalized_measures = array / row_sums

  return normalized_measures, robot_names


if __name__ == "__main__":
  measures, names = get_robot_measures()
  print(names)
  print(measures)