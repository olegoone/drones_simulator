class Grid:
  
  area = [(0,0),(0,10),(10,0),(10,10)]
  k = 4
  h = 20
  d = 1

  def __init__(self, k, h, d):
    self.k = 4
    self.h = 20
    self.d = 2


  def next_cell(self, current_cell):
    return (current_cell[0]+1, current_cell[1])
