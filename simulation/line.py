import numpy as np
import math

class line3d:
    def __init__(self, a, b):
        """
        a : a point on line
        b : a point on line
        """
        self.x1, self.y1, self.z1 = b
        self.l, self.m, self.n = a[0]-b[0], a[1]-b[1], a[2]-b[2]
    
    def perp_dist(self, p):
        """
        p : list of 3 integers
        """
        a, b, c = self.x1 - p[0], self.y1 - p[1], self.z1 - p[2]
        l, m, n = self.l, self.m, self.n
        
        temp = list(np.cross([a,b,c],[l,m,n]))

        temp = math.sqrt((temp[0] * temp[0]) + (temp[1] * temp[1]) + (temp[2] * temp[2]))
        temp = temp / (math.sqrt((l*l)+(m*m)+(n*n)))
        return abs(temp)

    def __str__(self):
        

        return 


# l = line3d([2, 1, 2], [3, 1, -1])

# print(l.perp_dist([0, 2, 3]))