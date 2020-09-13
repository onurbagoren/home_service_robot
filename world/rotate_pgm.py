import numpy
import matplotlib.pyplot as plt

def read_pgm( pgmf ):
        with open(pgmf) as f:
                lines = f.readlines()
        for line in list(lines):
                if line[0] == '#':
                        lines.remove(l)
        assert lines[0].strip() == 'P2'

        data =[]
        for line in lines[1:]:
                data.extend([int(c) for c in line.split()])
        return ( np.array(data[3:]), (data[1], data[0], data[2])
