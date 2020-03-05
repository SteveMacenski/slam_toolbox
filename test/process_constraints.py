import sys
import matplotlib.pyplot as plt
import random

'''
Purpose: This is a simple graphing tool to find the relationship between number of nodes
     and number of constraints over time.

To use: add the following code to publishGraph and disable the optimizer printouts

  std::map<karto::Name, std::vector<karto::Vertex<karto::LocalizedRangeScan>*> > vertices
    = mapper_->GetGraph()->GetVertices();
  std::vector<karto::Vertex<karto::LocalizedRangeScan>*>::const_iterator it;
  std::map<int, int> vertex_ctr;
  for (it = vertices[mapper_->GetMapperSensorManager()->GetSensorNames()[0]].begin();
    it != vertices[mapper_->GetMapperSensorManager()->GetSensorNames()[0]].end(); ++it)
  {
    int num = (*it)->GetEdges().size();
    if (vertex_ctr.find(num) == vertex_ctr.end())
    {
      vertex_ctr[num] = 1;
    }
    vertex_ctr[num]++;
  }

  std::cout << "UpdateMap: Vertex count: " << std::endl;
  std::map<int, int>::const_iterator it2;
  for (it2 = vertex_ctr.begin(); it2 != vertex_ctr.end(); ++it2)
  {
    std::cout << it2->first << " constraints are in "
      << it2->second << " vertexes" << std::endl;
  }
  std::cout << std::endl;
'''


def readFileToList(filename):
    with open(filename) as fp:
        line = fp.readline()
        lines = []
        lines.append(line)
        while line:
            line = fp.readline()
            lines.append(line)
    return lines


def getSingleSets(lines):
    measurements = []
    measurement = []
    for line in lines:
        if line == "\n":
            continue
        if "UpdateMap: Vertex count:" in line:
            measurements.append(measurement)
            measurement = []
        measurement.append(line)
    return measurements[1:]


def processForData(measurements):
    measurements_out = []
    measurement_out = []
    for measurement in measurements:
        for m in measurement:
            txt = m.split(" ")
            nums = [r for r in txt if r.isdigit()]
            if not nums:
                continue
            measurement_out.append(nums)
        measurements_out.append(measurement_out)
        measurement_out = []
    return measurements_out


def plotData(data):
    # plot lines

    # give us the number of lines to create and unque items
    keys = []
    for d in data:
        for m in d:
            if m[0] not in keys:
                keys.append(m[0])

    dat = {}
    for k in keys:
        dat[k] = []

    for d in data:
        local_keys = []
        for pt in d:
            local_keys.append(pt[0])
            dat[pt[0]].append(int(pt[1]))
        for k in keys:
            if k not in local_keys:
                dat[k].append(0)

    total_nodes = []
    for d in data:
        summ = 0
        for pt in d:
            summ = summ + int(pt[1])
        total_nodes.append(summ)

    for k in keys:
        plt.plot([10*i for i in range(0, len(dat[k]))], dat[k],
                 marker='o', color=[random.random(), random.random(), random.random()],
                 linewidth=2, label=k+' Constraints')

        plt.plot([10*i for i in range(0, len(dat[k]))],
                 total_nodes, marker='o', color=[random.random(),
                 random.random(), random.random()], linewidth=2, label='Total Num. Nodes')
        plt.legend()
        plt.xlabel("time (s)")
        plt.ylabel("Node count")
        # plt.yscale("log")
        plt.show()


if __name__ == "__main__":
    filename = sys.argv[1]
    print("reading file: " + filename)
    listOfContents = readFileToList(filename)
    listOfListOfMeasurements = getSingleSets(listOfContents)
    data = processForData(listOfListOfMeasurements)
    plotData(data)
