import math
from collections import defaultdict


class shared:
    FilePath = "driving.txt"
    pathList = []
    algo = ""
    source = 0
    destination = 1


def indexesToCities():
    shared.FilePath = "cities.txt"
    cityDict = dict()
    file = open(shared.FilePath, 'r')
    for line in file:
        CityNum = int(line.partition(':')[0])
        CityName = line.partition(':')[-1][:-1]
        cityDict[CityNum] = CityName
    return cityDict


def readingTheFile():
    inputfile = open(shared.FilePath, "r")
    fileList = []

    for line in inputfile:
        distances = line.replace('\n', '').split(' ')
        fileList.append(distances)

    ########converting the list' elements into integers#########
    for i in range(len(fileList)):
        for j in range(len(fileList)):
            fileList[i][j] = int(fileList[i][j])
    # print(fileList)
    return fileList


def convertAdjMatrixToDict(m):
    graph = {}
    for idx, row in enumerate(m):
        res = []
        for r in range(len(row)):
            if row[r] != 0:
                res.append(r)
            graph[idx] = res
    return graph


###################################
vertices_names = []
graph = defaultdict(list)


def convertMatrixToWeightedDictionary():
    with open(shared.FilePath) as f:
        for line_num, line in enumerate(f):
            line = line.strip()
            if line_num == 0:
                # set cols names
                vertices_names = line.split(' ')
            else:
                for idx, each_number in enumerate(line.split(' ')):
                    if int(each_number) > 0:
                        append_to = int(vertices_names[line_num-1])
                        append_what = (
                            int(vertices_names[idx]), int(each_number))
                        graph[append_to].append(append_what)
    return dict(graph)


def readHeuristic(goal):
    inputfile = open(shared.FilePath, "r")
    for idxLine, line in enumerate(inputfile):
        if(idxLine == goal+1):
            distances = line.replace('\n', '').split(' ')
    dynamicHeuristic = {int(k): int(v) for k, v in enumerate(distances)}
    return dynamicHeuristic


graphDictionary = convertAdjMatrixToDict(readingTheFile())


def calculateTheCost(pathList):
    cost = 0

    for idx, city in enumerate(pathList):
        if idx < len(pathList)-1:
            cost += readingTheFile()[city][pathList[idx+1]]
    return cost


def printThePath(pathList):
    path = ""
    for idx, city in enumerate(pathList):
        city = indexesToCities().get(city)
        if (idx < len(pathList)-1):
            path += city + "->"
        else:
            path += city
    return path

############################################################################### BFS


def breadthFirstSearch(graph, source, goal):
    #initialize a path queue
    pathQueue = []
    #insert the first path to the queue
    pathQueue.append([source])
    while pathQueue:
        #get the first path from the queue
        path = pathQueue.pop(0)
        # get the last node from the path
        node = path[-1]
        # path found
        if node == goal:
            return path
        #count the adjacent vertices and build new path and add it to the queue
        for a in graph.get(node, []):
            new_path = list(path)
            new_path.append(a)
            pathQueue.append(new_path)
############################################################################### uniform cost (dijkstra)


def minDistance(distance, pathQueue):
	#the initial min value = inf ==> the first coming value will be taken
	leastValueFound = math.inf
	#the initial min index = -1
	leastIndex = -1

	#from the dist array pick element that has minimum value
	for i in range(len(distance)):
		if distance[i] < leastValueFound and i in pathQueue:
			leastValueFound = distance[i]
			leastIndex = i
	return leastIndex


def findPath(parent, j):
	#if source = goal
	if parent[j] == -1:
		shared.pathList.append(j)
		return
	findPath(parent, parent[j])
	shared.pathList.append(j)
	return shared.pathList


def uniformCost(graph, source, goal):  # uniform cost (dijkstra)
	row = len(graph)
	column = len(graph[0])
    #distance[i] has the shortest distance from source to i
	#the initial value of distances is inf
	distance = [math.inf] * row
	#Parent array to store the shortest path
	parent = [-1] * row
	#distance from the source to itself = 0
	distance[source] = 0

	#add all vertices in queue
	pathQueue = []
	for i in range(row):
		pathQueue.append(i)

	#find shortest path for all vertices
	while pathQueue:
		#Pick the leastValueFound dist vertex from the set of vertices
		leastIndex = minDistance(distance, pathQueue)
		#remove min element
		pathQueue.remove(leastIndex)
		#update distance value and parent index of the adjacent vertices of the chosen vertex
		for i in range(column):
                    #update the distance only if it is in queue from the two vertices and total distance is smaller than current distance
			if graph[leastIndex][i] and i in pathQueue:
				if distance[leastIndex] + graph[leastIndex][i] < distance[i]:
					distance[i] = distance[leastIndex] + graph[leastIndex][i]
					parent[i] = leastIndex
	findPath(parent, goal)


########################################a*############ ASTAR

def AStarSearch():
    cost = {shared.source: 0}
    global tree, heuristic
    closed = []
    opened = [[shared.source, readHeuristic(shared.destination)]]

    #find the visited nodes
    while True:
        fn = [i[1] for i in opened]     # fn = f(n) = g(n) + h(n)
        chosen_index = fn.index(min(fn))
        node = opened[chosen_index][0]  # current node
        closed.append(opened[chosen_index])
        del opened[chosen_index]
        # if goal was found, break the loop.
        if closed[-1][0] == shared.destination:
            break
        for item in tree[node]:
            if item[0] in [closed_item[0] for closed_item in closed]:
                continue
            # add nodes to cost dictionary
            cost.update({item[0]: cost[node] + item[1]})
            fn_node = cost[node] + heuristic[item[0]] + \
                item[1]     # calculate f(n) of current node
            temp = [item[0], fn_node]
            # store f(n) of current node in array opened
            opened.append(temp)

    #find optimal sequence
    # correct optimal tracing node, initialize as the goal node.
    trace_node = shared.destination
    # optimal node sequence
    optimal_sequence = [shared.destination]
    for i in range(len(closed)-2, -1, -1):
        check_node = closed[i][0]           # current node
        if trace_node in [children[0] for children in tree[check_node]]:
            children_costs = [temp[1] for temp in tree[check_node]]
            children_nodes = [temp[0] for temp in tree[check_node]]

            #check whether h(s) + g(s) = f(s). If so, append current node to optimal sequence
            #change the correct optimal tracing node to current node.
            if cost[check_node] + children_costs[children_nodes.index(trace_node)] == cost[trace_node]:
                optimal_sequence.append(check_node)
                trace_node = check_node
    optimal_sequence.reverse()              # reverse the optimal sequence
    return optimal_sequence


def clearPathList():
    shared.pathList.clear
##########################################


def printOnTerminal():
    text = str(
        "***********************************************************************************\n" +
        "0:Aka\t\t\t1:Bethlehem\t\t2:Dura\t\t\t3:Haifa\t\n4:Halhoul\t\t"
        "5:Hebron\t\t6:Jenin\t\t\t7:Jericho\t\n8:Jerusalem\t\t9:Nablus\t\t"
        "10:Nazareth\t\t11:Qalqilya\t\n12:Ramallah\t\t13:Ramleh\t\t14:Sabastia\t\t"
        "15:Safad\t\n16:Salfit\t\t17:Tubas\t\t18:Tulkarm\t\t19:Yafa"
        + "\n***********************************************************************************")
    return text


# print(printOnTerminal())

# shared.algo = int(input("Please Choose an Algorithm from the menu:\n1-BFS\n2-Uniform cost\n3-A* (Walking with Areal as heuristic)\n4-A* (Driving with Walking as heuristic)\n"))
# shared.source = int(input("Insert a source city number from the above list: "))
# shared.destination = int(
#     input("Insert a destination city number from the above list: "))
# if (shared.algo == 1):
#     shared.FilePath = "driving.txt"
#     shared.pathList = breadthFirstSearch(graphDictionary, shared.source, shared.destination)
#     print(shared.pathList)
# elif(shared.algo == 2):
#     shared.FilePath = "driving.txt"
#     uniformCost(readingTheFile(), shared.source, shared.destination)
#     print(shared.pathList)
# elif(shared.algo == 3):
#     shared.FilePath = "walkingWithIndexes.txt"
#     tree = convertMatrixToWeightedDictionary()
#     shared.FilePath = "areal.txt"
#     heuristic = readHeuristic(shared.destination)
#     shared.FilePath = "walking.txt"
#     shared.pathList = AStarSearch()
#     print(shared.pathList)
# elif(shared.algo == 4):
#     shared.FilePath = "heuristicWalkingWithIndexes.txt"
#     tree = convertMatrixToWeightedDictionary()
#     shared.FilePath = "heuristicWalking.txt"
#     heuristic = readHeuristic(shared.destination)
#     shared.FilePath = "driving.txt"
#     shared.pathList = AStarSearch()
#     print(shared.pathList)

# x = calculateTheCost(shared.pathList)
# print(x)
# p = printThePath(shared.pathList)
# print(p)
