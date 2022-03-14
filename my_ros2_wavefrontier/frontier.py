from enum import Enum
import math

import utils

OCC_THRESHOLD = 10
MIN_FRONTIER_SIZE = 20
DISTANCE_THRESHOLD = 0.25  # m


class PointClassification(Enum):
    MapOpen = 1        # 0001
    MapClosed = 2      # 0010
    FrontierOpen = 4   # 0100
    FrontierClosed = 8 # 1000


class OccupancyGrid2d():
    class CostValues(Enum):
        FreeSpace = 0
        InscribedInflated = 100
        LethalObstacle = 100
        NoInformation = -1

    def __init__(self, map):
        self.map = map

    def getCost(self, mx, my):
        return self.map.data[self._getIndex(mx, my)]

    def getSize(self):
        return (self.map.info.width, self.map.info.height)

    def getSizeX(self):
        return self.map.info.width

    def getSizeY(self):
        return self.map.info.height

    def mapToWorld(self, mx, my):
        wx = self.map.info.origin.position.x + (mx + 0.5) * self.map.info.resolution
        wy = self.map.info.origin.position.y + (my + 0.5) * self.map.info.resolution

        return (wx, wy)

    def worldToMap(self, wx, wy):
        if (wx < self.map.info.origin.position.x or wy < self.map.info.origin.position.y):
            raise Exception("World coordinates out of bounds")

        mx = int((wx - self.map.info.origin.position.x) / self.map.info.resolution)
        my = int((wy - self.map.info.origin.position.y) / self.map.info.resolution)

        if  (my > self.map.info.height or mx > self.map.info.width):
            raise Exception("Out of bounds")

        return (mx, my)

    def _getIndex(self, mx, my):
        return my * self.map.info.width + mx


class FrontierPoint():
    def __init__(self, x, y):
        self.classification = 0
        self.mapX = x
        self.mapY = y


class FrontierCache():
    def __init__(self):
        self.cache = dict()

    def getPoint(self, x, y):
        idx = self._cantorHash(x, y)
        if idx in self.cache:
            return self.cache[idx]
        self.cache[idx] = FrontierPoint(x, y)
        return self.cache[idx]

    def _cantorHash(self, x, y):
        return (((x + y) * (x + y + 1)) / 2) + y

    def clear(self):
        self.cache = {}


def findFree(mx, my, costmap):
    fCache = FrontierCache()

    bfs = [fCache.getPoint(mx, my)]

    while len(bfs) > 0:
        loc = bfs.pop(0)

        if costmap.getCost(loc.mapX, loc.mapY) == OccupancyGrid2d.CostValues.FreeSpace.value:
            return (loc.mapX, loc.mapY)

        for n in getNeighbors(loc, costmap, fCache):
            if n.classification & PointClassification.MapClosed.value == 0:
                n.classification = n.classification | PointClassification.MapClosed.value
                bfs.append(n)

    return (mx, my)

def getFrontier(master, is_pub_queue):
    pose = master.currentPose
    costmap = master.costmap

    fCache = FrontierCache()
    fCache.clear()

    mx, my = costmap.worldToMap(pose.position.x, pose.position.y)

    freePoint = findFree(mx, my, costmap)
    start = fCache.getPoint(freePoint[0], freePoint[1])
    start.classification = PointClassification.MapOpen.value
    mapPointQueue = [start]

    frontiers = list()

    while len(mapPointQueue) > 0:
        p = mapPointQueue.pop(0)

        if p.classification & PointClassification.MapClosed.value != 0:
            # -- True when p is MapClosed
            continue

        if isFrontierPoint(p, costmap, fCache, pose, True):
            p.classification = PointClassification.FrontierOpen.value
            frontierQueue = [p]
            newFrontier = list()

            while len(frontierQueue) > 0:
                q = frontierQueue.pop(0)

                if q.classification & (PointClassification.MapClosed.value | PointClassification.FrontierClosed.value) != 0:
                    # -- True when q is MapClosed or FrontierClosed
                    continue

                if isFrontierPoint(q, costmap, fCache, pose, False):
                    newFrontier.append(q)

                    for w in getNeighbors(q, costmap, fCache):
                        if w.classification & (PointClassification.FrontierOpen.value | PointClassification.FrontierClosed.value | PointClassification.MapClosed.value) == 0:
                            # -- True when w is neither FrontierOpen, FrontierClosed nor MapClosed
                            w.classification = PointClassification.FrontierOpen.value
                            frontierQueue.append(w)

                q.classification = PointClassification.FrontierClosed.value

                if is_pub_queue:
                    msg = utils.Rviz.get_msg_markers_queue_frontier(frontierQueue, costmap)
                    master.pub_markers_queue_frontier.publish(msg)

            newFrontierCords = list()
            for x in newFrontier:
                x.classification = PointClassification.MapClosed.value
                newFrontierCords.append(costmap.mapToWorld(x.mapX, x.mapY))

            if len(newFrontier) > MIN_FRONTIER_SIZE:
                frontiers.append(newFrontierCords)

        for v in getNeighbors(p, costmap, fCache):
            if v.classification & (PointClassification.MapOpen.value | PointClassification.MapClosed.value) == 0:
                # -- True when v is neither MapOpen nor MapClosed
                if any(costmap.getCost(x.mapX, x.mapY) == OccupancyGrid2d.CostValues.FreeSpace.value for x in getNeighbors(v, costmap, fCache)):
                    # -- True when anyone of neighbors is free
                    v.classification = PointClassification.MapOpen.value
                    mapPointQueue.append(v)

        p.classification = PointClassification.MapClosed.value

        if is_pub_queue:
            msg = utils.Rviz.get_msg_markers_queue_map(mapPointQueue, costmap)
            master.pub_markers_queue_map.publish(msg)

    return frontiers

def getNeighbors(point, costmap, fCache):
    neighbors = list()

    for x in range(point.mapX - 1, point.mapX + 2):
        for y in range(point.mapY - 1, point.mapY + 2):
            if (x > 0 and x < costmap.getSizeX() and y > 0 and y < costmap.getSizeY()):
                neighbors.append(fCache.getPoint(x, y))

    return neighbors

def isFrontierPoint(point, costmap, fCache, pose, check_dist):
    # -- check distance to robot, for sometimes the point is too near to robot
    if check_dist:
        wx, wy = costmap.mapToWorld(point.mapX, point.mapY)
        dist = math.sqrt(((wx - pose.position.x)**2) + ((wy - pose.position.y)**2))
        if dist < DISTANCE_THRESHOLD:
            return False

    # -- check the cost is no-information
    if costmap.getCost(point.mapX, point.mapY) != OccupancyGrid2d.CostValues.NoInformation.value:
        return False

    # -- check anyone of neighbors is free
    hasFree = False
    for n in getNeighbors(point, costmap, fCache):
        cost = costmap.getCost(n.mapX, n.mapY)

        if cost > OCC_THRESHOLD:  # once the neighbor is an obstacle
            return False

        if cost == OccupancyGrid2d.CostValues.FreeSpace.value:
            hasFree = True

    return hasFree
