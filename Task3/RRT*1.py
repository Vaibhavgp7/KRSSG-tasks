import cv2
import numpy as np
from PIL import Image
import math
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import imutils
import random
class RRT:
    """
    Class for RRT planning
    """

    class Node:
        """
        RRT Node
        """

        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.path_x = []
            self.path_y = []
            self.cost = 0.0
            self.parent = None

    def __init__(self,
                 start,
                 goal,
                 cnts,
                 rand_area,
                 expand_dis=200.0,
                 path_resolution=0.5,
                 circle_radius=300.0,
                 search_until_max_iter=False):
        
        self.start = self.Node(start[0], start[1])
        self.end = self.Node(goal[0], goal[1])
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.circle_radius = circle_radius
        self.cnts = cnts
        self.node_list = []
        self.search_until_max_iter = search_until_max_iter

    def planning(self):
        
        self.node_list1 = [self.start]
        self.node_list2 = [self.end]
        for i in range(500):
            
            rnd = self.get_random_node(self.cnts)
            rec = self.get_nearest_node_index(self.node_list1,self.node_list2, rnd)
            if rec[0] == 1:
                near_node = self.node_list1[rec[1]]
                new_node = self.steer(self.node_list1[rec[1]], rnd,self.expand_dis)
                check = [1,new_node,near_node]
            else:
                new_node = self.steer(self.node_list2[rec[1]], rnd,self.expand_dis)
                near_node = self.node_list2[rec[1]]
                check = [2,new_node,near_node]
            new_node.cost = near_node.cost +  math.hypot(new_node.x-near_node.x, new_node.y-near_node.y)
            if self.check_collision(new_node, self.cnts):
                near_inds = self.find_near_nodes(new_node,check)
                node_with_updated_parent = self.parent(new_node, near_inds,check)
                if node_with_updated_parent:
                    self.rewire(node_with_updated_parent, near_inds,check)
                    if check[0]==1:
                        self.node_list1.append(node_with_updated_parent)
                    else:
                        self.node_list2.append(node_with_updated_parent)
                else:
                    if check[0]==1:
                        self.node_list1.append(new_node)
                    else:
                        self.node_list2.append(new_node)
        for s in self.node_list1:
            for g in self.node_list2:
                con = self.connect(s,g)
                if self.check_collision(con,self.cnts):
                    return self.generate_final_course(s,g,self.node_list1.index(s),self.node_list2.index(g))
        return None            

    def parent(self, new_node, near_inds,check):
        if not near_inds:
            return None

        # search nearest cost in near_inds
        costs = []
        for i in near_inds:
            if check[0]==1:
                near_node = self.node_list1[i]
                t_node = self.steer(near_node, new_node)
            else:
                near_node = self.node_list2[i]
                t_node = self.steer(near_node, new_node)
                
            if t_node and self.check_collision(t_node, self.cnts):
                costs.append(self.calc_new_cost(near_node, new_node))
            else:
                costs.append(float("inf"))  # the cost of collision node
        min_cost = min(costs)


        if min_cost == float("inf"):
            print("There is no good path.(min_cost is inf)")
            return None

        min_ind = near_inds[costs.index(min_cost)]
        if check[0] == 1:
            new_node = self.steer(self.node_list1[min_ind], new_node)
        else:
            new_node = self.steer(self.node_list2[min_ind], new_node)
        new_node.cost = min_cost
        return new_node
    
    def steer(self, from_node, to_node, extend_length=float("inf")):

        new_node = self.Node(from_node.x, from_node.y)
        d, theta = self.calc_distance_and_angle(new_node, to_node)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]

        if extend_length > d:
            extend_length = d

        n_expand = math.floor(extend_length / self.path_resolution)

        for _ in range(n_expand):
            new_node.x += self.path_resolution * math.cos(theta)
            new_node.y += self.path_resolution * math.sin(theta)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        d, _ = self.calc_distance_and_angle(new_node, to_node)
        if d <= self.path_resolution:
            new_node.path_x.append(to_node.x)
            new_node.path_y.append(to_node.y)
            new_node.x = to_node.x
            new_node.y = to_node.y

        new_node.parent = from_node
        return new_node

    def connect(self,from_node,to_node):
        new_node = self.Node(from_node.x, from_node.y)
        d, theta = self.calc_distance_and_angle(new_node, to_node)

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]

        n_expand = math.floor(d / self.path_resolution)

        for _ in range(n_expand):
            new_node.x += self.path_resolution * math.cos(theta)
            new_node.y += self.path_resolution * math.sin(theta)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)
        new_node.path_x.append(to_node.x)
        new_node.path_y.append(to_node.y)
        new_node.x = to_node.x
        new_node.y = to_node.y
        return new_node

    def generate_final_course(self,s,g,goal_ind1,goal_ind2):
        path = []
        node = self.node_list1[goal_ind1]
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([self.start.x,self.start.y])
        path = [node for node in reversed(path)]
        node = self.node_list2[goal_ind2]
        while node.parent is not None:
            path.append([node.x,node.y])
            node = node.parent
        path.append([self.end.x,self.end.y])  
        

        return path

    def calc_dist_to_goal(self, x, y):
        dx = x - self.end.x
        dy = y - self.end.y
        return math.hypot(dx, dy)

    def get_random_node(self,cnts):
        rnd = self.Node(random.randint(self.min_rand, self.max_rand),random.randint(self.min_rand, self.max_rand))
        print(rnd)
        lst = []
        for c in cnts:
            result = cv2.pointPolygonTest(c, (rnd.x,rnd.y), False)
            lst.append(result)
        if lst.count(-1)==len(cnts):
            return rnd
        else:
            return self.get_random_node(cnts)
            
        
    def get_nearest_node_index(self,node_list1,node_list2,rnd):
        for node in node_list1:
            dlist1 = []
            dlist1.append((node.x - rnd.x)**2 + (node.y - rnd.y)**2)
        minind1 = dlist1.index(min(dlist1))
        for node in node_list2:
            dlist2 = []
            dlist2.append((node.x - rnd.x)**2 + (node.y - rnd.y)**2)
        minind2 = dlist2.index(min(dlist2))
        if min(dlist1) >= min(dlist2):
            return (1,minind1)
        else:
            return (2,minind2)
    

    def check_collision(self,node,cnts):

        if node is None:
            return False
        merge = tuple(zip(node.path_x,node.path_y))
        for c in cnts:
            result = []
            for (x,y) in merge:
                result.append(cv2.pointPolygonTest(c, (x,y), False))    
            
            if result.count(0)>0 or result.count(1)>0:
                return False
        return True 

    def calc_distance_and_angle(self,from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta
    def find_near_nodes(self, new_node,check):
        if check[0] == 1:
            dist_list = [(node.x - new_node.x)**2 + (node.y - new_node.y)**2
                     for node in self.node_list1]
        else:
            dist_list = [(node.x - new_node.x)**2 + (node.y - new_node.y)**2
                     for node in self.node_list2]
        r = self.circle_radius 
        
        near_inds = [dist_list.index(i) for i in dist_list if i <= r**2]
        return near_inds

    def rewire(self, new_node, near_inds,check):
        for i in near_inds:
            if check[0]==1:
                near_node = self.node_list1[i]
                edge_node = self.steer(new_node, near_node)
            else:
                near_node = self.node_list2[i]
                edge_node = self.steer(new_node, near_node)
            if not edge_node:
                continue
            edge_node.cost = self.calc_new_cost(new_node, near_node)

            no_collision = self.check_collision(edge_node, self.cnts)
            improved_cost = near_node.cost > edge_node.cost

            if no_collision and improved_cost:
                near_node.x = edge_node.x
                near_node.y = edge_node.y
                near_node.cost = edge_node.cost
                near_node.path_x = edge_node.path_x
                near_node.path_y = edge_node.path_y
                near_node.parent = edge_node.parent
                self.propagate_cost_to_leaves(new_node,check[0])

    def calc_new_cost(self, from_node, to_node):
        d, _ = self.calc_distance_and_angle(from_node, to_node)
        return from_node.cost + d

    def propagate_cost_to_leaves(self, parent_node,c):
        if c==1:
            for node in self.node_list1:
                if node.parent == parent_node:
                    node.cost = self.calc_new_cost(parent_node, node)
                    self.propagate_cost_to_leaves(node)
        else:
            for node in self.node_list2:
                if node.parent == parent_node:
                    node.cost = self.calc_new_cost(parent_node, node)
                    self.propagate_cost_to_leaves(node)
        


def main():

    image = cv2.imread('k1.png')
    
    h,w = image.shape[:2]
    print("Height = {},  Width = {}".format(h, w))

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]
    cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    M = cv2.moments(cnts[-1])
    sX = int(M["m10"] / M["m00"])
    sY = int(M["m01"] / M["m00"])
    M = cv2.moments(cnts[0])
    gX = int(M["m10"] / M["m00"])
    gY = int(M["m01"] / M["m00"])
    cnts.pop(0)
    cnts.pop(-1)
    start = [sX,sY]
    goal = [gX,gY]
    # ====Search Path with RRT====
    
    rrt = RRT(start,goal,rand_area=[0,600],cnts = cnts)
    path = rrt.planning()
    print(path)
    
    """for i in range(len(path)):
        if path[i] == path[-1]:
            break
        
        cv2.line(image,(path[i][0],path[i][1]),(path[i+1][0],path[i+1][1]),(0,255,255),1)
    cv2.imshow('join',image)
    cv2.waitKey(0)"""
    plt.plot([x for (x, y) in path], [y for (x, y) in path], 'r--')
    plt.imshow(image)
    plt.show()
    
    

    
if __name__ == '__main__':
    main()
