# -*- coding: utf-8 -*-
"""
Created on Thu May 13 19:30:25 2021

@author: ASUS
"""

from typing import List, Union

from vertex import Vertex


class QuokkaMaze:
    """
    Quokka Maze
    -----------

    This class is the undirected graph class that will contain all the
    information about the locations between the Quokka colony's current home
    and their final destination.

    We _will_ be performing some minor adversarial testing this time, so make
    sure you're performing checks and ensuring that the graph is a valid simple
    graph!

    ===== Func tions =====

        * block_edge(u, v) - removes the edge between vertex `u` and vertex `v`
        * fix_edge(u, v) - fixes the edge between vertex `u` and `v`. or adds an
            edge if non-existent
        * find_path(s, t, k) - find a SIMPLE path from veretx `s` to vertex `t`
            such that from any location with food along this simple path we
            reach the next location with food in at most `k` steps
        * exists_path_with_extra_food(s, t, k, x) - returns whether it’s
            possible for the quokkas to make it from s to t along a simple path
            where from any location with food we reach the next location with
            food in at most k steps, by placing food at at most x new locations

    ===== Notes ======

    * We _will_ be adversarially testing, so make sure you check your params!
    * The ordering of vertices in the `vertex.edges` does not matter.
    * You MUST check that `k>=0` and `x>=0` for the respective functions
        * find_path (k must be greater than or equal to 0)
        * exists_path_with_extra_food (k and x must be greater than or equal to
            0)
    * This is an undirected graph, so you don't need to worry about the
        direction of traversing during your path finding.
    * This is a SIMPLE GRAPH, your functions should ensure that it stays that
        way.
    * All vertices in the graph SHOULD BE UNIQUE! IT SHOULD NOT BE POSSIBLE
        TO ADD DUPLICATE VERTICES! (i.e the same vertex instance)
    """

    def __init__(self) -> None:
        """
        Initialises an empty graph with a list of empty vertices.
        """
        self.vertices = []

    def add_vertex(self, v: Vertex) -> bool:
        """
        Adds a vertex to the graph.
        Returns whether the operation was successful or not.

        :param v - The vertex to add to the graph.
        :return true if the vertex was correctly added, else false
        """
        # TODO implement me, please?
        if v is None:return False
        if v in self.vertices:return False
        self.vertices.append(v)
        return True
        

    def fix_edge(self, u: Vertex, v: Vertex) -> bool:
        """
        Fixes the edge between two vertices, u and v.
        If an edge already exists, then this operation should return False.

        :param u - A vertex
        :param v - Another vertex
        :return true if the edge was successfully fixed, else false.
        """

        # TODO implement me please.
        if v is None:return False
        if u is None:return False
        if u not in self.vertices or v not in self.vertices:return False
        if u in v.edges:return False
        if v in u.edges:return False
        u.add_edge(v)
        v.add_edge(u)
        return True

    def block_edge(self, u: Vertex, v: Vertex) -> bool:
        """
        Blocks the edge between two vertices, u and v.
        Removes the edge if it exists.
        If not, it should be unsuccessful.

        :param u - A vertex
        :param v - Another vertex.
        :return true if the edge was successfully removed, else false.
        """

        # TODO implement me, please!
        if v is None:return False
        if u is None:return False
        if u not in self.vertices or v not in self.vertices:return False
        if u not in v.edges:return False
        if v not in u.edges:return False
        u.rm_edge(v)
        v.rm_edge(u)
        return True

    def find_path(
            self,
            s: Vertex,
            t: Vertex,
            k: int
    ) -> Union[List[Vertex], None]:
        """
        find_path returns a SIMPLE path between `s` and `t` such that from any
        location with food along this path we reach the next location with food
        in at most `k` steps

        :param s - The start vertex for the quokka colony
        :param t - The destination for the quokka colony
        :param k - The maximum number of hops between locations with food, so
        that the colony can survive!
        :returns
            * The list of vertices to form the simple path from `s` to `t`
            satisfying the conditions.
            OR
            * None if no simple path exists that can satisfy the conditions, or
            is invalid.

        Example:
        (* means the vertex has food)
                    *       *
            A---B---C---D---E

            1/ find_path(s=A, t=E, k=2) -> returns: [A, B, C, D, E]

            2/ find_path(s=A, t=E, k=1) -> returns: None
            (because there isn't enough food!)

            3/ find_path(s=A, t=C, k=4) -> returns: [A, B, C]

        """

        # TODO implement me please
        path=[]
        n=k
        result=self.dfs(s,t,k,n,path)
        if len(result)==0 or result==None:
            return None
        else:
            return result

    def dfs(
        self,
        s: Vertex,
        t: Vertex,
        k: int,
        n:int,
        path=[]
    ):
        temp=n-1
        if(s.has_food): temp=temp+2
        if(temp<0): return
        n=temp
        path.append(s)
        
        if(s==t):
            return [path]
        
        paths=[]
        for node in s.edges:
            if node not in path:
                ns = self.dfs(node, t, k,n, path)
                if ns is not None:
                    for n in ns:
                        paths.append(n)
        return paths
        
        
    def exists_path_with_extra_food(
        self,
        s: Vertex,
        t: Vertex,
        k: int,
        x: int
    ) -> bool:
        """
        Determines whether it is possible for the quokkas to make it from s to
        t along a SIMPLE path where from any location with food we reach the
        next location with food in at most k steps, by placing food at at most
        x new locations.

        :param s - The start vertex for the quokka colony
        :param t - The destination for the quokka colony
        :param k - The maximum number of hops between locations with food, so
        that the colony can survive!
        :param x - The number of extra foods to add.
        :returns
            * True if with x added food we can complete the simple path
            * False otherwise.

        Example:
        (* means the vertex has food)
                            *
            A---B---C---D---E

            1/ exists_with_extra_food(A, E, 2, 0) -> returns: False
                (because we can't get from A to E with k=2 and 0 extra food)

            2/ exists_with_extra_food(A, E, 2, 1) -> returns: True
                (Yes, if we put food on `C` then we can get to E with k=2)

            3/ exists_with_extra_food(A, E, 1, 6) -> returns: True
                (Yes, if we put food on `B`, `C`, `D` then we reach E!)

        """

        # TODO implement me please
        path=[]
        result=self.dfs(s,t,k,len(self.vertices)+1,path)
        if len(result)==0:
            return False
        for i in result:
                hops = 0
                foods = x
                for j in i:
                    if j == s:
                        continue
                    if j == t:
                        return True
                    if j.has_food is False:
                        hops += 1
                    if j.has_food:
                        hops = 0
                    if hops >= k:
                        if foods <= 0:
                            break
                        else:
                            foods -= 1
                            hops = 0
        return False
        

A = Vertex(False)
B = Vertex(False)
C = Vertex(False)
D = Vertex(False)
E = Vertex(True)
'''
F = Vertex(False)
G = Vertex(False)'''
m = QuokkaMaze()
m.add_vertex(A)
m.add_vertex(B)
m.add_vertex(C)
m.add_vertex(D)
m.add_vertex(E)
m.fix_edge(A, B)
m.fix_edge(B, C)
m.fix_edge(C, D)
m.fix_edge(D, E)

print(m.find_path(A,E,4))
print(m.exists_path_with_extra_food(A, E, 2, 1))