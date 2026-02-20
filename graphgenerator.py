import networkx as nx
import random
from typing import Optional

class Location:
    def __init__(self, name: str):
        self.name = name

class Graph:
    def __init__(self, num_nodes: int, extra_edges: Optional[int] = None,
                 traversable_prob: float = 0.5, seed: Optional[int] = None):
        self.locations = [Location(f"l_{i:02d}") for i in range(1, num_nodes + 1)]
        self.start = Location("l_start")
        self.end = Location("l_end")
        self.graph = nx.Graph()
        self.traversable_edges = set()
        self._build_icc_graph(num_nodes, extra_edges, traversable_prob, seed)

    def _build_icc_graph(self, num_nodes: int, extra_edges: Optional[int],
                        traversable_prob: float, seed: Optional[int]):
        if seed is not None: random.seed(seed)
            
        nodes = [self.start] + self.locations + [self.end]
        for loc in nodes:
            self.graph.add_node(loc.name)

        all_names = [loc.name for loc in nodes]
        # Eliminamos el shuffle para mantener una progresión lógica Start -> 01 -> ... -> End
        # Si quieres un laberinto caótico total, descomenta el shuffle.
        # random.shuffle(all_names) 

        # Step 1: build a spanning tree (mandatory backbone)
        for i in range(len(all_names) - 1):
            a, b = all_names[i], all_names[i + 1]
            self.graph.add_edge(a, b)
            self.traversable_edges.add((a, b))
            self.traversable_edges.add((b, a))

        # Step 2: add optional extra edges (branches/shortcuts)
        if extra_edges is None: extra_edges = num_nodes // 2
        for _ in range(extra_edges):
            a, b = random.sample(all_names, 2)
            if not self.graph.has_edge(a, b):
                self.graph.add_edge(a, b)
                if random.random() < traversable_prob:
                    self.traversable_edges.add((a, b))
                    self.traversable_edges.add((b, a))

    def edges(self):
        return list(self.graph.edges())