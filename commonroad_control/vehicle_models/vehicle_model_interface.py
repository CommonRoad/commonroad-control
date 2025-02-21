from abc import ABC, abstractmethod


import casadi as cas




class VehicleModelInterface(ABC):
    def __init__(self):
        pass

    @abstractmethod
    def simulate_forward(self,x,u,w):
        pass

    @abstractmethod
    def discretize(self,x,u):
        pass

    @abstractmethod
    def linearize(self,x,u):
        pass
