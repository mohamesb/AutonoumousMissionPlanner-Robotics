import numpy as np
import time
from math import pi, tan, sqrt
from itertools import product

from mission_planner_ttk4192 import map_grid_robplan
from utils.grid import Grid_robplan
from utils.car import RoboCar
from utils.environment import Environment_robplan
from utils.dubins_path import DubinsPath
from utils.astar import Astar
from utils.utils import (
    animate_solution,
    get_discretized_thetas,
    plotstart,
    round_theta,
    same_point
)

class Node:
    def __init__(self, grid_pos, pos):
        self.grid_pos = grid_pos
        self.pos = pos
        self.g = None
        self.g_ = None
        self.f = None
        self.parent = None
        self.phi = 0
        self.m = None
        self.branches = []

    def __eq__(self, other):
        return self.grid_pos == other.grid_pos
    
    def __hash__(self):

        return hash((self.grid_pos))



class HybridAStarPlanner:
    """Hybrid A* search algorithm adapted for TurtleBot3."""

    def __init__(self, car, grid, reverse, unit_theta, dt, dubins_check):
        self.car = car
        self.grid = grid
        self.reverse = reverse
        self.unit_theta = unit_theta
        self.dt = dt
        self.dubins_check = dubins_check

        self.start = car.start_pos
        self.goal = car.end_pos

        self.turn_radius = car.l / tan(car.max_phi)
        self.drive_steps = int(sqrt(2) * grid.cell_size / dt) + 1
        self.arc_length = self.drive_steps * dt

        self.steering_angles = [-car.max_phi, 0, car.max_phi]
        self.motions = [1, -1] if reverse else [1]
        self.motion_combinations = list(product(self.motions, self.steering_angles))

        self.dubins = DubinsPath(car)
        self.astar = Astar(grid, self.goal[:2])
        self.thetas = get_discretized_thetas(unit_theta)

        # Heuristic weight factors
        self.w_astar = 1.5
        self.w_manhattan = 0.05
        self.w_steer_change = 0.4
        self.w_turning = 0.2
        self.w_reverse = 1.0

    def _create_node(self, pos):
        theta = round_theta(pos[2] % (2 * pi), self.thetas)
        cell_id = self.grid.to_cell_id(pos[:2])
        return Node(grid_pos=cell_id + [theta], pos=pos)

    def _heuristic(self, pos):
        astar_cost = self.astar.search_path(pos[:2])
        h1 = astar_cost * self.grid.cell_size if astar_cost is not None else 1e9
        h2 = abs(self.goal[0] - pos[0]) + abs(self.goal[1] - pos[1])
        return self.w_astar * h1 + self.w_manhattan * h2

    def _generate_successors(self, node, use_heuristic, add_extra_cost):
        successors = []
        for motion, steer in self.motion_combinations:
            if node.m and node.phi == steer and node.m * motion == -1:
                continue
            if node.m and node.m == 1 and motion == -1:
                continue

            pos = node.pos
            path = [motion, pos[:2]]
            for _ in range(self.drive_steps):
                pos = self.car.step(pos, steer, motion)
                path.append(pos[:2])

            is_safe = (
                self.dubins.is_straight_route_safe(node.pos, pos)
                if steer == 0 else
                self.dubins.is_turning_route_safe(*self.car.get_params(node.pos, steer), node.pos, pos)
            )
            if not is_safe:
                continue

            child = self._create_node(pos)
            child.phi = steer
            child.m = motion
            child.parent = node
            child.g = node.g + self.arc_length
            child.g_ = node.g_ + self.arc_length

            if add_extra_cost:
                if steer != node.phi:
                    child.g += self.w_steer_change * self.arc_length
                if steer != 0:
                    child.g += self.w_turning * self.arc_length
                if motion == -1:
                    child.g += self.w_reverse * self.arc_length

            heuristic = self._heuristic(child.pos) if use_heuristic else (
                abs(self.goal[0] - child.pos[0]) + abs(self.goal[1] - child.pos[1])
            )
            child.f = child.g + heuristic
            successors.append((child, path))

        return successors

    def _trace_back(self, node):
        path = []
        while node.parent:
            path.append((node.pos, node.phi, node.m))
            node = node.parent
        return list(reversed(path))

    def _evaluate_final_path(self, open_list, closed_list, best, current_cost, dubins_route, max_checks=10):
        open_list.sort(key=lambda x: x.f)

        for candidate in open_list[:max_checks]:
            path_options = self.dubins.find_tangents(candidate.pos, self.goal)
            new_route, cost, valid = self.dubins.best_tangent(path_options)

            if valid and cost + candidate.g_ < current_cost + best.g_:
                best = candidate
                current_cost = cost
                dubins_route = new_route

        if best in open_list:
            open_list.remove(best)
            closed_list.append(best)

        return best, current_cost, dubins_route

    def find_path(self, use_heuristic=True, add_extra_cost=True):
        root = self._create_node(self.start)
        root.g = 0
        root.g_ = 0
        root.f = root.g + (self._heuristic(root.pos) if use_heuristic else 0)

        open_list = [root]
        closed_list = []

        iterations = 0
        while open_list:
            iterations += 1
            current = min(open_list, key=lambda x: x.f)
            open_list.remove(current)
            closed_list.append(current)

            if iterations % self.dubins_check == 0:
                paths = self.dubins.find_tangents(current.pos, self.goal)
                dubins_route, cost, is_valid = self.dubins.best_tangent(paths)

                if is_valid:
                    current, cost, dubins_route = self._evaluate_final_path(open_list, closed_list, current, cost, dubins_route)
                    path = self._trace_back(current) + dubins_route
                    print(f"Path cost: {round(cost + current.g_, 2)} | Iterations: {iterations}")
                    return self.car.get_path(self.start, path), closed_list

            for child, branch in self._generate_successors(current, use_heuristic, add_extra_cost):
                if child in closed_list:
                    continue

                if child not in open_list:
                    current.branches.append(branch)
                    open_list.append(child)
                elif child.g < open_list[open_list.index(child)].g:
                    current.branches.append(branch)
                    existing = open_list[open_list.index(child)]
                    for b in existing.parent.branches:
                        if same_point(b[-1], existing.pos[:2]):
                            existing.parent.branches.remove(b)
                            break
                    open_list.remove(child)
                    open_list.append(child)

        return None, None


def run_hybrid_astar(heu, start, goal, reverse=False, extra_cost=True, visualize=False, sim=True):
    car_length = 0.5
    car_width = 0.3
    max_steer = pi / 5

    tc = map_grid_robplan(sim)
    env = Environment_robplan(tc.obs, lx=5.21, ly=2.75, safe_distance=0.03)
    car = RoboCar(env, start, goal, car_length, max_steer, car_width)
    grid = Grid_robplan(env, cell_size=0.1)

    if visualize:
        plotstart(env, car)

    planner = HybridAStarPlanner(car, grid, reverse, unit_theta, dt, dubins_check)
    t0 = time.time()
    path, closed = planner.find_path(use_heuristic=heu == 1, add_extra_cost=extra_cost)
    print(f"Planning time: {round(time.time() - t0, 3)}s")

    if not path:
        raise RuntimeError("Hybrid A* failed to find a path.")

    path = path[:-1:7] + [path[-1]]
    waypoints = np.array([p.pos for p in path])
    cars = [p.model[0] for p in path]

    if visualize:
        animate_solution(car, env, path, waypoints[:, 0], waypoints[:, 1], cars, grid.cell_size)

    return waypoints