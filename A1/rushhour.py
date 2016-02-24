#Look for #IMPLEMENT tags in this file. These tags indicate what has
#to be implemented to complete the warehouse domain.

'''
rushhour STATESPACE
'''
#   You may add only standard python imports---i.e., ones that are automatically
#   available on CDF.
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

from search import *
from random import randint

##################################################
# The search space class 'rushhour'             #
# This class is a sub-class of 'StateSpace'      #
##################################################


class rushhour(StateSpace):
    def __init__(self, action, gval, parent, vehicles, board):
        """Initialize a rushhour search state object."""
        StateSpace.__init__(self, action, gval, parent)
        self.vehicles = vehicles
        self.board = board

        rushhour_set_goal(self.board[1], self.board[2])

    def successors(self):
        '''Return list of rushhour objects that are the successors of the current object.

        Each vehicle can move in two directions based on their is_horizontal property.
        If horizontal, they can move E and W. If vertical, they can move N and S.

        '''
        States = list()
        board = self.get_board_properties()
        board_x = board[0][1]
        board_y = board[0][0]

        vehiclelist = self.get_vehicle_statuses()

        for current_vehicle in vehiclelist:
            vehiclecopy = list(vehiclelist) #vehicle list copy
            if current_vehicle[3]: #horizontal
                if can_move(vehiclecopy, current_vehicle, 'W', board):
                    vehiclecopy.remove(current_vehicle)
                    v_length = current_vehicle[2]
                    x = current_vehicle[1][0] + 1 #vehicle's x coordinate + the amount of space it wants to move
                    if x >= board_x: #if adding 1 space brings it off the board coordinates
                        x = x - board_x
                    vehiclecopy.insert(0, [current_vehicle[0], (x, current_vehicle[1][1]), current_vehicle[2], current_vehicle[3], current_vehicle[4]])
                    States.append(rushhour("move_vehicle("+current_vehicle[0]+", 'W')", self.gval+1, self, vehiclecopy, board))
                    vehiclecopy = list(vehiclelist)
                if can_move(vehiclecopy, current_vehicle, 'E', board):
                    vehiclecopy.remove(current_vehicle)
                    x = current_vehicle[1][0] - 1
                    if x < 0:
                        x = board_x + x
                    vehiclecopy.insert(0, [current_vehicle[0], (x, current_vehicle[1][1]), current_vehicle[2], current_vehicle[3], current_vehicle[4]])
                    States.append(rushhour("move_vehicle("+current_vehicle[0]+", 'E')", self.gval+1, self, vehiclecopy, board))
            else:
                if can_move(vehiclecopy, current_vehicle, 'S', board):
                    vehiclecopy.remove(current_vehicle)
                    y = current_vehicle[1][1] - 1
                    if y < 0:
                        y = board_y + y
                    vehiclecopy.insert(0, [current_vehicle[0], (current_vehicle[1][0], y), current_vehicle[2], current_vehicle[3], current_vehicle[4]])
                    States.append(rushhour("move_vehicle("+current_vehicle[0]+", 'S')", self.gval+1, self, vehiclecopy, board))
                    vehiclecopy = list(vehiclelist)
                if can_move(vehiclecopy, current_vehicle, 'N', board):
                    vehiclecopy.remove(current_vehicle)
                    y = current_vehicle[1][1] + 1
                    if y >= board_y:
                        y = y - board_y
                    vehiclecopy.insert(0, [current_vehicle[0], (current_vehicle[1][0], y), current_vehicle[2], current_vehicle[3], current_vehicle[4]])
                    States.append(rushhour("move_vehicle("+current_vehicle[0]+", 'N')", self.gval+1, self, vehiclecopy, board))

        return States


    def hashable_state(self):
        '''Return a data item that can be used as a dictionary key to UNIQUELY represent the state.

        Return an immutable and unique representation of s (current state). This method is used by
        the search functions to check if s has been generated before (for cycle checking).

        '''

        return tuple(self.get_board_properties()), tuple(tuple(x) for x in self.get_vehicle_statuses())

    def print_state(self):
        #DO NOT CHANGE THIS FUNCTION---it will be used in auto marking
        #and in generating sample trace output.
        #Note that if you implement the "get" routines
        #(rushhour.get_vehicle_statuses() and rushhour.get_board_size())
        #properly, this function should work irrespective of how you represent
        #your state.

        if self.parent:
            print("Action= \"{}\", S{}, g-value = {}, (From S{})".format(self.action, self.index, self.gval, self.parent.index))
        else:
            print("Action= \"{}\", S{}, g-value = {}, (Initial State)".format(self.action, self.index, self.gval))

        print("Vehicle Statuses")
        for vs in sorted(self.get_vehicle_statuses()):
            print("    {} is at ({}, {})".format(vs[0], vs[1][0], vs[1][1]), end="")
        board = get_board(self.get_vehicle_statuses(), self.get_board_properties())
        print('\n')
        print('\n'.join([''.join(board[i]) for i in range(len(board))]))

#Data accessor routines.

    def get_vehicle_statuses(self):
        '''Return list containing the status of each vehicle
           This list has to be in the format: [vs_1, vs_2, ..., vs_k]
           with one status list for each vehicle in the state.
           Each vehicle status item vs_i is itself a list in the format:
                 [<name>, <loc>, <length>, <is_horizontal>, <is_goal>]
           Where <name> is the name of the vehicle (a string)
                 <loc> is a location (a pair (x,y)) indicating the front of the vehicle,
                       i.e., its length is counted in the positive x- or y-direction
                       from this point
                 <length> is the length of that vehicle
                 <is_horizontal> is true iff the vehicle is oriented horizontally
                 <is_goal> is true iff the vehicle is a goal vehicle
        '''
        return self.vehicles

    def get_board_properties(self):
        '''Return (board_size, goal_entrance, goal_direction)
           where board_size = (m, n) is the dimensions of the board (m rows, n columns)
                 goal_entrance = (x, y) is the location of the goal
                 goal_direction is one of 'N', 'E', 'S' or 'W' indicating
                                the orientation of the goal
        '''
        return self.board

#############################################
# heuristics                                #
#############################################


def heur_zero(state):
    '''Zero Heuristic use to make A* search perform uniform cost search'''
    return 0


def heur_min_moves(state):
    '''rushhour heuristic'''
    #We want an admissible heuristic. Getting to the goal requires
    #one move for each tile of distance.
    #Since the board wraps around, there are two different
    #directions that lead to the goal.
    #NOTE that we want an estimate of the number of ADDITIONAL
    #     moves required from our current state
    #1. Proceeding in the first direction, let MOVES1 =
    #   number of moves required to get to the goal if it were unobstructed
    #2. Proceeding in the second direction, let MOVES2 =
    #   number of moves required to get to the goal if it were unobstructed
    #
    #Our heuristic value is the minimum of MOVES1 and MOVES2 over all goal vehicles.
    #You should implement this heuristic function exactly, even if it is
    #tempting to improve it.

    min_moves = list()

    for v in state.get_vehicle_statuses():
        if v[4]: #goal_vehicle
            if state.goal_state[1] == 'E': #tail needs to be in goal location
                tail = v[1][0] + v[2] - 1
                MOVES1 = abs(state.goal_state[0][0] - tail)
                MOVES2 = state.get_board_properties()[0][1] - MOVES1
                min_moves.append(min(MOVES1, MOVES2))
            elif state.goal_state[1] == 'W': #head needs to be in goal location
                #calculate difference between x values if it goes one direction (left)
                head = v[1][0]
                MOVES1 = abs(state.goal_state[0][0] - head)
                MOVES2 = state.get_board_properties()[0][1] - MOVES1
                min_moves.append(min(MOVES1, MOVES2))
            elif state.goal_state[1] == 'N': #head needs to be in goal location
                head = v[1][1]
                MOVES1 = abs(state.goal_state[0][1] - head)
                MOVES2 = state.get_board_properties()[0][0] - MOVES1
                min_moves.append(min(MOVES1, MOVES2))
            elif state.goal_state[1] == 'S': #tail needs to be in goal location
                tail = v[1][1] + v[2] - 1
                MOVES1 = abs(tail - state.goal_state[0][1])
                MOVES2 = state.get_board_properties()[0][0] - MOVES1
                min_moves.append(min(MOVES1, MOVES2))

    return min(min_moves)


def rushhour_set_goal(gloc, orientation):
    '''set the current goal'''
    rushhour.goal_state = (gloc, orientation)

def rushhour_goal_fn(state):
    '''Have we reached a goal state

    Return True if StateSpace state is a goal and False otherwise
    '''
    for v in state.get_vehicle_statuses():
        if v[4]: #is_goal
            goal_vehicle = v
            if rushhour.goal_state[1] == 'E': #tail needs to be in goal position
                tail = goal_vehicle[1][0] + goal_vehicle[2] - 1
                if tail >= state.get_board_properties()[0][1]:
                    tail = tail - state.get_board_properties()[0][1]
                if (tail, goal_vehicle[1][1]) == rushhour.goal_state[0]:
                    return True

            elif rushhour.goal_state[1] == 'S': #tail needs to be in goal position
                tail = goal_vehicle[1][1] + goal_vehicle[2] - 1
                if tail >= state.get_board_properties()[0][0]:
                    tail = tail - state.get_board_properties()[0][0]
                if (goal_vehicle[1][0], tail) == rushhour.goal_state[0]:
                    return True

            elif rushhour.goal_state[1] == 'W' or rushhour.goal_state[1] == 'N': #head needs to be in goal position
                if (goal_vehicle[1]) == rushhour.goal_state[0]:
                    return True

    return False



def make_init_state(board_size, vehicle_list, goal_entrance, goal_direction):
    '''Input the following items which specify a state and return a rushhour object
       representing this initial state.
         The state's its g-value is zero
         The state's parent is None
         The state's action is the dummy action "START"
       board_size = (m, n)
          m is the number of rows in the board
          n is the number of columns in the board
       vehicle_list = [v1, v2, ..., vk]
          a list of vehicles. Each vehicle vi is itself a list
          vi = [vehicle_name, (x, y), length, is_horizontal, is_goal] where
              vehicle_name is the name of the vehicle (string)
              (x,y) is the location of that vehicle (int, int)
              length is the length of that vehicle (int)
              is_horizontal is whether the vehicle is horizontal (Boolean)
              is_goal is whether the vehicle is a goal vehicle (Boolean)
      goal_entrance is the coordinates of the entrance tile to the goal and
      goal_direction is the orientation of the goal ('N', 'E', 'S', 'W')

   NOTE: for simplicity you may assume that
         (a) no vehicle name is repeated
         (b) all locations are integer pairs (x,y) where 0<=x<=n-1 and 0<=y<=m-1
         (c) vehicle lengths are positive integers
    '''

    game = rushhour("START", 0, None, vehicle_list, (board_size, goal_entrance, goal_direction))
    return game

########################################################
#   Functions provided so that you can more easily     #
#   Test your implementation                           #
########################################################


def get_board(vehicle_statuses, board_properties):
    #DO NOT CHANGE THIS FUNCTION---it will be used in auto marking
    #and in generating sample trace output.
    #Note that if you implement the "get" routines
    #(rushhour.get_vehicle_statuses() and rushhour.get_board_size())
    #properly, this function should work irrespective of how you represent
    #your state.
    (m, n) = board_properties[0]
    board = [list(['.'] * n) for i in range(m)]
    for vs in vehicle_statuses:
        for i in range(vs[2]):  # vehicle length
            if vs[3]:
                # vehicle is horizontal
                board[vs[1][1]][(vs[1][0] + i) % n] = vs[0][0]
                # represent vehicle as first character of its name
            else:
                # vehicle is vertical
                board[(vs[1][1] + i) % m][vs[1][0]] = vs[0][0]
                # represent vehicle as first character of its name
    # print goal
    board[board_properties[1][1]][board_properties[1][0]] = board_properties[2]
    return board


def make_rand_init_state(nvehicles, board_size):
    '''Generate a random initial state containing
       nvehicles = number of vehicles
       board_size = (m,n) size of board
       Warning: may take a long time if the vehicles nearly
       fill the entire board. May run forever if finding
       a configuration is infeasible. Also will not work any
       vehicle name starts with a period.

       You may want to expand this function to create test cases.
    '''

    (m, n) = board_size
    vehicle_list = []
    board_properties = [board_size, None, None]
    for i in range(nvehicles):
        if i == 0:
            # make the goal vehicle and goal
            x = randint(0, n - 1)
            y = randint(0, m - 1)
            is_horizontal = True if randint(0, 1) else False
            vehicle_list.append(['gv', (x, y), 2, is_horizontal, True])
            if is_horizontal:
                board_properties[1] = ((x + n // 2 + 1) % n, y)
                board_properties[2] = 'W' if randint(0, 1) else 'E'
            else:
                board_properties[1] = (x, (y + m // 2 + 1) % m)
                board_properties[2] = 'N' if randint(0, 1) else 'S'
        else:
            board = get_board(vehicle_list, board_properties)
            conflict = True
            while conflict:
                x = randint(0, n - 1)
                y = randint(0, m - 1)
                is_horizontal = True if randint(0, 1) else False
                length = randint(2, 3)
                conflict = False
                for j in range(length):  # vehicle length
                    if is_horizontal:
                        if board[y][(x + j) % n] != '.':
                            conflict = True
                            break
                    else:
                        if board[(y + j) % m][x] != '.':
                            conflict = True
                            break
            vehicle_list.append([str(i), (x, y), length, is_horizontal, False])

    return make_init_state(board_size, vehicle_list, board_properties[1], board_properties[2])


def test(nvehicles, board_size):
    s0 = make_rand_init_state(nvehicles, board_size)
    se = SearchEngine('astar', 'full')
    se.trace_on(2)
    se.search(s0, rushhour_goal_fn, heur_min_moves)


def can_move(vehiclelist, vehicle, direction, board):
    '''
    Return True if vehicle can move in direction. Return false otherwise.

    Check if potential move's new coordinates is in the occupancy list of
    another vehicle.
    '''

    if (direction == 'N' or direction == 'S') and vehicle[3]:
        return False
    if (direction == 'E' or direction == 'W') and not vehicle[3]:
        return False

    #calculate occupancy list of tuples of the vehicle
    occ = list()

    for v in vehiclelist:
        #check individual conflicts but don't include self
        if v[0] != vehicle[0]:
            if v[3]: #current vehicle is horizontal
                for i in range(v[2]):
                    x = v[1][0]
                    y = v[1][1]
                    if (x + i) >= board[0][1]:
                        occ.append(((x + i) - board[0][1], y))
                    else:
                        occ.append((x + i, y))
            else: #current vehicle is vertical
                for i in range(v[2]):
                    x = v[1][0]
                    y = v[1][1]
                    if y + i >= board[0][0]:
                        occ.append((x, (y + i) - board[0][0]))
                    else:
                        occ.append((x, y + i))

    #check if directly on another's position
    if direction == 'W':
        newx = vehicle[1][0] + (vehicle[2] - 1) + 1
        if newx >= board[0][1]:
            newx = newx - board[0][1]
        if (newx, vehicle[1][1]) in occ:
            return False
    elif direction == 'E':
        newx = vehicle[1][0] - 1
        if newx < 0:
            newx = board[0][1] + newx
        if (newx, vehicle[1][1]) in occ:
            return False
    elif direction == 'S':
        newy = vehicle[1][1] - 1
        if newy < 0:
            newy = board[0][0] + newy
        if (vehicle[1][0], newy) in occ:
            return False
    elif direction == 'N':
        newy = vehicle[1][1] + (vehicle[2] - 1) + 1
        if newy >= board[0][0]:
            newy = newy - board[0][0]
        if (vehicle[1][0], newy) in occ:
            return False

    return True

if __name__ == "__main__":
    test(9, (7,7))