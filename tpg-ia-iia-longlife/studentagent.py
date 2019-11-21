#encoding: utf8

#nome: Daniel Gonçalves Nº 80069
#nome: Miguel Simões Nº 79957
#nome: Saveliy Ivanov Nº 80298

from agent import *
from math import *
import random
import operator

class StudentAgent(Agent):
    def __init__(self, name, body, world):
        super().__init__(name, body, world)
        self.domain = Paths(world)
        self.dead_ends = self.find_dead_ends()
        self.tunnels = self.find_tunnels()
        self.keypoints = self.calc_keypoints()
        self.keypoints_copy = [x for x in self.keypoints]
        self.path_to_food = []
        self.friend_food = None  # goal do outro agente 

    def remap(self, point):
        c1, c2 = point[0], point[1]
        if(point == (-1, c2)):
            return (self.world.size.x - 1, c2)
        elif(point == (c1, -1)):
            return (c1, self.world.size.y - 1)
        elif(point == (self.world.size.x, c2)):
            return (0, c2)
        elif(point == (c1, self.world.size.y)):
            return (c1, 0)
        else:
            return point

    # função de pré-processamento que tem a finalidade de descobrir todos os tuneis existentes no mapa e identificá-los com um id        
    def find_tunnels(self): 
        tunnel_id = 0
        tunnels = {}
        this_tunnel = []
        for x in range(self.world.size.x):
            for y in range(self.world.size.y):
                this_position = (x, y)
                all_tunnels = [item for tunnel in tunnels.values() for item in tunnel]
                if(this_position not in self.world.walls and this_position not in all_tunnels and this_position not in self.dead_ends):
                    if(self.remap((x + 1, y)) in self.world.walls and self.remap((x - 1, y)) in self.world.walls):
                        self.no_scope(this_position, this_tunnel)
                        if this_tunnel:
                            tunnels[tunnel_id] = this_tunnel
                            this_tunnel = []
                            tunnel_id += 1

                    if(self.remap((x, y + 1)) in self.world.walls and self.remap((x, y - 1)) in self.world.walls):
                        self.no_scope(this_position, this_tunnel)
                        if this_tunnel:
                            tunnels[tunnel_id] = this_tunnel
                            this_tunnel = []
                            tunnel_id += 1
        return tunnels

    # funcao de pré-processamento que encontra e retorna um array com todos os dead_ends existentes no mapa.
    def find_dead_ends(self):
        dead_ends = []
        for x in range(self.world.size.x):
            for y in range(self.world.size.y):
                this_position = (x, y)
                if(this_position not in self.world.walls):
                    if((x + 1, y) in self.world.walls and (x - 1, y) in self.world.walls and (x, y + 1) in self.world.walls):
                        dead_ends.append(this_position)
                        self.no_scope(this_position, dead_ends)
                    elif((x + 1, y) in self.world.walls and (x - 1, y) in self.world.walls and (x, y - 1) in self.world.walls):
                        dead_ends.append(this_position)
                        self.no_scope(this_position, dead_ends)
                    elif((x + 1, y) in self.world.walls and (x, y - 1) in self.world.walls and (x, y + 1) in self.world.walls):
                        dead_ends.append(this_position)
                        self.no_scope(this_position, dead_ends)
                    elif((x - 1, y) in self.world.walls and (x, y - 1) in self.world.walls and (x, y + 1) in self.world.walls):
                        dead_ends.append(this_position)
                        self.no_scope(this_position, dead_ends)
        return dead_ends

    # funcao que auxilia o funcionamento da funcao find_tunnels() e find_dead_ends()
    def no_scope(self, point, dead_ends):
        x, y = point[0], point[1]
        neighbours = [self.remap((x + 1, y)), self.remap((x - 1, y)), self.remap((x, y - 1)), self.remap((x, y + 1))]
        for neighbour in neighbours:
            if neighbour not in self.world.walls and neighbour not in dead_ends:
                neighbours_of_neighbour = [self.remap((neighbour[0] + 1, neighbour[1])), self.remap((neighbour[0] - 1, neighbour[1])), self.remap((neighbour[0], neighbour[1] - 1)), self.remap((neighbour[0], neighbour[1] + 1))]
                count = len(list(filter(lambda cell : cell in self.world.walls, neighbours_of_neighbour)))
                if count >= 2:
                    dead_ends.append(neighbour)
                    self.no_scope(neighbour, dead_ends)
                elif(count == 1 and len(list(filter(lambda cell : cell in dead_ends, neighbours_of_neighbour))) == 2):
                    dead_ends.append(neighbour)
                    self.no_scope(neighbour, dead_ends)
                else:
                    continue

    # cálculo de 6 pontos que dividem o mapa em 6 quadrados. Etes 6 pontos servem para 
    # quando os agentes não têm nenhum nutriente na sua visão se dirigirem para um destes 6 pontos.  
    def calc_keypoints(self, anchors = None):
        if(anchors == None):
            x, y = self.world.size.x, self.world.size.y
            anchors = [(x/6, y/4), ((x/2) - 1, y/4), (x - (x/6) - 1, y/4) , (x/6, y - (y/4) - 1), ((x/2) - 1, y - (y/4) - 1), (x - (x/6) - 1, y - (y/4) - 1)]
        if (all(list(map(lambda x: x not in self.world.walls and x not in self.dead_ends, anchors)))):
            return anchors
        else:
            for i in range(len(anchors)):
                el = anchors[i]
                if el in self.world.walls or el in self.dead_ends:
                    neighbours = [self.remap((el[0] + 1, el[1])), self.remap((el[0] - 1, el[1])), self.remap((el[0], el[1] - 1)), self.remap((el[0], el[1] + 1))]
                    for neighbour in neighbours:
                        anchors[i] = neighbour
                        return self.calc_keypoints(anchors)
        return None

    # função de auxílio para encontrar os nutrientes que estão na visão do agente
    # e escolher um desses pontos para ser o goal do agente . 
    def find_food(self, food_in_range):
        if not food_in_range:
            return None, None
        else:
            coordinates, food_type = food_in_range[0][0][0], food_in_range[0][0][1]
            if(self.nutrients['S'] > self.nutrients['M'] and food_type == 'M'):
                return coordinates, food_type
            elif(food_type == 'S' and self.nutrients['S'] < 500):
                return coordinates, food_type
            elif(food_type == 'M'):
                return coordinates, food_type
            else:
                food_in_range.pop(0)
                return self.find_food(food_in_range)

    def chooseAction(self, vision, msg):
        head = self.body[0]
        validact = ACTIONS[:1]
        action = (0, 0)
        tunnel = [] # tunel do outro agente 

        ## Mensagens Possiveis :    
        # msg = b'(12,34)'  -> contém o goal do outro agente
        # msg=b'(12,32)|t4' -> contém o goal do outro agente e o id do túnel em que ele entrou 
        if(msg != b''): 
            message_parts = msg.decode('utf-8').split('|')
            if message_parts[0]:
                self.friend_food = tuple(int(x) for x in message_parts[0][1:-1].split(',')) 
            if len(message_parts) > 1:
                tunnel = self.tunnels[int(message_parts[1].strip('t'))]

        msg = b''
        closer_food = [None, None]

        food_in_range = {}

        # Procura os nutrientes que estão na visão do agente e ordena-os num dicionário segundo a distância 
        # a que os mesmo estão da cabeça do agente. Depois é chamada a função "fin_food(dic)" onde se vai decidir 
        # qual será o goal do agente de entre os nutrientes que estão na sua visão  
        if (vision.food and not self.path_to_food):
            for food in vision.food:
                if (food not in self.dead_ends and self.friend_food != food):
                    food_in_range[(food, vision.food[food])] = self.world.dist(head, food)
            food_in_range = sorted(food_in_range.items(), key = lambda distance: distance[1])

            closer_food[0], closer_food[1] = self.find_food(food_in_range)

        if(closer_food != [None, None]): # se o agente tiver um nutriente como goal 
            if not self.path_to_food:

                problem = SearchProblem(self.domain, head, closer_food[0])
                tree = SearchTree(problem, self.body, 'a*')
                self.path_to_food = tree.search(tunnel)

                # caso em que o agente nao consegue encontrar um path para o goal que ele esta à procura porque tem o seu corpo a obstruir o caminho 
                if (self.path_to_food == [head]): 
                    self.path_to_food = []
                    rand_goal = self.world.randCoords()
                    while(rand_goal in self.world.walls or rand_goal in self.dead_ends or rand_goal in vision.bodies or rand_goal in tunnel):
                        rand_goal = self.world.randCoords()
                    problem = SearchProblem(self.domain, head, rand_goal)
                    tree = SearchTree(problem, self.body, 'a*')
                    self.path_to_food = tree.search(tunnel)     
                
                if not self.path_to_food:
                    self.path_to_food = [head]

                # sempre que é calculado um novo caminho para um nutriente envia-se mensagem ao outro agente para ele saber qual é o goal do agente 
                # atual e desta maneira terem os dois agentes goals diferentes 
                msg = str(closer_food[0]).encode('utf-8') 

                if(closer_food[1] == 'M'): # se o goal for um nutriente vermelho entao desprezamos metade do caminho calculado para este goal
                    if(len(self.path_to_food) > 2):
                        self.path_to_food[int(len(self.path_to_food)/2):] = []

            x, y = self.path_to_food[0][0], self.path_to_food[0][1] # proxima posicao do agente 

            # Se a proxima posicao do agente está num tunel entao adiciona-se o id do tunel em questao à mensagem
            for key in self.tunnels:  
                if ((x,y) in self.tunnels[key]):    
                    if 't' + str(key) not in msg.decode('utf-8'):
                        msg += ('|t' + str(key)).encode('utf-8')

            # se a proxima posicao do agente estiver no tunel da mensagem que o outro agente lhe enviou , 
            # entao este agente nao pode assumir essa proxima posicao e o que ele vai fazer é calcular um 
            # novo caminho para outro ponto qualquer que nao esteja nas paredes, nos bodies , no tunel onde o outro agente esta ou nos dead_ends
            if ((x,y) in tunnel):
                if not self.keypoints_copy:
                    self.keypoints_copy = [x for x in self.keypoints]

                rand_goal = self.keypoints_copy[0]
                self.keypoints_copy[0:1] = []
                while(rand_goal in self.world.walls or rand_goal in self.dead_ends or rand_goal in vision.bodies or rand_goal in tunnel):
                    rand_goal = self.world.randCoords()
                problem = SearchProblem(self.domain, head, rand_goal)
                tree = SearchTree(problem, self.body, 'a*')
                self.path_to_food = tree.search(tunnel)

            action = Point(x - head[0], y - head[1])# acção que será retornada pelo chooseAction
            newpos = self.world.translate(head, action)
            if(Point(head[0] + action[0], head[1] + action[1]) in vision.bodies): # verificar se o corpo do outro agente esta na proxima posicao deste agente 

                self.path_to_food = []
                validact.append((0, 0))
                for action in ACTIONS[1:]:
                    newpos = self.world.translate(head, action)
                    if(newpos not in self.world.walls and newpos not in vision.bodies and newpos not in self.dead_ends and newpos not in tunnel):
                        self.path_to_food = [newpos]
                        break
            elif(newpos not in self.world.walls and newpos not in vision.bodies and newpos not in tunnel):
                validact.append(action)
                self.path_to_food[0:1] = []
            else:
                validact.append((0,0))
        else: # caso o agente nao tenha nutrientes na sua visao 
            if not self.path_to_food: # se o agente ainda nao tem um goal entao tem de se calcular um random goal 
                if not self.keypoints_copy:
                    self.keypoints_copy = [x for x in self.keypoints]

                rand_goal = self.keypoints_copy[0]
                self.keypoints_copy[0:1] = []

                while(rand_goal in self.world.walls or rand_goal in self.dead_ends or rand_goal in vision.bodies or rand_goal in tunnel):
                    rand_goal = self.world.randCoords()

                problem = SearchProblem(self.domain, head, rand_goal)
                tree = SearchTree(problem, self.body, 'a*')
                self.path_to_food = tree.search(tunnel)

                # caso em que o agente nao consegue encontrar um path para o goal que ele esta à procura porque tem o seu corpo a obstruir o caminho
                if (self.path_to_food == [head]):
                    self.path_to_food = []
                    rand_goal = self.world.randCoords()
                    while(rand_goal in self.world.walls or rand_goal in self.dead_ends or rand_goal in vision.bodies or rand_goal in tunnel):
                        rand_goal = self.world.randCoords()
                    problem = SearchProblem(self.domain, head, rand_goal)
                    tree = SearchTree(problem, self.body, 'a*')
                    self.path_to_food = tree.search(tunnel)
                    
                if not self.path_to_food:
                    self.path_to_food = [head]

                if(len(self.path_to_food) > 2):  # despreza-se sempre metade do caminho calculado quando o agente vai em direcao a um random goal
                    self.path_to_food[int(len(self.path_to_food)/2):] = []

            x, y = self.path_to_food[0][0], self.path_to_food[0][1] # proxima posicao do agente 

            # Se a proxima posicao do agente está num tunel entao adiciona-se o id do tunel em questao à mensagem
            for key in self.tunnels: 
                if ((x,y) in self.tunnels[key]):
                    if 't' + str(key) not in msg.decode('utf-8'):
                        msg += ('|t' + str(key)).encode('utf-8')

            # se a proxima posicao do agente estiver no tunel da mensagem que o outro agente lhe enviou , 
            # entao este agente nao pode assumir essa proxima posicao e o que ele vai fazer é calcular um 
            # novo caminho para outro ponto qualquer que nao esteja nas paredes, nos bodies , no tunel onde o outro agente esta ou nos dead_ends
            if ((x,y) in tunnel):
                if not self.keypoints_copy:
                    self.keypoints_copy = [x for x in self.keypoints]
                rand_goal = self.keypoints_copy[0]
                self.keypoints_copy[0:1] = []
                while(rand_goal in self.world.walls or rand_goal in self.dead_ends or rand_goal in vision.bodies or rand_goal in tunnel):
                    rand_goal = self.world.randCoords()
                problem = SearchProblem(self.domain, head, rand_goal)
                tree = SearchTree(problem, self.body, 'a*')
                self.path_to_food = tree.search(tunnel)

            action = Point(x - head[0], y - head[1])# acção que será retornada pelo chooseAction
            newpos = self.world.translate(head, action)
            if(Point(head[0] + action[0], head[1] + action[1]) in vision.bodies):
                validact.append((0, 0))
                self.path_to_food = []
                for action in ACTIONS[1:]:
                    newpos = self.world.translate(head, action)
                    if(newpos not in self.world.walls and newpos not in vision.bodies and newpos not in self.dead_ends and newpos not in tunnel):
                        self.path_to_food = [newpos]
                        break
            elif(newpos not in self.world.walls and newpos not in vision.bodies and newpos not in tunnel):
                validact.append(action)
                self.path_to_food[0:1] = []
            else:
                validact.append((0,0))

        action = validact[1]

        # tratamento dos casos em que a action é um valor absurdo . Isto é quando os agentes saem do mapa e aparecem do lado oposto.
        if(action == (0,-39)):
            action = Point(0,1)
        elif (action == (-59,0)):
            action = Point(1,0)
        elif (action == (59,0)):
            action = Point(-1,0)
        elif (action == (0,39)):
            action = Point(0,-1)

        return action, msg

class SearchDomain:
    def __init__(self):
        abstract

    def actions(self, state):
        abstract

    def result(self, state, action):
        abstract

    def cost(self, state, action):
        abstract

    def heuristic(self, state, goal_state):
        abstract

class SearchProblem:
    def __init__(self, domain, initial, goal):
        self.domain = domain
        self.initial = initial
        self.goal = goal

    def goal_test(self, point):
        return point == self.goal

class SearchNode:
    def __init__(self, coords, parent, cost, heuristic, depth):
        self.coords = coords
        self.parent = parent
        self.cost = cost
        self.depth = depth
        self.heuristic = heuristic

    def __str__(self):
        return "node(" + str(self.coords) + "," + str(self.parent) + "," + str(self.cost) + str(self.heuristic) + ")"

    def __repr__(self):
        return str(self)

class SearchTree:
    def __init__(self, problem, body, strategy = 'breadth'):
        self.problem = problem
        self.head = body[0]
        root = SearchNode(self.head, None, 0, 0, 0)
        self.open_nodes = [root]
        self.strategy = strategy
        self.body = body
        self.limit = 100 # limite de expansão dos nós

    def get_path(self, node):
        if node.parent == None:
            return []
        path = self.get_path(node.parent)
        return path + [node.coords]

    # a funcao search recebe o tunel em que um dos agentes está(quando nao ha nenhum agente dentro de um tunel entao
    # este parâmetro de entrada é um array vazio) ,
    # de modo a que o outro agente nao calcule um caminho que passe por este tunel.
    def search(self,tunnel): 
        visited_nodes = []
        while self.open_nodes:
            node = self.open_nodes[0]
            if(self.problem.goal_test(node.coords)):
                return self.get_path(node)

            self.open_nodes[0:1] = []

            lnewnodes = []
            path = self.get_path(node)
            extra_coords = [t.coords for t in self.open_nodes]
            actions = self.problem.domain.actions(node.coords,tunnel)
            for action in actions:
                newpos = self.problem.domain.result(node.coords, action) # se a newpos já tiver sido visitada entao não é expandida
                if(newpos not in visited_nodes and newpos not in extra_coords and newpos not in self.body and node.depth < self.limit):
                    lnewnodes += [SearchNode(newpos, node, self.problem.domain.cost(node.coords, action) + node.cost,
                                             self.problem.domain.heuristic(newpos, self.problem.goal), node.depth+1)]
            visited_nodes.append(node.coords)
            self.add_to_open(lnewnodes)
        return [self.head]

    def add_to_open(self, lnewnodes):
        if self.strategy == 'a*':
            self.open_nodes += lnewnodes
            self.open_nodes.sort(key = lambda node : (node.heuristic + node.cost))
        elif(self.strategy == 'greedy'):
            self.open_nodes += lnewnodes
            self.open_nodes.sort(key = lambda node : (node.heuristic))

class Paths(SearchDomain):
    def __init__(self, world):
        self.world = world

    def actions(self, point,tunnel):
        validact = []
        for act in ACTIONS[1:]:
            newpos = self.world.translate(point, act)
            if newpos not in self.world.walls and newpos not in tunnel: # só as posicoes que nao estao nas paredes nem no tunel 
                                                                        # em que o outro agente está é que sao válidas
                validact.append(act)
        return validact

    def result(self, point, action):
        return self.world.translate(point, action)

    def cost(self, coords, action):
        return 1

    def heuristic(self, coords, goal_coords):
        return self.world.dist(coords, goal_coords)
