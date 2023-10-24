import rclpy
from path_optimization.turtle_simulation.turtle_sim import TurtleSimManager
from path_optimization.screen_manager.screen_manager import ScreenManager
import random
from deap import base, creator, tools, algorithms
import string
import time
import numpy as np

# Define the limits for the parameters
param_limits = [(0.1, 2.0), (0.1, 2.0), (0.1, 2.0)]

# Crie um conjunto para armazenar nomes gerados anteriormente
generated_names = set()

def generate_unique_random_name(name_length=8):
    characters = string.ascii_letters
    while True:
        random_name = ''.join(random.choice(characters) for _ in range(name_length))
        if random_name not in generated_names:
            generated_names.add(random_name)
            return random_name

# Define the fitness function
def fitness(params):
    k, k_alpha, k_beta = params

    print(f"k: {k}, k_alpha: {k_alpha}, k_beta: {k_beta}")
    # Create a robot with the parameters
    name = generate_unique_random_name(8)
    robot = TurtleSimManager(f"robot_{name.lower()}", k, k_alpha, k_beta)

    robot.spawn(1.0, 1.0, 0.0)
    robot.add_subscription()
    robot.pen_request()

    start_time = time.time() 
    stop_early = False
    elapsed_time = None

    while rclpy.ok():

        rclpy.spin_once(robot)

        if robot.collided:
            print(f"robot_{name.lower()} :This individual collided")
            stop_early = True
            break

        if robot.arrived:
            break

    current_time = time.time()
    elapsed_time = current_time - start_time

    # Execute the mission and get the distance traveled
    if not stop_early:
        # distance_traveled = robot.distance / elapsed_time
        distance_traveled = elapsed_time
        print(f"robot_{name.lower()} : {distance_traveled} s")
    else:
        distance_traveled = 1.0e10000

    robot.destroy_node()
    # Minimize the distance traveled
    return distance_traveled,

def create_individual():
    return creator.Individual([random.uniform(lim[0], lim[1]) for lim in param_limits])

def main(args=None):

    rclpy.init(args=args)

    screen = ScreenManager()
    screen.kill()
    screen.reset()

    # Define the problem as a minimization problem
    creator.create("FitnessMin", base.Fitness, weights=(-1.0,))

    # Define the structure of the individual
    creator.create("Individual", list, fitness=creator.FitnessMin)

    # Create the toolbox for the genetic algorithm
    toolbox = base.Toolbox()
    toolbox.register("attr_float", random.uniform, 0.1, 2.0)
    toolbox.register("individual", create_individual)
    toolbox.register("population", tools.initRepeat, list, toolbox.individual)
    toolbox.register("evaluate", fitness)
    toolbox.register("mate", tools.cxTwoPoint)
    toolbox.register("mutate", tools.mutGaussian, mu=0.0, sigma=0.2, indpb=0.2)
    toolbox.register("select", tools.selTournament, tournsize=3)

    # Create an initial population
    population = toolbox.population(n=30)

    # Crie um objeto Statistics
    stats = tools.Statistics(key=lambda ind: ind.fitness.values)

    # Registre as estatísticas que você deseja coletar
    stats.register("avg", np.mean)  # Média da aptidão
    stats.register("min", np.min)    # Menor aptidão
    stats.register("max", np.max)    # Maior aptidão

    # Defina ngen, a quantidade de gerações que você deseja executar
    ngen = 10

    # Executar o algoritmo genético em um loop para cada geração
    for gen in range(1, ngen + 1):
        # Aqui, você pode imprimir ou fazer qualquer coisa que desejar na geração atual
        print(f"Evaluating the generation {gen}/{ngen}")
        
        # Run the genetic algorithm (NSGA-II in this example)
        algorithms.eaMuPlusLambda(population, toolbox, mu=10, lambda_=15, cxpb=0.7, mutpb=0.2, ngen=1, stats=stats, halloffame=None, verbose=True)

        screen.reset()
        screen.kill()


    best_individuals = tools.selBest(population, k=5)

    print("5 Best individuals parameters found:", best_individuals)

    rclpy.shutdown()

if __name__ == '__main__':
    main()