import rclpy
from path_optimization.turtle_simulation.turtle_sim import TurtleSimManager
from path_optimization.screen_manager.screen_manager import ScreenManager
import random
from deap import base, creator, tools, algorithms
import string
import time
import numpy as np

# Define the limits for the parameters
param_limits = [(0.1, 2.0), (0.1, 1.0), (0.1, 1.0)]

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
    k, k_beta, k_alpha = params
    # Create a robot with the parameters
    name = generate_unique_random_name(8)
    robot = TurtleSimManager(f"robot_{name.lower()}", k, k_alpha, k_beta)

    robot.spawn(1.0, 5.0, 0.0)
    robot.add_subscription()
    robot.pen_request()

    distance_computed = False
    timeout_seconds = 10  # sec
    start_time = time.time() 
    is_timeout = False

    while rclpy.ok():

        distance_computed = robot.compute_dist

        rclpy.spin_once(robot)

        if robot.arrived and distance_computed:
            break

        current_time = time.time()
        elapsed_time = current_time - start_time
        
        if elapsed_time >= timeout_seconds:
            print(f"Timeout robot_{name.lower()}: Conditions not met within the time limit.")
            is_timeout = True
            break

    # Execute the mission and get the distance traveled
    if not is_timeout:
        distance_traveled = robot.distance
    else:
        distance_traveled = 1.0e1000

    robot.destroy_node()
    # Minimize the distance traveled
    return distance_traveled,

def create_individual():
    return creator.Individual([random.uniform(lim[0], lim[1]) for lim in param_limits])

def custom_mutation(individual):
    for i in range(len(individual)):
        if random.random() < 0.2:  # Probabilidade de mutação
            individual[i] += random.gauss(0, 0.2)
            individual[i] = max(min(individual[i], param_limits[i][1]), param_limits[i][0])
    return individual

def custom_crossover(parent1, parent2):
    child = tools.cxBlend(parent1, parent2, alpha=0.5)
    for i in range(len(child)):
        child[i] = max(min(child[i], param_limits[i][1]), param_limits[i][0])
    return creator.Individual(child)

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
    toolbox.register("mate", custom_crossover)
    toolbox.register("mutate", custom_mutation)
    toolbox.register("select", tools.selTournament, tournsize=3)

    # Create an initial population
    population = toolbox.population(n=20)

    # Crie um objeto Statistics
    stats = tools.Statistics(key=lambda ind: ind.fitness.values)

    # Registre as estatísticas que você deseja coletar
    stats.register("avg", np.mean)  # Média da aptidão
    stats.register("min", np.min)    # Menor aptidão
    stats.register("max", np.max)    # Maior aptidão

    # Run the genetic algorithm (NSGA-II in this example)
    algorithms.eaMuPlusLambda(population, toolbox, mu=5, lambda_=20, cxpb=0.7, mutpb=0.3, ngen=10, stats=None, halloffame=None)

    gen = range(0, len(stats.get("avg")))
    avg = stats.get("avg")
    min = stats.get("min")
    max = stats.get("max")

    # Get the best individual after optimization
    best_individual = tools.selBest(population, 1)[0]
    best_parameters = best_individual

    print("Best parameters found:", best_parameters)
    print("average:", avg)
    print("min:", min)
    print("max:", max)
    print("gen average: ", gen)


    rclpy.shutdown()

if __name__ == '__main__':
    main()
