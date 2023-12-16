import copy
import csv
import math

import os, platform, random
from kivy.logger import Logger
from kivy.config import Config

from pysimbotlib.core import Simbot, PySimbotApp, Robot

if platform.system() == "Linux" or platform.system() == "Darwin":
    os.environ["KIVY_VIDEO"] = "ffpyplayer"

next_gen_robots = []
fitness = []


class Util:
    @staticmethod
    def distance(p1, p2):
        return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    @staticmethod
    def crossover(gene1, gene2, crossover_probability=0.5):
        if random.random() < crossover_probability:
            crossover_point = random.randrange(len(gene1))
            for i in range(crossover_point):
                gene1[i], gene2[i] = gene2[i], gene1[i]

        return gene1, gene2

    @staticmethod
    def mutation(gene, flip_probability=0.001):
        for i in range(len(gene)):
            for j in range(len(gene[i])):
                if random.random() < flip_probability:
                    gene[i][j] = 255 - gene[i][j]

        return gene


def before_simulation(simbot: Simbot):
    for robot in simbot.robots:
        if simbot.simulation_count == 0:
            Logger.info("GA: initial population")
            for i, RULE in enumerate(robot.RULES):
                for k in range(len(RULE)):
                    robot.RULES[i][k] = random.randrange(256)
        else:
            Logger.info("GA: copy the rules from previous generation")
            for simbot_robot, robot_from_last_gen in zip(
                simbot.robots, next_gen_robots
            ):
                simbot_robot.RULES = robot_from_last_gen.RULES


def after_simulation(simbot: Simbot):
    Logger.info("GA: Start GA Process ...")
    eaten = 0

    # Evaluation – compute fitness values here
    for robot in simbot.robots:
        food_pos = simbot.objectives[0].pos
        robot_pos = robot.pos
        distance = Util.distance(food_pos, robot_pos)
        robot.fitness = 1000 - int(distance)
        robot.fitness -= robot.collision_count
        if robot.eat_count <= 0:
            robot.fitness -= 100
            robot.fitness -= simbot.iteration
        else:
            eaten += 1
            robot.fitness += 500
            robot.fitness += simbot.iteration

    # Descending sort and rank: the best 10 will be on the list at index 0 to 9
    simbot.robots.sort(key=lambda robot: robot.fitness, reverse=True)

    print("Best fitness: {}".format(simbot.robots[0].fitness))

    fitness.append(simbot.robots[0].fitness)

    # Empty the list
    next_gen_robots.clear()

    for i in range(10):
        next_gen_robots.append(simbot.robots[i])

    num_robots = len(simbot.robots)

    def select():
        if eaten == 0:
            index = random.randrange(num_robots) % 20
        else:
            index = random.randrange(num_robots) % 5
        return simbot.robots[index]

    for _ in range(num_robots - 10):
        select1 = select()
        select2 = select()

        while select1 == select2:
            select2 = select()

        child = StupidRobot()

        if eaten < 10:
            cutPointRule = random.randint(0, 15)
            cutPointRuleLen = random.randint(0, 10)

            child.RULES[:cutPointRule] = copy.deepcopy(select1.RULES[:cutPointRule])
            child.RULES[cutPointRule + 1 :] = copy.deepcopy(
                select2.RULES[cutPointRule + 1 :]
            )
            child.RULES[cutPointRule][:cutPointRuleLen] = copy.deepcopy(
                select1.RULES[cutPointRule][:cutPointRuleLen]
            )
            child.RULES[cutPointRule][cutPointRuleLen + 1 :] = copy.deepcopy(
                select2.RULES[cutPointRule][cutPointRuleLen + 1 :]
            )

            child.RULES = Util.mutation(child.RULES, 0.02)
            next_gen_robots.append(child)
        else:
            child.RULES = copy.deepcopy(select1.RULES)
            child.RULES = Util.mutation(child.RULES, 0.01)
            next_gen_robots.append(child)

    # Write the best rule to file
    write_rule(simbot.robots[0], "best_gen_0.csv".format(simbot.simulation_count))


class StupidRobot(Robot):
    RULE_LENGTH = 11
    NUM_RULES = 16

    def __init__(self, **kwarg):
        super(StupidRobot, self).__init__(**kwarg)
        self.RULES = [[0] * self.RULE_LENGTH for _ in range(self.NUM_RULES)]
        self.pos = (20, 560)

        self.rules = [0.0] * self.NUM_RULES
        self.turns = [0.0] * self.NUM_RULES
        self.moves = [0.0] * self.NUM_RULES

        self.fitness = 0

    def update(self):
        """Update method which will be called each frame"""
        self.ir_values = self.distance()
        (
            self.S0,
            self.S1,
            self.S2,
            self.S3,
            self.S4,
            self.S5,
            self.S6,
            self.S7,
        ) = self.ir_values
        self.target = self.smell()

        for i, RULE in enumerate(self.RULES):
            self.rules[i] = 1.0
            for k, RULE_VALUE in enumerate(RULE):
                if k < 8:
                    if RULE_VALUE % 3 == 1:
                        if k == 0:
                            self.rules[i] *= self.S0_near()
                        elif k == 1:
                            self.rules[i] *= self.S1_near()
                        elif k == 2:
                            self.rules[i] *= self.S2_near()
                        elif k == 3:
                            self.rules[i] *= self.S3_near()
                        elif k == 4:
                            self.rules[i] *= self.S4_near()
                        elif k == 5:
                            self.rules[i] *= self.S5_near()
                        elif k == 6:
                            self.rules[i] *= self.S6_near()
                        elif k == 7:
                            self.rules[i] *= self.S7_near()
                    elif RULE_VALUE % 3 == 2:
                        if k == 0:
                            self.rules[i] *= self.S0_far()
                        elif k == 1:
                            self.rules[i] *= self.S1_far()
                        elif k == 2:
                            self.rules[i] *= self.S2_far()
                        elif k == 3:
                            self.rules[i] *= self.S3_far()
                        elif k == 4:
                            self.rules[i] *= self.S4_far()
                        elif k == 5:
                            self.rules[i] *= self.S5_far()
                        elif k == 6:
                            self.rules[i] *= self.S6_far()
                        elif k == 7:
                            self.rules[i] *= self.S7_far()
                elif k == 8:
                    temp_val = RULE_VALUE % 4
                    if temp_val == 1:
                        self.rules[i] *= self.smell_left()
                    elif temp_val == 2:
                        self.rules[i] *= self.smell_center()
                    elif temp_val == 3:
                        self.rules[i] *= self.smell_right()
                elif k == 9:
                    self.turns[i] = (RULE_VALUE % 91) - 45
                elif k == 10:
                    self.moves[i] = RULE_VALUE % 11

        answerTurn = 0.0
        answerMove = 0.0
        for turn, move, rule in zip(self.turns, self.moves, self.rules):
            answerTurn += turn * rule
            answerMove += move * rule

        if self.just_eat is False:
            self.turn(answerTurn)
            self.move(answerMove)

    def S0_near(self):
        if self.S0 <= 0:
            return 1.0
        elif self.S0 >= 100:
            return 0.0
        else:
            return 1 - (self.S0 / 100)

    def S0_far(self):
        if self.S0 <= 0:
            return 0.0
        elif self.S0 >= 100:
            return 1.0
        else:
            return self.S0 / 100

    def S1_near(self):
        if self.S1 <= 0:
            return 1.0
        elif self.S1 >= 100:
            return 0.0
        else:
            return 1 - (self.S1 / 100)

    def S1_far(self):
        if self.S1 <= 0:
            return 0.0
        elif self.S1 >= 100:
            return 1.0
        else:
            return self.S1 / 100

    def S2_near(self):
        if self.S2 <= 0:
            return 1.0
        elif self.S2 >= 100:
            return 0.0
        else:
            return 1 - (self.S2 / 100)

    def S2_far(self):
        if self.S2 <= 0:
            return 0.0
        elif self.S2 >= 100:
            return 1.0
        else:
            return self.S2 / 100

    def S3_near(self):
        if self.S3 <= 0:
            return 1.0
        elif self.S3 >= 100:
            return 0.0
        else:
            return 1 - (self.S3 / 100)

    def S3_far(self):
        if self.S3 <= 0:
            return 0.0
        elif self.S3 >= 100:
            return 1.0
        else:
            return self.S3 / 100

    def S4_near(self):
        if self.S4 <= 0:
            return 1.0
        elif self.S4 >= 100:
            return 0.0
        else:
            return 1 - (self.S4 / 100)

    def S4_far(self):
        if self.S4 <= 0:
            return 0.0
        elif self.S4 >= 100:
            return 1.0
        else:
            return self.S4 / 100

    def S5_near(self):
        if self.S5 <= 0:
            return 1.0
        elif self.S5 >= 100:
            return 0.0
        else:
            return 1 - (self.S5 / 100)

    def S5_far(self):
        if self.S5 <= 0:
            return 0.0
        elif self.S5 >= 100:
            return 1.0
        else:
            return self.S5 / 100

    def S6_near(self):
        if self.S6 <= 0:
            return 1.0
        elif self.S6 >= 100:
            return 0.0
        else:
            return 1 - (self.S6 / 100)

    def S6_far(self):
        if self.S6 <= 0:
            return 0.0
        elif self.S6 >= 100:
            return 1.0
        else:
            return self.S6 / 100

    def S7_near(self):
        if self.S7 <= 0:
            return 1.0
        elif self.S7 >= 100:
            return 0.0
        else:
            return 1 - (self.S7 / 100)

    def S7_far(self):
        if self.S7 <= 0:
            return 0.0
        elif self.S7 >= 100:
            return 1.0
        else:
            return self.S7 / 100

    def smell_right(self):
        if self.target >= 180:
            return 1.0
        elif self.target <= 0:
            return 0.0
        else:
            return self.target / 180.0

    def smell_center(self):
        absTarget = abs(self.target)
        if absTarget > 45:
            return 0.0
        else:
            return 1 - (self.target / 45.0)

    def smell_left(self):
        if self.target <= -180:
            return 1.0
        elif self.target >= -0:
            return 0.0
        else:
            return -self.target / 180.0


def write_rule(robot, filename):
    with open(filename, "w") as f:
        writer = csv.writer(f, lineterminator="\n")
        writer.writerows(robot.RULES)


def read_rule(robot, filename):
    with open(filename, "r") as f:
        reader = csv.reader(f)
        robot.RULES = list(reader)


if __name__ == "__main__":
    Config.set("graphics", "width", "800")
    Config.set("graphics", "height", "800")
    app = PySimbotApp(
        num_robots=100,
        robot_cls=StupidRobot,
        customfn_before_simulation=before_simulation,
        customfn_after_simulation=after_simulation,
        interval=1 / 60,
        max_tick=200,
        simulation_forever=True,
        food_move_after_eat=False,
    )
    app.run()
