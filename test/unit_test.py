'''
Check variable values.
Check selected goals. Are they in the correct area ?
What if drone gets stuck ?
'''
import os
import pytest
import random
from src.GoToGoal_PID_DD import main

class TestCase:

    def relocatePath(self):
        os.chdir("..")
        new_path = os.getcwd() + "\src"
        os.chdir(new_path)

    def test_one_goal(self):
        self.relocatePath()
        main([(4,5)])
        #self.assertEqual(True, True)  # add assertion here

    def test_one_goal_random(self):
        self.relocatePath()
        goal_x = random.uniform(0.0, 6.0)
        goal_y = random.uniform(0.0, 6.0)
        main([(goal_x, goal_y)])

    def test_two_goals(self):
        self.relocatePath()
        main([(2, 4), (4.3, 5.1)])

    def test_two_goals_random(self):
        self.relocatePath()
        goal_x_1 = random.uniform(0.0, 6.0)
        goal_y_1 = random.uniform(0.0, 6.0)

        goal_x_2 = random.uniform(0.0, 6.0)
        goal_y_2 = random.uniform(0.0, 6.0)

        main([(goal_x_1, goal_y_1), (goal_x_2, goal_y_2)])

    def test_three_goals(self): # (1, 5.5), (2.5, 1)
        self.relocatePath()
        main([(1.3, 3.5), (2.1, 0.7), (4.5, 3.2)])

    def test_three_goals_random(self):
        self.relocatePath()
        goal_x_1 = random.uniform(0.0, 6.0)
        goal_y_1 = random.uniform(0.0, 6.0)

        goal_x_2 = random.uniform(0.0, 6.0)
        goal_y_2 = random.uniform(0.0, 6.0)

        goal_x_3 = random.uniform(0.0, 6.0)
        goal_y_3 = random.uniform(0.0, 6.0)

        main([(goal_x_1, goal_y_1), (goal_x_2, goal_y_2), (goal_x_3, goal_y_3)])

    def test_four_goals(self):
        self.relocatePath()
        main([(1.5, 3.2), (2.7, 4), (4, 3.9), (5.1, 3.2)])
        assert True == True

    def test_four_goals_random(self):
        assert True == True

    def test_ten_goals(self):
        assert True == True

    def test_ten_goals_random(self):
        assert True == True

    def test_goal_is_out_of_bounds(self):
        self.relocatePath()
        main([(10, 10)])
        assert True == True

    def test_no_goals(self):
        assert True == True