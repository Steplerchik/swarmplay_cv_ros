#!/usr/bin/env python3
# license removed for brevity
import rospy

from std_msgs.msg import String

import random
import numpy as np
#from math import inf as infinity
from random import choice

#subscriber_name = '/human_turns_sim'
subscriber_name = '/human_turns'


first_turn = 0 #Who first: 0-drones, 1-player
mode = 'mid'
#mode = 'easy'
#mode = 'minmax'

HUMAN = -1
COMP = +1

INTEGRATE_RATE = 1

class DecidionMakingNode(object):
    def __init__(self):
        rospy.init_node('decidion_making', anonymous=True)
        self.publisher_drone = rospy.Publisher('/move_drone', String, queue_size=10)
        self.rate = rospy.Rate(INTEGRATE_RATE)
        self.board = list(range(1,10))
        self.board_state = np.zeros((3,3))
        self.cells = []
        self.human_turn = ''
        self.win = False
        self.game_mode = {
            "first_turn": first_turn,
            "mode": mode
        }
        self.output = ''

    def empty_cells(self):
        self.cells = []
        for x, row in enumerate(self.board_state):
            for y, cell in enumerate(row):
                if cell == 0:
                    self.cells.append([x, y])
        return

    def evaluate(self):
        """
        Function to heuristic evaluation of state.
        :param state: the state of the current board
        :return: +1 if the computer wins; -1 if the human wins; 0 draw
        """
        if self.wins(COMP):
            score = 1
        elif self.wins(HUMAN):
            score = -1
        else:
            score = 0
        return score

    def wins(self, player):
        win_state = [
            [self.board_state[0][0], self.board_state[0][1], self.board_state[0][2]],
            [self.board_state[1][0], self.board_state[1][1], self.board_state[1][2]],
            [self.board_state[2][0], self.board_state[2][1], self.board_state[2][2]],
            [self.board_state[0][0], self.board_state[1][0], self.board_state[2][0]],
            [self.board_state[0][1], self.board_state[1][1], self.board_state[2][1]],
            [self.board_state[0][2], self.board_state[1][2], self.board_state[2][2]],
            [self.board_state[0][0], self.board_state[1][1], self.board_state[2][2]],
            [self.board_state[2][0], self.board_state[1][1], self.board_state[0][2]],
        ]
        if [player, player, player] in win_state:
            return True
        else:
            return False

    def game_over(self):
        """
        This function test if the human or computer wins
        :param state: the state of the current board
        :return: True if the human or computer wins
        """
        return self.wins(HUMAN) or self.wins(COMP)

    def minimax(self, depth, player):
        """
        AI function that choice the best move
        :param state: current state of the board
        :param depth: node index in the tree (0 <= depth <= 9),
        but never nine in this case (see iaturn() function)
        :param player: an human or a computer
        :return: a list with [the best row, best col, best score]
        """
        if player == COMP:
            best = [-1, -1, -10000]
        else:
            best = [-1, -1, 10000]

        if depth == 0 or self.win:
            score = self.evaluate()
            return [-1, -1, score]

        self.empty_cells()
        for cell in self.cells:
            x, y = cell[0], cell[1]
            self.board_state[x][y] = player
            score = self.minimax(depth - 1, -player)
            self.board_state[x][y] = 0
            score[0], score[1] = x, y

            if player == COMP:
                if score[2] > best[2]:
                    best = score  # max value
            else:
                if score[2] < best[2]:
                    best = score  # min value

        return best

    def ai_turn(self, symbol):
        """
        It calls the minimax function if the depth < 9,
        else it choices a random coordinate.
        :param c_choice: computer's choice X or O
        :param h_choice: human's choice X or O
        :return:
        """
        self.empty_cells()
        depth = len(self.cells)

        if depth == 0 or self.win:
            return

        print('Computer turn')
        #render(board, c_choice, h_choice)

        if depth == 9:
            print('depth == 9')
            x = choice([0, 1, 2])
            y = choice([0, 1, 2])
            print(x, y)
        else:
            move = self.minimax(depth, COMP)
            x, y = move[0], move[1]

        if self.board_state[x, y] == 0:
            self.board[3*x + y] = symbol
            self.board_state[x, y] = COMP
            self.output = str((3*x + y) + 1)
            print('akynamatata')


        #set_move(cell, COMP)
        #time.sleep(1)





    def decidion_make_drone(self, symbol):
        print('--decidion_make_drone----')

        if self.game_mode["mode"] == 'minmax':
            print('-----minmax mode-----')
            self.empty_cells()
            print(len(self.cells), self.win)
            if len(self.cells) > 0 and not self.win:
                print('----we are here----')
                self.ai_turn(symbol)
                print(self.board_state)
            return

        if self.game_mode["mode"] == 'easy':
            while not self.win:
                rand_board = random.randint(0, 8)
                if self.board[rand_board] != 'O' and self.board[rand_board] != 'X':
                    self.board[rand_board] = symbol
                    self.output = str(rand_board + 1)
                    break
            return

        if self.game_mode["mode"] == 'mid':
            for n in range(3):
                res, index = self.can_win(self.board[3 * n], self.board[3 * n + 1], self.board[3 * n + 2], 'X')
                if res:
                    self.board[3 * n + (index - 1)] = symbol
                    self.output = str((3 * n + (index - 1))+1)
                    return
                res, index = self.can_win(self.board[n], self.board[n + 3], self.board[n + 6], 'X')
                if res:
                    self.board[n + (index - 1) * 3] = symbol
                    self.output = str((n + (index - 1) * 3)+1)
                    return
            res, index = self.can_win(self.board[0], self.board[4], self.board[8], 'X')
            if res:
                self.board[(index - 1) * 4] = symbol
                self.output = str(((index - 1) * 4)+1)
                return
            res, index = self.can_win(self.board[2], self.board[4], self.board[6], 'X')
            if res:
                self.board[index * 2] = symbol
                self.output = str(index * 2 + 1)
                return
            for n in range(3):
                res, index = self.can_win(self.board[3 * n], self.board[3 * n + 1], self.board[3 * n + 2], 'O')
                if res:
                    self.board[3 * n + (index - 1)] = symbol
                    self.output = str((3 * n + (index - 1))+1)
                    return
                res, index = self.can_win(self.board[n], self.board[n + 3], self.board[n + 6], 'O')
                if res:
                    self.board[n + (index - 1) * 3] = symbol
                    self.output = str((n + (index - 1) * 3)+1)
                    return
            res, index = self.can_win(self.board[0], self.board[4], self.board[8], 'O')
            if res:
                self.board[(index - 1) * 4] = symbol
                self.output = str(((index - 1) * 4)+1)
                return
            res, index = self.can_win(self.board[2], self.board[4], self.board[6], 'O')
            if res:
                self.board[index * 2] = symbol
                self.output = str(index * 2 + 1)
                return
            while True:
                rand_board = random.randint(1, 8)
                if self.board[rand_board] != 'O' and self.board[rand_board] != 'X':
                    self.board[rand_board] = symbol
                    self.output = str(rand_board+1)
                    break
            return

    def can_win(self, a1, a2, a3, smb):
        res = False
        index = 0
        if a1 == smb and a2 == smb and a3 != 'O' and a3 != 'X':
            index = 3
            res = True
        if a1 == smb and a2 != 'O' and a2 != 'X' and a3 == smb:
            index = 2
            res = True
        if a1 != 'X' and a1 != 'O' and a2 == smb and a3 == smb:
            index = 1
            res = True
        return res, index

    def check_win(self):
        win_coord = ((0, 1, 2), (3, 4, 5), (6, 7, 8), (0, 3, 6), (1, 4, 7), (2, 5, 8), (0, 4, 8), (2, 4, 6))
        for each in win_coord:
            if self.board[each[0]] == self.board[each[1]] == self.board[each[2]]:
                if self.board[each[0]] == 'O':
                    print(self.board[each[0]], "Player Wins!")
                if self.board[each[0]] == 'X':
                    print(self.board[each[0]], "Drones Wins!")
                self.win = True
                self.draw_board()
                return
        count_sells = 0
        for i in range(9):
            if self.board[i] == 'O' and self.board[i] == 'X':
                count_sells += 1
            if count_sells == 9:
                print('No winner')
                self.draw_board()
                self.win = True
        return

    def draw_board(self):
        print("-" * 13)
        for i in range(3):
            print(self.board[0 + i * 3], self.board[1 + i * 3], self.board[2 + i * 3])
            print("-" * 13)

    def init_game(self):
        self.rate.sleep()
        while not rospy.is_shutdown():
            #rospy.sleep(1)
            rospy.loginfo('New_game')
            #rospy.sleep(1)
            self.publisher_drone.publish('New_game')
            rospy.sleep(1)
            self.win = False
            #---fisrt turns---
            if self.game_mode["first_turn"] == 0:
                print("---Drone's move---")
                self.decidion_make_drone('X')
                self.draw_board()
                self.publish_drone()
                self.subscriber_human = rospy.Subscriber(subscriber_name, String, self.callback)
                rospy.spin()
                #self.rate.sleep()

            if self.game_mode["first_turn"] == 1:
                print("---Player's move---")
                self.draw_board()
                self.subscriber_human = rospy.Subscriber(subscriber_name, String, self.callback)
                rospy.spin()
                #self.rate.sleep()


    def callback(self, data):
        if not self.win:
            self.draw_board()
            print("---Player's move---")
            rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
            self.human_turn = data.data
            self.board[int(self.human_turn) - 1] = 'O'
            self.board_state[(int(self.human_turn)-1)//3,(int(self.human_turn)-1)%3] = HUMAN
            self.check_win()

            if not self.win:
                print("---Drone's move---")
                self.decidion_make_drone('X')
                self.publish_drone()
                self.check_win()
            if not self.win:
                print("---Player's move---")
                self.draw_board()
            if self.win:
                print('---end game---')
                self.publisher_drone.publish('End_game')
                self.board = list(range(1, 10))
                self.win = False
                rospy.sleep(1)
                self.publisher_drone.publish('Start_game')
                print("---New Game---")


    def publish_drone(self):
        rospy.loginfo(self.output)
        self.publisher_drone.publish(self.output)


if __name__ == '__main__':
    try:
        node = DecidionMakingNode()
        node.init_game()
    except rospy.ROSInterruptException:
        pass